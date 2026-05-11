#include "simplebus.h"
#include "utils/list.h"

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define LOG_TAG "simplebus"
#undef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#include "log.h"

typedef struct
{
    list_head_t node;
    void *ctx;
    simple_bus_subscribe_callback_t cb;
} subscribe_callback_t;

typedef struct
{
    list_head_t node;
    char name[256];
    list_head_t subscribe_callback_list;
} subscribe_name_t;

typedef struct
{
    list_head_t node;
    char name[256];
    int msg_id;
    uint8_t *msg;
    uint32_t msg_len;
} async_msg_t;

typedef struct
{
    list_head_t node;
    char name[256];
    void *ctx;
    simple_bus_service_callback_t cb;
} service_t;

typedef struct simple_bus
{
    list_head_t subscribe_name_list;

    list_head_t async_msg_list;
    pthread_mutex_t async_msg_list_lock;
    pthread_cond_t async_msg_list_cond;

    list_head_t service_list;
    pthread_mutex_t service_list_lock;

    pthread_mutex_t lock;
    pthread_t pid;
} simple_bus_t;

static simple_bus_t bus;

static subscribe_name_t *simple_bus_get_name(const char *name);
static service_t *simple_bus_get_service(const char *name);

static void *simple_bus_thread(void *args);

void simple_bus_init(void)
{
    pthread_mutex_init(&bus.lock, NULL);
    pthread_mutex_init(&bus.service_list_lock, NULL);
    pthread_mutex_init(&bus.async_msg_list_lock, NULL);
    pthread_cond_init(&bus.async_msg_list_cond, NULL);

    list_head_init(&bus.subscribe_name_list);
    list_head_init(&bus.async_msg_list);
    list_head_init(&bus.service_list);

    pthread_create(&bus.pid, NULL, simple_bus_thread, NULL);
}

void simple_bus_deinit(void)
{
}

static void *simple_bus_thread(void *args)
{
    async_msg_t *msg;
    async_msg_t *msg_n;

    subscribe_name_t *n = NULL;
    subscribe_callback_t *pos;

    while (1)
    {
        pthread_mutex_lock(&bus.async_msg_list_lock);
        pthread_cond_wait(&bus.async_msg_list_cond, &bus.async_msg_list_lock);
        do
        {
            list_for_each_entry_safe(msg, msg_n, &bus.async_msg_list, async_msg_t, node)
            {
                pthread_mutex_unlock(&bus.async_msg_list_lock);
                pthread_mutex_lock(&bus.lock);
                n = simple_bus_get_name(msg->name);
                if (n)
                {
                    list_for_each_entry(pos, &n->subscribe_callback_list, subscribe_callback_t, node)
                    {
                        if (pos->cb)
                            pos->cb(msg->name, pos->ctx, msg->msg_id, msg->msg, msg->msg_len);
                    }
                }
                pthread_mutex_unlock(&bus.lock);
                pthread_mutex_lock(&bus.async_msg_list_lock);

                list_remove(&msg->node);
                if (msg->msg_len > 0 && msg->msg != NULL)
                {
                    free(msg->msg);
                }
                free(msg);
            }
            usleep(1000);
        } while (list_is_empty(&bus.async_msg_list) == 0);
        pthread_mutex_unlock(&bus.async_msg_list_lock);
        usleep(1000);
    }
}

int simple_bus_subscribe(const char *name, void *context, simple_bus_subscribe_callback_t callback)
{
    subscribe_name_t *n = NULL;
    subscribe_callback_t *sc = (subscribe_callback_t *)malloc(sizeof(subscribe_callback_t));

    if (sc == NULL)
        return -1;

    sc->cb = callback;
    sc->ctx = context;

    pthread_mutex_lock(&bus.lock);
    if ((n = simple_bus_get_name(name)) == NULL)
    {
        n = (subscribe_name_t *)malloc(sizeof(subscribe_name_t));
        if (n == NULL)
        {
            free(sc);
            pthread_mutex_unlock(&bus.lock);
            return -1;
        }
        strncpy(n->name, name, sizeof(n->name));
        list_head_init(&n->subscribe_callback_list);
        list_insert_tail(&bus.subscribe_name_list, &n->node);
    }
    list_insert_tail(&n->subscribe_callback_list, &sc->node);
    pthread_mutex_unlock(&bus.lock);

    return 0;
}

int simple_bus_unsubscribe(const char *name, void *context, simple_bus_subscribe_callback_t callback)
{
    pthread_mutex_lock(&bus.lock);
    subscribe_name_t *n = NULL;

    if ((n = simple_bus_get_name(name)) != NULL)
    {
        subscribe_callback_t *sc_pos;
        subscribe_callback_t *sc_n;
        list_for_each_entry_safe(sc_pos, sc_n, &n->subscribe_callback_list, subscribe_callback_t, node)
        {
            if (sc_pos->ctx == context)
            {
                list_remove(&sc_pos->node);
                free(sc_pos);
                break;
            }
        }

        if (list_length(&n->subscribe_callback_list) == 0)
        {
            list_remove(&n->node);
            free(n);
        }
    }
    pthread_mutex_unlock(&bus.lock);

    return 0;
}

int simple_bus_publish_sync(const char *name, int msg_id, void *msg, uint32_t msg_len)
{
    pthread_mutex_lock(&bus.lock);
    subscribe_name_t *n = NULL;
    if ((n = simple_bus_get_name(name)) != NULL)
    {
        subscribe_callback_t *pos;
        list_for_each_entry(pos, &n->subscribe_callback_list, subscribe_callback_t, node)
        {
            pthread_mutex_unlock(&bus.lock);
            if (pos->cb)
                pos->cb(name, pos->ctx, msg_id, msg, msg_len);
            pthread_mutex_lock(&bus.lock);
        }
    }
    pthread_mutex_unlock(&bus.lock);
    return 0;
}

int simple_bus_publish_async(const char *name, int msg_id, void *msg, uint32_t msg_len)
{
    async_msg_t *async_msg = (async_msg_t *)malloc(sizeof(async_msg_t));
    if (async_msg == NULL)
        return -1;

    memset(async_msg, 0, sizeof(*async_msg));

    if (msg_len && msg)
    {
        async_msg->msg = malloc(msg_len);
        if (async_msg->msg == NULL)
        {
            free(async_msg);
            return -1;
        }
        memcpy(async_msg->msg, msg, msg_len);
    }

    async_msg->msg_id = msg_id;
    async_msg->msg_len = msg_len;
    strncpy(async_msg->name, name, sizeof(async_msg->name));

    pthread_mutex_lock(&bus.async_msg_list_lock);
    list_insert_tail(&bus.async_msg_list, &async_msg->node);
    pthread_mutex_unlock(&bus.async_msg_list_lock);

    pthread_cond_signal(&bus.async_msg_list_cond);

    return 0;
}

int simple_bus_register_service(const char *name, void *context, simple_bus_service_callback_t callback)
{
    service_t *s;

    pthread_mutex_lock(&bus.service_list_lock);

    if ((s = simple_bus_get_service(name)) != NULL)
    {
        pthread_mutex_unlock(&bus.service_list_lock);
        return -1;
    }

    s = (service_t *)malloc(sizeof(service_t));
    if (s == NULL)
    {
        pthread_mutex_unlock(&bus.service_list_lock);
        return -1;
    }

    strncpy(s->name, name, sizeof(s->name));
    s->cb = callback;
    s->ctx = context;
    list_insert_tail(&bus.service_list, &s->node);

    pthread_mutex_unlock(&bus.service_list_lock);

    return 0;
}

int simple_bus_unregister_service(const char *name, void *context, simple_bus_service_callback_t callback)
{
    service_t *s;
    pthread_mutex_lock(&bus.service_list_lock);
    if ((s = simple_bus_get_service(name)) != NULL)
    {
        list_remove(&s->node);
        free(s);
    }
    pthread_mutex_unlock(&bus.service_list_lock);
    return 0;
}

int simple_bus_request(const char *name, int request_id, void *request, void *response)
{
    pthread_mutex_lock(&bus.service_list_lock);
    service_t *s = NULL;
    if ((s = simple_bus_get_service(name)) != NULL)
    {
        pthread_mutex_unlock(&bus.service_list_lock);
        // 提前解锁避免回调里面递归调用请求导致死锁
        if (s->cb)
            s->cb(name, s->ctx, request_id, request, response);
        return 0;
    }
    pthread_mutex_unlock(&bus.service_list_lock);
    return -1;
}

static subscribe_name_t *simple_bus_get_name(const char *name)
{
    subscribe_name_t *pos = NULL;
    list_for_each_entry(pos, &bus.subscribe_name_list, subscribe_name_t, node)
    {
        if (strncmp(pos->name, name, sizeof(pos->name)) == 0)
            return pos;
    }
    return NULL;
}

static service_t *simple_bus_get_service(const char *name)
{
    service_t *pos = NULL;
    list_for_each_entry(pos, &bus.service_list, service_t, node)
    {
        if (strncmp(pos->name, name, sizeof(pos->name)) == 0)
            return pos;
    }
    return NULL;
}
