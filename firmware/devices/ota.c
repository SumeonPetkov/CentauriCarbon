#include "config.h"
#if CONFIG_SUPPORT_OTA
// brief: ota相关内容

//--------------------------header file-------------------------
#include "ota.h"
#include "utils.h"
#include "cJSON.h"
#include "app.h"
#include "hl_disk.h"
#include "hl_net.h"
#include "params.h"
#include "hl_common.h"
#include "hl_boot.h"
#include "tr.h"
#define LOG_TAG "ota"
#undef LOG_LEVEL
#define LOG_LEVEL LOG_DEBUG
#include "log.h"
#include "jenkins.h"
#include "ai_camera.h"
#include "hl_assert.h"
#include "hl_camera.h"
#include <time.h>
#include <sys/time.h>

#define ENVIRONMENT_DEVELOPMENT 0 // 开发环境
#define ENVIRONMENT_TESTING 1     // 测试环境
#define ENVIRONMENT_FORMAL 2      // 正式环境

#ifndef ENVIRONMENT_CHOOSE
#define ENVIRONMENT_CHOOSE ENVIRONMENT_FORMAL // 0 开发环境 1 测试环境 2 正式环境
#endif

/* < 机型名称, cbd 内部使用 > */
#if CONFIG_PROJECT == PROJECT_ELEGO_3DS006
#define DEV_MARKING "ELEGOO SATURN 4 Ultra"
#elif CONFIG_PROJECT == PROJECT_ELEGO_3DM012
#define DEV_MARKING "ELEGOO MARS 5 Ultra"
#elif CONFIG_PROJECT == PROJECT_ELEGO_E100
#if ENVIRONMENT_CHOOSE == ENVIRONMENT_FORMAL && CONFIG_BOARD_E100 == BOARD_E100
#define DEV_MARKING "ELEGOO Centauri Carbon"
#elif ENVIRONMENT_CHOOSE == ENVIRONMENT_FORMAL && CONFIG_BOARD_E100 == BOARD_E100_LITE
#define DEV_MARKING "ELEGOO Centauri"
#else
#define DEV_MARKING "ELEGOO E100"
#endif
#endif

#define OTA_FILE_NAME "ota_upgrade.zip"
// #define OTA_JUMP_TIME (60 * 30) // s 测试
#define OTA_JUMP_TIME (60 * 60 * 24 * 7) // s

typedef struct ota_info
{
    int update;           // 是否有更新计划
    char version[64];     // 目标版本号
    char packageUrl[256]; // 包地址
    char packageHash[33]; // 包MD5code
    int updateStrategy;   // 更新策略 （1.强制 2.授权）
    char log[512];        // 日志
} ota_info_t;

void *ota_get_info_AIC(void *arg);
void *ota_get_info_SYS(void *arg);
static void ota_get_info_routine(hl_tpool_thread_t thread, void *args);


static ota_info_t ota_info = {0};
static hl_curl_download_ctx_t download_ctx = NULL;
static hl_tpool_thread_t ota_decompress_thread = NULL;
static uint8_t thread_complete_status = 0;
// static int ota_startup = STARTUP_DIDNT_OTA;


// #define OTA_RESULT_BUFF_SIZE 2048

/**
 * *Tips . 基于交付,当前暂仍使用原来框架
 */

// #define OTA_HANDLER_CMD(ch, cmd) ota_handler_##ch_##cmd
// #define OTA_DECLARE_CMD(ch, cmd) extern void *ota_handler_##ch_##cmd(void *arg)
// #define OTA_DEFINE_CMD(ch, cmd) void *ota_handler_##ch_##cmd(void *arg)

// OTA_DECLARE_CMD(OTA_FIREMARE_CH_AIC, OTAG_GET_VERSION);
struct
{
    OTA_API_ST_t ota_api_stat[OTA_FIREMARE_CH_MAX]; /* api 请求状态 */
    // char *ota_result[OTA_FIREMARE_CH_MAX][OTA_RESULT_BUFF_SIZE]; /* 结果存放 */
    void *(*get_info[OTA_FIREMARE_CH_MAX])(void *arg); /* 向云端获取ota信息 处理函数 */
    pthread_t pth[OTA_FIREMARE_CH_MAX][OTAG_MAX];
    pthread_mutex_t pth_lock;
    struct _ota_info info[OTA_FIREMARE_CH_MAX]; /* 信息存放 */
    int fetch_pro[OTA_FIREMARE_CH_MAX];         /* 下载进度 */
    int burn_pro[OTA_FIREMARE_CH_MAX];          /* 烧录进度 */
    /*  */
} ota_ctx = {
    .get_info[OTA_FIREMARE_CH_SYS] = ota_get_info_SYS, /* 获取系统OTA信息 */
    .pth_lock = PTHREAD_MUTEX_INITIALIZER,
#if CONFIG_SUPPORT_AIC
    .get_info[OTA_FIREMARE_CH_AIC] = ota_get_info_AIC, /* 获取AI摄像头OTA信息 */
#endif
};

const char http_address_list_ota[][128] = {
    "http://172.16.1.13:21052/",                         // 开发环境
    "https://cbdss-chitucloud-mms-stg2.chuangbide.com/", // 测试环境
    "https://mms.chituiot.com/",                         // 正式环境
};

const char http_path_list_ota[][128] = {
    "mainboardVersionUpdate/getInfo.do7", // ota
};

const char language_str[][64] = {
    "zh-Hans",  // 简体中文
    "en",       // 英文
    "spanish",  // 西班牙语
    "french",   // 法语
    "italiano", // 意大利语
    "russion",  // 俄语
    "deutsch",  // 德语

    "japanese", // 日语
};

/**
 * 疑似编译优化导致 MD5 二次校验失败
 * 此处功能单独放置一个函数
 */
int otalib_md5sum(char *filename, char out_string[32])
{
    FILE *file = fopen(filename, "rb");

    if (filename == NULL)
    {
        printf("无法打开文件\n");
        return -1;
    }

    unsigned char digest[16];
    MD5_CTX context;
    int bytesRead;
    char buffer[4096];
    MD5_Init(&context);

    while ((bytesRead = fread(buffer, sizeof(char), sizeof(buffer), file)) != 0)
    {
        MD5_Update(&context, buffer, bytesRead);
    }

    MD5_Final(digest, &context);
    fclose(file);

    for (int i = 0; i < 16; ++i)
    {
        printf("%02x", digest[i]);
    }
    printf("\n");

    utils_hex2str(digest, out_string, MD5_DIGEST_LENGTH);
    LOG_I("OTA info packHash: %s\n", out_string);
    return 0;
}

int ota_url_space_fill(char *i_string, char *o_string, int o_size)
{
    int i = 0, j = 0;
    int length = strlen(i_string);

    for (i = 0; i < length; i++)
    {
        if (i_string[i] != ' ')
            o_string[j] = i_string[i];
        else
        {
            o_string[j++] = '%';
            o_string[j++] = '2';
            o_string[j] = '0';
        }
        j++;
        if (j >= o_size)
            return -1;
    }

    return 0;
}

int ota_url_space_lessen(char *i_string, char *o_string, int o_size)
{
    int i = 0, j = 0;
    int length = strlen(i_string);

    for (i = 0; i < length; i++)
    {
        if (i_string[i] != ' ')
        {
            o_string[j] = i_string[i];
            j++;
        }

        if (j >= o_size)
            return -1;
    }

    return 0;
}

/**
 * 内部要求,与云端交互转换格式修改为 AA.BB.CC 
*/
void hl_convert_version_string_to_version2(const char *version_string, char *version, uint32_t size)
{
    char *token;
    char *saveptr;
    char tmp[256];
    int count = 0;
    int version_num[3] = {0};
    strncpy(tmp, version_string, sizeof(tmp));

    token = strtok_r(tmp, ".", &saveptr);
    while (token != NULL)
    {
        while (*token != '\0' && isdigit(*token) == 0)
            token++;
        version_num[count] = strtoul(token, NULL, 10);
        token = strtok_r(NULL, ".", &saveptr);
        count++;
        if (count == 3)
            break;
    }
    snprintf(version, size, "%02d.%02d.%02d", version_num[0], version_num[1], version_num[2]);
}

// void set_ota_startup(void)
// {
//     ota_startup = STARTUP_DIDNT_OTA;
// }

// void reset_ota_startup(void)
// {
//     ota_startup = STARTUP_DID_OTA;
// }

// int get_ota_startup(void)
// {
//     return ota_startup;
// }

time_t get_current_timestamp(void)
{
    time_t utc_now;
    utc_now = time(NULL);
    utc_now += 8 * 60 * 60;
    return utc_now;
}

// brief :获取此次是否需要进入ota升级
// int ota_get_upgrade_cmd(bool is_active)
// {
//     int upgrade = 0;
//     if (is_active == true)
//     {
//         // 主动请求
//         printf("ota_get cmd update : %d\n",ota_info.update);
//         upgrade = ota_info.update;
//     }
//     else
//     {
//         // 被动请求(开机上电第一次)
//         if (ota_info.update == -1)
//             upgrade = -1;
//         if (ota_info.update == 1)
//         {
//             if (ota_info.updateStrategy == 1)
//             {
//                 upgrade = 1;
//             }
//             else if (ota_info.updateStrategy == 2)
//             {
//                 if (ui_param.ota_no_remind_switch == 1)
//                 {
//                     if (get_current_timestamp() - ui_param.ota_no_remind_tick > OTA_JUMP_TIME)
//                     {
//                         ui_param.ota_no_remind_switch = 0;
//                         ui_params_save();
//                         upgrade = 1;
//                     }
//                     else
//                     {
//                         LOG_I("current time :%ld < no remind time:%ld,thus ignore ota upgrade!\n", get_current_timestamp(), ui_param.ota_no_remind_tick + OTA_JUMP_TIME);
//                         upgrade = 0;
//                     }
//                 }
//                 else
//                 {
//                     upgrade = 1;
//                 }
//             }
//         }
//     }

//     return upgrade;
// }

// void ota_get_upgrade_version(char *version, int len)
// {
//     strncpy(version, ota_info.version, len);
// }

// int ota_get_upgrade_strategy(void)
// {
//     return ota_info.updateStrategy;
// }

// char *ota_get_upgrade_md5(void)
// {
//     return ota_info.packageHash;
// }

void get_ota_upgrade_filepath(char *filepath, int len)
{
    hl_disk_get_default_mountpoint(HL_DISK_TYPE_EMMC, OTA_FILE_NAME, filepath, len);
}

static void get_client_id(char *client_id, int len)
{
    hl_get_chipid(client_id, len);
}

// int get_ota_upgrade_info_pthread_create(void)
// {
//     LOG_I("get_ota_upgrade_info_pthread_create\n");
//     hl_tpool_create_thread(&ota_decompress_thread, ota_get_info_routine, NULL, 0, 0, 0, 0);
//     hl_tpool_wait_started(ota_decompress_thread, 0);
//     return 0;
// }

int get_ota_upgrade_info_pthread_destroy(void)
{
    LOG_I("get_ota_upgrade_info_pthread_destroy\n");
    hl_tpool_wait_completed(ota_decompress_thread, 0);
    hl_tpool_destory_thread(&ota_decompress_thread);
    return 0;
}

uint8_t get_thread_complete_status(void)
{
    return thread_complete_status;
}

/*
    @brief :获取ota升级信息线程的句柄
*/
hl_tpool_thread_t get_ota_upgrade_info_thread_handle(void)
{
    return ota_decompress_thread;
}

int ota_download_upgradefile_init(void)
{
    LOG_I("ota_download_upgradefile_init \n");
    char path[1024] = {0};
    get_ota_upgrade_filepath(path, sizeof(path));
    hl_curl_download_task_create(&download_ctx, ota_info.packageUrl, path);
    if (download_ctx == NULL)
    {
        LOG_I("hl_curl_download_task_create failed (ota upgrade file download)\n");
        return -1;
    }
    return 0;
}

int ota_download_upgradefile_deinit(void)
{
    LOG_I("ota_download_upgradefile_deinit \n");
    hl_curl_download_task_destory(&download_ctx);
}

hl_curl_download_state_t get_ota_download_state(uint64_t *download_offset, uint64_t *download_size)
{
    return hl_curl_download_get_state(download_ctx, download_offset, download_size);
}

/*
    @brief: 获取ota升级信息
*/
static void ota_get_info_routine(hl_tpool_thread_t thread, void *args)
{
    LOG_I("ota_get_info_routine start\n");
    usleep(500 * 1000); // 留500ms 时间给 /etc/resolv.conf

    thread_complete_status = 0;
    do
    {
        char response[1100] = {0};
        char DeviceId[64] = {0};
        char m_marking[256] = {0};
        // 生成deviceID
        get_client_id(DeviceId, sizeof(DeviceId));

        // 获取版本号
        char current_version[256] = {0};
        // const board_information_t *board_information = board_information_get_instance();
        // // hl_convert_version_string_to_version((const char *)board_information->firmware_version, current_version, sizeof(current_version));
        // hl_convert_version_string_to_version2((const char *)board_information->firmware_version, current_version, sizeof(current_version));

        // ota url 空格 转义
        // ota_url_space_fill(machine_info.cbd_marking, m_marking, sizeof(m_marking));
        ota_url_space_fill(DEV_MARKING, m_marking, sizeof(m_marking));

        char url[2048] = {0};
        sprintf(url, "%s%s%s%s%s%s%s%s%s%s%s%d", http_address_list_ota[ENVIRONMENT_CHOOSE], http_path_list_ota[0], "?machineType=", m_marking, "&machineId=", DeviceId, "&version=", current_version, "&lan=", language_str[tr_get_language()], "&firmwareType=", 1);
        // sprintf(url, "%s%s%s%s%s%s%s%s%s%s%s%d", http_address_list_ota[ENVIRONMENT_CHOOSE], http_path_list_ota[0], "?machineType=", m_marking, "&machineId=", DeviceId, "&version=", current_version, "&lan=", language_str[0], "&firmwareType=", 1);
        LOG_I("curl url : %s\n", url);

        // 避免失败,增加重试
        int curl_rc = 0;
        int simple_retry = 5;
        do
        {
            if ((curl_rc = hl_curl_get(url, response, sizeof(response), 15)) != 0)
            {
                LOG_I("curl get ota info failed simple_retry %d\n", simple_retry);
                if (simple_retry-- <= 0)
                {
                    ota_info.update = -1;
                    break;
                }
            }
        } while (curl_rc != 0);

        // 解析
        LOG_I("respone : %s\n", response);
        cJSON *root = cJSON_Parse(response);
        if (root == NULL)
        {
            LOG_E("ota_get_info failed\n");
            ota_info.update = -1;
            break;
        }
        cJSON *success = cJSON_GetObjectItem(root, "success");
        cJSON *data = cJSON_GetObjectItem(root, "data");

        if (success == NULL || data == NULL)
        {
            LOG_I("ota get response failed\n");
            cJSON_Delete(root);
            ota_info.update = -1;
            break;
        }

        if (cJSON_IsTrue(success) == 1)
        {
            cJSON *obj = NULL;
            obj = cJSON_GetObjectItem(data, "update");
            if (cJSON_IsTrue(obj) == 1)
                ota_info.update = 1;
            else
                ota_info.update = 0;

            if (ota_info.update == 1)
            {
                obj = cJSON_GetObjectItem(data, "version");
                strncpy(ota_info.version, cJSON_GetStringValue(obj), sizeof(ota_info.version));

                obj = cJSON_GetObjectItem(data, "packageUrl");
                strncpy(ota_info.packageUrl, cJSON_GetStringValue(obj), sizeof(ota_info.packageUrl));

                obj = cJSON_GetObjectItem(data, "packageHash");
                strncpy(ota_info.packageHash, cJSON_GetStringValue(obj), sizeof(ota_info.packageHash));

                obj = cJSON_GetObjectItem(data, "updateStrategy");
                ota_info.updateStrategy = cJSON_GetNumberValue(obj);

                obj = cJSON_GetObjectItem(data, "log");
                strncpy(ota_info.log, cJSON_GetStringValue(obj), sizeof(ota_info.log));

                obj = cJSON_GetObjectItem(data, "timeMS");
                // 更新时间
                time_t timestamp = cJSON_GetNumberValue(obj) / 1000;
                stime(&timestamp);

                LOG_I("update : %d ,log: %s\n", ota_info.update, ota_info.log);
                LOG_I("strategy: %d ,url : %s, md5: %s ,version : %s\n", ota_info.updateStrategy, ota_info.packageUrl, ota_info.packageHash, ota_info.version);
            }
            else
                LOG_I("update : %d\n", ota_info.update);
        }

        cJSON_Delete(root);
        LOG_I("get ota upgrade info finished\n");
    } while (0);

    thread_complete_status = 1;
    LOG_I("ota_get_info_routine exit\n");
}

void ota_init(void)
{
    // for (int i = 0; i < OTA_FIREMARE_CH_MAX; i++)
    //     ota_ctx.ota_strate[i] = 0;

    for (int i = 0; i < OTA_FIREMARE_CH_MAX; i++)
        ota_ctx.ota_api_stat[i] = OTA_API_STAT_INIT;

    pthread_mutex_init(&ota_ctx.pth_lock, NULL);

    return;
}

int OTA_Ioctl(OTA_CH_t ch, OTA_CmdType_t type, void *buf, int buf_len)
{
    if (ch < OTA_FIREMARE_CH_SYS || ch >= OTA_FIREMARE_CH_MAX)
    {
        return -1;
    }

    switch (type)
    {
    case OTAG_GET_VERSION:
    {
        strncpy(buf, ota_ctx.info[ch].version, buf_len);
        ((char *)buf)[buf_len - 1] = '\0';
    }
    break;

    case OTAG_SET_VERSION:
    {
        strncpy(ota_ctx.info[ch].version, buf, sizeof(ota_ctx.info[ch].version));
        ((char *)ota_ctx.info[ch].version)[sizeof(ota_ctx.info[ch].version) - 1] = '\0';
    }
    break;

    case OTAG_FETCH_FIRMWARE:
    {
    }
    break;

    case OTAG_BURN_FIRMWARE:
    {
    }
    break;

    case OTAG_GET_FETCH_PROCESS:
    {
    }
    break;

    case OTAG_GET_BURN_PROCESS:
    {
        *((int *)buf) = ota_ctx.burn_pro[ch];
        return 0;
    }
    break;

    case OTAG_FILE_SIZE:
    {
    }
    break;
    }

    return -1;
}

OTA_API_ST_t ota_get_info_result(OTA_CH_t ch, struct _ota_info *info)
{
    if (ch < OTA_FIREMARE_CH_SYS || ch >= OTA_FIREMARE_CH_MAX)
    {
        return -1;
    }

    if (ota_ctx.ota_api_stat[ch] == OTA_API_STAT_SUCCESS && info != NULL)
        *info = ota_ctx.info[ch];
    return ota_ctx.ota_api_stat[ch];
}

int ota_get_info_request(OTA_CH_t ch)
{
    if (ota_ctx.get_info[ch] != NULL && ota_ctx.ota_api_stat[ch] != OTA_API_STAT_REQUESTING)
    {
        // int pthread_kill_err;
        pthread_mutex_lock(&ota_ctx.pth_lock);
        // pthread_kill_err = pthread_kill(ota_ctx.pth[ch][OTAG_GET_VERSION],0);
        // if(pthread_kill_err == ESRCH)
        //     printf("线程不存在或者已经退出\n");
        // else if (pthread_kill_err == EINVAL)
        //     printf("发送信号非法\n");
        // else
        //     printf("线程目前仍然存活");
        LOG_I("[%s] create a thread for getting ota version.\n", __FUNCTION__);
        pthread_create(&ota_ctx.pth[ch][OTAG_GET_VERSION], NULL, ota_ctx.get_info[ch], &ota_ctx);
        ota_ctx.ota_api_stat[ch] = OTA_API_STAT_REQUESTING; // 请求中
        pthread_mutex_unlock(&ota_ctx.pth_lock);
        return 0;
    }
    else
        return -1;
}

/**
 * 函数处理注册
 */
// OTA_DEFINE_CMD(OTA_FIREMARE_CH_AIC, OTAG_GET_VERSION)
#if CONFIG_SUPPORT_AIC
void *ota_get_info_AIC(void *arg)
{
    LOG_I("AIC get version start\n");
#if 1 // 暂时屏蔽
    if (aic_get_online() == false)
    {
        /* AI 摄像头不在线 */
        pthread_mutex_lock(&ota_ctx.pth_lock);
        ota_ctx.ota_api_stat[OTA_FIREMARE_CH_AIC] = OTA_API_STAT_SUCCESS;
        ota_ctx.info[OTA_FIREMARE_CH_AIC].update = false;
        pthread_mutex_unlock(&ota_ctx.pth_lock);
        pthread_exit((void *)0);
        return NULL;
    }
#endif

    do
    {
        char response[1100] = {0};
        char DeviceId[20] = {0};
        char m_marking[256] = {0};
        char url[2048] = {0};
        bool need_update = false;

        // 生成deviceID
        get_client_id(DeviceId, sizeof(DeviceId));

        // 获取版本号
        char current_version[256] = {0};
        hl_convert_version_string_to_version2(aic_get_version(), current_version, sizeof(current_version));

        // ota url 空格 转义
        // ota_url_space_fill(machine_info.cbd_marking, m_marking, sizeof(m_marking));
        // ota_url_space_fill(DEV_MARKING, m_marking, sizeof(m_marking));
        ota_url_space_lessen(DEV_MARKING, m_marking, sizeof(m_marking));
        // sprintf(current_version, "%s", JENKINS_VERSION);

        sprintf(url, "%s%s%s%s%s%s%s%s%s%s%s%d", 
        http_address_list_ota[ENVIRONMENT_CHOOSE],
        http_path_list_ota[0],
        "?machineType=", m_marking, 
        "&machineId=", DeviceId, 
        "&version=", current_version, 
        "&lan=", language_str[tr_get_language()],
        "&firmwareType=", 2);

        LOG_I("AIC curl url : %s\n", url);

        if (hl_curl_get(url, response, sizeof(response), 2 * 60) != 0)
        {
            LOG_I("AIC curl get ota info failed\n");
            pthread_mutex_lock(&ota_ctx.pth_lock);
            ota_ctx.ota_api_stat[OTA_FIREMARE_CH_AIC] = OTA_API_STAT_TIMEOUT;
            pthread_mutex_unlock(&ota_ctx.pth_lock);
        }
        else
        {
            LOG_I("AIC respone : %s\n", response);
            cJSON *root = cJSON_Parse(response);
            if (root == NULL)
            {
                LOG_E("AIC ota_get_info failed\n");
                pthread_mutex_lock(&ota_ctx.pth_lock);
                ota_ctx.ota_api_stat[OTA_FIREMARE_CH_AIC] = OTA_API_STAT_FAILED;
                pthread_mutex_unlock(&ota_ctx.pth_lock);
            }
            else
            {
                LOG_I("AIC ota_get_info success\n");
                cJSON *success, *data, *update, *version, *packageUrl, *packageHash, *updateStrategy, *timeMS, *log;
                time_t timestamp;
                pthread_mutex_lock(&ota_ctx.pth_lock);
                success = cJSON_GetObjectItem(root, "success");
                data = cJSON_GetObjectItem(root, "data");
                if (success != NULL && data != NULL && cJSON_IsTrue(success) == 1)
                {
                    update = cJSON_GetObjectItem(data, "update");
                    if (cJSON_IsTrue(update) == 1)
                        ota_ctx.info[OTA_FIREMARE_CH_AIC].update = true;
                    else
                        ota_ctx.info[OTA_FIREMARE_CH_AIC].update = false;
                    printf("update = %d\n", ota_ctx.info[OTA_FIREMARE_CH_AIC].update);

                    if (ota_ctx.info[OTA_FIREMARE_CH_AIC].update == true)
                    {
                        version = cJSON_GetObjectItem(data, "version");
                        strncpy(ota_ctx.info[OTA_FIREMARE_CH_AIC].version, version->valuestring, sizeof(ota_ctx.info[OTA_FIREMARE_CH_AIC].version));
                        packageUrl = cJSON_GetObjectItem(data, "packageUrl");
                        strncpy(ota_ctx.info[OTA_FIREMARE_CH_AIC].packUrl, packageUrl->valuestring, sizeof(ota_ctx.info[OTA_FIREMARE_CH_AIC].packUrl));
                        packageHash = cJSON_GetObjectItem(data, "packageHash");
                        strncpy(ota_ctx.info[OTA_FIREMARE_CH_AIC].packHash, packageHash->valuestring, sizeof(ota_ctx.info[OTA_FIREMARE_CH_AIC].packHash));
                        updateStrategy = cJSON_GetObjectItem(data, "updateStrategy");
                        ota_ctx.info[OTA_FIREMARE_CH_AIC].updateStrateg = cJSON_GetNumberValue(updateStrategy);
                        /* 更新时间 */
                        timeMS = cJSON_GetObjectItem(data, "timeMS");
                        timestamp = cJSON_GetNumberValue(timeMS) / 1000;
                        stime(&timestamp);

                        log = cJSON_GetObjectItem(data, "log");
                        strncpy(ota_ctx.info[OTA_FIREMARE_CH_AIC].log, log->valuestring, sizeof(ota_ctx.info[OTA_FIREMARE_CH_AIC].log));

                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_AIC].version = %s\n", ota_ctx.info[OTA_FIREMARE_CH_AIC].version);
                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_AIC].packUrl = %s\n", ota_ctx.info[OTA_FIREMARE_CH_AIC].packUrl);
                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_AIC].packHash = %s\n", ota_ctx.info[OTA_FIREMARE_CH_AIC].packHash);
                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_AIC].updateStrateg = %d\n", ota_ctx.info[OTA_FIREMARE_CH_AIC].updateStrateg);
                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_AIC].log = %s\n", ota_ctx.info[OTA_FIREMARE_CH_AIC].log);
                    }

                    ota_ctx.ota_api_stat[OTA_FIREMARE_CH_AIC] = OTA_API_STAT_SUCCESS;
                }
                else
                {
                    ota_ctx.ota_api_stat[OTA_FIREMARE_CH_AIC] = OTA_API_STAT_FAILED;
                }
                pthread_mutex_unlock(&ota_ctx.pth_lock);
            }
            cJSON_Delete(root);
        }

    } while (0);

    LOG_I("AIC pthread exit\n");
    pthread_exit((void *)0);

    return NULL;
}
#endif

#if CONFIG_SUPPORT_AIC
int ota_aic_fetch_start(struct _ota_info *info)
{
    char aic_file_name[64];
    char _path[PATH_MAX_LEN];
    if (info == NULL)
        return -1;
    /* 设置目标路径 */
    sprintf(aic_file_name, "UC021_V%s_ota.bin", info->version);
    hl_disk_get_default_mountpoint(HL_DISK_TYPE_EMMC, aic_file_name, _path, sizeof(_path));

    /* 开始下载 */
    printf("path:%s\n", _path);
    return hl_curl_download_task_create(&download_ctx, info->packUrl, _path);
}
#endif


int ota_sys_fetch_start(struct _ota_info *info)
{
    char sys_file_name[64];
    char _path[PATH_MAX_LEN];
    if (info == NULL)
        return -1;
    /* 设置目标路径 */
    get_ota_upgrade_filepath(_path, sizeof(_path));

    /* 开始下载 */
    return hl_curl_download_task_create(&download_ctx, info->packUrl, _path);
}

void ota_aic_upgrade_task_destory(void)
{
    hl_curl_download_task_destory(&download_ctx);
}

#if CONFIG_SUPPORT_AIC
pthread_t ota_aic_burn_pth;
void *_ota_aic_burn_start(void *arg)
{
    do
    {
        char cmd[1024];
        char path[PATH_MAX_LEN];
        char *ota_name = (char *)arg;
        if (ota_name == NULL)
            return NULL;
        hl_disk_get_default_mountpoint(HL_DISK_TYPE_EMMC, ota_name, path, sizeof(path));
        sprintf(cmd, "/lib/firmware/cdcupdate_sunxi -d /dev/media0 -f %s -l /user-resource/ai_ipc", path);
        LOG_I("%s\n", cmd);
        if (access(path, F_OK) != 0)
        {
            LOG_I("%s not exist!\n", path);
            return NULL;
        }

        {
            // AI摄像头升级没有进度，在应用做个假的
            ota_ctx.burn_pro[OTA_FIREMARE_CH_AIC] = 0;
            LOG_I("cdcupdate start\n");
            hl_camera_scan_enable(0); // 关闭摄像头扫描功能
            hl_system(cmd);
            ota_ctx.burn_pro[OTA_FIREMARE_CH_AIC] = 100;
            LOG_I("cdcupdate finish\n");
            hl_system("sync");
            hl_camera_scan_enable(1); // 开启摄像头扫描功能
        }
    } while (0);
}
#endif

#if CONFIG_SUPPORT_AIC
int ota_aic_burn_start(char *ota_name)
{
    pthread_create(&ota_aic_burn_pth, NULL, _ota_aic_burn_start, ota_name);
    return 0;
}
#endif

/**
 * @brief OTA 信息 ( 系统固件 )
 */
void *ota_get_info_SYS(void *arg)
{
    LOG_I("[%s] System get version start\n", __FUNCTION__);

    do
    {
        char response[1100] = {0};
        char DeviceId[64] = {0};
        char m_marking[256] = {0};
        char url[2048] = {0};
        bool need_update = false;

        // 生成deviceID
        get_client_id(DeviceId, sizeof(DeviceId));

        // 获取版本号
        char current_version[256] = {0};
        // const board_information_t *board_information = board_information_get_instance();
        // hl_convert_version_string_to_version2((const char *)board_information->firmware_version, current_version, sizeof(current_version));

        // ota url 空格 转义
        // ota_url_space_fill(machine_info.cbd_marking, m_marking, sizeof(m_marking));
        ota_url_space_fill

            (DEV_MARKING, m_marking, sizeof(m_marking));

        sprintf(current_version, "%s", JENKINS_VERSION); /* This macro is supposed be fetched from cloud */
        sprintf(url, "%s%s%s%s%s%s%s%s%s%s%s%d",
                http_address_list_ota[ENVIRONMENT_CHOOSE],
                http_path_list_ota[0],
                "?machineType=", m_marking,
                "&machineId=", DeviceId,
                "&version=", current_version,
                "&lan=", language_str[tr_get_language()],
                // "&lan=", language_str[0],
                "&firmwareType=", 1);
        LOG_D("curl url : %s\n", url);

        if (hl_curl_get(url, response, sizeof(response), 2 * 60) != 0)
        {
            LOG_I("curl get ota info failed\n");
            pthread_mutex_lock(&ota_ctx.pth_lock);
            ota_ctx.ota_api_stat[OTA_FIREMARE_CH_SYS] = OTA_API_STAT_TIMEOUT;
            pthread_mutex_unlock(&ota_ctx.pth_lock);
        }
        else
        {
            printf("respone : %s\n", response);
            cJSON *root = cJSON_Parse(response);
            if (root == NULL)
            {
                LOG_E("ota_get_info failed\n");
                pthread_mutex_lock(&ota_ctx.pth_lock);
                ota_ctx.ota_api_stat[OTA_FIREMARE_CH_SYS] = OTA_API_STAT_FAILED;
                pthread_mutex_unlock(&ota_ctx.pth_lock);
            }
            else
            {
                LOG_I("ota_get_info success\n");
                cJSON *success, *data, *update, *version, *packageUrl, *packageHash, *updateStrategy, *timeMS, *log;
                time_t timestamp;
                pthread_mutex_lock(&ota_ctx.pth_lock);
                success = cJSON_GetObjectItem(root, "success");
                data = cJSON_GetObjectItem(root, "data");
                if (success != NULL && data != NULL && cJSON_IsTrue(success) == 1)
                {
                    update = cJSON_GetObjectItem(data, "update");
                    if (cJSON_IsTrue(update) == 1)
                        ota_ctx.info[OTA_FIREMARE_CH_SYS].update = true;
                    else
                        ota_ctx.info[OTA_FIREMARE_CH_SYS].update = false;
                    printf("sys update = %d\n", ota_ctx.info[OTA_FIREMARE_CH_SYS].update);

                    if (ota_ctx.info[OTA_FIREMARE_CH_SYS].update == true)
                    {
                        version = cJSON_GetObjectItem(data, "version");
                        strncpy(ota_ctx.info[OTA_FIREMARE_CH_SYS].version, version->valuestring, sizeof(ota_ctx.info[OTA_FIREMARE_CH_SYS].version));
                        packageUrl = cJSON_GetObjectItem(data, "packageUrl");
                        strncpy(ota_ctx.info[OTA_FIREMARE_CH_SYS].packUrl, packageUrl->valuestring, sizeof(ota_ctx.info[OTA_FIREMARE_CH_SYS].packUrl));
                        packageHash = cJSON_GetObjectItem(data, "packageHash");
                        strncpy(ota_ctx.info[OTA_FIREMARE_CH_SYS].packHash, packageHash->valuestring, sizeof(ota_ctx.info[OTA_FIREMARE_CH_SYS].packHash));
                        updateStrategy = cJSON_GetObjectItem(data, "updateStrategy");
                        ota_ctx.info[OTA_FIREMARE_CH_SYS].updateStrateg = cJSON_GetNumberValue(updateStrategy);
                        /* 更新时间 */
                        timeMS = cJSON_GetObjectItem(data, "timeMS");
                        timestamp = cJSON_GetNumberValue(timeMS) / 1000;
                        stime(&timestamp);

                        log = cJSON_GetObjectItem(data, "log");
                        strncpy(ota_ctx.info[OTA_FIREMARE_CH_SYS].log, log->valuestring, sizeof(ota_ctx.info[OTA_FIREMARE_CH_SYS].log));

                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_SYS].version = %s\n", ota_ctx.info[OTA_FIREMARE_CH_SYS].version);
                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_SYS].packUrl = %s\n", ota_ctx.info[OTA_FIREMARE_CH_SYS].packUrl);
                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_SYS].packHash = %s\n", ota_ctx.info[OTA_FIREMARE_CH_SYS].packHash);
                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_SYS].updateStrateg = %d\n", ota_ctx.info[OTA_FIREMARE_CH_SYS].updateStrateg);
                        LOG_D("ota_ctx.info[OTA_FIREMARE_CH_SYS].log = %s\n", ota_ctx.info[OTA_FIREMARE_CH_SYS].log);
                    }

                    ota_ctx.ota_api_stat[OTA_FIREMARE_CH_SYS] = OTA_API_STAT_SUCCESS;
                }
                else
                {
                    ota_ctx.ota_api_stat[OTA_FIREMARE_CH_SYS] = OTA_API_STAT_FAILED;
                }
                pthread_mutex_unlock(&ota_ctx.pth_lock);
            }
            cJSON_Delete(root);
        }

    } while (0);

    return NULL;
}

// utility function to compare each substring of version1
// and version2
int compareSubstr(char* substr_version1,
                  char* substr_version2,
                  int len_substr_version1,
                  int len_substr_version2)
{
    // if length of substring of version 1 is greater then
    // it means value of substr of version1 is also greater
    if (len_substr_version1 > len_substr_version2)
        return 1;

    else if (len_substr_version1 < len_substr_version2)
        return -1;

    // when length of the substrings of both versions is
    // same.
    else {
        int i = 0, j = 0;

        // compare each character of both substrings and
        // return accordingly.
        while (i < len_substr_version1) {
            if (substr_version1[i] < substr_version2[j])
                return -1;
            else if (substr_version1[i]
                     > substr_version2[j])
                return 1;
            i++, j++;
        }
        return 0;
    }
}

// function to compare two versions.
int compareVersion(char* version1, char* version2)
{
    if (version1 == NULL || version2 == NULL){
        return -2;
    }

    int len_version1 = strlen(version1);
    int len_version2 = strlen(version2);

    char* substr_version1
        = (char*)malloc(sizeof(char) * 100);
    char* substr_version2
        = (char*)malloc(sizeof(char) * 100);

    //LOG_I("[%s] ver1:%s ver2:%s\n", __FUNCTION__, version1, version2);
    // loop until both strings are exhausted.
    // and extract the substrings from version1 and version2
    int i = 0, j = 0;
    int res = 0;
    while (i < len_version1 || j < len_version2) {
        int p = 0, q = 0;

        // skip the leading zeros in version1 string.
        while (version1[i] == '0')
            i++;

        // skip the leading zeros in version2 string.
        while (version2[j] == '0')
            j++;

        // extract the substring from version1.
        while (version1[i] != '.' && i < len_version1)
            substr_version1[p++] = version1[i++];

        // extract the substring from version2.
        while (version2[j] != '.' && j < len_version2)
            substr_version2[q++] = version2[j++];

        res = compareSubstr(substr_version1,
                                substr_version2, p, q);

        // if res is either -1 or +1 then simply return.
        if (res)
            break;
        i++;
        j++;
    }

    free(substr_version1);
    free(substr_version2);
    // here both versions are exhausted it implicitly
    // means that both strings are equal.
    return res;
}

bool is_ota_version_greater(char* cur_version)
{
    if (cur_version == NULL)
    {
        return 0;
    }

    if (compareVersion(ota_ctx.info[OTA_FIREMARE_CH_SYS].version, cur_version) == 1)
    {
        return 1;
    }

    return 0;
}

#endif
