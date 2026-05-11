#define _XOPEN_SOURCE 500
#include "config.h"

#if CONFIG_SUPPORT_TLP
#include "aic_tlp.h"
#include "hl_camera.h"
#include "hl_disk.h"
#include "hl_common.h"
#include "hl_tpool.h"
#include "utils.h"
#include "params.h"
#include "ai_camera.h"

#include <stdio.h>
#include <ftw.h>
#include <sys/stat.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>


#define LOG_TAG "aic_tlp"
#undef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#include "log.h"

#define TLP_BY_WRITE_FILE 1

typedef struct
{
    bool started;
    bool video_export_enable;
    char path_tmp[1024];
    char path_tag[1024];
    uint32_t format;
    uint32_t layer;
    char tlp_layer_path[1024];
    int png_count; // 图片数量
    int capture;
    int snapshot;                 // 快拍
    uint64_t snapshot_start_tick; /* 快拍开始时间 */
    uint64_t snapshot_int_tick;   /* 快拍间隔时间 */
    int snapshot_index;
    pthread_mutex_t lock;
    uint32_t framecount;
    uint8_t progress; // 视频生成进度

    // ENCODER
    AVCodec *enc;
    AVFormatContext *fmt_ctx;
    AVCodecContext *enc_ctx;
    AVStream *stream;
    AVPacket *pkt;

    int make_video;
    int stream_index;
} ai_tlp_t;

typedef enum
{
    TLP_THREAD_EVENT_ID_MJPEG,
    TLP_THREAD_EVENT_ID_START,
    TLP_THREAD_EVENT_ID_END,
} tlp_thread_event_id_t;

typedef struct
{
    tlp_thread_event_id_t id;
    uint8_t *data;
    uint32_t size;
    int h264;
    int make_video;
    uint32_t capture_total_frame;
    bool video_export_enable;
    /* start */
    char name[1024];
} tlp_thread_event_t;

static ai_tlp_t ai_tlp = {0};
static hl_tpool_thread_t mp4_handle_thread;
static hl_tpool_thread_t mp4_export_thread;

static void tlp_camera_callback(const void *data, void *user_data);
static int tlp_mpjeg_to_mp4(uint8_t *src, uint32_t src_len);
static int tlp_create_mp4(const char *path, int cw, int ch);
static void mp4_handle_routine(hl_tpool_thread_t thread, void *args);
static void mp4_export_handle(hl_tpool_thread_t thread, void *args);
static void tlp_close_mp4(void);
static int tlp_h264_to_mp4(uint8_t *src, uint32_t src_len);
static int tlp_mjpeg_to_h264(uint8_t *i_data, uint32_t i_size, uint8_t *o_data, uint32_t o_size);
static int tlp_yuyv422_to_mp4(uint8_t *src, uint32_t src_len);
static int tlp_mjpeg_to_yuyv422(uint8_t *i_data, uint32_t i_size, uint8_t **o_data, uint32_t *o_size);

int aic_tlp_check_mp4_handler()
{
    char disk_path[1024];
    char line[128];
    FILE *fp;
    int retry = 0;

    if (hl_disk_get_mountpoint(HL_DISK_TYPE_EMMC, 1, NULL, disk_path, sizeof(disk_path)) != 0)
    {
        LOG_I("hl_disk_get_mountpoint failed\n");
        return -1;
    }

    LOG_D("disk_path : %s\n", disk_path);

    do
    {
        fp = popen("ls -lt /user-resource/aic_tlp/ | awk \'{print $5,$9}\'", "r");
        if (fp == NULL)
        {
            LOG_I("popen failed %s\n", strerror(errno));
            usleep(100000);
        }
        else
        {
            LOG_D("fp open\n");
        }
    } while (fp == NULL && retry++ < 10);

    if (fp == NULL)
        LOG_D("fp == NULL\n");

    if (fp != NULL)
    {
        uint32_t file_size;
        uint64_t file_total_size = 0;
        uint32_t file_num = 0;
        char file_name[128];
        bool is_delete = false;

        while (fgets(line, sizeof(line), fp) != NULL)
        {
            // printf("%s", line);
            sscanf(line, "%d %s", &file_size, file_name);

            if (strstr(file_name, ".mp4.tmp") != NULL)
            {
                utils_vfork_system("rm %s/%s", "/user-resource/aic_tlp", file_name);
            }
            else
            {
                if (is_delete == false)
                {
                    file_num++;
                    file_total_size += (uint64_t)file_size;
                    printf("size : %llu\n", file_total_size);
                    printf("size M : %llu MB\n", (file_total_size >> 20));
                    if ((file_total_size >> 20) > TLP_FILE_TOTAL_SIZE || file_num > TLP_FILE_NUM_MAX)
                    {
                        printf("need delete : %s\n", file_name);
                        is_delete = true; // 触发删除操作
                        file_total_size -= (uint64_t)file_size;
                        utils_vfork_system("rm -r %s/%s", "/user-resource/aic_tlp", file_name);
                    }
                }
                else
                {
                    // 触发删除操作，时间更久的文件都需要删除
                    printf("need delete : %s\n", file_name);
                    utils_vfork_system("rm -r %s/%s", "/user-resource/aic_tlp", file_name);
                }
            }
            // printf("\n");
        }

        utils_vfork_system("sync");
        printf("file total : %lluMB\n", (file_total_size >> 20));
        pclose(fp);
    }

#if CONFIG_PRINT_HISTORY
    for (int i = 0; i < machine_info.print_history_valid_numbers; i++)
    {
        if (access(machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_path, F_OK) != 0)
            machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_state = 4; // 生成失败
        else
            machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_state = 1; // 生成成功
    }
    machine_info_save();
#endif

}

int aic_tlp_early_init()
{
    aic_tlp_check_mp4_handler();

    ai_tlp.capture = 0;
    pthread_mutex_init(&ai_tlp.lock, NULL);

    // 注册所有的编解码器和格式
    av_register_all();

    // 创建 mp4 处理线程
    hl_tpool_create_thread(&mp4_handle_thread, mp4_handle_routine, NULL, sizeof(tlp_thread_event_t), 256, 0, 0);

    // 创建 mp4 处理线程  dd
    hl_tpool_create_thread(&mp4_export_thread, mp4_export_handle, NULL, sizeof(tlp_thread_event_t), 256, 0, 0);
}

static void tlp_close_camera(void)
{
    hl_camera_unregister(tlp_camera_callback, NULL);
    // hl_camera_capture_off();
}

int aic_tlp_init(const char *name)
{
    tlp_thread_event_t ev;
    ev.id = TLP_THREAD_EVENT_ID_START;
    strncpy(ev.name, name, sizeof(ev.name));
    return hl_tpool_send_msg(mp4_handle_thread, &ev);
    // // 由于 ai camera 需要全程mjpeg拉流，这里不需要再打开摄像头

    // pthread_mutex_init(&ai_tlp.lock, NULL);

    // char disk_path[1024];
    // // 创建MP4
    // if (hl_disk_get_mountpoint(HL_DISK_TYPE_EMMC, 1, NULL, disk_path, sizeof(disk_path)) != 0)
    // {
    //     LOG_I("hl_disk_get_mountpoint failed\n");
    //     tlp_close_camera();
    //     pthread_mutex_destroy(&ai_tlp.lock);
    //     return -1;
    // }

    // snprintf(ai_tlp.path, sizeof(ai_tlp.path), "%s/%s.mp4.tmp", disk_path, name);
    // snprintf(ai_tlp.path2, sizeof(ai_tlp.path2), "%s/%s.mp4", disk_path, name);

    // int cw, ch;
    // cw=1280;        // AI 摄像头 mjpeg 固定 720P
    // ch=720;

    // if (tlp_create_mp4(ai_tlp.path, cw, ch) != 0)
    // {
    //     LOG_I("tlp_create_mp4 %s failed\n", ai_tlp.path);
    //     tlp_close_camera();
    //     pthread_mutex_destroy(&ai_tlp.lock);
    //     return -1;
    // }

    // if (hl_tpool_create_thread(&mp4_handle_thread, mp4_handle_routine, NULL, sizeof(tlp_thread_event_t), 32, 0, 0) != 0)
    // {
    //     LOG_I("tlp:hl_tpool_create_thread %s failed\n");
    //     if (access(ai_tlp.path, F_OK) == 0)
    //         hl_system("rm %s", ai_tlp.path);
    //     tlp_close_camera();
    //     pthread_mutex_destroy(&ai_tlp.lock);
    //     return -1;
    // }

    // hl_tpool_wait_started(mp4_handle_thread, 0);

    // aic_register(tlp_camera_callback, NULL);      // 最后调用注册 , 由于 AI摄像头 mjpeg 常驻.
}

int aic_tlp_start(const char *name)
{
    char disk_path[1024];
#if 1
    // 创建MP4
    if (hl_disk_get_mountpoint(HL_DISK_TYPE_EMMC, 1, NULL, disk_path, sizeof(disk_path)) != 0)
    {
        LOG_I("hl_disk_get_mountpoint failed\n");
        tlp_close_camera();
        pthread_mutex_destroy(&ai_tlp.lock);
        return -1;
    }

    snprintf(ai_tlp.path_tmp, sizeof(ai_tlp.path_tmp), "%s/%s.mp4.tmp", "/user-resource/aic_tlp", name);
    snprintf(ai_tlp.path_tag, sizeof(ai_tlp.path_tag), "%s/%s.mp4", "/user-resource/aic_tlp", name);
    LOG_I("ai_tlp path %s\n", ai_tlp.path_tag);
#else
    snprintf(ai_tlp.path_tmp, sizeof(ai_tlp.path_tmp), "%s/%s.mp4.tmp", "/mnt/exUDISK", "test_generate_tlp");
    snprintf(ai_tlp.path_tag, sizeof(ai_tlp.path_tag), "%s/%s.mp4", "/mnt/exUDISK", "test_generate_tlp");
#endif

    // strncpy(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].tlp_path, ai_tlp.path_tag,sizeof(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].tlp_path));
    // machine_info_save();

    int cw, ch;
    cw = TLP_RESOLUTION_WIDTH; // AI 摄像头 mjpeg 固定 720P
    ch = TLP_RESOLUTION_HEIGHT;

    if (tlp_create_mp4(ai_tlp.path_tmp, cw, ch) != 0)
    {
        LOG_I("tlp_create_mp4 %s failed\n", ai_tlp.path_tmp);
        tlp_close_camera();
        pthread_mutex_destroy(&ai_tlp.lock);
        return -1;
    }

    // aic_register(tlp_camera_callback, NULL); // 最后调用注册 , 由于 AI摄像头 mjpeg 常驻.
    return 0;
}

void aic_tlp_complte(int make_video, uint32_t tlp_total_frame)
{
    // 关闭摄像头
    tlp_close_camera();
    ai_tlp.snapshot = 0;
    // 关闭线程
    tlp_thread_event_t ev;
    ev.id = TLP_THREAD_EVENT_ID_END;
    ev.make_video = make_video;
    ev.capture_total_frame = tlp_total_frame;
    LOG_I("tlp_complted stream_index %d make_video %d\n", ai_tlp.stream_index, ev.make_video);
    hl_tpool_send_msg(mp4_handle_thread, &ev);
}

void aic_tlp_delete(const char *name)
{
    char file_path[1024];
    if (strcmp(name, "") != 0)
    {
        snprintf(file_path, sizeof(file_path), "rm -r /user-resource/aic_tlp/%s*",  name);
        hl_system("%s", file_path);
        LOG_I("delete file %s\n", file_path);
    }
}

void aic_tlp_export(const char *name, int video_export_enable)
{
    // aic_tlp_delete(name);
    // 关闭摄像头
    tlp_close_camera();
    ai_tlp.snapshot = 0;
    // 关闭线程
    tlp_thread_event_t ev;
    ev.video_export_enable = true;
    ev.make_video = 1;
    strncpy(ev.name, name, sizeof(ev.name));
    hl_tpool_send_msg(mp4_export_thread, &ev);
}

void aic_tlp_capture(uint32_t layer)
{
    LOG_I("tlp_capture %d\n", layer);
    pthread_mutex_lock(&ai_tlp.lock);
    ai_tlp.layer = layer;
    ai_tlp.capture = 1;
    pthread_mutex_unlock(&ai_tlp.lock);
}

void aic_tlp_snapshot()
{
    pthread_mutex_lock(&ai_tlp.lock);
    ai_tlp.snapshot = 1;
    ai_tlp.snapshot_start_tick = utils_get_current_tick();
    ai_tlp.snapshot_int_tick = utils_get_current_tick();
    ai_tlp.snapshot_index = 0;
    pthread_mutex_unlock(&ai_tlp.lock);
}

void aic_tlp_get_path(char *path, uint32_t size)
{
    strncpy(path, ai_tlp.path_tag, size);
}

static void tlp_camera_callback(const void *data, void *user_data)
{
    hl_camera_event_t *event = (hl_camera_event_t *)data;
    int i = 0, mjpeg_length = 0;
    bool got_mjpeg = false;

    switch (event->event)
    {
    case HL_CAMERA_EVENT_FRAME:
    {
        pthread_mutex_lock(&ai_tlp.lock);
        if (ai_tlp.capture)
        {
            // 抓取当前帧  JPEG图像帧的起始标记是0xFFD8，结束标记是0xFFD9。
            // if( ai_tlp.format == V4L2_PIX_FMT_MJPEG )
            if (event->data[0] == 0xFF && event->data[1] == 0xD8)
            {
                for (i = 0; i < event->data_length - 1; i++)
                {
                    if (event->data[i] == 0xFF && event->data[i + 1] == 0xD9)
                    {
                        // 拿到一帧 mjpeg
                        got_mjpeg = true;
                        mjpeg_length = i + 1;
                    }
                }
            }

            if (got_mjpeg == true)
            {
                got_mjpeg = false;
                tlp_thread_event_t ev;
                ev.id = TLP_THREAD_EVENT_ID_MJPEG;
                ev.data = malloc(mjpeg_length);
                ev.size = mjpeg_length;
                if (ev.data)
                {
                    memcpy(ev.data, event->data, ev.size);
                    LOG_I("mp4 send data\n");
                    if (hl_tpool_send_msg(mp4_handle_thread, &ev) != 0)
                    {
                        LOG_I("make mp4 queue full\n");
                        free(ev.data);
                    }
                    else
                    {
                        LOG_I("mjpeg send to tlp\n");
                    }
                }
            }
            ai_tlp.capture = 0;
        }

        if (ai_tlp.snapshot)
        {
            if (event->data[0] == 0xFF && event->data[1] == 0xD8)
            {
                for (i = 0; i < event->data_length - 1; i++)
                {
                    if (event->data[i] == 0xFF && event->data[i + 1] == 0xD9)
                    {
                        // 拿到一帧 mjpeg
                        got_mjpeg = true;
                        mjpeg_length = i + 1;
                    }
                }
            }

            // if (utils_get_current_tick() - ai_tlp.snapshot_start_tick < machine_param.aic_tlp_snapshot_time) // 4s 为最大限制
            // {
            //     if ((utils_get_current_tick() - ai_tlp.snapshot_int_tick) >= machine_param.aic_tlp_snapshot_int)
            //     {
            //         ai_tlp.snapshot_int_tick = utils_get_current_tick();
            //         ai_tlp.snapshot_index++;
            //         printf("snapshot index : %d\n", ai_tlp.snapshot_index);
            //         if (got_mjpeg == true)
            //         {
            //             got_mjpeg = false;
            //             tlp_thread_event_t ev;
            //             ev.id = TLP_THREAD_EVENT_ID_MJPEG;
            //             ev.data = malloc(mjpeg_length);
            //             ev.size = mjpeg_length;
            //             if (ev.data)
            //             {
            //                 memcpy(ev.data, event->data, ev.size);
            //                 tlp_thread_event_t request;
            //                 // if (hl_tpool_peek_msg(mp4_handle_thread, &request) != 0)
            //                 // {
            //                 // // 确认消费结束
            //                 if (hl_tpool_send_msg(mp4_handle_thread, &ev) != 0)
            //                 {
            //                     LOG_I("make mp4 queue full\n");
            //                     free(ev.data);
            //                 }
            //                 else
            //                 {
            //                     LOG_I("mjpeg send to tlp\n");
            //                 }
            //                 // }
            //                 // else
            //                 // {
            //                 //     // 线程未处理完，放弃这一帧
            //                 //     printf("there is message not handle\n");
            //                 //     free(ev.data);
            //                 // }
            //             }
            //         }
            //     }
            // }
            // else
            {
                ai_tlp.snapshot = 0;
                ai_tlp.snapshot_index = 0;
            }
        }
        pthread_mutex_unlock(&ai_tlp.lock);
    }
    break;
    }
}

static void mp4_handle_routine(hl_tpool_thread_t thread, void *args)
{
    tlp_thread_event_t request;
    printf("enter mp4_handle_routine\n");
    uint8_t *yuyv = NULL;
    uint32_t yuyv_length = 0;
    int ret;

    while (1)
    {
        if (hl_tpool_thread_recv_msg_try(thread, &request) == 0)
        {
            if (request.id == TLP_THREAD_EVENT_ID_START)
            {
                #if TLP_BY_WRITE_FILE
                
                // 删除/board-resource下所有.mp4后缀的文件夹
                hl_system("rm -rf /board-resource/video_mp4");
                
                // 创建 MP4
                LOG_I("mp4_handle_routine start name = %s\n", request.name);
                hl_system("mkdir -p \"video_mp4\"");
                sprintf(ai_tlp.tlp_layer_path, "/board-resource/video_mp4/%s", request.name);
                char video_filename[1024] = {0};
                sprintf(video_filename, "%s.mp4", ai_tlp.tlp_layer_path);
                LOG_I("mp4_handle_routine start video_filename = %s\n", video_filename);
                hl_system("mkdir -p \"%s\"",ai_tlp.tlp_layer_path);

                strncpy(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].tlp_path, ai_tlp.tlp_layer_path,sizeof(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].tlp_path));
                strncpy(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].video_path, video_filename,sizeof(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].video_path));
                machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].tlp_state = 1;
                machine_info_save();
                hl_camera_register(tlp_camera_callback, NULL); // 最后调用注册 , 由于 AI摄像头 mjpeg 常驻.
                ai_tlp.started = true;
#else
                ret = aic_tlp_start(request.name);
                if (ret == 0)
                    ai_tlp.started = true;
                LOG_D("ai_tlp.started : %d\n", ai_tlp.started);
#endif
            }
            else if (request.id == TLP_THREAD_EVENT_ID_MJPEG)
            {
                if (ai_tlp.started == true)
                {
#if TLP_BY_WRITE_FILE
                    char filename[1024];
                    snprintf(filename, sizeof(filename), "%s/tlp_layer_%d", ai_tlp.tlp_layer_path, ai_tlp.layer);
                    FILE *fp = fopen(filename, "wb");
                    if (fp == NULL)
                    {
                        LOG_E("open file %s failed\n", filename);
                        continue;
                    }

                    size_t written = fwrite(request.data, 1, request.size, fp);
                    
                    if (written != request.size)
                    {
                        LOG_E("write file %s failed\n", filename);
                        fclose(fp);
                        continue;
                    }
                    fclose(fp);
                    LOG_I("write file %s success\n", filename);
#else
                    int ret = tlp_mjpeg_to_yuyv422(request.data, request.size, &yuyv, &yuyv_length);
                    tlp_yuyv422_to_mp4(yuyv, yuyv_length);
                    free(yuyv);
#endif
                }
                free(request.data);
            }
            else if (request.id == TLP_THREAD_EVENT_ID_END)
            {
#if TLP_BY_WRITE_FILE

/*  延时摄影时间假计算 */

        for (int i = 0; i < machine_info.print_history_valid_numbers; i++)
        {
            printf("ai_tlp.tlp_layer_path = %s \t hist : %s\n", ai_tlp.tlp_layer_path, machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_path);
            if (strcmp(ai_tlp.tlp_layer_path, machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_path) == 0)
            {
                // find it
                // printf("find it uuid = %s\n\n",machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].uuid);
                printf("his index = %d\n", (machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE);
                printf("file path = %s\n", machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].filepath);
                if (access(ai_tlp.tlp_layer_path, F_OK) == 0)
                {
                    machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_time = ceil((double)request.capture_total_frame / TLP_FRAME_RATE); // 使用ceil函数向上取整
                }
                else
                {
                    machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_time = 0;
                }
                break;
            }
        }

        if (!request.make_video)
        {
            hl_system("rm -r \"%s\"", ai_tlp.tlp_layer_path);
            LOG_I("delete file \"%s\"\n", ai_tlp.tlp_layer_path);
        }
        else
        {
            if (access("/user-resource/aic_tlp", F_OK) != 0)
            {
                hl_system("mkdir -p /user-resource/aic_tlp");
            }
            hl_system("mv \"%s\" /user-resource/aic_tlp", ai_tlp.tlp_layer_path);
        }
        if (strncmp(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].tlp_path, "/board-resource/video_mp4", strlen("/board-resource/video_mp4")) == 0)
        {
            char new_path[1024];  
            snprintf(new_path, sizeof(new_path), "%s%s", "/user-resource/aic_tlp", machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].tlp_path + strlen("/board-resource/video_mp4"));  
            strcpy(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].tlp_path, new_path);
        }
        if (strncmp(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].video_path, "/board-resource/video_mp4", strlen("/board-resource/video_mp4")) == 0)
        {
            char new_path[1024];  
            snprintf(new_path, sizeof(new_path), "%s%s", "/user-resource/aic_tlp", machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].video_path + strlen("/board-resource/video_mp4"));  
            strcpy(machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].video_path, new_path);
        }
        LOG_I("tlp_path : %s\n", machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].tlp_path);
        LOG_I("video_path : %s\n", machine_info.print_history_record[(machine_info.print_history_current_index - 1) % PRINT_HISTORY_SIZE].video_path);
        machine_info_save();
#else
                if (ai_tlp.started == true)
                {
                    bool need_restart = false;
                    char restart_name[1023];
                    ai_tlp.started = false;
                    aic_unregister(tlp_camera_callback, NULL);
                    // 释放掉多余的 mjpeg 队列
                    while (hl_tpool_thread_recv_msg_try(thread, &request) == 0)
                        if (request.id == TLP_THREAD_EVENT_ID_MJPEG)
                            free(request.data);
                        else if (request.id == TLP_THREAD_EVENT_ID_START)
                        {
                            need_restart = true;
                            strncpy(restart_name, request.name, sizeof(restart_name));
                        }

                    // mp4 文件收尾
                    if (request.make_video)
                    {
                        char filename[64];
                        for (size_t i = 1; i < ai_tlp.layer + 1; i++)
                        {
                            sprintf(filename, "%stlp_layer_%d", ai_tlp.tlp_layer_path, i);
                            FILE *fp = fopen(filename, "r");
                            if (fp == NULL) 
                            {
                                LOG_E("无法打开文件 %s\n", filename);
                                continue; // 继续处理下一个文件
                            }

                            // 获取文件大小
                            fseek(fp, 0, SEEK_END);
                            long fileSize = ftell(fp);
                            rewind(fp);

                            // 分配内存
                            char *buffer = (char *)calloc(fileSize + 1, sizeof(char));
                            if (buffer == NULL) {
                                LOG_E("Memory allocation failed");
                                fclose(fp);
                                return EXIT_FAILURE;
                            }

                            // 读取文件内容
                            fread(buffer, 1, fileSize, fp);

                            int ret = tlp_mjpeg_to_yuyv422(buffer, fileSize, &yuyv, &yuyv_length);
                            tlp_yuyv422_to_mp4(yuyv, yuyv_length);
                            free(yuyv);

                            // 释放动态分配的内存
                            free(buffer);
                            fclose(fp);
                        }
                        // hl_system("rm -r '%s'", ai_tlp.tlp_layer_path);
                        int ret = 0;
                        avcodec_send_frame(ai_tlp.enc_ctx, NULL);
                        while ((ret = avcodec_receive_packet(ai_tlp.enc_ctx, ai_tlp.pkt)) == 0)
                        {
                            av_packet_rescale_ts(ai_tlp.pkt, ai_tlp.enc_ctx->time_base, ai_tlp.stream->time_base);
                            av_write_frame(ai_tlp.fmt_ctx, ai_tlp.pkt);
                            av_packet_unref(ai_tlp.pkt);
                        }

                        ret = av_write_frame(ai_tlp.fmt_ctx, NULL);
                        if ((ret = av_write_trailer(ai_tlp.fmt_ctx)) < 0)
                            LOG_I("av_write_trailer failed %s\n", av_err2str(ret));
                    }

                    {
                        // 关闭 mp4
                        tlp_close_mp4();
                        if (access(ai_tlp.path_tmp, F_OK) == 0 && request.make_video != 0)
                            rename(ai_tlp.path_tmp, ai_tlp.path_tag);
                        else
                            remove(ai_tlp.path_tmp);

                        for (int i = 0; i < machine_info.print_history_valid_numbers; i++)
                        {
                            printf("aic_tlp.path = %s \t hist : %s\n", ai_tlp.path_tag, machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_path);
                            if (strcmp(ai_tlp.path_tag, machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_path) == 0)
                            {
                                // find it
                                // printf("find it uuid = %s\n\n",machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].uuid);
                                printf("his index = %d\n", (machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE);
                                printf("file path = %s\n", machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].filepath);
                                if (access(ai_tlp.path_tag, F_OK) == 0)
                                {
                                    machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_state = 1;
                                    machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_time = request.capture_total_frame / TLP_FRAME_RATE;
                                }
                                else
                                {
                                    machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_state = 0;
                                    machine_info.print_history_record[(machine_info.print_history_current_index - i - 1) % PRINT_HISTORY_SIZE].tlp_time = 0;
                                }
                                break;
                            }
                        }
                        aic_tlp_check_mp4_handler();
                        machine_info_save();
                    }

                    if (need_restart == true)
                    {
                        tlp_thread_event_t ev;
                        ev.id = TLP_THREAD_EVENT_ID_START;
                        strncpy(ev.name, restart_name, sizeof(ev.name));
                        hl_tpool_send_msg(mp4_handle_thread, &ev);
                    }
                }
#endif
            }
        }
        // if (ai_tlp.started == false)
        usleep(10000);
    }
    LOG_I("mp4_handle_routine end\n");
}

// 回调函数，nftw 会为每个文件或目录调用此函数
static int count_callback(const char *fpath, const struct stat *sb, int typeflag, struct FTW *ftwbuf) {
    if (typeflag == FTW_F) { // 检查是否是普通文件
        ai_tlp.png_count++; // 普通文件计数加一
    }
    return 0; // 返回0以继续遍历
}

static void mp4_export_handle(hl_tpool_thread_t thread, void *args)
{
    tlp_thread_event_t request;
    printf("enter mp4_handle_routine\n");
    uint8_t *yuyv = NULL;
    uint32_t yuyv_length = 0;
    int ret;
    char video_name[1024];
    char file_name[1024];

    while (1)
    {
        if (hl_tpool_thread_recv_msg_try(thread, &request) == 0)
        {
            sprintf(video_name, "/user-resource/aic_tlp/%s.mp4", request.name);
            sprintf(file_name, "/user-resource/aic_tlp/%s", request.name);
            LOG_I("aic video_name = %s\n", video_name);
            ai_tlp.progress = 100;
            ai_tlp.png_count = 0;
            if(request.video_export_enable && access(video_name, F_OK) != 0)
            {
                ai_tlp.progress = 0;
                ret = aic_tlp_start(request.name);
                char filename[1024];
                int index = 1;
                int progress_index = 1;
                uint8_t failure_count = 0;

                // 使用nftw遍历目录，不设置同时打开文件描述符的最大数量
                int result = nftw(file_name, count_callback, 0, 0);

                if (result == -1) {
                    LOG_E("nftw");
                    break;
                }
                else
                {
                    LOG_I("%s file count = %d\n",file_name, ai_tlp.png_count);
                }

                while (1)
                {
                    sprintf(filename, "%s/tlp_layer_%d", file_name, index);
                    index++;
                    if(failure_count > 100)
                    {
                        //连续丢失100张图片自动退出
                        ai_tlp.png_count = 0;
                        ai_tlp.progress = 100;
                        break;
                    }
                    FILE *fp = fopen(filename, "r");
                    if (fp == NULL) 
                    {
                        failure_count++;
                        continue; // 继续处理下一个文件
                    }
                    failure_count = 0;
                    progress_index++;
                    ai_tlp.progress = progress_index *100.0f/(ai_tlp.png_count + 1);

                    if (!hl_disk_default_is_mounted(HL_DISK_TYPE_USB))
                    {
                        LOG_I("mp4_export usb not mounted\n");
                        snprintf(video_name, sizeof(video_name), "/user-resource/aic_tlp/%s.mp4.tmp",  request.name);
                        if (access(video_name, F_OK) == 0)
                        {
                            hl_system("rm '%s'", video_name);
                        }
                        break;
                    }

                    // 获取文件大小
                    fseek(fp, 0, SEEK_END);
                    long fileSize = ftell(fp);
                    rewind(fp);

                    // 分配内存
                    char *buffer = (char *)calloc(fileSize + 1, sizeof(char));
                    if (buffer == NULL) {
                        LOG_E("Memory allocation failed");
                        fclose(fp);
                        break;
                    }

                    // 读取文件内容
                    fread(buffer, 1, fileSize, fp);

                    int ret = tlp_mjpeg_to_yuyv422(buffer, fileSize, &yuyv, &yuyv_length);
                    tlp_yuyv422_to_mp4(yuyv, yuyv_length);
                    free(yuyv);

                    // 释放动态分配的内存
                    free(buffer);
                    fclose(fp);
                    usleep(100000);
                }

                // mp4 文件收尾
                int ret = 0;
                avcodec_send_frame(ai_tlp.enc_ctx, NULL);
                while ((ret = avcodec_receive_packet(ai_tlp.enc_ctx, ai_tlp.pkt)) == 0)
                {
                    av_packet_rescale_ts(ai_tlp.pkt, ai_tlp.enc_ctx->time_base, ai_tlp.stream->time_base);
                    av_write_frame(ai_tlp.fmt_ctx, ai_tlp.pkt);
                    av_packet_unref(ai_tlp.pkt);
                }
                ret = av_write_frame(ai_tlp.fmt_ctx, NULL);
                if ((ret = av_write_trailer(ai_tlp.fmt_ctx)) < 0)
                    LOG_I("av_write_trailer failed %s\n", av_err2str(ret));
                // 关闭 mp4
                tlp_close_mp4();

                LOG_D("ai_tlp.path_tmp = %s，ai_tlp.path_tag = %s\n", ai_tlp.path_tmp, ai_tlp.path_tag);
                if (access(ai_tlp.path_tmp, F_OK) == 0 && request.make_video != 0)
                    rename(ai_tlp.path_tmp, ai_tlp.path_tag);
                else
                     remove(ai_tlp.path_tmp);
            }
        }
        usleep(100000);
    }
    LOG_I("mp4_handle_routine end\n");
}

uint8_t get_aic_tlp_progress(void)
{
    return ai_tlp.progress;
}

static void tlp_close_mp4(void)
{
    av_packet_free(&ai_tlp.pkt);
    avcodec_close(ai_tlp.enc_ctx);
    avcodec_free_context(&ai_tlp.enc_ctx);
    avformat_free_context(ai_tlp.fmt_ctx);
    return;
}

static int tlp_create_mp4(const char *path, int cw, int ch)
{
    int ret;

    // 创建 mp4 前先检查一下空间是否已满
    // tlp_file_check();

    // 创建格式上下文
    if ((ret = avformat_alloc_output_context2(&ai_tlp.fmt_ctx, NULL, "mp4", path)) < 0)
    {
        LOG_I("avformat_alloc_output_context2 failed :%s\n", av_err2str(ret));
        return -1;
    }

    if ((ai_tlp.enc = avcodec_find_encoder(AV_CODEC_ID_H264)) == NULL)
    {
        LOG_I("avcodec_find_encoder failed");
        avformat_free_context(ai_tlp.fmt_ctx);
        return -1;
    }

    // 在MP4格式上创建视频流
    if ((ai_tlp.stream = avformat_new_stream(ai_tlp.fmt_ctx, ai_tlp.enc)) == NULL)
    {
        LOG_I("avformat_new_stream failed");
        avformat_free_context(ai_tlp.fmt_ctx);
        return -1;
    }

    if ((ai_tlp.enc_ctx = avcodec_alloc_context3(ai_tlp.enc)) == NULL)
    {
        LOG_I("avcodec_alloc_context3 failed");
        avformat_free_context(ai_tlp.fmt_ctx);
        return -1;
    }

    // 配置编码器参数
    ai_tlp.enc_ctx->codec_id = ai_tlp.enc->id;
    ai_tlp.enc_ctx->codec_type = AVMEDIA_TYPE_VIDEO;
    ai_tlp.enc_ctx->width = cw;
    ai_tlp.enc_ctx->height = ch;
    ai_tlp.enc_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    ai_tlp.enc_ctx->time_base = (AVRational){1, TLP_FRAME_RATE}; // FPS
    ai_tlp.enc_ctx->framerate = (AVRational){TLP_FRAME_RATE, 1};
    ai_tlp.enc_ctx->gop_size = 10;
    ai_tlp.enc_ctx->max_b_frames = 1;
    ai_tlp.enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    ai_tlp.enc_ctx->thread_count = 2;
    av_opt_set(ai_tlp.enc_ctx->priv_data, "preset", "ultrafast", 0);
    av_opt_set(ai_tlp.enc_ctx->priv_data, "tune", "zerolatency", 0);
    av_opt_set(ai_tlp.enc_ctx->priv_data, "profile", "main", 0);

    // 打开编码器
    if ((ret = avcodec_open2(ai_tlp.enc_ctx, ai_tlp.enc, NULL)) < 0)
    {
        LOG_I("avcodec_open2 failed :%s\n", av_err2str(ret));
        avcodec_free_context(&ai_tlp.enc_ctx);
        avformat_free_context(ai_tlp.fmt_ctx);
        return -1;
    }

    // 拷贝编码器参数到流参数
    if ((ret = avcodec_parameters_from_context(ai_tlp.stream->codecpar, ai_tlp.enc_ctx)) < 0)
    {
        LOG_I("avcodec_parameters_from_context failed :%s\n", av_err2str(ret));
        avcodec_close(ai_tlp.enc_ctx);
        avcodec_free_context(&ai_tlp.enc_ctx);
        avformat_free_context(ai_tlp.fmt_ctx);
        return -1;
    }

    if ((ret = avio_open(&ai_tlp.fmt_ctx->pb, path, AVIO_FLAG_WRITE)) < 0)
    {
        LOG_I("avio_open failed :%s\n", av_err2str(ret));
        avcodec_close(ai_tlp.enc_ctx);
        avcodec_free_context(&ai_tlp.enc_ctx);
        avformat_free_context(ai_tlp.fmt_ctx);
        return -1;
    }

    if ((ret = avformat_write_header(ai_tlp.fmt_ctx, NULL)) < 0)
    {
        LOG_I("avformat_write_header failed :%s\n", av_err2str(ret));
        avcodec_close(ai_tlp.enc_ctx);
        avcodec_free_context(&ai_tlp.enc_ctx);
        avformat_free_context(ai_tlp.fmt_ctx);
        return -1;
    }

    if ((ai_tlp.pkt = av_packet_alloc()) == NULL)
    {
        LOG_I("av_packet_alloc  failed\n");
        avcodec_close(ai_tlp.enc_ctx);
        avcodec_free_context(&ai_tlp.enc_ctx);
        avformat_free_context(ai_tlp.fmt_ctx);
        return -1;
    }

    ai_tlp.stream_index = 0; // 初始化 stream 变量

    return 0;
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
static int tlp_mpjeg_to_mp4(uint8_t *src, uint32_t src_len)
{
    // AVPacket *pkt;
    // AVCodecParserContext *parser_ctx;

    // Find video decoder
    AVCodec *decoder = avcodec_find_decoder(AV_CODEC_ID_MPEG4);
    if (!decoder)
    {
        fprintf(stderr, "Failed to find video encoder\n");
        return -1;
    }

    // 初始化解析器上下文
    // parser_ctx = av_parser_init(decoder->id);
    // if (parser_ctx == NULL)
    // {
    //     LOG_I("av_parser_init failed\n");
    //     return -1;
    // }

    // Create video codec context
    AVCodecContext *decoder_ctx = avcodec_alloc_context3(decoder);
    if (!decoder_ctx)
    {
        fprintf(stderr, "Failed to allocate video codec context\n");
        // av_parser_close(parser_ctx);
        return -1;
    }

    decoder_ctx->bit_rate = 400000;
    decoder_ctx->width = TLP_RESOLUTION_WIDTH;
    decoder_ctx->height = TLP_RESOLUTION_HEIGHT;
    decoder_ctx->time_base = (AVRational){1, TLP_FRAME_RATE};
    decoder_ctx->framerate = (AVRational){TLP_FRAME_RATE, 1};
    decoder_ctx->gop_size = 10;
    decoder_ctx->pix_fmt = AV_PIX_FMT_YUV420P;

    // Open video encoder
    if (avcodec_open2(decoder_ctx, decoder, NULL) < 0)
    {
        fprintf(stderr, "Failed to open video encoder\n");
        avcodec_free_context(&decoder_ctx);
        // av_parser_close(parser_ctx);
        return -1;
    }

    // Allocate frame and buffer
    AVFrame *frame = av_frame_alloc();
    if (!frame)
    {
        fprintf(stderr, "Failed to allocate frame\n");
        avcodec_close(decoder_ctx);
        avcodec_free_context(&decoder_ctx);
        // av_parser_close(parser_ctx);
        return -1;
    }
    frame->format = decoder_ctx->pix_fmt;
    frame->width = decoder_ctx->width;
    frame->height = decoder_ctx->height;
    int ret;
    ret = av_frame_get_buffer(frame, 0);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to allocate frame data\n");
        LOG_E("Failed to allocate frame data\n");
        // av_parser_close(parser_ctx);
    }

    // if ((pkt = av_packet_alloc()) == NULL)
    // {
    //     LOG_I("av_packet_alloc h264_decoder failed\n");
    //     av_parser_close(parser_ctx);
    //     avcodec_free_context(&decoder_ctx);
    //     av_parser_close(parser_ctx);
    //     av_frame_free(&frame);
    //     return -1;
    // }
    // Generate MJPEG data (replace with your own data source)
    // unsigned char *mjpegData = <your MJPEG data>;
    // int mjpegDataSize = <size of MJPEG data>;

    // Encode MJPEG frames and write to output file
    // int frameCount = 0;
    // AVPacket packet;
    // av_init_packet(&packet);
    // packet.data = NULL;
    // packet.size = 0;

    printf("src len = %d\n", src_len);
    // int got_frame = 0;
    // AVFrame *last_frame = NULL;

    // while( src_len > 0 )
    // {
    //     ret = av_parser_parse2(parser_ctx, decoder_ctx, &pkt->data, &pkt->size, src, src_len, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    //     if( ret < 0 )
    //     {
    //         LOG_I("av_parser_parse2 failed %s\n", av_err2str(ret));
    //         break;
    //     }

    //     src += ret;
    //     src_len -= ret;

    //     if(pkt->size)
    //         ret = avcodec_send_packet(decoder_ctx, pkt);

    //     if (ret < 0)
    //     {
    //         LOG_I("avcodec_send_packet failed %s\n", av_err2str(ret));
    //         break;
    //     }

    //     while( (ret = avcodec_receive_frame(decoder_ctx, frame)) == 0 )
    //     {
    //         // if (frame->pict_type == AV_PICTURE_TYPE_I || frame->pict_type == AV_PICTURE_TYPE_P || frame->pict_type == AV_PICTURE_TYPE_B)
    //         LOG_D("frame pict type : %d\n" , frame->pict_type );
    //         if( frame->pict_type == AV_PICTURE_TYPE_S )
    //         {
    //             got_frame++;
    //             av_frame_free(&last_frame);
    //             last_frame = av_frame_clone(frame);
    //         }
    //         av_frame_unref(frame);
    //     }
    //     av_packet_unref(pkt);
    // }
    // if( got_frame )
    // {
    //     uint64_t ticks = hl_get_tick_ms();
    //     last_frame->pts = ai_tlp.stream_index;
    //     avcodec_send_frame(ai_tlp.enc_ctx, last_frame);

    //     while ((ret = avcodec_receive_packet(ai_tlp.enc_ctx, ai_tlp.pkt)) == 0)
    //     {
    //         av_packet_rescale_ts(ai_tlp.pkt, ai_tlp.enc_ctx->time_base, ai_tlp.stream->time_base);
    //         av_write_frame(ai_tlp.fmt_ctx, ai_tlp.pkt);
    //         av_packet_unref(ai_tlp.pkt);
    //     }
    //     printf("flush encoder\n");
    //     av_write_frame(ai_tlp.fmt_ctx, NULL);
    //     av_frame_free(&last_frame);
    //     ai_tlp.stream_index++;
    // }

    memcpy(frame->data[0], src, src_len);
    frame->pts = ai_tlp.stream_index;

    // Encode frame
    ret = avcodec_send_frame(ai_tlp.enc_ctx, frame);
    LOG_D("ret = %d\n", ret);

    if (ret < 0)
    {
        fprintf(stderr, "Failed to send frame for encoding\n");
        av_frame_free(&frame);
        avcodec_free_context(&decoder_ctx);
        if (ret == AVERROR_EOF)
            LOG_D("AVERROR_EOF\n");
        else if (ret == AVERROR(EAGAIN))
            LOG_D("AVERROR(EAGAIN)\n");
        else if (ret == AVERROR(EINVAL))
            LOG_D("AVERROR(EINVAL)\n");
        else if (ret == AVERROR(ENOMEM))
            LOG_D("AVERROR(ENOMEM)\n");
        else
            LOG_D("other error\n");
        return -1;
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_packet(ai_tlp.enc_ctx, ai_tlp.pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            break;
        else if (ret < 0)
        {
            fprintf(stderr, "Failed to receive packet from encoder\n");
            break;
        }

        // Write packet to output file
        av_write_frame(ai_tlp.fmt_ctx, ai_tlp.pkt);
        av_packet_unref(ai_tlp.pkt);
    }
    printf("flush encoder\n");

    // Flush encoder
    av_write_frame(ai_tlp.fmt_ctx, NULL);

    ai_tlp.stream_index++;

    // Free resources
    // avcodec_close(decoder_ctx);
    // av_packet_free(&pkt);
    av_frame_free(&frame);
    // av_parser_close(parser_ctx);
    avcodec_free_context(&decoder_ctx);
    printf("finish\n");

    return 0;
}

static int tlp_mjpeg_to_h264(uint8_t *i_data, uint32_t i_size, uint8_t *o_data, uint32_t o_size)
{
    const AVCodec *codec;
    AVCodecContext *codec_ctx = NULL;
    AVFrame *frame = NULL;
    AVPacket packet;
    int ret;

    codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec)
    {
        fprintf(stderr, "无法找到H.264编码器\n");
        return -1;
    }

    // 创建编码器上下文
    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx)
    {
        fprintf(stderr, "无法分配编码器上下文\n");
        return -1;
    }

    // 设置编码器参数
    codec_ctx->width = TLP_RESOLUTION_WIDTH;                // 设置为输入图像的宽度
    codec_ctx->height = TLP_RESOLUTION_HEIGHT;              // 设置为输入图像的高度
    codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;                // 设置像素格式
    codec_ctx->time_base = (AVRational){1, TLP_FRAME_RATE}; // 设置时间基准

    // 打开编码器
    if (avcodec_open2(codec_ctx, codec, NULL) < 0)
    {
        fprintf(stderr, "无法打开编码器\n");
        avcodec_free_context(&codec_ctx);
        return -1;
    }

    // 分配帧内存
    frame = av_frame_alloc();
    if (!frame)
    {
        fprintf(stderr, "无法分配帧内存\n");
        avcodec_close(codec_ctx);
        avcodec_free_context(&codec_ctx);
        return -1;
    }

    // 设置帧参数
    frame->format = codec_ctx->pix_fmt;
    frame->width = codec_ctx->width;
    frame->height = codec_ctx->height;

    // 分配帧数据内存
    ret = av_frame_get_buffer(frame, 0);
    if (ret < 0)
    {
        fprintf(stderr, "无法分配帧数据内存\n");
        av_frame_free(&frame);
        avcodec_close(codec_ctx);
        avcodec_free_context(&codec_ctx);
        return -1;
    }

    // 初始化输出数据数组索引
    int output_index = 0;
    memcpy(frame->data[0], i_data, i_size);

    // 编码H.264帧
    ret = avcodec_send_frame(codec_ctx, frame);
    if (ret < 0)
    {
        fprintf(stderr, "无法编码H.264帧\n");
        // break;
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_packet(codec_ctx, &packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            break;
        else if (ret < 0)
        {
            fprintf(stderr, "无法接收H.264数据包\n");
            break;
        }

        // 将H.264数据包存储到output_data数组中
        if (output_index + packet.size <= o_size)
        {
            memcpy(o_data + output_index, packet.data, packet.size);
            output_index += packet.size;
        }

        av_packet_unref(&packet);
    }

    // 刷新编码器
    ret = avcodec_send_frame(codec_ctx, NULL);
    if (ret < 0)
    {
        fprintf(stderr, "无法刷新编码器\n");
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_packet(codec_ctx, &packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            break;
        else if (ret < 0)
        {
            fprintf(stderr, "无法接收H.264数据包\n");
            break;
        }

        // 将剩余的H.264数据包存储到output_data数组中
        if (output_index + packet.size <= o_size)
        {
            memcpy(o_data + output_index, packet.data, packet.size);
            output_index += packet.size;
        }

        av_packet_unref(&packet);
    }

    // 释放资源
    av_frame_free(&frame);
    avcodec_close(codec_ctx);
    avcodec_free_context(&codec_ctx);
    return output_index;
}

static int tlp_h264_to_mp4(uint8_t *src, uint32_t src_len)
{
    AVCodec *decoder;
    AVCodecContext *decoder_ctx;
    AVCodecParserContext *parser_ctx;
    AVPacket *pkt;
    AVFrame *frame;
    // LOG_I("tlp_h264_to_mp4 size %d\n", src_len);

    // 寻找解码器
    decoder = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (decoder == NULL)
    {
        LOG_I("avcodec_find_decoder failed\n");
        return -1;
    }

    // 初始化解析器上下文
    parser_ctx = av_parser_init(decoder->id);
    if (parser_ctx == NULL)
    {
        LOG_I("av_parser_init failed\n");
        return -1;
    }

    // 分配解码器上下文
    decoder_ctx = avcodec_alloc_context3(decoder);
    if (decoder_ctx == NULL)
    {
        av_parser_close(parser_ctx);
        LOG_I("avcodec_alloc_context3 failed\n");
        return -1;
    }

    if ((pkt = av_packet_alloc()) == NULL)
    {
        LOG_I("av_packet_alloc h264_decoder failed\n");
        av_parser_close(parser_ctx);
        avcodec_free_context(&decoder_ctx);
        return -1;
    }

    if ((frame = av_frame_alloc()) == NULL)
    {
        LOG_I("av_frame_alloc h264_decoder failed\n");
        av_packet_free(&pkt);
        av_parser_close(parser_ctx);
        avcodec_free_context(&decoder_ctx);
        return -1;
    }

    // 打开h264解码器
    if (avcodec_open2(decoder_ctx, decoder, NULL) != 0)
    {
        LOG_I("avcodec_open2 h264_decoder failed\n");
        av_frame_free(&frame);
        av_packet_free(&pkt);
        av_parser_close(parser_ctx);
        avcodec_free_context(&decoder_ctx);
        return -1;
    }

    int ret = 0;
    int got_frame = 0;
    AVFrame *last_frame = NULL;

    // 查找最后一帧
    uint64_t ticks = hl_get_tick_ms();
    while (src_len > 0)
    {
        ret = av_parser_parse2(parser_ctx, decoder_ctx, &pkt->data, &pkt->size, src, src_len, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
        if (ret < 0)
        {
            LOG_I("av_parser_parse2 failed %s\n", av_err2str(ret));
            break;
        }
        src += ret;
        src_len -= ret;

        if (pkt->size)
            ret = avcodec_send_packet(decoder_ctx, pkt);

        if (ret < 0)
        {
            LOG_I("avcodec_send_packet failed %s\n", av_err2str(ret));
            break;
        }
        else
        {
            LOG_I("avcodec_send_packet success\n");
        }

        while ((ret = avcodec_receive_frame(decoder_ctx, frame)) == 0)
        {
            LOG_I("pict type : %d\n", frame->pict_type);
            if (frame->pict_type == AV_PICTURE_TYPE_I || frame->pict_type == AV_PICTURE_TYPE_P || frame->pict_type == AV_PICTURE_TYPE_B)
            {
                got_frame++;
                av_frame_free(&last_frame);
                last_frame = av_frame_clone(frame);
            }
            av_frame_unref(frame);
        }
        LOG_I("avcodec_receive_frame ret = %d\n", ret);
        if (ret == AVERROR_EOF)
            LOG_D("AVERROR_EOF\n");
        else if (ret == AVERROR(EAGAIN))
            LOG_D("AVERROR(EAGAIN)\n");
        else if (ret == AVERROR(EINVAL))
            LOG_D("AVERROR(EINVAL)\n");
        else if (ret == AVERROR(ENOMEM))
            LOG_D("AVERROR(ENOMEM)\n");
        else
            LOG_D("other error\n");
        av_packet_unref(pkt);
    }

    LOG_I("avcodec_receive_frame frame count %d spend %llu\n", got_frame, hl_get_tick_ms() - ticks);

    if (got_frame)
    {
        uint64_t ticks = hl_get_tick_ms();
        last_frame->pts = ai_tlp.stream_index;
        avcodec_send_frame(ai_tlp.enc_ctx, last_frame);

        while ((ret = avcodec_receive_packet(ai_tlp.enc_ctx, ai_tlp.pkt)) == 0)
        {
            av_packet_rescale_ts(ai_tlp.pkt, ai_tlp.enc_ctx->time_base, ai_tlp.stream->time_base);
            av_write_frame(ai_tlp.fmt_ctx, ai_tlp.pkt);
            av_packet_unref(ai_tlp.pkt);
        }
        av_write_frame(ai_tlp.fmt_ctx, NULL);
        av_frame_free(&last_frame);
        // LOG_I("write frame done %d %llu\n", tlp.stream_index, hl_get_tick_ms() - ticks);
        ai_tlp.stream_index++;
    }

    av_frame_free(&frame);
    av_packet_free(&pkt);
    av_parser_close(parser_ctx);
    avcodec_free_context(&decoder_ctx);
    return 0;
}

static int tlp_mjpeg_to_yuyv422(uint8_t *i_data, uint32_t i_size, uint8_t **o_data, uint32_t *o_size)
{
    AVCodecContext *codec_ctx = NULL;
    AVCodec *codec = NULL;
    AVFrame *frame = NULL;
    AVPacket packet;
    int ret;

    uint8_t *output_data; // 转换后的数据将存储在这个数组中
    int output_size;      // 输出数据的大小

    codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if (!codec)
    {
        fprintf(stderr, "无法找到MJPEG解码器\n");
        return -1;
    }

    // 创建解码器上下文
    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx)
    {
        fprintf(stderr, "无法分配解码器上下文\n");
        return -1;
    }

    // 设置解码器参数
    codec_ctx->pix_fmt = AV_PIX_FMT_YUV422P;

    // 打开解码器
    if (avcodec_open2(codec_ctx, codec, NULL) < 0)
    {
        fprintf(stderr, "无法打开解码器\n");
        avcodec_free_context(&codec_ctx);
        return -1;
    }

    // 创建帧
    frame = av_frame_alloc();
    if (!frame)
    {
        fprintf(stderr, "无法分配帧\n");
        avcodec_close(codec_ctx);
        avcodec_free_context(&codec_ctx);
        return -1;
    }

    // 分配输出数据内存
    codec_ctx->width = TLP_RESOLUTION_WIDTH;
    codec_ctx->height = TLP_RESOLUTION_HEIGHT;

    output_size = codec_ctx->width * codec_ctx->height * 2; // YUYV422格式每个像素占2字节
    output_data = (uint8_t *)malloc(output_size);
    if (!output_data)
    {
        fprintf(stderr, "无法分配输出数据内存\n");
        av_frame_free(&frame);
        avcodec_close(codec_ctx);
        avcodec_free_context(&codec_ctx);
        return -1;
    }

    // 设置输入数据
    AVPacket input_packet;
    av_init_packet(&input_packet);
    input_packet.data = i_data;
    input_packet.size = i_size;

    // 解码输入数据
    ret = avcodec_send_packet(codec_ctx, &input_packet);
    if (ret < 0)
    {
        fprintf(stderr, "无法发送数据包给解码器\n");
        free(output_data);
        av_frame_free(&frame);
        avcodec_close(codec_ctx);
        avcodec_free_context(&codec_ctx);
        return -1;
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_frame(codec_ctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            break;
        else if (ret < 0)
        {
            fprintf(stderr, "无法接收解码器帧\n");
            free(output_data);
            av_frame_free(&frame);
            avcodec_close(codec_ctx);
            avcodec_free_context(&codec_ctx);
            return -1;
        }

        // 转换为YUYV422格式
        struct SwsContext *sws_ctx = sws_getContext(codec_ctx->width, codec_ctx->height, codec_ctx->pix_fmt,
                                                    codec_ctx->width, codec_ctx->height, AV_PIX_FMT_YUYV422,
                                                    0, NULL, NULL, NULL);
        if (!sws_ctx)
        {
            fprintf(stderr, "无法创建图像转换上下文\n");
            free(output_data);
            av_frame_free(&frame);
            avcodec_close(codec_ctx);
            avcodec_free_context(&codec_ctx);
            return -1;
        }

        uint8_t *output_planes[3] = {output_data, NULL, NULL};
        int output_strides[3] = {codec_ctx->width * 2, 0, 0};
        sws_scale(sws_ctx, (const uint8_t *const *)frame->data, frame->linesize, 0, codec_ctx->height, output_planes, output_strides);

        sws_freeContext(sws_ctx);
    }

    *o_size = output_size;
    *o_data = output_data;

    // 清理资源
    // free(output_data);
    av_frame_free(&frame);
    avcodec_close(codec_ctx);
    avcodec_free_context(&codec_ctx);

    return 0;
}

static int tlp_yuyv422_to_mp4(uint8_t *src, uint32_t src_len)
{
    int ret = 0;
    AVFrame *yuyv422_frame = NULL;
    AVFrame *yuv420p_frame = NULL;
    struct SwsContext *sws_ctx;

    uint8_t *yuv420p;
    uint32_t yuv420p_size = av_image_get_buffer_size(AV_PIX_FMT_YUV420P, ai_tlp.enc_ctx->width, ai_tlp.enc_ctx->height, 1);
    yuv420p = (uint8_t *)av_malloc(yuv420p_size);

    if (yuv420p == NULL)
    {
        LOG_I("av_malloc failed\n");
        return -1;
    }

    yuyv422_frame = av_frame_alloc();
    yuv420p_frame = av_frame_alloc();
    if (yuyv422_frame == NULL || yuv420p_frame == NULL)
    {
        av_free(yuv420p);
        if (yuyv422_frame)
            av_frame_free(&yuyv422_frame);
        if (yuv420p_frame)
            av_frame_free(&yuv420p_frame);
        LOG_I("av_frame_alloc failed\n");
    }

    sws_ctx = sws_getContext(ai_tlp.enc_ctx->width, ai_tlp.enc_ctx->height, AV_PIX_FMT_YUYV422,
                             ai_tlp.enc_ctx->width, ai_tlp.enc_ctx->height, AV_PIX_FMT_YUV420P,
                             SWS_BICUBIC, NULL, NULL, NULL);
    if (sws_ctx == NULL)
    {
        av_free(yuv420p);
        av_frame_free(&yuyv422_frame);
        av_frame_free(&yuv420p_frame);
        LOG_I("sws_getContext failed\n");
        return -1;
    }

    // YUYV422 TO YUV420P
    av_image_fill_arrays(yuyv422_frame->data, yuyv422_frame->linesize, src, AV_PIX_FMT_YUYV422, ai_tlp.enc_ctx->width, ai_tlp.enc_ctx->height, 1);
    av_image_fill_arrays(yuv420p_frame->data, yuv420p_frame->linesize, yuv420p, AV_PIX_FMT_YUV420P, ai_tlp.enc_ctx->width, ai_tlp.enc_ctx->height, 1);
    sws_scale(sws_ctx, (const uint8_t **)yuyv422_frame->data, (const int *)yuyv422_frame->linesize, 0, ai_tlp.enc_ctx->height,
              yuv420p_frame->data, yuv420p_frame->linesize);

    // tlp_save_gop(ai_tlp.stream_index, yuv420p, yuv420p_size);

    yuv420p_frame->pts = ai_tlp.stream_index;
    yuv420p_frame->width = ai_tlp.enc_ctx->width;
    yuv420p_frame->height = ai_tlp.enc_ctx->height;
    yuv420p_frame->format = ai_tlp.enc_ctx->pix_fmt;

    avcodec_send_frame(ai_tlp.enc_ctx, yuv420p_frame);
    while ((ret = avcodec_receive_packet(ai_tlp.enc_ctx, ai_tlp.pkt)) == 0)
    {
        av_packet_rescale_ts(ai_tlp.pkt, ai_tlp.enc_ctx->time_base, ai_tlp.stream->time_base);
        av_write_frame(ai_tlp.fmt_ctx, ai_tlp.pkt);
        av_packet_unref(ai_tlp.pkt);
    }
    av_write_frame(ai_tlp.fmt_ctx, NULL);

    sws_freeContext(sws_ctx);
    av_free(yuv420p);
    av_frame_free(&yuyv422_frame);
    av_frame_free(&yuv420p_frame);
    ai_tlp.stream_index++;
}

#endif
