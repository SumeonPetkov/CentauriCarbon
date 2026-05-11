// #include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>      // struct timespec
#include <asm/unistd.h>    //flush_dcache_all
#include <ion_mem_alloc.h> //GetMemAdapterOpsS
#include "SharespaceInit.h"
#include "msgbox.h"
#include "debug.h"
#define SHARE_SPACE_HEAD_END 1

#define flush_dcache_all() //

#if SHARE_SPACE_HEAD_END
#define SHARE_SPACE_HEAD_OFFSET (4096 - sizeof(struct msg_head_t))
#else
#define SHARE_SPACE_HEAD_OFFSET 0
#endif

// static
uint8_t *pu8ArmBuf = NULL;
uint8_t *pu8DspBuf = NULL;
static char *pVirArmBuf = NULL;
static char *pVirDspBuf = NULL;
void (*mem_sync)(void *, uint32_t);

int fd_sharespace = -1;
uint16_t sharespace_arm_addr[2];
uint16_t sharespace_dsp_addr[2];
struct dsp_sharespace_t sharespace_addr;
struct msg_head_t dsp_head;
struct msg_head_t arm_head;

int GAM_DEBUG_printf_time()
{
    struct timeval tNow;
    gettimeofday(&tNow, NULL); // get_monotonic
    printf("[%d %d]:", tNow.tv_sec, tNow.tv_usec);
    return 0;
}

size_t virtual_to_physical(size_t addr)
{
    int fd = open("/proc/self/pagemap", O_RDONLY);
    if (fd < 0)
    {
        printf("open '/proc/self/pagemap' failed!\n");
        return 0;
    }
    size_t pagesize = getpagesize();
    size_t offset = (addr / pagesize) * sizeof(uint64_t);
    if (lseek(fd, offset, SEEK_SET) < 0)
    {
        printf("lseek() failed!\n");
        close(fd);
        return 0;
    }
    uint64_t info;
    if (read(fd, &info, sizeof(uint64_t)) != sizeof(uint64_t))
    {
        printf("read() failed!\n");
        close(fd);
        return 0;
    }
    if ((info & (((uint64_t)1) << 63)) == 0)
    {
        printf("page is not present!\n");
        close(fd);
        return 0;
    }
    size_t frame = info & ((((uint64_t)1) << 55) - 1);
    size_t phy = frame * pagesize + addr % pagesize;
    close(fd);
    return phy;
}

int dev_mem_CS(uint32_t addr)
{
    int fd;
    char *rdbuf = (char *)malloc(256);
    char *wrbuf = "butterfly";
    int i;
    fd = open("/dev/mem", O_RDWR);
    if (fd < 0)
    {
        printf("open /dev/mem failed.\n");
        return -1;
    }
    printf("open /dev/mem lseek %x %x %x.\n", addr, rdbuf, wrbuf);
    // addr = 0;
    lseek(fd, addr, 0);
    read(fd, rdbuf, 10);

    for (i = 0; i < 10; i++)
    {
        printf("old mem[%d]:%c\n", i, *(rdbuf + i));
    }
    lseek(fd, addr, 0);
    write(fd, wrbuf, 10);
    lseek(fd, addr, 0); // move f_ops to the front
    read(fd, rdbuf, 10);
    for (i = 0; i < 10; i++)
    {
        printf("new mem[%d]:%c\n", i, *(rdbuf + i));
    }
    free(rdbuf);
    return 0;
}

void GAM_DEBUG_printf_sendbuf_HEX()
{
    char *readaddr = pVirArmBuf;
    printf("--write_buf-%d--%d---------\n", arm_head.write_addr, msgbox_send_msg[1]);
    int i = 0;
    for (i = 0; i < 4096; i++)
    {
        printf("%x ", readaddr[i]);
        if (readaddr[i] == 0x7e)
        {
            printf("\n%d: ", i + 1);
        }
        if (i == arm_head.read_addr)
        {
            printf("--write_addr-%d----------\n", i);
        }
    }
    printf("\n");
}
void GAM_DEBUG_printf_receivebuf_HEX()
{
    char *readaddr = pVirDspBuf;
    printf("--read_buf-%d--%d---------\n", msgbox_new_msg[1], arm_head.read_addr);
    int i = 0;
    for (i = 0; i < 4096; i++)
    {
        printf("%x ", readaddr[i]);
        if (readaddr[i] == 0x7e)
        {
            printf("\n%d: ", i + 1);
        }
        if (i == arm_head.read_addr)
        {
            printf("--read_addr-%d----------\n", i + 1);
        }
    }
    printf("\n");
}

void GAM_printf_sendMSG(uint8_t *buf, uint32_t len, uint8_t msg_id);
void GAM_DEBUG_printf_HEX(uint8_t write, uint8_t *buf, uint32_t buf_len)
{ //-----gg-2-----------
    int m = 0;
    // GAM_DEBUG_printf_time();
    // if (write)
    // {
        //    printf("ARM_DSP:%4d=%4d:-%4d:%4d:%4d   %4d:  ", buf_len, arm_head.write_addr,sharespace_arm_addr[SHARESPACE_WRITE], dsp_head.read_addr,sharespace_dsp_addr[SHARESPACE_READ],   arm_head.init_state);
        //   printf("111:%d=%d %d:", buf_len, arm_head.write_addr,  dsp_head.read_addr );
        // printf("111:%d:", buf_len);
    // }
    // else
    // {
        // printf("DSP_ARM:%4d=%4d::%4d-%4d:%4d  %4d:  ", buf_len,dsp_head.write_addr,sharespace_dsp_addr[SHARESPACE_WRITE], arm_head.read_addr,sharespace_arm_addr[SHARESPACE_READ], arm_head.init_state);
        // printf("000:%3d=%3d %3d:", buf_len, dsp_head.write_addr, arm_head.read_addr);
    // }
    GAM_printf_sendMSG(buf, buf_len, buf[2]);
    // GAM_DEBUG_printf_sendbuf_HEX();
    // GAM_DEBUG_printf_receivebuf_HEX();
}

void GAM_DEBUG_printf_HEX1(char *msg, uint8_t *buf, uint_fast8_t buf_len)
{ //-----gg-2-----------
    int m = 0;
    GAM_DEBUG_printf_time();
    printf("%s %d:", msg, buf_len);
    for (; m < buf_len; m++)
    {
        printf(" %x", buf[m]);
        // if (m%16 == 0 && m != 0)
        //     printf("\n");
    }
    printf("\n  DSP<-ARM: %d-%d  ARM<-DSP:%d:%d-%d\n", sharespace_dsp_addr[SHARESPACE_READ], sharespace_arm_addr[SHARESPACE_WRITE], arm_head.read_addr, sharespace_arm_addr[SHARESPACE_READ], sharespace_dsp_addr[SHARESPACE_WRITE]);
}

int choose_sharespace(int fd, struct dsp_sharespace_t *msg, uint32_t choose)
{
    int ret = 0;

    /* get debug msg value */
    ret = ioctl(fd, CMD_READ_DEBUG_MSG, (unsigned long)msg);
    if (ret < 0)
        return ret;
    switch (choose)
    {
    case CHOOSE_DSP_WRITE_SPACE:
        msg->mmap_phy_addr = msg->dsp_write_addr;
        break;
    case CHOOSE_ARM_WRITE_SPACE:
        msg->mmap_phy_addr = msg->arm_write_addr;
        break;
    case CHOOSE_DSP_LOG_SPACE:
        msg->mmap_phy_addr = msg->dsp_log_addr;
        break;
    }
    /* update debug msg */
    ret = ioctl(fd, CMD_WRITE_DEBUG_MSG, (unsigned long)msg);
    return ret;
}

int sharespace_open()
{
    int ret = 0;
    char *dev_path = "/dev/dsp_debug";
    // O_DIRECT: 无缓冲的输入、输出。
    // O_SYNC：以同步IO方式打开文件。
    // O_NONBLOCK: 把文件的打开和后继 I/O设置为非阻塞模式--
    // create mmap file
    if (fd_sharespace > 0)
    {
        return fd_sharespace;
    }
    fd_sharespace = open(dev_path, O_RDWR | O_SYNC | O_NONBLOCK);
    if (fd_sharespace <= 0)
    {
        printf("open sharespace device is error, fd = %d", fd_sharespace);
        fd_sharespace = -1;
        return -1;
    }
    return fd_sharespace;
}

int sharespace_mmap()
{
    int ret = 0;
    int fd = sharespace_open();
    if (fd < 0)
    {
        GAM_ERR_printf("sharespace_open errer  \n");
        return -1;
    }
    ret = choose_sharespace(fd, &sharespace_addr, CHOOSE_ARM_WRITE_SPACE);
    if (ret < 0)
        return ret;
    pu8ArmBuf = (char *)mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (pu8ArmBuf < 0)
    {
        GAM_ERR_printf("dev mmap to fail\n");
        ret = -1;
        return -1;
    }
    ret = choose_sharespace(fd, &sharespace_addr, CHOOSE_DSP_WRITE_SPACE);
    if (ret < 0)
        return ret;
    pu8DspBuf = (char *)mmap(NULL, 4096, PROT_READ, MAP_SHARED, fd, 0);
    if (pu8DspBuf < 0)
    {
        GAM_ERR_printf("dev mmap to fail\n");
        ret = -1;
        return -1;
    }
    return fd;
}
int sharespace_isinit()
{
    memset(&arm_head, 0, sizeof(struct msg_head_t));
    memcpy((void *)&arm_head, (void *)(pu8ArmBuf + SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
    if ((arm_head.init_state == 1) || (arm_head.init_state == 2))
    {
        return 1;
    }
    return 0;
}
int set_dsp_ops_addr_no_mmap(uint32_t arm_write_addr, uint32_t dsp_write_addr)
{
    if (sharespace_isinit())
    {
        arm_head.init_state = 2;
    }
    else
    {
        arm_head.init_state = 1;
    }
    arm_head.read_addr = dsp_write_addr; // + sizeof(struct debug_msg_t);
    arm_head.write_addr = arm_write_addr;

    memcpy((void *)(pu8ArmBuf + SHARE_SPACE_HEAD_OFFSET), (void *)&arm_head, sizeof(struct msg_head_t));
    return 0;
}
int set_dsp_ops_addr(uint32_t arm_write_addr, uint32_t dsp_write_addr)
{
    int fd = sharespace_mmap();
    if (fd < 0)
    {
        GAM_ERR_printf("sharespace_mmap errer  \n");
        return -1;
    }
    set_dsp_ops_addr_no_mmap(arm_write_addr, dsp_write_addr);
    return fd;
}

int set_arm_ops_addr(char *arm_write_addr, char *dsp_write_addr)
{
    pVirArmBuf = arm_write_addr;
    pVirDspBuf = dsp_write_addr;

    arm_head.read_addr = sizeof(struct msg_head_t); // + sizeof(struct debug_msg_t);
    arm_head.write_addr = sizeof(struct msg_head_t);
    arm_head.init_state = 1;
    // GAM_DEBUG_printf("arm_write_addr %x %x\n", pVirArmBuf, SHARE_SPACE_HEAD_OFFSET);
    memcpy((void *)(pVirArmBuf + SHARE_SPACE_HEAD_OFFSET), (void *)&arm_head, sizeof(struct msg_head_t));
    sharespace_arm_addr[SHARESPACE_WRITE] = arm_head.write_addr;
    sharespace_arm_addr[SHARESPACE_READ] = arm_head.read_addr;
    msgbox_send_msg[SHARESPACE_WRITE] = arm_head.write_addr;
    msgbox_send_msg[SHARESPACE_READ] = arm_head.read_addr;
}

void set_arm_mem_sync(void *fun_sync)
{
    mem_sync = fun_sync;
}

int dsp_set_reinit() //
{
    arm_head.read_addr = sizeof(struct msg_head_t); // + sizeof(struct debug_msg_t);
    arm_head.write_addr = sizeof(struct msg_head_t);
    arm_head.init_state = 1;
    msync(pVirDspBuf, 4096, MS_INVALIDATE);
    memcpy((void *)(pVirArmBuf + SHARE_SPACE_HEAD_OFFSET), (void *)&arm_head, sizeof(struct msg_head_t));
    memcpy((void *)&dsp_head, (void *)(pVirDspBuf + SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
    sharespace_dsp_addr[SHARESPACE_WRITE] = dsp_head.write_addr;
    sharespace_dsp_addr[SHARESPACE_READ] = dsp_head.read_addr;
}

int wait_dsp_set_init() //
{
    arm_head.read_addr = sizeof(struct msg_head_t); // + sizeof(struct debug_msg_t);
    arm_head.write_addr = sizeof(struct msg_head_t);
    arm_head.init_state = 1;

    while (1)
    {
        msync(pVirDspBuf, 4096, MS_INVALIDATE);
        memcpy((void *)&dsp_head, (void *)(pVirDspBuf + SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
        memcpy((void *)(pVirArmBuf + SHARE_SPACE_HEAD_OFFSET), (void *)&arm_head, sizeof(struct msg_head_t));
        // GAM_DEBUG_printf("dsp_write_addr %x %x\n", pVirDspBuf, SHARE_SPACE_HEAD_OFFSET);
        // GAM_DEBUG_printf("dsp_head.init_state %x\n", dsp_head.init_state);
        // GAM_DEBUG_printf("dsp_head.read_addr %x\n", dsp_head.read_addr);
        // GAM_DEBUG_printf("dsp_head.write_addr %x\n", dsp_head.write_addr);

        if (dsp_head.init_state == 1)
        {
            sharespace_dsp_addr[SHARESPACE_WRITE] = dsp_head.write_addr;
            sharespace_dsp_addr[SHARESPACE_READ] = dsp_head.read_addr;
            break;
        }
        usleep(10000);
    }
}

int DSP_mem_write(uint8_t *cmd, int len) //
{
    int ret = 0;
    uint32_t free_size = 0;
#if SHARE_SPACE_HEAD_END
    uint32_t min_addr = sizeof(struct msg_head_t);        // msg_head_addr
    uint32_t max_addr = 4096 - sizeof(struct msg_head_t); // msg_end_addr  msg.arm_write_size;
#else
    uint32_t min_addr = sizeof(struct msg_head_t);
    uint32_t max_addr = 4096; // msg_end_addr  msg.arm_write_size;
#endif

    if ((len > 4000) || (len < 0))
    {
        return arm_head.write_addr;
    }
    while (1)
    {
        //-----G-G-2022-07-27----
        dsp_head.read_addr = msgbox_new_msg[0];
        if (dsp_head.read_addr <= arm_head.write_addr)
        {
            free_size = max_addr - min_addr - (arm_head.write_addr - dsp_head.read_addr);
            if (free_size > len)
            {
                break;
            }
            else
            {
                GAM_ERR_printf("len = %d free_size = %d dsp_head.read_addr = %d arm_head.write_addr = %d   \n", len, free_size, dsp_head.read_addr, arm_head.write_addr);
            }
        }
        else if (dsp_head.read_addr > arm_head.write_addr)
        {
            free_size = dsp_head.read_addr - arm_head.write_addr;
            if (free_size > len)
            {
                break;
            }
            else
            {
                GAM_ERR_printf("len = %d free_size = %d dsp_head.read_addr = %d arm_head.write_addr = %d   \n", len, free_size, dsp_head.read_addr, arm_head.write_addr);
            }
        }
    }
    uint32_t pmsg = arm_head.write_addr;
    //------G-G-2022-07-27----
    if (arm_head.write_addr + len <= max_addr)
    {
        memcpy(pVirArmBuf + arm_head.write_addr, cmd, (unsigned int)(len));
        mem_sync(pVirArmBuf + arm_head.write_addr, (unsigned int)(len));
        pmsg += len;
        if (pmsg >= max_addr)
        {
            pmsg = min_addr;
        }
    }
    else
    {
        int len1 = max_addr - arm_head.write_addr;
        memcpy(pVirArmBuf + arm_head.write_addr, cmd, (unsigned int)(len1));
        mem_sync(pVirArmBuf + arm_head.write_addr, (unsigned int)(len1));
        len -= len1;
        memcpy(pVirArmBuf + min_addr, cmd + len1, (unsigned int)(len));
        mem_sync(pVirArmBuf + min_addr, (unsigned int)(len));
        pmsg = min_addr + len;
    }
    //------G-G-2022-07-27----
    arm_head.write_addr = pmsg;
    arm_head.init_state = 1;
    memcpy(pVirArmBuf + SHARE_SPACE_HEAD_OFFSET, &arm_head, (unsigned int)(sizeof(struct msg_head_t)));
    mem_sync(pVirArmBuf + SHARE_SPACE_HEAD_OFFSET, (unsigned int)(sizeof(struct msg_head_t)));
    //------G-G-2022-07-27----
    sharespace_arm_addr[SHARESPACE_WRITE] = arm_head.write_addr;
    sharespace_dsp_addr[SHARESPACE_READ] = dsp_head.read_addr;
    return arm_head.write_addr;
}

int DSP_mem_read(uint8_t *buf) //
{
    int ret = -1, i = 0, tmp = 0;
    uint32_t msg_start_addr = 0;
    uint32_t msg_end_addr = 0;
    uint32_t msg_size = 0;

#if SHARE_SPACE_HEAD_END
    uint32_t min_addr = sizeof(struct msg_head_t);        //
    uint32_t max_addr = 4096 - sizeof(struct msg_head_t); //
#else
    uint32_t min_addr = sizeof(struct msg_head_t);
    uint32_t max_addr = 4096; //  msg.arm_write_size;
#endif

    // mem_sync( pVirArmBuf+SHARE_SPACE_HEAD_OFFSET,  sizeof(struct msg_head_t));
    // memcpy((void *)&arm_head, (void *)(pVirArmBuf+SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
    mem_sync(pVirDspBuf + SHARE_SPACE_HEAD_OFFSET, sizeof(struct msg_head_t));
    memcpy((void *)&dsp_head, (void *)(pVirDspBuf + SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));

    if (arm_head.read_addr == dsp_head.write_addr) // msg kong
    {
        return 0;
    }

    msg_start_addr = arm_head.read_addr;
    msg_end_addr = dsp_head.write_addr;

    if (arm_head.read_addr < dsp_head.write_addr)
    {
        msg_size = dsp_head.write_addr - arm_head.read_addr;
    }
    if (arm_head.read_addr > dsp_head.write_addr)
    {
        msg_size = max_addr - min_addr - (arm_head.read_addr - dsp_head.write_addr);
    }

    if (msg_start_addr + msg_size <= max_addr)
    {
        mem_sync(pVirDspBuf + msg_start_addr, msg_size);
        memcpy((void *)buf, (void *)(pVirDspBuf + msg_start_addr), msg_size);

        msg_start_addr += msg_size;
        if (msg_start_addr >= max_addr)
        {
            msg_start_addr = min_addr;
        }
    }
    else
    {
        int len1 = max_addr - msg_start_addr;
        mem_sync(pVirDspBuf + msg_start_addr, len1);
        memcpy((void *)buf, (void *)(pVirDspBuf + msg_start_addr), len1);
        mem_sync(pVirDspBuf + min_addr, msg_size - len1);
        memcpy((void *)(buf + len1), (void *)(pVirDspBuf + min_addr), msg_size - len1);
        msg_start_addr = min_addr + msg_size - len1;
    }

    sharespace_arm_addr[SHARESPACE_READ] = msg_start_addr;
    sharespace_dsp_addr[SHARESPACE_WRITE] = dsp_head.write_addr;

    return msg_size;
}

void set_DSP_mem_read_pos(uint32_t read_addr)
{
    if (read_addr > 0)
    {
        arm_head.read_addr = sharespace_arm_addr[SHARESPACE_READ];
        // memcpy((void *)(armBuf+SHARE_SPACE_HEAD_OFFSET), (void *)&arm_head, sizeof(struct msg_head_t));
        // msync (armBuf,4096,MS_SYNC);
    }
    else
    {
        sharespace_arm_addr[SHARESPACE_READ] = arm_head.read_addr;
    }
}

int sharespace_wait_dsp_init()
{
    arm_head.read_addr = sizeof(struct msg_head_t);
    arm_head.write_addr = sizeof(struct msg_head_t);
    arm_head.init_state = 1;
    memcpy((void *)(pu8ArmBuf + SHARE_SPACE_HEAD_OFFSET), (void *)&arm_head, sizeof(struct msg_head_t));
    sharespace_arm_addr[SHARESPACE_WRITE] = arm_head.write_addr;
    sharespace_arm_addr[SHARESPACE_READ] = arm_head.read_addr;
    while (1)
    {
        msync(pu8DspBuf, 4096, MS_INVALIDATE);
        memcpy((void *)&dsp_head, (void *)(pu8DspBuf + SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));

        GAM_DEBUG_printf("dsp_head.init_state %x\n", dsp_head.init_state);
        GAM_DEBUG_printf("dsp_head.read_addr %x\n", dsp_head.read_addr);
        GAM_DEBUG_printf("dsp_head.write_addr %x\n", dsp_head.write_addr);
        if (dsp_head.init_state == 1)
        {
            sharespace_dsp_addr[SHARESPACE_WRITE] = dsp_head.write_addr;
            sharespace_dsp_addr[SHARESPACE_READ] = dsp_head.read_addr;
            break;
        }
    }
}

int sharespace_read_dsp(uint8_t *buf)
{
    int ret = -1, i = 0, tmp = 0;
    uint32_t msg_start_addr = 0;
    uint32_t msg_end_addr = 0;
    uint32_t msg_size = 0;

#if SHARE_SPACE_HEAD_END
    uint32_t min_addr = sizeof(struct msg_head_t);        //
    uint32_t max_addr = 4096 - sizeof(struct msg_head_t); //
#else
    uint32_t min_addr = sizeof(struct msg_head_t);
    uint32_t max_addr = 4096; //
#endif

    msync(pu8DspBuf, 4096, MS_INVALIDATE);
    memcpy((void *)&dsp_head, (void *)(pu8DspBuf + SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));

    if (arm_head.read_addr == dsp_head.write_addr) // msg kong
    {
        return 0;
    }

    msg_start_addr = arm_head.read_addr;
    msg_end_addr = dsp_head.write_addr;

    while (msg_start_addr != msg_end_addr)
    {
        buf[msg_size++] = pu8DspBuf[msg_start_addr++];
        if (msg_start_addr >= max_addr)
        {
            msg_start_addr = min_addr;
        }
    }
    sharespace_arm_addr[SHARESPACE_READ] = msg_start_addr;
    sharespace_dsp_addr[SHARESPACE_WRITE] = dsp_head.write_addr;
    arm_head.read_addr = sharespace_arm_addr[SHARESPACE_READ];
    return msg_size;
}

void sharespace_munmap()
{
    int ret = 0;
    ret = munmap(pu8ArmBuf, 4096);
    if (ret < 0)
    {
        printf("munmap pu8ArmBuf fail!\n");
    }
    ret = munmap(pu8DspBuf, 4096);
    if (ret < 0)
    {
        printf("munmap pu8DspBuf fail!\n");
    }
}
