// #include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h> // struct timespec
// #include <asm/cacheflush.h>     //flush_dcache_all
 #include <asm/unistd.h>     //flush_dcache_all
 #include <ion_mem_alloc.h>     //GetMemAdapterOpsS

#include "debug.h"
#include "SharespaceInit.h"
#include "Sharespace.h"
#include "msgbox.h"
#define SHARE_SPACE_DEV_MEM_SYNC 0
#define SHARE_SPACE_HEAD_END 1

#define flush_dcache_all() //sunxi_ion_alloc_flush_cache(armBuf,armBuf+4095)

 #if SHARE_SPACE_HEAD_END
#define SHARE_SPACE_HEAD_OFFSET (4096-sizeof(struct msg_head_t))
#else
   #define SHARE_SPACE_HEAD_OFFSET 0
#endif

// uint16_t sharespace_arm_addr[2];
// uint16_t sharespace_dsp_addr[2];
struct msg_head_t dsp_head;
struct msg_head_t arm_head;

 #if SHARE_SPACE_DEV_MEM_SYNC
  int fd_dev_mem= -1;
#endif


#if SHARE_SPACE_SYNC
struct state_head_t {
    volatile uint32_t dsp_wr_addr_state;
    volatile uint32_t dsp_wr_addr_dsp_cmd;
    volatile uint32_t dsp_wr_addr_arm_cmd;
    volatile uint32_t arm_wr_addr_state;
    volatile uint32_t arm_wr_addr_dsp_cmd;
    volatile uint32_t arm_wr_addr_arm_cmd;
};
volatile static struct state_head_t *state_p= NULL;
 int input_wait_read = 0;
#endif

static char *armBuf = NULL;
static char *dspBuf = NULL;
extern struct DspMemOps R528_Dsp_sharespace_Ops ;
int init_arm_write_space( )
{
    int ret = 0;
    int fd = sharespace_mmap();
      if(fd < 0)
       {
         return -1;
       }
    armBuf = pu8ArmBuf;
    dspBuf = pu8DspBuf;
    // dev_mem_CS(sharespace_addr.dsp_write_addr);
    // virtual_to_physical(dspBuf);
 #if SHARE_SPACE_DEV_MEM_SYNC
// O_DIRECT: 无缓冲的输入、输出。
// O_SYNC：以同步IO方式打开文件。

       fd_dev_mem = open("/dev/mem",O_RDWR  | O_SYNC );
       if(fd_dev_mem < 0)
       {
         printf("open /dev/mem failed.\n");
         return -1;
       }
        lseek(fd_dev_mem,sharespace_addr.dsp_write_addr,0);
        // fsync(fd_dev_mem);  fsync针对单个文件起作用，会阻塞等到PageCache的更新数据真正写入到了磁盘才会返回
#endif

#if SHARE_SPACE_SYNC
    
    ret = choose_sharespace(fd, &sharespace_addr, CHOOSE_DSP_LOG_SPACE);
    if(ret < 0)
        return ret;
    state_p = (volatile  struct state_head_t  *)mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    if(state_p < 0){
        DEBUG_printf("dev mmap to fail\n");
        ret = -1;
        return ret;
    }
    
    GAM_DEBUG_printf("state_p %x\n", state_p);
    GAM_DEBUG_printf("dspBuf %x\n", dspBuf);
    GAM_DEBUG_printf("armBuf %x\n", armBuf);
    GAM_DEBUG_printf("arm_write_addr %x\n", sharespace_addr.arm_write_addr);
    GAM_DEBUG_printf("dsp_log_addr %x\n", sharespace_addr.dsp_log_addr);
    GAM_DEBUG_printf("dsp_write_addr %x\n", sharespace_addr.dsp_write_addr);
#endif

 #if SHARE_SPACE_HEAD_END
    arm_head.read_addr = sizeof(struct msg_head_t);
    arm_head.write_addr = sizeof(struct msg_head_t);
    arm_head.init_state = 1;
#else

    arm_head.read_addr = sizeof(struct msg_head_t);// + sizeof(struct debug_msg_t);
    arm_head.write_addr = sizeof(struct msg_head_t);
    arm_head.init_state = 1;
#endif

#if SHARE_SPACE_DEV_MEM_SYNC
        lseek(fd_dev_mem,sharespace_addr.arm_write_addr+SHARE_SPACE_HEAD_OFFSET,0);
        write(fd_dev_mem, (void *)&arm_head,sizeof(struct msg_head_t));
        fsync(fd_dev_mem);

    //     memcpy((void *)&arm_head, (void *)(armBuf + SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
    //    GAM_DEBUG_printf("arm_head.init_state %d\n", arm_head.init_state);
    //     GAM_DEBUG_printf("arm_head.read_addr %d\n", arm_head.read_addr);
    //     GAM_DEBUG_printf("arm_head.write_addr %d\n", arm_head.write_addr);

#else
    memcpy((void *)(armBuf + SHARE_SPACE_HEAD_OFFSET), (void *)&arm_head, sizeof(struct msg_head_t));
#endif

    sharespace_arm_addr[SHARESPACE_WRITE] = arm_head.write_addr; 
    sharespace_arm_addr[SHARESPACE_READ] = arm_head.read_addr;
    msgbox_send_msg[SHARESPACE_WRITE] = arm_head.write_addr; 
    msgbox_send_msg[SHARESPACE_READ] = arm_head.read_addr; 
    DEBUG_printf("");
    while(1)
    {
#if SHARE_SPACE_DEV_MEM_SYNC
        lseek(fd_dev_mem,sharespace_addr.dsp_write_addr +SHARE_SPACE_HEAD_OFFSET,0);
        fsync(fd_dev_mem);
        read(fd_dev_mem, (void *)&dsp_head,sizeof(struct msg_head_t));
#else
        msync (dspBuf,4096,MS_INVALIDATE); 
        memcpy((void *)&dsp_head, (void *)(dspBuf + SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
#endif
    // GAM_DEBUG_printf("arm_write_addr %x %x\n", sharespace_addr.arm_write_addr, sharespace_addr.arm_write_size);
    // GAM_DEBUG_printf("dsp_log_addr %x %x\n", sharespace_addr.dsp_log_addr, sharespace_addr.dsp_log_size);
    // GAM_DEBUG_printf("dsp_write_addr %x %x\n", sharespace_addr.dsp_write_addr, sharespace_addr.dsp_write_size);
        
        // GAM_DEBUG_printf("dsp_head.init_state %x\n", dsp_head.init_state);
        // GAM_DEBUG_printf("dsp_head.read_addr %x\n", dsp_head.read_addr);
        // GAM_DEBUG_printf("dsp_head.write_addr %x\n", dsp_head.write_addr);
        if(dsp_head.init_state == 1)
        {
            sharespace_dsp_addr[SHARESPACE_WRITE] = dsp_head.write_addr; 
            sharespace_dsp_addr[SHARESPACE_READ] = dsp_head.read_addr;
            break;
        }
            
    }
    R528_Dsp_sharespace_Ops.fd_write = fd;
    R528_Dsp_sharespace_Ops.fd_read = fd;
   R528_Dsp_sharespace_Ops.write_lenth = 0;
    R528_Dsp_sharespace_Ops.read_lenth = 0;

    return fd;
}


#if SHARE_SPACE_SYNC

uint32_t share_space_set_write_idle_state()
{
    while( (state_p->arm_wr_addr_arm_cmd) != SYS_IDLE_STATE )
    {   
        if((state_p->arm_wr_addr_arm_cmd) == ARM_WRITE_STATE )
        {
             if((state_p->arm_wr_addr_state) == ARM_WRITE_STATE ) 
             {
                while((state_p->arm_wr_addr_state) != SYS_IDLE_STATE ) 
                {
                    state_p->arm_wr_addr_state = SYS_IDLE_STATE;
                }
               while((state_p->arm_wr_addr_arm_cmd) != SYS_IDLE_STATE ) 
                {
                    state_p->arm_wr_addr_arm_cmd = SYS_IDLE_STATE;
                }
                return 1;
             }
            else if((state_p->arm_wr_addr_state) == SYS_IDLE_STATE ) 
            {
               while((state_p->arm_wr_addr_arm_cmd) != SYS_IDLE_STATE ) 
                {
                    state_p->arm_wr_addr_arm_cmd = SYS_IDLE_STATE;
                }
                return 1;
            }
             else{
                 GAM_DEBUG_printf("share_space_set_write_idle_state 1  SDA %x %x %x\n",state_p->arm_wr_addr_state ,state_p->arm_wr_addr_dsp_cmd,state_p->arm_wr_addr_arm_cmd );
                //     while((state_p->dsp_wr_addr_dsp_cmd) != SYS_IDLE_STATE ) 
                //     {
                //         state_p->dsp_wr_addr_dsp_cmd = SYS_IDLE_STATE;
                //     }
                //    return 0;
             }
        }
        GAM_DEBUG_printf("share_space_set_write_idle_state 2  SDA %x %x %x\n",state_p->arm_wr_addr_state ,state_p->arm_wr_addr_dsp_cmd,state_p->arm_wr_addr_arm_cmd ); 
    }
    return 1;
}

uint32_t share_space_set_write_state()
{
   msync (state_p,4096,MS_INVALIDATE); 
    while( (state_p->arm_wr_addr_dsp_cmd) != SYS_IDLE_STATE )
    {   
        GAM_DEBUG_printf("SDA-1 %x %x %x\n",state_p->arm_wr_addr_state ,state_p->arm_wr_addr_dsp_cmd,state_p->arm_wr_addr_arm_cmd );
    }
    while( (state_p->arm_wr_addr_state) != ARM_WRITE_STATE )
    {   
       state_p->arm_wr_addr_arm_cmd = ARM_WRITE_STATE;
        if((state_p->arm_wr_addr_arm_cmd) == ARM_WRITE_STATE )
        {
             if((state_p->arm_wr_addr_dsp_cmd) == SYS_IDLE_STATE ) 
             {
                while((state_p->arm_wr_addr_state) != ARM_WRITE_STATE ) 
                {
                    state_p->arm_wr_addr_state = ARM_WRITE_STATE;
                }
                return 1;
             }
             else{
                 GAM_DEBUG_printf("share_space_set_write_state 1  SDA %x %x %x\n",state_p->arm_wr_addr_state ,state_p->arm_wr_addr_dsp_cmd,state_p->arm_wr_addr_arm_cmd );
                //     while((state_p->arm_wr_addr_arm_cmd) != SYS_IDLE_STATE ) 
                //     {
                //         state_p->arm_wr_addr_arm_cmd = SYS_IDLE_STATE;
                //     }
                //    return 0;
             }
        } 
        else{
            GAM_DEBUG_printf("share_space_set_write_state 2  SDA %x %x %x\n",state_p->arm_wr_addr_state ,state_p->arm_wr_addr_dsp_cmd,state_p->arm_wr_addr_arm_cmd );
            // while((state_p->arm_wr_addr_arm_cmd) != SYS_IDLE_STATE ) 
            // {
            //     state_p->arm_wr_addr_arm_cmd = SYS_IDLE_STATE;
            // }
            // return 0;
        }
    }
}
uint32_t share_space_set_read_state()
{
    while( (state_p->dsp_wr_addr_state) != ARM_READ_STATE )
    {   
       state_p->dsp_wr_addr_arm_cmd = ARM_READ_STATE;
        if((state_p->dsp_wr_addr_arm_cmd) == ARM_READ_STATE )
        {
             if((state_p->dsp_wr_addr_dsp_cmd) == SYS_IDLE_STATE ) 
             {
                while((state_p->dsp_wr_addr_state) != ARM_READ_STATE ) 
                {
                    state_p->dsp_wr_addr_state = ARM_READ_STATE;
                }
                return 1;
             }
             else{
                    while((state_p->dsp_wr_addr_arm_cmd) != SYS_IDLE_STATE ) 
                    {
                        state_p->dsp_wr_addr_arm_cmd = SYS_IDLE_STATE;
                    }
                    GAM_DEBUG_printf("share_space_set_read_state 1  SDA %x %x %x\n",state_p->dsp_wr_addr_state ,state_p->dsp_wr_addr_dsp_cmd,state_p->dsp_wr_addr_arm_cmd );
                   return 0;
             }
        } 
        else{
            while((state_p->dsp_wr_addr_arm_cmd) != SYS_IDLE_STATE ) 
            {
                state_p->dsp_wr_addr_arm_cmd = SYS_IDLE_STATE;
            }
            GAM_DEBUG_printf("share_space_set_read_state 2  SDA %x %x %x\n",state_p->dsp_wr_addr_state ,state_p->dsp_wr_addr_dsp_cmd,state_p->dsp_wr_addr_arm_cmd );
            return 0;
        }
    }
}

uint32_t share_space_set_read_idle_state()
{
    while( (state_p->dsp_wr_addr_arm_cmd) != SYS_IDLE_STATE )
    {   
        if((state_p->dsp_wr_addr_arm_cmd) == ARM_READ_STATE )
        {
             if((state_p->dsp_wr_addr_state) == ARM_READ_STATE ) 
             {
                while((state_p->dsp_wr_addr_state) != SYS_IDLE_STATE ) 
                {
                    state_p->dsp_wr_addr_state = SYS_IDLE_STATE;
                }
                 while((state_p->dsp_wr_addr_arm_cmd) != SYS_IDLE_STATE ) 
                {
                    state_p->dsp_wr_addr_arm_cmd = SYS_IDLE_STATE;
                }
                return 1;
             }
            else if((state_p->dsp_wr_addr_state) == SYS_IDLE_STATE ) 
            {
               while((state_p->dsp_wr_addr_arm_cmd) != SYS_IDLE_STATE ) 
                {
                    state_p->dsp_wr_addr_arm_cmd = SYS_IDLE_STATE;
                }
                return 1;
            }
             else{
                 GAM_DEBUG_printf("share_space_set_read_idle_state 1  SDA %x %x %x\n",state_p->dsp_wr_addr_state ,state_p->dsp_wr_addr_dsp_cmd,state_p->dsp_wr_addr_arm_cmd );
                //    return 0;
             }
        } 
         GAM_DEBUG_printf("share_space_set_read_idle_state 2  SDA %x %x %x\n",state_p->dsp_wr_addr_state ,state_p->dsp_wr_addr_dsp_cmd,state_p->dsp_wr_addr_arm_cmd );

    }
    return 1;
}


#endif



int write_into_arm_write_space(uint8_t* cmd, int len)   //-6-send-g-g-2022-06-16
{
    int ret = 0;

    uint32_t free_size = 0;
 #if SHARE_SPACE_HEAD_END
    uint32_t min_addr = sizeof(struct msg_head_t); //msg_head_addr
    uint32_t max_addr = 4096 - sizeof(struct msg_head_t) ;//msg_end_addr  msg.arm_write_size;

#else
    uint32_t min_addr = sizeof(struct msg_head_t);
    uint32_t max_addr = 4096;//msg_end_addr  msg.arm_write_size;
#endif

      
    if((len > 4000) || (len < 0) )
    {
        return arm_head.write_addr;
    }

#if SHARE_SPACE_SYNC
    
    msync (state_p,4096,MS_INVALIDATE); 
    while(!share_space_set_write_state())
    {
    }
    msync (state_p,4096,MS_SYNC); 
    
#endif

#if SHARE_SPACE_DEV_MEM_SYNC
        // lseek(fd_dev_mem,sharespace_addr.arm_write_addr+SHARE_SPACE_HEAD_OFFSET,0);
        // read(fd_dev_mem, (void *)&arm_head,sizeof(struct msg_head_t));
        // arm_head.init_state = 3;
        // lseek(fd_dev_mem,sharespace_addr.arm_write_addr+SHARE_SPACE_HEAD_OFFSET,0);
        // write(fd_dev_mem, (void *)&arm_head,sizeof(struct msg_head_t));
        fsync(fd_dev_mem);
        
        
#else
    // memcpy((void *)&arm_head, (void *)(armBuf+SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
    // arm_head.init_state = 3;
    // memcpy((void *)(armBuf+SHARE_SPACE_HEAD_OFFSET), &arm_head, sizeof(struct msg_head_t));
    // msync (armBuf,4096,MS_SYNC); 
    // flush_dcache_all();
//  __clear_cache(armBuf,armBuf+4095);
#endif

    while(1)
    {
#if SHARE_SPACE_DEV_MEM_SYNC
        // lseek(fd_dev_mem,sharespace_addr.dsp_write_addr +SHARE_SPACE_HEAD_OFFSET,0);
        // fsync(fd_dev_mem);
        // read(fd_dev_mem, (void *)&dsp_head,sizeof(struct debug_msg_t));
        msgbox_poll_read();
        dsp_head.read_addr = msgbox_new_msg[0];
#else
        // memcpy((void *)&dsp_head, (void *)(dspBuf +SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
        msgbox_poll_read();
        dsp_head.read_addr = msgbox_new_msg[0];
#endif
        if(dsp_head.read_addr <= arm_head.write_addr)
        {
            free_size = max_addr - min_addr - (arm_head.write_addr - dsp_head.read_addr);
            if(free_size > len)
                break;
             else
             {
                GAM_ERR_printf("len = %d free_size = %d dsp_head.read_addr = %d arm_head.write_addr = %d   \n", len,free_size,dsp_head.read_addr,arm_head.write_addr);
             }
        }
        else if(dsp_head.read_addr > arm_head.write_addr)
        {
            free_size = dsp_head.read_addr - arm_head.write_addr;
            if(free_size > len)
                break;
            else
             {
                GAM_ERR_printf("len = %d free_size = %d dsp_head.read_addr = %d arm_head.write_addr = %d   \n", len,free_size,dsp_head.read_addr,arm_head.write_addr);
             }
        }
    }
    uint32_t pmsg = arm_head.write_addr;

#if SHARE_SPACE_DEV_MEM_SYNC
    if(arm_head.write_addr + len  <= max_addr  )
    {
       lseek(fd_dev_mem,sharespace_addr.arm_write_addr+arm_head.write_addr,0);
        write(fd_dev_mem, (void *)cmd,len);
        fsync(fd_dev_mem);
        pmsg += len;
        if(pmsg >= max_addr)
        {
            pmsg = min_addr;
        }
    }
    else
    {
        int len1 = max_addr -  arm_head.write_addr;
        lseek(fd_dev_mem,sharespace_addr.arm_write_addr+arm_head.write_addr,0);
        write(fd_dev_mem, (void *)cmd,len1 );
        fsync(fd_dev_mem);
        len -= len1;
        lseek(fd_dev_mem,sharespace_addr.arm_write_addr+min_addr,0);// 
        write(fd_dev_mem, (void *)(cmd + len1),len );
        fsync(fd_dev_mem);
        pmsg = min_addr + len;
    }
    arm_head.write_addr = pmsg;
    arm_head.init_state = 1;
    lseek(fd_dev_mem,sharespace_addr.arm_write_addr+SHARE_SPACE_HEAD_OFFSET,0);
    write(fd_dev_mem, (void *)&arm_head,sizeof(struct msg_head_t));
    fsync(fd_dev_mem);

#else
    for(int i = 0; i < len; i++)
    {
        armBuf[pmsg] = *cmd;
        pmsg++;
        cmd++;
        if(pmsg >= max_addr)
        {
            pmsg = min_addr;
        }
    }
    arm_head.write_addr = pmsg;
    arm_head.init_state = 1;
    memcpy((void *)(armBuf+SHARE_SPACE_HEAD_OFFSET), &arm_head, sizeof(struct msg_head_t));
    msync (armBuf,4096,MS_SYNC ); 
    //  
     flush_dcache_all();
    //  __clear_cache(armBuf,armBuf+4095);
 
#endif

#if SHARE_SPACE_SYNC

    msync (state_p,4096,MS_INVALIDATE); 
    while(!share_space_set_write_idle_state())
    {
    }
    msync (state_p,4096,MS_SYNC); 
    
#endif
    sharespace_arm_addr[SHARESPACE_WRITE] = arm_head.write_addr; 
    sharespace_dsp_addr[SHARESPACE_READ] = dsp_head.read_addr;
    return arm_head.write_addr;
}

int read_dsp_space(uint8_t* buf)
{
    int ret = -1, i = 0, tmp = 0;
    uint32_t msg_start_addr = 0;
    uint32_t msg_end_addr = 0;
    uint32_t msg_size = 0;

 #if SHARE_SPACE_HEAD_END
    uint32_t min_addr = sizeof(struct msg_head_t); //
    uint32_t max_addr = 4096 - sizeof(struct msg_head_t) ;//    
#else
    uint32_t min_addr = sizeof(struct msg_head_t);
    uint32_t max_addr = 4096;//  msg.arm_write_size;
#endif


#if SHARE_SPACE_SYNC

    if(!share_space_set_read_state())
    {
        
        while(!share_space_set_read_idle_state())
        {
        }
        
         return -1;
    }
    
     input_wait_read = 0;
#endif


#if SHARE_SPACE_DEV_MEM_SYNC
        // lseek(fd_dev_mem,sharespace_addr.arm_write_addr+SHARE_SPACE_HEAD_OFFSET,0);
        // read(fd_dev_mem, (void *)&arm_head,sizeof(struct msg_head_t));

        lseek(fd_dev_mem,sharespace_addr.dsp_write_addr+SHARE_SPACE_HEAD_OFFSET ,0);
        fsync(fd_dev_mem);
        read(fd_dev_mem, (void *)&dsp_head,sizeof(struct msg_head_t));
#else
        
        // memcpy((void *)&arm_head, (void *)(armBuf+SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
        msync (dspBuf,4096,MS_INVALIDATE); 
        memcpy((void *)&dsp_head, (void *)(dspBuf + SHARE_SPACE_HEAD_OFFSET), sizeof(struct msg_head_t));
#endif
    
    
    if(arm_head.read_addr == dsp_head.write_addr)  //msg kong
    {
#if SHARE_SPACE_SYNC
        
        while(!share_space_set_read_idle_state())
        {
        }
        
        ret = 0;
#endif
        return 0;
    }

    msg_start_addr = arm_head.read_addr;
    msg_end_addr = dsp_head.write_addr;

#if SHARE_SPACE_DEV_MEM_SYNC
    if(arm_head.read_addr < dsp_head.write_addr)
    {
        msg_size = dsp_head.write_addr - arm_head.read_addr;
    }
    if(arm_head.read_addr > dsp_head.write_addr)
    {
        msg_size = max_addr - min_addr - (arm_head.read_addr - dsp_head.write_addr);
    }
    
    if(msg_start_addr + msg_size  <= max_addr  )
    {
        lseek(fd_dev_mem,sharespace_addr.dsp_write_addr+msg_start_addr,0);
        fsync(fd_dev_mem);
        read(fd_dev_mem, (void *)buf,msg_size);
        msg_start_addr += msg_size;
        if(msg_start_addr >= max_addr)
        {
            msg_start_addr = min_addr;
        }
    }
    else
    {
         int len1 = max_addr -  msg_start_addr;
        lseek(fd_dev_mem,sharespace_addr.dsp_write_addr+msg_start_addr,0);
        read(fd_dev_mem, (void *)buf,len1);
        lseek(fd_dev_mem,sharespace_addr.dsp_write_addr+min_addr,0);
        read(fd_dev_mem, (void *)(buf + len1),msg_size-len1  );
        msg_start_addr = min_addr + msg_size-len1;
    }
#else
    while(msg_start_addr != msg_end_addr)
    {
        buf[msg_size++] = dspBuf[msg_start_addr++];
        if(msg_start_addr >= max_addr)
        {
            msg_start_addr = min_addr;
        }
    }
#endif
    sharespace_arm_addr[SHARESPACE_READ] = msg_start_addr;
    sharespace_dsp_addr[SHARESPACE_WRITE] = dsp_head.write_addr;
#if SHARE_SPACE_SYNC
    while(!share_space_set_read_idle_state())
    {
    }

#endif
    
	return msg_size;
}
void set_read_dsp_space_pos(uint32_t read_addr)
{
     if (read_addr > 0) {  
        arm_head.read_addr = sharespace_arm_addr[SHARESPACE_READ];
        #if SHARE_SPACE_DEV_MEM_SYNC
            // lseek(fd_dev_mem,sharespace_addr.arm_write_addr+SHARE_SPACE_HEAD_OFFSET,0);
            // write(fd_dev_mem, (void *)&arm_head,sizeof(struct msg_head_t));
            // fsync(fd_dev_mem);
        #else
            // memcpy((void *)(armBuf+SHARE_SPACE_HEAD_OFFSET), (void *)&arm_head, sizeof(struct msg_head_t));
            // msync (armBuf,4096,MS_SYNC); 
            // flush_dcache_all();
        #endif
     }
     else
     {
         sharespace_arm_addr[SHARESPACE_READ] = arm_head.read_addr ;
     }
}




int munmap_arm_dsp()
{
    int ret = 0;
    sharespace_munmap();
 #if SHARE_SPACE_SYNC
   ret = munmap((char *)state_p, sharespace_addr.dsp_log_size);
    if (ret < 0)
    {
        printf("state_p fail!\n");
    }
 #endif
    return 0;
}
void msync_arm_dsp()
{
   #if SHARE_SPACE_SYNC
    msync (state_p,4096,MS_SYNC); 
    #endif
}

struct DspMemOps R528_Dsp_sharespace_Ops =
{
    init:				init_arm_write_space,
    de_init:				munmap_arm_dsp,
    mem_read:			read_dsp_space,
    set_read_pos:			set_read_dsp_space_pos,
    fd_write_type:			SQT_MEM,
    mem_write:			write_into_arm_write_space
};
struct DspMemOps* GetDspsharespaceOps()
{
	return &R528_Dsp_sharespace_Ops;
}


