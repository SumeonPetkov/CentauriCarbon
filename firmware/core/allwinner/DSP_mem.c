#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/fb.h>
#include <linux/videodev2.h>
#include <sys/time.h>
#include <ion_mem_alloc.h>
#include "Sharespace.h"
#include "msgbox.h"


#define SHARE_SPACE_HEAD_END 1
 #if SHARE_SPACE_HEAD_END
#define SHARE_SPACE_HEAD_OFFSET (4096-sizeof(struct msg_head_t))
#else
   #define SHARE_SPACE_HEAD_OFFSET 0
#endif

#define DSP_MEM_SIZE 4096


struct SunxiMemOpsS* pDSP_Memops;
static struct msg_head_t dsp_head;
static struct msg_head_t arm_head;
static char *pVirArmBuf = NULL;
static char *pVirDspBuf = NULL;
// extern char *armBuf;


int set_dsp_ops_addr( uint32_t arm_write_addr ,uint32_t dsp_write_addr );

void DSP_mem_sync( void *addr,uint32_t len)
{
    SunxiMemFlushCache(pDSP_Memops, addr,  len);
}
extern struct DspMemOps R528_Dsp_Mem_Ops ;
int DSP_mem_Init( )     //
{
    printf("DSP_mem_Init (%s %d) \n",__FUNCTION__, __LINE__);
    pDSP_Memops = GetMemAdapterOpsS();
    SunxiMemOpen(pDSP_Memops);
    
	pVirArmBuf = (char*) SunxiMemPalloc(pDSP_Memops,(unsigned int)(DSP_MEM_SIZE )); // sunxi_ion_alloc_palloc
	uint32_t arm_write_addr = ( uint32_t)SunxiMemGetPhysicAddressCpu(pDSP_Memops,pVirArmBuf);

    if ((NULL == pVirArmBuf) || (0 == arm_write_addr)) {
        printf("(%s %d) ION Malloc ArmBuf failed\n",__FUNCTION__, __LINE__);
        return -1;
    }
	pVirDspBuf = (char*) SunxiMemPalloc(pDSP_Memops,(unsigned int)(DSP_MEM_SIZE ));
	uint32_t dsp_write_addr = ( uint32_t)SunxiMemGetPhysicAddressCpu(pDSP_Memops,pVirDspBuf);
    if ((NULL == pVirDspBuf) || (0 == dsp_write_addr)) {
        printf("(%s %d) ION Malloc DspBuf failed\n",__FUNCTION__, __LINE__);
        return -1;
    }
    printf("pVirArmBuf (%p %x) \n",pVirArmBuf, arm_write_addr);
    printf("pVirDspBuf (%p %x) \n",pVirDspBuf, dsp_write_addr);
    set_dsp_ops_addr(arm_write_addr,dsp_write_addr);
    set_arm_ops_addr(pVirArmBuf,pVirDspBuf);
    set_arm_mem_sync(DSP_mem_sync);
    wait_dsp_set_init();
    R528_Dsp_Mem_Ops.fd_write = -1;
    R528_Dsp_Mem_Ops.fd_read = -1;
    R528_Dsp_Mem_Ops.write_lenth = 0;
    R528_Dsp_Mem_Ops.read_lenth = 0;
    return -1;
}

int DSP_mem_DeInit()
{
    SunxiMemPfree(pDSP_Memops,(void *)pVirArmBuf);
    if(pDSP_Memops )
    SunxiMemClose(pDSP_Memops);
    return 0;
}

struct DspMemOps R528_Dsp_Mem_Ops =
{
    init:				            DSP_mem_Init,
    de_init:				    DSP_mem_DeInit,
    mem_read:	        DSP_mem_read,
    set_read_pos:       set_DSP_mem_read_pos,
    fd_write_type:			SQT_MEM,
    mem_write:			DSP_mem_write
};

struct DspMemOps* GetDspIonMemOps()
{
	return &R528_Dsp_Mem_Ops;
}

