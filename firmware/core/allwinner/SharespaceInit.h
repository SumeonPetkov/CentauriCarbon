#ifndef __SHARESPACE_INIT__H__
#define __SHARESPACE_INIT__H__




#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdarg.h>
#include <stdbool.h>
#include <getopt.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <stdio.h>

#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <sys/ioctl.h>
#include "../klippy/chelper/pyhelper.h"
#include "dspMemOps.h"
// #include "../klippy/debug.h"

#define CHOOSE_DSP_WRITE_SPACE		0
#define CHOOSE_ARM_WRITE_SPACE		1
#define CHOOSE_DSP_LOG_SPACE		2

#ifdef __cplusplus
extern "C"
{
#endif


enum dsp_debug_cmd {
	CMD_REFRESH_LOG_HEAD_ADDR = 0x00,
	CMD_READ_DEBUG_MSG = 0x01,
	CMD_WRITE_DEBUG_MSG = 0x03,
};
struct debug_msg_t {
	uint32_t sys_cnt;
	uint32_t log_head_addr;
	uint32_t log_end_addr;
	uint32_t log_head_size;
};
struct dsp_sharespace_t {
	/* dsp write space msg */
	uint32_t dsp_write_addr;
	uint32_t dsp_write_size;

	/* arm write space msg */
	uint32_t arm_write_addr;
	uint32_t arm_write_size;

	/* dsp log space msg */
	uint32_t dsp_log_addr;
	uint32_t dsp_log_size;

	uint32_t mmap_phy_addr;
	uint32_t mmap_phy_size;

	/* arm read addr about dsp log*/
	uint32_t arm_read_dsp_log_addr;
	/* save msg value */
	struct debug_msg_t debug_msg;
};

struct msg_head_t {
	uint32_t read_addr;
	uint32_t write_addr;
	uint32_t init_state;
};

#define SHARESPACE_READ 1
#define SHARESPACE_WRITE 0
extern uint8_t *pu8ArmBuf ;
extern uint8_t *pu8DspBuf ;
extern struct dsp_sharespace_t sharespace_addr;
extern uint16_t sharespace_arm_addr[2];

int sharespace_mmap();
int set_dsp_ops_addr( uint32_t arm_write_addr ,uint32_t dsp_write_addr );
int set_dsp_ops_addr_no_mmap( uint32_t arm_write_addr ,uint32_t dsp_write_addr );

int set_arm_ops_addr(char * arm_write_addr,char *dsp_write_addr );
void sharespace_munmap();
int sharespace_wait_dsp_init(   );
int sharespace_read_dsp(uint8_t* buf);
int DSP_mem_write(uint8_t* cmd, int len);
int DSP_mem_read( uint8_t* buf) ;       //
void set_DSP_mem_read_pos(uint32_t read_addr);
void set_arm_mem_sync(  void *fun_sync);
int wait_dsp_set_init( );

#ifdef __cplusplus
}
#endif


#endif