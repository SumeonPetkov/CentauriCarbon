#ifndef __MSGBOX_H__
#define __MSGBOX_H__

#include <linux/ioctl.h>
#include <linux/types.h>
#include <stdint.h> // uint64_t
/**
 * struct rpmsg_endpoint_info - endpoint info representation
 * @name: name of service
 * @src: local address
 * @dst: destination address
 */
struct rpmsg_endpoint_info {
	char name[32];
	__u32 src;
	__u32 dst;
};

#define RPMSG_CREATE_EPT_IOCTL	_IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL	_IO(0xb5, 0x2)

#define MSGBOX_ALLOW_READ 1
#define MSGBOX_ALLOW_WRITE 0


#define MSGBOX_IS_READ 0
#define MSGBOX_IS_WRITE 1

extern uint16_t msgbox_new_msg[2];
extern uint16_t msgbox_send_msg[2];

 extern int fd_msgbox;

#define MSGBOX_ENABLE 1
#define READ_MSGBOX_ENABLE 0
#define WRITE_MSGBOX_ENABLE 1
#define SYNC_MSGBOX_ENABLE 0


#define SHARE_SPACE_SYNC 0
#if SHARE_SPACE_SYNC
extern int input_wait_read;

#define SYS_IDLE_STATE              0xcbda0001
#define DSP_READ_STATE           0xcbda0002
#define DSP_WRITE_STATE         0xcbda0003
#define ARM_READ_STATE           0xcbda0004
#define ARM_WRITE_STATE         0xcbda0005
#endif

int msgbox_rpmsg_init();
void msgbox_send_signal(uint32_t read_write , uint16_t value);
uint32_t msgbox_read_signal(uint32_t read);
void msgbox_poll_read();
#endif