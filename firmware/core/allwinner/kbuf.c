#include <unistd.h>
#include <getopt.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include "dspMemOps.h"
#include "SharespaceInit.h"
#define KBUF_DEBUG
#include "kbuf.h"
#include "debug.h"
#define KBUF_MGR_DEV_PATH "/dev/kbuf-mgr-0"

static volatile int run_flag = 1;
static void signal_deal_SIGINT(int sig)
{
	signal(sig, SIG_IGN);
	run_flag = 0;
}

struct user_wapper_buf_data_t
{
	struct kbuf_buf_data_t buf_data;
	int mgr_fd;
	int map_fd;
	void *addr; // user vaddr
};

int kbuf_free_buffer(struct user_wapper_buf_data_t *data)
{
	printf("<%s:%d>\n", __func__, __LINE__);
	if (data->addr && data->addr != MAP_FAILED)
	{
		printf("<%s:%d>munmap\n", __func__, __LINE__);
		munmap(data->addr, data->buf_data.len);
	}
	data->addr = NULL;

	if (data->map_fd > 0)
		close(data->map_fd);
	data->map_fd = 0;

	while (data->mgr_fd > 0 && data->buf_data.pa)
	{
		if (ioctl(data->mgr_fd, KBUF_MGR_DEV_IOCTL_DESTROY_BUF, &data->buf_data) < 0)
		{
			fprintf(stderr, "failed to ioctl DESTROY_BUF (%s)\n", strerror(errno));
			printf("<%s:%d>wait 1s...\n", __func__, __LINE__);
			usleep(1 * 1000 * 1000);
			continue;
		}
		data->buf_data.pa = 0;
	}

	if (data->mgr_fd > 0)
		close(data->map_fd);
	data->mgr_fd = 0;

	return 0;
}

int kbuf_alloc_buffer(struct user_wapper_buf_data_t *data)
{
	int ret;
	char map_dev_path[128];
	// printf("<%s:%d>\n", __func__, __LINE__);
	data->mgr_fd = open(KBUF_MGR_DEV_PATH, O_RDWR);
	if (data->mgr_fd < 0)
	{
		fprintf(stderr, "failed to open %s (%s)\n", KBUF_MGR_DEV_PATH, strerror(errno));
		goto err;
	}
	// printf("mgr_fd:%d\n", data->mgr_fd);

	ret = ioctl(data->mgr_fd, KBUF_MGR_DEV_IOCTL_CREATE_BUF, &data->buf_data);
	if (ret < 0)
	{
		fprintf(stderr, "failed to ioctl CREATE_BUF (%s)\n", strerror(errno));
		goto err;
	}
	// printf("ioctl ret:%d\n", ret);

	snprintf(map_dev_path, sizeof(map_dev_path), "/dev/kbuf-map-%d-%s", data->buf_data.minor, data->buf_data.name);
	data->map_fd = open(map_dev_path, O_RDWR);
	if (data->map_fd < 0)
	{
		fprintf(stderr, "failed to open %s (%s)\n", map_dev_path, strerror(errno));
		usleep(5 * 1000 * 1000);
		goto err;
	}
	// printf("map_fd:%d\n", data->map_fd);

	data->addr = mmap(NULL, data->buf_data.len, PROT_READ | PROT_WRITE, MAP_SHARED, data->map_fd, 0);
	if (data->addr == MAP_FAILED)
	{
		data->addr = NULL;
		fprintf(stderr, "failed to mmap failed (%s)\n", strerror(errno));
		goto err;
	}
	// printf("mmap:%p\n", data->addr);

	return 0;
err:
	kbuf_free_buffer(data);
	return ret;
}

int kbuf_put_buffer(struct user_wapper_buf_data_t *data)
{
	printf("<%s:%d>\n", __func__, __LINE__);
	if (data->addr && data->addr != MAP_FAILED)
	{
		printf("<%s:%d>munmap\n", __func__, __LINE__);
		munmap(data->addr, data->buf_data.len);
	}
	data->addr = NULL;

	if (data->map_fd > 0)
		close(data->map_fd);
	data->map_fd = 0;

	if (data->mgr_fd > 0)
		close(data->map_fd);
	data->mgr_fd = 0;

	return 0;
}

int kbuf_get_buffer(struct user_wapper_buf_data_t *data)
{
	int ret;
	char map_dev_path[128];
	printf("<%s:%d>\n", __func__, __LINE__);
	data->mgr_fd = open(KBUF_MGR_DEV_PATH, O_RDWR);
	if (data->mgr_fd < 0)
	{
		fprintf(stderr, "failed to open %s (%s)\n", KBUF_MGR_DEV_PATH, strerror(errno));
		goto err;
	}
	printf("mgr_fd:%d\n", data->mgr_fd);

	ret = ioctl(data->mgr_fd, KBUF_MGR_DEV_IOCTL_GET_BUF, &data->buf_data);
	if (ret < 0)
	{
		fprintf(stderr, "failed to ioctl GET_BUF (%s)\n", strerror(errno));
		goto err;
	}
	printf("ioctl ret:%d\n", ret);

	snprintf(map_dev_path, sizeof(map_dev_path), "/dev/kbuf-map-%d-%s", data->buf_data.minor, data->buf_data.name);
	data->map_fd = open(map_dev_path, O_RDWR);
	if (data->map_fd < 0)
	{
		fprintf(stderr, "failed to open %s (%s)\n", map_dev_path, strerror(errno));
		usleep(5 * 1000 * 1000);
		goto err;
	}
	printf("map_fd:%d\n", data->map_fd);

	data->addr = mmap(NULL, data->buf_data.len, PROT_READ | PROT_WRITE, MAP_SHARED, data->map_fd, 0);
	if (data->addr == MAP_FAILED)
	{
		data->addr = NULL;
		fprintf(stderr, "failed to mmap failed (%s)\n", strerror(errno));
		goto err;
	}
	printf("mmap:%p\n", data->addr);

	return 0;
err:
	kbuf_put_buffer(data);
	return ret;
}

void kbuf_test_use_cur_buf(unsigned long paddr)
{
	struct user_wapper_buf_data_t data;
	memset(&data, 0, sizeof(data));
	data.buf_data.pa = paddr;

	if (kbuf_get_buffer(&data))
	{
		printf("<%s:%d>failed\n", __func__, __LINE__);
		goto exit;
	}

	kbuf_show_buf_data(&data.buf_data);

	// TODO
	while (run_flag)
	{
		unsigned long *ptr = (unsigned long *)data.addr;
		usleep(1 * 1000 * 1000);
		printf("<%s:%d>read  %lx from paddr:%lx(user vaddr:%lx, kernel vaddr:%lx)\n",
			   __func__,
			   __LINE__,
			   (unsigned long)*ptr,
			   (unsigned long)data.buf_data.pa,
			   (unsigned long)data.addr,
			   (unsigned long)data.buf_data.va);
	}

	kbuf_put_buffer(&data);
exit:
	return;
}

void kbuf_test_use_new_buf(void)
{
	unsigned long cnt = 0;
	struct user_wapper_buf_data_t data;
	memset(&data, 0, sizeof(data));
	snprintf(data.buf_data.name, sizeof(data.buf_data.name), "test");
	data.buf_data.len = 128 * 1024;
	data.buf_data.type = KBUF_TYPE_NONCACHE;

	if (kbuf_alloc_buffer(&data))
	{
		printf("<%s:%d>failed\n", __func__, __LINE__);
		goto exit;
	}

	kbuf_show_buf_data(&data.buf_data);

#if 1 // test
	{
		unsigned int *ptr = data.addr;
		printf("<%s:%d>read: %x\n", __func__, __LINE__, *ptr);
	}
#endif

	// check
	for (unsigned long i = 0; i < (data.buf_data.len / sizeof(unsigned long)); i++)
	{
		unsigned long *ptr = data.addr;
		ptr[i] = i;
	}
	for (unsigned long i = 0; i < (data.buf_data.len / sizeof(unsigned long)); i++)
	{
		unsigned long *ptr = data.addr;
		if (ptr[i] != i)
		{
			printf("<%s:%d>cmp data failed: %lx != %lx\n", __func__, __LINE__, ptr[i], i);
			break;
		}
	}

	// TODO
	while (run_flag)
	{
		unsigned long *ptr = (unsigned long *)data.addr;
		usleep(1 * 1000 * 1000);
		*ptr = cnt;
		printf("<%s:%d>write %lx to paddr:%lx(user vaddr:%lx, kernel vaddr:%lx)\n",
			   __func__,
			   __LINE__,
			   (unsigned long)cnt,
			   (unsigned long)data.buf_data.pa,
			   (unsigned long)data.addr,
			   (unsigned long)data.buf_data.va);
		cnt++;
	}

	kbuf_free_buffer(&data);
exit:
	return;
}

int test(int argc, char *argv[])
{
	signal(SIGINT, signal_deal_SIGINT);

	if (argc >= 2)
	{
		char *ptr;
		unsigned long paddr = strtoul(argv[1], &ptr, 16);
		printf("run kbuf_test_use_cur_buf(0x%lx)\n", paddr);
		kbuf_test_use_cur_buf(paddr);
	}
	else
	{
		kbuf_test_use_new_buf();
	}

	printf("return\n");
	return 0;
}

extern struct DspMemOps R528_Dsp_kbuf_Ops;
void kbuf_mem_sync(void *addr, uint32_t len)
{
}
int kbuf_use_cur_buf(struct user_wapper_buf_data_t *data)
{
	// data.buf_data.pa = paddr;//unsigned long paddr
	data->buf_data.len = 4 * 4096;
	data->buf_data.type = KBUF_TYPE_NONCACHE;

	if (kbuf_get_buffer(data))
	{
		printf("<%s:%d>failed\n", __func__, __LINE__);
		return -1;
	}
	// kbuf_show_buf_data(&data.buf_data);
	return 0;
}

int kbuf_use_new_buf(struct user_wapper_buf_data_t *data)
{
	snprintf(data->buf_data.name, sizeof(data->buf_data.name), "test");
	data->buf_data.len = 4 * 4096; // 128 *1024;
	data->buf_data.type = KBUF_TYPE_NONCACHE;

	if (kbuf_alloc_buffer(data))
	{
		printf("<%s:%d>failed\n", __func__, __LINE__);
		return -1;
	}
	// printf("data.addr 1 : %p \n", data.addr);
	// kbuf_show_buf_data(&data.buf_data);
	return 0;
}
int init_dsp_kbuf()
{
	printf("init_dsp_kbuf 0\n");
	unsigned long cnt = 0;
	struct user_wapper_buf_data_t data;
	memset(&data, 0, sizeof(data));

	int fd = sharespace_mmap();
	if (fd <= 0)
	{
		GAM_ERR_printf("sharespace_mmap errer  \n");
		return -1;
	}
	data.buf_data.pa = sharespace_addr.arm_write_addr;
	// printf("init_dsp_kbuf %x\n",sharespace_addr.dsp_write_addr );
	// printf("init_dsp_kbuf %x\n",sharespace_addr.arm_write_addr );
	// printf("init_dsp_kbuf %x\n",sharespace_addr.dsp_log_addr );
	printf("init_dsp_kbuf 1\n");

	if (kbuf_use_new_buf(&data) < 0)
	{
		return -1;
	}
	set_arm_ops_addr((char *)(data.addr), (char *)(data.addr + 4096));
	// printf("data.addr 2 : %p \n", data.addr);
	set_dsp_ops_addr_no_mmap(data.buf_data.pa, data.buf_data.pa + 4096);
	// printf("data.addr 3 : %p \n", data.buf_data.pa);
	printf("init_dsp_kbuf 2\n");

	set_arm_mem_sync(kbuf_mem_sync);
	printf("init_dsp_kbuf 3\n");

	wait_dsp_set_init();
	printf("init_dsp_kbuf 4\n");

	R528_Dsp_kbuf_Ops.fd_write = -1;
	R528_Dsp_kbuf_Ops.fd_read = -1;
	R528_Dsp_kbuf_Ops.write_lenth = 0;
	R528_Dsp_kbuf_Ops.read_lenth = 0;

	R528_Dsp_kbuf_Ops.receive_window = RECEIVE_WINDOW_DSP528;
	R528_Dsp_kbuf_Ops.mcu_type = MCU_TYPE_DSP528;
	R528_Dsp_kbuf_Ops.clock_freq = CLOCK_FREQ_DSP528;
	R528_Dsp_kbuf_Ops.serial_baud = SERIAL_BAUD_DSP528;
	R528_Dsp_kbuf_Ops.pwm_max = PWM_MAX_DSP528;
	R528_Dsp_kbuf_Ops.adc_max = ADC_MAX_DSP528;
	R528_Dsp_kbuf_Ops.stats_sumsq_base = STATS_SUMSQ_BASE_DSP528;

	// printf("init_dsp_kbuf ok \n");
	return 0;
}
int reinit_dsp_kbuf()
{
    return 0;
}
int deinit_dsp_kbuf()
{
#if 0 // have bug
	kbuf_free_buffer(&data);
#endif
    return 0;
}
int read_dsp_kbuf(uint8_t *buf)
{
	return sharespace_read_dsp(buf);
}
void set_read_dsp_kbuf_pos(uint32_t read_addr)
{
}
int write_dsp_kbuf(uint8_t *cmd, int len) //-6-send-g-g-2022-06-16
{
    return 0;
}

struct DspMemOps R528_Dsp_kbuf_Ops =
	{
		init : init_dsp_kbuf,
		de_init : deinit_dsp_kbuf,
		re_init : reinit_dsp_kbuf,
		mem_read : DSP_mem_read,
		set_read_pos : set_DSP_mem_read_pos,
		fd_write_type : SQT_MEM,
		mem_write : DSP_mem_write,

		receive_window : RECEIVE_WINDOW_DSP528,
		mcu_type : MCU_TYPE_DSP528,
		clock_freq : CLOCK_FREQ_DSP528,
		serial_baud : SERIAL_BAUD_DSP528,
		pwm_max : PWM_MAX_DSP528,
		adc_max : ADC_MAX_DSP528,
		stats_sumsq_base : STATS_SUMSQ_BASE_DSP528,

	};

struct DspMemOps *GetDspKbufOps()
{
	return &R528_Dsp_kbuf_Ops;
}
