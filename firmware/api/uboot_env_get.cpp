
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <dirent.h>

#include "uboot.h"
#include "ota_update.h"
#include "debug.h"

static const char *lockname = "/var/lock/fw_printenv.lock";
static int lock_uboot_env(void)
{
	int lockfd = -1;
	lockfd = open(lockname, O_WRONLY | O_CREAT | O_TRUNC, 0666);
	if (lockfd < 0)
	{
		GAM_DEBUG_printf("Error opening U-Boot lock file %s, %s\n", lockname, strerror(errno));
		return -1;
	}
	if (flock(lockfd, LOCK_EX) < 0)
	{
		GAM_DEBUG_printf("Error locking file %s, %s\n", lockname, strerror(errno));
		close(lockfd);
		return -1;
	}
	return lockfd;
}
static void unlock_uboot_env(int lock)
{
	flock(lock, LOCK_UN);
	close(lock);
}

char *bootloader_env_get(const char *name)
{
	struct env_opts fw_env_opts = {
		.config_file = (char *)CONFIG_UBOOT_FWENV};

	int lock;
	char *value = NULL;
	char *var;

	lock = lock_uboot_env();
	if (lock < 0)
		return NULL;

	if (fw_env_open(&fw_env_opts))
	{
		unlock_uboot_env(lock);
		return NULL;
	}

	var = fw_getenv((char *)name);
	if (var)
		value = strdup(var);

	fw_env_close(&fw_env_opts);

	unlock_uboot_env(lock);
	GAM_DEBUG_printf(" bootloader_env_get  %s : %s\n", name, (value == NULL) ? "?" : value);
	return value;
}
int get_image_selector(char *cmd_str) //
{
	char *envval = bootloader_env_get("boot_partition");
	GAM_DEBUG_printf("parse: bootloader_env_get boot_partition strlen:%d: %s\n", strlen(envval), (envval == NULL) ? "?" : envval);
	char *swu_version = bootloader_env_get("swu_version");
	if ((envval == NULL) || !strcmp(envval, "bootA") || (5 != strlen(envval)))
	{
		strcat(cmd_str, "now_A_next_B");
	}
	else
	{
		strcat(cmd_str, "now_B_next_A");
	}
	strcat(cmd_str, " -k /etc/swupdate_public.pem");
	strcat(cmd_str, " -p reboot"); // 升级后重启
	// if ((swu_version != NULL))
	// {
	// 	strcat(cmd_str, " -N ");
	// 	strcat(cmd_str, swu_version);
	// }
	free(envval);
	free(swu_version);
	return 0;
}

int get_update_status() //
{
	char *recovery_status = bootloader_env_get("recovery_status");
	if ((recovery_status != NULL) && !strcmp(recovery_status, "progress"))
	{
		GAM_DEBUG_printf("get_update_status: bootloader_env_get recovery_status strlen:%d: %s\n", strlen(recovery_status), (recovery_status == NULL) ? "?" : recovery_status);
	}
	free(recovery_status);
	return 0;
}

int bootloader_env_set(const char *name, const char *value)
{
	struct env_opts fw_env_opts = {
		.config_file = (char *)CONFIG_UBOOT_FWENV};
	int lock = lock_uboot_env();
	int ret;

	if (lock < 0)
		return -1;

	if (fw_env_open(&fw_env_opts))
	{
		printf("Error: environment not initialized, %s", strerror(errno));
		unlock_uboot_env(lock);
		return -1;
	}
	fw_env_write((char *)name, (char *)value);
	ret = fw_env_flush(&fw_env_opts);
	fw_env_close(&fw_env_opts);

	unlock_uboot_env(lock);

	return ret;
}
