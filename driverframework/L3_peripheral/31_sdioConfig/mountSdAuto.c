#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

#include <dfs_elm.h>
#include <dfs_fs.h>
#include <dfs_file.h>

#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <stdint.h>

#define DBG_TAG "app.card"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef AUTO_MOUNT_SD

void mountSdAuto(void) {
  static rt_bool_t mount_status = RT_FALSE;
  if (!mount_status) {
    if (rt_device_find("sd0") != RT_NULL) {
      if (dfs_mount("sd0", "/", "elm", 0, 0) == RT_EOK) {
        LOG_I("sd card mount to '/'");
        mount_status = RT_TRUE;
      } else {
        LOG_W("sd card mount to '/' failed!");
      }
    }
  }
}

#endif