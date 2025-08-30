#include <rtdevice.h>
#include <string.h>
#include <stdbool.h>
#include "deviceManager.h"

#ifdef PROJECT_MINIFLY_TASK_NRF_EN
#include "taskNrfRec.h"
#endif

#define ATKP_MAX_DATA_SIZE 128
#define ATKP_PROTOCOL_HEAD_SIZE 6
#define ATKP_ANOTC_TELEM_BUF_SIZE (ATKP_MAX_DATA_SIZE + ATKP_PROTOCOL_HEAD_SIZE)

#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA

rt_device_t dev_anotc_telem_ = RT_NULL;

/* Pack and send one ATKP packet via device */
void anotcDeviceSendDirect(atkp_t* p) {
  int dataSize;
  uint8_t cksum = 0;
  static uint8_t sendBuffer[ATKP_ANOTC_TELEM_BUF_SIZE];

  RT_ASSERT((p->dataLen <= ATKP_MAX_DATA_SIZE) == true);

  sendBuffer[0] = UP_BYTE1;
  sendBuffer[1] = UP_BYTE2;
  sendBuffer[2] = p->msgID;
  sendBuffer[3] = p->dataLen;

  memcpy(&sendBuffer[4], p->data, p->dataLen);
  dataSize = p->dataLen + 5; /* include checksum */
  for (int i = 0; i < dataSize - 1; i++) {
    cksum += sendBuffer[i];
  }
  sendBuffer[dataSize - 1] = cksum;
  if (dev_anotc_telem_ != RT_NULL) {
    rt_device_write(dev_anotc_telem_, 0, sendBuffer, dataSize);
  } else {
#ifdef PROJECT_MINIFLY_TASK_NRF_EN
    dev_anotc_telem_ = getNrfDevice();
#endif
  }
}

rt_err_t task_dev_init(char* device_name) {
  if (dev_anotc_telem_ && (dev_anotc_telem_->open_flag & RT_DEVICE_OFLAG_OPEN)) {
    return RT_EOK;
  }

#ifdef PROJECT_MINIFLY_TASK_NRF_EN
  dev_anotc_telem_ = getNrfDevice();
  return RT_EOK;
#endif

  rt_device_t new_dev = rt_device_find(device_name);
  if (new_dev == NULL) {
    return RT_ERROR;
  }

  if (rt_device_open(new_dev, RT_DEVICE_FLAG_TX_BLOCKING) != RT_EOK) {
    return RT_ERROR;
  }

  if (!strncmp(device_name, "uart", 4)) {
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = TASK_TOOL_01_ANOTC_TELEM_BAUD_RATE;
    config.rx_bufsz = 512;
    config.tx_bufsz = 512;
    config.dma_ping_bufsz = 128;
    if (rt_device_control(new_dev, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK) {
      rt_device_close(new_dev);
      return RT_ERROR;
    }
  }

  dev_anotc_telem_ = new_dev;
  return RT_EOK;
}