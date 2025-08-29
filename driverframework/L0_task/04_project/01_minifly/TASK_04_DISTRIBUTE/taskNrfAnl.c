#include <rtthread.h>
#include <rtdevice.h>
#include "taskNrfRec.h"
#include "taskNrfAnl.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

static struct rt_thread taskNrfAnlTid;
static rt_align(RT_ALIGN_SIZE) rt_uint8_t taskNrfAnlStack[THREAD_STACK_SIZE];

static void atkpReceiveAnl(atkp_t *anlPacket) {
  if (anlPacket->msgID == DOWN_COMMAND) {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_COMMAND_EN
    anlCommand(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_ACK) {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_ACK_EN
    anlAck(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_RCDATA) {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_RCDATA_EN
    anlRcData(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_POWER) /*nrf51822*/
  {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_POWER_EN
    anlPower(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_REMOTER) /*遥控器*/
  {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_REMOTE_EN
    anlRemote(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_PID1) {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_PID_EN
    anlPid1(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_PID2) {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_PID_EN
    anlPid2(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_PID3) {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_PID_EN
    anlPid3(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_PID4) {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_PID_EN
    anlPid4(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_PID5) {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_PID_EN
    anlPid5(anlPacket);
#endif
  } else if (anlPacket->msgID == DOWN_PID6) {
#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_PID_EN
    anlPid6(anlPacket);
#endif
  }
}

static void taskNrfAnlEntry(void *parameter) {
  struct rt_messagequeue *recv_mq = NULL;
  while (1) {
    if (recv_mq != NULL) {
      break;
    }
    recv_mq = getNrfRecvMq();
    rt_thread_mdelay(100);
  }

  while (1) {
    atkp_t pkt;
    rt_memset(&pkt, 0, sizeof(pkt));
    if (rt_mq_recv(recv_mq, &pkt, sizeof(pkt), RT_WAITING_FOREVER) > 0) {
      atkpReceiveAnl(&pkt);
    }
  }
}

static int taskNrfAnlInit(void) {
  rt_thread_init(&taskNrfAnlTid, "L0_minifly_nrfAnl", taskNrfAnlEntry, RT_NULL, taskNrfAnlStack, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&taskNrfAnlTid);
  rt_kprintf("nrf anl task started\n");
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_EN
INIT_APP_EXPORT(taskNrfAnlInit);
#endif