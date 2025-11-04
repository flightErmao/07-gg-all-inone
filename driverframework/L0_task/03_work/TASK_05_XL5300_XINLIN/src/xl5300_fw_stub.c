#include <stdint.h>

/* 若外部固件库不可用，提供空实现以便通过链接。
 * 当 LoadFirmware 返回 0 时，上层应跳过下载流程。 */

const unsigned char Firmware_Ranging[1] = { 0x00 };

unsigned short LoadFirmware(void)
{
    return 0;
}


