#include "XL5300_API.h"
#include "XL5300_Config.h"
#include "XL5300_Firmware_8.h"
#include "string.h"
#include "XL5300_UserPlatform.h"
#include "flash.h"
#include "function.h"
#include "das.h"
uint32_t DATA[64];
extern XL5300_Calibration_TypeDef XL5300_Cali_Data;
utemp fa;
utemp fb;
extern uint8_t ref_tof_flag;
/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/
/**
  * @brief  FLASH擦除
  * @param  无
  * @retval 无
  */
 void APP_FlashErase(void)
{
  uint32_t PAGEError = 0;
  FLASH_EraseInitTypeDef EraseInitStruct;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGEERASE;        /* 擦写类型FLASH_TYPEERASE_PAGEERASE=Page擦, FLASH_TYPEERASE_SECTORERASE=Sector擦 */
  EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;            /* 擦写起始地址 */
  EraseInitStruct.NbPages  = sizeof(DATA) / FLASH_PAGE_SIZE;      /* 需要擦写的页数量 */
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)  /* 执行page擦除,PAGEError返回擦写错误的page,返回0xFFFFFFFF,表示擦写成功 */
  {
			Error_Handler();
  }
}

/**
  * @brief  FLASH写入
  * @param  无
  * @retval 无
  */
void APP_FlashProgram(void)
{
  uint32_t flash_program_start = FLASH_USER_START_ADDR ;                /* flash写起始地址 */
  uint32_t flash_program_end = (FLASH_USER_START_ADDR + sizeof(DATA));  /* flash写结束地址 */
  uint32_t *src = (uint32_t *)DATA;                                     /* 数组指针 */

  while (flash_program_start < flash_program_end)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, flash_program_start, src) == HAL_OK)/* 执行Program */
    {
      flash_program_start += FLASH_PAGE_SIZE; /* flash起始指针指向第一个page */
      src += FLASH_PAGE_SIZE / 4;             /* 更新数据指针 */
    }
  }
}

/**
  * @brief  FLASH查空
  * @param  无
  * @retval 无
  */
void APP_FlashBlank(void)
{
  uint32_t addr = 0;

  while (addr < sizeof(DATA))
  {
    if (0xFFFFFFFF != HW32_REG(FLASH_USER_START_ADDR + addr))
    {
			Error_Handler();
    }
    addr += 4;
  }
}

/**
  * @brief  FLASH校验
  * @param  无
  * @retval 无
  */
void APP_FlashVerify(void)
{
  uint32_t addr = 0;

  while (addr < sizeof(DATA))
  {
    DATA[addr / 4]= HW32_REG(FLASH_USER_START_ADDR + addr);
    addr += 4;
  }
}
void F003_Flash_Write(void)
{
  DATA[0]=0x01010101;
  DATA[1]=XL5300_Cali_Data.XL5300_Calibration_CK;
  DATA[2]=XL5300_Cali_Data.XL5300_Calibration_CG_Pos;
  DATA[3]=XL5300_Cali_Data.XL5300_Calibration_CG_Maxratio;
  DATA[4]=XL5300_Cali_Data.XL5300_Calibration_CG_peak;
  DATA[5]=XL5300_Cali_Data.XL5300_Calibration_Reftof;
  fa.fa = XL5300_Cali_Data.XL5300_Calibration_Offset;;
  DATA[6] = (fa.farray[3] << 24) | (fa.farray[2] << 16)| (fa.farray[1] << 8)| fa.farray[0];
  DATA[7]=XL5300_Cali_Data.XL5300_Calibration_MP;
  DATA[8]=ref_tof_flag;
  /* 解锁FLASH */	
  HAL_FLASH_Unlock();

  /* 擦除FLASH */
  APP_FlashErase();

  /* 查空FLASH */
  APP_FlashBlank();               

  /* 写FLASH */
  APP_FlashProgram();

  /* 锁定FLASH */
  HAL_FLASH_Lock();

  /* 校验FLASH */
  APP_FlashVerify();

}
void F003_Flash_Read(void)
{
	APP_FlashVerify();
	if(DATA[0]== 0x01010101)
	{
	XL5300_Cali_Data.XL5300_Calibration_CK= DATA[1];
	printf("VI530x_Calibration_CK = %#x\r\n",XL5300_Cali_Data.XL5300_Calibration_CK);
    XL5300_Cali_Data.XL5300_Calibration_CG_Pos= DATA[2];
	printf("XL5300_Calibration_CG_Pos = %d\r\n",XL5300_Cali_Data.XL5300_Calibration_CG_Pos);	
    XL5300_Cali_Data.XL5300_Calibration_CG_Maxratio= DATA[3];
	printf("XL5300_Calibration_CG_Maxratio = %d\r\n",XL5300_Cali_Data.XL5300_Calibration_CG_Maxratio);
    XL5300_Cali_Data.XL5300_Calibration_CG_peak= DATA[4];
	printf("XL5300_Calibration_CG_Pos = %d\r\n",XL5300_Cali_Data.XL5300_Calibration_CG_peak);
    XL5300_Cali_Data.XL5300_Calibration_Reftof= DATA[5];
	printf("XL5300_Calibration_Reftof = %4d\r\n",XL5300_Cali_Data.XL5300_Calibration_Reftof);  
      fb.farray[3] = VarA.arrA[6] >> 24;
      fb.farray[2] = VarA.arrA[6] >> 16;
      fb.farray[1] = VarA.arrA[6] >> 8;
      fb.farray[0] = VarA.arrA[6];
    XL5300_Cali_Data.XL5300_Calibration_Offset=fb.fa;
	printf("VI530x_Calibration_Offset = %f\r\n",XL5300_Cali_Data.XL5300_Calibration_Offset);
    XL5300_Cali_Data.XL5300_Calibration_MP= DATA[7];
	printf("XL5300_Calibration_MP = %4d\r\n",XL5300_Cali_Data.XL5300_Calibration_MP);
		ref_tof_flag= DATA[8];
	printf("REF_flag = %d\r\n",ref_tof_flag);

	}
}




