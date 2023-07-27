/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c (for L5)
  * @brief   SD Disk I/O driver
  * @note    To be completed by the user according to the project board in use.
  *          (see templates available in the FW pack, Middlewares\Third_Party\FatFs\src\drivers folder).
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "dbg.h"

#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SD_DEFAULT_BLOCK_SIZE 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern SD_HandleTypeDef hsd1;

extern uint32_t sd_dma_rx_semaphore;
extern uint32_t sd_dma_tx_semaphore;
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT SD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE lun)
{
  /* USER CODE BEGIN SD_initialize */
  Stat = STA_NOINIT;

  /* Place for user code (may require BSP functions/defines to be added to the project) */

  HAL_StatusTypeDef res1;
  HAL_SD_CardCSDTypeDef CSD;
  HAL_SD_CardInfoTypeDef pCardInfo;

  DSTATUS stat = RES_OK;


  print_str("Init");
  while(HAL_SD_Init(&hsd1)!=HAL_OK){

	  print_str("Error");
	  while(1){};
  }

  HAL_SD_GetCardInfo(&hsd1, &pCardInfo);

  print_param("type",pCardInfo.CardType);
  print_param("ver",pCardInfo.CardVersion);
  print_param("class",pCardInfo.Class);
  print_param("BlockNbr",pCardInfo.BlockNbr);
  print_param("BlockSize",pCardInfo.BlockSize);
  print_param("LogBlockNbr",pCardInfo.LogBlockNbr);
  print_param("LogBlockSize",pCardInfo.LogBlockSize);


  print_param("Speed",hsd1.SdCard.CardSpeed);

  return stat;
  /* USER CODE END SD_initialize */
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  /* USER CODE BEGIN SD_status */
  Stat = STA_NOINIT;
  // HAL_SD_CardStateTypeDef pStatus;
  /* Place for user code (may require BSP functions/defines to be added to the project) */

  //pStatus=HAL_SD_GetCardState(&hsd1);
  //print_param("status",pStatus);
  return RES_OK;
  /* USER CODE END SD_status */
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN SD_read */
  uint32_t tickstart;
  sd_dma_rx_semaphore=0;

  while(HAL_SD_GetCardState(&hsd1)==HAL_SD_CARD_RECEIVING){}

  if(HAL_SD_ReadBlocks_DMA(&hsd1, buff, sector, count)==HAL_OK){
   tickstart = HAL_GetTick();
   while (sd_dma_rx_semaphore==0)
   {
    if ((HAL_GetTick() - tickstart) >=  10000) {
    	//print_param("rerr",0);
    	return RES_ERROR;
    }
   }
   if(HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER){
     //print_str("rok",0);
     return RES_OK;
   }
  }
  return RES_ERROR;
  /* USER CODE END SD_read */
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN SD_write */

  uint32_t tickstart;
  sd_dma_tx_semaphore=0;
  HAL_StatusTypeDef write_status;

  /* Place for user code (may require BSP functions/defines to be added to the project) */

  while(HAL_SD_GetCardState(&hsd1)==HAL_SD_CARD_PROGRAMMING){}

  write_status=HAL_SD_WriteBlocks_DMA(&hsd1, buff, sector, count);
  if(write_status==HAL_OK){
   tickstart = HAL_GetTick();
   while (sd_dma_tx_semaphore==0)
   {
    if ((HAL_GetTick() - tickstart) >=  10000) {
    	print_str("write_tmout_err");
    	return RES_ERROR;
    }
   }
   return RES_OK;

  }
  print_param("write_hal_err",write_status);
  return RES_ERROR;
  /* USER CODE END SD_write */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  /* USER CODE BEGIN SD_ioctl */
  DRESULT res = RES_ERROR;
  HAL_SD_CardInfoTypeDef  CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    HAL_SD_GetCardInfo(&hsd1, &CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
	HAL_SD_GetCardInfo(&hsd1, &CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
	HAL_SD_GetCardInfo(&hsd1, &CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
	res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;

  /* USER CODE END SD_ioctl */
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN UserCode */
/* can be used to add code */
/* USER CODE END UserCode */
