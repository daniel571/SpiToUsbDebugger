/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct TxBufferConf
{
	uint8_t* buff;
	uint16_t size;
	uint16_t write_index;
	uint8_t read_enable;
}TxBuffer;
uint8_t buffer[7];
int x = 0;
TxBuffer CDC_TxBuff1;
TxBuffer CDC_TxBuff2;
uint8_t UserTxBufferFS2[APP_TX_DATA_SIZE];
uint8_t FlushFlag = 0;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
//const char data_stringBig2f[] = {'1',' ','A', ' ', 'B', ' ', 'C', ' ', 'D', ' ', 'E', ' ', 'F', ' ', 'G', ' ', 'H', ' ', 'I', ' ', 'J', ' ', 'K', ' ', 'L', ' ', 'M', ' ', 'N', ' ','O', ' ', 'P',' ', 'Q', ' ','S', ' ', 'T',' ', 'U', ' ', 'V', ' ', 'X', ' ', 'Y', ' ', 'Z',' ','!',
//		'A', ' ', 'A', ' ', 'B', ' ', 'C', ' ', 'D', ' ', 'E', ' ', 'F', ' ', 'G', ' ', 'H', ' ', 'I', ' ', 'J', ' ', 'K', ' ', 'L', ' ', 'M', ' ', 'N', ' ','O', ' ', 'P',' ', 'Q', ' ','S', ' ', 'T',' ', 'U', 'f','f','f','f','f','f','f','f','f','f','f','f','f','f','f','f','\n'};
uint16_t gCDC_TxFifoWriteIndex = 0;
uint16_t gCDC_TxFifoReadIndex = 0;
uint16_t gCDC_TxFifoUsage = 0;
uint8_t gCDC_TxSelect = 0;
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
extern int g_enable_usb_push;
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  CDC_TxBuff1.buff=UserTxBufferFS;
  CDC_TxBuff1.size = APP_TX_DATA_SIZE;
  CDC_TxBuff1.write_index = 0;
  CDC_TxBuff2.buff=UserTxBufferFS2;
  CDC_TxBuff2.size = APP_TX_DATA_SIZE;
  CDC_TxBuff2.write_index = 0;

  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
    	buffer[0] = pbuf[0];
    	buffer[1] = pbuf[1];
    	buffer[2] = pbuf[2];
    	buffer[3] = pbuf[3];
    	buffer[4] = pbuf[4];
    	buffer[5] = pbuf[5];
    	buffer[6] = pbuf[6];
    	break;

    case CDC_GET_LINE_CODING:
    	pbuf[0] = buffer[0];
    	pbuf[1] = buffer[1];
    	pbuf[2] = buffer[2];
    	pbuf[3] = buffer[3];
    	pbuf[4] = buffer[4];
    	pbuf[5] = buffer[5];
    	pbuf[6] = buffer[6];
    	break;

    case CDC_SET_CONTROL_LINE_STATE: // DTS - data terminal ready H->D control
    	g_enable_usb_push = 1;
    break;

    case CDC_SEND_BREAK:
    	g_enable_usb_push =  0;
    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmited callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
static uint16_t CDC_TxFifoInsert(uint8_t* data, uint16_t Len) //APP_TX_DATA_SIZE
{
	TxBuffer *ptx_buf;
	if(gCDC_TxSelect&0x01)
		ptx_buf = &CDC_TxBuff1;
	else
		ptx_buf = &CDC_TxBuff2;

	if (ptx_buf->size < (ptx_buf->write_index+Len))
		return ptx_buf->write_index;
	ptx_buf->read_enable = 0;
	memcpy(&ptx_buf->buff[ptx_buf->write_index], data, Len);
	ptx_buf->write_index += Len;
	ptx_buf->read_enable = 1;
	return ptx_buf->write_index;
}

uint8_t CDC_TxFifoFlush() //APP_TX_DATA_SIZE
{
	TxBuffer *ptx_buf;
	uint8_t retval;
	if(gCDC_TxSelect&0x01)
		ptx_buf = &CDC_TxBuff1;
	else
		ptx_buf = &CDC_TxBuff2;

	if (ptx_buf->write_index == 0)
		return 1;

	// flush to usb in ep fifo
	while(!ptx_buf->read_enable);
	retval = CDC_Transmit_FS(ptx_buf->buff, ptx_buf->write_index);

	if(retval == USBD_OK)
	{
		// change fifo to insert operation so avoiding conflict during flush
		ptx_buf->write_index = 0;
		return 0;
	}
	return 1;
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
void CDC_TransmitWithFifo_FS(uint8_t* Buf, uint16_t Len)
{
  uint16_t cur_size;
  cur_size = CDC_TxFifoInsert(Buf, Len);
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
	  return;
  }
  if((cur_size > APP_TX_DATA_SIZE/4) && (!FlushFlag))
  {
	  FlushFlag = 1;
  	  gCDC_TxSelect++;
  }
//	  CDC_TxFifoFlush();
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
