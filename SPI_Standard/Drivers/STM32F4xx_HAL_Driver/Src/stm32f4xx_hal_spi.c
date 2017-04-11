/**
  ******************************************************************************
  * @file    stm32f4xx_hal_spi.c
  * @author  MCD Application Team
  * @version V1.7.0
  * @date    17-February-2017
  * @brief   SPI HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Serial Peripheral Interface (SPI) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
      The SPI HAL driver can be used as follows:

      (#) Declare a SPI_HandleTypeDef handle structure, for example:
          SPI_HandleTypeDef  hspi;

      (#)Initialize the SPI low level resources by implementing the HAL_SPI_MspInit() API:
          (##) Enable the SPIx interface clock
          (##) SPI pins configuration
              (+++) Enable the clock for the SPI GPIOs
              (+++) Configure these SPI pins as alternate function push-pull
          (##) NVIC configuration if you need to use interrupt process
              (+++) Configure the SPIx interrupt priority
              (+++) Enable the NVIC SPI IRQ handle
          (##) DMA Configuration if you need to use DMA process
              (+++) Declare a DMA_HandleTypeDef handle structure for the transmit or receive stream
              (+++) Enable the DMAx clock
              (+++) Configure the DMA handle parameters
              (+++) Configure the DMA Tx or Rx stream
              (+++) Associate the initialized hdma_tx handle to the hspi DMA Tx or Rx handle
              (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the DMA Tx or Rx stream

      (#) Program the Mode, BidirectionalMode , Data size, Baudrate Prescaler, NSS
          management, Clock polarity and phase, FirstBit and CRC configuration in the hspi Init structure.

      (#) Initialize the SPI registers by calling the HAL_SPI_Init() API:
          (++) This API configures also the low level Hardware GPIO, CLOCK, CORTEX...etc)
              by calling the customized HAL_SPI_MspInit() API.
     [..]
       Circular mode restriction:
      (#) The DMA circular mode cannot be used when the SPI is configured in these modes:
          (##) Master 2Lines RxOnly
          (##) Master 1Line Rx
      (#) The CRC feature is not managed when the DMA circular mode is enabled
      (#) When the SPI DMA Pause/Stop features are used, we must use the following APIs
          the HAL_SPI_DMAPause()/ HAL_SPI_DMAStop() only under the SPI callbacks
     [..]
       Master Receive mode restriction:
      (#) In Master unidirectional receive-only mode (MSTR =1, BIDIMODE=0, RXONLY=0) or
          bidirectional receive mode (MSTR=1, BIDIMODE=1, BIDIOE=0), to ensure that the SPI
          does not initiate a new transfer the following procedure has to be respected:
          (##) HAL_SPI_DeInit()
          (##) HAL_SPI_Init()

  @endverbatim

    Using the HAL it is not possible to reach all supported SPI frequency with the differents SPI Modes,
    the following tables resume the max SPI frequency reached with data size 8bits/16bits,
    according to frequency used on APBx Peripheral Clock (fPCLK) used by the SPI instance :
    
    DataSize = SPI_DATASIZE_8BIT:
    +----------------------------------------------------------------------------------------------+
    |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
    | Process | Tranfert mode  |---------------------|----------------------|----------------------|
    |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
    |==============================================================================================|
    |    T    |     Polling    | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |    /    |     Interrupt  | Fpclk/4  | Fpclk/8  |    NA     |    NA    |    NA     |   NA     |
    |    R    |----------------|----------|----------|-----------|----------|-----------|----------|
    |    X    |       DMA      | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
    |=========|================|==========|==========|===========|==========|===========|==========|
    |         |     Polling    | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
    |         |----------------|----------|----------|-----------|----------|-----------|----------|
    |    R    |     Interrupt  | Fpclk/8  | Fpclk/8  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |         |       DMA      | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/128 | Fpclk/2  |
    |=========|================|==========|==========|===========|==========|===========|==========|
    |         |     Polling    | Fpclk/2  | Fpclk/4  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
    |         |----------------|----------|----------|-----------|----------|-----------|----------|
    |    T    |     Interrupt  | Fpclk/2  | Fpclk/4  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |         |       DMA      | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/128|
    +----------------------------------------------------------------------------------------------+
    
    DataSize = SPI_DATASIZE_16BIT:
    +----------------------------------------------------------------------------------------------+
    |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
    | Process | Tranfert mode  |---------------------|----------------------|----------------------|
    |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
    |==============================================================================================|
    |    T    |     Polling    | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |    /    |     Interrupt  | Fpclk/4  | Fpclk/4  |    NA     |    NA    |    NA     |   NA     |
    |    R    |----------------|----------|----------|-----------|----------|-----------|----------|
    |    X    |       DMA      | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
    |=========|================|==========|==========|===========|==========|===========|==========|
    |         |     Polling    | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/32  | Fpclk/2  |
    |         |----------------|----------|----------|-----------|----------|-----------|----------|
    |    R    |     Interrupt  | Fpclk/4  | Fpclk/4  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |         |       DMA      | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/128 | Fpclk/2  |
    |=========|================|==========|==========|===========|==========|===========|==========|
    |         |     Polling    | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/32 |
    |         |----------------|----------|----------|-----------|----------|-----------|----------|
    |    T    |     Interrupt  | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |         |       DMA      | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/128|
    +----------------------------------------------------------------------------------------------+
     [..]
       (@) The max SPI frequency depend on SPI data size (8bits, 16bits),
           SPI mode(2 Lines fullduplex, 2 lines RxOnly, 1 line TX/RX) and Process mode (Polling, IT, DMA).
       (@)
            (+@) TX/RX processes are HAL_SPI_TransmitReceive(), HAL_SPI_TransmitReceive_IT() and HAL_SPI_TransmitReceive_DMA()
            (+@) RX processes are HAL_SPI_Receive(), HAL_SPI_Receive_IT() and HAL_SPI_Receive_DMA()
            (+@) TX processes are HAL_SPI_Transmit(), HAL_SPI_Transmit_IT() and HAL_SPI_Transmit_DMA()
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */
/** @defgroup SPI SPI
  * @brief SPI HAL module driver
  * @{
  */
#ifdef HAL_SPI_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup SPI_Private_Constants SPI Private Constants
  * @{
  */
#define SPI_DEFAULT_TIMEOUT 100U
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup SPI_Private_Functions
  * @{
  */
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfTransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAError(DMA_HandleTypeDef *hdma);
static void SPI_DMAAbortOnError(DMA_HandleTypeDef *hdma);
static void SPI_DMATxAbortCallback(DMA_HandleTypeDef *hdma);
static void SPI_DMARxAbortCallback(DMA_HandleTypeDef *hdma);
static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, uint32_t State, uint32_t Timeout, uint32_t Tickstart);
static void SPI_TxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_TxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_RxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_RxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesTxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesTxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
#if (USE_SPI_CRC != 0U)
static void SPI_RxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi);
static void SPI_RxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi);
#endif /* USE_SPI_CRC */
static void SPI_AbortRx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_AbortTx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_CloseRxTx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_CloseRx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_CloseTx_ISR(SPI_HandleTypeDef *hspi);
static HAL_StatusTypeDef SPI_CheckFlag_BSY(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SPI_Exported_Functions SPI Exported Functions
  * @{
  */

/** @defgroup SPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the SPIx peripheral:

      (+) User must implement HAL_SPI_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_SPI_Init() to configure the selected device with
          the selected configuration:
        (++) Mode
        (++) Direction
        (++) Data Size
        (++) Clock Polarity and Phase
        (++) NSS Management
        (++) BaudRate Prescaler
        (++) FirstBit
        (++) TIMode
        (++) CRC Calculation
        (++) CRC Polynomial if CRC enabled

      (+) Call the function HAL_SPI_DeInit() to restore the default configuration
          of the selected SPIx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Init1(void)
{

	HAL_SPI_MspInit(void);
  /*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
  /* Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
  Communication speed, First bit and CRC calculation state */
  WRITE_REG(SPI_Init_SPIx[0].Instance->CR1, (SPI_Init_SPIx[0].Init.Mode | SPI_Init_SPIx[0].Init.Direction | SPI_Init_SPIx[0].Init.DataSize |
		  SPI_Init_SPIx[0].Init.CLKPolarity | SPI_Init_SPIx[0].Init.CLKPhase | (SPI_Init_SPIx[0].Init.NSS & SPI_CR1_SSM) |
		  SPI_Init_SPIx[0].Init.BaudRatePrescaler | SPI_Init_SPIx[0].Init.FirstBit  | SPI_Init_SPIx[0].Init.CRCCalculation) );

  /* Configure : NSS management */
  WRITE_REG(SPI_Init_SPIx[0].Instance->CR2, (((SPI_Init_SPIx[0].Init.NSS >> 16U) & SPI_CR2_SSOE) | SPI_Init_SPIx[0].Init.TIMode));

#if (USE_SPI_CRC != 0U)
  /*---------------------------- SPIx CRCPOLY Configuration ------------------*/
  /* Configure : CRC Polynomial */
  if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    WRITE_REG(hspi->Instance->CRCPR, hspi->Init.CRCPolynomial);
  }
#endif /* USE_SPI_CRC */

#if defined(SPI_I2SCFGR_I2SMOD)
  /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
  CLEAR_BIT(SPI_Init_SPIx[0].Instance->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif /* USE_SPI_CRC */
  return HAL_OK;
}

/**
  * @brief  De Initialize the SPI peripheral.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi)
{
  /* Check the SPI handle allocation */
  if(hspi == NULL)
  {
    return HAL_ERROR;
  }

  /* Check SPI Instance parameter */
  //assert_param(IS_SPI_ALL_INSTANCE(hspi->Instance));

  //hspi->State = HAL_SPI_STATE_BUSY;

  /* Disable the SPI Peripheral Clock */
  __HAL_SPI_DISABLE(hspi);

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
  HAL_SPI_MspDeInit(hspi);

  //hspi->ErrorCode = HAL_SPI_ERROR_NONE;
  //hspi->State = HAL_SPI_STATE_RESET;

  /* Release Lock */
 // __HAL_UNLOCK(hspi);

  return HAL_OK;
}

/**
  * @brief  Initialize the SPI MSP.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_MspInit(void)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hspi);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_MspInit should be implemented in the user file
  */
}

/**
  * @brief  De-Initialize the SPI MSP.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_MspDeInit should be implemented in the user file
  */
}

/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData: pointer to transmission data buffer
  * @param  pRxData: pointer to reception data buffer
  * @param  Size: amount of data to be sent and received
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
uint8_t HAL_SPI_TransmitReceive(SPI_Channel_en Channel_en, uint8_t *pTxData)
{
  void HAL_GPIO_WritePin(SPI_ChannelDefine_st.SPI_GPIO, SPI_ChannelDefine_st.PinChannel,GPIO_PIN_SET);
   SPI_Init_SPIx[SPI_ChannelDefine_st.SPIx].Instance->DR = *pTxData;


    while (__HAL_SPI_GET_FLAG(SPI_Init_SPIx[SPI_ChannelDefine_st.SPIx], SPI_FLAG_TXE){};


      /* Check RXNE flag */
      while(__HAL_SPI_GET_FLAG(SPI_Init_SPIx[SPI_ChannelDefine_st.SPIx], SPI_FLAG_RXNE))){};
    void HAL_GPIO_WritePin(SPI_ChannelDefine_st.SPI_GPIO, SPI_ChannelDefine_st.PinChannel,GPIO_PIN_RESET);

    return SPI_Init_SPIx[0].Instance->DR;

}

/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @retval HAL status
  */

/**
  * @brief  Pause the DMA Transfer.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi)
{
  /* Process Locked */
  __HAL_LOCK(hspi);

  /* Disable the SPI DMA Tx & Rx requests */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  /* Process Unlocked */
  __HAL_UNLOCK(hspi);

  return HAL_OK;
}

/**
  * @brief  Resume the DMA Transfer.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi)
{
  /* Process Locked */
  __HAL_LOCK(hspi);

  /* Enable the SPI DMA Tx & Rx requests */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  /* Process Unlocked */
  __HAL_UNLOCK(hspi);

  return HAL_OK;
}

/**
  * @brief Stop the DMA Transfer.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi)
{
  /* The Lock is not implemented on this API to allow the user application
     to call the HAL SPI API under callbacks HAL_SPI_TxCpltCallback() or HAL_SPI_RxCpltCallback() or HAL_SPI_TxRxCpltCallback():
     when calling HAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
     and the correspond call back is executed HAL_SPI_TxCpltCallback() or HAL_SPI_RxCpltCallback() or HAL_SPI_TxRxCpltCallback()
     */

  /* Abort the SPI DMA tx Stream */
  if(hspi->hdmatx != NULL)
  {
    HAL_DMA_Abort(hspi->hdmatx);
  }
  /* Abort the SPI DMA rx Stream */
  if(hspi->hdmarx != NULL)
  {
    HAL_DMA_Abort(hspi->hdmarx);
  }

  /* Disable the SPI DMA Tx & Rx requests */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
  hspi->State = HAL_SPI_STATE_READY;
  return HAL_OK;
}

/**
  * @brief  Handle SPI interrupt request.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval None
  */
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi)
{
  uint32_t itsource = hspi->Instance->CR2;
  uint32_t itflag   = hspi->Instance->SR;

  /* SPI in mode Receiver ----------------------------------------------------*/
  if(((itflag & SPI_FLAG_OVR) == RESET) &&
     ((itflag & SPI_FLAG_RXNE) != RESET) && ((itsource & SPI_IT_RXNE) != RESET))
  {
    hspi->RxISR(hspi);
    return;
  }

  /* SPI in mode Transmitter -------------------------------------------------*/
  if(((itflag & SPI_FLAG_TXE) != RESET) && ((itsource & SPI_IT_TXE) != RESET))
  {
    hspi->TxISR(hspi);
    return;
  }

  /* SPI in Error Treatment --------------------------------------------------*/
  if(((itflag & (SPI_FLAG_MODF | SPI_FLAG_OVR | SPI_FLAG_FRE)) != RESET) && ((itsource & SPI_IT_ERR) != RESET))
  {
    /* SPI Overrun error interrupt occurred ----------------------------------*/
    if((itflag & SPI_FLAG_OVR) != RESET)
    {
      if(hspi->State != HAL_SPI_STATE_BUSY_TX)
      {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
      }
      else
      {
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
        return;
      }
    }

    /* SPI Mode Fault error interrupt occurred -------------------------------*/
    if((itflag & SPI_FLAG_MODF) != RESET)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
      __HAL_SPI_CLEAR_MODFFLAG(hspi);
    }

    /* SPI Frame error interrupt occurred ------------------------------------*/
    if((itflag & SPI_FLAG_FRE) != RESET)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FRE);
      __HAL_SPI_CLEAR_FREFLAG(hspi);
    }

    if(hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
      /* Disable all interrupts */
      __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);

      hspi->State = HAL_SPI_STATE_READY;
      /* Disable the SPI DMA requests if enabled */
      if ((HAL_IS_BIT_SET(itsource, SPI_CR2_TXDMAEN))||(HAL_IS_BIT_SET(itsource, SPI_CR2_RXDMAEN)))
      {
        CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));

        /* Abort the SPI DMA Rx channel */
        if(hspi->hdmarx != NULL)
        {
          /* Set the SPI DMA Abort callback :
          will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
          hspi->hdmarx->XferAbortCallback = SPI_DMAAbortOnError;
          HAL_DMA_Abort_IT(hspi->hdmarx);
        }
        /* Abort the SPI DMA Tx channel */
        if(hspi->hdmatx != NULL)
        {
          /* Set the SPI DMA Abort callback :
          will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
          hspi->hdmatx->XferAbortCallback = SPI_DMAAbortOnError;
          HAL_DMA_Abort_IT(hspi->hdmatx);
        }
      }
      else
      {
        /* Call user error callback */
        HAL_SPI_ErrorCallback(hspi);
      }
    }
    return;
  }
}

/**
  * @brief Tx Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxCpltCallback should be implemented in the user file
  */
}

/**
  * @brief Rx Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxCpltCallback should be implemented in the user file
  */
}

/**
  * @brief Tx and Rx Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxRxCpltCallback should be implemented in the user file
  */
}

/**
  * @brief Tx Half Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxHalfCpltCallback should be implemented in the user file
  */
}

/**
  * @brief Rx Half Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxHalfCpltCallback() should be implemented in the user file
  */
}

/**
  * @brief Tx and Rx Half Transfer callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxRxHalfCpltCallback() should be implemented in the user file
  */
}

/**
  * @brief SPI error callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
 __weak void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_ErrorCallback should be implemented in the user file
   */
  /* NOTE : The ErrorCode parameter in the hspi handle is updated by the SPI processes
            and user can use HAL_SPI_GetError() API to check the latest error occurred
  */
}

/**
  * @brief  SPI Abort Complete callback.
  * @param  hspi SPI handle.
  * @retval None
  */
__weak void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_AbortCpltCallback can be implemented in the user file.
   */
}

/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   SPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SPI.
     (+) HAL_SPI_GetState() API can be helpful to check in run-time the state of the SPI peripheral
     (+) HAL_SPI_GetError() check in run-time Errors occurring during communication
@endverbatim
  * @{
  */

/**
  * @brief  Return the SPI handle state.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI state
  */
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi)
{
  /* Return SPI handle state */
  return hspi->State;
}

/**
  * @brief  Return the SPI error code.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI error code in bitmap format
  */
uint32_t HAL_SPI_GetError(SPI_HandleTypeDef *hspi)
{
  /* Return SPI ErrorCode */
  return hspi->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup SPI_Private_Functions
  * @brief   Private functions
  * @{
  */

/**
  * @brief DMA SPI transmit process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  uint32_t tickstart = 0U;

  /* Init tickstart for timeout managment*/
  tickstart = HAL_GetTick();

  /* DMA Normal Mode */
  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0U)
  {
    /* Disable Tx DMA Request */
    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);

    /* Check the end of the transaction */
    if(SPI_CheckFlag_BSY(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    }

    /* Clear overrun flag in 2 Lines communication mode because received data is not read */
    if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->TxXferCount = 0U;
    hspi->State = HAL_SPI_STATE_READY;

    if(hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
      HAL_SPI_ErrorCallback(hspi);
      return;
    }
  }
  HAL_SPI_TxCpltCallback(hspi);
}

/**
  * @brief DMA SPI receive process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
#if (USE_SPI_CRC != 0U)
  uint32_t tickstart = 0U;
  __IO uint16_t tmpreg = 0U;

  /* Init tickstart for timeout management*/
  tickstart = HAL_GetTick();
#endif /* USE_SPI_CRC */
 
  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0U)
  {
#if (USE_SPI_CRC != 0U)
    /* CRC handling */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Wait until RXNE flag */
      if(SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SPI_FLAG_RXNE, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
      {
        /* Error on the CRC reception */
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
      }
      /* Read CRC */
      tmpreg = hspi->Instance->DR;
      /* To avoid GCC warning */
      UNUSED(tmpreg);
    }
#endif /* USE_SPI_CRC */

    /* Disable Rx/Tx DMA Request (done by default to handle the case master rx direction 2 lines) */
    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    /* Check the end of the transaction */
    if((hspi->Init.Mode == SPI_MODE_MASTER)&&((hspi->Init.Direction == SPI_DIRECTION_1LINE)||(hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
    {
      /* Disable SPI peripheral */
      __HAL_SPI_DISABLE(hspi);
    }

    hspi->RxXferCount = 0U;
    hspi->State = HAL_SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
      __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
    }
#endif /* USE_SPI_CRC */

    if(hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
      HAL_SPI_ErrorCallback(hspi);
      return;
    }
  }
  HAL_SPI_RxCpltCallback(hspi);
}

/**
  * @brief  DMA SPI transmit receive process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  uint32_t tickstart = 0U;
#if (USE_SPI_CRC != 0U)
  __IO int16_t tmpreg = 0U;
#endif /* USE_SPI_CRC */
  /* Init tickstart for timeout management*/
  tickstart = HAL_GetTick();

  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0U)
  {
#if (USE_SPI_CRC != 0U)
    /* CRC handling */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Wait the CRC data */
      if(SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
      {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
      }
      /* Read CRC to Flush DR and RXNE flag */
      tmpreg = hspi->Instance->DR;
      /* To avoid GCC warning */
      UNUSED(tmpreg);
    }
#endif /* USE_SPI_CRC */
    /* Check the end of the transaction */
    if(SPI_CheckFlag_BSY(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    }

    /* Disable Rx/Tx DMA Request */
    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    hspi->TxXferCount = 0U;
    hspi->RxXferCount = 0U;
    hspi->State = HAL_SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
      __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
    }
#endif /* USE_SPI_CRC */

    if(hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
      HAL_SPI_ErrorCallback(hspi);
      return;
    }
  }
  HAL_SPI_TxRxCpltCallback(hspi);
}

/**
  * @brief  DMA SPI half transmit process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  HAL_SPI_TxHalfCpltCallback(hspi);
}

/**
  * @brief  DMA SPI half receive process complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  HAL_SPI_RxHalfCpltCallback(hspi);
}

/**
  * @brief  DMA SPI half transmit receive process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfTransmitReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  HAL_SPI_TxRxHalfCpltCallback(hspi);
}

/**
  * @brief  DMA SPI communication error callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = (SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

/* Stop the disable DMA transfer on SPI side */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
  hspi->State = HAL_SPI_STATE_READY;
  HAL_SPI_ErrorCallback(hspi);
}

/**
  * @brief  DMA SPI communication abort callback, when initiated by HAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  HAL_SPI_ErrorCallback(hspi);
}

/**
  * @brief  DMA SPI Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_DMATxAbortCallback(DMA_HandleTypeDef *hdma)
{
  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  hspi->hdmatx->XferAbortCallback = NULL;

  /* Disable Tx DMA Request */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN );

  /* Wait until TXE flag is set */
  do
  {
    if(count-- == 0U)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      break;
    }
  }
  while((hspi->Instance->SR & SPI_FLAG_TXE) == RESET);

  /* Check if an Abort process is still ongoing */
  if(hspi->hdmarx != NULL)
  {
    if(hspi->hdmarx->XferAbortCallback != NULL)
    {
      return;
    }
  }
  
  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  /* Reset errorCode */
  hspi->ErrorCode = HAL_SPI_ERROR_NONE;

  /* Clear the Error flags in the SR register */
  __HAL_SPI_CLEAR_FREFLAG(hspi);

  /* Restore hspi->State to Ready */
  hspi->State  = HAL_SPI_STATE_READY;

  /* Call user Abort complete callback */
  HAL_SPI_AbortCpltCallback(hspi);
}

/**
  * @brief  DMA SPI Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_DMARxAbortCallback(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Disable SPI Peripheral */
  __HAL_SPI_DISABLE(hspi);

  hspi->hdmarx->XferAbortCallback = NULL;

  /* Disable Rx DMA Request */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

  /* Check if an Abort process is still ongoing */
  if(hspi->hdmatx != NULL)
  {
    if(hspi->hdmatx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  /* Reset errorCode */
  hspi->ErrorCode = HAL_SPI_ERROR_NONE;

  /* Clear the Error flags in the SR register */
  __HAL_SPI_CLEAR_OVRFLAG(hspi);
  __HAL_SPI_CLEAR_FREFLAG(hspi);  

  /* Restore hspi->State to Ready */
  hspi->State  = HAL_SPI_STATE_READY;

  /* Call user Abort complete callback */
  HAL_SPI_AbortCpltCallback(hspi);
}

/**
  * @brief  Rx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
  /* Receive data in 8bit mode */
  *hspi->pRxBuffPtr++ = *((__IO uint8_t *)&hspi->Instance->DR);
  hspi->RxXferCount--;

  /* check end of the reception */
  if(hspi->RxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      hspi->RxISR =  SPI_2linesRxISR_8BITCRC;
      return;
    }
#endif /* USE_SPI_CRC */

    /* Disable RXNE interrupt */
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

    if(hspi->TxXferCount == 0U)
    {
      SPI_CloseRxTx_ISR(hspi);
    }
  }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Rx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi)
{
  __IO uint8_t tmpreg = 0U;

  /* Read data register to flush CRC */
  tmpreg = *((__IO uint8_t *)&hspi->Instance->DR);

  /* To avoid GCC warning */

  UNUSED(tmpreg);

   /* Disable RXNE interrupt */
  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

  if(hspi->TxXferCount == 0U)
  {
    SPI_CloseRxTx_ISR(hspi);
  }
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Tx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesTxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
  *(__IO uint8_t *)&hspi->Instance->DR = (*hspi->pTxBuffPtr++);
  hspi->TxXferCount--;

  /* check the end of the transmission */
  if(hspi->TxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
      __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);
      return;
    }
#endif /* USE_SPI_CRC */

    /* Disable TXE interrupt */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);

    if(hspi->RxXferCount == 0U)
    {
      SPI_CloseRxTx_ISR(hspi);
    }
  }
}

/**
  * @brief  Rx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
  /* Receive data in 16 Bit mode */
  *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
  hspi->pRxBuffPtr += sizeof(uint16_t);
  hspi->RxXferCount--;

  if(hspi->RxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      hspi->RxISR =  SPI_2linesRxISR_16BITCRC;
      return;
    }
#endif /* USE_SPI_CRC */

    /* Disable RXNE interrupt */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE);

    if(hspi->TxXferCount == 0U)
    {
      SPI_CloseRxTx_ISR(hspi);
    }
  }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Manage the CRC 16-bit receive for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi)
{
  /* Receive data in 16 Bit mode */
  __IO uint16_t tmpreg = 0U;

  /* Read data register to flush CRC */
  tmpreg = hspi->Instance->DR;

  /* To avoid GCC warning */
  UNUSED(tmpreg);

  /* Disable RXNE interrupt */
  __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE);

  SPI_CloseRxTx_ISR(hspi);
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Tx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesTxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
  /* Transmit data in 16 Bit mode */
  hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
  hspi->pTxBuffPtr += sizeof(uint16_t);
  hspi->TxXferCount--;

  /* Enable CRC Transmission */
  if(hspi->TxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
      __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);
      return;
    }
#endif /* USE_SPI_CRC */

    /* Disable TXE interrupt */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);

    if(hspi->RxXferCount == 0U)
    {
      SPI_CloseRxTx_ISR(hspi);
    }
  }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Manage the CRC 8-bit receive in Interrupt context.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi)
{
  __IO uint8_t tmpreg = 0U;

  /* Read data register to flush CRC */
  tmpreg = *((__IO uint8_t*)&hspi->Instance->DR);

  /* To avoid GCC warning */
  UNUSED(tmpreg);

  SPI_CloseRx_ISR(hspi);
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Manage the receive 8-bit in Interrupt context.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
  *hspi->pRxBuffPtr++ = (*(__IO uint8_t *)&hspi->Instance->DR);
  hspi->RxXferCount--;

#if (USE_SPI_CRC != 0U)
  /* Enable CRC Transmission */
  if((hspi->RxXferCount == 1U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
  {
    SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
  }
#endif /* USE_SPI_CRC */

  if(hspi->RxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      hspi->RxISR =  SPI_RxISR_8BITCRC;
      return;
    }
#endif /* USE_SPI_CRC */
    SPI_CloseRx_ISR(hspi);
  }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Manage the CRC 16-bit receive in Interrupt context.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi)
{
  __IO uint16_t tmpreg = 0U;

  /* Read data register to flush CRC */
  tmpreg = hspi->Instance->DR;

  /* To avoid GCC warning */
  UNUSED(tmpreg);

  /* Disable RXNE and ERR interrupt */
  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

  SPI_CloseRx_ISR(hspi);
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Manage the 16-bit receive in Interrupt context.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
  *((uint16_t *)hspi->pRxBuffPtr) = hspi->Instance->DR;
  hspi->pRxBuffPtr += sizeof(uint16_t);
  hspi->RxXferCount--;

#if (USE_SPI_CRC != 0U)
  /* Enable CRC Transmission */
  if((hspi->RxXferCount == 1U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
  {
    SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
  }
#endif /* USE_SPI_CRC */

  if(hspi->RxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      hspi->RxISR = SPI_RxISR_16BITCRC;
      return;
    }
#endif /* USE_SPI_CRC */
    SPI_CloseRx_ISR(hspi);
  }
}

/**
  * @brief  Handle the data 8-bit transmit in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_TxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
  *(__IO uint8_t *)&hspi->Instance->DR = (*hspi->pTxBuffPtr++);
  hspi->TxXferCount--;

  if(hspi->TxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Enable CRC Transmission */
      SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
    }
#endif /* USE_SPI_CRC */
    SPI_CloseTx_ISR(hspi);
  }
}

/**
  * @brief  Handle the data 16-bit transmit in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_TxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
  /* Transmit data in 16 Bit mode */
  hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
  hspi->pTxBuffPtr += sizeof(uint16_t);
  hspi->TxXferCount--;

  if(hspi->TxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Enable CRC Transmission */
      SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
    }
#endif /* USE_SPI_CRC */
    SPI_CloseTx_ISR(hspi);
  }
}

/**
  * @brief Handle SPI Communication Timeout.
  * @param hspi: pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param Flag: SPI flag to check
  * @param State: flag state to check
  * @param Timeout: Timeout duration
  * @param Tickstart: tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, uint32_t State, uint32_t Timeout, uint32_t Tickstart)
{
  while((((hspi->Instance->SR & Flag) == (Flag)) ? SET : RESET) != State)
  {
    if(Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0U) || ((HAL_GetTick()-Tickstart) >= Timeout))
      {
        /* Disable the SPI and reset the CRC: the CRC value should be cleared
        on both master and slave sides in order to resynchronize the master
        and slave for their respective CRC calculation */

        /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

        if((hspi->Init.Mode == SPI_MODE_MASTER)&&((hspi->Init.Direction == SPI_DIRECTION_1LINE)||(hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
        {
          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);
        }

        /* Reset CRC Calculation */
        if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SPI_RESET_CRC(hspi);
        }

        hspi->State= HAL_SPI_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hspi);

        return HAL_TIMEOUT;
      }
    }
  }

  return HAL_OK;
}
/**
  * @brief Handle to check BSY flag before start a new transaction.
  * @param hspi: pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param Timeout: Timeout duration
  * @param Tickstart: tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_CheckFlag_BSY(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart)
{
  /* Control the BSY flag */
  if(SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != HAL_OK)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    return HAL_TIMEOUT;
  }
  return HAL_OK;
}

/**
  * @brief  Handle the end of the RXTX transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_CloseRxTx_ISR(SPI_HandleTypeDef *hspi)
{
  uint32_t tickstart = 0U;
  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);
  /* Init tickstart for timeout managment*/
  tickstart = HAL_GetTick();

  /* Disable ERR interrupt */
  __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

  /* Wait until TXE flag is set */
  do
  {
    if(count-- == 0U)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      break;
    }
  }
  while((hspi->Instance->SR & SPI_FLAG_TXE) == RESET);
  
  /* Check the end of the transaction */
  if(SPI_CheckFlag_BSY(hspi, SPI_DEFAULT_TIMEOUT, tickstart)!=HAL_OK)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
  }

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
  }

#if (USE_SPI_CRC != 0U)
  /* Check if CRC error occurred */
  if(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET)
  {
    hspi->State = HAL_SPI_STATE_READY;
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
    __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
    HAL_SPI_ErrorCallback(hspi);
  }
  else
  {
#endif /* USE_SPI_CRC */
    if(hspi->ErrorCode == HAL_SPI_ERROR_NONE)
    {
      if(hspi->State == HAL_SPI_STATE_BUSY_RX)
      {
      	hspi->State = HAL_SPI_STATE_READY;
        HAL_SPI_RxCpltCallback(hspi);
      }
      else
      {
      	hspi->State = HAL_SPI_STATE_READY;
        HAL_SPI_TxRxCpltCallback(hspi);
      }
    }
    else
    {
      hspi->State = HAL_SPI_STATE_READY;
      HAL_SPI_ErrorCallback(hspi);
    }
#if (USE_SPI_CRC != 0U)
  }
#endif /* USE_SPI_CRC */
}

/**
  * @brief  Handle the end of the RX transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_CloseRx_ISR(SPI_HandleTypeDef *hspi)
{
    /* Disable RXNE and ERR interrupt */
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

    /* Check the end of the transaction */
    if((hspi->Init.Mode == SPI_MODE_MASTER)&&((hspi->Init.Direction == SPI_DIRECTION_1LINE)||(hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
    {
      /* Disable SPI peripheral */
      __HAL_SPI_DISABLE(hspi);
    }

    /* Clear overrun flag in 2 Lines communication mode because received is not read */
    if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }
    hspi->State = HAL_SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
      __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
      HAL_SPI_ErrorCallback(hspi);
    }
    else
    {
#endif /* USE_SPI_CRC */
      if(hspi->ErrorCode == HAL_SPI_ERROR_NONE)
      {
        HAL_SPI_RxCpltCallback(hspi);
      }
      else
      {
        HAL_SPI_ErrorCallback(hspi);
      }
#if (USE_SPI_CRC != 0U)
    }
#endif /* USE_SPI_CRC */
}

/**
  * @brief  Handle the end of the TX transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_CloseTx_ISR(SPI_HandleTypeDef *hspi)
{
  uint32_t tickstart = 0U;
  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

  /* Init tickstart for timeout management*/
  tickstart = HAL_GetTick();

  /* Wait until TXE flag is set */
  do
  {
    if(count-- == 0U)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      break;
    }
  }
  while((hspi->Instance->SR & SPI_FLAG_TXE) == RESET);

  /* Disable TXE and ERR interrupt */
  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_ERR));

  /* Check Busy flag */
  if(SPI_CheckFlag_BSY(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
  }

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
  }

  hspi->State = HAL_SPI_STATE_READY;
  if(hspi->ErrorCode != HAL_SPI_ERROR_NONE)
  {
    HAL_SPI_ErrorCallback(hspi);
  }
  else
  {
    HAL_SPI_TxCpltCallback(hspi);
  }
}

/**
  * @}
  */

/**
  * @brief  Handle abort a Tx or Rx transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_AbortRx_ISR(SPI_HandleTypeDef *hspi)
{
  __IO uint32_t tmpreg = 0U;
  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

  /* Wait until TXE flag is set */
  do
  {
    if(count-- == 0U)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      break;
    }
  }
  while((hspi->Instance->SR & SPI_FLAG_TXE) == RESET);

  /* Disable SPI Peripheral */
  __HAL_SPI_DISABLE(hspi);    

  /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
  CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE));

  /* Flush DR Register */
  tmpreg = (*(__IO uint32_t *)&hspi->Instance->DR);

  /* To avoid GCC warning */
  UNUSED(tmpreg);
}

/**
  * @brief  Handle abort a Tx or Rx transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_AbortTx_ISR(SPI_HandleTypeDef *hspi)
{
  /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
  CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE));

  /* Disable SPI Peripheral */
  __HAL_SPI_DISABLE(hspi);
}
/**
  * @}
  */
#endif /* HAL_SPI_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
