/*
 * uart.c
 *
 *  Created on: May 6, 2026
 *      Author: TEMP
 */


#include "uart.h"
#include "qbuffer.h"


#ifdef _USE_HW_UART

static bool is_open[UART_MAX_CH];
static bool is_rx_open[UART_MAX_CH];

static qbuffer_t qbuffer[UART_MAX_CH];
static uint8_t rx_buf[UART_MAX_CH][256];
static uint8_t rx_data[UART_MAX_CH];


#ifdef _USE_UART1
UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_rx;
#endif

#ifdef _USE_UART2
UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_tx;

static uint8_t           tx_buf[2][256];
static volatile bool     tx_busy = false;
static volatile bool     tx_pending = false;
static volatile uint8_t  tx_active_buf = 0U;
static volatile uint16_t tx_pending_length = 0U;
#endif


#ifdef _USE_UART2
/* Reset only the display TX software state. */
static void uartDisplayTxReset(void)
{
  uint32_t primask;


  primask = __get_PRIMASK();
  __disable_irq();

  tx_busy = false;
  tx_pending = false;
  tx_active_buf = 0U;
  tx_pending_length = 0U;

  __set_PRIMASK(primask);
}

/* Keep only the newest full display snapshot while DMA is busy. */
static uint32_t uartDisplayWriteDma(uint8_t *p_data, uint32_t length)
{
  uint32_t primask;
  uint8_t buf_index;


  if ((is_open[_DEF_UART2] != true) ||
      (p_data == NULL) ||
      (length == 0U) ||
      (length > sizeof(tx_buf[0])))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (tx_busy == true)
  {
    buf_index = tx_active_buf ^ 1U;
    memcpy(tx_buf[buf_index], p_data, length);
    tx_pending_length = (uint16_t)length;
    tx_pending = true;

    __set_PRIMASK(primask);
    return length;
  }

  if (huart1.gState != HAL_UART_STATE_READY)
  {
    __set_PRIMASK(primask);
    return 0U;
  }

  buf_index = tx_active_buf;
  memcpy(tx_buf[buf_index], p_data, length);
  tx_busy = true;
  tx_pending = false;
  tx_pending_length = 0U;

  __set_PRIMASK(primask);

  if (HAL_UART_Transmit_DMA(&huart1,
                            tx_buf[buf_index],
                            (uint16_t)length) != HAL_OK)
  {
    uartDisplayTxReset();
    return 0U;
  }

  return length;
}
#endif


bool uartInit(void)
{
  for (int i=0; i<UART_MAX_CH; i++)
  {
    is_open[i] = false;
    is_rx_open[i] = false;
  }

#ifdef _USE_UART2
  uartDisplayTxReset();
#endif


  return true;
}

bool uartOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;


  switch(ch)
  {
    case _DEF_UART1:
#ifdef _USE_UART1
      huart2.Instance = USART2;
      huart2.Init.BaudRate = baud;
      huart2.Init.WordLength = UART_WORDLENGTH_8B;
      huart2.Init.StopBits = UART_STOPBITS_1;
      huart2.Init.Parity = UART_PARITY_NONE;
      huart2.Init.Mode = UART_MODE_TX_RX;
      huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart2.Init.OverSampling = UART_OVERSAMPLING_16;
      huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
      huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

      if (is_open[ch] == true)
      {
        HAL_UART_DeInit(&huart2);
      }
      is_open[ch] = false;
      is_rx_open[ch] = false;

      qbufferCreate(&qbuffer[ch], &rx_buf[ch][0], sizeof(rx_buf[ch]));

      if (HAL_UART_Init(&huart2) == HAL_OK)
      {
        qbuffer[ch].in  = 0;
        qbuffer[ch].out = 0;

        if (HAL_UART_Receive_IT(&huart2, &rx_data[ch], 1) == HAL_OK)
        {
          ret = true;
          is_open[ch] = true;
          is_rx_open[ch] = true;
        }
        else
        {
          (void)HAL_UART_DeInit(&huart2);
        }
      }
#endif
      break;

    case _DEF_UART2:
#ifdef _USE_UART2
      huart1.Instance = USART1;
      huart1.Init.BaudRate = baud;
      huart1.Init.WordLength = UART_WORDLENGTH_8B;
      huart1.Init.StopBits = UART_STOPBITS_1;
      huart1.Init.Parity = UART_PARITY_NONE;
      huart1.Init.Mode = UART_MODE_TX;
      huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart1.Init.OverSampling = UART_OVERSAMPLING_16;
      huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
      huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

      if (is_open[ch] == true)
      {
        HAL_UART_DeInit(&huart1);
      }
      is_open[ch] = false;
      is_rx_open[ch] = false;
      uartDisplayTxReset();

      if (HAL_UART_Init(&huart1) == HAL_OK)
      {
        ret = true;
        is_open[ch] = true;
      }
#endif
      break;
  }

  return ret;
}

bool uartClose(uint8_t ch)
{
  HAL_StatusTypeDef status = HAL_OK;

  if ((ch >= UART_MAX_CH) || (is_open[ch] != true))
  {
    return ch < UART_MAX_CH;
  }

  switch (ch)
  {
    case _DEF_UART1:
#ifdef _USE_UART1
      (void)HAL_UART_AbortReceive(&huart2);
      status = HAL_UART_DeInit(&huart2);
      qbufferFlush(&qbuffer[ch]);
#endif
      break;

    case _DEF_UART2:
#ifdef _USE_UART2
      (void)HAL_UART_AbortTransmit(&huart1);
      uartDisplayTxReset();
      status = HAL_UART_DeInit(&huart1);
#endif
      break;

    default:
      return false;
  }

  is_open[ch] = false;
  is_rx_open[ch] = false;

  return status == HAL_OK;
}

uint32_t uartAvailable(uint8_t ch)
{
  uint32_t ret = 0;


  switch(ch)
  {
    case _DEF_UART1:
#ifdef _USE_UART1
      if ((is_open[ch] == true) &&
          (is_rx_open[ch] != true) &&
          (huart2.RxState == HAL_UART_STATE_READY))
      {
        if (HAL_UART_Receive_IT(&huart2, &rx_data[ch], 1) == HAL_OK)
        {
          is_rx_open[ch] = true;
        }
      }
      ret = qbufferAvailable(&qbuffer[ch]);
#endif
      break;

    case _DEF_UART2:
#ifdef _USE_UART2
      ret = 0U;
#endif
      break;
  }

  return ret;
}

uint8_t uartRead(uint8_t ch)
{
  uint8_t ret = 0;


  switch(ch)
  {
    case _DEF_UART1:
#ifdef _USE_UART1
      qbufferRead(&qbuffer[_DEF_UART1], &ret, 1);
#endif
      break;

    case _DEF_UART2:
#ifdef _USE_UART2
      ret = 0U;
#endif
      break;
  }

  return ret;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status = HAL_ERROR;


  switch(ch)
  {
    case _DEF_UART1:
#ifdef _USE_UART1
      status = HAL_UART_Transmit(&huart2, p_data, length, 100);
#endif
      break;

    case _DEF_UART2:
#ifdef _USE_UART2
      ret = uartDisplayWriteDma(p_data, length);
#endif
      break;
  }

  if (status == HAL_OK)
  {
    ret = length;
  }

  return ret;
}

/* Continue with the newest pending snapshot after USART1 TX completes. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef _USE_UART2
  uint32_t primask;
  uint16_t length = 0U;
  uint8_t buf_index = 0U;


  if (huart->Instance != USART1)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (tx_pending == true)
  {
    tx_active_buf ^= 1U;
    buf_index = tx_active_buf;
    length = tx_pending_length;
    tx_pending = false;
    tx_pending_length = 0U;
  }
  else
  {
    tx_busy = false;
  }

  __set_PRIMASK(primask);

  if (length > 0U)
  {
    if (HAL_UART_Transmit_DMA(&huart1, tx_buf[buf_index], length) != HAL_OK)
    {
      uartDisplayTxReset();
    }
  }
#else
  (void)huart;
#endif
}

uint32_t uartPrintf(uint8_t ch, char *fmt, ...)
{
  char buf[256];
  va_list args;
  int len;
  uint32_t ret = 0;


  va_start(args, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len > 0)
  {
    if (len >= (int)sizeof(buf))
    {
      len = sizeof(buf) - 1;
    }
    ret = uartWrite(ch, (uint8_t *)buf, (uint32_t)len);
  }

  return ret;
}

uint32_t uartGetBaud(uint8_t ch)
{
  uint32_t ret = 0;


  switch(ch)
  {
    case _DEF_UART1:
#ifdef _USE_UART1
      ret = huart2.Init.BaudRate;
#endif
      break;

    case _DEF_UART2:
#ifdef _USE_UART2
      ret = huart1.Init.BaudRate;
#endif
      break;
  }

  return ret;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint8_t ch;


  if (huart->Instance == USART2)
  {
    ch = _DEF_UART1;
  }
  else if (huart->Instance == USART1)
  {
#ifdef _USE_UART2
    if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U)
    {
      uartDisplayTxReset();
    }
#endif

    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF |
                          UART_CLEAR_NEF  |
                          UART_CLEAR_FEF  |
                          UART_CLEAR_PEF);
    return;
  }
  else
  {
    return;
  }

  is_rx_open[ch] = false;

  __HAL_UART_CLEAR_FLAG(huart,
                        UART_CLEAR_OREF |
                        UART_CLEAR_NEF  |
                        UART_CLEAR_FEF  |
                        UART_CLEAR_PEF);

  if (huart->RxState == HAL_UART_STATE_READY)
  {
    if (HAL_UART_Receive_IT(huart, &rx_data[ch], 1) == HAL_OK)
    {
      is_rx_open[ch] = true;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t ch;


  if (huart->Instance == USART2)
  {
    ch = _DEF_UART1;
  }
  else if (huart->Instance == USART1)
  {
    return;
  }
  else
  {
    return;
  }

  is_rx_open[ch] = false;
  (void)qbufferWrite(&qbuffer[ch], &rx_data[ch], 1);

  if (HAL_UART_Receive_IT(huart, &rx_data[ch], 1) == HAL_OK)
  {
    is_rx_open[ch] = true;
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


/* USART1(PA9) 은 _USE_UART2 채널 전용이다. 그 채널을 쓰지 않으면 핸들과 DMA
 * 정의 자체가 없으므로 이 블록도 같이 빠져야 한다. */
#ifdef _USE_UART2
  if (uartHandle->Instance == USART1)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * USART1 GPIO
     * PA9 -> USART1_TX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Active USART1 TX DMA: DMA1 Channel4 / Request2. */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart1_tx);

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    /* USART1 IRQ handles the final UART TC stage of DMA TX. */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else
#endif
  if (uartHandle->Instance == USART2)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * USART2 GPIO
     * PA2       -> USART2_TX
     * PA15/JTDI -> USART2_RX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CubeMX RX DMA setup; active CLI RX still uses Receive_IT(). */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart2_rx);

    HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
#ifdef _USE_UART2
  if (uartHandle->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

    HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);

    if (uartHandle->hdmarx != NULL)
    {
      HAL_DMA_DeInit(uartHandle->hdmarx);
    }
    if (uartHandle->hdmatx != NULL)
    {
      HAL_DMA_DeInit(uartHandle->hdmatx);
    }

    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  }
  else
#endif
  if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_15);

    if (uartHandle->hdmarx != NULL)
    {
      HAL_DMA_DeInit(uartHandle->hdmarx);
    }

    HAL_NVIC_DisableIRQ(USART2_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  }
}

#endif
