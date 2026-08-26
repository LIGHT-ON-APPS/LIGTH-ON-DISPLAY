/*
 * spi.c
 *
 *  Created on: Dec 30, 2025
 *      Author: TEMP
 */

#include "spi.h"
#include "cli.h"


#ifdef _USE_HW_SPI


typedef struct
{
  bool is_open;
  volatile bool is_tx_done;
  volatile bool is_rx_done;

  void (*func_tx)(void);
  void (*func_rx)(void);

  SPI_HandleTypeDef *h_spi;
  DMA_HandleTypeDef *h_dma_rx;
  DMA_HandleTypeDef *h_dma_tx;
} spi_t;



spi_t spi_tbl[SPI_MAX_CH];

#ifdef _USE_HW_CLI
static void cliSpi(cli_args_t *args);
static void cliSpiInfo(void);
#endif

// ili9488
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;


bool spiInit(void)
{
  bool ret = true;


  for (int i=0; i<SPI_MAX_CH; i++)
  {
    spi_tbl[i].is_open = false;

    spi_tbl[i].is_tx_done = true;
    spi_tbl[i].is_rx_done = true;

    spi_tbl[i].func_tx = NULL;
    spi_tbl[i].func_rx = NULL;
    spi_tbl[i].h_spi = NULL;
    spi_tbl[i].h_dma_rx = NULL;
    spi_tbl[i].h_dma_tx = NULL;
  }

#ifdef _USE_HW_CLI
  cliAdd("spi", cliSpi);
#endif

  return ret;
}

bool spiBegin(uint8_t ch)
{
  bool ret = false;
  spi_t *p_spi = &spi_tbl[ch];

  switch(ch)
  {
  	  //ili9488
  /*
   * SYSCLK 80 MHz, APB2 분주 1 이므로 SPI1 은 80 MHz / prescaler 다.
   * ILI9488 은 SPI 로 16bpp 를 받지 못해(데이터시트상 16-bit/pixel 은 병렬
   * 인터페이스 전용) 픽셀당 3바이트를 보내야 하므로 전송 시간이 길다. 그래서
   * 데이터시트 쓰기 사이클 하한(50 ns = 20 MHz)에 딱 맞는 4 를 쓴다.
   *
   * 배선이 길거나 화면에 노이즈/찌그러짐이 보이면 8(10 MHz)로 내리면 된다.
   */
    case _DEF_SPI1:
      p_spi->h_spi = &hspi1;
      p_spi->h_dma_rx = &hdma_spi1_rx;
      p_spi->h_dma_tx = &hdma_spi1_tx;

      hspi1.Instance                = SPI1;
      hspi1.Init.Mode               = SPI_MODE_MASTER;
      hspi1.Init.Direction          = SPI_DIRECTION_2LINES;
      hspi1.Init.DataSize           = SPI_DATASIZE_8BIT;
      hspi1.Init.CLKPolarity        = SPI_POLARITY_LOW;
      hspi1.Init.CLKPhase           = SPI_PHASE_1EDGE;
      hspi1.Init.NSS                = SPI_NSS_SOFT;
      hspi1.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_4;
      hspi1.Init.FirstBit           = SPI_FIRSTBIT_MSB;
      hspi1.Init.TIMode             = SPI_TIMODE_DISABLE;
      hspi1.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
      hspi1.Init.CRCPolynomial      = 7;
      hspi1.Init.CRCLength          = SPI_CRC_LENGTH_DATASIZE;
      hspi1.Init.NSSPMode           = SPI_NSS_PULSE_DISABLE;

      if (HAL_SPI_Init(&hspi1) == HAL_OK)
      {
        p_spi->is_open = true;
        ret = true;
      }
      break;

  }

  return ret;
}

void spiSetDataMode(uint8_t ch, uint8_t dataMode)
{
  spi_t  *p_spi = &spi_tbl[ch];


  if (p_spi->is_open == false) return;


  switch( dataMode )
  {
    // CPOL=0, CPHA=0
    case SPI_MODE0:
      p_spi->h_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
      p_spi->h_spi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      HAL_SPI_Init(p_spi->h_spi);
      break;

    // CPOL=0, CPHA=1
    case SPI_MODE1:
      p_spi->h_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
      p_spi->h_spi->Init.CLKPhase    = SPI_PHASE_2EDGE;
      HAL_SPI_Init(p_spi->h_spi);
      break;

    // CPOL=1, CPHA=0
    case SPI_MODE2:
      p_spi->h_spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
      p_spi->h_spi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      HAL_SPI_Init(p_spi->h_spi);
      break;

    // CPOL=1, CPHA=1
    case SPI_MODE3:
      p_spi->h_spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
      p_spi->h_spi->Init.CLKPhase    = SPI_PHASE_2EDGE;
      HAL_SPI_Init(p_spi->h_spi);
      break;
  }
}


void spiSetBitWidth(uint8_t ch, uint8_t bit_width)
{
  spi_t  *p_spi = &spi_tbl[ch];

  if (p_spi->is_open == false) return;

  p_spi->h_spi->Init.DataSize = SPI_DATASIZE_8BIT;

  if (bit_width == 16)
  {
    p_spi->h_spi->Init.DataSize = SPI_DATASIZE_16BIT;
  }
  HAL_SPI_Init(p_spi->h_spi);
}

/*
 * 전송 클럭을 바꾼다. divider 는 PCLK 를 나누는 값(2~256)이고, 목록에 없는 값은
 * 가장 느린 256 으로 떨어진다.
 *
 * 한 번 정하면 spiSetBitWidth() 나 spiSetDataMode() 가 HAL_SPI_Init() 을 다시
 * 불러도 핸들에 남아 그대로 유지된다.
 */
void spiSetClockDivider(uint8_t ch, uint16_t divider)
{
  spi_t  *p_spi = &spi_tbl[ch];
  uint32_t prescaler;

  if (p_spi->is_open == false) return;

  switch (divider)
  {
    case 2:    prescaler = SPI_BAUDRATEPRESCALER_2;   break;
    case 4:    prescaler = SPI_BAUDRATEPRESCALER_4;   break;
    case 8:    prescaler = SPI_BAUDRATEPRESCALER_8;   break;
    case 16:   prescaler = SPI_BAUDRATEPRESCALER_16;  break;
    case 32:   prescaler = SPI_BAUDRATEPRESCALER_32;  break;
    case 64:   prescaler = SPI_BAUDRATEPRESCALER_64;  break;
    case 128:  prescaler = SPI_BAUDRATEPRESCALER_128; break;
    default:   prescaler = SPI_BAUDRATEPRESCALER_256; break;
  }

  if (p_spi->h_spi->Init.BaudRatePrescaler == prescaler) return;

  p_spi->h_spi->Init.BaudRatePrescaler = prescaler;
  HAL_SPI_Init(p_spi->h_spi);
}

// 1bye씩
uint8_t spiTransfer8(uint8_t ch, uint8_t data)
{
  uint8_t ret;
  spi_t  *p_spi = &spi_tbl[ch];


  if (p_spi->is_open == false) return 0;

  HAL_SPI_TransmitReceive(p_spi->h_spi, &data, &ret, 1, 0xffff);

  return ret;
}

// 2bye씩
// 상위비트 먼저 처리하는 이유는
// 먼저 MSB부터 처리하는 방식이기 때문이다.
uint16_t spiTransfer16(uint8_t ch, uint16_t data)
{
  uint8_t tBuf[2];
  uint8_t rBuf[2];
  uint16_t ret;
  spi_t  *p_spi = &spi_tbl[ch];


  if (p_spi->is_open == false) return 0;

  if (p_spi->h_spi->Init.DataSize == SPI_DATASIZE_8BIT)
  {
    tBuf[1] = (uint8_t)data;
    tBuf[0] = (uint8_t)(data>>8);
    HAL_SPI_TransmitReceive(p_spi->h_spi, (uint8_t *)&tBuf, (uint8_t *)&rBuf, 2, 0xffff);

    ret = rBuf[0];
    ret <<= 8;
    ret += rBuf[1];
  }
  else
  {
    HAL_SPI_TransmitReceive(p_spi->h_spi, (uint8_t *)&data, (uint8_t *)&ret, 1, 0xffff);
  }

  return ret;
}

/*
 * 폴링 송신.
 *
 * ILI9488 초기화 명령과 짧은 레지스터 쓰기에 쓴다. 화면 픽셀은 이 경로가
 * 아니라 spiDmaTxStart 로 나간다.
 */
bool spiWrite(uint8_t ch, const uint8_t *p_data, uint32_t length, uint32_t timeout)
{
  spi_t *p_spi;

  if ((ch >= SPI_MAX_CH) || (p_data == NULL) || (length == 0U))
  {
    return false;
  }

  p_spi = &spi_tbl[ch];

  if (p_spi->is_open == false)
  {
    return false;
  }

  while (length > 0U)
  {
    uint16_t tx_length = (length > UINT16_MAX) ? UINT16_MAX : (uint16_t)length;

    if (HAL_SPI_Transmit(p_spi->h_spi, (uint8_t *)p_data, tx_length, timeout) != HAL_OK)
    {
      return false;
    }

    p_data += tx_length;
    length -= tx_length;
  }

  return true;
}

// DMA 송신 시작하는 함수 프레임 버퍼 데이터를 시작해라 false에서 시작하고 나면 Tx call back이 호출되면서 true로 바뀌게 됨.
void spiDmaTxStart(uint8_t spi_ch, uint8_t *p_buf, uint32_t length)
{
  spi_t  *p_spi = &spi_tbl[spi_ch];

  if (p_spi->is_open == false) return;

  p_spi->is_tx_done = false;
  HAL_SPI_Transmit_DMA(p_spi->h_spi, p_buf, length);
}

// timeout만큼 DMA전송이 완료될때까지 기다리고 되면 무시하고 return 함
void spiDmaTxTransfer(uint8_t ch, void *buf, uint32_t length, uint32_t timeout)
{
  uint32_t t_time;


  spiDmaTxStart(ch, (uint8_t *)buf, length);

  t_time = millis();

  if (timeout == 0) return;

  while(1)
  {
    if(spiDmaTxIsDone(ch))
    {
      break;
    }
    if((millis()-t_time) > timeout)
    {
      break;
    }
  }
}

bool spiDmaTxIsDone(uint8_t ch)
{
  spi_t  *p_spi = &spi_tbl[ch];

  if (p_spi->is_open == false)     return true;

  return p_spi->is_tx_done;
}

void spiAttachTxInterrupt(uint8_t ch, void (*func)())
{
  spi_t  *p_spi = &spi_tbl[ch];


  if (p_spi->is_open == false)     return;

  p_spi->func_tx = func;
}

void spiDmaRxStart(uint8_t spi_ch, uint8_t *p_buf, uint32_t length)
{
  spi_t  *p_spi = &spi_tbl[spi_ch];

  if (p_spi->is_open == false) return;

  p_spi->is_rx_done = false;
  HAL_SPI_Receive_DMA(p_spi->h_spi, p_buf, length);
}

void spiDmaRxTransfer(uint8_t ch, void *buf, uint32_t length, uint32_t timeout)
{
  uint32_t t_time;


  spiDmaRxStart(ch, (uint8_t *)buf, length);

  t_time = millis();

  if (timeout == 0) return;

  while(1)
  {
    if(spiDmaRxIsDone(ch))
    {
      break;
    }
    if((millis()-t_time) > timeout)
    {
      break;
    }
  }
}

bool spiDmaRxIsDone(uint8_t ch)
{
  spi_t  *p_spi = &spi_tbl[ch];

  if (p_spi->is_open == false)     return true;

  return p_spi->is_rx_done;
}

void spiAttachRxInterrupt(uint8_t ch, void (*func)())
{
  spi_t  *p_spi = &spi_tbl[ch];


  if (p_spi->is_open == false)     return;

  p_spi->func_rx = func;
}



void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)    //SPI 다 출력하고 나서 완료되면 interrupt 콜백함 is_tx_done이 true이면 완료되었구나를 알게됨
{
  spi_t  *p_spi;

  for (uint8_t i=0; i<SPI_MAX_CH; i++)
  {
    p_spi = &spi_tbl[i];

    if ((p_spi->is_open == true) &&
        (p_spi->h_spi != NULL) &&
        (hspi->Instance == p_spi->h_spi->Instance))
    {
      p_spi->is_tx_done = true;

      if (p_spi->func_tx != NULL)
      {
        (*p_spi->func_tx)();
      }

      break;
    }
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  spi_t  *p_spi;

  for (uint8_t i=0; i<SPI_MAX_CH; i++)
  {
    p_spi = &spi_tbl[i];

    if ((p_spi->is_open == true) &&
        (p_spi->h_spi != NULL) &&
        (hspi->Instance == p_spi->h_spi->Instance))
    {
      p_spi->is_rx_done = true;

      if (p_spi->func_rx != NULL)
      {
        (*p_spi->func_rx)();
      }

      break;
    }
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    if(spiHandle->Instance==SPI1)
      {
      /* USER CODE BEGIN SPI1_MspInit 0 */

      /* USER CODE END SPI1_MspInit 0 */
        /* SPI1 clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**SPI1 GPIO Configuration
        PA1     ------> SPI1_SCK
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* SPI1 DMA Init */
        /* SPI1_RX Init */
        hdma_spi1_rx.Instance = DMA1_Channel2;
        hdma_spi1_rx.Init.Request = DMA_REQUEST_1;
        hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi1_rx.Init.Mode = DMA_NORMAL;
        hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
        {
          Error_Handler();
        }

        __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

        /* SPI1_TX Init */
        hdma_spi1_tx.Instance = DMA1_Channel3;
        hdma_spi1_tx.Init.Request = DMA_REQUEST_1;
        hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi1_tx.Init.Mode = DMA_NORMAL;
        hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
        {
          Error_Handler();
        }

        __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

        /*
         * SPI1 DMA interrupt Init.
         *
         * 화면은 DMA 로만 나가고, 블록 하나가 다 나갔다는 통보는 이 DMA 전송
         * 완료 인터럽트에서 온다. 이걸 켜지 않으면 HAL 이 TxCpltCallback 을
         * 부르지 못해 첫 블록을 보낸 뒤 화면이 그대로 멎는다.
         */
        HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
        HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

        /* SPI1 interrupt Init */
        HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
      /* USER CODE BEGIN SPI1_MspInit 1 */

      /* USER CODE END SPI1_MspInit 1 */
      }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

	  if(spiHandle->Instance==SPI1)
	  {
	  /* USER CODE BEGIN SPI1_MspDeInit 0 */

	  /* USER CODE END SPI1_MspDeInit 0 */
	    /* Peripheral clock disable */
	    __HAL_RCC_SPI1_CLK_DISABLE();

	    /**SPI1 GPIO Configuration
	    PA1     ------> SPI1_SCK
	    PA7     ------> SPI1_MOSI
	    */
	    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_7);

	    /* SPI1 DMA DeInit */
	    HAL_DMA_DeInit(spiHandle->hdmarx);
	    HAL_DMA_DeInit(spiHandle->hdmatx);

	    /* SPI1 interrupt Deinit */
	    HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
	    HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
	    HAL_NVIC_DisableIRQ(SPI1_IRQn);
	  /* USER CODE BEGIN SPI1_MspDeInit 1 */

	  /* USER CODE END SPI1_MspDeInit 1 */
	  }
}

#ifdef _USE_HW_CLI

static void cliSpiInfo(void)
{
  spi_t *p_spi;

  for (uint8_t i = 0; i < SPI_MAX_CH; i++)
  {
    p_spi = &spi_tbl[i];

    cliPrintf("spi ch %d\n", i);
    cliPrintf("  open    : %d\n", p_spi->is_open);
    cliPrintf("  tx done : %d\n", p_spi->is_tx_done);
    cliPrintf("  rx done : %d\n", p_spi->is_rx_done);

    if (p_spi->h_spi != NULL)
    {
      cliPrintf("  datasize: %lu\n", (unsigned long)p_spi->h_spi->Init.DataSize);
      cliPrintf("  prescale: %lu\n", (unsigned long)p_spi->h_spi->Init.BaudRatePrescaler);
    }
  }
}

static void cliSpi(cli_args_t *args)
{
  bool ret = false;

  if (args->argc == 1 && args->isStr(0, "info") == true)
  {
    cliSpiInfo();
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "open") == true)
  {
    uint8_t ch;

    ch = (uint8_t)args->getData(1);
    if (ch < SPI_MAX_CH)
    {
      cliPrintf("spi open %d : %d\n", ch, spiBegin(ch));
    }
    else
    {
      cliPrintf("spi open %d fail\n", ch);
    }

    ret = true;
  }

  if (args->argc == 3 && args->isStr(0, "mode") == true)
  {
    uint8_t ch;
    uint8_t mode;

    ch = (uint8_t)args->getData(1);
    mode = (uint8_t)args->getData(2);

    if ((ch < SPI_MAX_CH) && (mode <= SPI_MODE3))
    {
      spiSetDataMode(ch, mode);
      cliPrintf("spi mode %d %d\n", ch, mode);
    }
    else
    {
      cliPrintf("spi mode fail\n");
    }

    ret = true;
  }

  if (args->argc == 3 && args->isStr(0, "width") == true)
  {
    uint8_t ch;
    uint8_t width;

    ch = (uint8_t)args->getData(1);
    width = (uint8_t)args->getData(2);

    if ((ch < SPI_MAX_CH) && ((width == 8U) || (width == 16U)))
    {
      spiSetBitWidth(ch, width);
      cliPrintf("spi width %d %d\n", ch, width);
    }
    else
    {
      cliPrintf("spi width fail\n");
    }

    ret = true;
  }

  if (args->argc == 3 && args->isStr(0, "tx") == true)
  {
    uint8_t ch;
    uint8_t tx_data;
    uint8_t rx_data;

    ch = (uint8_t)args->getData(1);
    tx_data = (uint8_t)args->getData(2);

    if (ch < SPI_MAX_CH)
    {
      rx_data = spiTransfer8(ch, tx_data);
      cliPrintf("spi tx ch=%d tx=0x%02X rx=0x%02X\n",
                ch,
                (unsigned int)tx_data,
                (unsigned int)rx_data);
    }
    else
    {
      cliPrintf("spi tx fail\n");
    }

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("spi info\n");
    cliPrintf("spi open ch[0~%d]\n", SPI_MAX_CH - 1);
    cliPrintf("spi mode ch[0~%d] mode[0~3]\n", SPI_MAX_CH - 1);
    cliPrintf("spi width ch[0~%d] 8:16\n", SPI_MAX_CH - 1);
    cliPrintf("spi tx ch[0~%d] data\n", SPI_MAX_CH - 1);
  }
}

#endif

#endif
