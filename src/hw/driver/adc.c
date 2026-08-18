/*
 * adc.c
 *
 * ADC driver
 */

#include "adc.h"

#ifdef _USE_HW_ADC

#ifdef _USE_HW_CLI
#include "cli.h"
#endif

#define ADC_12BIT_FULL_SCALE         4095U
#define ADC_REFERENCE_MV             3300U
#define ADC_REFERENCE_VOLTAGE        3.3f
#define ADC_SAMPLE_HZ                1000U
#define ADC_TIM_BASE_HZ              1000000U


typedef struct
{
  bool     is_open;
  volatile uint16_t data;
  uint32_t channel;
} adc_t;


#ifdef _USE_HW_CLI
static void cliAdc(cli_args_t *args);
static void cliAdcPrintChannel(uint8_t ch);
#endif

static bool adcOpen(void);
static bool adcConfigChannels(void);
static bool adcTim6Init(void);
static uint32_t adcTim6GetClockHz(void);


static ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
static TIM_HandleTypeDef htim6;

static adc_update_callback_t adc_update_callback = NULL;
static uint16_t adc_dma_buf[ADC_MAX_CH];
static volatile bool adc_sample_ready = false;

static adc_t adc_tbl[ADC_MAX_CH] =
{
  {false, 0, ADC_CHANNEL_5},  /* ADC_CH_0: PA0 / ADC1_IN5 */
  {false, 0, ADC_CHANNEL_6},  /* ADC_CH_1: PA1 / ADC1_IN6 */
};


bool adcInit(void)
{
  bool ret;

  for (int i = 0; i < ADC_MAX_CH; i++)
  {
    adc_tbl[i].is_open = false;
    adc_tbl[i].data = 0;
    adc_dma_buf[i] = 0;
  }
  adc_sample_ready = false;

  ret = adcOpen();

#ifdef _USE_HW_CLI
  cliAdd("adc", cliAdc);
#endif

  return ret;
}

static bool adcOpen(void)
{
  bool ret = true;

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.NbrOfConversion       = ADC_MAX_CH;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion   = 1;
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode      = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    ret = false;
  }

  if (ret == true)
  {
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
      ret = false;
    }
  }

  if (ret == true)
  {
    ret = adcConfigChannels();
  }

  if (ret == true)
  {
    ret = adcTim6Init();
  }

  if (ret == true)
  {
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_MAX_CH) != HAL_OK)
    {
      ret = false;
    }
  }

  if (ret == true)
  {
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
    {
      ret = false;
    }
  }

  if (ret == true)
  {
    for (int i = 0; i < ADC_MAX_CH; i++)
    {
      adc_tbl[i].is_open = true;
    }
  }

  return ret;
}

bool adcIsInit(void)
{
  for (int i = 0; i < ADC_MAX_CH; i++)
  {
    if (adc_tbl[i].is_open != true)
    {
      return false;
    }
  }

  return true;
}

static bool adcConfigChannels(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel      = adc_tbl[0].channel;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return false;
  }

  sConfig.Channel = adc_tbl[1].channel;
  sConfig.Rank    = ADC_REGULAR_RANK_2;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return false;
  }

  return true;
}

static uint32_t adcTim6GetClockHz(void)
{
  RCC_ClkInitTypeDef clk_init;
  uint32_t flash_latency;
  uint32_t tim_clk;

  tim_clk = HAL_RCC_GetPCLK1Freq();
  HAL_RCC_GetClockConfig(&clk_init, &flash_latency);

  if (clk_init.APB1CLKDivider != RCC_HCLK_DIV1)
  {
    tim_clk *= 2U;
  }

  return tim_clk;
}

static bool adcTim6Init(void)
{
  TIM_MasterConfigTypeDef master_config = {0};
  uint32_t tim_clk;
  uint32_t prescaler_div;
  uint32_t tick_hz;
  uint32_t period_tick;

  tim_clk = adcTim6GetClockHz();
  if (tim_clk == 0U)
  {
    return false;
  }

  prescaler_div = (tim_clk + ADC_TIM_BASE_HZ - 1U) / ADC_TIM_BASE_HZ;
  if (prescaler_div == 0U)
  {
    prescaler_div = 1U;
  }

  tick_hz = tim_clk / prescaler_div;
  period_tick = tick_hz / ADC_SAMPLE_HZ;
  if (period_tick == 0U)
  {
    period_tick = 1U;
  }

  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = prescaler_div - 1U;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = period_tick - 1U;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    return false;
  }

  master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
  master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &master_config) != HAL_OK)
  {
    return false;
  }

  return true;
}

bool adcUpdate(void)
{
  if (adcIsInit() != true)
  {
    return false;
  }

  if (adc_sample_ready != true)
  {
    return false;
  }

  return true;
}

void adcSetUpdateCallback(adc_update_callback_t callback)
{
  adc_update_callback = callback;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *p_hadc)
{
  if (p_hadc->Instance != ADC1)
  {
    return;
  }

  for (int i = 0; i < ADC_MAX_CH; i++)
  {
    adc_tbl[i].data = adc_dma_buf[i];
  }

  adc_sample_ready = true;

  if (adc_update_callback != NULL)
  {
    adc_update_callback();
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *p_hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if (p_hadc->Instance == ADC1)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(p_hadc, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *p_hadc)
{
  if (p_hadc->Instance == ADC1)
  {
    (void)HAL_TIM_Base_Stop(&htim6);

    __HAL_RCC_ADC_CLK_DISABLE();
    __HAL_RCC_TIM6_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);

    HAL_DMA_DeInit(p_hadc->DMA_Handle);
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  }
}

int32_t adcRead(uint8_t ch)
{
  if ((ch >= ADC_MAX_CH) || (adc_tbl[ch].is_open != true))
  {
    return 0;
  }

  return adc_tbl[ch].data;
}

int32_t adcRead8(uint8_t ch)
{
  return adcRead(ch) >> 4;
}

int32_t adcRead10(uint8_t ch)
{
  return adcRead(ch) >> 2;
}

int32_t adcRead12(uint8_t ch)
{
  return adcRead(ch);
}

int32_t adcRead16(uint8_t ch)
{
  return adcRead(ch) << 4;
}

uint8_t adcGetRes(uint8_t ch)
{
  (void)ch;

  return 12;
}

float adcReadVoltage(uint8_t ch)
{
  return adcConvVoltage(ch, (uint32_t)adcRead(ch));
}

float adcConvVoltage(uint8_t ch, uint32_t adc_value)
{
  (void)ch;

  if (adc_value > ADC_12BIT_FULL_SCALE)
  {
    adc_value = ADC_12BIT_FULL_SCALE;
  }

  return ((float)adc_value * ADC_REFERENCE_VOLTAGE) /
         (float)ADC_12BIT_FULL_SCALE;
}

uint32_t adcReadMillivolts(uint8_t ch)
{
  return adcConvMillivolts(ch, (uint32_t)adcRead(ch));
}

uint32_t adcConvMillivolts(uint8_t ch, uint32_t adc_value)
{
  (void)ch;

  if (adc_value > ADC_12BIT_FULL_SCALE)
  {
    adc_value = ADC_12BIT_FULL_SCALE;
  }

  return ((adc_value * ADC_REFERENCE_MV) + (ADC_12BIT_FULL_SCALE / 2U)) / ADC_12BIT_FULL_SCALE;
}


#ifdef _USE_HW_CLI

static void cliAdcPrintChannel(uint8_t ch)
{
  uint32_t raw;
  uint32_t voltage_mv;

  raw = (uint32_t)adcRead(ch);
  voltage_mv = adcConvMillivolts(ch, raw);

  cliPrintf("%02d. raw=%4d, voltage=%d.%03dV (%dmV)\n",
            ch,
            (int)raw,
            (int)(voltage_mv / 1000U),
            (int)(voltage_mv % 1000U),
            (int)voltage_mv);
}

static void cliAdc(cli_args_t *args)
{
  bool ret = false;

  if (args->argc == 1 && args->isStr(0, "info") == true)
  {
    cliPrintf("adc init : %d\n", adcIsInit());
    cliPrintf("adc res  : %d\n", adcGetRes(0));

    if (adcUpdate() == true)
    {
      for (int i = 0; i < ADC_MAX_CH; i++)
      {
        cliAdcPrintChannel((uint8_t)i);
      }
    }
    else
    {
      cliPrintf("adc read fail\n");
    }

    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "show") == true)
  {
    while (cliKeepLoop())
    {
      if (adcUpdate() == true)
      {
        for (int i = 0; i < ADC_MAX_CH; i++)
        {
          cliAdcPrintChannel((uint8_t)i);
        }
      }
      else
      {
        cliPrintf("adc read fail\n");
      }

      delay(100);
    }

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("adc info\n");
    cliPrintf("adc show\n");
  }
}

#endif

#endif
