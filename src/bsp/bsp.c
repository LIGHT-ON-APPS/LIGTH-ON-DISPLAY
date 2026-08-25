/*
 * bsp.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "bsp.h"
#include "uart.h"



void SystemClock_Config(void);

static bool is_delay_us_init = false;

static uint32_t delayUsGetTickPerUs(void)
{
  uint32_t tick_per_us;

  tick_per_us = SystemCoreClock / 1000000U;

  if(tick_per_us == 0)
  {
    tick_per_us = 1;
  }

  return tick_per_us;
}


void bspInit(void)
{
  HAL_Init();
  SystemClock_Config();
  delayUsInit();

  /*
   * TouchGFX 는 CRC 주변장치가 켜져 있는지 확인하고, 꺼져 있으면 정상 동작을
   * 거부한다. Application 생성자가 내부에서 CRC 를 검사한 뒤 실패 경로로 빠지는데,
   * 그 경로가 아직 만들어지지도 않은 Screen 의 가상 함수를 부르기 때문에 첫 화면을
   * 그리기 전에 vtable 이 0 인 곳으로 점프해 HardFault 로 죽는다. 화면은 하얗게만
   * 남는다.
   *
   * HAL 의 CRC 드라이버는 필요 없고 클럭만 켜 주면 된다.
   */
  __HAL_RCC_CRC_CLK_ENABLE();

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void delay(uint32_t ms)
{
#ifdef _USE_HW_RTOS
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    osDelay(ms);
  }
  else
  {
    HAL_Delay(ms);
  }
#else
  HAL_Delay(ms);
#endif
}

bool delayUsInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  is_delay_us_init = (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) ? true : false;

  return is_delay_us_init;
}

void delayUs(uint32_t us)
{
  uint32_t tick_per_us;

  if(us == 0)
  {
    return;
  }

  if(is_delay_us_init != true)
  {
    if(delayUsInit() != true)
    {
      return;
    }
  }

  tick_per_us = delayUsGetTickPerUs();

  while(us > 0)
  {
    uint32_t max_us;
    uint32_t wait_us;
    uint32_t wait_tick;
    uint32_t start_tick;

    max_us = 0xFFFFFFFFU / tick_per_us;
    wait_us = us;

    if(wait_us > max_us)
    {
      wait_us = max_us;
    }

    wait_tick = wait_us * tick_per_us;
    start_tick = DWT->CYCCNT;

    while((DWT->CYCCNT - start_tick) < wait_tick)
    {
    }

    us -= wait_us;
  }
}

uint32_t millis(void)
{
  return HAL_GetTick();
}

uint32_t micros(void)
{
  uint32_t tick_per_us;

  if(is_delay_us_init != true)
  {
    if(delayUsInit() != true)
    {
      return 0;
    }
  }

  tick_per_us = delayUsGetTickPerUs();

  return DWT->CYCCNT / tick_per_us;
}

int __io_putchar(int ch)
{
  //uartWrite(_DEF_UART2, (uint8_t *)&ch, 1);
  return 1;
}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /*
   * 여기서 멈춘다.
   *
   * 예전에는 본문이 비어 있어 그냥 리턴했다. SystemClock_Config() 에서 LSE 를
   * 못 잡아 HAL_RCC_OscConfig() 가 타임아웃으로 실패해도 그대로 지나가고,
   * PLL 이 안 켜진 채 HAL_RCC_ClockConfig(PLLCLK) 까지 실패한 뒤 SYSCLK 가
   * MSI 4 MHz 로 남은 상태로 부팅이 이어졌다. 그 상태에서 CAN 은 500 kbps 가
   * 아니라 25 kbps 로 돌아 한 프레임도 못 받는다. 화면은 느릴 뿐 그럭저럭
   * 그려지므로, 눈으로는 클럭이 틀어진 줄 알 수가 없다.
   *
   * 절름발이로 조용히 도는 것보다 여기서 멈추는 편이 낫다. 디버거를 붙이면
   * 콜스택에 원인이 그대로 남고, 못 붙이는 상황이면 "화면이 아예 안 뜬다" 는
   * 증상으로 드러난다.
   */
  __disable_irq();

  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
