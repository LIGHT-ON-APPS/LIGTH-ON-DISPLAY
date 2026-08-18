/**
  ******************************************************************************
  * File Name          : app_touchgfx.c
  ******************************************************************************
  * This file was created by TouchGFX Generator 4.26.0. This file is only
  * generated once! Delete this file from your project and re-generate code
  * using STM32CubeMX or change this file manually to update it.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
/* The view/model timing is expressed in 60 Hz ticks. A 50 ms software VSYNC
 * made the decimated demo path visibly step; 17 ms supplies a 59 Hz base tick
 * while the model limits dirty-region redraws to a sustainable rate. */
#define TOUCHGFX_FRAME_INTERVAL_MS  17U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t touchgfx_last_vsync_ms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void touchgfx_init(void);
void touchgfx_components_init(void);
void touchgfx_taskEntry(void);
void touchgfxSignalVSync(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
 * Initialize TouchGFX application
 */
void MX_TouchGFX_Init(void)
{
    // Calling forward to touchgfx_init in C++ domain
    touchgfx_components_init();
    touchgfx_init();
}

/**
 * TouchGFX application entry function
 */
void MX_TouchGFX_Process(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - touchgfx_last_vsync_ms) >= TOUCHGFX_FRAME_INTERVAL_MS)
    {
        touchgfx_last_vsync_ms = now;
        touchgfxSignalVSync();
    }

    // Calling forward to touchgfx_taskEntry in C++ domain
    touchgfx_taskEntry();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
