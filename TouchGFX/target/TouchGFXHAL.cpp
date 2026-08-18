/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : TouchGFXHAL.cpp
  ******************************************************************************
  * This file was created by TouchGFX Generator 4.22.0. This file is only
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
/* USER CODE END Header */

#include <TouchGFXHAL.hpp>

/* USER CODE BEGIN TouchGFXHAL.cpp */
#include "stm32l4xx_hal.h"
#include "main.h"
#include <touchgfx/hal/GPIO.hpp>
extern "C"
{
#include "lcd.h"
void DisplayDriver_TransferCompleteCallback(void);
}

/*
 * 부분 프레임버퍼 블록 전송.
 *
 * 예전에는 여기서 SPI 로 다 밀어 넣고 돌아온 뒤 완료 콜백을 직접 불렀다. 그러면
 * 한 블록을 그리는 동안 SPI 가 놀고, SPI 가 도는 동안 CPU 가 노는 식으로 두 시간이
 * 직렬로 더해져 갱신이 눈에 띄게 끊긴다. ILI9488 은 SPI 로 16bpp 를 받지 못해
 * 픽셀당 3바이트를 보내야 하므로 전송 시간이 특히 길다.
 *
 * 이제 전송은 DMA 로 시작만 하고 돌아온다. 완료는 SPI TX 인터럽트가 알려 주고,
 * 그동안 TouchGFX 는 다음 블록을 그린다.
 */
/*
 * 프레임 시간이 어디로 가는지 재는 계측.
 *
 * 블록 하나를 화면에 올리는 데 드는 시간은 "TouchGFX 가 그리는 시간"과 "SPI 가
 * 밀어내는 시간"으로 나뉜다. 둘 중 어느 쪽이 프레임을 잡아먹는지 모르면 최적화가
 * 전부 추측이 되므로, DWT 사이클 카운터로 나눠서 누적한다.
 *
 * tgfxPerfTxCycles  : DMA 가 SPI 를 물고 있던 시간
 * tgfxPerfDrawCycles: 그 사이에 CPU 가 다음 블록을 그린 시간
 * 두 값의 비가 곧 병목이 어디인지에 대한 답이다.
 */
extern "C"
{
volatile uint32_t tgfxPerfTxCycles = 0U;
volatile uint32_t tgfxPerfDrawCycles = 0U;
volatile uint32_t tgfxPerfBlocks = 0U;
}

static volatile uint32_t perfTxStart = 0U;
static volatile uint32_t perfLastTxDone = 0U;

static void perfInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* SPI TX 완료 인터럽트에서 불린다. 여기서 프레임워크에 다음 블록을 요청한다. */
static void onDisplayTransferDone(void)
{
    const uint32_t now = DWT->CYCCNT;

    tgfxPerfTxCycles += now - perfTxStart;
    tgfxPerfBlocks++;
    perfLastTxDone = now;

    DisplayDriver_TransferCompleteCallback();
}

extern "C" int touchgfxDisplayDriverTransmitActive(void)
{
    return lcdIsTxBusy() ? 1 : 0;
}

extern "C" int touchgfxDisplayDriverShouldTransferBlock(uint16_t bottom)
{
    (void)bottom;
    return 1;
}

extern "C" void touchgfxDisplayDriverTransmitBlock(const uint8_t* pixels, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    bool transferOk = false;
    const uint32_t now = DWT->CYCCNT;

    /* 앞 블록 전송이 끝난 뒤 여기까지 온 시간이 곧 그리기에 쓴 시간이다. */
    tgfxPerfDrawCycles += now - perfLastTxDone;
    perfTxStart = now;

    if ((pixels != 0) && (w > 0U) && (h > 0U) && lcdIsInit())
    {
        transferOk = lcdSetWindow(x, y, w, h) &&
                     lcdSendBuffer(const_cast<uint8_t*>(pixels),
                                   (uint32_t)w * (uint32_t)h, 0);
    }

    if (!transferOk)
    {
        Error_Handler();
    }

    /* 완료 통보는 lcdSetTransferDoneCallBack 으로 등록한 쪽에서 온다. */
}
using namespace touchgfx;

/* ******************************************************
 * Functions required by Partial Frame Buffer Strategy
 * ******************************************************
 *
 *  *  uint8_t IsTransmittingBlock() must return whether or not data is currently being transmitted, over e.g. SPI.
 *  *  void TransmitBlock(uint8_t* pixels, uint16_t x, uint16_t y, uint16_t w, uint16_t h) will be called
 *       when the framework wants to send a block. The user must then transfer
 *       the data represented by the arguments.
 *
 *  A user must call touchgfx::startNewTransfer(); once TransmitBlock() has succesfully sent a block.
 *  E.g. if using DMA to transfer the block, this could be called in the "Transfer Completed" interrupt handler.
 *
 * Remove the following preprocessor directives after defining the required functions
 *
 */

void TouchGFXHAL::initialize()
{
  // Calling parent implementation of initialize().
  //
  // To overwrite the generated implementation, omit call to parent function
  // and implemented needed functionality here.
  // Please note, HAL::initialize() must be called to initialize the framework.

  TouchGFXGeneratedHAL::initialize();

  if (!lcdIsInit())
  {
      if (!lcdInit())
      {
          Error_Handler();
      }
  }

  perfInit();

  /* 블록 전송 완료(SPI DMA TX 인터럽트)를 프레임워크에 연결한다. 이게 없으면
   * 첫 블록을 보낸 뒤 완료 통보가 오지 않아 화면이 그대로 멎는다. */
  lcdSetTransferDoneCallBack(onDisplayTransferDone);
}

/**
 * Gets the frame buffer address used by the TFT controller.
 *
 * @return The address of the frame buffer currently being displayed on the TFT.
 */
uint16_t* TouchGFXHAL::getTFTFrameBuffer() const
{
    // Calling parent implementation of getTFTFrameBuffer().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    return TouchGFXGeneratedHAL::getTFTFrameBuffer();
}

/**
 * Sets the frame buffer address used by the TFT controller.
 *
 * @param [in] address New frame buffer address.
 */
void TouchGFXHAL::setTFTFrameBuffer(uint16_t* address)
{
    // Calling parent implementation of setTFTFrameBuffer(uint16_t* address).
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::setTFTFrameBuffer(address);
}

/**
 * This function is called whenever the framework has performed a partial draw.
 *
 * @param rect The area of the screen that has been drawn, expressed in absolute coordinates.
 *
 * @see flushFrameBuffer().
 */
void TouchGFXHAL::flushFrameBuffer(const touchgfx::Rect& rect)
{
    // Calling parent implementation of flushFrameBuffer(const touchgfx::Rect& rect).
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.
    // Please note, HAL::flushFrameBuffer(const touchgfx::Rect& rect) must
    // be called to notify the touchgfx framework that flush has been performed.

    TouchGFXGeneratedHAL::flushFrameBuffer(rect);
}

bool TouchGFXHAL::beginFrame()
{
    return TouchGFXGeneratedHAL::beginFrame();
}

void TouchGFXHAL::endFrame()
{
    TouchGFXGeneratedHAL::endFrame();
}

bool TouchGFXHAL::blockCopy(void* RESTRICT dest, const void* RESTRICT src, uint32_t numBytes)
{
    return TouchGFXGeneratedHAL::blockCopy(dest, src, numBytes);
}

/**
 * Configures the interrupts relevant for TouchGFX. This primarily entails setting
 * the interrupt priorities for the DMA and LCD interrupts.
 */
void TouchGFXHAL::configureInterrupts()
{
    // Calling parent implementation of configureInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::configureInterrupts();
}

/**
 * Used for enabling interrupts set in configureInterrupts()
 */
void TouchGFXHAL::enableInterrupts()
{
    // Calling parent implementation of enableInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::enableInterrupts();
}

/**
 * Used for disabling interrupts set in configureInterrupts()
 */
void TouchGFXHAL::disableInterrupts()
{
    // Calling parent implementation of disableInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::disableInterrupts();
}

/**
 * Configure the LCD controller to fire interrupts at VSYNC. Called automatically
 * once TouchGFX initialization has completed.
 */
void TouchGFXHAL::enableLCDControllerInterrupt()
{
    // Calling parent implementation of enableLCDControllerInterrupt().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::enableLCDControllerInterrupt();
}

/* USER CODE END TouchGFXHAL.cpp */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
