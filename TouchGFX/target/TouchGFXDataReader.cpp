/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : TouchGFXDataReader.cpp
  ******************************************************************************
  * This file was created by TouchGFX Generator 4.23.2. This file is only
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

#include <TouchGFXDataReader.hpp>
#include <cstring>

/* USER CODE BEGIN TouchGFXDataReader.cpp */

bool TouchGFXDataReader::addressIsAddressable(const void* address)
{
    // Calling parent implementation of addressIsAddressable().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    return TouchGFXGeneratedDataReader::addressIsAddressable(address);
}

void TouchGFXDataReader::copyData(const void* src, void* dst, uint32_t bytes)
{
    /*
     * Generated UnmappedDataFont objects use this method even when their
     * glyph tables are linked into the MCU's internal flash. The generated
     * parent implementation always routes copy requests through the external
     * flash hooks, which are intentionally unused on this board.
     */
    if (TouchGFXGeneratedDataReader::addressIsAddressable(src))
    {
        std::memcpy(dst, src, bytes);
        return;
    }

    TouchGFXGeneratedDataReader::copyData(src, dst, bytes);
}

void TouchGFXDataReader::startFlashLineRead(const void* src, uint32_t bytes)
{
    // Calling parent implementation of startFlashLineRead().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedDataReader::startFlashLineRead(src, bytes);
}

const uint8_t* TouchGFXDataReader::waitFlashReadComplete()
{
    // Calling parent implementation of waitFlashReadComplete().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    return TouchGFXGeneratedDataReader::waitFlashReadComplete();
}

extern "C" {
    /* ******************************************************
     * Functions required by DataReader Strategy
     * ******************************************************
     *
     *  *  uint8_t isTransmittingData() must return whether or not data is currently being transmitted, over e.g. SPI.
     *  *  void transmitFrameBufferBlock(uint8_t* pixels, uint16_t x, uint16_t y, uint16_t w, uint16_t h) will be called
     *       when the framework wants to send a block. The user must then transfer
     *       the data represented by the arguments.
     *
     *  A user must call touchgfx::startNewTransfer(); once transmitFrameBufferBlock() has succesfully sent a block.
     *  E.g. if using DMA to transfer the block, this could be called in the "Transfer Completed" interrupt handler.
     *
     * Remove the preprocessor warning directives in each __attribute__((weak)) declared function after implementation.
     *
     */

    __attribute__((weak)) void DataReader_WaitForReceiveDone()
    {
      // #error "weak defined implementation for void DataReader_WaitForReveiceDone().
      // Implement functionality here or create void DataReader_WaitForReveiceDone() togeher with DataReader driver code.

//        // Wait for SPI2 DMA to complete
//        while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
//            // Optional: Add timeout for safety
//            // uint32_t tickstart = HAL_GetTick();
//            // while (HAL_GetTick() - tickstart < 1000) { /* 1s timeout */ }
//        }
//        HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);  // Deselect flash
    }

    __attribute__((weak)) void DataReader_ReadData(uint32_t address24, uint8_t* buffer, uint32_t length)
    {
      // #error "weak defined implementation for void DataReader_ReadData(uint32_t address24, uint8_t* buffer, uint32_t length).
      // Implement functionality here or create void DataReader_ReadData(uint32_t address24, uint8_t* buffer, uint32_t length) togeher with DataReader driver code.

//        // Translate virtual address (0x90000000+) to physical flash address
//        uint32_t flash_addr = address24 - 0x90000000;
//        uint8_t cmd[4] = {0x03, (flash_addr >> 16) & 0xFF, (flash_addr >> 8) & 0xFF, flash_addr & 0xFF};
//
//        HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);  // Select flash
//        HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);  // Send read command and address
//        HAL_SPI_Receive(&hspi2, buffer, length, HAL_MAX_DELAY);  // Receive data
//        HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);  // Deselect flash
    }

    __attribute__((weak)) void DataReader_StartDMAReadData(uint32_t address24, uint8_t* buffer, uint32_t length)
    {
      // #error "weak defined implementation for void DataReader_StartDMAReadData(uint32_t address24, uint8_t* buffer, uint32_t length).
      // Implement functionality here or create void DataReader_StartDMAReadData(uint32_t address24, uint8_t* buffer, uint32_t length) togeher with DataReader driver code.

//        // Translate virtual address to physical
//        uint32_t flash_addr = address24 - 0x90000000;
//        uint8_t cmd[4] = {0x03, (flash_addr >> 16) & 0xFF, (flash_addr >> 8) & 0xFF, flash_addr & 0xFF};
//
//        HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);  // Select flash
//        HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);  // Send command/address (blocking)
//        HAL_SPI_Receive_DMA(&hspi2, buffer, length);  // Start DMA for data
    }
}

/* USER CODE END TouchGFXDataReader.cpp */
