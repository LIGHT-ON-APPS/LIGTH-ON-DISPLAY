/*
 * hw_def.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 *
 *      하드웨어가 공통으로 쓸 관련된 정의들
 */

#ifndef SRC_HW_HW_DEF_H_
#define SRC_HW_HW_DEF_H_

#include "def.h"
#include "bsp.h"

#define _USE_HW_UART
#define      HW_UART_MAX_CH         1
/*
 * 채널 번호와 실제 USART 의 대응.
 *   _USE_UART1 : _DEF_UART1 = USART2 (PA2 / PA15). ST-LINK VCP 로 나가며 CLI 가 쓴다.
 *   _USE_UART2 : _DEF_UART2 = USART1 (PA9 TX). 이 보드에서는 쓰지 않는다.
 * 인터럽트 핸들러(stm32l4xx_it.c)도 이 매크로를 보고 갈리므로 uart.c 안이 아니라
 * 여기에 둔다.
 */
#define _USE_UART1

#define _USE_HW_CLI
#define      HW_CLI_CMD_LIST_MAX    16
#define      HW_CLI_CMD_NAME_MAX    16
#define      HW_CLI_LINE_HIS_MAX    4
#define      HW_CLI_LINE_BUF_MAX    64

#define	_USE_HW_BUTTON
#define		 HW_BUTTON_MAX_CH		1

#define	_USE_HW_GPIO
#define		 HW_GPIO_MAX_CH			4

//#define _USE_HW_ADC
//#define      HW_ADC_MAX_CH          2

#define _USE_HW_CAN
#define      HW_CAN_FD              0
#define      HW_CAN_MAX_CH          1
#define      HW_CAN_MSG_RX_BUF_MAX  32

#define	_USE_HW_SPI
#define HW_SPI_MAX_CH				1

#define _USE_HW_LCD
#define _USE_HW_ILI9488
#define		HW_LCD_WIDTH			480
#define 	HW_LCD_HEIGHT        	320

#define _USE_HW_TOUCHGFX
/*
 * 화면에 넣을 값의 출처.
 *   0 : 차량 CAN 에서 실제로 디코딩한 값 (Orion BMS / Bamocar). 평소 이 값.
 *   1 : 내부에서 만들어 내는 애니메이션 값. CAN 을 물리지 않고 화면과 배선만
 *       확인할 때 쓴다. 이때는 display_link 없이도 화면이 돈다.
 */
#ifndef HW_USE_TOUCHGFX_DEMO
#define      HW_USE_TOUCHGFX_DEMO    0
#endif

#endif /* SRC_HW_HW_DEF_H_ */
