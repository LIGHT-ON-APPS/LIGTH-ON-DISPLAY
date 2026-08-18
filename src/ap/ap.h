/*
 * ap.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_AP_AP_H_
#define SRC_AP_AP_H_
/*
 * 이 헤더 아래로 딸려 오는 드라이버 선언은 전부 C 링키지여야 한다.
 * TouchGFX 쪽(C++)이 main.h -> ap.h -> hw.h 순으로 이 트리를 먼저 끌어가는데,
 * 그때 링키지를 잡아 두지 않으면 lcdInit 같은 심볼이 C++ 이름으로 참조되어
 * 링크에서 어긋난다.
 */
#ifdef __cplusplus
extern "C" {
#endif


#include "hw.h"


void apInit(void);
void apMain(void);


#ifdef __cplusplus
}
#endif

#endif /* SRC_AP_AP_H_ */
