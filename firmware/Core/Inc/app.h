/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层：串口协议 + 50Hz 控制节拍 + 舵机随动
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 固件版本 */
#define APP_VERSION_MAJOR  0U
#define APP_VERSION_MINOR  1U

void App_Init(void);
void App_Loop(void);
void App_Tick50Hz(void);
void App_OnRxByte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
