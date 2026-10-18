#ifndef TRANSMIT_H
#define TRANSMIT_H

#define SLAVE_NUMBER 10 // 最大从机数

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "ethercat.h"
#include "shared_types.h"
#include "sys/time.h"

// 只将 C 风格的函数声明放在 extern "C" 中
#ifdef __cplusplus
extern "C" {
#endif

// 函数声明 (仅 C 风格的接口)
void EtherCAT_Transmit(EtherCAT_Msg *MasterCommand); 
void EtherCAT_Init(char *ifname); 
void EtherCAT_Run(); 
void EtherCAT_Command_Set(); 
void startRun(); 

#ifdef __cplusplus
}
#endif

#endif // TRANSMIT_H
