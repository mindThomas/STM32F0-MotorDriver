/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */ 

#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#include "stm32f0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOOTLOADER_MAGIC_ADDR ((uint32_t*) ((uint32_t) 0x2001FFF0))
#define BOOTLOADER_MAGIC_TOKEN 0xDEADBEEF

//Value taken from CD00167594.pdf page 35, system memory start.
#define BOOTLOADER_START_ADDR 0x1fffc400 //for ST32F042

#define A_VALUE 0x12345678

void MainTask(void * pvParameters);
void Enter_DFU_Bootloader(void);

#ifdef __cplusplus
}

//#include "LSPC.hpp"
#include <vector>
void Reboot_Callback(void * param, const std::vector<uint8_t>& payload);
void EnterBootloader_Callback(void * param, const std::vector<uint8_t>& payload);

#endif

#endif 
