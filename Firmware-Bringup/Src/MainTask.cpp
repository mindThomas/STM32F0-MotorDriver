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

#include "MainTask.h"
#include "cmsis_os.h"
#include "Priorities.h"
#include "stm32f0xx_hal_timebase_tim.h" // for HAL_GetHighResTick()

/* Include Periphiral drivers */
#include "IO.h"
#include "PWM.h"
#include "Timer.h"
#include "Encoder.h"
#include "ADC.h"
#include "UART.h"
#include "I2C.h"
#include "USBCDC.h"
#include "CANBus.h"
#include "SPI.h"

/* Include Drivers */
#include "QuadratureKnob.h"

/* Include Module libraries */
#include "Debug.h"

/* Include Application-layer libraries */

/* Miscellaneous includes */
#include <stdlib.h>
#include <string.h>
#include <vector>

void ButtonInterrupt(void * param);
void ReadAnalog(ADC::measurement_t * ADC_Samples, ADC * CS, ADC * VNH_CS, ADC * VBAT, ADC * VBUS);
void jetColor(const float v, const float vmin, const float vmax, float RGB[3]);

static ADC::measurement_t ADC_Samples[4];

void CAN_MotorPackage(void * param, const CANBus::package_t &package);
typedef struct MotorStruct {
	uint32_t Timestamp;
	uint16_t PWM;
	uint8_t Direction;
} MotorStruct;

void MainTask(void * pvParameters)
{
	/* Use this task to:
	 * - Create objects for each module
	 *     (OBS! It is very important that objects created with "new"
	 *      only happens within a thread due to the usage of the FreeRTOS managed heap)
	 * - Link any modules together if necessary
	 * - Create message exchange queues and/or semaphore
	 * - (Create) and start threads related to modules
	 *
	 * Basically anything related to starting the system should happen in this thread and NOT in the main() function !!!
	 */

	IO * Button = new IO(GPIOA, GPIO_PIN_4, IO::PULL_NONE);
	Button->RegisterInterrupt(IO::TRIGGER_FALLING, ButtonInterrupt);

	PWM * red = new PWM(PWM::TIMER3, PWM::CH3, 1000, 255); red->SetRaw(25);
	PWM * green = new PWM(PWM::TIMER3, PWM::CH2); green->SetRaw(0);
	PWM * blue = new PWM(PWM::TIMER3, PWM::CH1); blue->SetRaw(0);

	//Timer * tim = new Timer(Timer::TIMER17, 1000000);

	Encoder * knob = new Encoder(Encoder::TIMER2);

	//USBCDC * usb = new USBCDC(USBCDC_TRANSMITTER_PRIORITY);

	IO * canStdby = new IO(GPIOA, GPIO_PIN_5); canStdby->Low();
	CANBus * can = new CANBus();

	//IO * PWMpin = new IO(GPIOA, GPIO_PIN_8); PWMpin->High();
	IO * INA = new IO(GPIOA, GPIO_PIN_9); INA->High();
	IO * INB = new IO(GPIOA, GPIO_PIN_10); INB->Low();
	IO * SEL0 = new IO(GPIOA, GPIO_PIN_3); SEL0->High();
	PWM * pwm = new PWM(PWM::TIMER1, PWM::CH1, 2000, 255);
	//pwm->SetRaw(255);
	//IO * pwm = new IO(GPIOA, GPIO_PIN_8); pwm->Low();
	//pwm->High();
	//INA->High(); INB->Low();

	IO * DebugPin = new IO(GPIOB, GPIO_PIN_10); DebugPin->Low();
	Debug::SetDebugPin((void*)DebugPin);

	ADC * CS = new ADC(ADC::ADC_1, ADC::ADC_CHANNEL_0, 100, TIM1); // synchronize ADC samplings with PWM generation (to avoid aliasing spikes)
	//ADC * VNH_CS = new ADC(ADC::ADC_1, ADC::ADC_CHANNEL_1, 0);
	//ADC * VBAT = new ADC(ADC::ADC_1, ADC::ADC_CHANNEL_2, 0);
	//ADC * VBUS = new ADC(ADC::ADC_1, ADC::ADC_CHANNEL_9, 0);

	//I2C * i2c = new I2C(I2C::PORT_I2C1, 0x68);
	//SPI * spi = new SPI(SPI::PORT_SPI2, 200000, GPIOB, LL_GPIO_PIN_12);

	//UART * uart = new UART(UART::PORT_UART1, 115200, 100);
	char str[30];

	ALIGN_32BYTES (MotorStruct motor) {0, 128, 0};
	can->registerCallback(0x101, CAN_MotorPackage, (void *)&motor);

	//char * pcWriteBuffer = (char *)pvPortMalloc(100);

#if 0
	/* Send CPU load every second */
	char * pcWriteBuffer = (char *)pvPortMalloc(100);
	while (1)
	{
		vTaskGetRunTimeStats(pcWriteBuffer);
		char * endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
		*endPtr++ = '\n'; *endPtr++ = '\n'; *endPtr++ = 0;

		// Split into multiple packages and send
		uint16_t txIdx = 0;
		uint16_t remainingLength = strlen(pcWriteBuffer);
		uint16_t txLength;

		/*while (remainingLength > 0) {
			txLength = remainingLength;
			if (txLength > LSPC_MAXIMUM_PACKAGE_LENGTH) {
				txLength = LSPC_MAXIMUM_PACKAGE_LENGTH-1;
				while (pcWriteBuffer[txIdx+txLength] != '\n' && txLength > 0) txLength--; // find and include line-break (if possible)
				if (txLength == 0) txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
				else txLength++;
			}
			lspcUSB->TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)&pcWriteBuffer[txIdx], txLength);

			txIdx += txLength;
			remainingLength -= txLength;
		}*/
	}
#endif

	uint8_t value = 255;
	uint8_t RGB[3];
	int32_t knobValue;

	uint8_t testRead;
	int32_t analog_value = 0;
	float vbat = 0;
	float vref = 0;
	uint16_t temperature = 0;
	uint32_t i = 0;

	const uint8_t payload[] = { 0xBB, 0xAD };

	while (1)
	{
		/*
		//pwm->High();
		//pwm->SetRaw(200);

		for (i = 0; i < 500; i++) {
			ReadAnalog(ADC_Samples, CS, VNH_CS, VBAT, VBUS);
			can->Transmit(0x101, (uint8_t *)&ADC_Samples[0], 6);
			can->Transmit(0x102, (uint8_t *)&ADC_Samples[1], 6);
			osDelay(1);
		}

		//pwm->Low();
		pwm->SetRaw(0);
		for (i = 0; i < 250; i++) {
			ReadAnalog(ADC_Samples, CS, VNH_CS, VBAT, VBUS);
			can->Transmit(0x101, (uint8_t *)&ADC_Samples[0], 6);
			can->Transmit(0x102, (uint8_t *)&ADC_Samples[1], 6);
			osDelay(1);
		}

		INA->Low(); SEL0->Low(); INB->High();
		for (i = 0; i < 250; i++) {
			ReadAnalog(ADC_Samples, CS, VNH_CS, VBAT, VBUS);
			can->Transmit(0x101, (uint8_t *)&ADC_Samples[0], 6);
			can->Transmit(0x102, (uint8_t *)&ADC_Samples[1], 6);
			osDelay(1);
		}

		//pwm->High();
		//pwm->SetRaw(200);
		for (i = 0; i < 500; i++) {
			ReadAnalog(ADC_Samples, CS, VNH_CS, VBAT, VBUS);
			can->Transmit(0x101, (uint8_t *)&ADC_Samples[0], 6);
			can->Transmit(0x102, (uint8_t *)&ADC_Samples[1], 6);
			osDelay(1);
		}

		//pwm->Low();
		pwm->SetRaw(0);
		for (i = 0; i < 250; i++) {
			ReadAnalog(ADC_Samples, CS, VNH_CS, VBAT, VBUS);
			can->Transmit(0x101, (uint8_t *)&ADC_Samples[0], 6);
			can->Transmit(0x102, (uint8_t *)&ADC_Samples[1], 6);
			osDelay(1);
		}

		INA->High(); SEL0->High(); INB->Low();
		for (i = 0; i < 250; i++) {
			ReadAnalog(ADC_Samples, CS, VNH_CS, VBAT, VBUS);
			can->Transmit(0x101, (uint8_t *)&ADC_Samples[0], 6);
			can->Transmit(0x102, (uint8_t *)&ADC_Samples[1], 6);
			osDelay(1);
		}
		*/

		if (motor.Direction == 0) { // reverse
			//INA->Low(); /*SEL0->Low();*/ INB->High();
		} else { // forward
			//INA->High(); /*SEL0->High();*/ INB->Low();
		}
		pwm->SetRaw(motor.PWM);
		motor.Timestamp = HAL_GetHighResTick();

		//ReadAnalog(ADC_Samples, CS, VNH_CS, VBAT, VBUS);
		ReadAnalog(ADC_Samples, CS, 0, 0, 0);
		can->Transmit(0x101, (uint8_t *)&ADC_Samples[0], 6); // CS
		//can->Transmit(0x102, (uint8_t *)&ADC_Samples[1], 6); // VNH_CS
		//can->Transmit(0x103, (uint8_t *)&ADC_Samples[2], 6); // VBAT
		//can->Transmit(0x104, (uint8_t *)&motor, 7); // Motor out

		//vTaskGetRunTimeStats(pcWriteBuffer);
		//char * endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
		//*endPtr++ = '\n'; *endPtr++ = '\n'; *endPtr++ = 0;
		osDelay(10);
	}

#if 0
	while (1)
	{
		/*jetColor(value, 0, 255, RGB);
		value++;

		RGB[0] /= 10;
		RGB[1] /= 10;
		RGB[2] /= 10;*/

		knobValue = knob->Get();
		if (knobValue >= 0) {
			RGB[0] = 20;
			RGB[1] = 0;
			RGB[2] = 0;
		} else {
			RGB[0] = 0;
			RGB[1] = 20;
			RGB[2] = 0;
		}

		uint32_t divisor = abs(knobValue)/2;
		if (divisor == 0) divisor = 1;
		RGB[0] /= divisor;
		RGB[1] /= divisor;
		RGB[2] /= divisor;

		red->SetRaw(RGB[0]);
		green->SetRaw(RGB[1]);
		blue->SetRaw(RGB[2]);

		sprintf(str, "This is a test %d\n", i++);
		usb->WriteBlocking((uint8_t *)str, strlen(str));

		//can->Transmit(0x321, (uint8_t *)payload, 2);

		//testRead = i2c->Read(0x75);
		//testRead = spi->Read(0x75 | 0x80);

		HAL_Delay(20);
	}
#endif

	/*while (1)
	{
		vTaskSuspend(NULL); // suspend this task
	}*/
}

void CAN_MotorPackage(void * param, const CANBus::package_t &package)
{
	MotorStruct * motor = (MotorStruct *)param;
	if (!motor) return;

	if (package.DataLength == 2) {
		motor->Direction = package.Data[0];
		motor->PWM = package.Data[1];
	}
}

void ReadAnalog(ADC::measurement_t * ADC_Samples, ADC * CS, ADC * VNH_CS, ADC * VBAT, ADC * VBUS)
{
	/*if (CS->MeasurementAvailable())
		ADC_Samples[0] = CS->GetLatestMeasurement();
	if (VNH_CS->MeasurementAvailable())
		ADC_Samples[1] = VNH_CS->GetLatestMeasurement();
	if (VBAT->MeasurementAvailable())
		ADC_Samples[2] = VBAT->GetLatestMeasurement();
	if (VBUS->MeasurementAvailable())
		ADC_Samples[3] = VBUS->GetLatestMeasurement();*/
	ADC_Samples[0].timestamp = ADC_Samples[1].timestamp = ADC_Samples[2].timestamp = ADC_Samples[3].timestamp = CS->GetLatestSampleTimestamp();
	ADC_Samples[0].mVolt = CS->Read_mVolt();
	//ADC_Samples[1].mVolt = VNH_CS->Read_mVolt();
	//ADC_Samples[2].mVolt = VBAT->Read_mVolt();
	//ADC_Samples[3].mVolt = VBUS->Read_mVolt();
}

void ButtonInterrupt(void * param)
{
	// Interrupt code here
}

void Reboot_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	NVIC_SystemReset();
}

void EnterBootloader_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	//USBD_Stop(&USBCDC::hUsbDeviceFS);
	//USBD_DeInit(&USBCDC::hUsbDeviceFS);
	Enter_DFU_Bootloader();
}

void Enter_DFU_Bootloader(void)
{
	//call this at any time to initiate a reboot into bootloader
	//__disable_irq();

	// Important to stop current USB periphiral before entering DFU bootloader (see "EnterBootloader_Callback" in MainTask.cpp)

	//HAL_SuspendTick();
	/* Disable SysTick Interrupt */
    //SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    //taskENTER_CRITICAL();
    //portDISABLE_INTERRUPTS();
    *BOOTLOADER_MAGIC_ADDR = BOOTLOADER_MAGIC_TOKEN;

	//(*(__IO uint32_t *) (BKPSRAM_BASE + 0)) = A_VALUE;
    NVIC_SystemReset();
}


void jetColor(const float v, const float vmin, const float vmax, float RGB[3])
{
	float value = v;
	float dv = vmax - vmin;

	if (value < vmin) value = vmin;
	if (value > vmax) value = vmax;

	RGB[0] = 1.0;
	RGB[1] = 1.0;
	RGB[2] = 1.0;

    if (value < (vmin + 0.25 * dv)) {
    	RGB[0] = 0;
    	RGB[1] = 4 * (value - vmin) / dv;
	} else if (value < (vmin + 0.5 * dv)) {
		RGB[0] = 0;
		RGB[2] = 1 + 4 * (vmin + 0.25 * dv - value) / dv;
	} else if (value < (vmin + 0.75 * dv)) {
		RGB[0] = 4 * (value - vmin - 0.5 * dv) / dv;
		RGB[2] = 0;
	} else {
		RGB[1] = 1 + 4 * (vmin + 0.75 * dv - value) / dv;
		RGB[2] = 0;
	}
}
