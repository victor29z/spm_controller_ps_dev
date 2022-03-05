/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters_ps.h"
#include "xil_io.h"


#include "xscutimer.h"
#include "xil_exception.h"

#include "sleep.h"
#include "xscugic.h"
#include "uart_parameter.h"



#define TIMER_DEVICE_ID		XPAR_XSCUTIMER_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_IRPT_INTR		XPAR_SCUTIMER_INTR

// timeup = 1s/1000 = 1ms
#define TIMER_LOAD_VALUE	(XPAR_PS7_CORTEXA9_0_CPU_CLK_FREQ_HZ/2/1000 - 1)
/*
 * The following constant is used to wait after an LED is turned on to make
 * sure that it is visible to the human eye.  This constant might need to be
 * tuned for faster or slower processor speeds.
 */


/*******************************UART relevant ********************************/


//#define UART_DEVICE_ID      XPAR_XUARTPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_INT_IRQ_ID		XPAR_XUARTPS_0_INTR

/* Statement */
#define UART_TX      0
#define UART_RXCHECK 1
#define UART_WAIT    2

/* maximum receiver length */
#define MAX_LEN    32
/* maximum moving heartbeat wait time in milisecond */
#define MOTOR_MOVING_TIMEOUT 100

/**************************** Type Definitions *******************************/
typedef union{
	unsigned char cmd_bytes[9];
	struct {
		unsigned int steps;
		unsigned int speed;
		unsigned char direction;

	}cmd;

}SER_CMD_TYPE;


/*------------------------------------------------------------------------------------------*/

#define DDR_BASEARDDR      XPAR_DDR_MEM_BASEADDR + 0x10000000
#define MC_BASE_ADDR	0x43C00000
#define FREQ_ADDR	(MC_BASE_ADDR + 0)	//0
#define VMIN_ADDR	(MC_BASE_ADDR + 4)	//1
#define DTYC_ADDR	(MC_BASE_ADDR + 8)	//2
#define CCW_ADDR	(MC_BASE_ADDR + 12)	//3
#define ST_ADDR		(MC_BASE_ADDR + 16)	//4
#define NSTEP_ADDR	(MC_BASE_ADDR + 20)	//5
//6
//7

#define ANA_BASE_ADDR	0x43C10000

#define CL_EN			(ANA_BASE_ADDR + 0)		//0
#define DAC_H_SET		(ANA_BASE_ADDR + 4)		//1
#define CL_SP			(ANA_BASE_ADDR + 8)		//2
#define KP				(ANA_BASE_ADDR + 12)	//3
#define KI				(ANA_BASE_ADDR + 16)	//4
//4
//5
#define ADC_OUT			(ANA_BASE_ADDR + 24)	//6
#define DBG_IOUT		(ANA_BASE_ADDR + 28)	//7
#define DAC_H_IN		(ANA_BASE_ADDR + 32)	//8
#define DBG_POUT		(ANA_BASE_ADDR + 36)	//9



#define AD_CH0_ADDR	(MC_BASE_ADDR + 32)	//8
#define AD_CH1_ADDR	(MC_BASE_ADDR + 36)	//9



XUartPs Uart_PS;		/* Instance of the UART Device */
XScuGic IntcInstPtr ;

/* UART receiver buffer */
u8 ReceivedBuffer[MAX_LEN] ;
/* UART receiver buffer pointer*/
u8 *ReceivedBufferPtr ;
/* UART receiver byte number */
volatile u32 ReceivedByteNum ;

volatile u32 ReceivedFlag  ;
volatile u8 MovingHeartBeat = 0;
volatile u8 Motor_Moving_State = 0;
u32 Motor_Moving_Dead_Count = 0;
u32 ReceivedCount = 0 ;
SER_CMD_TYPE ser_cmd;
u32 IncrementEnc_data;


/*
 * Function declaration
 */
int UartPsSend(XUartPs *InstancePtr, u8 *BufferPtr, u32 NumBytes) ;
int UartPsRev (XUartPs *InstancePtr, u8 *BufferPtr, u32 NumBytes) ;

int SetupInterruptSystem(XScuGic *IntcInstancePtr,	XUartPs *UartInstancePtr, u16 UartIntrId);
void UartIntrHandler(void *CallBackRef);



int ScuTimerIntrExample(XScuGic *IntcInstancePtr, XScuTimer *TimerInstancePtr,
			u16 TimerDeviceId, u16 TimerIntrId);

static void TimerIntrHandler(void *CallBackRef);

static int TimerSetupIntrSystem(XScuGic *IntcInstancePtr,
				XScuTimer *TimerInstancePtr, u16 TimerIntrId);

static void TimerDisableIntrSystem(XScuGic *IntcInstancePtr, u16 TimerIntrId);

XScuTimer TimerInstance;	/* Cortex A9 Scu Private Timer Instance */
XScuGic IntcInstance;		/* Interrupt Controller Instance */

volatile int TimerExpired;

u64 system_ticks = 0;

int main()
{
    init_platform();
    System_Init();

    int i;

	int rev;
	sleep(1);
	print("Hello World\n\r");
	Xil_Out32(ST_ADDR,(u32)0);
// 1 count = 20ns
	Xil_Out32(VMIN_ADDR,(u32)200000);
	Xil_Out32(CCW_ADDR,(u32)0);
	Xil_Out32(FREQ_ADDR,(u32)2500);
	Xil_Out32(DTYC_ADDR,(u32)700);
	//Xil_Out32(ST_ADDR,(u32)1);
	//Xil_Out32(NSTEP_ADDR,(u32)300);

	while(1){

		main_loop();
	}

    cleanup_platform();
    return 0;
}
void read_adc(void){
	u32 adc_value = Xil_In32(ADC_OUT);
	u32 dac_value = Xil_In32(DAC_H_IN);

	u8 buffer[10];

	//head
	buffer[0] = 0xc0;
	buffer[1] = 0x5e;
	// adc value
	buffer[2] = adc_value & 0xff;
	buffer[3] = (adc_value & 0xff00) >> 8;
	//dac value
	buffer[4] = dac_value & 0xff;
	buffer[5] = (dac_value & 0xff00) >> 8;

	// eof
	buffer[6] = 0xb4;
	buffer[7] = 0xf8;

	UartPsSend(&Uart_PS, buffer, 8);
	//printf("ADch0: %f\tADch1: %f \n\r",dch0,dch1);
}

void main_loop(void){
	//read adc every 50ms
	if(system_ticks % 50 == 0){
		read_adc();

	}
	if(Motor_Moving_State == 1){
		if(Motor_Moving_Dead_Count > MOTOR_MOVING_TIMEOUT){
			//XGpio_DiscreteWrite(&Gpio_1, START_STOP_CHANNEL, 0);
			Xil_Out32(ST_ADDR,(u32)2);// send emergency stop
			Motor_Moving_State = 0;

		}

	}
	//check serial port receive flag
	if(ReceivedFlag){
		ReceivedFlag = 0;
		//check receive command type
		if(ReceivedCount == 2){
			// string line serial command
			if(ReceivedBuffer[0] == 'f'){
				Xil_Out32(CCW_ADDR,(u32)0);
				Xil_Out32(ST_ADDR,(u32)1);

				Motor_Moving_State = 1;
				Motor_Moving_Dead_Count = 0;

			}
			if(ReceivedBuffer[0] == 'e'){
				Xil_Out32(CCW_ADDR,(u32)0);
				Xil_Out32(ST_ADDR,(u32)1);
				Motor_Moving_State = 1;
				Motor_Moving_Dead_Count = 0;

			}
			if(ReceivedBuffer[0] == 'c'){
				Xil_Out32(CCW_ADDR,(u32)0);
				Xil_Out32(ST_ADDR,(u32)1);
				Motor_Moving_State = 1;
				Motor_Moving_Dead_Count = 0;

			}
			if(ReceivedBuffer[0] == 'i'){
				Xil_Out32(CCW_ADDR,(u32)1);
				Xil_Out32(ST_ADDR,(u32)1);
				Motor_Moving_State = 1;
				Motor_Moving_Dead_Count = 0;

			}
			if(ReceivedBuffer[0] == 'u'){
				Xil_Out32(CCW_ADDR,(u32)1);
				Xil_Out32(ST_ADDR,(u32)1);
				Motor_Moving_State = 1;
				Motor_Moving_Dead_Count = 0;

			}
			if(ReceivedBuffer[0] == 't'){
				Xil_Out32(CCW_ADDR,(u32)1);
				Xil_Out32(ST_ADDR,(u32)1);
				Motor_Moving_State = 1;
				Motor_Moving_Dead_Count = 0;

			}
			if(ReceivedBuffer[0] == 'n'){
				//XGpio_DiscreteWrite(&Gpio_1, START_STOP_CHANNEL, 0);
				if( Motor_Moving_State == 2)
					Xil_Out32(ST_ADDR,(u32)2);// emergency stop in nstep mode
				else
					Xil_Out32(ST_ADDR,(u32)0);
				Motor_Moving_State = 0;
			}

		}
		// compact serial command
		if(ReceivedCount == 11){
			// motor control command
			if (ReceivedBuffer[0] == 'c'){
				Xil_Out32(ST_ADDR,(u32)0);//stop
				memcpy((void*)&ser_cmd, (void*)(ReceivedBuffer +2), 9);
				//write direction
				if(ser_cmd.cmd.direction == 1)
					Xil_Out32(CCW_ADDR,(u32)1);
				else
					Xil_Out32(CCW_ADDR,(u32)0);
				//set moving state to nstep mode
				Motor_Moving_State = 2;
				//set mode to nstep
				Xil_Out32(ST_ADDR,(u32)3);
				//write speed
				//XGpio_DiscreteWrite(&Gpio_0, FREQ_CHANNEL, (u32)ser_cmd.cmd.speed);
				//write steps
				Xil_Out32(NSTEP_ADDR,(u32)ser_cmd.cmd.steps);
				//XGpio_DiscreteWrite(&Gpio_4, nStep_CHANNEL, (u32)ser_cmd.cmd.steps);

				//XGpio_DiscreteWrite(&Gpio_1, START_STOP_CHANNEL, 3);
			}
			// pzt control command
			if (ReceivedBuffer[0] == 'p'){
				// enable/disable pid command
				if(ReceivedBuffer[2] == 'e'){
					//set enable pid
					if(ReceivedBuffer[3] == 'e'){
						Xil_Out32(CL_EN,(u32)1);

					}
					//set disable pid
					else{
						Xil_Out32(CL_EN,(u32)0);

					}

				}
				// open loop da control command
				else if(ReceivedBuffer[2] == 'd'){
					unsigned int dac_code = ReceivedBuffer[4] * 256 + ReceivedBuffer[3];
					//set dac code
					Xil_Out32(DAC_H_SET,(u32)dac_code);
				}

				// kp setting command
				else if(ReceivedBuffer[2] == 'p'){
					unsigned int kp_code = ReceivedBuffer[4] * 256 + ReceivedBuffer[3];
					//set kp code
					Xil_Out32(KP,(u32)kp_code);
				}

				// ki setting command
				else if(ReceivedBuffer[2] == 'i'){
					unsigned int ki_code = ReceivedBuffer[4] * 256 + ReceivedBuffer[3];
					//set kp code
					Xil_Out32(KI,(u32)ki_code);
				}

				// set point setting command
				else if(ReceivedBuffer[2] == 's'){
					unsigned int setpoint_code = ReceivedBuffer[4] * 256 + ReceivedBuffer[3];
					//set kp code
					Xil_Out32(CL_SP,(u32)setpoint_code);
				}


			}

		}
		if(ReceivedBuffer[0] == '0'){
			MovingHeartBeat = 1;
			Motor_Moving_Dead_Count = 0;
		}



	}

}


void System_Init(void){
	int Status;
	XUartPs_Config *Config;

	u32 SendByteNum ;
	u8 *SendBufferPtr ;
	u8 state = UART_TX ;

	ReceivedBufferPtr = ReceivedBuffer ;

	ReceivedFlag = 0 ;
	ReceivedByteNum = 0 ;
	//volatile int Delay;

	Config = XUartPs_LookupConfig(UART_DEVICE_ID);
	if (NULL == Config) {
		return XST_FAILURE;
	}
	Status = XUartPs_CfgInitialize(&Uart_PS, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	/* Use Normal mode. */
	XUartPs_SetOperMode(&Uart_PS, XUARTPS_OPER_MODE_NORMAL);
	/* Set uart mode Baud Rate 115200, 8bits, no parity, 1 stop bit */
	XUartPs_SetDataFormat(&Uart_PS, &UartFormat) ;//115200, 8N1
	/*Set receiver FIFO interrupt trigger level*/
	XUartPs_SetFifoThreshold(&Uart_PS,32) ;
	XUartPs_SetRecvTimeout(&Uart_PS,5);
	/* Enable the receive FIFO trigger level interrupt and empty interrupt for the device */
	XUartPs_SetInterruptMask(&Uart_PS,XUARTPS_IXR_RXOVR | XUARTPS_IXR_TOUT);

	SetupInterruptSystem(&IntcInstPtr, &Uart_PS, UART_INT_IRQ_ID) ;


	//xil_printf("SCU Timer Interrupt Example Test \r\n");
	Status = ScuTimerIntrExample(&IntcInstPtr, &TimerInstance,
						TIMER_DEVICE_ID, TIMER_IRPT_INTR);
	if (Status != XST_SUCCESS) {
		xil_printf("SCU Timer Interrupt Example Test Failed\r\n");
		return XST_FAILURE;
	}

	//xil_printf("Successfully ran SCU Timer Interrupt Example Test\r\n");
	return XST_SUCCESS;


}

int ScuTimerIntrExample(XScuGic *IntcInstancePtr, XScuTimer * TimerInstancePtr,
			u16 TimerDeviceId, u16 TimerIntrId)
{
	int Status;
	int LastTimerExpired = 0;
	XScuTimer_Config *ConfigPtr;

	/*
	 * Initialize the Scu Private Timer driver.
	 */
	ConfigPtr = XScuTimer_LookupConfig(TimerDeviceId);

	/*
	 * This is where the virtual address would be used, this example
	 * uses physical address.
	 */
	Status = XScuTimer_CfgInitialize(TimerInstancePtr, ConfigPtr,
					ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	Status = XScuTimer_SelfTest(TimerInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the device to interrupt subsystem so that interrupts
	 * can occur.
	 */
	Status = TimerSetupIntrSystem(IntcInstancePtr,
					TimerInstancePtr, TimerIntrId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	/*
	 * Enable Auto reload mode.
	 */
	XScuTimer_EnableAutoReload(TimerInstancePtr);

	/*
	 * Load the timer counter register.
	 */
	XScuTimer_LoadTimer(TimerInstancePtr, TIMER_LOAD_VALUE);

	/*
	 * Start the timer counter and then wait for it
	 * to timeout a number of times.
	 */
	XScuTimer_Start(TimerInstancePtr);



	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function sets up the interrupt system such that interrupts can occur
* for the device.
*
* @param	IntcInstancePtr is a pointer to the instance of XScuGic driver.
* @param	TimerInstancePtr is a pointer to the instance of XScuTimer
*		driver.
* @param	TimerIntrId is the Interrupt Id of the XScuTimer device.
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
static int TimerSetupIntrSystem(XScuGic *IntcInstancePtr,
			      XScuTimer *TimerInstancePtr, u16 TimerIntrId)
{
	int Status;

#ifndef TESTAPP_GEN
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

//	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
//					IntcConfig->CpuBaseAddress);
//	if (Status != XST_SUCCESS) {
//		return XST_FAILURE;
//	}
//
//	while(1);
	Xil_ExceptionInit();




	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
//	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
//				(Xil_ExceptionHandler)XScuGic_InterruptHandler,
//				IntcInstancePtr);
#endif

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, TimerIntrId,
				(Xil_ExceptionHandler)TimerIntrHandler,
				(void *)TimerInstancePtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}


	/*
	 * Enable the interrupt for the device.
	 */
	XScuGic_Enable(IntcInstancePtr, TimerIntrId);

	/*
	 * Enable the timer interrupts for timer mode.
	 */
	XScuTimer_EnableInterrupt(TimerInstancePtr);

#ifndef TESTAPP_GEN
	/*
	 * Enable interrupts in the Processor.
	 */
	Xil_ExceptionEnable();
#endif

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function is the Interrupt handler for the Timer interrupt of the
* Timer device. It is called on the expiration of the timer counter in
* interrupt context.
*
* @param	CallBackRef is a pointer to the callback function.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void TimerIntrHandler(void *CallBackRef)
{
	XScuTimer *TimerInstancePtr = (XScuTimer *) CallBackRef;



	/*
	 * Check if the timer counter has expired, checking is not necessary
	 * since that's the reason this function is executed, this just shows
	 * how the callback reference can be used as a pointer to the instance
	 * of the timer counter that expired, increment a shared variable so
	 * the main thread of execution can see the timer expired.
	 */
	if (XScuTimer_IsExpired(TimerInstancePtr)) {
		XScuTimer_ClearInterruptStatus(TimerInstancePtr);
//		XGpio_DiscreteWrite(&Gpio_IO, DIR_CHANNEL, DIR);
//		DIR=~DIR;
//		XGpio_DiscreteWrite(&Gpio_IO, PWM_CHANNEL, PWM);
//		PWM=~PWM;
//		AD1=XGpio_DiscreteRead(&Gpio_AD, AD1_CHANNEL);
//		AD2=XGpio_DiscreteRead(&Gpio_AD, AD2_CHANNEL);
//		//xil_printf("AD1: %d Second\n\r",AD1);
//		//xil_printf("AD2 %d Second\n\r",AD2);
		system_ticks++;
		if(Motor_Moving_State == 1){
			if(MovingHeartBeat == 1){
				MovingHeartBeat = 0;
			}
			else{
				Motor_Moving_Dead_Count++;

			}


		}
	}
}

/*****************************************************************************/
/**
*
* This function disables the interrupts that occur for the device.
*
* @param	IntcInstancePtr is the pointer to the instance of XScuGic
*		driver.
* @param	TimerIntrId is the Interrupt Id for the device.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void TimerDisableIntrSystem(XScuGic *IntcInstancePtr, u16 TimerIntrId)
{
	/*
	 * Disconnect and disable the interrupt for the Timer.
	 */
	XScuGic_Disconnect(IntcInstancePtr, TimerIntrId);
}



int SetupInterruptSystem(XScuGic *IntcInstancePtr,	XUartPs *UartInstancePtr, u16 UartIntrId)
{
	int Status;
	/* Configuration for interrupt controller */
	XScuGic_Config *IntcConfig;

	/* Initialize the interrupt controller driver */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
			IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the interrupt controller interrupt handler to the
	 * hardware interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler) XScuGic_InterruptHandler,
			IntcInstancePtr);


	Status = XScuGic_Connect(IntcInstancePtr, UartIntrId,
			(Xil_ExceptionHandler) UartIntrHandler,
			(void *) UartInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XScuGic_Enable(IntcInstancePtr, UartIntrId);
	Xil_ExceptionEnable();

	return Status ;
}


void UartIntrHandler(void *CallBackRef)
{
	XUartPs *UartInstancePtr = (XUartPs *) CallBackRef ;
	//u32 ReceivedCount = 0 ;
	u32 UartSrValue ;
	UartSrValue = XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress, XUARTPS_IMR_OFFSET);
	UartSrValue &= XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET);
	//UartSrValue = XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress, XUARTPS_SR_OFFSET) & (XUARTPS_IXR_RXOVR|XUARTPS_IXR_TOUT);
	ReceivedFlag = 0 ;

	if (UartSrValue & XUARTPS_IXR_RXOVR)   /* check if receiver FIFO trigger */
	{
//		ReceivedCount = UartPsRev(&Uart_PS, ReceivedBufferPtr, MAX_LEN) ;
//		ReceivedByteNum += ReceivedCount ;
//		ReceivedBufferPtr += ReceivedCount ;
		/* clear trigger interrupt */
		XUartPs_WriteReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET, XUARTPS_IXR_RXOVR) ;
	}

	else if (UartSrValue & XUARTPS_IXR_TOUT){
		XUartPs_WriteReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET, XUARTPS_IXR_TOUT) ;
		ReceivedCount = UartPsRev(&Uart_PS, ReceivedBuffer, MAX_LEN) ;

		ReceivedFlag = 1 ;

	}
	//XUartPs_WriteReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET, XUARTPS_IXR_TOUT) ;


}




int UartPsSend(XUartPs *InstancePtr, u8 *BufferPtr, u32 NumBytes)
{

	u32 SentCount = 0U;

	/* Setup the buffer parameters */
	InstancePtr->SendBuffer.RequestedBytes = NumBytes;
	InstancePtr->SendBuffer.RemainingBytes = NumBytes;
	InstancePtr->SendBuffer.NextBytePtr = BufferPtr;


	while (InstancePtr->SendBuffer.RemainingBytes > SentCount)
	{
		/* Fill the FIFO from the buffer */
		if (!XUartPs_IsTransmitFull(InstancePtr->Config.BaseAddress))
		{
			XUartPs_WriteReg(InstancePtr->Config.BaseAddress,
					XUARTPS_FIFO_OFFSET,
					((u32)InstancePtr->SendBuffer.
							NextBytePtr[SentCount]));

			/* Increment the send count. */
			SentCount++;
		}
	}

	/* Update the buffer to reflect the bytes that were sent from it */
	InstancePtr->SendBuffer.NextBytePtr += SentCount;
	InstancePtr->SendBuffer.RemainingBytes -= SentCount;


	return SentCount;
}

int UartPsRev(XUartPs *InstancePtr, u8 *BufferPtr, u32 NumBytes)
{
	u32 ReceivedCount = 0;
	u32 CsrRegister;

	/* Setup the buffer parameters */
	InstancePtr->ReceiveBuffer.RequestedBytes = NumBytes;
	InstancePtr->ReceiveBuffer.RemainingBytes = NumBytes;
	InstancePtr->ReceiveBuffer.NextBytePtr = BufferPtr;

	/*
	 * Read the Channel Status Register to determine if there is any data in
	 * the RX FIFO
	 */
	CsrRegister = XUartPs_ReadReg(InstancePtr->Config.BaseAddress,
			XUARTPS_SR_OFFSET);

	/*
	 * Loop until there is no more data in RX FIFO or the specified
	 * number of bytes has been received
	 */
	while((ReceivedCount < InstancePtr->ReceiveBuffer.RemainingBytes)&&
			(((CsrRegister & XUARTPS_SR_RXEMPTY) == (u32)0)))
	{
		InstancePtr->ReceiveBuffer.NextBytePtr[ReceivedCount] =
				XUartPs_ReadReg(InstancePtr->Config.BaseAddress,XUARTPS_FIFO_OFFSET);

		ReceivedCount++;

		CsrRegister = XUartPs_ReadReg(InstancePtr->Config.BaseAddress,
				XUARTPS_SR_OFFSET);
	}
	InstancePtr->is_rxbs_error = 0;
	/*
	 * Update the receive buffer to reflect the number of bytes just
	 * received
	 */
	if(InstancePtr->ReceiveBuffer.NextBytePtr != NULL){
		InstancePtr->ReceiveBuffer.NextBytePtr += ReceivedCount;
	}
	InstancePtr->ReceiveBuffer.RemainingBytes -= ReceivedCount;

	return ReceivedCount;
}



