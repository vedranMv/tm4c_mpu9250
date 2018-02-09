/**
 * uartHW.c
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 */
#include "libs/myLib.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "uartHW.h"

///-----------------------------------------------------------------------------
///         Functions for returning static instance                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return reference to a singleton
 * @return reference to an internal static instance
 */
SerialPort& SerialPort::GetI()
{
    static SerialPort singletonInstance;
    return singletonInstance;
}

/**
 * Return pointer to a singleton
 * @return pointer to a internal static instance
 */
SerialPort* SerialPort::GetP()
{
    return &(SerialPort::GetI());
}

SerialPort::SerialPort() {}
SerialPort::~SerialPort() {}

void SerialPort::Send(const char* arg, ...)
{
    va_list vaArgP;


    //	Start the varargs processing.
    va_start(vaArgP, arg);

    UARTvprintf(arg, vaArgP);

    //	We're finished with the varargs now.
    va_end(vaArgP);
}

/**
 * Initialize UART port used in communication with Raspberry Pi
 */
int8_t SerialPort::InitHW()
{
	uint32_t refClock, refClockHz;

	/*
	 * Check desired communication speed and adjust clock settings used to
	 * 		Derive desired baud rate
	 */
	if (COMM_BAUD > 2000000)
	{
		refClock =  UART_CLOCK_SYSTEM;
		refClockHz = g_ui32SysClock;
	}
	else
	{
		refClock =  UART_CLOCK_PIOSC;
		refClockHz = 16000000;
	}

	/*
	 * Configure UARTstdio and accompanying GPIO pins for UART communication
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    UARTClockSourceSet(UART0_BASE, refClock);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, COMM_BAUD, refClockHz);

    /*
     * Enable Interrupt on received data
     */
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
   	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
   	UARTIntRegister(UART0_BASE,UART0RxIntHandler);
   	IntEnable(INT_UART0);

    IntMasterEnable();

	return STATUS_OK;
}

void SerialPort::AddHook(void((*funPoint)(uint8_t*, uint16_t*)))
{
	custHook = funPoint;
}

/**
 * Interrupt service routine for handling incoming data on UART (Tx)
 * @note IMPORTANT Custom hook HAS TO clear the buffer and bufLen variable
 */
void UART0RxIntHandler(void)
{
	static uint8_t txBuffer[TX_BUF_LEN];
	static uint16_t txBufLen = 0;

	//Clear interrupt flag
	UARTIntClear(UART0_BASE, UART_INT_RX);

	//Take all chars from Tx FIFO and put them in buffer
	while(UARTCharsAvail(UART0_BASE))
	{
		txBuffer[ txBufLen++ ] = UARTCharGet(UART0_BASE);
		//	Add echo fo debugging purpose
		UARTCharPut(UART0_BASE, txBuffer[ txBufLen-1 ]);
	}

	if (SerialPort::GetP()->custHook != 0)
	    SerialPort::GetP()->custHook(txBuffer, &txBufLen);
}



