/**
 *	uartHW.h
 *
 *  Created on: 30. 7. 2016.
 *      Author: Vedran Mikov
 *
 *  Debug bridge between PC<--(USB)-->TM4C
 *
 */
#ifndef UARTHW_H_
#define UARTHW_H_
#include "libs/myLib.h"

/*		Communication settings	 	*/
#define COMM_BAUD	115200
#define TX_BUF_LEN	512

/*      Macro to short the expression needed to print to debug port     */
#define DEBUG_WRITE(...) SerialPort::GetI().Send(__VA_ARGS__)

/*      Macro for printing float numbers    */
#define _FTOI_(X) (int32_t)(trunc(X)),(int32_t)fabs(trunc((X-trunc(X))*100))


/**
 * Interface to a UART-to-USB port, used for debugging
 */
class SerialPort
{
	public:
        static SerialPort& GetI();
        static SerialPort* GetP();

		int8_t	InitHW();
		void	Send(const char* arg, ...);
		void	AddHook(void((*custHook)(uint8_t*, uint16_t*)));

		void	((*custHook)(uint8_t*, uint16_t*));  // Hook to user routine
	protected:
		SerialPort();
        ~SerialPort();

};


/*
 * Function for receiving and processing incomming data - no need to call them
 */

extern "C"
{
    void UART0RxIntHandler(void);
    extern uint32_t g_ui32SysClock;
}

#endif /* UARTHW_H_ */
