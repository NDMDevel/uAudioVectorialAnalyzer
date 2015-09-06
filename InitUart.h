/* 
 * File:   InitUart.h
 * Author: Damian
 *
 * Created on 6 de septiembre de 2015, 15:24
 */

#ifndef INITUART_H
#define	INITUART_H

#ifdef	__cplusplus
extern "C" {
#endif

#define FP 40000000
#define BAUDRATE 128000
#define BRGVAL ((FP/BAUDRATE)/16)-1
#define PIN_U1TX 0x0003
#define PIN_U1RX 0x0008 //pin RP14

#ifdef	__cplusplus
}
#endif

#endif	/* INITUART_H */
