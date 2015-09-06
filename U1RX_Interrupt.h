/* 
 * File:   U1RX_Interrupt.h
 * Author: Damian
 *
 * Created on 6 de septiembre de 2015, 15:27
 */

#ifndef U1RX_INTERRUPT_H
#define	U1RX_INTERRUPT_H

#ifdef	__cplusplus
extern "C" {
#endif

#define UartSize 10
char Uart[UartSize];
uint8_t UartPos;
/* UpdateNewPhaseInc:
 * Setea asincronicamente (independientemente del barrido) el valor del
 * incremento de fase del DDS, es decir, setea la variable NewPhaseInc.
 * (Nota: generar saltos en NewPhaseInc muy grandes puede generar problemas
 * en el sistema. Es conveniente no setear un valor mayor a 500 respecto del
 * anterior)
 */
#define COM_0 'a'  //Update NewPhaseInc
/* SetInitFrec:
 * Configura la frecuencia desde la que comenzara el barrido */
#define COM_1 'b'
/* SetEndFrec:
 * Configura la frecuencia en la que el barrido termina */
#define COM_2 'c'  //Set End Frec
/* SetDeltaFrec:
 * Configura el salto de frecuencia del barrido */
#define COM_3 'd'  //Set Delta Frec
/* InitSweep:
 * Inicia el barrido con la configuracion ya establecida por SetInitFrec,
 * SetEndFrec y SetDeltaFrec. */
#define COM_4 'e'
/* :
 * */
#define COM_5 'f'

#ifdef	__cplusplus
}
#endif

#endif	/* U1RX_INTERRUPT_H */

