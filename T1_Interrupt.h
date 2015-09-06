/* 
 * File:   T1_Interrupt.h
 * Author: Damian
 *
 * Created on 6 de septiembre de 2015, 15:27
 */

#ifndef T1_INTERRUPT_H
#define	T1_INTERRUPT_H

#ifdef	__cplusplus
extern "C" {
#endif

#define POSITIVO 0xFF
#define NEGATIVO 0x00
#define FS_DW() LATBbits.LATB14 = 0
#define FS_UP() LATBbits.LATB14 = 1
uint16_t DAC_SAMPLE;
int32_t PhaseAcc;
int16_t PhaseInc;
int16_t NewPhaseInc;
int16_t InitPhase;
int16_t EndPhase;
int16_t DeltaPhase;
int16_t SignPhaseInc;
uint8_t Sign;

#ifdef	__cplusplus
}
#endif

#endif	/* T1_INTERRUPT_H */

