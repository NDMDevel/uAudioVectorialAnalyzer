/* 
 * File:   Global_Defines.h
 * Author: Damian
 *
 * Created on 6 de septiembre de 2015, 15:41
 */

#ifndef GLOBAL_DEFINES_H
#define	GLOBAL_DEFINES_H

#ifdef	__cplusplus
extern "C" {
#endif

#define PI 3.1415926535897932384626433832795
#define FRes 8.33333333333333333333333333333333
#define INT_RISE 0
#define INT_FALL 1
#define IN_INT_RISE()       INTCON2bits.INT0EP = INT_RISE
#define IN_INT_FALL()       INTCON2bits.INT0EP = INT_FALL
#define IN_INT_EDGE         INTCON2bits.INT0EP
#define OUT_INT_RISE()      INTCON2bits.INT1EP = INT_RISE
#define OUT_INT_FALL()      INTCON2bits.INT1EP = INT_FALL
#define OUT_INT_EDGE        INTCON2bits.INT1EP
#define IN_INT_ENABLE()     IFS0bits.INT0IF = 0;IEC0bits.INT0IE = 1
#define IN_INT_DISABLED()   IFS0bits.INT0IF = 0;IEC0bits.INT0IE = 0
#define OUT_INT_ENABLE()    IFS1bits.INT1IF = 0;IEC1bits.INT1IE = 1
#define OUT_INT_DISABLED()  IFS1bits.INT1IF = 0;IEC1bits.INT1IE = 0
#define ADC_INT_ENABLE()    IFS0bits.AD1IF = 0;IEC0bits.AD1IE = 1;          // Enable ADC interrupt
#define ADC_INT_DISABLED()  IFS0bits.AD1IF = 0;IEC0bits.AD1IE = 0;          // Disable ADC interrupt
#define ADC_IN_CONFIG()     AD1CON1bits.ADON = 0;AD1CHS0bits.CH0SA = 4; AD1CHS0bits.CH0NA = 0;   // Select AN4 for CH0 +ve input
#define ADC_OUT_CONFIG()    AD1CON1bits.ADON = 0;AD1CHS0bits.CH0SA = 5; AD1CHS0bits.CH0NA = 0;   // Select AN5 for CH0 +ve input
#define ADC_DISABLED()      AD1CON1bits.ADON = 0;
#define ADC_ENABLE()        AD1CON1bits.ADON = 1;
#define SET_TMR32_TIMEOUT_T PR3 = ((0xFFFF0000)&(Period>>2))>>16;PR2 = (0x0000FFFF)&(Period>>2)
#define CLEAR_TMR32()       TMR3HLD = 0;TMR2 = 0
#define GET_TMR32()         ((uint32_t)TMR2)|(((uint32_t)TMR3HLD)<<16)
#define TMR32_ON()          T2CONbits.TON=1
#define TMR32_OFF()         T2CONbits.TON=0
#define TMR32_INT_ENABLE()      IFS0bits.T3IF=0;IEC0bits.T3IE=1
#define TMR32_INT_DISABLED()    IEC0bits.T3IE=0
//#define SET_TMR32_DEFAULT()   PR3=0x0049;PR2=0xFFFF   //interrumpe a los 120mS (muy cerca del Periodo de una señal de 8.33Hz)
#define SET_TMR32_TIMEOUT_DEFAULT_MAX_T()   PR3=0x004F;PR2=0x5880   //interrumpe a los 130mS (algo mas alejado del Periodo de una señal de 8.33Hz)
#define K 4800000 //constante para calcular TMR32 con fs=500000, N=15000 y Tcy=25ns
#define TMR1_PRIOR_ISR  7
#define INT_PRIOR_ISR   2
#define ADC_PRIOR_ISR   1



#ifdef	__cplusplus
}
#endif

#endif	/* GLOBAL_DEFINES_H */

