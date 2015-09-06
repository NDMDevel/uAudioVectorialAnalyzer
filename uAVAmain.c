/* 
 * File:   uAVAmain.c
 * Author: Damian
 *
 * Created on 16 de septiembre de 2015, 14:20
 */

/*
 * SECCION A:
 * 
 * - Directivas de procesador (no de usuario)
 * - Includes de procesador y standar (no de usuario)
 */
#include <xc.h>
#include <stdint.h>       //incluye uint8_t
#include <stdlib.h>     //incluye atoi
#define FCY 40000000
#include <libpic30.h>   //incluye __delay_us
//configuracion de la Palabra de Configuracion:
// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)
// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)
// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)
// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)
// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)
// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)
// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)
// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)
// <<<<<<<<<<<<<<<<< FIN SECCION A

/*
 * SECCION B:
 * 
 * - Defines de usuario
 * - Includes de usuario
 * - Variables globales
 */
#include "qSinTable.h"
#include "InitSpi.h"
#include "InitUart.h"
#include "T1_Interrupt.h"
#include "U1RX_Interrupt.h"
#include "Global_Vars.h"
#include "Global_Defines.h"
// <<<<<<<<<<<<<<<<< FIN SECCION B

/*
 * SECCION C:
 * 
 * - Funciones de uso general (no vectores de interrupcion ni tareas multitasking)
 */
inline void SetDev40MIPS(void)
{
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBD = 41; // M = 43
    CLKDIVbits.PLLPOST=0; // N2 = 2
    CLKDIVbits.PLLPRE=0; // N1 = 2
    // Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001);
        RCONbits.SWDTEN = 0;
    // Wait for PLL to lock
    while(OSCCONbits.LOCK!=1) {};
}
void InitSpi(void)
{
    //configura los pines
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPOR6bits.RP12R = PIN_SDO1;
    RPOR6bits.RP13R = PIN_SCK1OUT;
    __builtin_write_OSCCONL(OSCCON | (1<<6));

    //configura el modulo SPI
    IFS0bits.SPI1IF = 0;        // Clear the Interrupt flag
    IEC0bits.SPI1IE = 0;        // Disable the interrupt

    // Fspi = 13.333 MHz
    SPI1CON1bits.PPRE = 3;      // Primary 1:1
    SPI1CON1bits.SPRE = 5;      // Secundary 3:1

    // SPI1CON1 Register Settings
    SPI1CON1bits.DISSCK = 0;    // Internal serial clock is enabled
    SPI1CON1bits.DISSDO = 0;    // SDOx pin is controlled by the module
    SPI1CON1bits.MODE16 = 1;    // Communication is word-wide (16 bits)
    SPI1CON1bits.MSTEN = 1;     // Master mode enabled
    SPI1CON1bits.SMP = 0;       // Input data is sampled at the middle of data output time
    SPI1CON1bits.CKE = 0;       // Serial output data changes on transition from
                                // Idle clock state to active clock state
    SPI1CON1bits.CKP = 0;       // Idle state for clock is a low level;
                                // active state is a high level
    SPI1STATbits.SPIEN = 1;     // Enable SPI module

    // Interrupt Controller Settings
    ///IFS0bits.SPI1IF = 0;        // Clear the Interrupt flag
    //IEC0bits.SPI1IE = 1;        // Enable the interrupt
}
void InitUart(void)
{
    //configura los pines
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPOR4bits.RP9R = PIN_U1TX;
    RPINR18bits.U1RXR = PIN_U1RX;
    __builtin_write_OSCCONL(OSCCON | (1<<6));

    //configura el modulo USART
    U1MODEbits.STSEL = 0; // 1-Stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-Data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud disabled
    U1MODEbits.BRGH = 0; // Standard-Speed mode
    U1BRG = BRGVAL; // Baud Rate setting for 9600
    U1STAbits.UTXISEL0 = 0; // Interrupt after one TX character is transmitted
    U1STAbits.UTXISEL1 = 0;
//    IEC0bits.U1TXIE = 1; // Enable UART TX interrupt
    IEC0bits.U1RXIE = 1;
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART T
    __delay_us(110);        //Una vez terminado el delay, la UART esta lista para operar
}
void InitTimer1(void)
{
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TCS = 0; // Select internal instruction cycle clock (FOSC/2)
    T1CONbits.TGATE = 0; // Disable Gated Timer mode

    T1CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    PR1 = 80;   //80: interrumpe con una f de  500000 Hz, 39: Interrumpe cada 2us (todo @ 40Mips)
    TMR1 = 0x00; // Clear timer register

    IPC0bits.T1IP = TMR1_PRIOR_ISR;
    //    IPC0bits.T1IP = 0x01; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // bajo la bandera de interrupcion Timer 1
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt
    T1CONbits.TON = 1; // Start Timer
}
void InitTimer2_3_32bits(void)
{
    T3CONbits.TON = 0; // Stop any 16-bit Timer3 operation
    T2CONbits.TON = 0; // Stop any 16/32-bit Timer2 operation
    T2CONbits.T32 = 1; // Enable 32-bit Timer mode
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR3 = 0x0000; // Clear 32-bit Timer (msw)
    TMR2 = 0x0000;  // Clear 32-bit Timer (lsw)

    //desborda a los 121 ms (tiempo mayor al periodo de la señal de menor frecuencia)
    SET_TMR32_TIMEOUT_DEFAULT_MAX_T();

    IPC2bits.T3IP = 0x05; // Set Timer3 Interrupt Priority Level
    IFS0bits.T3IF = 0; // bajo la bandera de interrupcion Timer 3
    IEC0bits.T3IE = 1; // Enable Timer1 interrupt
    T2CONbits.TON = 1; // Start 32-bit Timer

    /*
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal instruction cycle clock (FOSC/2)
    T2CONbits.TGATE = 0; // Disable Gated Timer mode

    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    PR2 = 0x0000;
    TMR2 = 0x0000;          // Clear timer register

    IPC1bits.T2IP = TMR1_PRIOR_ISR;
    IFS0bits.T2IF = 0; // bajo la bandera de interrupcion Timer 1
    IEC0bits.T2IE = 1; // Enable Timer1 interrupt
    T2CONbits.TON = 0; // Keep inoperative the Timer
    */
}
void InitAdc(void)
{
    //1
    AD1CON1bits.AD12B = 1;      // Select 12-bit mode
    //2
    AD1CON2bits.VCFG = 0;       // Vrefh: AVdd, Vrefl: AVss
    //3
    AD1CON3bits.ADCS = 4;       // TAD = 125 ns (the minimun is 117.6 ns)
    //4
    AD1PCFGLbits.PCFG4=0;
    AD1PCFGLbits.PCFG5=0;
    //5
    AD1CHS0bits.CH0SA = 4;      // Select AN4 for CH0 +ve input
    AD1CHS0bits.CH0NA = 0;      // Select VREF- for CH0 -ve input
    //6
    AD1CON2bits.CHPS = 0;       // Select 1-channel mode (CH0)
    //7
    AD1CON1bits.SIMSAM = 0;     // Disable Simultaneous Sampling
    //8
    AD1CON1bits.ASAM = 0;       // Manual Sampling
    //9
    AD1CON1bits.SSRC = 7;       // Automatic convertion trigger (start convetion after finished the sampling process)
    AD1CON3bits.SAMC = 5;       // 5 TAD as sampling time (minimun is 3 TAD)
    //este no esta en la lista del datasheet
    AD1CON3bits.ADRC = 0;       // Convertion clock derived from system clock
    //10
    AD1CON1bits.FORM = 0;       //uint16
    
    AD1CON2bits.ALTS = 0;       // Disable Alternate Input Selection
    AD1CON2bits.SMPI = 0;       // Interrupt is generated every convertion

    //The sampling frequency is determined by the following expretion:
    //            Fcy
    //fs = ------------------
    //     (SAMC+14)*(ADCS+1)
    //
    //select Positive inputs for MUXs
//    AD1CHS123bits.CH123SA = 0;
//    AD1CHS123bits.CH123NA = 0;

    IPC3bits.AD1IP = ADC_PRIOR_ISR; // Lowest priority interrupt
    IFS0bits.AD1IF = 0;         // Clear interrupt flag
    IEC0bits.AD1IE = 1;         // Enable ADC interrupt
    AD1CON1bits.ADON = 1;       // Start ADC module
}
void InitGlobalVars(void)
{
    uAVATaskSt = 0;
    UartPos = 0;
    SignPhaseInc = 1;
    NewPhaseInc = 1; //1: 8.33' Hz
    InitPhase = 1;
    EndPhase = 2;
    DeltaPhase = 1;
    PhaseAcc = 0;
    PhaseInc = 1;
    Sign = (~NEGATIVO)&0xFF;
}
void InitPorts(void)
{
    //Solo AN4 y AN5 como entradas analogicas, el resto Digitales
    AD1PCFGLbits.PCFG0=1;
    AD1PCFGLbits.PCFG1=1;
    AD1PCFGLbits.PCFG2=1;
    AD1PCFGLbits.PCFG3=1;
    AD1PCFGLbits.PCFG4=0;
    AD1PCFGLbits.PCFG5=0;

    //aux pin
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    //FS como salida y nivel alto
    TRISBbits.TRISB14 = 0;
    FS_UP();

    // External Interrupt 1 on RP6
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPINR0bits.INT1R = 6;
    __builtin_write_OSCCONL(OSCCON | (1<<6));
}
void InitExternalInt(void)
{
    // Rise edge generates interrupts
    INTCON2bits.INT0EP = INT_RISE;
    INTCON2bits.INT1EP = INT_RISE;

    // Configure and enable interrupts
    IPC5bits.INT1IP = INT_PRIOR_ISR+1;
    IPC0bits.INT0IP = INT_PRIOR_ISR;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;
    IFS0bits.INT0IF = 0;
    IEC0bits.INT0IE = 1;
}
void InitDac(void)
{
    InitSpi();
    SPI1BUF = 0xD000;
}
// <<<<<<<<<<<<<<<<< FIN SECCION C

/*
 * SECCION D:
 * 
 * - Vectores de Interrupcion
 */
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void) // code for Timer1 ISR
{
    IFS0bits.T1IF=0;    //bajo la bandera de interrupcion Timer 1
    FS_UP();
    DAC_SAMPLE = qSin[PhaseAcc];
    FS_DW();
    if( Sign == NEGATIVO )  //hay que invertir el signo de la tabla?
        DAC_SAMPLE = 0x0FFF & (~DAC_SAMPLE - 1);   //si, calcula el complemento y resta 1 para lograrlo
    SPI1BUF = DAC_SAMPLE | 0x4000;     //envia la muestra al DAC (el nibble 0x4 es comando para el DAC)
    if( PhaseInc != NewPhaseInc )
    {
        if( NewPhaseInc > PhaseInc )
            PhaseInc++;
        else
            PhaseInc--;
    }
    PhaseAcc += PhaseInc * SignPhaseInc;
    if( PhaseAcc < 0 )
    {
        PhaseAcc *= -1;
        SignPhaseInc *= -1;
        Sign = ~Sign;   //Invierte el signo
    }
    else
        if( PhaseAcc >= qSin_SIZE )
        {
            SignPhaseInc *= -1;
            PhaseAcc = qSin_LESS2 - PhaseAcc;
        }
}
void __attribute__((__interrupt__,__auto_psv__)) _U1RXInterrupt(void)// interrupcion por comunicacion
{
    IFS0bits.U1RXIF = 0; // clear RX interrupt flag
    if( UartPos == UartSize-1 )
        UartPos = 0;
    Uart[UartPos] = U1RXREG;
    if( Uart[UartPos]=='0' || Uart[UartPos]=='1' || Uart[UartPos]=='2' ||
        Uart[UartPos]=='3' || Uart[UartPos]=='4' || Uart[UartPos]=='5' ||
        Uart[UartPos]=='6' || Uart[UartPos]=='7' || Uart[UartPos]=='8' || Uart[UartPos]=='9' )
        UartPos++;
    else
    {
        if( UartPos==0 )
            if( Uart[0]==COM_0 || Uart[0]==COM_1 ||
                Uart[0]==COM_2 || Uart[0]==COM_3 ||
                Uart[0]==COM_4 || Uart[0]==COM_5 )
            {
                UartPos++;
                return;
            }
        if( UartPos!=0 && Uart[UartPos]=='\r' )
        {
            Uart[UartPos] = '\0';
            uint8_t Command = Uart[0];
            Uart[0]='0';
            switch( Command )
            {
                case COM_0:
                    NewPhaseInc = atoi(Uart);
                    break;
                case COM_1:
                    InitPhase = atoi(Uart);
                    break;
                case COM_2:
                    EndPhase = atoi(Uart);
                    break;
                case COM_3:
                    DeltaPhase = atoi(Uart);
                    break;
                case COM_4:
                    break;
                case COM_5:
                    break;
                default:
                    break;
            }
            //just for debug
            LATAbits.LATA0 = 1;
            LATAbits.LATA0 = 0;
        }
        UartPos = 0;
    }
}
void __attribute__((__interrupt__,__auto_psv__)) _ADC1Interrupt(void)
{
    IFS0bits.AD1IF = 0; // clear ADC1 interrupt flag
}
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(void)
{
    IFS0bits.INT0IF = 0;    // Clear INT0 interrupt flag
}
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(void)
{
    IFS1bits.INT1IF = 0; // clear INT1 interrupt flag
}
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)
{
    IFS0bits.T3IF = 0;  // Clear Timer3 Interrupt Flag
}
// <<<<<<<<<<<<<<<<< FIN SECCION D
/*
 * SECCION E:
 * 
 * - Tareas multitasking
 */
void uAVATask(void)
{
    if( uAVATaskSt == 0 )
        return;
}
// <<<<<<<<<<<<<<<<< FIN SECCION E

int main(void)
{
    SetDev40MIPS(); //configura el micro a maxima velocidad (40MIPS)
    InitGlobalVars();
    InitPorts();
    InitTimer2_3_32bits();
    InitAdc();
    InitUart();
    InitTimer1();
    InitDac();
    InitExternalInt();

    while(1)
    {

    }
    return 0;
}
