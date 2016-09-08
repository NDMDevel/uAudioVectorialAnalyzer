/* 
 * File:   uAVAmain.c
 * Author: Damian
 *
 * Created on 11 de octubre de 2014, 14:20
 */

//#include "uAVAlib.h"
#include <xc.h>
#include <stdint.h>


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


#include "qSinTable.h"
#include <stdlib.h>

#define FS_DW() LATBbits.LATB14 = 0
#define FS_UP() LATBbits.LATB14 = 1
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
uint32_t Period;
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

uint32_t TMR32;
//#define TMR32H (TMR32>>16)&0x00FF
//#define TMR32L TMR32&0x0000FFFF
//#define SET_TMR32() TMR3=TMR32H;TMR2=TMR32L
uint8_t uAVATaskSt;

#define FCY 40000000
#include <libpic30.h>
#define PI 3.1415926535897932384626433832795

// Select Internal FRC at POR
/*_FOSCSEL(FNOSC_FRC);
// Enable Clock Switching and Configure
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF);
_FWDT(FWDTEN_OFF);*/
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

#define POSITIVO 0xFF
#define NEGATIVO 0x00
uint16_t DAC_SAMPLE;
int32_t PhaseAcc;
int16_t PhaseInc;
int16_t NewPhaseInc;
int16_t InitPhase;
int16_t EndPhase;
int16_t DeltaPhase;
int16_t SignPhaseInc;
uint8_t Sign;
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

#define UartSize 10
char Uart[UartSize];
uint8_t UartPos;
/* UpdateNewPhaseInc:
 * Setea asincronicamente (independientemente del barrido) el valor del
 * incremento de fase del DDS, es decir, setea la variable NewPhaseInc.
 * (Nota: generar saltos en NewPhaseInc muy grandes puede generar problemas
 * en el sistema. Es conveniente no setear un valor mayor a 500 respecto del
 * anterior) */
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
                    uAVATaskSt = 255;
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

#define FRes 8.33333333333333333333333333333333
uint32_t DeltaT;
float Freq;
float Rho;
float Phi;
uint8_t INT0_St;
uint8_t INT1_St;
uint8_t ADC_St;
uint8_t TMR32_St;
uint8_t ErrorFlag;
uint8_t TaskCompleted;
#define false 0
#define true !false

inline void uAVATaskResetSubTasks(void)
{
    TMR32_INT_DISABLED();
    IN_INT_DISABLED();
    OUT_INT_DISABLED();
    ADC_INT_DISABLED();

    ADC_DISABLED();

    IN_INT_RISE();
    OUT_INT_RISE();

    TMR32_OFF();
    CLEAR_TMR32();

    INT0_St = 0;
    INT1_St = 0;
    ADC_St = 0;
    TMR32_St = 0;
    ErrorFlag = false;
    TaskCompleted = false;
}
uint16_t VIN;
uint16_t VOUT;
void __attribute__((__interrupt__,__auto_psv__)) _ADC1Interrupt(void)
{
    IFS0bits.AD1IF = 0; // clear ADC1 interrupt flag
    //just for debug
    LATAbits.LATA0 = ~LATAbits.LATA0;

    if( ADC_St == 255 )
    {
        U1TXREG = (ADC1BUF0>>4) & 0xFF;
        ADC_St = 0;
        return;
    }

    if( ADC_St == 0 )
        return;
    if( ADC_St == 1 )
    {
        if( uAVATaskSt == 6 )
        {
            //ya se muestreo la entrada, ahora en ADCBUF esta VIN
            VIN = ADC1BUF0;
            //ADC_St = 2;
            //configurar todo para tomar muestra en OUT ************************
            CLEAR_TMR32();
//            PR3 = ((0xFFFF0000)&(Period+(Period>>2)))>>16;
//            PR2 = (0x0000FFFF)&(Period+(Period>>2));
            SET_TMR32_TIMEOUT_DEFAULT_MAX_T();
            TMR32_St = 1;
            TMR32_INT_ENABLE();
            TMR32_ON();
            INT1_St = 1;
            OUT_INT_ENABLE();
            ADC_OUT_CONFIG();
            ADC_ENABLE();
            return;
        }
        return;
    }
    if( ADC_St == 2 )
    {
        if( uAVATaskSt == 6 )
        {
            uAVATaskResetSubTasks();
            VOUT = ADC1BUF0;
            TaskCompleted = true;
            return;
        }
        return;
    }
}

void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(void)
{
    //Se produjo flanco de entrada al DUT.
    IFS0bits.INT0IF = 0;    // Clear INT0 interrupt flag
    if( INT0_St == 0 )
        return;
    if( INT0_St == 1 )
    {
        if( uAVATaskSt == 2 )
        {
            //Reinicia TMR32 para medir el periodo
            CLEAR_TMR32();
            INT0_St = 2;
            return;
        }
        if( uAVATaskSt == 4 )
        {
            //comienza a medir desfasaje, entre IN y OUT
            //limpia el TMR32 y habilita INT1 para detener el TMR32
            //cuando el flanco se produzca
            CLEAR_TMR32();
            INT0_St = 2;
            INT1_St = 1;
            OUT_INT_ENABLE();
            return;
        }
        if( uAVATaskSt == 6 )
        {
            //a partir de ahora hay que esperar T/4 y tomar una muestra con el ADC
            TMR32_OFF();
            CLEAR_TMR32();
            PR3 = ((0xFFFF0000)&(Period>>2))>>16;
            PR2 = (0x0000FFFF)&(Period>>2);
            INT0_St = 0;
            TMR32_St = 2;
            IN_INT_DISABLED();
            TMR32_ON();
            return;
        }
        return;
    }
    if( INT0_St == 2 )
    {
        if( uAVATaskSt == 2 )
        {
            //Periodo terminado, almacena el tiempo
            Period = GET_TMR32();
            TMR32_OFF();
            CLEAR_TMR32();
    //        PR3 = ((0xFFFF0000)&(Period>>2))>>16;
    //        PR2 = (0x0000FFFF)&(Period>>2);
            INT0_St = 0;
            TaskCompleted = true;   //tarea terminada
            return;
        }
        if( uAVATaskSt == 4 )
        {
            //esto sucede si el flanco OUT del DUT no sucedio o sucede
            //simultaneamente a este (y por un infinitesimal de tiempo entra aca
            //primero -ademas INT0 tiene mas prioridad de interrupcion que INT1-)
            //de cualquier modo DeltaT = 0
            DeltaT = 0;
            uAVATaskResetSubTasks();
            TaskCompleted = true;
            return;
        }
        return;
    }
    if( INT0_St == 3 )
    {
        if( uAVATaskSt == 2 )
        {

            return;
        }
        return;
    }
    if( INT0_St == 4 )
    {
        if( uAVATaskSt == 2 )
        {

            return;
        }
        return;
    }
}
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(void)
{
    IFS1bits.INT1IF = 0; // clear INT1 interrupt flag
    //Al entrar aca, es porque se detecto un flanco ascendente en la señal de
    //salida del DUT.
    if( INT1_St == 0 )
        return;
    if( INT1_St == 1 )
    {
        if( uAVATaskSt == 4 )
        {
            //detiene el TMR32 y guarda el desfasaje
            TMR32_OFF();
            DeltaT = GET_TMR32();
            uAVATaskResetSubTasks();
            TaskCompleted = true;
            return;
        }
        if( uAVATaskSt == 6 )
        {
//            INT1_St = 0;
//            CLEAR_TMR32();
//            TMR32_St = 2;
//            OUT_INT_DISABLED();
            //a partir de ahora hay que esperar T/4 y tomar una muestra con el ADC
            TMR32_OFF();
            CLEAR_TMR32();
            PR3 = ((0xFFFF0000)&(Period>>2))>>16;
            PR2 = (0x0000FFFF)&(Period>>2);
            INT1_St = 0;
            TMR32_St = 2;
            OUT_INT_DISABLED();
            TMR32_ON();
            return;
        }
        return;
    }
}
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)
{
    IFS0bits.T3IF = 0;  // Clear Timer3 Interrupt Flag
    if( TMR32_St == 0 )
        return;
    if( TMR32_St == 1 )
    {
        if( uAVATaskSt == 2 || uAVATaskSt == 4 )
        {
            //error: no hay señal IN o OUT en DUT  ***************************************
            TMR32_St = 0;
            TMR32_OFF();    //detiene el timer para no generar mas interrupciones innecesarias
            INT0_St = 0;
            INT1_St = 0;
            ErrorFlag = true;
            return;
        }
        if( uAVATaskSt == 6 )
        {
            if( INT0_St == 1 )
            {
                //error, no hay señal IN (esto no deberia estar sucediendo)
                TMR32_St = 0;
                TMR32_OFF();    //detiene el timer para no generar mas interrupciones innecesarias
                INT0_St = 0;
                INT1_St = 0;
                ErrorFlag = true;
                return;
            }
            if( INT1_St == 1 )
            {
                //no se produjo flanco de señal OUT, se asume nivel muy chico
                //por lo que VOUT = 0, se da por termminada la tarea sin errores
                TMR32_St = 0;
                TMR32_OFF();    //detiene el timer para no generar mas interrupciones innecesarias
                INT1_St = 0;
                TaskCompleted = true;
                VOUT = 0;
                return;
            }
            return;
        }
        return;
    }
    if( TMR32_St == 2 )
    {
        if( uAVATaskSt == 6 )
        {
            //ya paso T/4, tomar una muestra con el DAC
            TMR32_INT_DISABLED();
            ADC_INT_ENABLE();
            ADC_St++;

            //ADC GET SAMPLE!!
            AD1CON1bits.SAMP = 1; // Start sampling

            return;
        }
        return;
    }
}

void uAVATask(void)
{
    if( uAVATaskSt == 0 )
        return;
    if( uAVATaskSt == 255 )
    {
        //Estado de configuracion:
        //Al entrar a este estado, la tarea configura el micro apropiadamente
        //para el correcto desarrollo de la tarea.
        //La tarea pasa automaticamente a Estado 1 despues de este estado.
//        TMR32_INT_DISABLED();
//        IN_INT_DISABLED();
//        OUT_INT_DISABLED();
//        IN_INT_RISE();      //configura interrupcion por flanco ascendente
//        OUT_INT_RISE();      //configura interrupcion por flanco ascendente
        NewPhaseInc = InitPhase;    //comienza en 8.33Hz por defecto
        uAVATaskSt = 1;
//        INT0_St = 0;
//        INT1_St = 0;
//        TMR32_St = 0;
        uAVATaskResetSubTasks();
        return;
    }
    if( uAVATaskSt == 1 )
    {
        //Mientras NewPhaseInc sea distinto de PhaseInc, el sistema esta
        //cambiando de frecuencia, por lo que la salida no es estable
        if( NewPhaseInc != PhaseInc )   //freq de salida estable?
            return; //no, sale y espera a que sea estable

        //La freq de salida es estable
        //Configura el micro para esperar flanco de la señal de entrada al DUT
        //y habilita interrupcion por desborde de TMR32
        uAVATaskSt++;
        uAVATaskResetSubTasks();        //reset vars state
        ErrorFlag = false;
        TaskCompleted = false;
        SET_TMR32_TIMEOUT_DEFAULT_MAX_T();  //configura timeout al pasar un periodo de la señal mas lenta
        CLEAR_TMR32();
        TMR32_INT_ENABLE();
        TMR32_ON();
        INT0_St = 1;
//        INT1_St = 0;
        TMR32_St = 1;   //Habilita el TMR32 por si no aparece ningun flanco en IN del DUT
//        IN_INT_RISE();
        IN_INT_ENABLE();
        return;
    }
    if( uAVATaskSt == 2 )
    {
        //Estado en que mide la frecuencia de salida tomando el tiempo entre
        //dos flancos ascendentes

        //chequea si se produjo error
        if( ErrorFlag == true )
        {
            //ERROR: no hay señal IN DUT
            //hacer algo con este error
            uAVATaskResetSubTasks();
            uAVATaskSt = 0;     //apaga la tarea
            return;
        }
        //espera a que el periodo termine de medirse
        if( TaskCompleted == false )
            return;
        //apartir de ahora Periodo contiene el periodo de la señal de salida
        //medidos en Tcy (es decir, el periodo en segundos esta dado por Periodo*Tcy)

        //pasa a estado 3 para medir desfasaje entre IN y OUT del DUT
        uAVATaskResetSubTasks();
        uAVATaskSt = 3;
        return;
    }
    if( uAVATaskSt == 3 )
    {
        //ahora mide la diferencia en tiempo entre la señal OUT e IN

        //configura el TMR32 para interrumpir pasado un ciclo de la señal IN
        //(con un T/4 adicionales como margen de error para poder detectar
        //desfasajes nulos)
        uAVATaskResetSubTasks();
//        PR3 = ((0xFFFF0000)&(Period+(Period>>2)))>>16;
//        PR2 = (0x0000FFFF)&(Period+(Period>>2));
        SET_TMR32_TIMEOUT_DEFAULT_MAX_T();        //esto no estaba
        //Habilita el TMR32 por si no aparece ningun flanco en IN del DUT
        TMR32_St = 1;
        TMR32_ON();
        TMR32_INT_ENABLE();
        INT0_St = 1;
        IN_INT_ENABLE();
        uAVATaskSt = 4;
        return;
    }
    if( uAVATaskSt == 4 )
    {
        //Espera que el flanco de la señal IN del DUT se produzca.
        //sino sucede genera error. Si sucede, automaticamente espera un flanco
        //en OUT del DUT y cuenta el tiempo entre ellos. Si no se
        //produce el flanco OUT del DUT, genera error de timeout
        if( ErrorFlag == true )
        {
            //ERROR de time out
            //hacer algo con este error
            uAVATaskResetSubTasks();
            uAVATaskSt = 0;
            return;
        }
        //espera a que la tarea se complete
        if( TaskCompleted == false )
            return;

        //ahora DeltaT tiene la diferencia en tiempo entre IN y OUT
        uAVATaskResetSubTasks();
        uAVATaskSt = 5;
        return;
    }
    if( uAVATaskSt == 5 )
    {
        //ahora espera el proximo flanco de IN para apartir de ahi tomar
        //una muestra T/4 despues
        uAVATaskResetSubTasks();
        //para prevenir erroes se espera el flanco de IN un tiempo de T + T/4
        //si en ese tiempo no sucede el flanco, se genera error por timeout
//        PR3 = ((0xFFFF0000)&(Period+(Period>>2)))>>16;
//        PR2 = (0x0000FFFF)&(Period+(Period>>2));
        SET_TMR32_TIMEOUT_DEFAULT_MAX_T();  //esto no estaba
        TMR32_St = 1;
        TMR32_INT_ENABLE();
        TMR32_ON();
        INT0_St = 1;
        IN_INT_ENABLE();
        ADC_IN_CONFIG();
        ADC_ENABLE();
        uAVATaskSt = 6;
        return;
    }
    if( uAVATaskSt == 6 )
    {
        if( ErrorFlag == true )
        {
            //no se produjo el flanco en IN
            uAVATaskResetSubTasks();
            uAVATaskSt = 0;
            uAVATaskSt = 0;
            return;
        }
        if( TaskCompleted == false )
            return;

        uAVATaskResetSubTasks();
/*
        U1TXREG = ((Period>>24)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((Period>>16)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((Period>>8)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = (Period&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);

        U1TXREG = ((DeltaT>>24)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((DeltaT>>16)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((DeltaT>>8)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = (DeltaT&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);

        U1TXREG = ((VOUT>>8)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = (VOUT&0xFF);      //L despues
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((VIN>>8)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = (VIN&0xFF);      //L despues
        while(U1STAbits.TRMT == 0);
*/
        uAVATaskSt = 7;
//        return;
    }
    if( uAVATaskSt == 7 )
    {
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((Period>>24)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((Period>>16)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((Period>>8)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = (Period&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);

        U1TXREG = ((DeltaT>>24)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((DeltaT>>16)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = ((DeltaT>>8)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = (DeltaT&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);

        U1TXREG = ((VOUT>>8)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = (VOUT&0xFF);      //L despues
        while(U1STAbits.TRMT == 0);

        U1TXREG = ((VIN>>8)&0xFF);   //H primero
        while(U1STAbits.TRMT == 0);
        U1TXREG = (VIN&0xFF);      //L despues
        while(U1STAbits.TRMT == 0);

        if( InitPhase+DeltaPhase>EndPhase )
            uAVATaskSt = 0;
        else
        {
            InitPhase += DeltaPhase;
            uAVATaskSt = 255;
        }
        return;
    }
}

#define PIN_SDO1 0x0007
#define PIN_SCK1OUT 0x0008
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

#define FP 40000000
#define BAUDRATE 128000
#define BRGVAL ((FP/BAUDRATE)/16)-1
#define PIN_U1TX 0x0003
#define PIN_U1RX 0x0008 //pin RP14
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
    PR1 = 80;   //80: interrumpe con una f de  500000 Hz, 39: Interrumpe cada 1us (todo @ 40Mips)
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

/*
#define DAC_RES 14  //14 bits de resolucion
#define DAC_MAX_VAL pow(2.0,DAC_RES)-1.0
//tarda 231 ms @ 40Mips
void InitTable(void)
{
    uint16_t i;
    float A = DAC_MAX_VAL;
    for( i=0 ; i<3000 ; i++ )
        qSin[i] = A*sin(2.0*PI*((double)i)/11996.0);
}*/

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

    //estado 255 especial de inicializacion de la tarea
    uAVATaskSt = 0;

    //envia 0x55 indicando que el modulo USART y el micro estan operativos
    U1TXREG = 0x55;

    ADC_St = 0;
    while(1)
    {
        uAVATask();
    }
    return 0;
}