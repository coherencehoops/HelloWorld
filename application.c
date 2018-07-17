#include <p24Fxxxx.h>

#include <xc.h>

#pragma config POSCMOD = NONE           // Primary Oscillator Select (Primary oscillator disabled)
#pragma config I2C1SEL = PRI            // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF            // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON            // Primary Oscillator Output Function (OSC2/CLKO/RC15 functions as port I/O (RC15))
#pragma config FCKSM = CSECME           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL           // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))
#pragma config SOSCSEL = SOSC           // Sec Oscillator Select (Default Secondary Oscillator (SOSC))
#pragma config WUTSEL = LEG             // Wake-up timer Select (Legacy Wake-up Timer)
#pragma config IESO = ON                // Internal External Switch Over Mode (IESO mode (Two-Speed Start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = ON              // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx1               // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)


// example interrupt function
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void)
{
}

void setupSPI(void) {
    RPOR6bits.RP13R = 9; // SS1OUT pin
    RPOR7bits.RP14R = 8; // SCK1OUT pin
    RPOR7bits.RP15R = 7; // SDO1 pin

    SPI1CON1 = 0;               // spi control register 1
    SPI1CON1bits.DISSCK = 0;    // internal SPI clock
    SPI1CON1bits.DISSDO = 0;    // module controls data pin
    SPI1CON1bits.MODE16 = 0;    // 8-bit mode
    SPI1CON1bits.SMP = 0;       // sample data at end of output time
    SPI1CON1bits.CKE = 1;       // change data on act->idle clock
    SPI1CON1bits.SSEN = 0;      // no SSEN pin
    SPI1CON1bits.CKP = 0;       // active rising edge
    SPI1CON1bits.MSTEN = 1;     // master mode
    SPI1CON1bits.SPRE = 7;      // 1:1 secondary prescale
    SPI1CON1bits.PPRE = 1;      // 1:16 primary prescale

    SPI1CON2 = 0; // SPI control register 2
}

void writeSPI(char value) {
    while (SPI1STATbits.SPITBF);    //wait until fifo has open space
    SPI1STATbits.SPIROV = 0;        //reset receive flag (unnecessary?)
    SPI1STATbits.SPIEN = 1;         //enable SPI module (don't need to do this every time)
    SPI1BUF = value;                //write value to SPI output buffer
}


int main(int argc, char *argv[])
{
    int i;
    int count;
    unsigned char col = 0x01;
    CLKDIVbits.RCDIV = 0;
    AD1PCFG = 0x9fff;
    
    T1CON = 0;              //reset T1 control
    PR1 = 0xffff;           //set T1 period
    TMR1 = 0;               //clear T1
    T1CONbits.TCKPS1 = 1;   //set prescale select bits
    T1CONbits.TCKPS0 = 0;   //00=1:1, 01=1:8, 10=1:64, 11=1:256
    T1CONbits.TCS = 0;      //set T1 internal clock
    T1CONbits.TON = 1;      //start T1

    TRISB = 0x0000;     //set all portB pins as outputs
    PORTB = 0x0000;     //set all portB pins high

    setupSPI();
    while (1)
    {

        // start frame
        writeSPI(0);
        writeSPI(0);
        writeSPI(0);
        writeSPI(0);

        // a bunch of off LEDs
        for (i=0; i<count; i++){
            writeSPI(0xE0); // global
            writeSPI(0x00); // blue
            writeSPI(0x00); // green
            writeSPI(0x00); // red
        }

        // one bright LED
        writeSPI(0xE2); // global
        writeSPI(0xFF); // blue
        writeSPI(0xFF); // green
        writeSPI(0xFF); // red

        // a bunch of off LEDs
        for (i=172-count; i>=0; i--){
            writeSPI(0xE0); // global
            writeSPI(0x00); // blue
            writeSPI(0x00); // green
            writeSPI(0x00); // red
        }
        count = (count+1)%172;



//        // start frame
//        writeSPI(0);
//        writeSPI(0);
//        writeSPI(0);
//        writeSPI(0);
//
//        // turn on 172 LEDs a solid color
//        for (i=0; i<172; i++){
//            writeSPI(0xFF); //global
//            writeSPI(0x01); //blue
//            writeSPI(0x01); //green
//            writeSPI(0x01); //red
//        }




//        // end frame
//        writeSPI(0xFF);
//        writeSPI(0xFF);
//        writeSPI(0xFF);
//        writeSPI(0xFF);


        // This code waits for a timer interrupt and blinks pins RB14 and RB15
//        IFS0bits.T1IF = 0;
//////        PORTBbits.RB14 = 1;
//////        PORTBbits.RB15 = 0;
////        PORTB = 0xC000;
//        while(!IFS0bits.T1IF);
//        IFS0bits.T1IF = 0;
//////        PORTBbits.RB14 = 0;
//////        PORTBbits.RB15 = 1;
////        PORTB = 0x0000;
//        while(!IFS0bits.T1IF);


//        // wait for timer interrupt
//        IFS0bits.T1IF = 0;
//        while(!IFS0bits.T1IF);
    }

    return 0;
}


