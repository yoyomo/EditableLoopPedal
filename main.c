/* 
 * Editable Loop Pedal, a project for the Embedded Systems Design (ICOM4217) course, using a dsPIC30F4013.
 * Author: Jose Rodriguez, Armando Ortiz, Edgardo Acosta
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <p30f4013.h>
#define SPACE_LIMIT 500

//Global Variables
int data12bit, data8bit, mixedSignal;
int recording, recorded;
int recordedSignal[SPACE_LIMIT];
int sampleIndex = 0;
int play = 0;
int endIndex = 0;

//Variable used for Timer Period calculation
const int CLOCK_FREQ = 31250;


/**
 * Sets the timer period. Used for counting time passed.
 * @param period the period of the signal (the time to be counted)
 */
void setTimerPeriod(int period){
    /*Calculate the PR1 value equivalent to the desired timer period, taking
     * into account the dsPIC operating frequency*/
    PR1 = (period * (CLOCK_FREQ/4));
}

/**
 * Initializes LCD
 */
void initializeLCD(){
  //To be Completed
}

/**
 * Initializes the timer module.
 */
void initializeTimer(){
    //Reset the timer module
    T1CON = 0;
    
    //Prescaler set to 256 (clock source = 31.25kHz)
    T1CONbits.TCKPS = 3;
    
    /*Set the timer period for counting purposes. Used as the record time
    limit (1 second for now)*/
    setTimerPeriod(1);
    
    //Clear timer interrupt flag, and enables the timer interrupt
    _T1IF = 0;
    _T1IE = 1;
}

/**
 * Initializes all the I/O ports to be used in this program.
 */
void initializePorts(){
    //Configure RB0 as an Analog Input
    TRISBbits.TRISB0 = 1;
    ADPCFGbits.PCFG0 = 0;   
         
    /*Configure 8 output pins for the Digital-to-Analog Converter (might be 
    more later)*/
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB2 = 0;
    
    //Configure 2 output pins for the Record and Play/Pause LEDs.
    TRISFbits.TRISF0 = 0;
    TRISFbits.TRISF1 = 0;
       
    /*Configure a digital input, and enable interrupts on the pin 
     (negative edge triggered) -> Record button*/
    TRISDbits.TRISD8 = 1;
    _INT1EP = 1;
    _INT1IF = 0;
    _INT1IE = 1;
    
    /*Configure a digital input, and enable interrupts on the pin 
     (negative edge triggered) -> Play/Pause button*/
    TRISDbits.TRISD9 = 1;
    _INT2EP = 1;
    _INT2IF = 0;
    _INT2IE = 1;
    
    
    
}

/**
 * Configures the built-in Analog-to-Digital Converter.
 */
void configureADC(){
       /*ADCON1:
    bit 15 ADON: A/D Operating Mode bit
            1 = A/D converter module is operating
            0 = A/D converter is off
    bit 14 Unimplemented: Read as ‘0’
    bit 13 ADSIDL: Stop in Idle Mode bit
            1 = Discontinue module operation when device enters Idle mode
            0 = Continue module operation in Idle mode
    bit 12-10 Unimplemented: Read as ‘0’
    bit 9-8 FORM<1:0>: Data Output Format bits
            11 = Signed fractional (DOUT = sddd dddd dddd 0000)
            10 = Fractional (DOUT = dddd dddd dddd 0000)
            01 = Signed integer (DOUT = ssss sddd dddd dddd)
            00 = Integer (DOUT = 0000 dddd dddd dddd)
    bit 7-5 SSRC<2:0>: Conversion Trigger Source Select bits
            111 = Internal counter ends sampling and starts conversion (auto convert)
            110 = Reserved
            101 = Reserved
            100 = Reserved
            011 = Motor Control PWM interval ends sampling and starts conversion
            010 = General purpose Timer3 compare ends sampling and starts conversion
            001 = Active transition on INT0 pin ends sampling and starts conversion
            000 = Clearing SAMP bit ends sampling and starts conversion
    bit 4-3 Unimplemented: Read as ‘0’
    bit 2 ASAM: A/D Sample Auto-Start bit
            1 = Sampling begins immediately after last conversion completes. SAMP bit is auto set.
            0 = Sampling begins when SAMP bit set
    bit 1 SAMP: A/D Sample Enable bit
            1 = At least one A/D sample/hold amplifier is sampling
            0 = A/D sample/hold amplifiers are holding
            When ASAM = 0, writing ‘1’ to this bit will start sampling.
            When SSRC = 000, writing ‘0’ to this bit will end sampling and start conversion.
    bit 0 DONE: A/D Conversion Status bit
            1 = A/D conversion is done
            0 = A/D conversion is not done
            Clearing this bit will not effect any operation in progress.
            Cleared by software or start of a new conversion.*/
    ADCON1bits.ADSIDL=0;
    ADCON1bits.FORM=0;
    ADCON1bits.SSRC=7;      
    ADCON1bits.SAMP=1;
    /*
    ADCON2:
    bit 15-13 VCFG<2:0>: Voltage Reference Configuration bits
                            A/D VREFH               A/D VREFL
            000     AVDD                    AVSS
            001     External VREF+ pin      AVSS
            010     AVDD                    External VREF- pin
            011     External VREF+ pin      External VREF- pin
            1xx     AVDD                    AVSS
    bit 12 Reserved: User should write ‘0’ to this location
    bit 11 Unimplemented: Read as ‘0’
    bit 10 CSCNA: Scan Input Selections for CH0+ S/H Input for MUX A Input Multiplexer Setting bit
            1 = Scan inputs
            0 = Do not scan inputs
    bit 9-8 Unimplemented: Read as ‘0’
    bit 7 BUFS: Buffer Fill Status bit
            Only valid when BUFM = 1 (ADRES split into 2 x 8-word buffers)
            1 = A/D is currently filling buffer 0x8-0xF, user should access data in 0x0-0x7
            0 = A/D is currently filling buffer 0x0-0x7, user should access data in 0x8-0xF
    bit 6 Unimplemented: Read as ‘0’
    bit 5-2 SMPI<3:0>: Sample/Convert Sequences Per Interrupt Selection bits
            1111 = Interrupts at the completion of conversion for each 16th sample/convert sequence
            1110 = Interrupts at the completion of conversion for each 15th sample/convert sequence
            .....
            0001 = Interrupts at the completion of conversion for each 2nd sample/convert sequence
            0000 = Interrupts at the completion of conversion for each sample/convert sequence
    bit 1 BUFM: Buffer Mode Select bit
            1 = Buffer configured as two 8-word buffers ADCBUF(15...8), ADCBUF(7...0)
            0 = Buffer configured as one 16-word buffer ADCBUF(15...0)
    bit 0 ALTS: Alternate Input Sample Mode Select bit
            1 = Uses MUX A input multiplexer settings for first sample, then alternate between MUX B and MUX A input
            multiplexer settings for all subsequent samples
            0 = Always use MUX A input multiplexer settings*/
    ADCON2bits.VCFG=7;
    ADCON2bits.CSCNA=1;
    ADCON2bits.SMPI=1;
    ADCON2bits.BUFM=0;
    ADCON2bits.ALTS=0;
    /*
    ADCON3:
    bit 15-13 Unimplemented: Read as ‘0’
    bit 12-8 SAMC<4:0>: Auto Sample Time bits
            11111 = 31 TAD
            ·····
            00001 = 1 TAD
            00000 = 0 TAD
    bit 7 ADRC: A/D Conversion Clock Source bit
            1 = A/D internal RC clock
            0 = Clock derived from system clock
    bit 6 Unimplemented: Read as ‘0’
    bit 5-0 ADCS<5:0>: A/D Conversion Clock Select bits
            111111 = TCY/2 • (ADCS<5:0> + 1) = 32 • TCY
            ······
            000001 = TCY/2 • (ADCS<5:0> + 1) = TCY
            000000 = TCY/2 • (ADCS<5:0> + 1) = TCY/2*/
    ADCON3bits.SAMC=31;
    ADCON3bits.ADRC=1;
    ADCON3bits.ADCS=31;
    /*
    ADCHS: A/D Input Select Register
    bit 15-13 Unimplemented: Read as ‘0’
    bit 12 CH0NB: Channel 0 Negative Input Select for MUX B Multiplexer Setting bit
            Same definition as bit <4> (see Note).
    bit 11-8 CH0SB<3:0>: Channel 0 Positive Input Select for MUX B Multiplexer Setting bit
            Same definition as bits <3:0> (see Note).
    bit 7-5 Unimplemented: Read as ‘0’
    bit 4 CH0NA: Channel 0 Negative Input Select for MUX A Multiplexer Setting bit
            1 = Channel 0 negative input is AN1
            0 = Channel 0 negative input is VREFbit
    3-0 CH0SA<3:0>: Channel 0 Positive Input Select for MUX A Multiplexer Setting bit
            1111 = Channel 0 positive input is AN15
            1110 = Channel 0 positive input is AN14
            1101 = Channel 0 positive input is AN13
            ·····
            0001 = Channel 0 positive input is AN1
            0000 = Channel 0 positive input is AN0
    */
    ADCHSbits.CH0NB=0;
    ADCHSbits.CH0NA=0;
    ADCHSbits.CH0SA=0;
    ADCHSbits.CH0SB=0;
    /*ADPCFG: A/D Port Configuration Register
    bit 15-0 PCFG<15:0>: Analog Input Pin Configuration Control bits
            1 = Analog input pin in Digital mode, port read input enabled, A/D input multiplexer input connected to AVSS
            0 = Analog input pin in Analog mode, port read input disabled, A/D samples pin voltage*/
    //ADPCFG=0;
    /*ADCSSL: A/D Input Scan Select Register
    bit 15-0 CSSL<15:0>: A/D Input Pin Scan Selection bits
            1 = Select ANx for input scan
            0 = Skip ANx for input scan*/
   
    
    ADCSSL=0b0000000000000001;  //RB0 as ADC input
    ADCON1bits.ASAM=1;
    IFS0bits.ADIF=1;
    IEC0bits.ADIE=1;
}


/**
 * Interrupt Service Routine for the Record button.
 */
void __attribute__((interrupt,no_auto_psv)) _INT1Interrupt( void )
{
    //Turn off interrupt flag
      _INT1IF = 0;      
      
    //If the system was recording, set flags to stop recording and play the recorded track.
    if(recording == 1){
       recording = 0;
     sampleIndex = 0;
     recorded= 1;
     LATFbits.LATF0 = 0;
     play = 1;
     LATFbits.LATF1 = 1;
     //T1CONbits.TON = 0;  
    }
    else{
        //Set flags for recording, and reset the timer
        recording = 1;
        LATFbits.LATF0 = 1;
        TMR1 = 0x00;
       T1CONbits.TON = 1;  

    }
}

/**
 * Interrupt Service Routine for the Play/Pause button.
 */
void __attribute__((interrupt,no_auto_psv)) _INT2Interrupt( void )
{
    //Turn off the interrupt
    _INT2IF = 0;      
     
    //Toggle the Play/Pause status, and the corresponding LED
    play ^= 0x01; 
    LATFbits.LATF1 = ~LATFbits.LATF1;
}

/**
 * Interrupt Service Routine for the Timer module.
 */
void __attribute__((interrupt,no_auto_psv)) _T1Interrupt( void )
{
    //Turn off interrupt flag
    _T1IF = 0;   
    
    /*If recording, time limit has been reached. Set flags to stop recording 
     and play the recorded signal*/
    if(recording == 1){
    recording = 0;
     sampleIndex = 0;
     recorded= 1;
     LATFbits.LATF0 = 0;
     play = 1;
     LATFbits.LATF1 = 1;
    }
     //T1CONbits.TON = 0;  
    
}

/**
 * Interrupt Service Routine for the Analog-to-Digital Converter sampling/converting process.
 */
void __attribute__((interrupt,no_auto_psv)) _ADCInterrupt( void )
{
    //Read a sample of the analog input from the ADC buffer
     data12bit = ADCBUF0; 
  
     //If the system is recording, save the signal to internal memory.
    if(recording == 1){
        recordedSignal[sampleIndex] = data12bit;
        sampleIndex = (sampleIndex+1)%SPACE_LIMIT;
        endIndex++;

    }
    
    /*If there is a signal recorded, and the Play button is activated, mix both the
     recorded signal and the input signal*/
    if(recorded == 1 && play == 1){
        //Mixing
        mixedSignal = recordedSignal[sampleIndex] + data12bit;
        sampleIndex = (sampleIndex+1)%SPACE_LIMIT;
        
    }
     
    //If not recording, and nothing has been recorded yet, bypass the input signal.
    else
        mixedSignal = data12bit;
    
    //Output the digital signal to the DAC (converting 12-bit to 8-bit, might be changed later)
    data8bit = (int)(mixedSignal * (255.0/4095.0));
    LATB = (data8bit << 2);
    

    //Turn off interrupt flag
    IFS0bits.ADIF = 0;  
    
}

/*
 * Main function of the program.
 * 
 * Pins:
 * RD8 -> Rec Button
 * RD9 -> Play/Pause
 * RB9-RB2 -> D7-D0 DAC
 * RB0 -> Analog Input
 * RF0 -> LED Record
 * RF1 -> LED Play/Pause
 */
int main(int argc, char** argv) {
    //Set ports and peripherals
    initializeLCD();
    initializeTimer();
    initializePorts();
    configureADC();
    
    //Turn on the timer module
    T1CONbits.TON = 1;
    
    //Turn on the ADC module
    ADCON1bits.ADON = 1;
   
    
    while(1){
       //LCD Button Polling (To be completed)
        
    }
    return (EXIT_SUCCESS);
}

