/* 
 * Editable Loop Pedal, a project for the Embedded Systems Design (ICOM4217) course, using a dsPIC30F4013.
 * Author: Jose Rodriguez, Armando Ortiz, Edgardo Acosta
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <spi.h>
#include <p30f4013.h>

//#define SPACE_LIMIT 500
#define SPACE_LIMIT 16000
#define PAGE 512
#define NUMBER_OF_TRACKS 2
//Defines the possible instructions that can be sent to the memory
#define READ_MODE 0x03
#define WRITE_MODE 0x02
#define READ_SR 0x05
#define WRITE_SR 0x01

//LCD Command Definition
#define FUNCTION_SET 0x02
#define FUNCTION_SET1 0x02
#define FUNCTION_SET2 0x08
#define DISPLAY1 0x00
#define DISPLAY2 0x0F
#define CLEAR_DISPLAY1 0x00
#define CLEAR_DISPLAY2 0x01
#define ENTRY_MODE1 0x00
#define ENTRY_MODE2 0x06
#define TOP_ROW1 0x08
#define TOP_ROW2 0x00
#define BOTTOM_ROW1 0x0C
#define BOTTOM_ROW2 0x00

#define FIRST_ROW1 0x08
#define FIRST_ROW2 0x00
#define SECOND_ROW1 0x0C
#define SECOND_ROW2 0x00
#define THIRD_ROW1 0x09
#define THIRD_ROW2 0x04
#define FOURTH_ROW1 0x0D
#define FOURTH_ROW2 0x04

#define CURSOR_RIGHT1 0x01
#define CURSOR_RIGHT2 0x04
#define CURSOR_LEFT1 0x01
#define CURSOR_LEFT2 0x00
#define SHIFT_RIGHT1 0x01
#define SHIFT_RIGHT2 0x0C
#define SHIFT_LEFT1 0x01
#define SHIFT_LEFT2 0x08

//Delay Mod
#define FOSC    (7372800ULL)
#define FCY     (FOSC/2)
#include <libpic30.h>

_FWDT(WDT_OFF);
/*******************************************************************************
 ******************************* Global Variables ******************************
 ******************************************************************************/
int data12bit;
unsigned char data8bit, mixedSignal;
int recording[NUMBER_OF_TRACKS], recorded[NUMBER_OF_TRACKS];
unsigned char recordedSignal[NUMBER_OF_TRACKS][PAGE];
int sampleIndex = 0;
int pageLimit;
int play[NUMBER_OF_TRACKS] = {0,0};
int endIndex[NUMBER_OF_TRACKS] = {SPACE_LIMIT / PAGE,SPACE_LIMIT/PAGE};
int ramPointer[NUMBER_OF_TRACKS];
int accessRAM;
int bpm;
double bpmRatio;
int bpmStep = 0;
int setJump;
int vol;
//Variable used for Timer Period calculation
const int CLOCK_FREQ = 31250;
//Array for menu items
char *menu[5] = {
    "Empty Track    ",
    "Recording...   ",
    "Edit           ",
    "Delete         ",
    "Track "
};
int menuPointer;
int recWritten[NUMBER_OF_TRACKS];
int trackWritten[NUMBER_OF_TRACKS];
int emptyWritten[NUMBER_OF_TRACKS];
int i;
//memory
unsigned char throwaway;
//int track;
int offset[NUMBER_OF_TRACKS] = {PAGE,PAGE};
int lastWrite;
int selected;

//LCD
int wait;
int charIndex;
int charValue;
int init;

/*******************************************************************************
 ********************************** Functions **********************************
 ******************************************************************************/
static unsigned char WriteSPI(unsigned char data_out);
void mem_init (void);
void mem_write(unsigned short address, unsigned char data);
unsigned char mem_read(unsigned short address);
void mem_close (void);
void setTimerPeriod(int period);
void setTimerFrequency(int freq);
void initializeLCD();
void initializeTimer();
void initializePorts();
void configureADC();
void reset();

/////////////////////// LCD Functions //////////////////////////////////////////
void enableSwitch();
void writeInstruction();
void writeData();
void writeHex(int value);
void writeCharacter(char letra);
void writeCommand(int command);
void writeMessage(char* message);
void functionSet();
void display();
void clearDisplay();
void entryMode();
void top();
void bottom();
void firstRow();
void secondRow();
void thirdRow();
void fourthRow();
void cursorRight();
void cursorLeft();
void shiftRight();
void shiftLeft();
void initLCD();
void updateMenuPointer();
void updateCursor();
void goUpMenu();
void goDownMenu();

/*******************************************************************************
 ********************************* Interrupts **********************************
 ******************************************************************************/

/**
 * Interrupt Service Routine for the Record button.
 */
void __attribute__((interrupt,no_auto_psv)) _INT1Interrupt( void )
{
     //Turn off interrupt flag
      _INT1IF = 0;      
      
    //If the system was recording, set flags to stop recording and play the recorded track.
    if(recording[menuPointer] == 1){
        if(recorded[menuPointer]==0){ // first time recording, reset sample index to replay
           endIndex[menuPointer] = ramPointer[menuPointer] + 1;
           offset[menuPointer] = sampleIndex;
        }
        
        accessRAM = 1;
        lastWrite = 1; 
    }
    else{
        //if not playing, reset recorded signal
        if(play[menuPointer]==0){
            recording[menuPointer] = 0;
            recorded[menuPointer] = 0;
            recWritten[menuPointer] = 1;
            trackWritten[menuPointer] = 1;
            emptyWritten[menuPointer] = 0;
            endIndex[menuPointer] = SPACE_LIMIT / PAGE;
            offset[menuPointer] = PAGE;
            ramPointer[menuPointer] = 0;
            play[menuPointer] = 0;
        }
        //Set flags for recording, and reset the timer
        recording[menuPointer] = 1;
        LATDbits.LATD0 = 1;
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
    //only if there is something recorded
    if(recorded[menuPointer]==1){
        play[menuPointer] ^= 0x01; 
        LATDbits.LATD1 = ~LATDbits.LATD1;
    }
}

/**
 * Interrupt Service Routine for the Select button.
 */
void __attribute__((interrupt,no_auto_psv)) _INT0Interrupt( void )
{
    //Turn off the interrupt
    _INT0IF = 0;      
     
    selected ^= 0x01; 
    
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
    if(recording[menuPointer] == 1){
        
        if(recorded[menuPointer]==0){ // first time recording, reset sample index to replay
           endIndex[menuPointer] = ramPointer[menuPointer] + 1;
           offset[menuPointer] = sampleIndex;
           
        }
        if(recorded[menuPointer]==0){
           accessRAM = 1;
           lastWrite = 1;
        }
        else{
            recording[menuPointer] = 0;
            recorded[menuPointer]= 1;
            sampleIndex = 0;
            ramPointer[menuPointer] = 0;
            i=0;
            LATDbits.LATD0 = 0;
            play[menuPointer] = 1;
            LATDbits.LATD1 = 1;
        }
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
    data8bit = (unsigned char)(data12bit * (255.0/4095.0));
    //bypass
    mixedSignal = data8bit;
     
    //Get BPM value
    bpm = ADCBUF1;
    
    //Apply BPM value
    if(bpm > 3000){
        bpmStep = 2;
    }
    else if(bpm > 1500){
        bpmStep = 0;
    }
    else{
        if(setJump){
            bpmStep = -1;
            setJump = 0;
        }
    }
    
    //If playing and/or recording update read and/or write variables
    if(play[menuPointer] || recording[menuPointer]){
        //If the system is recording, save the signal to internal memory.
        if(recording[menuPointer]){

            recordedSignal[menuPointer][sampleIndex] = data8bit;
            bpmStep = 0; //ignore knob while writing

        }

        /*If there is a signal recorded, and the Play button is activated, mix both the
     recorded signal and the input signal*/
        if(play[menuPointer]){
            
//            if(selected){
            if(recorded[menuPointer]==1 && !recording[menuPointer]){
                mixedSignal = mixedSignal + recordedSignal[menuPointer][sampleIndex];
            }
//            }
            
        }
        
        if(ramPointer[menuPointer]+1 == endIndex[menuPointer]){
            pageLimit = offset[menuPointer];
        }
        else{
            pageLimit = PAGE;
        }
        if(bpmStep==2){
            sampleIndex = (sampleIndex+bpmStep)%pageLimit;

        }
        else if(bpmStep==-1){
            bpmStep = bpmStep + 1;
        }
        else if(bpmStep==0){
            //increment local sample index
            sampleIndex = (sampleIndex+1)%pageLimit;
            setJump = 1;
        }
        
        if(sampleIndex == 0){
            accessRAM = 1;
        }
    }
     
    
    //Get Volume value
    vol = ADCBUF2;
    
    //Apply volume value
    mixedSignal = (unsigned char)(mixedSignal * (vol/2048.0));
    
    //Output the digital signal to the DAC (not affecting LCD)
    LATBbits.LATB7 = (mixedSignal & 0x80)? 1 : 0;
    LATBbits.LATB6 = (mixedSignal & 0x40)? 1 : 0;
    LATBbits.LATB5 = (mixedSignal & 0x20)? 1 : 0;
    LATBbits.LATB4 = (mixedSignal & 0x10)? 1 : 0;
    LATBbits.LATB3 = (mixedSignal & 0x08)? 1 : 0;
    LATBbits.LATB2 = (mixedSignal & 0x04)? 1 : 0;
    LATBbits.LATB1 = (mixedSignal & 0x02)? 1 : 0;
    LATBbits.LATB0 = (mixedSignal & 0x01)? 1 : 0;
    
//    LATB = mixedSignal | 0x300; // affects LCD like crazy
    

    //Turn off interrupt flag
    IFS0bits.ADIF = 0;  
    
}

/*******************************************************************************
 ************************************ Main *************************************
 ******************************************************************************/
/*
 * Pins:
 * RD0 -> LED Record
 * RD1 -> LED Play/Pause
 * RD2 -> Up Button
 * RD3 -> Down Button
 * RD8 -> Rec Button
 * RD9 -> Play/Pause
 * RB9-RB0 -> D7-D0 DAC
 * RB10 -> Analog Signal Input
 * RB11 -> Analog BPM Input
 * RB12 -> Analog Volume Input
 * RA11 -> Select Button
 * R/W GND
 * E   RB9
 * RS  RB8
 * DB7 RC13
 * DB6 RF4
 * DB5 RF1
 * DB4 RF0

*/

int main(int argc, char** argv) {
    //Set ports and peripherals
    initializeLCD();
   // initializeTimer();
    initializePorts();
    configureADC();
    mem_init();
    
    //Turn on the timer module
    T1CONbits.TON = 1;
    
    //Turn on the ADC module
    ADCON1bits.ADON = 1;
    
    reset();
    clearDisplay();
    menuPointer = 0;

    while(1){
        //polling to write to RAM
        if(accessRAM){
            int i;
            for(i=0;i<PAGE;i++){
                if(ramPointer[menuPointer] + 1 == endIndex[menuPointer]){
                        if(i>offset[menuPointer]) {
                            i = PAGE;
                            break;
                        }
                }
                if(recording[menuPointer]){
                    
                    mem_write(((ramPointer[menuPointer]*PAGE + i) + (menuPointer*SPACE_LIMIT)),
                            recordedSignal[menuPointer][i]);
                    
                }

                if(recorded[menuPointer] && !recording[menuPointer]){
                    recordedSignal[menuPointer][i] = 
                        mem_read((ramPointer[menuPointer]*PAGE + i) + (menuPointer*SPACE_LIMIT));
                }
                
            }
            ramPointer[menuPointer] = (ramPointer[menuPointer] + 1) % endIndex[menuPointer];
            if(ramPointer[menuPointer] == 0){
                sampleIndex = 0;
                ramPointer[menuPointer] = 0;
                if(recorded[menuPointer]==0){
                    lastWrite = 1;
                }
            }
            if(lastWrite){
                recording[menuPointer] = 0;
                recorded[menuPointer]= 1;
                sampleIndex = 0;
                ramPointer[menuPointer] = 0;
                LATDbits.LATD0 = 0;
                play[menuPointer] = 1;
                LATDbits.LATD1 = 1;
                lastWrite = 0;
            }
            
          
            //__delay_us(250);
            accessRAM = 0;
        }
        
        //LCD Button Polling 
        if (!recorded[menuPointer] & !emptyWritten[menuPointer]){
            updateMenuPointer();
            writeMessage(menu[0]);
            emptyWritten[menuPointer] = 1;
            recWritten[menuPointer] = 0;
        }
        // LCD Process #0
        else if(recording[menuPointer] & !recWritten[menuPointer]){
            updateMenuPointer();
            
            writeMessage(menu[1]);
            recWritten[menuPointer] = 1;
            trackWritten[menuPointer] = 0;
        }
        
        // LCD Process #1
        else if(recorded[menuPointer] & !trackWritten[menuPointer]){
            updateMenuPointer();
            writeMessage(menu[4]);
            char buffer[50];
            sprintf(buffer,"%d        ",menuPointer+1);
            writeMessage(buffer);
            
            trackWritten[menuPointer] = 1;
            emptyWritten[menuPointer] = 0;
            
        }
        
        //LCD button check
        // Up
        if(PORTDbits.RD2 == 1){
            
            goUpMenu();
            updateMenuPointer();
            updateCursor();
        }
        //Down
        if(PORTDbits.RD3 == 1){
            goDownMenu();
            updateMenuPointer();
            updateCursor();
        }
        
        
        if(PORTCbits.RC14 == 1){   
            clearDisplay();
            writeMessage("Reseting...    ");
            
            reset();
            
            __delay_ms(100);
        }
        
    
    }
    return (EXIT_SUCCESS);
}

/*******************************************************************************
 ********************************** Functions **********************************
 ******************************************************************************/

/**
 * Writes the given data to the memory.
 * @param data_out the data to be written to memory
 * @return the value returned by the memory
 */
static unsigned char WriteSPI(unsigned char data_out)
{   
    if (SPI1CONbits.MODE16)          /* SPI in 16-bit mode*/
        SPI1BUF = data_out;
    else 
        SPI1BUF = data_out & 0xff;   /*  SPI in 8-bit mode (Byte mode) */
    
    //Wait while the MCU receives data
    while(!DataRdySPI1());
    
    //Return the data received from the memory
    return SPI1BUF;
}

/**
 * Initializes the memory.
 */
void mem_init (void)
{
    //Variables for storing the values of the configuration registers
    unsigned int SPICONValue;
    unsigned int SPISTATValue;
    
    //GPIO configuration for SPI communication
    TRISFbits.TRISF6 = 0;        //Output RF6/SCK1
    TRISFbits.TRISF2 = 1;        //Input  RF2/SDI1
    TRISFbits.TRISF3 = 0;        //Output RF3/SDO1
    TRISFbits.TRISF5 = 0;       //Output  RF5/Mem CS
    
    //Deselects the memory (memory in idle mode)
    LATFbits.LATF5 = 1;
    
    //Sets the configuration values for the SPI1CON register
    SPICONValue =           FRAME_ENABLE_OFF &     //FRMEN:  0 = Framed SPI support disabled
                            FRAME_SYNC_OUTPUT &    //SPIFSD: 0 = Frame sync pulse output (master) 
                            ENABLE_SDO_PIN &       //DISSDO: 0 = SDOx pin is controlled by the module
                            SPI_MODE16_OFF &       //MODE16: 0 = Communication is 8 bits
                            SPI_SMP_OFF &          //SMP:    0
                            SPI_CKE_OFF &          //CKE:    0 = Serial output data changes on transition from Idle clk state to active clk state
                            SLAVE_ENABLE_OFF  &    //SSEN:   0 = SS pin not used by module. Pin controlled by port function
                            CLK_POL_ACTIVE_HIGH &  //CKP:    0 = SS pin not used by module. Pin controlled by port function
                            MASTER_ENABLE_ON &     //MSTEN:  1 = Master mode
                            SEC_PRESCAL_1_1 &      //SPRE<2:0>: Secondary Prescale 1:1
                            PRI_PRESCAL_4_1;       //PPRE<1:0> Primary Prescale 4:1
    
    
    //Sets the configuration values for the SPI1STAT register
    SPISTATValue =          SPI_ENABLE &           //SPIEN:   1 = Enables module and configures SCKx, SDOx, SDIx and SSx as serial port pins
                            SPI_IDLE_CON &         //SPISIDL: 0 = Continue module operation in Idle mode
                            SPI_RX_OVFLOW_CLR;     //SPIROV:  0 = No overflow has occurred. Clear receive overflow bit.
   
    //Starts the SPI module with the given configuration
    OpenSPI1(SPICONValue, SPISTATValue);
    
    //Initialize the Memory's Status Register
    LATFbits.LATF5 = 0;             //Select the memory
    WriteSPI(WRITE_SR);             //Send the instruction for writing to the memory Status Register
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted
    
    WriteSPI((char)1);              //Send the value for the Status Register (8-bit mode, ignore HOLD pin)
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted
   
    LATFbits.LATF5 = 1;             //Deselect the memory
    
    
    //Dummy Read
    throwaway = SPI1BUF;

}

/**
 * Writes the data given to the memory at the given address.
 * @param address the address to write to
 * @param data the data to be 
 */
void mem_write(unsigned short address, unsigned char data)
{
    unsigned char addressHB = (address & 0xFF00) >> 8;
    unsigned char addressLB = address & 0x00FF;
    
    LATFbits.LATF5 = 0;                 //Select the memory
    WriteSPI(WRITE_MODE);               //Send the instruction for writing to the memory
    while(SPI1STATbits.SPITBF);         //Wait until the data is transmitted
    
    
    WriteSPI(addressHB);                //Send the address MSByte (High Byte)
    while(SPI1STATbits.SPITBF);         //Wait until the data is transmitted
  
    WriteSPI(addressLB);                //Send the address LSByte (Low Byte)
    while(SPI1STATbits.SPITBF);         //Wait until the data is transmitted
   
    WriteSPI(data);                     //Send the data to be written
    while(SPI1STATbits.SPITBF);         //Wait until the data is transmitted
    
    LATFbits.LATF5 = 1;                 //Deselect the memory
    
    //Dummy Read
    throwaway = SPI1BUF;
 
}


/**
 * Reads the data from memory at the given address.
 * @param address the address to read from
 * @return the data located in the given address in memory
 */
unsigned char mem_read(unsigned short address)
{
    //Variable to store the data read by the memory
    unsigned int tmp = 0;
    
    unsigned char addressHB = (address & 0xFF00) >> 8;
    unsigned char addressLB = address & 0x00FF;
    
    LATFbits.LATF5 = 0;             //Select the memory
    WriteSPI(READ_MODE);            //Send the instruction for reading from memory
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted
    
    WriteSPI(addressHB);            //Send the address MSByte (High Byte)
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted
    
    
    WriteSPI(addressLB);            //Send the address LSByte (Low Byte)
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted

    tmp = WriteSPI(0x00);           //Dummy Write to keep the clock running, reads the data sent by the memory
 
    LATFbits.LATF5 = 1;             //Deselect the memory
    return tmp;                     //Returns the data read from memory
}

/**
 * Closes the SPI module.
 */
void mem_close (void)
{
    CloseSPI1();
}

/**
 * Sets the timer period. Used for counting time passed.
 * @param period the period of the signal (the time to be counted)
 */
void setTimerPeriod(int period){
    /*Calculate the PR1 value equivalent to the desired timer period, taking
     * into account the dsPIC operating frequency*/
    PR1 = (period * (CLOCK_FREQ/4));
}
void setTimerFrequency(int freq){
    /*Calculate the PR1 value equivalent to the desired timer period, taking
     * into account the dsPIC operating frequency*/
    PR1 = (int) ((1.0/freq) * (CLOCK_FREQ/4.0));
}

/**
 * Initializes LCD
 */
void initializeLCD(){
  //To be Completed
    initLCD();
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
    limit */
    setTimerPeriod(4);
    
    //Clear timer interrupt flag, and enables the timer interrupt
    _T1IF = 0;
    _T1IE = 1;
}

/**
 * Initializes all the I/O ports to be used in this program.
 */
void initializePorts(){
    //Configure RB10 as an Analog Input for Audio Signal
    TRISBbits.TRISB10 = 1;
    ADPCFGbits.PCFG10 = 0;
    
    //Configure RB11 as an Analog Input for BPM Signal
    TRISBbits.TRISB11 = 1;
    ADPCFGbits.PCFG11 = 0;
    
    //Configure RB12 as an Analog Input for Volume Signal
    TRISBbits.TRISB12 = 1;
    ADPCFGbits.PCFG12 = 0;
    
    
    /*Configure 8 output pins for the Digital-to-Analog Converter (might be 
    more later)*/
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB1 = 0;
    TRISBbits.TRISB0 = 0;
    
    //Configure 2 output pins for the Record and Play/Pause LEDs.
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
       
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
    
    /*Up/Down/Select Button Configuration*/
    TRISDbits.TRISD2 = 1; // Up
    TRISDbits.TRISD3 = 1; //Down
    
    /*Configure a digital input, and enable interrupts on the pin 
     (negative edge triggered) -> Select button*/
    TRISAbits.TRISA11 = 1;
    _INT0EP = 0;
    _INT0IF = 0;
    _INT0IE = 1;
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
    ADCON2bits.SMPI=3;
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
   
    
    ADCSSL=0b0001110000000000;  //RB10 & RB11 & RB12 as ADC input
    ADCON1bits.ASAM=1;
    IFS0bits.ADIF=1;
    IEC0bits.ADIE=1;
}

void reset(){
    sampleIndex = 0;
    
    
    accessRAM = 0;
    menuPointer = 0;
    bpm = 0;
    bpmRatio = 0;
    bpmStep = 0;
    vol = 0;
    lastWrite = 0;

    int track;
    for(track = 0; track < NUMBER_OF_TRACKS; track++){
        recording[track] = 0;
        recorded[track] = 0;
        recWritten[track] = 1;
        trackWritten[track] = 1;
        emptyWritten[track] = 0;
        endIndex[track] = SPACE_LIMIT / PAGE;
        offset[track] = PAGE;
        ramPointer[track] = 0;
        play[track] = 0;
    }
}
  
/////////////////////// LCD Functions //////////////////////////////////////////
//Controls the LCD's Enable pin. 
void enableSwitch(){
 LATBbits.LATB9 = 1;
  if(init)
    for(wait=1000; wait > 0; wait--);
  else
      for(wait=100; wait > 0; wait--);
 LATBbits.LATB9 = 0;
}


//Sets the LCD operation mode to Instruction Mode
void writeInstruction(){
  //LATF &= 0x0F;
 LATBbits.LATB8 = 0;
  enableSwitch();
}

//Sends data to LCD
void sendData(int data){
    LATCbits.LATC13 = (data & 0x08) ? 1 :0;
    LATFbits.LATF4 = (data & 0x04) ? 1 :0;
    LATFbits.LATF1 = (data & 0x02) ? 1 :0;
    LATFbits.LATF0 = (data & 0x01) ? 1 :0;
    
}
//Sets the LCD operation mode to Data Mode
void writeData(){
  //LATF &= 0x0F;
 LATBbits.LATB8 = 1;
  enableSwitch();
}

void writeHex(int value){
    int valueHB, valueLB;
    
    valueHB = (value & 0xF0) >> 4;
    sendData(valueHB);
    writeData();
    valueLB = value & 0x0F;
    sendData(valueLB);
    writeData();
}

//Writes a character to the LCD display (In 4-bit mode, data is sent in two 
//parts, the Most Significant Nibble first, and then the Least Significant Nibble
void writeCharacter(char letra){
  int valueHB, valueLB;
  
  charValue = (int) letra;
  valueHB = (charValue & 0xF0) >> 4;
  sendData(valueHB);
  writeData();
  valueLB = charValue & 0x0F;
  sendData(valueLB);
  writeData();
  //for(wait=500; wait > 0; wait--); 
}


//Sends a command to the LCD screen
void writeCommand(int command){
  sendData(command);
  writeInstruction();
}


//Writes a message (set of characters) to the LCD display
void writeMessage(char* message){
  charIndex =0 ;
  while(message[charIndex] != '\0'){
    writeCharacter(message[charIndex++]);
  }
}

void functionSet(){
    writeCommand(FUNCTION_SET1);
    writeCommand(FUNCTION_SET2);
}

void display(){
    writeCommand(DISPLAY1);
    writeCommand(DISPLAY2);
}

void clearDisplay(){
    writeCommand(CLEAR_DISPLAY1);
    writeCommand(CLEAR_DISPLAY2);
}

void entryMode(){
    writeCommand(ENTRY_MODE1);
    writeCommand(ENTRY_MODE2);
}

void top(){
    writeCommand(TOP_ROW1);
    writeCommand(TOP_ROW2);
}
void bottom(){
    writeCommand(BOTTOM_ROW1);
    writeCommand(BOTTOM_ROW2);
}
void firstRow(){
    writeCommand(FIRST_ROW1);
    writeCommand(FIRST_ROW2);
}
void secondRow(){
    writeCommand(SECOND_ROW1);
    writeCommand(SECOND_ROW2);
}
void thirdRow(){
    writeCommand(THIRD_ROW1);
    writeCommand(THIRD_ROW2);
}
void fourthRow(){
    writeCommand(FOURTH_ROW1);
    writeCommand(FOURTH_ROW2);
}

void cursorRight(){
    writeCommand(CURSOR_RIGHT1);
    writeCommand(CURSOR_RIGHT2);
}
void cursorLeft(){
    writeCommand(CURSOR_LEFT1);
    writeCommand(CURSOR_LEFT2);
}
void shiftRight(){
    writeCommand(SHIFT_RIGHT1);
    writeCommand(SHIFT_RIGHT2);
}
void shiftLeft(){
    writeCommand(SHIFT_LEFT1);
    writeCommand(SHIFT_LEFT2);
}

//LCD initialization sequence
void initLCD(){
  TRISBbits.TRISB9 = 0;
  TRISBbits.TRISB8 = 0;
  
  TRISCbits.TRISC13 = 0;
  TRISFbits.TRISF4 = 0;
  TRISFbits.TRISF1 = 0;
  TRISFbits.TRISF0 = 0;
  
  init = 1;
  writeCommand(FUNCTION_SET);
  functionSet();
  functionSet();
  display();
  clearDisplay();
  entryMode();
  init = 0;
}

void updateMenuPointer(){
    if(menuPointer==0) firstRow();
    else if(menuPointer==1) secondRow();
    else if(menuPointer==2) thirdRow();
    else if(menuPointer==3) fourthRow();

}
void updateCursor(){
    int i;
    for(i=0;i<15;i++){
        cursorRight();
    }
}
void goUpMenu(){
    menuPointer = menuPointer - 1;
    if(menuPointer<0) menuPointer = 0;
}
void goDownMenu(){
    menuPointer = menuPointer + 1;
    if(menuPointer >= NUMBER_OF_TRACKS)
        menuPointer = NUMBER_OF_TRACKS - 1 ;
}
