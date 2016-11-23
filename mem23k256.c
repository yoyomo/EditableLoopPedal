/* 
 * File:   23k256SPI.c
 * Author: Jose Antonio
 *
 * Created on November 11, 2016, 2:28 PM
 */

#include <spi.h>



//Defines the possible instructions that can be sent to the memory
#define READ_MODE 0x03
#define WRITE_MODE 0x02
#define READ_SR 0x05
#define WRITE_SR 0x01


//#define FOSC    (7370000ULL)
//#define FCY     (FOSC/2)
//#include <libpic30.h>

//Variable used for dummy reads
unsigned char throwaway;


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
    TRISFbits.TRISF4 = 0;       //Output  RF4/Mem CS
    
    //Deselects the memory (memory in idle mode)
    LATFbits.LATF4 = 1;
    
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
    LATFbits.LATF4 = 0;             //Select the memory
    WriteSPI(WRITE_SR);             //Send the instruction for writing to the memory Status Register
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted
    
    WriteSPI((char)1);              //Send the value for the Status Register (8-bit mode, ignore HOLD pin)
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted
   
    LATFbits.LATF4 = 1;             //Deselect the memory
    
    
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
    unsigned char addressLB = address * 0x00FF;
    
    LATFbits.LATF4 = 0;                 //Select the memory
    WriteSPI(WRITE_MODE);               //Send the instruction for writing to the memory
    while(SPI1STATbits.SPITBF);         //Wait until the data is transmitted
    
    
    WriteSPI(addressHB);                //Send the address MSByte (High Byte)
    while(SPI1STATbits.SPITBF);         //Wait until the data is transmitted
  
    WriteSPI(addressLB);                //Send the address LSByte (Low Byte)
    while(SPI1STATbits.SPITBF);         //Wait until the data is transmitted
   
    WriteSPI(data);                     //Send the data to be written
    while(SPI1STATbits.SPITBF);         //Wait until the data is transmitted
    
    LATFbits.LATF4 = 1;                 //Deselect the memory
    
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
    unsigned char addressLB = address * 0x00FF;
    
    LATFbits.LATF4 = 0;             //Select the memory
    WriteSPI(READ_MODE);            //Send the instruction for reading from memory
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted
    
    WriteSPI(addressHB);            //Send the address MSByte (High Byte)
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted
    
    
    WriteSPI(addressLB);            //Send the address LSByte (Low Byte)
    while(SPI1STATbits.SPITBF);     //Wait until the data is transmitted

    tmp = WriteSPI(0x00);           //Dummy Write to keep the clock running, reads the data sent by the memory
 
    LATFbits.LATF4 = 1;             //Deselect the memory
    return tmp;                     //Returns the data read from memory
}

/**
 * Closes the SPI module.
 */
void mem_close (void)
{
    CloseSPI1();
}