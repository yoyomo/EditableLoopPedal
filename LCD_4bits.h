#include <p30f4013.h>
#include <string.h>

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

#define CURSOR_RIGHT1 0x01
#define CURSOR_RIGHT2 0x04
#define CURSOR_LEFT1 0x01
#define CURSOR_LEFT2 0x00
#define SHIFT_RIGHT1 0x01
#define SHIFT_RIGHT2 0x0C
#define SHIFT_LEFT1 0x01
#define SHIFT_LEFT2 0x08

/*
 * R/W GND
 * E  p2.5
 * RS p2.4
 * DB7 p2.3
 * DB6 p2.2
 * DB5 p2.1
 * DB4 p2.0
 */




/////////////////////// LCD Functions //////////////////////////////////////////
//Controls the LCD's Enable pin. 
void enableSwitch();


//Sets the LCD operation mode to Instruction Mode
void writeInstruction();


//Sets the LCD operation mode to Data Mode
void writeData();


//Writes a character to the LCD display (In 4-bit mode, data is sent in two 
//parts, the Most Significant Nibble first, and then the Least Significant Nibble
void writeCharacter(char letra);


//Sends a command to the LCD screen
void writeCommand(int command);


//Writes a message (set of characters) to the LCD display
void writeMessage(char* message);


void functionSet();
void display();
void clearDisplay();
void entryMode();
void top();
void bottom();
void cursorRight();
void cursorLeft();
void shiftRight();
void shiftLeft();

//LCD initialization sequence
void initLCD();
