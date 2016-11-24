#include <p30f4013.h>

#include "LCD_4bits.h"
/*


R/W GND
E   RF5
RS  RF4
DB7 RF3
DB6 RF2
DB5 RF1
DB4 RF0

*/

  int wait;
  int charIndex;
  int charValue;
  int init;
/////////////////////// LCD Functions //////////////////////////////////////////
//Controls the LCD's Enable pin. 
void enableSwitch(){
  LATFbits.LATF5 = 1;
  if(init)
    for(wait=500; wait > 0; wait--);
  else
      for(wait=50; wait > 0; wait--);
  LATFbits.LATF5 = 0;
}


//Sets the LCD operation mode to Instruction Mode
void writeInstruction(){
  //LATF &= 0x0F;
  LATFbits.LATF4 = 0;
  enableSwitch();
}


//Sets the LCD operation mode to Data Mode
void writeData(){
  //LATF &= 0x0F;
  LATFbits.LATF4 = 1;
  enableSwitch();
}

void writeHex(int value){
    LATF = (value & 0xF0) >> 4;
    writeData();
    LATF = value & 0x0F;
    writeData();
}

//Writes a character to the LCD display (In 4-bit mode, data is sent in two 
//parts, the Most Significant Nibble first, and then the Least Significant Nibble
void writeCharacter(char letra){
  charValue = (int) letra;
  LATF = (charValue & 0xF0) >> 4;
  writeData();
  LATF = charValue & 0x0F;
  writeData();
  //for(wait=500; wait > 0; wait--); 
}


//Sends a command to the LCD screen
void writeCommand(int command){
  LATF = command;
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
  LATF = 0;
  TRISFbits.TRISF4 = 0;
  TRISFbits.TRISF5 = 0;
  
  TRISFbits.TRISF3 = 0;
  TRISFbits.TRISF2 = 0;
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






