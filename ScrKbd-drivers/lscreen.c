//Display
#include "pico/stdlib.h"
#include "ili9341.h"
#include "gfx.h"
#include <string.h>
#include "lscreen.h"
#include "font.h"
#include "malloc.h"
#include "stdarg.h"
#include <stdio.h>
#include <stdlib.h>
#include "font8x8.h"



#ifndef swap
#define swap(a, b)     \
        {                  \
                int16_t t = a; \
                a = b;         \
                b = t;         \
        }
#endif

//font defaults for SMALLEST font



//VT TEXT colours
uint16_t palette[17] = {
    GFX_RGB565(0   ,0   ,0),          //black
    GFX_RGB565(0xAA,0   ,0),       //red 
    GFX_RGB565(0   ,0xAA,0),       //green
    GFX_RGB565(0xAA,0x55,0),    //brown
    GFX_RGB565(0   ,0   ,0xAA),       //blue
    GFX_RGB565(0xAA,0   ,0xAA),    //Magenta
    GFX_RGB565(0   ,0xAA,0xAA),    //Cyan
    GFX_RGB565(0xAA,0xAA,0xAA), //light Grey 
    GFX_RGB565(0x55,0x55,0x55), //grey
    GFX_RGB565(0xff,0x55,0x55), //bright red
    GFX_RGB565(0x55,0xff,0x55), //bright green
    GFX_RGB565(0xff,0xff,0x55), //yellow
    GFX_RGB565(0   ,0   ,0xff),       //bright blue
    GFX_RGB565(0xff,0x55,0xff), //bright magenta
    GFX_RGB565(0x55,0xff,0xff), //bright cyan
    GFX_RGB565(0xff,0xff,0xff),  //white
// not used for VT but useful colours 
    GFX_RGB565(0xff,0x70,0)//display Amber

};




void screenInit(){
//    clearTxtBuff();
    LCD_initDisplay();
//    LCD_setRotation(0);//normal
    LCD_setRotation(3); //upside down
//    quickClearScreen(defaultTextBgColor);
    GFX_setCursor(0, 0);

//    screenbuffer = (uint16_t *)malloc(screenbuffsize);
//    if(screenbuffer == NULL){
//      printf("Failed to allocate Screen Buffer memory");;
//      while(1);
//    }


}


void ClearScreen(uint16_t bg){
//   clearTxtBuff();
   GFX_setClearColor(bg);
   GFX_clearScreen();
}







