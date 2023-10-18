//Display
#include "pico/stdlib.h"
#include "ili9341.h"
#include "gfx.h"
#include <string.h>
#include "screen.h"
#include "font.h"
#include "malloc.h"
#include "stdarg.h"
#include <stdio.h>
#include <stdlib.h>
#include "font8x8.h"
#include "sound.h"

//char in buffer size  
#define screenbuffmax 2048
//poll speed for screen updates
#define screenscantimeus 20000

#ifndef swap
#define swap(a, b)     \
        {                  \
                int16_t t = a; \
                a = b;         \
                b = t;         \
        }
#endif

//font defaults for SMALLEST font

//The largest the screen size in chars can be 
//with the smallest font 
#define MAXscreensize_x 52
#define MAXscreensize_y 30

//font  vars
uint8_t fontsel=0;

uint8_t size_x=6;
uint8_t size_y=8;
uint8_t fwx=5;//   number of rows in the font for each character
uint8_t fontmaxchar=127;


//screen size (font depentent)
uint8_t screensize_x=MAXscreensize_x;
uint8_t screensize_y=MAXscreensize_y;

//line buffer width (display width)
int lineBuffWidth=320;   //width of display in px

//char in screen buffer
char screenbuff[screenbuffmax];
//screen buffer pointers
int screenptrout=0;
volatile int screenptrin=0;

//cursor
int csrtimeus=0;
int csron=0;


//screen buffer structure
typedef struct screenChar {
   char c;
   uint16_t fg;
   uint16_t bg;
   
} screenChar ;


//graphics command buffer
#define GraffBuffMAX 40
uint8_t graphBuff[GraffBuffMAX][3];
int graffptrout=0;
volatile int graffptrin=0;
uint8_t TFTlastX=0;
uint8_t TFTlastY=0;


//screen buffer (copy of screen in char and colours)
screenChar txtBuff[MAXscreensize_y][MAXscreensize_x];

//timer for screen (if not polled)
struct repeating_timer screen_timer;

//terminal escape cursor positions.
int posx=0;
int posy=0;
int savedposx=0;
int savedposy=0;
int cursorvisible=1;

//uint16_t lineBuffWidth=0; //buffer width in pixels

//default colours set in inifile
//uint16_t defaultTextFgColor=GFX_RGB565(0,255,0);
//uint16_t defaultTextBgColor=GFX_RGB565(0,0,0);
uint16_t r_textsize_x=1;
uint16_t r_textsize_y=1;

uint16_t defaultTextFgColor=GFX_RGB565(0,255,0);
uint16_t defaultTextBgColor=GFX_RGB565(0,0,0);

uint16_t currentTextBgColour;
uint16_t currentTextFgColour;


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


// escape sequence state
#define ESC_READY               0
#define ESC_ESC_RECEIVED        1
#define ESC_PARAMETER_READY     2

#define MAX_ESC_PARAMS          5

//escape vars
static int esc_state = ESC_READY;
static int esc_parameters[MAX_ESC_PARAMS];
static bool parameter_q;
static int esc_parameter_count;
static unsigned char esc_c1;
static unsigned char esc_final_byte;
static bool rvs = false;
static unsigned char chr_under_csr;



//char line buffer 
uint16_t *lineBuffer = NULL;

void SelectFont(int f){
   fontsel=f;

   if(f==0){
      size_x=6;
      size_y=8;

//screen size (font depentent)
      screensize_x=52;
      screensize_y=30;
      fwx=size_x-1;
      fontmaxchar=127;
  }
  
  if(f==1){
      size_x=8;
      size_y=8;

//screen size (font depentent)
      screensize_x=40;
      screensize_y=30;
      fwx=size_x;
      fontmaxchar=255;
  }
 
}
  
void clear_escape_parameters(){
    for(int i=0;i<MAX_ESC_PARAMS;i++){
        esc_parameters[i]=0;
    }
    esc_parameter_count = 0;
}

void reset_escape_sequence(){
    clear_escape_parameters();
    esc_state=ESC_READY;
    esc_c1=0;
    esc_final_byte=0;
    parameter_q=false;
}

void constrain_cursor_values(){
    if(posx<0) posx=0;
    if(posx>=screensize_x) posx=screensize_x-1;
    if(posy<0) posy=0;
    if(posy>=screensize_y) posy=screensize_y-1;
}

void esc_sequence_received(){
/*
// these should now be populated:
    static int esc_parameters[MAX_ESC_PARAMS];
    static int esc_parameter_count;
    static unsigned char esc_c1;
    static unsigned char esc_final_byte;
*/


int n,m;
if(esc_c1=='['){
    // CSI
    switch(esc_final_byte){
    case 'H':
        // Moves the cursor to row n, column m
        // The values are 1-based, and default to 1

        n = esc_parameters[0];
        m = esc_parameters[1];
        n--;
        m--;

        // these are zero based
        posx = m;
        posy = n;
        constrain_cursor_values();
    break;

    case 'h':
        if(parameter_q && esc_parameters[0]==25){
            // show csr
            cursorvisible=1;
        }
    break;

    case 'l':
        if(parameter_q && esc_parameters[0]==25){
            // hide csr
            cursorvisible=0;
        }
    break;

    case 'm':
        //SGR
        // Sets colors and style of the characters following this code
        //TODO: allows multiple paramters
        if(esc_parameters[0]==0){
            rvs = false;
        }
        if(esc_parameters[0]==7){
            rvs = true;
        }
        if(esc_parameters[0]>=30 && esc_parameters[0]<=37){
            currentTextFgColour = palette[esc_parameters[0]-30];
        }
        if(esc_parameters[0]>=40 && esc_parameters[0]<=47){
            currentTextFgColour = palette[esc_parameters[0]-40];
        }
        if(esc_parameters[0]>=90 && esc_parameters[0]<=97){
            currentTextFgColour = palette[esc_parameters[0]-82]; // 90 is palette[8]
        }
        if(esc_parameters[0]>=100 && esc_parameters[0]<=107){
            currentTextFgColour = palette[esc_parameters[0]-92];  // 100 is palette[8]
        }

        //case 38:
        //Next arguments are 5;n or 2;r;g;b
        if(esc_parameters[0]==38 && esc_parameters[1]==5){
            if(esc_parameters[2]>=0 && esc_parameters[2]<=15){
                currentTextFgColour = palette[esc_parameters[2]];
            }
            if(esc_parameters[2]>=16 && esc_parameters[2]<=231){
                // 16-231:  6 × 6 × 6 cube (216 colors): 16 + 36 × r + 6 × g + b (0 ≤ r, g, b ≤ 5)

                uint16_t cube = esc_parameters[2]-16;
                uint16_t r = cube/36;
                cube -= (r*36);
                uint16_t g = cube/6;
                cube -= (g*6);
                uint16_t b = cube;

                currentTextFgColour = GFX_RGB565(r*42,g*42,b*42);

            }
            if(esc_parameters[2]>=232 && esc_parameters[2]<=255){
                // grayscale from black to white in 24 steps
                uint16_t gre = esc_parameters[2]-232; // 0-24
                gre *= 10;
                currentTextBgColour = GFX_RGB565(gre,gre,gre);
            }

        }
        if(esc_parameters[0]==48 && esc_parameters[1]==5){
            if(esc_parameters[2]>=0 && esc_parameters[2]<=15){
                currentTextBgColour = palette[esc_parameters[2]];
            }
            if(esc_parameters[2]>=16 && esc_parameters[2]<=231){
                // 16-231:  6 × 6 × 6 cube (216 colors): 16 + 36 × r + 6 × g + b (0 ≤ r, g, b ≤ 5)

                uint16_t cube = esc_parameters[2]-16;
                uint16_t r = cube/36;
                cube -= (r*36);
                uint16_t g = cube/6;
                cube -= (g*6);
                uint16_t b = cube;

                currentTextBgColour = GFX_RGB565(r*42,g*42,b*42);
            }
            if(esc_parameters[2]>=232 && esc_parameters[2]<=255){
                // grayscale from black to white in 24 steps
                uint16_t gre = esc_parameters[2]-232; // 0-24
                gre *= 10;
                currentTextBgColour = GFX_RGB565(gre,gre,gre);
            }
        }
        //Next arguments are 5;n or 2;r;g;b
        if(esc_parameters[0]==38 && esc_parameters[1]==2){
            // 2,3,4 = r,g,b
            currentTextFgColour=GFX_RGB565(esc_parameters[2],esc_parameters[3],esc_parameters[4]);
        }
        if(esc_parameters[0]==48 && esc_parameters[1]==2){
            // 2,3,4 = r,g,b
            currentTextBgColour=GFX_RGB565(esc_parameters[2],esc_parameters[3],esc_parameters[4]);
        }

    break;


    case 's':
        // save cursor position
        savedposx = posx;
        savedposy = posy;
    break;
    case 'u':
        // move to saved cursor position
        posx = savedposx;
        posy = savedposy;
    break;

    case 'J':
    // Clears part of the screen. If n is 0 (or missing), clear from cursor to end of screen.
    // If n is 1, clear from cursor to beginning of the screen. If n is 2, clear entire screen
    // (and moves cursor to upper left on DOS ANSI.SYS).
    // If n is 3, clear entire screen and delete all lines saved in the scrollback buffer
    // (this feature was added for xterm and is supported by other terminal applications).
        switch(esc_parameters[0]){
            case 0:
            // clear from cursor to end of screen
//            clear_screen_from_csr();
        break;
            case 1:
            // clear from cursor to beginning of the screen
//            clear_screen_to_csr();
        break;
            case 2:
            // clear entire screen
            DispClear();
        break;
        case 3:
            // clear entire screen
            DispClear();
        break;
        }

    break;

    case 'K':
    // Erases part of the line. If n is 0 (or missing), clear from cursor to the end of the line.
    // If n is 1, clear from cursor to beginning of the line. If n is 2, clear entire line.
    // Cursor position does not change.
        switch(esc_parameters[0]){
            case 0:
            // clear from cursor to the end of the line
 //           clear_line_from_cursor();
        break;
            case 1:
            // clear from cursor to beginning of the line
 //           clear_line_to_cursor();
        break;
            case 2:
            // clear entire line
 //           clear_entire_line();
        break;
        }
    break;


    case 'A':
    // Cursor Up
    //Moves the cursor n (default 1) cells
        n = esc_parameters[0];
        if(n==0)n=1;
        posy -= n;
        constrain_cursor_values();
    break;
    case 'B':
    // Cursor Down
    //Moves the cursor n (default 1) cells
        n = esc_parameters[0];
        if(n==0)n=1;
        posy += n;
        constrain_cursor_values();  // todo: should possibly do a scroll up?
    break;
    case 'C':
    // Cursor Forward
    //Moves the cursor n (default 1) cells
        n = esc_parameters[0];
        if(n==0)n=1;
        posx += n;
        constrain_cursor_values();
    break;
    case 'D':
    // Cursor Backward
    //Moves the cursor n (default 1) cells
        n = esc_parameters[0];
        if(n==0)n=1;
        posx -= n;
        constrain_cursor_values();
    break;

    case 'S':
    // Scroll whole page up by n (default 1) lines. New lines are added at the bottom. (not ANSI.SYS)
        n = esc_parameters[0];
        if(n==0)n=1;
        for(int i=0;i<n;i++){
            scrollup();
        }
    break;

    // MORE



    case 'L':
    // 'INSERT LINE' - scroll rows down from and including cursor position. (blank the cursor's row??)
        n = esc_parameters[0];
        if(n==0)n=1;
//        insert_lines(n);
    break;

    case 'M':
    // 'DELETE LINE' - delete row at cursor position, scrolling everything below, up to fill. Leaving blank line at bottom.
        n = esc_parameters[0];
        if(n==0)n=1;
//        delete_lines(n);
    break;


    }

}
else{
    // ignore everything else
}


// our work here is done
reset_escape_sequence();

}


void clearTxtBuff(){
  int l;
  int x;
  currentTextFgColour=defaultTextFgColor;
  currentTextBgColour=defaultTextBgColor;
  for (l=0;l<screensize_y;l++){
    for(x=0;x<screensize_x;x++){
      txtBuff[l][x].c=' ';
      txtBuff[l][x].fg=defaultTextFgColor;
      txtBuff[l][x].bg=defaultTextBgColor;
    }
  }   
}


bool screen_repeating_timer_callback(struct repeating_timer *t) {
   screenProcess();
}



void screenInit(int poll){
    SelectFont(1);
    clearTxtBuff();
    LCD_initDisplay();
//    LCD_setRotation(1);//normal
    LCD_setRotation(3); //upside down
    quickClearScreen(defaultTextBgColor);
    GFX_setCursor(0, 0);

    //start scan interrupt
    int uf=screenscantimeus;
    if (!poll){
        if(add_repeating_timer_us(-uf, screen_repeating_timer_callback, NULL, &screen_timer)){
            printf("screen timer Started \n\r");
        }  
    }

}

void lineBuffer_drawPixel(int16_t x, int16_t y, uint16_t color,int width){
    lineBuffer[x + y * width] = color ; //(color >> 8) | (color << 8);
}

void lineBuffer_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color){
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep){
        swap(x0, y0); swap(x1, y1);
    }
    if (x0 > x1){
        swap(x0, x1); swap(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1){
        ystep = 1;
    }else{
        ystep = -1;
    }
    for (; x0 <= x1; x0++){
        if (steep){
            lineBuffer_drawPixel(y0, x0, color,lineBuffWidth);
        }else{
            lineBuffer_drawPixel(x0, y0, color,lineBuffWidth);
        }
        err -= dy;
        if (err < 0){
            y0 += ystep;
            err += dx;
        }
    }
}

void lineBuffer_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color){
    lineBuffer_drawLine(x, y, x, y + h - 1, color);
}

void lineBuffer_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    for (int16_t i = x; i < x + w; i++){
        lineBuffer_drawFastVLine(i, y, h, color);
    }
}

//called ONLY with created buffer, either as part of a line or char write
void lb_drawch(char c, uint16_t fg,uint16_t bg,int cpx,int cpy,int lbw){    

    unsigned char line;
    if(fontsel==0){
        for (int8_t i = 0; i < fwx; i++){ // Char bitmap = size_x columns
            line = font[c * fwx + i];
            for (int8_t j = 0; j < size_y; j++, line >>= 1){
                if (line & 1){
                    lineBuffer_drawPixel(cpx + i, cpy + j, fg,lbw);
                }else{
                    lineBuffer_drawPixel(cpx + i, cpy + j, bg,lbw); 
                }
                
            }
        }
    }
    
    if (fontsel==1){
        for (int8_t i = 0; i < fwx; i++){ // Char bitmap = size_x columns
            line = font8x8[c * fwx + i];
            for (int8_t j = 0; j < size_y; j++, line >>= 1){
                if (line & 1){
                    lineBuffer_drawPixel(cpx + j, cpy + i, fg,lbw);
                }else{
                    lineBuffer_drawPixel(cpx + j, cpy + i, bg,lbw);
                }
            }
        }
    }

}

//x,y screen position 
//no chars number of chars in line
//which line from txtBuff 
//only works for 6x8 chars
void WriteTxtLine(int ax,int ay, int bline){
    char c;
    uint16_t fg;
    uint16_t bg;
    lineBuffer = malloc(lineBuffWidth * size_y * sizeof(uint16_t));
    int cx;
    int cpx=0;
    int cpy=0;
//    unsigned char line;
    //clear linebuffer
    for(cx=0;cx<lineBuffWidth * size_y ;cx++) lineBuffer[cx]=bg;
    
    for (cx=0;cx<screensize_x;cx++){
        cpx = cx * size_x;
         
        c=txtBuff[bline][cx].c;
        fg=txtBuff[bline][cx].fg;
        bg=txtBuff[bline][cx].bg;
        
        lb_drawch(c,fg,bg,cpx,0,lineBuffWidth);    

    }
    LCD_WriteBitmap(ax, ay, lineBuffWidth, size_y, lineBuffer);
    free(lineBuffer);
}

void lineBuff_drawChar(int16_t ax, int16_t ay, unsigned char c, uint16_t fg, uint16_t bg){
    
    lineBuffer = malloc(size_x * size_y * sizeof(uint16_t));

//    unsigned char line;
    for(int cx=0;cx<size_x * size_y ;cx++) lineBuffer[cx]=bg;
    
    lb_drawch(c,fg,bg,0,0,size_x);
    
    LCD_WriteBitmap(ax, ay, size_x, size_y, lineBuffer);
    free(lineBuffer);
}

void WriteBlankTxtLine(int ax,int ay,uint16_t bg){
//    uint16_t bg;
    lineBuffer = malloc(lineBuffWidth * size_y * sizeof(uint16_t));
    int cx;
    //clear linebuffer
    for(cx=0;cx<lineBuffWidth * size_y ;cx++) lineBuffer[cx]=bg;
    LCD_WriteBitmap(ax, ay, lineBuffWidth, size_y, lineBuffer);

    free(lineBuffer);
}


void ClearScreen(uint16_t bg){
   clearTxtBuff();
   GFX_setClearColor(bg);
   GFX_clearScreen();
}


void quickClearScreen(uint16_t bg){
//new row of spaces
    int l;
    int x;
    GFX_setCursor(0, 0);

    for (l=0;l<screensize_y;l++){
        for (x=0;x<screensize_x;x++){
            txtBuff[l][x].c=' '; // was l-1 for some reason?
            txtBuff[l][x].fg=bg;
            txtBuff[l][x].bg=bg;
        }
        WriteBlankTxtLine(0,l*size_y,bg);
    }
}


void scrollup(){
    int l;
    int x;
    GFX_setCursor(0, 0);
  
    for (l=1;l<screensize_y;l++){
        for (x=0;x<screensize_x;x++){
            txtBuff[l-1][x].c=txtBuff[l][x].c;
            txtBuff[l-1][x].fg=txtBuff[l][x].fg;
            txtBuff[l-1][x].bg=txtBuff[l][x].bg;
        }
        WriteTxtLine(0,(l-1)*size_y , l);
    }
    
    //new row of spaces
    for(x=0;x<screensize_x;x++){
        txtBuff[screensize_y-1][x].c=' ';
    }
    WriteBlankTxtLine(0,(screensize_y-1)*size_y,defaultTextBgColor);  
}


void charTxtXY(int x, int y, char c,uint16_t fg,uint16_t bg){
   //make a copy for fast screen ops
    txtBuff[y][x].c=c;
    txtBuff[y][x].fg=fg;
    txtBuff[y][x].bg=bg;
    
//    GFX_drawChar(posx*size_x, posy*size_y, c, fg,bg,r_textsize_x, r_textsize_y);
    lineBuff_drawChar(posx*size_x, posy*size_y, c, fg,bg); 

}  

void doCursor(int on){
  csron=on;
  uint16_t cc;
  if(on && cursorvisible){
    cc=defaultTextFgColor;
  }else{
    cc=defaultTextBgColor;
  }
  
  GFX_drawFastHLine(posx*size_x, (posy+1)*size_y-1, size_x, cc);
}


void incCursorY(void){
    posy++;
    if(posy==screensize_y){
        posy--;
        scrollup();
    }
}

void incCursorX(void){
    posx++;
    if (posx==screensize_x){
        posx=0;
        incCursorY();
    }  
}

void DispClear(){
//    GFX_clearScreen();
    clearTxtBuff();
    quickClearScreen(0);
    GFX_setCursor(0, 0);
    posx=0;
    posy=0;
}

void charTxtBuff(char c){
    screenbuff[screenptrin]=c;
    screenptrin++;
    if(screenptrin>=screenbuffmax)screenptrin=0;
}

void stringTxtBuff(char s[]){
        uint8_t n = strlen(s);
        for (int i = 0; i < n; i++)
            charTxtBuff(s[i]);
}

void addToGraffBuff(uint8_t x, uint8_t y, uint8_t cmd){
   graphBuff[graffptrin][0]=x;
   graphBuff[graffptrin][1]=y;
   graphBuff[graffptrin][2]=cmd;
   graffptrin++ ;
   if(graffptrin>=GraffBuffMAX){
      graffptrin=0;
   }   
}

//screen text
//call faster than every 20ms if possible.
void screenProcess(){
    char c;    
    if(screenptrout!=screenptrin){
        doCursor(0);
        while(screenptrout!=screenptrin){
            c=screenbuff[screenptrout];
            screenptrout++;
            if(screenptrout>=screenbuffmax)screenptrout=0;
            doChar(c);
        }
    }
//cursor    
    if(csrtimeus<time_us_32()){
        csrtimeus=time_us_32()+300000;
    
        if(csron){
          doCursor(0);
        }else{
          doCursor(1);  
        }
    }

//graphics    
   if(graffptrout!=graffptrin){
       uint8_t val=graphBuff[graffptrout][2];
       uint8_t command=val & 0xf0;
       uint16_t colour=palette[val & 0x0f];
              //command
       if (command==0x00){   } //move to
       
       if (command==0x10){ GFX_drawPixel(graphBuff[graffptrout][0],graphBuff[graffptrout][1], colour);   } //pixel
                     
       if (command==0x20){ GFX_drawLine(TFTlastX,TFTlastY,graphBuff[graffptrout][0],graphBuff[graffptrout][1], colour);} //line
       
       if (command==0x30){ GFX_drawRect(TFTlastX,TFTlastY,graphBuff[graffptrout][0],graphBuff[graffptrout][1], colour);} //rectangle
       if (command==0x40){ GFX_fillRect(TFTlastX,TFTlastY,graphBuff[graffptrout][0],graphBuff[graffptrout][1], colour);} //filled rectangle

       
       if (command==0x50){
          int radius=abs(TFTlastX-graphBuff[graffptrout][0]);
          GFX_drawCircle(TFTlastX,TFTlastY,radius, colour);// circle, x,y, radius=abs(x-x1)
       } //circle
       if (command==0x60){ 
          int radius=abs(TFTlastX-graphBuff[graffptrout][0]);
          GFX_fillCircle(TFTlastX,TFTlastY,radius, colour);// circle, x,y, radius=abs(x-x1)
       } //circle
       if (command==0x70){
         //x,y = last X,Y - char in x BGcolor in y 
          uint16_t c2=palette[graphBuff[graffptrout][1] & 0x0f];
          char c=graphBuff[graffptrout][0];
          lineBuff_drawChar(TFTlastX,TFTlastY,c, colour,c2);
          graphBuff[graffptrout][0]=TFTlastX+8; //move to next char
          graphBuff[graffptrout][1]=TFTlastY; // keep Y co ord.
       }

       if (command==0xf0){ quickClearScreen(colour);clearTxtBuff();}//clear screen

       TFTlastX=graphBuff[graffptrout][0]; //fg
       TFTlastY=graphBuff[graffptrout][1]; //bg
       graffptrout++;
       if(graffptrout>=GraffBuffMAX){
          graffptrout=0;
       }

   }
    
    
}


void doChar(char asc){

    // handle escape sequences
    if(esc_state != ESC_READY){
        switch(esc_state){
            case ESC_ESC_RECEIVED:
                // waiting on c1 character
                if(asc>='N' && asc<'_'){
                    // 0x9B = CSI, that's the only one we're interested in atm
                    // the others are 'Fe Escape sequences'
                    // usually two bytes, ie we have them already.
                    if(asc=='['){    // ESC+[ =  0x9B){
                        // move forward

                        esc_c1 = asc;
                        esc_state=ESC_PARAMETER_READY;
                        clear_escape_parameters();
                    }
                    // other type Fe sequences go here
                    else{
                        // for now, do nothing
                        reset_escape_sequence();
                    }
                }
                else{
                    // unrecognised character after escape.
                    reset_escape_sequence();
                }
                break;
            case ESC_PARAMETER_READY:
                // waiting on parameter character, semicolon or final byte
                if(asc>='0' && asc<='9'){
                    // parameter value
                    if(esc_parameter_count<MAX_ESC_PARAMS){
                        unsigned char digit_value = asc - 0x30; // '0'
                        esc_parameters[esc_parameter_count] *= 10;
                        esc_parameters[esc_parameter_count] += digit_value;
                    }

                }
                else if(asc==';'){
                    // move to next param
                    esc_parameter_count++;
                    if(esc_parameter_count>MAX_ESC_PARAMS) esc_parameter_count=MAX_ESC_PARAMS;
                }
                else if(asc=='?'){
                    parameter_q=true;
                }
                else if(asc>=0x40 && asc<0x7E){
                    // final byte. Log and handle
                    esc_final_byte = asc;
                    esc_sequence_received();
                }
                else{
                    // unexpected value, undefined
                }
                break;
        }

    }
    else{
        // regular characters -
//        if(asc>=0x20 && asc<0x7f){
        if(asc>=0x20 && asc<=fontmaxchar){

            charTxtXY(posx,posy,asc,currentTextFgColour,currentTextBgColour);
            incCursorX();
          
        }
        //is it esc?
        else if(asc==0x1B){
            esc_state=ESC_ESC_RECEIVED;
        }
        else{
        
            // return, backspace etc
            switch (asc){
                case BSP:
                if(posx>0){
                    posx--;
                }
                break;
                
                case LF:
                  incCursorY();posx=0;
                break;
                
                case CR:
                  posx=0;
                break;                
                
                case FF:
                posx=0;posy=0;DispClear();
                break;
                
                case BELL:
                  Beep(64);
                  sleep_ms(100);
                  Beep(0);

                case TAB:
                  posx=(posx+8) & 0xfff8;

            }

        }

    } // not esc sequence


}

//int screenprintf(const char*, ...)
int screenprintf(const char *format, ...)
{
    va_list args;
    va_start(args,format);
    int done=vprintf(format,args);

    int size = vsnprintf(NULL, 0, format,args);
    char * buff = malloc(size + 1);
    vsprintf(buff, format,args);

    stringTxtBuff(buff);

    va_end(args);

    return done;
}





