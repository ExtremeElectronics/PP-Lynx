#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "keyboard.h"
#include "ScanCodes.h"

#define PICO_I2C_SDA_PIN 8
#define PICO_I2C_SCL_PIN 9

#define i2c_block i2c0

#define kbd_port 32


////0 normal 1=shift, 2=ctrl, 3=func, 4=alt
#define normal 0
#define shift 1
#define ctrl 2
#define func 3
#define alt 4
#define caps 5
#define capsshift 6

#define displayscancodes 0 //display keyboard scan codes for debugging
 
#define kbdscantimeus 20000

#define repeatdelay 10 //in 20ms's

char lastkey=0;
char repeatcnt=0;

#define kbdbuffmax 100

char kbdbuff[kbdbuffmax];
int kbdptrout=0;
int kbdptrin=0;

uint8_t KbdCaps=0;

//kbd state
uint8_t kbdstate[16];

#define kbdcaps 0x10f
#define kbdf0 0
#define kbdf1 1
#define kbdf2 2
#define kbdf3 3
#define kbdf4 4
#define kbdf5 5
#define kbdf6 6
#define kbdf7 7
#define kbdf8 8
#define kbdf9 9

struct repeating_timer kbd_timer;

//true false is char waiting
int testKbdCharWaiting(void){
  return kbdptrin!=kbdptrout;
}

//char or 0 if no char waiting, doesnt remove from buffer
char testKbdGetCharWaiting(void){
    char c=0;
    if (kbdptrin!=kbdptrout){
        c=kbdbuff[kbdptrout];
    }
}

//adds key to buffer
void kbdBuffIn(char c){
    kbdbuff[kbdptrin]=c;
    kbdptrin++;
    if(kbdptrin>kbdbuffmax)kbdptrin=0;
}

//output char in buffer, removes from buffer
char kbdGetCharWaiting(void){
    char c;
    if(kbdptrin==kbdptrout){
        c=0;
        printf("kbd buffer underrun\n\r");
    }else{  
        c=kbdbuff[kbdptrout];
        kbdptrout++;
        if(kbdptrout>kbdbuffmax)kbdptrout=0;
    }
    return c;
}


uint16_t scanForKey(void){
    int b=0;
    int c=0;
    uint16_t r=0;
    uint8_t  scancode[5]={0xff,0xff,0xff,0xff,0xff};
    int scancodes=0;
    unsigned int keymodifier=0; //1=shift, 2=ctrl, 3=func, 4=alt 

    uint8_t txdata[3];
    uint8_t rxdata[3];

    uint8_t x=0;
    for(b=0;b<8;b++){
      x=1<<b;

      txdata[0]=0x14; //PortA output Latches
      txdata[1]=~x; //port a value
      i2c_write_blocking(i2c_block, kbd_port,&txdata,2,false);

      txdata[0]=0x13; //port b input
      i2c_write_blocking(i2c_block, kbd_port,&txdata,1,true);
      i2c_read_blocking(i2c_block, kbd_port,&rxdata,1,false);
      //port b 0-7 bits keyboard. last bit caps light
      rxdata[0]=rxdata[0] & 0x7f;
      if (rxdata[0]!=0x7f){
        for(c=0;c<7;c++){
          if(!(rxdata[0] & (1<<c))) {
            scancode[scancodes]=8*b+c;
            scancodes++;
          }
        }
      }
    }

    txdata[0]=0x15; //PortB output Latches
    txdata[1]=0; //turn off caps
    if (KbdCaps){
       keymodifier=caps; //overridden by the others
       txdata[1]=0x80; //turn caps LED on
    }
    i2c_write_blocking(i2c_block, kbd_port,&txdata,2,false); //CapsLED
    
    if (scancodes<0xff){
      //check for modifiers
      for(c=0;c<scancodes;c++){

        if (scancode[c]==rshiftkey || scancode[c]==lshiftkey){
          if (!KbdCaps){
            keymodifier=shift;
          }else{
            keymodifier=capsshift; // if in caps and shifted, use caps shift table
          }
          scancode[c]=0xff;
        }

        if (scancode[c]==lctrlkey || scancode[c]==rctrlkey){
          keymodifier=ctrl;
          scancode[c]=0xff;
        }

        if (scancode[c]==funckey){
          keymodifier=func;
          scancode[c]=0xff;
        }
      }


      for(c=0;c<scancodes;c++){
        if(scancode[c]<0xff){
          r=keys[scancode[c]+(64*keymodifier)];
          if(displayscancodes){
            printf("%i %i %i %c \n\r",scancode[c],keymodifier,r,r & 0xff);
          }
        }
      }
    }
    return r;
}

bool kbd_repeating_timer_callback(struct repeating_timer *t) {
  keyboardProcess();
}

void DoKbdF(int f){
    if(f==15){//caps key
      if (KbdCaps==1){
         KbdCaps=0;
      }else{
         KbdCaps=1;
      }  
    
    }else{ //function keys
      if (kbdstate[f]==1){
          kbdstate[f]=0;
      }else{
          kbdstate[f]=1;
      }
    } 
}

void keyboardProcess(void){
    uint16_t c;
    c=scanForKey();
    if(c<=0xff){
      if(c>0){
          if(lastkey==0){
             kbdBuffIn(c);
          }
          repeatcnt++;
          if(repeatcnt>repeatdelay){
            lastkey=0;
            repeatcnt=repeatdelay/2;
          }else{
            lastkey=c;
          //  repeatcnt=0;
          }
      }else{
          lastkey=0;
          repeatcnt=0;
      }
      //return true;
    }else{
       //function (caps=f0)
       if (c>=0x100 && c<=0x10f) {
          if(lastkey==0){
            int ks=c-0x100;
            DoKbdF(ks);
            //printf("F%i[%i]\n",ks,kbdstate[ks]);
            lastkey=c;
         }  
       }  
    }  
}


                  

void kbd_init(int poll){
    printf("KBD init");

    uint8_t txdata[3];
    uint8_t rxdata[3];
    
    //setup ports
    i2c_init(i2c_block, 100 * 1000);
    gpio_set_function(PICO_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C_SDA_PIN);
    gpio_pull_up(PICO_I2C_SCL_PIN);

    
    txdata[0]=0; //Reg 0 - DIRA
    txdata[1]=0; //direction outputs
    i2c_write_blocking(i2c_block, kbd_port,&txdata,2,false);

    txdata[0]=1; //Reg 1 - DIRB
    txdata[1]=0x7f; //direction inputs except Caps LED
    i2c_write_blocking(i2c_block, kbd_port,&txdata,2,false);

    txdata[0]=0x0d; //Reg 1 - Pullups B
//    txdata[1]=0xff; //all pulled up
    txdata[1]=0x7f; //all inputs except Caps LED
    i2c_write_blocking(i2c_block, kbd_port,&txdata,2,false);

    printf("Setup Done\n\r");
    
//start scan interrupt
    int uf=kbdscantimeus;
    if (!poll){
        if(add_repeating_timer_us(-uf, kbd_repeating_timer_callback, NULL, &kbd_timer)){
           printf("KBD Timer Started\n\r");
        }
    }
}

char scanForKeyBlocking(){
  char c=255;
  while(c>0){c=scanForKey();}
  while(c==0){c=scanForKey();}
//  printf("k%i",c);  
  return c;
}

