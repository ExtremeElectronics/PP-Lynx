/*
 *	PLYNX
 *
 *      Based on Charles Peter Debenham Todd's Pale ESP32VGA
 *      https://github.com/retrogubbins/PaleESP32VGA
 *      Permission for this grotesque hack granted by the owner.
 * and
 *	LIB Z80 Z80 Emnulation - from Alan Cox's - Emulator Kit
 *	https://github.com/EtchedPixels/EmulatorKit/blob/master/libz80/z80.c
 *	Origionaly by Gabriel Gambetta (gabriel.gambetta@gmail.com
 */

#include <stdio.h>

//pico headers
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"

//iniparcer
#include "dictionary.h"
#include "iniparser.h"


//sd card reader
#include "f_util.h"
#include "ff.h"
#include "pico/stdlib.h"
//
#include "hw_config.h"


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <tusb.h>
#include "system.h"
#include "libz80/z80.h"

#include "z80dma.h"
#include "z80dis.h"
#include "ctype.h"

//#define enable_disks 1

//ide file handles
FIL fili;
FIL fild;


//2 pages of RAM/ROM for PICO
#define USERAM 1
#define USEROM 0

//sound 

#define HIGH 1
#define LOW 0

//Lynx keyboard
#include "keys.h"
unsigned char keymap[256];

//screen
#include "ScrKbd-drivers/lscreen.h"
#include "ScrKbd-drivers/gfx/gfx.h"
#include "ScrKbd-drivers/ili9341/ili9341.h"

#define USEBUFFER 1
uint16_t screenbuffer[256*128];


//must be pins on the same slice

#define SPEAKER_PINa 6
#define SPEAKER_PINb 7
//#define soundIO1 7
//#define soundIO2 7
//#define PWMrate 90

//uint PWMslice;


//watch
uint16_t watch= 0x0000;

//debug
bool run_debug = false;

//RAMROM
//lynx rom
#include "LynxRom.c"

//lynx disk rom
#ifdef enable_disks
  #include "LynxDOSRom1-0.c"
#endif

//sd disk
#include "paledisk.h"


byte *bank1;
byte *bank2;
byte *bank3;
byte *diskbuf;
//uint16_t *tftmem;
byte z80ports_in[16];

int interruptfps = 0;
int interruppted = 0;
bool show_redblue = true;
bool use_interrupts = false;
static unsigned long elapsed_t;
int current_diskno = 1;

uint8_t bank_latch = 0x00;
uint8_t video_latch = 0x00;
uint8_t speaker_enable = 0x0;
bool  freeze_z80 = false;
uint8_t Line_Blank = 0;

bool stop_z80 =true;
int speed_mult=1;


//max files for ls on SD card.
#define MaxBinFiles 100
char BinFiles[MaxBinFiles];

//IDE
static int ide =1; //set to 1 to init IDE
struct ide_controller *ide0;

/* Real UART setup*/
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

char usbcharbuf=0;
int hasusbcharwaiting=0;


//serial in circular buffer
#define INBUFFERSIZE 10000
//static char charbufferUART[INBUFFERSIZE];
//#static int charinUART=0;
//static int charoutUART=0;

//usb serial buffer
static char charbufferUSB[INBUFFERSIZE];
static int charinUSB=0;
static int charoutUSB=0;

#define ENDSTDIN 0xFF //non char rx value


//interupts
int int_recalc=0;

//PIO
//int PIOA=0;
//uint8_t PIOAp[]={16,17,18,19,20,21,26,27};

//PICO GPIO
// use regular LED (gpio 25 most likly)
const uint LEDPIN = PICO_DEFAULT_LED_PIN;

//const uint HASSwitchesIO =22;
//
//int HasSwitches=0;
//

//ROM Address Switches
//const uint ROMA13 = 10;
//const uint ROMA14 = 11;
//const uint ROMA15 = 12;

//serial selection
//const uint SERSEL = 13;

//buttons
//const uint DUMPBUT =9;
//const uint AUXBUT =8;
//const uint RESETBUT =7;

//LED
const uint PCBLED =6;


/* use stdio for errors via usb uart */

/* Real UART setup*/
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1


static uint16_t tstate_steps = 500;	/* RC2014 core v peritherals - higher z80 - lower pepherals  */

/* IRQ source that is live in IM2 */
static uint8_t live_irq;

#define IRQ_SIOA	1
#define IRQ_SIOB	2
#define IRQ_CTC		3	/* 3 4 5 6 */

static Z80Context cpu_z80;

volatile int emulator_done;

#define TRACE_MEM	0x000001
#define TRACE_IO	0x000002
#define TRACE_ROM	0x000004
#define TRACE_UNK	0x000008
#define TRACE_CPU	0x000010
#define TRACE_512	0x000020
#define TRACE_RTC	0x000040
#define TRACE_SIO	0x000080
#define TRACE_CTC	0x000100
#define TRACE_CPLD	0x000200
#define TRACE_IRQ	0x000400
#define TRACE_UART	0x000800
#define TRACE_Z84C15	0x001000
#define TRACE_IDE	0x002000
#define TRACE_SPI	0x004000
#define TRACE_SD	0x008000
#define TRACE_PPIDE	0x010000
#define TRACE_COPRO	0x020000
#define TRACE_COPRO_IO	0x040000
#define TRACE_TMS9918A  0x080000
#define TRACE_FDC	0x100000
#define TRACE_PS2	0x200000
#define TRACE_ACIA	0x400000

int trace = 00000;
//static int trace = 00000;

static void reti_event(void);

static unsigned int nbytes;

static void z80_vardump(void)
{
	static uint32_t lastpc = -1;
//	char buf[256];

	nbytes = 0;

	lastpc = cpu_z80.M1PC;
	printf( "%04X: ", lastpc);
//	z80_disasm(buf, lastpc);
	while(nbytes++ < 6)
		printf( "   ");
//	printf( "%-16s ", buf);
	printf( "[ %02X:%02X %04X %04X %04X %04X %04X %04X ]\n",
		cpu_z80.R1.br.A, cpu_z80.R1.br.F,
		cpu_z80.R1.wr.BC, cpu_z80.R1.wr.DE, cpu_z80.R1.wr.HL,
		cpu_z80.R1.wr.IX, cpu_z80.R1.wr.IY, cpu_z80.R1.wr.SP);
	sleep_ms(2);
}


static uint8_t mem_read0(uint16_t A)
{
  // code from Z80io.cpp
  if((bank_latch & 0x10) == 0x00)
  {
    if (A < 0x6000)
    {
//        printf(("Returning ");
//        printf(A);
// if(A == 0xc95)printf(("hitc95 ");//return(0xc3);  //set up an infint loop to wait here whilst VB is loading the RAM
// if(A == 0xc96)return(0x95);
// if(A == 0xc97)return(0x0c);
// if(A == 0x3f62)return(0xc3);  //set up an infint loop to wait here whilst VB is loading the RAM
// if(A == 0x3f63)return(0x62);
// if(A == 0x3f64){printf(("tapload ");return(0x3f);}
        return lynxrom[A];
    }
#ifdef enable_disks
    if(A >= 0xe000 && disk_rom_enabled)
    {
            return dosrom[A-0xE000];   // DISK ROM
    }       
#endif
  }

  if((bank_latch & 0x20) == 0x00)  // BANK 1  user ram
  {
    return bank1[A];
  }

  if((bank_latch & 0x40) == 0x40)   // BANK 2 video 
  {
    if((video_latch & 0x04) == 0x00)
    {
        if(A<0x4000 || (A >= 0x8000 && A < 0xc000))
          return bank2[A % 0x2000];        // mirror
        else //if((A>=0x4000 && A<0x8000) || A >=0xc000)
          return bank2[0x2000+(A % 0x2000)];       // BLUE RED
    }
    if((video_latch & 0x08) == 0x00)
    {
        if(A<0x4000 || (A >= 0x8000 && A < 0xc000))
          return bank3[A % 0x2000];        // mirror
        else //if((A>=0x4000 && A<0x8000) || A >=0xc000)
          return bank3[0x2000+(A % 0x2000)];       // GREEN  ALTGREEN
    }
  }
 
  return 0xff;  

}

static void mem_write0(uint16_t A, uint8_t V)
{
    if((bank_latch & 0x01)==0)
    {      
        bank1[A]=V;
    }
    if((bank_latch & 0x02)==0x02)
    {
      if ((video_latch & 0x04)==0) 
      {
        if(A<0x4000 || (A >= 0x8000 && A < 0xc000))
          bank2[A % 0x2000]=V;        // mirror
        else //if((A>=0x4000 && A<0x8000) || A >=0xc000)
          bank2[0x2000+(A % 0x2000)]=V;       // BLUE RED
 
      }
    }
    if((bank_latch & 0x04)==0x04)
    {
      if ((video_latch & 0x08)==0) 
      {
       if(A<0x4000 || (A >= 0x8000 && A < 0xc000))
          bank3[A % 0x2000]=V;        // mirror
        else //if((A>=0x4000 && A<0x8000) || A >=0xc000)
         bank3[0x2000+(A % 0x2000)]=V;       //AGREEN  GREEN
      }
    }  
}



uint8_t do_mem_read(uint16_t addr, int quiet)
{
      if (watch>0 && addr==watch){        z80_vardump();      }
	return mem_read0(addr);
}

uint8_t mem_read(int unused, uint16_t addr)
{
	static uint8_t rstate = 0;
	uint8_t r = do_mem_read(addr, 0);

	if (cpu_z80.M1) {
		/* DD FD CB see the Z80 interrupt manual */
		if (r == 0xDD || r == 0xFD || r == 0xCB) {
			rstate = 2;
			return r;
		}
		/* Look for ED with M1, followed directly by 4D and if so trigger
		   the interrupt chain */
		if (r == 0xED && rstate == 0) {
			rstate = 1;
			return r;
		}
	}
	if (r == 0x4D && rstate == 1)
		reti_event();
	rstate = 0;
	return r;
}

void mem_write(int unused, uint16_t addr, uint8_t val)
{
	 mem_write0(addr, val);
}


uint8_t z80dis_byte(uint16_t addr)
{
	uint8_t r = do_mem_read(addr, 1);
	printf( "%02X ", r);
	nbytes++;
	return r;
}

uint8_t z80dis_byte_quiet(uint16_t addr)
{
	return do_mem_read(addr, 1);
}

static void z80_trace(unsigned unused)
{
	static uint32_t lastpc = -1;
	char buf[256];

	if ((trace & TRACE_CPU) == 0)
		return;
	nbytes = 0;
	/* Spot XXXR repeating instructions and squash the trace */
	if (cpu_z80.M1PC == lastpc && z80dis_byte_quiet(lastpc) == 0xED &&
		(z80dis_byte_quiet(lastpc + 1) & 0xF4) == 0xB0) {
		return;
	}
	lastpc = cpu_z80.M1PC;
	printf( "%04X: ", lastpc);
	z80_disasm(buf, lastpc);
	while(nbytes++ < 6)
		printf( "   ");
	printf( "%-16s ", buf);
	printf( "[ %02X:%02X %04X %04X %04X %04X %04X %04X ]\n",
		cpu_z80.R1.br.A, cpu_z80.R1.br.F,
		cpu_z80.R1.wr.BC, cpu_z80.R1.wr.DE, cpu_z80.R1.wr.HL,
		cpu_z80.R1.wr.IX, cpu_z80.R1.wr.IY, cpu_z80.R1.wr.SP);
}



// USB char in circular buffer

int intUSBcharwaiting(){
// no interrupt or waiting check so use unblocking getchar, adds to buff if available
    char c = getchar_timeout_us(0); 
    if(c!=ENDSTDIN){
        charbufferUSB[charinUSB]=c;
        charinUSB++;
        if (charinUSB==INBUFFERSIZE){
            charinUSB=0;
        }
    }
    return charinUSB!=charoutUSB;
}

int testUSBcharwaiting(){
   return charinUSB!=charoutUSB;
}

char getUSBcharwaiting(void){
    char c=0;
    if(charinUSB!=charoutUSB){
        c=charbufferUSB[charoutUSB];
        charoutUSB++;
        if (charoutUSB==INBUFFERSIZE){
            charoutUSB=0;
        }
    }else{
        printf("USB Buffer underrun");
    }
  return c;
}

/*
// experimental UART char in circular buffer rx via USB interrupt
void intUARTcharwaiting(void){
//   char c = getchar_timeout_us(0); 
    while (uart_is_readable(UART_ID)) {
        char c =uart_getc(UART_ID);
        charbufferUART[charinUART]=c;
        charinUART++;
        if (charinUART==INBUFFERSIZE){
            charinUART=0;
        }
    }
}

//char waiting test is inbuff=outbuffer?
int testUARTcharwaiting(){
      return charinUART!=charoutUART;
}

char getUARTcharwaiting(void){
    char c=0;
    if(charinUART!=charoutUART){
        c=charbufferUART[charoutUART];
        charoutUART++;
        if (charoutUART==INBUFFERSIZE){
            charoutUART=0;
        }

    }else{
        printf("UART Buffer underrun");
    }
  return c;

}
*/





void recalc_interrupts(void)
{
	int_recalc = 1;
}




/*
 *	Interrupts. We don't handle IM2 yet.
 */


void bitWrite(uint8_t x, char n, char value) {
   if (value)
      x |= (1 << n);
   else
      x &= ~(1 << n);
}

/* LYNX Keyboard Emulation */

//from Z80IO.cpp 

void k_delay(int del);

void pump_key(char k)
{
  int f;
 
//  if key[SDLK_CAPSLOCK]) z80ports_in[0x0080] &= 0xF7;
  if (k == 0x83){ z80ports_in[0] &= 0xEF; k_delay(4); return;}
  if (k == 0x81){ z80ports_in[0] &= 0xDF; k_delay(4); return;}
  if  (k == 0x80){z80ports_in[9] &= 0xFB; k_delay(4); return;}
  if  (k == 0x82){z80ports_in[9] &= 0xDF; k_delay(4); return;}
//  if key[SDLK_ESCAPE]) z80ports_in[0x0080] &= 0xBF;
//  if ((key[SDLK_RSHIFT]) || (key[SDLK_LSHIFT])) z80ports_in[0x0080] &= 0x7F;


    k=tolower (k);
     
  // Real Keyboard Table        
  if (k=='1') z80ports_in[0] &= 0xFE;
  if (k=='!')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[0] &= 0xFE;
  }

  if (k=='3') z80ports_in[1] &= 0xFE; // 01
  if(k=='#')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[1] &= 0xFE;// 3
  }
  if (k=='4') z80ports_in[1] &= 0xFD;
  if(k=='$')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[1] &= 0xFD;// 4
  }
// 02
  if (k=='e') z80ports_in[1] &= 0xFB;
// 04
  if (k=='x') z80ports_in[1] &= 0xF7;
// 08
  if (k=='d') z80ports_in[1] &= 0xEF;
// 10
    if (k=='c') z80ports_in[1] &= 0xDF; // 20
  if (k=='2') z80ports_in[2] &= 0xFE;
  if(k=='\"')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[2] &= 0xFE;// 2
  }

  if (k=='q') z80ports_in[2] &= 0xFD;
  if (k=='w') z80ports_in[2] &= 0xFB;
  if (k=='z') z80ports_in[2] &= 0xF7;
  if (k=='s') z80ports_in[2] &= 0xEF;
  if (k=='a') z80ports_in[2] &= 0xDF;
//  if ((key[SDLK_RCTRL]) || (key[SDLK_LCTRL])) z80ports_in[0x0280] &= 0xBF;
  
  if (k=='5') z80ports_in[3] &= 0xFE;
  if(k=='%')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[3] &= 0xFE;// 5
  }
  if (k=='r') z80ports_in[3] &= 0xFD;
  if (k=='t') z80ports_in[3] &= 0xFB;
  if (k=='v') z80ports_in[3] &= 0xF7;
  if (k=='g') z80ports_in[3] &= 0xEF;
  if (k=='f') z80ports_in[3] &= 0xDF;


  if (k=='6') z80ports_in[4] &= 0xFE;
  if(k=='&')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[4] &= 0xFE;// 6
  }
  if (k=='y') z80ports_in[4] &= 0xFD;
  if (k=='h') z80ports_in[4] &= 0xFB;
  if (k==' ') z80ports_in[4] &= 0xF7;
  if (k=='n') z80ports_in[4] &= 0xEF;
  if (k=='b') z80ports_in[4] &= 0xDF;

  if (k=='7') z80ports_in[5] &= 0xFE;
  if(k=='\'')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[5] &= 0xFE;// 7
  }
  if (k=='8') z80ports_in[5] &= 0xFD;
  if(k=='(')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[5] &= 0xFD;// 8
  }
  if (k=='u') z80ports_in[5] &= 0xFB;
  if (k=='m') z80ports_in[5] &= 0xF7;
  if (k=='j') z80ports_in[5] &= 0xDF;

  if (k=='9') z80ports_in[6] &= 0xFE;
  if(k==')')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[6] &= 0xFE;// 9
  }
  if (k=='i') z80ports_in[6] &= 0xFD;
  if (k=='o') z80ports_in[6] &= 0xFB;
  if (k==',') z80ports_in[6] &= 0xF7;
  if(k=='<')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[6] &= 0xF7;// ,
  }

  if (k=='k') z80ports_in[6] &= 0xDF;

  if (k=='0') z80ports_in[7] &= 0xFE;

  if (k=='p') z80ports_in[7] &= 0xFD;
  if (k=='l') z80ports_in[7] &= 0xFB;

// unsure about these
  if (k=='.') z80ports_in[7] &= 0xF7;
  if(k=='>')
  {
    z80ports_in[0] &= 0x7F;//SHIFT
    z80ports_in[7] &= 0xF7;// .
  }
  if (k==';') z80ports_in[7] &= 0xDF;
  if (k=='+')
  {
    z80ports_in[0] &= 0x7F;//SHIFT ;
    z80ports_in[7] &= 0xDF;
  }
  


  if (k=='-') z80ports_in[8] &= 0xFE;
  // = key is shifted -
  if (k=='=') 
  {
    z80ports_in[0] &= 0x7F;//SHIFT -
    z80ports_in[8] &= 0xFE;
  }

//  if (k=='(') z80ports_in[0x0880] &= 0xFB;
  if (k=='/') z80ports_in[8] &= 0xF7;
  if (k=='?')
  {
    z80ports_in[0] &= 0x7F;//SHIFT /
    z80ports_in[8] &= 0xF7;
  }

  if (k==':') z80ports_in[8] &= 0xDF;
  if (k=='*')
  {
    z80ports_in[0] &= 0x7F;//SHIFT :
    z80ports_in[8] &= 0xDF;
  }

  if (k=='[') z80ports_in[8] &= 0xFB;

// Does this work? gives a n amperrsand
//  if (k=='@') z80ports_in[0x0580] &= 0xEF;


  if (k=='@') z80ports_in[8] &= 0xFD;
  if (k=='\\')
  {
    z80ports_in[0] &= 0x7F;//SHIFT @
    z80ports_in[8] &= 0xFD;
  }


  // Newline Character 13
//  if (k==')') z80ports_in[0x0980] &= 0xFD;  
  if (k==']') z80ports_in[9] &= 0xFD;
  if (k=='\x0d') z80ports_in[9] &= 0xF7;

  if (k=='`' || k == 0x1b) z80ports_in[0] &= 0xBF;  //escape key
  if(k == 0x08) z80ports_in[9] &= 0x02;

 k_delay(4);
}

void k_delay(int del)
{
//  int PUMP_DELAY=4;
  //Make sure the keypress is recognised
  //in future check in the keyboard buffer on the ynx that they key gets there
  
  for(int f=0;f<del;f++)
     //  execz80(1000);
      Z80ExecuteTStates(&cpu_z80, (10000));

  // Clear all keypresses
  z80ports_in[0] = 0xFF;
  z80ports_in[1] = 0xFF;
  z80ports_in[2] = 0xFF;
  z80ports_in[3] = 0xFF;
  z80ports_in[4] = 0xFF;
  z80ports_in[5] = 0xFF;
  z80ports_in[6] = 0xFF;
  z80ports_in[7] = 0xFF;
  z80ports_in[8] = 0xFF;
  z80ports_in[9] = 0xFF;

  //Make sure the keypress is recognised
  //in future check in the keyboard buffer on the ynx that they key gets there
  for(int f=0;f<del;f++)
//     execz80(1000);
    Z80ExecuteTStates(&cpu_z80, (10000)); 
}

void patchrom()
{

  //These are for the TAPE load/save routines
  lynxrom[0xd67]=0xed;  //change Tape Byte output, just return 0 in A ?
  lynxrom[0xd68]=0x00;
  lynxrom[0xd69]=0xc9;  

  lynxrom[0xb65]=0xc9;  //disabled completely - Read Tape Sync
  
  lynxrom[0xcd4]=0xed;  //change Read Bit, just return 1 in A
  lynxrom[0xcd5]=0x01;
  lynxrom[0xcd6]=0xc9;
//  lynxrom[0xcd4]=0xc3;  //change Read Bit, just return 1 in A
//  lynxrom[0xcd5]=0xd4;
//  lynxrom[0xcd6]=0x0c;
  lynxrom[0xc95]=0xc3;  //setup an infint loop to wait here whilst VB is loading the RAM
  lynxrom[0xc96]=0x95;
  lynxrom[0xc97]=0x0c;

  lynxrom[0x3f62]=0xc3; //and again for MLOAD
  lynxrom[0x3f63]=0x62;
  lynxrom[0x3f64]=0x3f;

  
  //Patch Save routine to output OUT 93,x trapped here as SAVE
  //jump back in at 0cfb

  int sav_ad=0xbcb;
  lynxrom[sav_ad+0]=0x20;
  lynxrom[sav_ad+1]=0xf4;
  lynxrom[sav_ad+2]=0x01;//ld BC,0093
  lynxrom[sav_ad+3]=0x93;
  lynxrom[sav_ad+4]=0x00;
  lynxrom[sav_ad+5]=0xed;//out a (c)
  lynxrom[sav_ad+6]=0x79;
  lynxrom[sav_ad+7]=0x00;//never gets to these :)
  lynxrom[sav_ad+8]=0x00;

//  lynxrom[0x8cf]=0xc9;      //dev clear srcreen ld a,b etc bugs araound 08da in 48/96k rom 

}

/*

static void PIOA_init(void){
//init gpio ports
    int a;
    for (a=0;a<8;a++){
       gpio_init(PIOAp[a]);
    }

}

static uint8_t PIOA_read(void){
// set as inputs and read
    int a;
    uint8_t v=1;
    uint8_t r=0;
    //set pullups make input
    for (a=0;a<8;a++){
       gpio_set_dir(PIOAp[a],GPIO_IN);
       gpio_pull_up(PIOAp[a]);
    }
    sleep_us(200);
    //get bits disable pullups
    for (a=0;a<8;a++){   
       if(gpio_get(PIOAp[a]))r=r+v;
       gpio_disable_pulls(PIOAp[a]);
       v=v << 1;
    }
    return r;

}

static void PIOA_write(uint8_t val){
//set as outputs and write
  int a;
  uint8_t v=1;
  for (a=0;a<8;a++){
    gpio_set_dir(PIOAp[a],GPIO_OUT);
    gpio_put(PIOAp[a],v & val); 
    v=v << 1;
  }

}
*/

void toggle_speaker(){
  if(gpio_get(SPEAKER_PINb)){
            gpio_put(SPEAKER_PINa,HIGH);
            gpio_put(SPEAKER_PINb,LOW);
        }else{
            gpio_put(SPEAKER_PINa,LOW);
            gpio_put(SPEAKER_PINb,HIGH);
        }


}


void io_write(int unused, uint16_t Port, uint8_t Value)
{
	 Port = Port & 0xFF;
  switch (Port) 
  {
    case 0x7F:
    {
      bank_latch = Value;

//FIXME  bank latch has bit 2? is read 23 enable
//    if ((value & 0x40)==0x40)     //READ 2 & 3 Enable 
//        update_vid_maps96k();
      
      
      break;
    }
    case 0x80:
    {
      speaker_enable = Value & 0x01;
//      digitalWrite(SPEAKER_PIN,speaker_enable);
//      gpio_put(SPEAKER_PIN,speaker_enable);
//      toggle_speaker();
      video_latch = Value;
      if ((Value & 0x40) && Line_Blank==0)  //FIXME synch with video draw
      {   //Line Blanking monostable - freezes z80 till next scanline end
          Line_Blank=1;
          stop_z80 = true;
      }  
      
      break;
    }
    case 0x84:
    {
      if(speaker_enable == 1)
      {
        if(Value > 32){
//            digitalWrite(SPEAKER_PIN,HIGH);
            gpio_put(SPEAKER_PINa,HIGH);
            gpio_put(SPEAKER_PINb,LOW);
        }else{
//            digitalWrite(SPEAKER_PIN,LOW);
            gpio_put(SPEAKER_PINa,LOW);
            gpio_put(SPEAKER_PINb,HIGH);
        }
      }
    }
  }
#ifdef enable_disks
  if((Port & 0xf0) == 0x50)
      disk_outp(Port, Value);
#endif
  
}

uint8_t io_read(int unused, uint16_t Port)
{
  if((Port & 0x00ff)  == 0x0080)
  {
    int16_t kbdportvalmasked = (Port & 0x0f00);
    int16_t kbdarrno = kbdportvalmasked >> 8;
    return( z80ports_in[kbdarrno] );
  }

#ifdef enable_disks
    if((Port & 0xf0)==0x50)
      return(disk_inp(Port));
#endif
  
  return 0xff;
}

static void poll_irq_event(void)
{
}

static void reti_event(void)
{
}


void dumpPC(Z80Context* z80ctx){
    printf("PC %04x\n",z80ctx->PC);
}



void setup_led(void){
  gpio_init(LEDPIN);
  gpio_set_dir(LEDPIN, GPIO_OUT);
}


void flash_led(int t){
  //flash LED
  gpio_put(LEDPIN, 1);
  sleep_ms(t);
  gpio_put(LEDPIN, 0);
  sleep_ms(t);

}



/*
void WriteRamromToSd(FRESULT fr,char * filename,int writesize,int readfromram){
    if (readfromram){
      printf("\n##### Writing %s to SD from RAM for %04x bytes #####\n\r",filename,writesize);
    }else{
      printf("\n##### Writing %s to SD from ROM for %04x bytes #####\n\r",filename,writesize);
    }
    FIL fil;
    fr = f_open(&fil, filename, FA_WRITE | FA_OPEN_APPEND);
    if (FR_OK != fr && FR_EXIST != fr){
        panic("\nf_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    }else{
      int a;
      char c;
      UINT bw;
      for(a=0;a<writesize;a++){
        printf("%02x ",ram[a]);
        if (readfromram){
          c=ram[a];
        }else{
          c=rom[a];
        }
        fr= f_write(&fil, &c, sizeof c, &bw);
      }
    }
    fr = f_close(&fil);
}
*/

/*
void ReadSdToRamrom(FRESULT fr,const char * filename,int readsize,int SDoffset,int writetoram ){
    FIL fil;
    fr = f_open(&fil, filename, FA_READ); //FA_WRITE
    if (FR_OK != fr && FR_EXIST != fr){
        panic("\nf_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    }else{
      fr = f_lseek(&fil, SDoffset);

      int a;
      char c;
      UINT br;
      for(a=0;a<readsize;a++){
        fr = f_read(&fil, &c, sizeof c, &br);
        if (br==0){printf("Read Fail");}
        if(writetoram){
          ram[a]=c;
        }else{
          rom[a]=c;
        }
      }

    }
    fr = f_close(&fil);

}
*/
/*
void WriteRamToSD(FRESULT fr,const char * filename,int readsize ){
    char temp[128];
    sprintf(temp,"\n###### Writing %s to SD from RAM  for %04x bytes#####\n\r",filename,readsize);
    PrintToSelected(temp,1);
    FIL fil;
    fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS); //FA_WRITE
    if (FR_OK != fr && FR_EXIST != fr){
        panic("\nf_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    }else{

      int a;
      char c;
      UINT bw;
      for(a=0;a<readsize;a++){
        c=ram[a];
        fr = f_write(&fil, &c, sizeof c, &bw);
        if (bw==0){printf("Write Fail");}
      }

    }
    fr = f_close(&fil);

}


void CopyRamRom2Ram(int FromAddr, int ToAddr, int copysize, int fromram, int toram){
    int a;
    for(a=0;a<copysize;a++){
      if(fromram){
        if(toram){
          ram[a+ToAddr]=ram[a+FromAddr];
        }else{
          ram[a+ToAddr]=rom[a+FromAddr];
        }
      }else{
        if(toram){
          rom[a+ToAddr]=ram[a+FromAddr];
        }else{
          rom[a+ToAddr]=rom[a+FromAddr];
        }
      }
    }
}


void DumpRamRom(unsigned int FromAddr, int dumpsize,int dram){
  int rc=0;
  int a;
  if(dram){
    printf("RAM %04X ",FromAddr);
  }else{
    printf("ROM %04X ",FromAddr);
  }
  for(a=0;a<dumpsize;a++){
    if (dram){
      printf("%02x ",ram[a+FromAddr]);
    }else{
      printf("%02x ",rom[a+FromAddr]);
    }
      rc++;

      if(rc % 32==0){
        printf("\nR-M %04x ",rc+FromAddr);
      }else{
        if (rc % 8==0){
          printf(" ");
        }
      }
  }

}

*/




void DessembleMemoryUSB(unsigned int FromAddr, int dumpsize){
   int a=0;
   char buf[32];
   while(a<dumpsize){
       printf( "%04X: ", a+FromAddr);
       //c=mem_read0(a+FromAddr);
       nbytes=0;
       z80_disasm(buf, a+FromAddr);
       printf("%s\n\r",buf);
       a=a+nbytes;
   }

}


void DumpScreen(byte * bank,int dumpsize,int FromAddr,int rowwidth){
  int rc=0;
  int a;
  printf("%04X ",FromAddr);
  
  for(a=0;a<dumpsize;a++){
      printf("%02x ",bank[a+FromAddr]);
      rc++;

      if(rc % rowwidth==0){
        printf("\n%04x ",rc+FromAddr);
      }else{
//        if (rc % 8==0){
//          printf(" ");
//        }
      }
  }

}



#ifdef USEBUFFER


void DumpHalfScreenToLCD(int startline){
      int memcnt=0;
      for (int lin = 0;lin < 120;lin++)
      {
        int16_t coladdr = 0;
        for(int col = 0;col < 32;col++)
        {
//          uint16_t bytaddr =  (lin+startline) * 32 + col;  //startlinebyteaddr +
          uint16_t bytaddr =  (lin+startline+4) * 32 + col;  //startlinebyteaddr +
          byte redbyte = bank2[bytaddr + 0x2000];
          byte bluebyte = bank2[bytaddr];
          byte greenbyte = bank3[bytaddr + 0x2000];
          for(int bb = 0;bb < 8;bb++)
          {
            char dored = 0;
            char dogreen = 0;
            char doblue = 0;

            byte bitpos = (0x80 >> bb);
            if(show_redblue || ((video_latch & 0x04) == 0))     // emulate level 9 video show banks from video latch
            {
              if((redbyte & bitpos) != 0)  dored = 255;
              if((bluebyte & bitpos) != 0) doblue = 255;
            }
            if((greenbyte & bitpos) != 0) dogreen = 255;

            screenbuffer[memcnt]=GFX_RGB565(dored,dogreen,doblue);
            memcnt++;
          }
        }
      }

      LCD_WriteBitmap(0, startline,256,120, screenbuffer);


}


void DumpScreenToLCD(){
    DumpHalfScreenToLCD(0);
    DumpHalfScreenToLCD(120);

/*      int memcnt=0;
      for (int lin = 0;lin < 248;lin++)
      {
        int16_t coladdr = 0;
        for(int col = 0;col < 32;col++)
        {
          uint16_t bytaddr =  lin * 32 + col;  //startlinebyteaddr +
          byte redbyte = bank2[bytaddr + 0x2000];
          byte bluebyte = bank2[bytaddr];
          byte greenbyte = bank3[bytaddr + 0x2000];
          for(int bb = 0;bb < 8;bb++)
          {
            char dored = 0;
            char dogreen = 0;
            char doblue = 0;

            byte bitpos = (0x80 >> bb);
            if(show_redblue || ((video_latch & 0x04) == 0))     // emulate level 9 video show banks from video latch
            {
              if((redbyte & bitpos) != 0)  dored = 255;
              if((bluebyte & bitpos) != 0) doblue = 255;
            }
            if((greenbyte & bitpos) != 0) dogreen = 255;

            screenbuffer[memcnt]=GFX_RGB565(dored,dogreen,doblue);
            memcnt++;
          }
        }
      }

      LCD_WriteBitmap(0, 0,256,256, screenbuffer);
*/      
}



#else

#pragma message "USING BUFFER "

void DumpScreenToLCD(){
 
      for (int lin = 0;lin < 248;lin++)
      {
        int16_t coladdr = 0;
        for(int col = 0;col < 32;col++)  
        {
          uint16_t bytaddr =  lin * 32 + col;  //startlinebyteaddr +
          byte redbyte = bank2[bytaddr + 0x2000];
          byte bluebyte = bank2[bytaddr];
          byte greenbyte = bank3[bytaddr + 0x2000];
          for(int bb = 0;bb < 8;bb++)
          {
            char dored = 0;
            char dogreen = 0;
            char doblue = 0;
            
            byte bitpos = (0x80 >> bb);
            if(show_redblue || ((video_latch & 0x04) == 0))     // emulate level 9 video show banks from video latch
            {
              if((redbyte & bitpos) != 0)  dored = 255;
              if((bluebyte & bitpos) != 0) doblue = 255;
            }
            if((greenbyte & bitpos) != 0) dogreen = 255;
            
//            vga.dotFast(col * 8 + bb, lin,vga.RGB(dored,dogreen,doblue)) ;          
            GFX_drawPixel(col*8+bb, lin, GFX_RGB565(dored,dogreen,doblue));
//            LCD_WritePixel(col*8+bb, lin, GFX_RGB565(dored,dogreen,doblue));
          }
        }
      } 
}


#endif



//dumpmemory to USB for serial dump command
/*
void DumpMemoryUSB(unsigned int FromAddr, int dumpsize){
    int rc=0;
    int a;
    const unsigned char width=16;
    unsigned char c;
    char string[width+1];
    string[width]=0;
    printf("%04X ",FromAddr);
    for(a=0;a<dumpsize;a++){
       c=mem_read0(a+FromAddr);
       printf("%02x ",c);
       string[rc % width]='.';
       if ((c>=' ') && (c<='~')) string[rc % width ]=c;
       rc++;
       if(rc % width==0){
           printf(" %s ",string);
           printf("\r\n%04X ",rc+FromAddr);
       }else{
           if (rc % 8==0){
              printf(" ");
           }
       }
    }

}


// dump memory image to console and sd if fr is set
void DumpMemory(unsigned int FromAddr, int dumpsize,FRESULT fr){
  int rc=0;
  int a;
  char temp[128];
  if(romdisable){
        sprintf(temp,"\n\rROM Disabled\n\r");
        PrintToSelected(temp,0);
  }else{
        sprintf(temp,"\n\rROM Enabled\n\r");
        PrintToSelected(temp,0);
  }
  sprintf(temp,"MEM %04X ",FromAddr);
  PrintToSelected(temp,0);
  if (fr!=0) WriteRamToSD(fr,"DUMP.BIN",0x10000 );
  for(a=0;a<dumpsize;a++){
      sprintf(temp,"%02x ",mem_read0(a+FromAddr));
      PrintToSelected(temp,0);
      rc++;

      if(rc % 32==0){
        sprintf(temp,"\r\nMEM %04x ",rc+FromAddr);
        PrintToSelected(temp,0);
      }else{
        if (rc % 8==0){
          sprintf(temp," ");
          PrintToSelected(temp,0);
        }
      }
  }

}
*/

int sdls(const char *dir,const char * search) {
    int filecnt=0;
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr; /* Return value */
    char const *p_dir;
    if (dir[0]) {
        p_dir = dir;
    } else {
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr) {
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            return 0;
        }
        p_dir = cwdbuf;
    }
    printf("%s files %s\n",search, p_dir);
    DIR dj;      /* Directory object */
    FILINFO fno; /* File information */
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, p_dir, search);
    if (FR_OK != fr) {
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        return 0;
    }
    while (fr == FR_OK && fno.fname[0]) { /* Repeat while an item is found */
        /* Create a string that includes the file name, the file size and the
         attributes string. */
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        /* Point pcAttrib to a string that describes the file. */
        if (fno.fattrib & AM_DIR) {
            pcAttrib = pcDirectory;
        } else if (fno.fattrib & AM_RDO) {
            pcAttrib = pcReadOnlyFile;
        } else {
            pcAttrib = pcWritableFile;
        }
        /* Create a string that includes the file name, the file size and the
         attributes string. */
        printf("%i) %s [%s] [size=%llu]\n", filecnt+1, fno.fname, pcAttrib, fno.fsize);
        if (filecnt<MaxBinFiles){
          sprintf(&BinFiles[filecnt],"%s",fno.fname);
          filecnt++;
        }
        fr = f_findnext(&dj, &fno); /* Search for next item */
    }
    f_closedir(&dj);
    return filecnt++;
}

/*
int GetRomSwitches(){
  gpio_init(HASSwitchesIO); 
  gpio_set_dir(HASSwitchesIO,GPIO_IN);
  gpio_pull_up(HASSwitchesIO);

  gpio_init(ROMA13);
  gpio_set_dir(ROMA13,GPIO_IN);
  gpio_pull_up(ROMA13);
  
  gpio_init(ROMA14);
  gpio_set_dir(ROMA14,GPIO_IN);
  gpio_pull_up(ROMA14);
  
  gpio_init(ROMA15);
  gpio_set_dir(ROMA15,GPIO_IN);
  gpio_pull_up(ROMA15);

//serial port selection swithch
  gpio_init(SERSEL);
  gpio_set_dir(SERSEL,GPIO_IN);
  gpio_pull_up(SERSEL);

  sleep_ms(1); //wait for io to settle.

  int v=0;
  if (gpio_get(HASSwitchesIO)==1){
    PrintToSelected("\r\nNo Switches, no settings changed \n\r",1);
    HasSwitches=0;
  }else{

    if (gpio_get(SERSEL)==1){
        UseUsb=1;
        PrintToSelected("Console Via USB  \n\r",1);
    }else{
        UseUsb=0;
        PrintToSelected("Console Via UART \n\r",1);
    }


//if has switches, then it has buttons and an LED too.

//setup DUMP gpio
    gpio_init(DUMPBUT);
    gpio_set_dir(DUMPBUT,GPIO_IN);
    gpio_pull_up(DUMPBUT);

//setup RESETBUT gpio
    gpio_init(RESETBUT);
    gpio_set_dir(RESETBUT,GPIO_IN);
    gpio_pull_up(RESETBUT);

//setup AUXBUT gpio
    gpio_init(AUXBUT);
    gpio_set_dir(AUXBUT,GPIO_IN);
    gpio_pull_up(AUXBUT);

//setup PCBLED
    gpio_init(PCBLED);
    gpio_set_dir(PCBLED,GPIO_OUT);





  }
  return rombank;

}
*/

int SDFileExists(char * filename){
    FRESULT fr;
    FILINFO fno;

    fr = f_stat(filename, &fno);
    return fr==FR_OK;
}


//############################################################################################################
//################################################# Sound ####################################################
//############################################################################################################

void init_sound(){
   gpio_init(SPEAKER_PINa);
   gpio_init(SPEAKER_PINb);
   gpio_set_dir(SPEAKER_PINa,GPIO_OUT);
   gpio_set_dir(SPEAKER_PINb,GPIO_OUT);

}


/*
void SetPWM(void){
    gpio_init(soundIO1);
    gpio_set_dir(soundIO1,GPIO_OUT);
    gpio_set_function(soundIO1, GPIO_FUNC_PWM);

    gpio_init(soundIO2);
    gpio_set_dir(soundIO2,GPIO_OUT);
    gpio_set_function(soundIO2, GPIO_FUNC_PWM);

    PWMslice=pwm_gpio_to_slice_num (soundIO1);
    pwm_set_clkdiv(PWMslice,16);
    pwm_set_both_levels(PWMslice,0x80,0x80);

    pwm_set_output_polarity(PWMslice,true,false);

    pwm_set_wrap (PWMslice, 256);
    pwm_set_enabled(PWMslice,true);

}
*/
/*
void Beep(uint8_t note){
    int w;     
    //set frequency    
    pwm_set_clkdiv(PWMslice,256);
    if (note>0 && note<128){
      //get divisor from Midi note table.
      w=MidiNoteWrap[note];  
      pwm_set_both_levels(PWMslice,w>>1,w>>1);
      //set frequency from midi note table.
      pwm_set_wrap(PWMslice,w);
    }else{
      pwm_set_both_levels(PWMslice,0x0,0x0);  
    }
}
*/





//############################################################################################################
//################################################# Core 1 ####################################################
//############################################################################################################

void Core1Main(void){

  printf("\n#Core 1 Starting#\n");

   while(1){
     DumpScreenToLCD();
     //sleep_ms(100);
     
   
     tight_loop_contents();
    
   }  

}


void init_pico_uart(void){
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // And set up and enable the interrupt handlers
//    irq_set_exclusive_handler(UART0_IRQ,intUARTcharwaiting );
//    irq_set_enabled(UART0_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

//    uart_puts(UART_ID, "\nUart INIT OK\n\r");


}





void do_keyboard()
{
    bitWrite(z80ports_in[0], 0, keymap[0x16]);
 //   bitWrite(z80ports_in[0], 1, keymap[0x1a]);
 //   bitWrite(z80ports_in[0], 2, keymap[0x22]);
    bitWrite(z80ports_in[0], 3, keymap[0x58]);
    bitWrite(z80ports_in[0], 4, keymap[0x75]);
    bitWrite(z80ports_in[0], 5, keymap[0x72]);
    bitWrite(z80ports_in[0], 6, keymap[0x76]);
    bitWrite(z80ports_in[0], 7, keymap[0x12]);

    bitWrite(z80ports_in[1], 0, keymap[0x26]);
    bitWrite(z80ports_in[1], 1, keymap[0x25]);
    bitWrite(z80ports_in[1], 2, keymap[0x24]);
    bitWrite(z80ports_in[1], 3, keymap[0x22]);
    bitWrite(z80ports_in[1], 4, keymap[0x23]);
    bitWrite(z80ports_in[1], 5, keymap[0x21]);

    bitWrite(z80ports_in[2], 0, keymap[0x1e]);
    bitWrite(z80ports_in[2], 1, keymap[0x15]);
    bitWrite(z80ports_in[2], 2, keymap[0x1d]);
    bitWrite(z80ports_in[2], 3, keymap[0x1a]);
    bitWrite(z80ports_in[2], 4, keymap[0x1b]);
    bitWrite(z80ports_in[2], 5, keymap[0x1c]);
    bitWrite(z80ports_in[2], 6, keymap[0x14]); // control

    bitWrite(z80ports_in[3], 0, keymap[0x2e]);
    bitWrite(z80ports_in[3], 1, keymap[0x2d]);
    bitWrite(z80ports_in[3], 2, keymap[0x2c]);
    bitWrite(z80ports_in[3], 3, keymap[0x2a]);
    bitWrite(z80ports_in[3], 4, keymap[0x34]);
    bitWrite(z80ports_in[3], 5, keymap[0x2b]); //F
    
    bitWrite(z80ports_in[4], 0, keymap[0x36]);
    bitWrite(z80ports_in[4], 1, keymap[0x35]);
    bitWrite(z80ports_in[4], 2, keymap[0x33]);
    bitWrite(z80ports_in[4], 3, keymap[0x29]);
    bitWrite(z80ports_in[4], 4, keymap[0x31]);
    bitWrite(z80ports_in[4], 5, keymap[0x32]); //B
    
    bitWrite(z80ports_in[5], 0, keymap[0x3d]);
    bitWrite(z80ports_in[5], 1, keymap[0x3e]);
    bitWrite(z80ports_in[5], 2, keymap[0x3c]);
    bitWrite(z80ports_in[5], 3, keymap[0x3a]);
 //   bitWrite(z80ports_in[5], 4, keymap[0x35]);
    bitWrite(z80ports_in[5], 5, keymap[0x3b]); //J
    
    bitWrite(z80ports_in[6], 0, keymap[0x46]);
    bitWrite(z80ports_in[6], 1, keymap[0x43]);
    bitWrite(z80ports_in[6], 2, keymap[0x44]);
    bitWrite(z80ports_in[6], 3, keymap[0x41]);
 //   bitWrite(z80ports_in[6], 4, keymap[0x3b]);
    bitWrite(z80ports_in[6], 5, keymap[0x42]); // K
    
    bitWrite(z80ports_in[7], 0, keymap[0x45]);
    bitWrite(z80ports_in[7], 1, keymap[0x4d]);
    bitWrite(z80ports_in[7], 2, keymap[0x4b]);
    bitWrite(z80ports_in[7], 3, keymap[0x49]);
 //   bitWrite(z80ports_in[7], 4, keymap[0x32]);
    bitWrite(z80ports_in[7], 5, keymap[0x4c]); // semi colon

    bitWrite(z80ports_in[8], 0, keymap[0x4e]);
    bitWrite(z80ports_in[8], 1, keymap[0x52]);
    bitWrite(z80ports_in[8], 2, keymap[0x54]);
    bitWrite(z80ports_in[8], 3, keymap[0x4a]);
 //   bitWrite(z80ports_in[8], 4, keymap[0x32]);
    bitWrite(z80ports_in[8], 5, keymap[0x5d]); //colon 


    bitWrite(z80ports_in[9], 0, keymap[0x66]);
    bitWrite(z80ports_in[9], 1, keymap[0x5b]);
    bitWrite(z80ports_in[9], 2, keymap[0x6b]);
    bitWrite(z80ports_in[9], 3, keymap[0x5a]);
 //   bitWrite(z80ports_in[9], 4, keymap[0x32]);
    bitWrite(z80ports_in[9], 5, keymap[0x74]); // right arrow
    
//    if(keymap[KEY_F1] == 0)   // F1  key    Help
//    {
//      keymap[KEY_F1] = 1;
//       //load_lynx_tap();
//    }
    if(keymap[KEY_F2] == 0)   // F2  key  Toggle Turbo speed
    {
       keymap[KEY_F2] = 1;
       if(speed_mult > 1)
            speed_mult = 1;
       else
            speed_mult = 16;
    } 
    
    if(keymap[KEY_F3] == 0)   // F3  key    Get System Info
    {
    /*
      keymap[KEY_F3] = 1;
      printf(("Free Heap: ");
      printf(system_get_free_heap_size());
      printf(("vga free memory: ");
      printf((int)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
      printf(("Total heap: ");
      printf(ESP.getHeapSize());
        printf(("Free heap: ");
        printf( ESP.getFreeHeap());
        printf(("Total PSRAM: ");
        printf(ESP.getPsramSize());
        printf(("Free PSRAM: ");
      printf(ln(ESP.getFreePsram());
    */
    }

 
/*
    if(keymap[KEY_F4] == 0)   //   Toggle display red/blue for Level9 Adventures
    {
       keymap[KEY_F4] = 1;
       show_redblue = !show_redblue;
    } 
     if(keymap[KEY_F5] == 0)   //  
    {
       keymap[KEY_F5] = 1;
       printf("%i",elapsed_t);
    } 
     if(keymap[KEY_F6] == 0)   //  Increment Disk number and reload
    {
       keymap[KEY_F6] = 1;
       current_diskno++;
       if(current_diskno > 20)
          current_diskno = 0;
       open_working_disk(current_diskno);
    } 
     if(keymap[KEY_F7] == 0)  
    {
       keymap[KEY_F7] = 1;
       use_interrupts = !use_interrupts;
    } 
        
      
//    if(keymap[KEY_F8] == 0)   //   Save Current Working Disk buffer as ldump.ldf on SD card
//    {
//     keymap[KEY_F8] = 1;
//       save_working_disk();
//    }   
    if(keymap[KEY_F10] == 0)   //  CLOSE SDCard 
    {
      keymap[KEY_F10] = 1;
    //  SPI.end();
      // .close();
    }  
    if(keymap[KEY_F11] == 0)   //  REBOOT ESP32
    {
     keymap[KEY_F11] = 1;
  //     esp_restart();
    }    
    if(keymap[KEY_F12] == 0)   //  REBOOT Z80
    {
    keymap[KEY_F12] = 1;
//       resetz80();
       Z80RESET(&cpu_z80);
    }    
*/
    
}



/* #################### SD Card and DISK ######################### */


#ifdef SD_ENABLED
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    printf(f("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        printf("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        printf("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            printf(("  DIR : ");
            printf(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            printf(("  FILE: ");
            printf((file.name());
            printf(("  SIZE: ");
            printf(file.size());
        }
        file = root.openNextFile();
    }
}
//// Flash and SDCard configuration
//#define FORMAT_ON_FAIL     true
//#define SPIFFS_MOUNT_PATH  "/flash"
//#define SDCARD_MOUNT_PATH  "/SD"
//// base path (can be SPIFFS_MOUNT_PATH or SDCARD_MOUNT_PATH depending from what was successfully mounted first)
//char const * basepath = nullptr;
//
//void test_sd()
//{
//
//    if (FileBrowser::mountSDCard(false, SDCARD_MOUNT_PATH))
//    basepath = SDCARD_MOUNT_PATH;
//  else if (FileBrowser::mountSPIFFS(false, SPIFFS_MOUNT_PATH))
//    basepath = SPIFFS_MOUNT_PATH;
//
//}
/*
void test_sd()
{
    SPI.begin(14, 2, 12);
    if (!SD.begin(13)) 
    {
        printf("Mount SD failed");
    }

    fileSystem = &SD;
    vTaskDelay(2);
   
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        printf("No SD card attached");
        return;
    }

    printf(("SD Card Type: ");
    if(cardType == CARD_MMC){
        printf("MMC");
    } else if(cardType == CARD_SD){
        printf("SDSC");
    } else if(cardType == CARD_SDHC){
        printf("SDHC");
    } else {
        printf("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    printf("SD Card Size: %lluMB\n", cardSize);
 */
 /*
  // re-open the file for reading:
  myFile = SD.open("/AirRaid.tap", FILE_READ);
  if (myFile) {
    printf("COMMAND.TXT:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    printf("error opening COMMAND.TXT");
  }
*/

//listDir(SD, "/", 0);
}

#endif












//#######################################################################################################
//#                                      MAIN                                                           #
//#######################################################################################################



int main(int argc, char *argv[])
{
	static struct timespec tc;
		
	int opt;
	int fd;
	
	const char * idepathi ="";
	const char * idepath ="";
	
	int indev;
	char *patha = NULL, *pathb = NULL;
        char temp[250];
	
//        char RomTitle[200];
        
//over clock done in ini parcer now
        set_sys_clock_khz(250000, true);

// led gpio
        setup_led();
        flash_led(250);
        stdio_usb_init();
        flash_led(250);
        printf("\n %c[2J\n\n\rUSB INIT OK \n\r",27);
                
// init sound
init_sound();                
                
//init uart
//        init_pico_uart();
//        sprintf(RomTitle,"\n %c[2J \n\n\rUART INIT OK \n\r",27);
//        uart_puts(UART_ID,RomTitle);


// mount SD Card
        sd_card_t *pSD = sd_get_by_num(0);
        FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
        if (FR_OK != fr){
            // panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
            printf("SD INIT FAIL  \n\r");
            while(1); //halt
        }

        printf("SD INIT OK \n\r");




// inifile parse
	dictionary * ini ;
	char       * ini_name ;
        const char  *   s ;
        const char * inidesc;
                
        int overclock;
        int jpc;
        int iscf=0;

       	ini_name = "rc2040.ini";

	if (SDFileExists(ini_name)){
	  sprintf("Ini file %s Exists Loading ... \n\r",ini_name);

//########################################### INI Parser ######################################		
	  ini = iniparser_load(fr, ini_name);
	  //iniparser_dump(ini, stdout);

	  // start at
	  jpc = iniparser_getint(ini, "ROM:jumpto", 0);

          // Use Ide
	  ide = iniparser_getint(ini, "IDE:ide",1);
	  
	  // IDE cf file
//	  iscf = iniparser_getint(ini, "IDE:iscf", iscf);
	  
//	  idepathi = iniparser_getstring(ini, "IDE:idefilei", "");
//	  idepath = iniparser_getstring(ini, "IDE:idefile", idepath);
	  
	  // USB or UART
//	  UseUsb = iniparser_getint(ini, "CONSOLE:port", 1);

          // ININAME
          inidesc = iniparser_getstring(ini, "EMULATION:inidesc","Default ini" );
          printf("INI Description: %s \n\r",inidesc,1);

          // Trace enable from inifile
	  trace = iniparser_getint(ini, "DEBUG:trace",0 );
	  watch = iniparser_getint(ini, "DEBUG:watch",0 );

	  // PORT


          // Overclock
	  overclock = iniparser_getint(ini, "SPEED:overclock",0 );
	  if (overclock>0){
	        printf("Overclock to %i000\n\r",overclock,1);
	  	set_sys_clock_khz(overclock*1000, true);
          }

	  //iniparser_freedict(ini); // cant free, settings are pointed to dictionary.

	  printf("Loaded INI\n\r",1);	

//########################################### End of INI Parser ###########################


//IF switches link present, get switches and select rom bank and UART from switches
/*
          if (overridejumpers==0){
            rombank=GetRomSwitches();
          }else{
            printf("Override jumpers set in INI \n\r");
          }  
*/                 
        }else{
            uart_puts(UART_ID,"No  \n\r");
            printf("SD INIT OK \n\r",1);
        }

        flash_led(200);
        
        printf("\rWaiting for USB to connect\n\r",1);
        //if usb wait for usb to connect.
        while (!tud_cdc_connected()) { sleep_ms(100);  }

//compiled time
	printf("\n\rCompiled %s %s\n",__DATE__,__TIME__);

// ALLOCATE MEMORY
//
    bank1 = (byte *)malloc(65536);
    if(bank1 == NULL){
      printf("Failed to allocate Bank 1 memory");;
      while(1);
    }

    bank2 = (byte *)malloc(16384);
    if(bank2 == NULL){
      printf("Failed to allocate Bank 2 memory");
      while(1);
    }

    bank3 = (byte *)malloc(16384);
    if(bank3 == NULL){
      printf("Failed to allocate Bank 3 memory");
      while(1);
    }

#ifdef enable_disks
    init_disks(); 
    open_working_disk(1); // loads jd1.ldf disk from sd card  
#endif

//patc rom
patchrom();
        
//init PIO
//    if(PIOA<256) PIOA_init();

//init linyx keyboard ports
// make sure keyboard ports are FF
    for(int t = 0;t < 16;t++)
    {
        z80ports_in[t] = 0xff;
    }

//init screen
  screenInit(1);
  printf("init Screen");

//banner
        printf( "\n\r     ________________________________");
        printf( "\n\r    /                                |");
        printf( "\n\r   /            PLYNX                |");
        printf( "\n\r  /         Derek Woodroffe          |");
        printf( "\n\r |  O        Extreme Kits            |");
        printf( "\n\r |     Kits at extkits.uk/PLYNX      |");
        printf( "\n\r |               2023                |");
        printf( "\n\r |___________________________________|");
        printf( "\n\r   | | | | | | | | | | | | | | | | |  \n\n\r");

//init Emulation

//Start Core1
        multicore_launch_core1(Core1Main);
        printf("\n#Core 1 Started#\n\n");
        
//Init Z80
	tc.tv_sec = 0;
	tc.tv_nsec = 20000000L;
	int stop_z80=0;
	int screendumpcounter=999999;

	Z80RESET(&cpu_z80);
	//nonstandard start vector
	if(jpc){
	    cpu_z80.PC=jpc;
            printf("Starting at 0x%04X \n\r",jpc);
	}
	
	cpu_z80.ioRead = io_read;
	cpu_z80.ioWrite = io_write;
	cpu_z80.memRead = mem_read;
	cpu_z80.memWrite = mem_write;
	cpu_z80.trace = z80_trace;

	printf("\r\n ########################################\n\r",0);
	printf(" ###########  PLYNX STARTING  ###########\n\r",0);
	printf(" ########################################\n\n\r",0);

	/* This is the wrong way to do it but it's easier for the moment. We
	   should track how much real time has occurred and try to keep cycle
	   matched with that. The scheme here works fine except when the host
	   is loaded though */

	  


	/* We run 7372000 t-states per second */
	/* We run 365 cycles per I/O check, do that 50 times then poll the
	   slow stuff*/
	while (!emulator_done) {
		int i;
		/* 36400 T states for base RC2014 - varies for others */
/*                if(HasSwitches){       
	            if(gpio_get(DUMPBUT)==0){
                        DumpMemory(0,0x10000,fr);
                        while(gpio_get(DUMPBUT)==0);
                    }

                    if(gpio_get(RESETBUT)==0) {
                        printf("\r\n ########################### \n\r");
                        printf(" ####### Z80 RESET ######### \n\r");
                        printf(" ########################### \n\n\r");
			z80_vardump();                        
                        Z80RESET(&cpu_z80);
                        while(gpio_get(RESETBUT)==0);
                    }                    

                }
*/
		for (i = 0; i < 40; i++) {  //origional
		    int j;
		    for (j = 0; j < 50; j++) { Z80ExecuteTStates(&cpu_z80, (tstate_steps + 5)/ 10);	}
		}
		
		//fake USB char in interrupts
		if (intUSBcharwaiting()){
		   char c=getUSBcharwaiting();
		   putchar(c); //echo
	           pump_key(c);
	        }	  
	        
//	        screendumpcounter++;
//	        if (screendumpcounter>100){
//	            printf("\n----------------ScreenDump-----------------\n");
//	            DumpScreen(bank2,20*42,0x120,0x20);
//	            DumpScreenToLCD();
//	            screendumpcounter=0;
//	        }	
	        
		if (int_recalc) {
			/* If there is no pending Z80 vector IRQ but we think
			   there now might be one we use the same logic as for
			   reti */
			if (!live_irq )
				poll_irq_event();
			/* Clear this after because reti_event may set the
			   flags to indicate there is more happening. We will
			   pick up the next state changes on the reti if so */
			if (!(cpu_z80.IFF1|cpu_z80.IFF2))
				int_recalc = 0;
		}
	}
}


