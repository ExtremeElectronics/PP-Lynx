/*
Taken from 
Taps Read From Windows Pale source https://github.com/retrogubbins/palewin32/tree/master/paledll/PaleTAPS.cpp

Charles Peter Debenham Todd (used with permission)

*/
#include <stdio.h>
#include <time.h>
#include "ff.h"
#include "pico/stdlib.h"
#include "libz80/z80.h"
#include "taps.h"
#include <string.h>

#define TAPSBUFFSIZE 4096 
unsigned char lbuffer[TAPSBUFFSIZE];//leave room for 'A' prog

// unsigned char lbuffer[LYNX_MAXMEM];//leave room for 'big' prog
// int level9; //kludge
int file_id; //type offile loaded
int breakin=0;	//stops program after loading
//unsigned char taprom[30];

//referance to Lynx RAM 
extern uint8_t *bank1;
extern int SD_OK;
 
int load_lynx_tap(FRESULT fr,char * fn, uint8_t tape_type,Z80Context* ctx)
{
        char fname[50];
        int fs=0;
	if(SD_OK==0){
	   printf("No SD\n");
	   return(0);
	}   
	FIL  lhandle; //,lhandle2;
	char fn2[100];
	
	printf("Opening %s Type %X\n",fn,tape_type);
	
	unsigned int  buf_p=0,quot=0,load_address;
	UINT size_read;
	unsigned char csum;
	unsigned int cdd,f,ret;
	unsigned int tap_leng,exec_address, buf_pale;
	char csum_ok[10];
	
	// open a file for input		  
	if (f_open(&lhandle, (char*)fn, FA_READ )==FR_OK)
	{
	   if(f_read(&lhandle, lbuffer ,sizeof( lbuffer ),&size_read)!=FR_OK){ 
	        printf("Cant Read File\n");
	   	return(0);
	   }
	}else{
		printf("cant open file\n");
		return(0);
	}

	printf("Read %X bytes (Buffer max %X) \n",size_read,sizeof( lbuffer ));

	if(tape_type==0) //standard tape with name
	{
		//Get filename - skip to second "
		while (quot<2)
		{
			if(lbuffer[buf_p++]=='"') quot++;
			if(quot==1){
			  fname[fs]=lbuffer[buf_p];
			  fs++;
			}
		}
		fname[fs]=0;
		lbuffer[buf_p-1]=0;
		lbuffer[0]=' ';		//wipe out first " ready for printing
	}

	//If next char is A5 we forgot to remove it from the TAP file !
	//we could have either 4d 'M' mc or 42 'B' basic, *** or 41 - 'A' level 9 data ***
	//If next char is 42 only then we have a basic proggy
	if(lbuffer[buf_p]==0xa5)
	{
                printf("A5 Byte Found in Header\n");	
		buf_p++;
		file_id=lbuffer[buf_p];
		buf_p++;
	}
	else
	{
		file_id=lbuffer[buf_p];
		buf_p++; //skip over the 42 ( or 4D) B or M (when no a5 there - everyone but me !)
	}

	//Get Length
	if(file_id==TAP_BASIC)
	{
		printf("TAP_BASIC\n");
		load_address=bank1[0x61fa]+256*bank1[0x61fb];//should be 694D
		tap_leng=lbuffer[buf_p]+256*lbuffer[buf_p+1];
		buf_p+=2;
	}
	else if(file_id==TAP_BINARY)
	{
		printf("TAP_BINARY\n");
		tap_leng=lbuffer[buf_p]+256*lbuffer[buf_p+1];
		load_address=lbuffer[buf_p+2]+256*lbuffer[buf_p+3];
		buf_p+=4;
	}else //DATA - swap dest and length
	{
		printf("TAP_DATA\n");
		load_address=lbuffer[buf_p]+256*lbuffer[buf_p+1];
		tap_leng=lbuffer[buf_p+2]+256*lbuffer[buf_p+3];
		buf_p+=4;
	}
        printf("%4x datastart\n",buf_p);
	buf_pale=load_address;
	//Get Prog
	csum=0;
	for(f=0;f<tap_leng;f++)
	{
		csum+=lbuffer[buf_p];//only used for binary MLOADed progs & Level 9 Data
		bank1[buf_pale++]=lbuffer[buf_p++];
		if (buf_p==TAPSBUFFSIZE){
		  printf("Aux read read buff %x, writebuff %x, BUFF SIZE %x\n",buf_p,buf_pale,TAPSBUFFSIZE);	
		  if (f_read(&lhandle, lbuffer ,sizeof( lbuffer ),&size_read)!=FR_OK){
		    printf("Aux read failed at %X\n",buf_pale);
		    return 0;
		  }else{
		    printf("read a further %X chars\n",size_read);
		  }
		  buf_p=0;
		}
	}

	//dec ptr to point to last byte of prog in memory
	buf_pale--;
	
	if(file_id==TAP_BASIC)
	{
		//Update end of program pointer in the BASIC os 
		bank1[0x61fc]=buf_pale % 256;
		bank1[0x61fd]=buf_pale / 256;
	}

	sprintf(csum_ok,"No Checksum");
	//skip over next two bytes if binary file - csum related
	if(file_id==TAP_BINARY || file_id==TAP_DATA)
	{
		sprintf(csum_ok,"Checksum OK");
		if(csum!=lbuffer[buf_p++])
		   sprintf(csum_ok,"Checksum BAD");
		if(lbuffer[buf_p++]==0x4e)
		   printf("Got a funny N byte thing after the checksum!\n");
	}
	
	//Get Execution Addr
	if(file_id==TAP_BASIC || file_id==TAP_BINARY)
		exec_address=lbuffer[buf_p]+256*lbuffer[buf_p+1];
	else
		exec_address=0;

	if(file_id==TAP_BASIC || file_id==TAP_BINARY)
		sprintf((char*)fn2,"Name:%s \x0d\x0a ID:%02x \x0d\x0a Start %04x \x0d\x0a End %04x \x0d\x0a Length %04x \x0d\x0a Run %04x\x0d\x0a %s",fname,file_id,load_address,buf_pale,tap_leng-1,exec_address,csum_ok);
	else  //Data
		sprintf((char*)fn2,"Name: n/a \x0d\x0a ID: %04x \x0d\x0a Start %04x \x0d\x0a End %04x \x0d\x0a Length %04x \x0d\x0a Run %04x",file_id,load_address,buf_pale,tap_leng-1);

	printf("%s\n",fn2);	

	if(breakin==1)
	{
		//set_hl(buf_pale);
		ctx->R1.wr.HL=buf_pale;
		//set_de(0);
		ctx->R1.wr.DE=0;
		//set_pc(0xcfb);  //jump back into the ROM load routine - this one to the prompt (but MEM not set correctly)
		ctx->PC=0xcfb;
	}

	if(exec_address!=0 && file_id!=TAP_DATA)
	{
		if(file_id==TAP_BINARY)
		{
			printf("Run Binary from %4X\n",exec_address);
			//set_hl(exec_address);//as the ROM does it
			ctx->R1.wr.HL=exec_address;
			//set_pc(exec_address);  //jump to the invaders routine :))
			ctx->PC=exec_address;
		}
		else
		{
			printf("Run BASIC from %4X\n",exec_address);
			//set_hl(buf_pale);//end byte of program
			ctx->R1.wr.HL=buf_pale;
			//set_de(exec_address);
			ctx->R1.wr.DE=exec_address;
			//set_pc(0xcc1);  //jump back into the ROM load routine
			ctx->PC=0xcc1;
		}
	}
	else
	{
		printf("Run DATA from %4X\n",exec_address);
		//set_hl(buf_pale);
		ctx->R1.wr.HL=buf_pale;
		//set_de(exec_address);
		ctx->R1.wr.DE=exec_address;
		//set_pc(0xcfb);  //jump back into the ROM load routine - this one to the prompt (but MEM not set correctly)
		ctx->PC=0xcfb;
	}
        f_close( &lhandle ); 

/*        
        int a=0,c=0;
        for (a=load_address;a<buf_pale;a++){
          
          if (c>15){
            printf("\n\r%4X ",a);
            c=0;
          }
          c++;   
          printf("%2X ",bank1[a]);
        
        }
*/                
	return(1);
}


void save_lynx_tap(Z80Context* ctx)
{
	if(SD_OK==0){
           printf("No SD\n");
        }


	unsigned int  buf_p=0,quot=0,size_read,load_address;
//	unsigned char buffer[65536];//leave room for 'big' proggy ;)
	char lblu[100],plab[100],pnam[100];
	unsigned char csum;
	unsigned int cdd,f,ret,e1,e2,e3;
	unsigned int tap_leng,exec_address, buf_pale,end_address,start_address,prog_size;

	FIL  handle2;

	int end_nam;
        UINT size_written;
	//jump here from ROM save routine
	//get filename and any parameters
	//save header
	//save program between bounds
	e1=ctx->R1.wr.DE;
	quot=0;buf_p=0;
	//Get filename - skip to second "
	while (quot<2)
	{
		plab[buf_p]=bank1[e1+buf_p];
		if(bank1[e1+buf_p++]=='"')
			quot++;
	}
	plab[buf_p-1]='\0';
	plab[0]=' ';		//wipe out first " ready for save

	e2=strlen(plab);
	for(f=0;f<e2;f++)
		pnam[f]=plab[f+1];

	strcat(pnam,".TAP");	
	//sprintf(lbl,"DE points to %s, pnam is %s",plab,pnam);
	//MessageBox(NULL,lbl,"PALE ",MB_YESNOCANCEL | MB_DEFBUTTON1);

	printf("Saving to %s\n",pnam);
    // open a file for output		  
    if( f_open(&handle2, pnam,FA_READ | FA_CREATE_ALWAYS | FA_WRITE  )==FR_OK) 
	{
		//write header - the filename
		lbuffer[0]='"';
		for(f=1;f<e2;f++)
		{
				lbuffer[f]=plab[f];		
		}
		end_nam=f;
		lbuffer[end_nam]='"';
		lbuffer[end_nam+1]=0x42;	//BASIC proggy designator
		start_address=bank1[0x61fa]+256*bank1[0x61fb];//should be 694D
		end_address=bank1[0x61fc]+256*bank1[0x61fd];
		//write tape length (end of program pointer-load address)
		tap_leng=end_address-start_address+1;
		prog_size=tap_leng+end_nam+4+3+1;	
		lbuffer[end_nam+2]=tap_leng % 256;
		lbuffer[end_nam+3]=tap_leng / 256;
		//write Prog
		csum=0;
		int save_size=0;
		int save_address=end_nam+4;
		for(f=0;f<tap_leng;f++)
		{
   		   lbuffer[save_address]=bank1[start_address+f];
   		   save_address++;
		   if(save_size==TAPSBUFFSIZE){
		     f_write(& handle2, lbuffer,save_size,&size_written );
		     printf("Written address %X to file for %X bytes",save_address,save_size);
		     save_address=save_address-save_size;
		     save_size=0;
		   }else{
		     save_size++;
	           }			
		}
	        
		lbuffer[save_address++]=0;		//exec addr
		lbuffer[save_address++]=0;
		lbuffer[save_address++]=0;	//copy byte
                save_size+=3;
   	        f_write(& handle2, lbuffer,save_size,&size_written );
   	        printf("Written address %X to file for %X bytes",save_address,save_size);

		//e2=get_ix();
		e2=ctx->R1.wr.IX;
		e1=bank1[0x61fc]+256*bank1[0x61fd];

//		lbuffer[end_nam+4+f]=0;		//exec addr
//		lbuffer[end_nam+4+f+1]=0;
//		lbuffer[end_nam+4+f+2]=0;	//copy byte

		// OKAY, actually write the file to disk
//		f_write(& handle2, lbuffer,prog_size,&size_written );
                //	size_written = write( handle2, lbuffer,prog_size );
		//sprintf(lbl,"Hello Pete,I Wrote %d bytes",size_written);
		//MessageBox(NULL,lbl,"PALE ",MB_YESNOCANCEL | MB_DEFBUTTON1);
		f_close( &handle2 );
	}
	//return to ROM
	//set_pc(0xc59);  //  do a ret jump back into the ROM 
	ctx->PC=0xc59;
}


/* 

extern  int _stdcall load_lev9_tap(unsigned char fn[],unsigned char fn2[])
{
	int  handle,handle2;
	int file_id;
	unsigned int  buf_p=0,quot=0,size_read,load_address;
	unsigned char buffer[65536];//leave room for 'big' proggy ;)
	char lbl[100];
	unsigned char csum;
	unsigned int cdd,f,ret;
	unsigned int tap_leng,exec_address, buf_pale;

		
	//Remove .TAP
	f=0;
	while(fn[f]!='.')
		f++;
	fn[f]='\0';

	strcpy(lbl,(char*)fn);
	strcat(lbl,"3.tap");
	
	//LOAD RED BANK
	handle = open( lbl, O_RDONLY | O_BINARY );
	if( handle != -1 )
	{
	  size_read = read( handle, buffer,
			sizeof( buffer ) );
	  close( handle );
	  if( size_read == -1 )
				return(0);
	}
	else
	{
		return(0);
	}

	buf_p=0;
	//If next char is A5 we forgot to remove it from the TAP file !
	//we could have either 4d 'M' mc or 42 'B' basic, *** or 41 - 'A' level 9 data ***
	//If next char is 42 only then we have a basic proggy
	if(buffer[buf_p]==0xa5)
	{
		buf_p++;
		file_id=buffer[buf_p];
		buf_p++;
	}
	else
	{
		file_id=buffer[buf_p];
		buf_p++; //skip over the 42 ( or 4D) B or M (when no a5 there - everyone but me !)
	}
	load_address=buffer[buf_p]+256*buffer[buf_p+1];
	tap_leng=buffer[buf_p+2]+256*buffer[buf_p+3];
	buf_p+=4;
	//Get DATA
	for(f=0;f<tap_leng;f++)
	{
		bank2[0xc000+f]=buffer[buf_p++];
	}
	strcpy(lbl,(char*)fn);
	strcat(lbl,"4.tap");
	//LOAD BLUE BANK
	handle = open( lbl, O_RDONLY | O_BINARY );
	if( handle != -1 )
	{
	  size_read = read( handle, buffer,
			sizeof( buffer ) );
	  close( handle );
	  if( size_read == -1 )
				return(0);
	}
	else
	{
		return(0);
	}
	buf_p=0;
	if(buffer[buf_p]==0xa5)
	{
		buf_p++;
		file_id=buffer[buf_p];
		buf_p++;
	}
	else
	{
		file_id=buffer[buf_p];
		buf_p++; //skip over the 42 ( or 4D) B or M (when no a5 there - everyone but me !)
	}
	load_address=buffer[buf_p]+256*buffer[buf_p+1];
	tap_leng=buffer[buf_p+2]+256*buffer[buf_p+3];
	buf_p+=4;
	//Get DATA
	for(f=0;f<tap_leng;f++)
	{
		bank2[0xa000+f]=buffer[buf_p++];
	}
	strcpy(lbl,(char*)fn);
	strcat(lbl,"5.tap");
	//LOAD normal memory 7800 after advloader
	handle = open( lbl, O_RDONLY | O_BINARY );
	if( handle != -1 )
	{
	  size_read = read( handle, buffer,
			sizeof( buffer ) );
	  close( handle );
	  if( size_read == -1 )
				return(0);
	}
	else
	{
		return(0);
	}
	buf_p=0;
	if(buffer[buf_p]==0xa5)
	{
		buf_p++;
		file_id=buffer[buf_p];
		buf_p++;
	}
	else
	{
		file_id=buffer[buf_p];
		buf_p++; //skip over the 42 ( or 4D) B or M (when no a5 there - everyone but me !)
	}
	load_address=buffer[buf_p]+256*buffer[buf_p+1];
	tap_leng=buffer[buf_p+2]+256*buffer[buf_p+3];
	buf_p+=4;
	//Get DATA
	for(f=0;f<tap_leng;f++)
	{
		bank1[load_address+f]=buffer[buf_p++];
	}
	set_pc(0x7056); //re-enter level 9 loader
	level9=1;//kludge
	return(1);
}

extern  int _stdcall z80_to_tap(unsigned char in_fnam[],unsigned char out_fnam[], dword Tleng,dword Tstart,dword prog_type,dword Texecute)
{
	int  handle,handle2,bufp;
	unsigned int  buf_p=0,quot=0,size_read,load_address;
	unsigned char buffer[65536];//leave room for 'big' proggy ;)
	unsigned char buffer_out[65536];//leave room for 'big' proggy ;)
	char lbl[100];
	unsigned char csum;
	unsigned int cdd,f,ret;
	unsigned int tap_leng,exec_address, buf_pale;
	char csum_ok[10];
    int size_written;

  // open a file for input		  
	handle = open( (char*)in_fnam, O_RDONLY | O_BINARY );
	if( handle != -1 )
	{
	  size_read = read( handle, buffer,
			sizeof( buffer ) );
	  close( handle );
	  if( size_read == -1 )
				return(0);
	}
	else
	{
		return(0);
	}


    // open a file for output		  
    handle2 = open( (char*)out_fnam,O_RDWR | O_CREAT | O_TRUNC | O_BINARY,_S_IWRITE);
    if( handle2 != -1 )
	{
		bufp=0;
		//write header
		buffer_out[bufp++]='"';
		buffer_out[bufp++]='N';
		buffer_out[bufp++]='O';
		buffer_out[bufp++]='N';
		buffer_out[bufp++]='A';
		buffer_out[bufp++]='M';
		buffer_out[bufp++]='E';
		buffer_out[bufp++]='"';

		prog_type=1;
		
		if(prog_type==0)
		{
			buffer_out[bufp++]=TAP_BASIC;	//BASIC proggy designator
			//write tape length (end of program pointer-load address)
			buffer_out[bufp++]=Tleng % 256;
			buffer_out[bufp++]=Tleng / 256;
		}
		else if(prog_type==1)
		{
			buffer_out[bufp++]=TAP_BINARY;	//BINARY proggy designator
			//write tape length (end of program pointer-load address)
			buffer_out[bufp++]=size_read % 256;
			buffer_out[bufp++]=size_read / 256;
			//write tape start 
			buffer_out[bufp++]=Tstart % 256;
			buffer_out[bufp++]=Tstart / 256;
		}
		else if(prog_type==2)
			buffer_out[bufp++]=TAP_DATA;	//LEVEL9 data designator


		csum=0;
		for(f=0;f<size_read;f++)
		{
				csum+=buffer[f] % 256;
				buffer_out[bufp++]=buffer[f];		
		}
		if(prog_type==0)
		{
			buffer_out[bufp++]=0;
			buffer_out[bufp++]=0;
		}
		else if(prog_type==1)
		{
			//write checksum
			buffer_out[bufp++]=csum;
			buffer_out[bufp++]=csum;
			//write Execution Addr
			buffer_out[bufp++]=Texecute % 256;
			buffer_out[bufp++]=Texecute / 256;
			buffer_out[bufp++]=Texecute / 256;	//last byte is a copy
		}

		size_written = write( handle2, buffer_out,bufp );
		close( handle2 );
	}

	return(size_read);
}



extern  int _stdcall set_breakin (unsigned int modee)
{
	breakin=modee;
	return(0);
}


extern  int  _declspec(dllexport) load_raw(char fnam[])
{
	int ret,errorr,g,h;
	int  handle,handle2;
	int  size_read,size_written;
	int cdd,f;
	char lbl[200];

	handle=open( fnam, O_RDONLY | O_BINARY );
	if( handle != -1 )
	{
	  raw_samples = read( handle, raw_tape,raw_buflen);
	  close( handle );
	  if( raw_samples == -1 )
	  {
				ret=MessageBox(NULL,":( Couldn't Read the Input file","PALE Load RAW",MB_ICONERROR | MB_OK);
				return(1);
	  }
	}
	else
	{
		ret=MessageBox(NULL,":( Couldn't open the Input file","PALE Load RAW",MB_ICONERROR | MB_OK);
		return(1);
	}
	raw_position=0;

	if(raw_samples==raw_buflen)
		ret=MessageBox(NULL,"This RAW file is too big, try increasing the buffer size","PALE Load RAW",MB_ICONERROR | MB_OK);
	
	
	return(raw_samples);			
}



extern  int  _stdcall  save_raw(char fnam[])
{
	int  handle,handle2;
	int size_written,size_read,ret;
	
	// open a file for output		  
	handle2 = open( fnam,O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,_S_IWRITE);
	if( handle2 != -1 )
	{
		size_written = write( handle2, raw_tape,raw_position );
		close( handle2 );
	}
	else
	{
		ret=MessageBox(NULL,":( Couldn't open the Output file","PALE Save RAW",MB_ICONERROR | MB_OK);
		return(1);
	}
	raw_position=0;
	return(0);
}

*/