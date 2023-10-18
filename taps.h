
#define TAP_DATA       0x41 //Level 9 Data
#define TAP_BINARY     0x4d   //Lynx Binary format
#define TAP_BASIC      0x42


int load_lynx_tap(FRESULT fr,char * fn, uint8_t tape_type,Z80Context* ctx );
void save_lynx_tap(Z80Context* ctx);

