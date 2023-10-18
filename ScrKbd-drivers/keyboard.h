
void kbdBuffIn(char c);
//char kbdBuffOut(void);
//int kbdBuffWaiting(void);
char scanForKeyBlocking(void);
void keyboardProcess(void);
char testKbdGetCharWaiting(void);
int testKbdCharWaiting(void);
char kbdGetCharWaiting(void);
int ReadUntilCr(char * inbuff);
void kbd_init(int poll);
uint16_t scanForKey(void);
uint8_t KbdCaps;
