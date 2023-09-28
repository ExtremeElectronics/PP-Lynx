
#define SPC         0x20
#define ESC         0x1b
#define DEL         0x7f
#define BSP         0x08
#define LF          0x0a
#define CR          0x0d
#define FF          0x0c
#define BELL	    0x07
#define TAB         0x09

void screenInit(int poll);

void quickClearScreen(uint16_t bg);
void ClearScreen(uint16_t bg);

void displayTxtBuff(void);

void charTxtBuffXY(int x, int y, char c);

void incCursorY(void);

void incCursorX(void);

void charTxtBuff(char c);

void stringTxtBuff(char s[]);

void DispClear(void);

void scrollup(void);

void doChar(char asc);

bool screen_repeating_timer_callback(struct repeating_timer *t);

void screenProcess();

void WriteTxtLine(int x,int y, int line);

void addToGraffBuff(uint8_t x, uint8_t y, uint8_t cmd);

uint16_t palette[17];

uint16_t defaultTextFgColor;
uint16_t defaultTextBgColor;

//static int screenprintf(const char*, ...)
int screenprintf(const char *format, ...);


