//********************************************************************************
//*                                                                              *
//*   NCAT SYSTEM 1.0 by Nathalis                                                *
//*                                                                              *
//*   Includes: NES EMULATOR only                                                *
//*                                                                              *
//*   Requirements: ESP32 WEMOS D1 MINI, SD7789 LCD, MICROSD CARD slot,          *
//*   PCM5102 I2S AUDIO MODULE, OTHER PARTS...                                   *
//*                                                                              *
//*   Only for personal & educational use!                                       *
//*   (GPL-3.0 License)                                                          *
//*                                                                              *
//********************************************************************************
//*                                                                              *
//*   features:                                                                  *
//*   - ESP32-WROVER PSRAM module support                                        *
//*   - MicroSD card support                                                     *
//*   - 240*240 1.3" LCD ST7789 display                                          *
//*   - Composite TV OUT Video PAL                                               *
//*   - I2S AUDIO support PCM5102 module                                         *
//*   - PS2 (USB) KEYBOARD support (wireless not work)                           *
//*   - huge NES ROMs up to 512kB (read from FLASH)                              *
//*   - PS4 DualShock BlueTooth Gamepad (experimental)                           *
//*                                                                              *
//********************************************************************************

// Enable modified included libraries in "/src/" folder but if you have libraries conflict try:
// ... in other case you need copy modified from "/LIBRARIES/" to "/Arduino/libraries/" folder 
// and disable this option in this case (but you must delete modified included libraries from "/src/" folder):

#define INCLUDED_LIBRARIES true //better enable this feature!                           

#define LCD_ENABLED true
#define COMPOSITE_VIDEO_ENABLED true  //Do not disable! it also disable ADC.
#define KEYBOARD_ENABLED true
#define SOUND_ENABLED true
#define BLUETOOTH_ENABLED false //experimental.

#define DEBUG true       //Serial debugging enable.
#define DEBUGEXTRA false //Extra Serial debugging enable.

//********************************************************************************

//KEY BUTTONS PINS:
#define PIN_UP     39  //SVN
#define PIN_DOWN   35  //IO35
#define PIN_LEFT   36  //SVP
#define PIN_RIGHT  12  //TDI => Do not install 330R resistor!!!
#define PIN_A      2   //IO2
#define PIN_B      14  //TMS
#define PIN_START  15  //TDO
#define PIN_SELECT 13  //TCK

///!!! do not forget 1KOHM resistors
#define KEYBOARD_DATA 4  /// ---[ 1K ]--- // -D
#define KEYBOARD_CLK 0   /// ---[ 1K ]--- // +D

//COMPOSITE_VIDEO: - //DAC_GPIO25_CHANNEL or DAC_GPIO26_CHANNEL
#define VIDEO_OUT (DAC_GPIO26_CHANNEL)

//AUDIO_i2S:
#define I2S_BCK_IO (GPIO_NUM_27) //BCK
#define I2S_WS_IO  (GPIO_NUM_32) //LCK
#define I2S_DO_IO  (GPIO_NUM_25) //DIN
#define I2S_DI_IO  (-1)

//LCD_ST7789:
#define TFT_CS   -1  // define chip select pin
#define TFT_DC    5  // define data/command pin
#define TFT_RST  19
#define TFT_MOSI 23  // Data out (SDA) //better not change
#define TFT_SCLK 18  // Clock out (SCL) //better not change

//micro_SD_Card: //GPIO16 and GPIO17 can not use on WROVER
#define SOFTSD_MOSI_PIN (GPIO_NUM_33)
#define SOFTSD_MISO_PIN (GPIO_NUM_22)
#define SOFTSD_SCK_PIN (GPIO_NUM_21)
#define SD_CS_PIN -1  //

//Oscilloscope INPUT:
#define ADC_CHANNEL   ADC1_CHANNEL_6 // GPIO34

// player Digital I/O used
#define SD_CS          -1
#define SPI_MOSI      SOFTSD_MOSI_PIN
#define SPI_MISO      SOFTSD_MISO_PIN
#define SPI_SCK       SOFTSD_SCK_PIN
#define I2S_DOUT      I2S_DO_IO
#define I2S_BCLK      I2S_BCK_IO
#define I2S_LRC       I2S_WS_IO
//********************************************************************************
//MAIN LIBRARIES:

#include <esp_task_wdt.h>
#include "esp_types.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "soc/gpio_reg.h"
#include "soc/rtc.h"
#include "soc/soc.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "soc/ledc_struct.h"
#include "soc/rtc_io_reg.h"
#include "soc/io_mux_reg.h"
#include "rom/gpio.h"
#include "rom/lldesc.h"
#include "driver/periph_ctrl.h"
#include "driver/dac.h"
#include "driver/gpio.h"
#if SOUND_ENABLED
#include "driver/i2s.h"
#endif

//LIBRARIES:
#include "Arduino.h"
//AUDIO_i2S:

//********************************************************************************
#if INCLUDED_LIBRARIES

//MODIFIED INCLUDED_LIBRARIES in "/src/" folder

//LCD_ST7789:
#include <SPI.h>
#include "src/Adafruit-GFX-Library/Adafruit_GFX.h"    // Core graphics library
#include "src/Adafruit-ST7735-Library/Adafruit_ST7789.h"    // Core graphics library

//micro_SD_Card:
#include "src/sdfat-beta/src/SdFat.h" // Use SDFAT-beta...

//AUDIO_i2S:
#if SOUND_ENABLED
#include "driver/i2s.h"
#endif

//player LIBs
#include "src/ESP32-audioI2S/src/Audio.h" // Use SDFAT-beta...

//--------------------------------------------------------------------------------
///#include "arduinoFFT.h" // Standard Arduino FFT library
#include "src/arduinoFFT/src/arduinoFFT.h" // Standard Arduino FFT library

//********************************************************************************
#else
//********************************************************************************

//!!! or use MODIFIED LIBRARIES in "Arduino/libraries/" folder (need copy from /LIBRARIES/)

//LCD_ST7789:
#include <SPI.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

//micro_SD_Card:
#include <SdFat.h> // Use SDFAT-beta...

//player LIBs
#include "Audio.h" //ESP32-audioI2S

#include "arduinoFFT.h" // Standard Arduino FFT library
//********************************************************************************
#endif



// https://github.com/kosme/arduinoFFT, in IDE, Sketch, Include Library, Manage Library, then search for FFT
arduinoFFT FFT = arduinoFFT();

//********************************************************************************
//SETUP:

//USE SDFAT BETA!
///SdFatSoftSpi<SOFTSD_MISO_PIN, SOFTSD_MOSI_PIN, SOFTSD_SCK_PIN> SD;
SoftSpiDriver<SOFTSD_MISO_PIN, SOFTSD_MOSI_PIN, SOFTSD_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(-1, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)
SdFat SD;
File fp;

//LCD_ST7789
#define ST7789_DRIVER     // Configure all registers
#define TFT_WIDTH  240
#define TFT_HEIGHT 240
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//********************************************************************************
//VARIABLES:

//Allocated MEMORY variables:
uint8_t* SCREENMEMORY[256 + 1];  //256*256 bytes + 256 offset
uint16_t SCREENBUFFER[256];  //512 bytes

uint32_t PSRAMSIZE = 0;
uint8_t* PSRAM;

//--------------------------------------------------------------------------------
//SPI FLASH MEMORY ACCESS:
#include "esp_spi_flash.h"

#define SPI_FLASH_ADDRESS 0x00300000 //0x00300000 working (empty flash area)
#define SPI_FLASH_SECTOR_SIZE 0x1000 //4kB = better not change

uint32_t FILE_ROM_SIZE = 0;
uint32_t FLASH_ROM_SIZE = 0;

//constant data pointer for direct access
const void *ROM;

spi_flash_mmap_handle_t handle1;

uint8_t flashdata[4096] = {0}; //4kB buffer

//--------------------------------------------------------------------------------
//DRAM MEMORY ACCESS:
//--------------------------------------------------------------------------------

char* ROMFILENAME; //NES load File name
unsigned char *rom = 0; //Actual ROM pointer

//********************************************************************************


//--------------------------------------------------------------------------------
//NES PALETTES 32bit & 16bit:

const uint32_t nes_32bit[64] = { //4colors Green-Yellow-Red-Blue
  0x2D253469, 0x000000D9, 0x000000A1, 0x0F111CA1, 0x001F3F70, 0x0024491C, 0x06264900, 0x061F3B00,
  0x12162300, 0x2D0B0000, 0x27090000, 0x21080000, 0x1806004D, 0x14141414, 0x14141414, 0x14141414,
  0x453950A1, 0x2D0B00D9, 0x210800D9, 0x181C2DD9, 0x002F5EAF, 0x0031624D, 0x153B6C00, 0x2139620E,
  0x2D304900, 0x45110000, 0x3F0F0000, 0x3F0F0038, 0x330C0077, 0x14141414, 0x14141414, 0x14141414,
  0x5D4D6CD9, 0x451D18D9, 0x33232DD9, 0x2D2C42D9, 0x2D416CD9, 0x213E6C85, 0x2D416C4D, 0x3C456C38,
  0x45476C00, 0x5D3F5015, 0x51272646, 0x5D2A2685, 0x571500BD, 0x2D253469, 0x14141414, 0x14141414,
  0x5D4D6CD9, 0x543846D9, 0x453950D9, 0x45405ED9, 0x45476CD9, 0x3C456CA8, 0x4E48699A, 0x544B6C93,
  0x514A6C69, 0x5D465E69, 0x5D3F50A1, 0x5D3F50BD, 0x5D1700D9, 0x514A6CD9, 0x14141414, 0x14141414,
};

const uint16_t nes_16bit[64] = {
  0x7BEF, 0x001F, 0x0017, 0x4157, 0x9010, 0xA804, 0xA880, 0x88A0,
  0x5180, 0x03C0, 0x0340, 0x02C0, 0x020B, 0x0000, 0x0000, 0x0000,
  0xBDF7, 0x03DF, 0x02DF, 0x6A3F, 0xD819, 0xE00B, 0xF9C0, 0xE2E2,
  0xABE0, 0x05C0, 0x0540, 0x0548, 0x0451, 0x0000, 0x0000, 0x0000,
  0xFFDF, 0x3DFF, 0x6C5F, 0x9BDF, 0xFBDF, 0xFAD3, 0xFBCB, 0xFD08,
  0xFDC0, 0xBFC3, 0x5ECA, 0x5FD3, 0x075B, 0x7BCF, 0x0000, 0x0000,
  0xFFFF, 0xA73F, 0xBDDF, 0xDDDF, 0xFDDF, 0xFD38, 0xF696, 0xFF15,
  0xFECF, 0xDFCF, 0xBFD7, 0xBFDB, 0x07FF, 0xFEDF, 0x0000, 0x0000,
};
//--------------------------------------------------------------------------------
//for player...
bool PLAYING = false;
bool PAUSED = false;

//===============================================================================
//===============================================================================
//===============================================================================
//INPUT SYSTEM:
uint8_t JOY_UP = 0;
uint8_t JOY_DOWN = 0;
uint8_t JOY_LEFT = 0;
uint8_t JOY_RIGHT = 0;
uint8_t JOY_CROSS = 0;
uint8_t JOY_SQUARE = 0;
uint8_t JOY_CIRCLE = 0;
uint8_t JOY_TRIANGLE = 0;
uint8_t JOY_SHARE = 0; //(START)
uint8_t JOY_OPTIONS = 0; //(SELECT)
//--------------------------------------------------------------------------------
uint8_t scancode = 0;
boolean keyup = false;
byte keymap[256];
//--------------------------------------------------------------------------------
IRAM_ATTR void USB_KEYBOARD() {
#if KEYBOARD_ENABLED
  if (keymap[0x75] == 0) {
    JOY_UP = 1;
  } else JOY_UP = 0;
  if (keymap[0x72] == 0) {
    JOY_DOWN = 1;
  } else JOY_DOWN = 0;
  if (keymap[0x6B] == 0) {
    JOY_LEFT = 1;
  } else JOY_LEFT = 0;
  if (keymap[0x74] == 0) {
    JOY_RIGHT = 1;
  } else JOY_RIGHT = 0;

  if (keymap[0x11] == 0 || keymap[0x1A] == 0) JOY_CROSS = 1; //ALT or X
  else JOY_CROSS = 0;
  if (keymap[0x14] == 0 || keymap[0x22] == 0) JOY_SQUARE = 1; //CTRL or Z
  else JOY_SQUARE = 0;
  if (keymap[0x5A] == 0) JOY_SHARE = 1;
  else JOY_SHARE = 0;
  if (keymap[0x66] == 0) JOY_OPTIONS = 1;
  else JOY_OPTIONS = 0;

  if (keymap[0x76] == 0) {
    JOY_SHARE = 1;
    JOY_OPTIONS = 1;
  } else if (JOY_SHARE == 1 && JOY_OPTIONS == 1 ) {
    JOY_SHARE = 0;
    JOY_OPTIONS = 0;
  }

  if (DEBUGEXTRA) {
    if (JOY_UP) Serial.print("UP.");
    if (JOY_DOWN) Serial.print("DOWN.");
    if (JOY_LEFT) Serial.print("LEFT.");
    if (JOY_RIGHT) Serial.print("RIGHT.");
    if (JOY_SHARE) Serial.print("START.");
    if (JOY_OPTIONS) Serial.print("SELECT.");
    if (JOY_CROSS) Serial.print("A.");
    if (JOY_SQUARE) Serial.print("B.");
    Serial.println();
  }
#endif
}
//--------------------------------------------------------------------------------
void IRAM_ATTR kb_interruptHandler(void) {
#if KEYBOARD_ENABLED
  static uint8_t bitcount = 0;
  uint8_t val;

  int clock = digitalRead(KEYBOARD_CLK);
  if (clock == 0) {
    return;
  }

  val = digitalRead(KEYBOARD_DATA);
  if (DEBUGEXTRA) Serial.print(".");
  if (DEBUGEXTRA) Serial.print(val);

  bitcount++;

  if (bitcount > 1 && bitcount < 10) { //8bits
    scancode |= ((val & 1) << (bitcount - 2));
  }

  if (bitcount > 10) {
    if (keyup == true) {
      keymap[scancode] = 1;

      if (DEBUGEXTRA) {
        Serial.print(" {");
        Serial.print(scancode, HEX);
        Serial.print("} ");
      }
      keyup = false;
    } else {
      keymap[scancode] = 0;
    }

    if (scancode  == 0xF0  ) {
      keyup = true;
    } else {
      keyup = false;
    }

    if (DEBUGEXTRA) {
      Serial.print("[");
      Serial.print(scancode, HEX);
      Serial.println("]");
    }

    USB_KEYBOARD();

    bitcount = 0;
    scancode = 0;
  }
#endif
}
//--------------------------------------------------------------------------------
void kb_begin() {
#if KEYBOARD_ENABLED
  pinMode(KEYBOARD_DATA, INPUT_PULLUP);
  pinMode(KEYBOARD_CLK, INPUT_PULLUP);
  digitalWrite(KEYBOARD_DATA, true);
  digitalWrite(KEYBOARD_CLK, true);
  attachInterrupt(digitalPinToInterrupt(KEYBOARD_CLK), kb_interruptHandler, RISING);
  memset(keymap, 1, sizeof(keymap));
#endif
}
//********************************************************************************


//===============================================================================
//INCLUDES:

//COMPOSITE_VIDEO_OUT
#if COMPOSITE_VIDEO_ENABLED
#include "src/compositevideo/video.h"
#endif

//MAIN FONT
#include "Retro8x16.c" //used font...

//MENU ICON
///#include "icons.c" //used font...

//===============================================================================
//===============================================================================
//===============================================================================
//VIDEO SYSTEM:
QueueHandle_t vidQueue;
//VIDEO_SETUP
// Visible (NTSC) screen height
#define  NES_VISIBLE_HEIGHT   240
#define  NES_SCREEN_WIDTH     256
#define  NES_SCREEN_HEIGHT    240
#define DEFAULT_WIDTH NES_SCREEN_WIDTH
#define DEFAULT_HEIGHT NES_VISIBLE_HEIGHT
//--------------------------------------------------------------------------------
//BUFFER TEXT DRAW FUNCTIONS
//--------------------------------------------------------------------------------
void screenmemory_fillscreen(uint8_t COLOR) {
  for (uint16_t Ypos = 0; Ypos < DEFAULT_HEIGHT; Ypos++)
    for (uint16_t Xpos = 0; Xpos < DEFAULT_WIDTH; Xpos++) {
      SCREENMEMORY[Ypos][Xpos] = COLOR;
    }
  if (LCD_ENABLED) xQueueSend(vidQueue, &SCREENMEMORY, 0); //refresh LCD
}
//--------------------------------------------------------------------------------
void screenmemory_drawpixel(uint16_t X, uint16_t Y, uint8_t COLOR) {
  if (Y < DEFAULT_HEIGHT && X < DEFAULT_WIDTH) SCREENMEMORY[Y][X] = COLOR;
}
//--------------------------------------------------------------------------------
void screenmemory_line(int startx, int starty, int endx, int endy, uint8_t color) {
  int t, distance;
  int xerr = 0, yerr = 0, delta_x, delta_y;
  int incx, incy;
  // compute the distances in both directions
  delta_x = endx - startx;
  delta_y = endy - starty;
  // Compute the direction of the increment,
  //   an increment of 0 means either a horizontal or vertical
  //   line.
  if (delta_x > 0) incx = 1;
  else if (delta_x == 0) incx = 0;
  else incx = -1;

  if (delta_y > 0) incy = 1;
  else if (delta_y == 0) incy = 0;
  else incy = -1;

  // determine which distance is greater
  delta_x = abs(delta_x);
  delta_y = abs(delta_y);
  if (delta_x > delta_y) distance = delta_x;
  else distance = delta_y;

  // draw the line
  for (t = 0; t <= distance + 1; t++) {
    screenmemory_drawpixel(startx, starty, color);

    xerr += delta_x;
    yerr += delta_y;
    if (xerr > distance) {
      xerr -= distance;
      startx += incx;
    }
    if (yerr > distance) {
      yerr -= distance;
      starty += incy;
    }
  }
}
//--------------------------------------------------------------------------------
void screenmemory_drawrectangle(int16_t X, int16_t Y, int16_t Width, int16_t Height, uint8_t COLOR) {
  screenmemory_line(X, Y, X + Width, Y, COLOR);
  screenmemory_line(X, Y, X, Y + Height, COLOR);
  screenmemory_line(X + Width, Y, X + Width, Y + Height, COLOR);
  screenmemory_line(X, Y + Height, X + Width, Y + Height, COLOR);
}
//--------------------------------------------------------------------------------
void screenmemory_drawfillrectangle(int16_t X, int16_t Y, int16_t Width, int16_t Height, uint8_t COLOR) {
  for (uint16_t Ypos = Y; Ypos < Y + Height; Ypos++)
    for (uint16_t Xpos = X; Xpos < X + Width; Xpos++) {
      screenmemory_drawpixel(Xpos, Ypos, COLOR);
    }
  if (LCD_ENABLED) xQueueSend(vidQueue, &SCREENMEMORY, 0); //refresh LCD
}
//--------------------------------------------------------------------------------
uint8_t draw_char_xy(uint16_t Main_x, uint16_t Main_y, char Main_char, const char* font, uint8_t color = 48) {
  uint8_t XcharSize = font[0]; //x char size
  uint8_t YcharSize = font[1]; //y char size
  uint8_t CHAROFFSET = font[2]; //char start offset

  font[3]; //char count
  if (Main_char != '\n' && Main_char != '\r') for (uint16_t Ypos = 0; Ypos < YcharSize; Ypos++)
      for (uint16_t Xpos = 0; Xpos < XcharSize; Xpos += 8) {
        uint8_t CHARLINE = font[(Main_char - CHAROFFSET) * (YcharSize * (XcharSize / 8)) +   (Ypos) * (XcharSize / 8) + Xpos / 8  + 4];

        if ((Xpos + 0 < XcharSize) && (CHARLINE & 0b10000000) != 0) screenmemory_drawpixel(Main_x + Xpos + 0, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 0, Main_y + Ypos, 63);
        if ((Xpos + 1 < XcharSize) && (CHARLINE & 0b01000000) != 0) screenmemory_drawpixel(Main_x + Xpos + 1, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 1, Main_y + Ypos, 63);
        if ((Xpos + 2 < XcharSize) && (CHARLINE & 0b00100000) != 0) screenmemory_drawpixel(Main_x + Xpos + 2, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 2, Main_y + Ypos, 63);
        if ((Xpos + 3 < XcharSize) && (CHARLINE & 0b00010000) != 0) screenmemory_drawpixel(Main_x + Xpos + 3, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 3, Main_y + Ypos, 63);
        if ((Xpos + 4 < XcharSize) && (CHARLINE & 0b00001000) != 0) screenmemory_drawpixel(Main_x + Xpos + 4, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 4, Main_y + Ypos, 63);
        if ((Xpos + 5 < XcharSize) && (CHARLINE & 0b00000100) != 0) screenmemory_drawpixel(Main_x + Xpos + 5, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 5, Main_y + Ypos, 63);
        if ((Xpos + 6 < XcharSize) && (CHARLINE & 0b00000010) != 0) screenmemory_drawpixel(Main_x + Xpos + 6, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 6, Main_y + Ypos, 63);
        if ((Xpos + 7 < XcharSize) && (CHARLINE & 0b00000001) != 0) screenmemory_drawpixel(Main_x + Xpos + 7, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 7, Main_y + Ypos, 63);
      }
  return XcharSize;
}
//--------------------------------------------------------------------------------
uint16_t draw_string_xy(uint16_t x, uint16_t y, char *c, const char* font, uint8_t color = 48)
{
  uint8_t width;
  uint8_t XcharSize = font[0]; //x char size
  uint8_t YcharSize = font[1]; //y char size
  uint16_t textwidth = 0;
  while (*c) {
    width = draw_char_xy(x, y, *c, font, color);
    textwidth += (width);
    x += (width);
    c++;
  }
  if (LCD_ENABLED) xQueueSend(vidQueue, &SCREENMEMORY, 0);
  return textwidth;
}
//--------------------------------------------------------------------------------
const char* DisplayFontSet = NULL;
uint16_t XPOS_CHAR = 0;
uint16_t YPOS_CHAR = 0;
//--------------------------------------------------------------------------------
void set_font(const char* font) {
  DisplayFontSet = font;
}
//--------------------------------------------------------------------------------
void set_font_XY(uint16_t x, uint16_t y) {
  XPOS_CHAR = x;
  YPOS_CHAR = y;
}
//--------------------------------------------------------------------------------
void draw_string(char *c, uint8_t color = 48) {
  if (c[strlen(c) - 1] == '\n') {
    draw_string_xy(XPOS_CHAR, YPOS_CHAR, c, DisplayFontSet, color);
    YPOS_CHAR += (uint8_t)(DisplayFontSet[1]);
    XPOS_CHAR = 0;
  } else {
    XPOS_CHAR += draw_string_xy(XPOS_CHAR, YPOS_CHAR, c, DisplayFontSet, color);
  }
}
//--------------------------------------------------------------------------------
static void lcd_write_frame(const uint16_t x, const uint16_t y, const uint16_t width, const uint16_t height) {
  uint8_t n, l;
  uint16_t i, len;
  uint16_t w = TFT_WIDTH;
  uint16_t h = TFT_HEIGHT;
  uint16_t X_ = 0;

  for (i = 0; i < height; i++) {
    uint16_t horizontal_offset = 8;
    ///      for (X_ = 0; X_ < 240; X_++) { //not 256 the 240
    ///         SCREENBUFFER[X_] = nes_16bit[(0x3F & ((uint8_t *)SCREENMEMORY[i])[X_])];
    ///      }
    for (X_ = 8; X_ < 248; X_++) { //not 256 the 240
      SCREENBUFFER[X_] = nes_16bit[(0x3F & ((uint8_t *)SCREENMEMORY[i])[X_])];
    }


    ///PAL optimalisation in this case:
    tft.drawRGBBitmap(0, i, (uint16_t *)(SCREENBUFFER), 48, 1);
    tft.drawRGBBitmap(48, i, (uint16_t *)(SCREENBUFFER + 48), 48, 1);
    tft.drawRGBBitmap(96, i, (uint16_t *)(SCREENBUFFER + 96), 48, 1);
    tft.drawRGBBitmap(144, i, (uint16_t *)(SCREENBUFFER + 144), 48, 1);
    tft.drawRGBBitmap(192, i, (uint16_t *)(SCREENBUFFER + 192), 48, 1);
  }
}
//--------------------------------------------------------------------------------

int audiovideo_init() {
  // disable Core 0 WDT
  TaskHandle_t idle_0 = xTaskGetIdleTaskHandleForCPU(0);
  esp_task_wdt_delete(idle_0);

  ///stop driver for mp3 player
  ///   i2s_stop((i2s_port_t) 1);


  ///   init_sound(); //START AUDIO ??? here?

  vidQueue = xQueueCreate( 1, sizeof( unsigned int*  ));

  if (LCD_ENABLED) {
    xTaskCreatePinnedToCore(&videoTask, "videoTask", 2048, NULL, 0, NULL, 0 );
    if (DEBUG) Serial.println("videoTask Pinned To Core 0...");
  }
  return 0;
}
//--------------------------------------------------------------------------------







bool EXIT = false;

#define MAXFILES 512
#define MAXFILENAME_LENGTH 64
char* filename[MAXFILES];
char fileext[4];
#define FILESPERPAGE 8

SdFile dirFile;
SdFile file;

char* MAINPATH;
char textbuf[64] = {0};

//--------------------------------------------------------------------------------
void sortStrings(char* arr[], int n)
{
  char temp[MAXFILENAME_LENGTH];

  // Sorting strings using bubble sort
  for (int j = 0; j < n - 1; j++)
  {
    for (int i = j + 1; i < n; i++)
    {
      if (strcmp(arr[j], arr[i]) > 0)
      {
        strcpy(temp, arr[j]);
        strcpy(arr[j], arr[i]);
        strcpy(arr[i], temp);
      }
    }
  }
}
//--------------------------------------------------------------------------------
void secondsToHMS( const uint32_t seconds, uint16_t &h, uint8_t &m, uint8_t &s ) {
  uint32_t t = seconds;
  s = t % 60;
  t = (t - s) / 60;
  m = t % 60;
  t = (t - m) / 60;
  h = t;
}
//--------------------------------------------------------------------------------










//________________________________________________________________________________
//
// NES EMU SECTION
//________________________________________________________________________________

#include "NESemulator_part1.h"

//MAPPERS !!!
#include "mappers.h"

#include "NESemulator_part2.h"








//********************************************************************************



//--------------------------------------------------------------------------------
static void videoTask(void *arg) {
  int x = (320 - DEFAULT_WIDTH) / 2;
  int y = ((240 - DEFAULT_HEIGHT) / 2);
  while (1) {
    ///if (PLAYING) visualyze(); //visualyze when playing...

    if (LCD_ENABLED) {
      xQueueReceive(vidQueue, &SCREENMEMORY, portMAX_DELAY);
      lcd_write_frame(x, y, DEFAULT_WIDTH, DEFAULT_HEIGHT);
    }
  }
}
//--------------------------------------------------------------------------------

//********************************************************************************
IRAM_ATTR void MEMORY_STATUS() {
  if (DEBUG) {
    Serial.println();
    Serial.println("--------------------------------");

    Serial.print("TOTAL HEAP: ");
    Serial.println(ESP.getHeapSize());
    Serial.print("FREE HEAP: ");
    Serial.println(ESP.getFreeHeap());

    Serial.print("heap_caps_get_free_size: ");
    Serial.println(heap_caps_get_free_size(MALLOC_CAP_8BIT));
    Serial.println("--------------------------------");
  }
}
//********************************************************************************
//theese varaiables cannot be in while loop!
rominfo_t *rominfo;
unsigned char *romdata = 0;
char* PATH;
//================================================================================
char loadmessage[64];
unsigned char *getromdata(char* ROMFILENAME_)
{
   fp = SD.open(ROMFILENAME_);
   if (DEBUG) Serial.print("FILE SIZE: ");
   if (DEBUG) Serial.println(fp.size());
   FILE_ROM_SIZE = fp.size();

   uint16_t BLOCKCOUNT = (FILE_ROM_SIZE + SPI_FLASH_SECTOR_SIZE) / SPI_FLASH_SECTOR_SIZE;
   uint16_t BLOCKSIZEPX = DEFAULT_WIDTH / BLOCKCOUNT;
   Serial.print("BLOCKCOUNT: ");
   Serial.print(BLOCKCOUNT);
   Serial.print(" BLOCKSIZEPX: ");
   Serial.print(BLOCKSIZEPX);
   Serial.println();
   BLOCKSIZEPX++; 

   if (PSRAMSIZE > 0) {
      uint32_t i = 0;
      for (i = 0; i < fp.size(); i++) {
         PSRAM[i] = fp.read();
      }
      fp.close();
      return (unsigned char *)PSRAM;
   } else {
//********************************************************************************
      // Read NES rom to SPI FLASH!
      uint32_t i = 0;
      for (i = 0; i < fp.size() + SPI_FLASH_SECTOR_SIZE; i++) {
         if (DEBUGEXTRA && i % SPI_FLASH_SECTOR_SIZE == 0) {
            Serial.print("ROM loaded 4kB:");
            Serial.println(i / 0x1000);
         }
         if (i > 0 && i % SPI_FLASH_SECTOR_SIZE == 0) {
            ///spi_flash_erase_sector(SPI_FLASH_ADDRESS/SPI_FLASH_SECTOR_SIZE + (i/SPI_FLASH_SECTOR_SIZE)-SPI_FLASH_SECTOR_SIZE);
            delayMicroseconds(300);
            spi_flash_erase_range(SPI_FLASH_ADDRESS + i - SPI_FLASH_SECTOR_SIZE, sizeof(flashdata));
            delayMicroseconds(300);
            spi_flash_write(SPI_FLASH_ADDRESS + i - SPI_FLASH_SECTOR_SIZE, flashdata, sizeof(flashdata));
            delayMicroseconds(300);

            sprintf(loadmessage, " %d / %d", i, FILE_ROM_SIZE);
            set_font_XY(8, 8);
            draw_string("Loaded:");
            draw_string(loadmessage);
            screenmemory_drawfillrectangle(((i / SPI_FLASH_SECTOR_SIZE) - 1)* BLOCKSIZEPX, 24, BLOCKSIZEPX, 16, 57);
         }
         delayMicroseconds(50);
         if (fp.available()) flashdata[i % SPI_FLASH_SECTOR_SIZE] = fp.read();
         delayMicroseconds(50);
      }
      fp.close();

      FLASH_ROM_SIZE = i; //Size of File and Offset Align

      if (DEBUG) Serial.print("FLASH SIZE: ");
      if (DEBUG) Serial.println(FLASH_ROM_SIZE);

      ROM = 0;
      ///if (handle1) spi_flash_munmap(handle1);
      printf("Mapping %x (+%x)\n", SPI_FLASH_ADDRESS, FLASH_ROM_SIZE);
      ESP_ERROR_CHECK( spi_flash_mmap(SPI_FLASH_ADDRESS, FLASH_ROM_SIZE, SPI_FLASH_MMAP_DATA, &ROM, &handle1) );
      printf("mmap_res: handle=%d ptr=%p\n", handle1, ROM);
      Serial.println("[NES ROM MAPPED ON FLASH!]");
      return (unsigned char *)ROM; 
   }
}
//================================================================================





//********************************************************************************
//*  SETUP:                                                                      *
//********************************************************************************
void setup() {
   Serial.begin(115200);
   MEMORY_STATUS();

   if (ESP.getPsramSize() > 0) {
      PSRAMSIZE = ESP.getPsramSize();
      PSRAM = (uint8_t*)ps_malloc(2097152); //PSRAM malloc 2MB
   }

   Serial.print("Total PSRAM: ");
   Serial.println(ESP.getPsramSize());
   Serial.print("Free PSRAM: ");
   Serial.println(ESP.getFreePsram());


//--------------------------------------------------------------------------------
   // VIDEO MEMORY ALLOCATION (force)
   for (uint32_t tmp = 0; tmp < 240; tmp++) {
      SCREENMEMORY[tmp] = (uint8_t*)malloc(256 + 1 );
      memset(SCREENMEMORY[tmp], 0, 256);
   }
//--------------------------------------------------------------------------------
   //Buttons Pins Input Init
   pinMode(PIN_A, INPUT);    //A
   pinMode(PIN_B, INPUT);   //B
   pinMode(PIN_SELECT, INPUT);   //SELECT
   pinMode(PIN_START, INPUT);   //START
   pinMode(PIN_UP, INPUT);    //UP
   pinMode(PIN_DOWN, INPUT);   //DOWN  //TCK
   pinMode(PIN_LEFT, INPUT);       //LEFT
   pinMode(PIN_RIGHT, INPUT_PULLDOWN);  //RIGHT
//--------------------------------------------------------------------------------
#if LCD_ENABLED
   // if the display has CS pin try with SPI_MODE0
   tft.init(240, 240, SPI_MODE2);    // Init ST7789 display 240x240 pixel
   // if the screen is flipped, remove this command
   tft.setRotation(3);
   tft.setSPISpeed(80000000);
   tft.fillScreen(ST77XX_BLACK); 

   tft.println("NCAT SYSTEM: MEOW!..."); //??? :D
#endif
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
   //DAC COMPOSITE VIDEO & ADC has setup here:
#if COMPOSITE_VIDEO_ENABLED
   video_init(4, 2, nes_32bit, 1); // start the A/V pump on app core
#endif
//--------------------------------------------------------------------------------
   //SCREENMEMORY LCD DRAW INIT
   audiovideo_init();
//--------------------------------------------------------------------------------
   //PS2/USB KEYBOARD SUPPORT
#if KEYBOARD_ENABLED
   kb_begin();
#endif
//--------------------------------------------------------------------------------
   //INTRO TEXT
   screenmemory_fillscreen(63); //black color
   set_font(Retro8x16); //Very important
   set_font_XY(32, 240 / 2 - 8);
   draw_string("NESCat Emulator by Nathalis", 48);
   delay(500);
//--------------------------------------------------------------------------------
   I2S0.conf.rx_start = 0; /// stop DMA ADC
   I2S0.in_link.start = 0;
//--------------------------------------------------------------------------------
   //malloc MAINPATH for player browser
   MAINPATH = (char*)malloc(256);
//--------------------------------------------------------------------------------
   ///microSD CARD INIT
   if (!SD.begin(SD_CONFIG)) {
      Serial.println("SD error!");
   } else Serial.println("SD OK.");
//--------------------------------------------------------------------------------
   install_timer(60); //60Hz
//--------------------------------------------------------------------------------
  ///... SETUP DONE

  MEMORY_STATUS();
}


//********************************************************************************
//*  LOOP:                                                                       *
//********************************************************************************
void loop() {

  EXIT = 0;

      MEMORY_STATUS();
      //--------------------------------------------------------------------------------
      Serial.println("Starting NES EMULATOR appliication.");
      screenmemory_fillscreen(63); //black color

      init_sound(); //START AUDIO

      Serial.println("NES_POWER: ##################");

      if (NULL == NESmachine) NESmachine = nes_create();
      if (NULL == NESmachine) {
        if (DEBUG) Serial.println("Failed to create NES instance.");
        if (DEBUG) tft.println("Failed to create NES instance.");
        while (1) {} //FREEZE
      }

      sprintf(MAINPATH, "/NES/"); ///must be malloc(256);

      for (uint16_t tmp = 0; tmp < MAXFILES; tmp++) filename[tmp] = (char*)calloc(MAXFILENAME_LENGTH, sizeof(char));

      NESBrowse(MAINPATH);
      delay(300);

      for (uint16_t tmp = 0; tmp < MAXFILES; tmp++) free(filename[tmp]);

      if (EXIT) {
        return; //return from NESBrowse();
      }

      screenmemory_fillscreen(63); //black color

      if (DEBUG) Serial.print("SELECTED NES: ");
      if (DEBUG) Serial.println(MAINPATH);

      romdata = (unsigned char *)getromdata(MAINPATH);

      if (DEBUG) Serial.println("Inserting cartridge.");
      if (DEBUG) tft.println("Inserting cartridge.");

//--------------------------------------------------------------------------------

      if (NULL == rominfo) rominfo = (rominfo_t*)malloc(sizeof(rominfo_t));
      if (NULL == rominfo) goto rom_load_fail;

      memset(rominfo, 0, sizeof(rominfo_t));

      // Get the header and stick it into rominfo struct
      if (rom_getheader(&romdata, rominfo)) goto rom_load_fail;

      // Make sure we really support the mapper
      if (false == mmc_peek(rominfo->mapper_number)) {
        if (DEBUG) {
          Serial.print("Mapper not yet implemented:");
          Serial.println(rominfo->mapper_number);
          tft.print("Mapper not yet implemented:");
          tft.println(rominfo->mapper_number);
        }
        goto rom_load_fail;
      }

      // iNES format doesn't tell us if we need SRAM, so we have to always allocate it -- bleh!* UNIF, TAKE ME AWAY!  AAAAAAAAAA!!!
      if (rom_allocsram(rominfo)) goto rom_load_fail; ///NEED FOR example: SMB2-LostLevels 
      rom_loadtrainer(&romdata, rominfo);
      if (rom_loadrom(&romdata, rominfo)) goto rom_load_fail;

      NESmachine->rominfo = rominfo;

      goto rom_load_success;

rom_load_fail:
      if (DEBUG) {
        Serial.println("ROMLOAD FAIL.");
        tft.println("ROMLOAD FAIL.");
      }
      while (1) {} //FREEZE

rom_load_success:

//................................................................................

      if (DEBUG) {
        Serial.println("ROMLOAD SUCCESS.");
        tft.println("ROMLOAD SUCCESS.");
        Serial.println("Inserting Cartridge...");
        tft.println("Inserting Cartridge...");
      }

      // map cart's SRAM to CPU $6000-$7FFF
      if (NESmachine->rominfo->sram) {
        NESmachine->cpu->mem_page[6] = NESmachine->rominfo->sram;
        NESmachine->cpu->mem_page[7] = NESmachine->rominfo->sram + 0x1000;
      }
      
      // mapper
      NESmachine->mmc = mmc_create(NESmachine->rominfo);
      if (NULL == NESmachine->mmc) goto inscart_fail;

      // if there's VRAM, let the PPU know
      if (NULL != NESmachine->rominfo->vram) NESmachine->ppu->vram_present = true;
      if (SOUND_ENABLED) apu_setext(NESmachine->apu, NESmachine->mmc->intf->sound_ext);
      if (SOUND_ENABLED) osd_setsound(NESmachine->apu->process);
      build_address_handlers(NESmachine);
      nes_setcontext(NESmachine);
      nes_reset();

      goto inscart_success;

inscart_fail:
      if (DEBUG) {
        Serial.println("Insert Cartridge Fail!");
        tft.println("Insert Cartridge Fail!");
      }
      while (1) {} //FREEZE

inscart_success:
      if (DEBUG) {
        Serial.println("Insert Cartridge OK.");
        tft.println("Insert Cartridge OK.");
      }

      MEMORY_STATUS();
      
      //START!
      while (NES_POWER == 1) { //NES EMULATION LOOP
        nes_renderframe(true);
        /*#ifdef BLUETOOTH_ENABLED
              hid_update();
              PS4_JOY();
          #endif      */
      }
      delay(1000);
      osd_stopsound(); ///must be delayed because crash...
      if (rominfo->sram!=NULL) free(rominfo->sram); ///free SRAM

      MEMORY_STATUS();
      NES_POWER = 1;

  delay(250);

  JOY_SHARE = 0;
  JOY_OPTIONS = 0;
  EXIT = false;
}
//********************************************************************************
//*  ...END.                                                                     *
//********************************************************************************
