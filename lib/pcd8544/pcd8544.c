#include <stdlib.h>
#include "pcd8544.h"
#include "ks0108_font.h"

// the memory buffer for the LCD
uint8_t pcd8544_buffer[LCDWIDTH * LCDHEIGHT / 8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFC, 0xFE, 0xFF, 0xFC, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8,
    0xF8, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80, 0xC0, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x7F,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x87, 0x8F, 0x9F, 0x9F, 0xFF, 0xFF, 0xFF,
    0xC1, 0xC0, 0xE0, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC, 0xFC, 0xFE, 0xFE, 0xFE,
    0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xE0, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0xC0, 0xE0, 0xF1, 0xFB, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F, 0x0F, 0x0F, 0x87,
    0xE7, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x3F, 0xF9, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFD, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xF0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
    0x7E, 0x3F, 0x3F, 0x0F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFC, 0xF0, 0xE0, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFC, 0xF0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01,
    0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x1F, 0x3F, 0x7F, 0x7F,
    0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const Ks0108Char_t spaceChar = {2, {0x00, 0x00}};

#ifndef _swap_int16_t //thnx Adafruit GFX
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

void delay(uint32_t delay);
void send(uint8_t data);
void command(uint8_t cmd);
void reset(void);
void waitFinished(void);
const Ks0108Char_t *getCharacterW(uint16_t s);
const Ks0108Char_t *getCharacter(char s);
void pcd8544_GoTo(uint8_t x, uint8_t y);

void delay(uint32_t delay) {
  while (delay--) {
    __asm__("nop");
  }
}

void waitFinished(void) {
  /* Wait for transfer finished. */
  while (!(SPI_SR(PCD8544_SPI) & SPI_SR_TXE));
  /* Wait until not busy */
  while (SPI_SR(PCD8544_SPI) & SPI_SR_BSY);
  /* Wait for transfer finished. */
  //while (!(SPI_SR(PCD8544_SPI) & SPI_SR_RXNE));
}
void reset() {
  gpio_clear(PCD8544_SCE_PORT, PCD8544_SCE);
  gpio_clear(PCD8544_RST_PORT, PCD8544_RST);
  delay(10000);
  gpio_set(PCD8544_RST_PORT, PCD8544_RST);
  gpio_set(PCD8544_SCE_PORT, PCD8544_SCE);
}


void send(uint8_t data) {
  spi_xfer(PCD8544_SPI, data);
}

void command(uint8_t cmd) {
  DC_LOW;
  gpio_clear(PCD8544_SCE_PORT, PCD8544_SCE);
  delay(100);
  send(cmd);
  delay(100);
  gpio_set(PCD8544_SCE_PORT, PCD8544_SCE);
  delay(100);
}

void pcd8544_init(void) {
  reset();
  command(PCD8544_EXTENDED_MODE); // function set PD = 0 and V = 0, select extended instruction set (H = 1 mode)
  // command(0xBF); // VOP
  command(0x06); // Set Temp coefficent 2
  // command(0x14); // bias mode 1:48
  command(PCD8544_SETBIAS | PCD8544_BIAS); // Set Temp coefficent.  default is 0x04
  pcd8544_setContrast(0x38); //experimental determined
  command(PCD8544_COMMAND_MODE);// function set PD = 0 and V = 0, select normal instruction set (H = 0 mode)
  command(PCD8544_DISPLAY_MODE);// display control set normal mode (D = 1 and E = 0)
  pcd8544_display();
}

void pcd8544_setContrast(uint8_t contrast) {
  command(PCD8544_EXTENDED_MODE);
  command(PCD8544_SETVOP | contrast);
  command(PCD8544_COMMAND_MODE);
}

void pcd8544_display(void) {
  uint8_t c, maxC = LCDWIDTH - 1, p;
  for (p = 0; p < 6; p++) {
    command(PCD8544_SET_PAGE | p);
    c = 0;
    command(PCD8544_SET_ADDRESS | c);
    DC_HIGH;
    gpio_clear(PCD8544_SCE_PORT, PCD8544_SCE);
    for (; c < maxC; c++) {
      send(pcd8544_buffer[(LCDWIDTH * p) + c]); //todo make it through the DMA
    }
    gpio_set(PCD8544_SCE_PORT, PCD8544_SCE);
  }
}

void pcd8544_drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= LCDWIDTH) || (y < 0) || (y >= LCDHEIGHT))
    return;

  if (color)
    pcd8544_buffer[x + (y / 8) * LCDWIDTH] |= _BV(y % 8);
  else
    pcd8544_buffer[x + (y / 8) * LCDWIDTH] &= ~_BV(y % 8);
}

uint8_t pcd8544_getPixel(int8_t x, int8_t y) {
  if ((x < 0) || (x >= LCDWIDTH) || (y < 0) || (y >= LCDHEIGHT))
    return 0;

  return (pcd8544_buffer[x + (y / 8) * LCDWIDTH] >> (y % 8)) & 0x1;
}

void pcd8544_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  pcd8544_drawPixel(x0, y0 + r, color);
  pcd8544_drawPixel(x0, y0 - r, color);
  pcd8544_drawPixel(x0 + r, y0, color);
  pcd8544_drawPixel(x0 - r, y0, color);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    pcd8544_drawPixel(x0 + x, y0 + y, color);
    pcd8544_drawPixel(x0 - x, y0 + y, color);
    pcd8544_drawPixel(x0 + x, y0 - y, color);
    pcd8544_drawPixel(x0 - x, y0 - y, color);
    pcd8544_drawPixel(x0 + y, y0 + x, color);
    pcd8544_drawPixel(x0 - y, y0 + x, color);
    pcd8544_drawPixel(x0 + y, y0 - x, color);
    pcd8544_drawPixel(x0 - y, y0 - x, color);
  }
}

void pcd8544_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      pcd8544_drawPixel(y0, x0, color);
    } else {
      pcd8544_drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}


void pcd8544_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  pcd8544_drawHLine(x, y, w, color);
  pcd8544_drawHLine(x, y + h - 1, w, color);
  pcd8544_drawVLine(x, y, h, color);
  pcd8544_drawVLine(x + w - 1, y, h, color);
}

void pcd8544_drawVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
  pcd8544_drawLine(x, y, x, y + h - 1, color);
}

void pcd8544_drawHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  pcd8544_drawLine(x, y, x + w - 1, y, color);
}

void pcd8544_clearDisplay() {
  uint16_t length = LCDWIDTH * LCDHEIGHT / 8 - 1;
  for (int i = 0; i < length; i++)
    pcd8544_buffer[i] = 0x00;
}

const Ks0108Char_t *getCharacterW(uint16_t s) {
  int i = 0;
  for (; i < charTableSize; ++i) {
    if (charTable[i] == s)
      return chars[i];
  }
  return &spaceChar;
}

const Ks0108Char_t *getCharacter(char s) {
  int i = 0;
  uint16_t wide_s = (uint16_t) s;
  for (; i < charTableSize; ++i) {
    if (charTable[i] == wide_s)
      return chars[i];
  }
  return &spaceChar;
}

void pcd8544_GoTo(uint8_t x, uint8_t y) {
  command(PCD8544_SET_ADDRESS | x);
  command(PCD8544_SET_PAGE | (y / 8));
}

void
pcd8544_drawTextW(uint8_t x, uint8_t y, uint8_t color, wchar_t *text) {
  int charPos = 0;
  uint16_t symbol = 0x00;
  do {
    symbol = text[charPos++];
    if (symbol == L'\n') {
      y += 8;
      x = 0;
    } else {
      const Ks0108Char_t *charCur = getCharacterW(symbol);
      uint8_t cBites = (uint8_t) (y % 8);

      int i = 0;
      for (; i < charCur->size; i++) {
        if (x > LCDWIDTH)
          break;
        uint8_t calcSymLine = pcd8544_buffer[x + (y / 8) * LCDWIDTH];
        if (color)
          calcSymLine |= charCur->l[i] << cBites;
        else
          calcSymLine &= ~(charCur->l[i] << cBites);
        pcd8544_buffer[x + (y / 8) * LCDWIDTH] = calcSymLine;
        if (y < 40 && cBites > 0) {
          calcSymLine = pcd8544_buffer[x + ((y + 8) / 8) * LCDWIDTH];
          if (color)
            calcSymLine |= charCur->l[i] >> (8 - cBites);
          else
            calcSymLine &= ~(charCur->l[i] >> (8 - cBites));
          pcd8544_buffer[x + ((y + 8) / 8) * LCDWIDTH] = calcSymLine;
        }
        x += 1;
      }
    }


  } while (symbol != 0x00);
}

void
pcd8544_drawText(uint8_t x, uint8_t y, uint8_t color, char *text) {
  int charPos = 0;
  uint16_t symbol = 0x00;
  do {
    symbol = text[charPos++];
    if (symbol == L'\n') {
      y += 8;
      x = 0;
    } else {
      const Ks0108Char_t *charCur = getCharacter(symbol);
      uint8_t cBites = (uint8_t) (y % 8);

      int i = 0;
      for (; i < charCur->size; i++) {
        if (x > LCDWIDTH)
          break;
        uint8_t calcSymLine = pcd8544_buffer[x + (y / 8) * LCDWIDTH];
        if (color)
          calcSymLine |= charCur->l[i] << cBites;
        else
          calcSymLine &= ~(charCur->l[i] << cBites);
        pcd8544_buffer[x + (y / 8) * LCDWIDTH] = calcSymLine;
        if (y < 40 && cBites > 0) {
          calcSymLine = pcd8544_buffer[x + ((y + 8) / 8) * LCDWIDTH];
          if (color)
            calcSymLine |= charCur->l[i] >> (8 - cBites);
          else
            calcSymLine &= ~(charCur->l[i] >> (8 - cBites));
          pcd8544_buffer[x + ((y + 8) / 8) * LCDWIDTH] = calcSymLine;
        }
        x += 1;
      }
    }


  } while (symbol != 0x00);
}
