//
// Thermal_Printer Arduino library
//
// Copyright (c) 2020 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// Project started 1/6/2020
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <Arduino.h>
// uncomment this line to see debug info on serial monitor
#define DEBUG_OUTPUT

#include <NimBLEDevice.h>

#include "Thermal_Printer.h"

static char szPrinterName[32];
static int bb_width, bb_height; // back buffer width and height in pixels
static int tp_wrap, bb_pitch;
static int16_t iCursorX = 0, iCursorY = 0;
static uint8_t bWithResponse = 0; // default to not wait for a response
static uint8_t *pBackBuffer = NULL;
static uint8_t bConnected = 0;
static uint8_t bFound = 0; // flag to indicate if a printer was found during scan
static void tpWriteData(uint8_t *pData, int iLen);
extern "C" {
extern unsigned char ucFont[], ucBigFont[];
};
static void tpPreGraphics(int iWidth, int iHeight);
static void tpPostGraphics(void);
static void tpSendScanline(uint8_t *pSrc, int iLen);
static uint8_t CheckSum(uint8_t *pData, int iLen);
static void tpWriteCatCommandD8(uint8_t command, uint8_t data);
static void tpWriteCatCommandD16(uint8_t command, uint16_t data);

// X18-9556 specific UUIDs
const char *X18_SERVICE_UUID = "0000ae30-0000-1000-8000-00805f9b34fb";
const char *X18_TX_UUID = "0000ae01-0000-1000-8000-00805f9b34fb";
const char *X18_RX_UUID = "0000ae02-0000-1000-8000-00805f9b34fb";

// X18-9556 specific commands
const uint8_t paperRetract = 0xA0;
const uint8_t paperFeed = 0xA1;
const uint8_t setEnergy = 0xAF;
const uint8_t setDrawingMode = 0xBE;

// X18-9556 specific lattice control commands
const uint8_t x18LatticeStart[] = {0xaa, 0x55, 0x17, 0x38, 0x44, 0x5f, 0x5f, 0x5f, 0x44, 0x38, 0x2c};
const uint8_t x18LatticeEnd[] = {0xaa, 0x55, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17};
//OtherFeedPaper = 0xBD  # Data: one byte, set to a device-specific "Speed" value before printing
//#                              and to 0x19 before feeding blank paper

int i;

//CRC8 pre calculated values
const uint8_t cChecksumTable[] PROGMEM = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,  0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d, 
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,  0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,  0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd, 
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,  0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd, 
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,  0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,  0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a, 
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,  0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a, 
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,  0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,  0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4, 
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,  0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4, 
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,  0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,  0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34, 
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,  0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63, 
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,  0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,  0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83, 
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,  0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3};

/* Table of byte flip values to mirror-image incoming CCITT data */
const unsigned char ucMirror[256] PROGMEM =
     {0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
      0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
      0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
      0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
      0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
      0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
      0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
      0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
      0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
      0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
      0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
      0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
      0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
      0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
      0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
      0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF};



// X18-9556 specific NimBLE variables
static NimBLEScan *pBLEScan;
static NimBLEClient* pX18Client = nullptr;
static NimBLERemoteCharacteristic* pX18TxCharacteristic = nullptr;
static NimBLERemoteCharacteristic* pX18RxCharacteristic = nullptr;
static NimBLEAdvertisedDevice* pX18Device = nullptr;
static char Scanned_BLE_Name[32];

// Flow control variables
static bool bPaused = false;
static bool bDataFlowEnabled = true;
static int iPendingDataSize = 0;
static uint8_t ucPendingData[512];
static const int MTU_SIZE = 200;
static const int BUFFER_THRESHOLD = MTU_SIZE * 16;
// Flow control commands
const uint8_t dataFlowPause[] = {0x51, 0x78, 0xae, 0x01, 0x01, 0x00, 0x10, 0x70, 0xff};
const uint8_t dataFlowResume[] = {0x51, 0x78, 0xae, 0x01, 0x01, 0x00, 0x00, 0x00, 0xff};

void tpSetWriteMode(uint8_t bWriteMode)
{
   bWithResponse = bWriteMode;
} /* tpSetWriteMode() */

// NimBLE-specific callback class for X18-9556
class tpNimBLEAdvertisedDeviceCallbacks: public NimBLEScanCallbacks
{
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override
    {
      int iLen = strlen(szPrinterName);
#ifdef DEBUG_OUTPUT
      Serial.printf("Scan Result: %s \n", advertisedDevice->toString().c_str());
      Serial.printf("szPrinterName length: %d, value: '%s'\n", iLen, szPrinterName);
      Serial.printf("Device name: '%s'\n", advertisedDevice->getName().c_str());
#endif
      if (iLen > 0 && memcmp(advertisedDevice->getName().c_str(), szPrinterName, iLen) == 0)
      { // this is what we want
        pX18Device = new NimBLEAdvertisedDevice(*advertisedDevice);
        strcpy(Scanned_BLE_Name, advertisedDevice->getName().c_str());
#ifdef DEBUG_OUTPUT
        Serial.println("A match!");
        Serial.println(Scanned_BLE_Name);
#endif
        bFound = true;
      } else if (iLen == 0) { // check for supported printers
        char szName[32];
        strcpy(szName, advertisedDevice->getName().c_str());
#ifdef DEBUG_OUTPUT
        Serial.print("Checking device name: ");
        Serial.println(szName);
#endif
        if (strcmp(szName, "X18-9556") == 0) {
            pX18Device = new NimBLEAdvertisedDevice(*advertisedDevice);
            strcpy(Scanned_BLE_Name, szName);
            strcpy(szPrinterName, szName);
            bFound = true;
#ifdef DEBUG_OUTPUT
            Serial.print("A match! - ");
            Serial.println(Scanned_BLE_Name);
#endif
        }
      }
    }
}; // class tpNimBLEAdvertisedDeviceCallbacks

// Provide a back buffer for your printer graphics
// This allows you to manage the RAM used on
// embedded platforms like Arduinos
// The memory is laid out horizontally (384/576 pixels across = 48/72 bytes)
// So a 384x384 buffer would need to be 48x384 = 18432 bytes
//
void tpSetBackBuffer(uint8_t *pBuffer, int iWidth, int iHeight)
{
  pBackBuffer = pBuffer;
  bb_width = iWidth;
  bb_height = iHeight;
  bb_pitch = (iWidth + 7) >> 3;
} /* tpSetBackBuffer() */

//
// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
//
void tpFill(unsigned char ucData)
{
  if (pBackBuffer != NULL)
    memset(pBackBuffer, ucData, bb_pitch * bb_height);
} /* tpFill() */
//
// Turn text wrap on or off for the oldWriteString() function
//
void tpSetTextWrap(int bWrap)
{
  tp_wrap = bWrap;
} /* tpSetTextWrap() */
//
// Invert font data
//
static void InvertBytes(uint8_t *pData, uint8_t bLen)
{
uint8_t i;
   for (i=0; i<bLen; i++)
   {
      *pData = ~(*pData);
      pData++;
   }
} /* InvertBytes() */
//
// Return the measurements of a rectangle surrounding the given text string
// rendered in the given font
//
void tpGetStringBox(GFXfont *pFont, char *szMsg, int *width, int *top, int *bottom)
{
int cx = 0;
int c, i = 0;
GFXglyph *pGlyph;
int miny, maxy;

   if (width == NULL || top == NULL || bottom == NULL || pFont == NULL || szMsg == NULL) return; // bad pointers
   miny = 100; maxy = 0;
   while (szMsg[i]) {
      c = szMsg[i++];
      if (c < pFont->first || c > pFont->last) // undefined character
         continue; // skip it
      c -= pFont->first; // first char of font defined
      pGlyph = &pFont->glyph[c];
      cx += pGlyph->xAdvance;
      if (pGlyph->yOffset < miny) miny = pGlyph->yOffset;
      if (pGlyph->height+pGlyph->yOffset > maxy) maxy = pGlyph->height+pGlyph->yOffset;
   }
   *width = cx;
   *top = miny;
   *bottom = maxy;
} /* tpGetStringBox() */

//
// Draw a string of characters in a custom font into the gfx buffer
//
int tpDrawCustomText(GFXfont *pFont, int x, int y, char *szMsg)
{
int i, end_y, dx, dy, tx, ty, c, iBitOff;
uint8_t *s, *d, bits, ucMask, ucClr, uc;
GFXglyph glyph, *pGlyph;

   if (pBackBuffer == NULL || pFont == NULL || x < 0 || y > bb_height)
      return -1;
   pGlyph = &glyph;

   i = 0;
   while (szMsg[i] && x < bb_width)
   {
      c = szMsg[i++];
      if (c < pFont->first || c > pFont->last) // undefined character
         continue; // skip it
      c -= pFont->first; // first char of font defined
      memcpy_P(&glyph, &pFont->glyph[c], sizeof(glyph));
      dx = x + pGlyph->xOffset; // offset from character UL to start drawing
      dy = y + pGlyph->yOffset;
      s = pFont->bitmap + pGlyph->bitmapOffset; // start of bitmap data
      // Bitmap drawing loop. Image is MSB first and each pixel is packed next
      // to the next (continuing on to the next character line)
      iBitOff = 0; // bitmap offset (in bits)
      bits = uc = 0; // bits left in this font byte
      end_y = dy + pGlyph->height;
      if (dy < 0) { // skip these lines
          iBitOff += (pGlyph->width * (-dy));
          dy = 0;
      }
      for (ty=dy; ty<=end_y && ty < bb_height; ty++) {
         d = &pBackBuffer[ty * bb_pitch]; // internal buffer dest
         for (tx=0; tx<pGlyph->width; tx++) {
            if (uc == 0) { // need to read more font data
               tx += bits; // skip any remaining 0 bits
               uc = pgm_read_byte(&s[iBitOff>>3]); // get more font bitmap data
               bits = 8 - (iBitOff & 7); // we might not be on a byte boundary
               iBitOff += bits; // because of a clipped line
               uc <<= (8-bits);
               if (tx >= pGlyph->width) {
                  while(tx >= pGlyph->width) { // rolls into next line(s)
                     tx -= pGlyph->width;
                     ty++;
                  }
                  if (ty >= end_y) { // we're past the end
                     tx = pGlyph->width;
                     continue; // exit this character cleanly
                  }
                  d = &pBackBuffer[ty * bb_pitch];
               }
            } // if we ran out of bits
            if (uc & 0x80) { // set pixel
               ucMask = 0x80 >> ((dx+tx) & 7);
               d[(dx+tx)>>3] |= ucMask;
            }
            bits--; // next bit
            uc <<= 1;
         } // for x
      } // for y
      x += pGlyph->xAdvance; // width of this character
   } // while drawing characters
   return 0;
} /* tpDrawCustomText() */
//
// Print a string of characters in a custom font to the connected printer
//
int tpPrintCustomText(GFXfont *pFont, int startx, char *szMsg)
{
int i, x, y, end_y, dx, dy, tx, ty, c, iBitOff;
int maxy, miny, height;
uint8_t *s, *d, bits, ucMask, ucClr, uc;
GFXglyph glyph, *pGlyph;
uint8_t ucTemp[80]; // max width of 1 scan line (576 pixels)
int iPrintWidth = 384;

   if (!bConnected)
      return -1;
   if (pFont == NULL || x < 0)
      return -1;
   pGlyph = &glyph;

   // Get the size of the rectangle enclosing the text
//   tpGetStringBox(pFont, szMsg, &tx, &miny, &maxy);
//   height = (maxy - miny) + 1;

   tpPreGraphics(iPrintWidth, pFont->yAdvance);
   miny = 0 - (pFont->yAdvance * 2)/3; // 2/3 of char is above the baseline
   maxy = pFont->yAdvance + miny;
   for (y=miny; y<=maxy; y++)
   {
     i = 0;
     x = startx;
     memset(ucTemp, 0, sizeof(ucTemp));
     while (szMsg[i] && x < iPrintWidth)
     {
       c = szMsg[i++];
       if (c < pFont->first || c > pFont->last) // undefined character
         continue; // skip it
       c -= pFont->first; // first char of font defined
       memcpy_P(&glyph, &pFont->glyph[c], sizeof(glyph));
       dx = x + pGlyph->xOffset; // offset from character UL to start drawing
       dy = /*y +*/ pGlyph->yOffset;
       s = pFont->bitmap + pGlyph->bitmapOffset; // start of bitmap data
       // Bitmap drawing loop. Image is MSB first and each pixel is packed next
       // to the next (continuing on to the next character line)
       iBitOff = 0; // bitmap offset (in bits)
       bits = uc = 0; // bits left in this font byte
       end_y = dy + pGlyph->height;
       for (ty=dy; ty<=end_y; ty++) {
         for (tx=0; tx<pGlyph->width; tx++) {
            if (uc == 0) { // need to read more font data
               tx += bits; // skip any remaining 0 bits
               uc = pgm_read_byte(&s[iBitOff>>3]); // get more font bitmap data
               bits = 8 - (iBitOff & 7); // we might not be on a byte boundary
               iBitOff += bits; // because of a clipped line
               uc <<= (8-bits);
               if (tx >= pGlyph->width) {
                  while(tx >= pGlyph->width) { // rolls into next line(s)
                     tx -= pGlyph->width;
                     ty++;
                  }
                  if (ty >= end_y) { // we're past the end
                     tx = pGlyph->width;
                     continue; // exit this character cleanly
                  }
               }
            } // if we ran out of bits
            if (uc & 0x80 && ty == y) { // set pixel if we're drawing this line
               ucMask = 0x80 >> ((dx+tx) & 7);
               ucTemp[(dx+tx)>>3] |= ucMask;
            }
            bits--; // next bit
            uc <<= 1;
         } // for tx
      } // for ty
      x += pGlyph->xAdvance; // width of this character
    } // while drawing characters
    tpSendScanline(ucTemp, (iPrintWidth+7)/8); // send to printer 
  } // for each line of output
  tpPostGraphics();
  return 0;
} /* tpPrintCustomText() */
//
// Draw text into the graphics buffer
//
int tpDrawText(int x, int y, char *szMsg, int iFontSize, int bInvert)
{
int i, ty, iFontOff;
unsigned char c, *s, *d, ucTemp[64];

    if (x == -1 || y == -1) // use the cursor position
    {
      x = iCursorX; y = iCursorY;
    }
    else
    {
      iCursorX = x; iCursorY = y; // set the new cursor position
    }
    if (iCursorX >= bb_width || iCursorY >= bb_height-7)
       return -1; // can't draw off the display

    if (iFontSize == FONT_SMALL) // 8x8 font
    {
       i = 0;
       while (iCursorX < bb_width && szMsg[i] != 0 && iCursorY < bb_height)
       {
          c = (unsigned char)szMsg[i];
          iFontOff = (int)(c-32) * 8;
          memcpy(ucTemp, &ucFont[iFontOff], 8);
          if (bInvert) InvertBytes(ucTemp, 8);
          d = &pBackBuffer[(iCursorY * bb_pitch) + iCursorX/8];
          for (ty=0; ty<8; ty++)
          {
             d[0] = ucTemp[ty];
             d += bb_pitch;
          }
          iCursorX += 8;
          if (iCursorX >= bb_width && tp_wrap) // word wrap enabled?
          {
             iCursorX = 0; // start at the beginning of the next line
             iCursorY +=8;
          }
       i++;
       } // while
    return 0;
    } // 8x8
    else if (iFontSize == FONT_LARGE) // 16x32 font
    {
      i = 0;
      while (iCursorX < bb_width && iCursorY < bb_height-31 && szMsg[i] != 0)
      {
          s = (unsigned char *)&ucBigFont[(unsigned char)(szMsg[i]-32)*64];
          memcpy(ucTemp, s, 64);
          if (bInvert) InvertBytes(ucTemp, 64);
          d = &pBackBuffer[(iCursorY * bb_pitch) + iCursorX/8];
          for (ty=0; ty<32; ty++)
          {
             d[0] = s[0];
             d[1] = s[1];
             s += 2; d += bb_pitch;
          }
          iCursorX += 16;
          if (iCursorX >= bb_width && tp_wrap) // word wrap enabled?
          {
             iCursorX = 0; // start at the beginning of the next line
             iCursorY += 32;
          }
          i++;
       } // while
       return 0;
    } // 16x32
   return -1;
} /* tpDrawText() */
//
// Set (or clear) an individual pixel
//
int tpSetPixel(int x, int y, uint8_t ucColor)
{
uint8_t *d, mask;

  if (pBackBuffer == NULL)
     return -1;
  d = &pBackBuffer[(bb_pitch * y) + (x >> 3)];
  mask = 0x80 >> (x & 7);
  if (ucColor)
     d[0] |= mask;
  else
     d[0] &= ~mask;  
  return 0;
} /* tpSetPixel() */
//
// Load a 1-bpp Windows bitmap into the back buffer
// Pass the pointer to the beginning of the BMP file
// along with a x and y offset (upper left corner)
//
int tpLoadBMP(uint8_t *pBMP, int bInvert, int iXOffset, int iYOffset)
{
int16_t i16;
int iOffBits; // offset to bitmap data
int iPitch;
int16_t cx, cy, x, y;
uint8_t *d, *s, pix;
uint8_t srcmask, dstmask;
uint8_t bFlipped = false;

  i16 = pBMP[0] | (pBMP[1] << 8);
  if (i16 != 0x4d42) // must start with 'BM'
     return -1; // not a BMP file
  if (iXOffset < 0 || iYOffset < 0)
     return -1;
  cx = pBMP[18] + (pBMP[19] << 8);
  cy = pBMP[22] + (pBMP[23] << 8);
  if (cy > 0) // BMP is flipped vertically (typical)
     bFlipped = true;
  if (cx + iXOffset > bb_width || cy + iYOffset > bb_height) // too big
     return -1;
  i16 = pBMP[28] + (pBMP[29] << 8);
  if (i16 != 1) // must be 1 bit per pixel
     return -1;
  iOffBits = pBMP[10] + (pBMP[11] << 8);
  iPitch = (cx + 7) >> 3; // byte width
  iPitch = (iPitch + 3) & 0xfffc; // must be a multiple of DWORDS

  if (bFlipped)
  {
     iOffBits += ((cy-1) * iPitch); // start from bottom
     iPitch = -iPitch;
  }
  else
  {
     cy = -cy;
  }

// Send it to the gfx buffer
     for (y=0; y<cy; y++)
     {
         s = &pBMP[iOffBits + (y * iPitch)]; // source line
         d = &pBackBuffer[((iYOffset+y) * bb_pitch) + iXOffset/8];
         srcmask = 0x80; dstmask = 0x80 >> (iXOffset & 7);
         pix = *s++;
         if (bInvert) pix = ~pix;
         for (x=0; x<cx; x++) // do it a bit at a time
         {
           if (pix & srcmask)
              *d |= dstmask;
           else
              *d &= ~dstmask;
           srcmask >>= 1;
           if (srcmask == 0) // next pixel
           {
              srcmask = 0x80;
              pix = *s++;
              if (bInvert) pix = ~pix;
           }
           dstmask >>= 1;
           if (dstmask == 0)
           {
              dstmask = 0x80;
              d++;
           }
         } // for x
  } // for y
  return 0;
} /* tpLoadBMP() */
//
// Connection status
// true = connected, false = disconnected
//
int tpIsConnected(void)
{
  if (bConnected == 1) {
     // we are/were connected, check...
     if (pX18Client && pX18Client->isConnected())
        return 1;
     bConnected = 0; // change status to disconnected
  }
  return 0; // not connected
} /* tpIsConnected() */

// X18-9556 specific connection function using NimBLE library
// This function implements the connection retry logic from cat_test.ino
static int tpConnectX18_9556(void)
{
    Serial.printf("正在连接到 %s ...\n", Scanned_BLE_Name);
    
    const int MAX_RETRIES = 3;
    int retryCount = 0;
    bool connected = false;

    while (retryCount < MAX_RETRIES && !connected) {
        if (pX18Client != nullptr) {
            NimBLEDevice::deleteClient(pX18Client);
            pX18Client = nullptr;
        }

        pX18Client = NimBLEDevice::createClient();

        // Set connection parameters (loose parameters for better compatibility)
        pX18Client->setConnectionParams(24, 40, 0, 600);

        // Attempt to connect
        bool connectResult = pX18Client->connect(pX18Device, false);

        if (connectResult) {
            Serial.println("连接成功 (标准路径)!");
            connected = true;
        } else {
            Serial.println("连接返回 false，检查是否为假死状态...");
            delay(200);
            if (pX18Client->isConnected()) {
                Serial.println(">>> 判定为连接成功！(忽略了 status=2 错误)");
                connected = true;
            } else {
                Serial.printf("连接失败，重试 %d/%d\n", retryCount + 1, MAX_RETRIES);
                retryCount++;
                delay(500);
            }
        }
    }

    if (!connected) {
        Serial.println("连接彻底失败");
        return 0;
    }

    delay(200);

    uint16_t mtu = pX18Client->getMTU();
    Serial.printf("当前MTU: %d\n", mtu);

    delay(200);

    // Get service
    NimBLERemoteService* pRemoteService = pX18Client->getService(X18_SERVICE_UUID);
    if (!pRemoteService) {
        Serial.println("获取服务失败");
        pX18Client->disconnect();
        return 0;
    }

    // Get TX characteristic
    pX18TxCharacteristic = pRemoteService->getCharacteristic(X18_TX_UUID);
    if (!pX18TxCharacteristic) {
        Serial.println("未找到TX特征值");
        return 0;
    }

    // Get RX characteristic
    pX18RxCharacteristic = pRemoteService->getCharacteristic(X18_RX_UUID);
    if (!pX18RxCharacteristic) {
        Serial.println("未找到RX特征值");
        return 0;
    }

    // Set up notification callback for flow control
    pX18RxCharacteristic->subscribe(true, [](const NimBLERemoteCharacteristic* pChar, const uint8_t* pData, size_t length, bool isNotify) {
        if (length == 8) {
            // Check for flow control commands
            if (memcmp(pData, dataFlowPause, 8) == 0) {
                bPaused = true;
#ifdef DEBUG_OUTPUT
                Serial.println("Received flow pause signal");
#endif
            } else if (memcmp(pData, dataFlowResume, 8) == 0) {
                bPaused = false;
#ifdef DEBUG_OUTPUT
                Serial.println("Received flow resume signal");
#endif
            }
        }
    });

    return 1;
} /* tpConnectX18_9556() */

// X18-9556 multi-byte command function (used by both CAT and X18-9556)
static void tpWriteCatCommandMulti(uint8_t command, const uint8_t* payload, size_t payloadSize)
{
    uint8_t packet[256];
    size_t index = 0;
    packet[index++] = 0x51;
    packet[index++] = 0x78;
    packet[index++] = command;
    packet[index++] = 0x00;
    packet[index++] = payloadSize & 0xff;
    packet[index++] = 0x00;
    memcpy(&packet[index], payload, payloadSize);
    index += payloadSize;
    packet[index++] = CheckSum((uint8_t*)payload, payloadSize);
    packet[index++] = 0xff;
    
    tpWriteData(packet, index);
}

// X18-9556 lattice control functions
static void tpSendX18LatticeStart(void)
{
    tpWriteCatCommandMulti(0xa6, x18LatticeStart, sizeof(x18LatticeStart));
    delay(100);
}

static void tpSendX18LatticeEnd(void)
{
    tpWriteCatCommandMulti(0xa6, x18LatticeEnd, sizeof(x18LatticeEnd));
    delay(100);
}

// X18-9556 initialization function
// Implements the initialization sequence from cat_test.ino
static void tpUpdateDevice(void)
{
    tpWriteCatCommandD8(0xa9, 0x00);
    delay(50);
}

static void tpFlush(void)
{
    delay(200);
}

static void tpInitX18_9556(void)
{
    Serial.println("=== 开始初始化 ===");
    
    Serial.println("1. get_device_state(0xa3, 0x00)");
    tpWriteCatCommandD8(0xa3, 0x00); delay(100);
    
    Serial.println("2. start_printing(0xa3, 0x01)");
    tpWriteCatCommandD8(0xa3, 0x01); delay(50);
    
    Serial.println("3. set_dpi_as_200(0xa4, 0x36)");
    tpWriteCatCommandD8(0xa4, 0x36); delay(50);
    
    Serial.println("4. set_speed(0xbd, 0x10)");
    tpWriteCatCommandD8(0xbd, 0x10); delay(50);
    
    Serial.println("5. set_energy(0xaf, 0x7FFF)");
    tpWriteCatCommandD16(0xaf, 0x7FFF); delay(50);
    
    Serial.println("6. apply_energy(0xbe, 0x00)");
    tpWriteCatCommandD8(0xbe, 0x00); delay(50);
    
    Serial.println("7. update_device(0xa9, 0x00)");
    tpUpdateDevice();
    
    Serial.println("8. flush()");
    tpFlush();
    
    Serial.println("9. start_lattice()");
    tpSendX18LatticeStart(); delay(100);
    
    Serial.println("=== 初始化完成 ===");
} /* tpInitX18_9556() */

//
// After a successful scan, connect to the printer
// returns 1 if successful, 0 for failure
//
int tpConnect(void)
{
   return tpConnect(NULL);
} /* tpConnect() */

//
// After a successful scan, connect to X18-9556 printer
// returns 1 if successful, 0 for failure
//
int tpConnect(const char *szMacAddress)
{
    if (tpConnectX18_9556()) {
        Serial.println("连接成功，等待打印机稳定...");
        delay(2000);
        bConnected = 1;
        return 1;
    }
    return 0;
} /* tpConnect() */

void tpDisconnect(void)
{
  if (!bConnected) return;
  if (pX18Client != nullptr) {
      pX18Client->disconnect();
      bConnected = 0;
  }
} /* tpDisconnect() */

//
// Parameterless version
// finds supported printers automatically
//
int tpScan(void)
{
  return tpScan("", 5);
} /* tpScan() */

//
// Scan for X18-9556 printer
// returns true if found
// iSeconds = how many seconds to scan for devices
//
int tpScan(const char *szName, int iSeconds)
{
unsigned long ulTime;
int iLen = strlen(szName);
    
    strcpy(szPrinterName, szName);
    Scanned_BLE_Name[0] = 0;
    bFound = 0;
    
    NimBLEDevice::init("ESP32");
    pBLEScan = NimBLEDevice::getScan();
    
    if (pBLEScan != NULL) {
      pBLEScan->setScanCallbacks(new tpNimBLEAdvertisedDeviceCallbacks());
      pBLEScan->setActiveScan(true);
      pBLEScan->start(iSeconds * 1000, false);
    }
    
    ulTime = millis();
    while (!bFound && (millis() - ulTime) < iSeconds*1000L) {
       delay(10);
    }
    
    if (bFound) {
#ifdef DEBUG_OUTPUT
        Serial.println("Found X18-9556 printer!");
#endif
        return 1;
    }
    
#ifdef DEBUG_OUTPUT
    Serial.println("Didn't find a printer :(");
#endif
    return 0;
} /* tpScan() */
//
// Write data to X18-9556 printer over BLE
// Implements MTU size limit and data chunking
//
static void tpWriteData(uint8_t *pData, int iLen)
{
    if (!bConnected || !pX18TxCharacteristic)
        return;

    Serial.printf("tpWriteData: 发送 %d 字节\n", iLen);

    int offset = 0;
    while (offset < iLen) {
        while (bPaused) {
            delay(10);
        }

        int chunkSize = iLen - offset;
        if (chunkSize > MTU_SIZE) {
            chunkSize = MTU_SIZE;
        }

        Serial.printf("  发送chunk: offset=%d, size=%d\n", offset, chunkSize);
        pX18TxCharacteristic->writeValue(pData + offset, chunkSize, false);
        offset += chunkSize;
    }
    
    Serial.println("tpWriteData: 完成");
} /* tpWriteData() */

void tpWriteRawData(uint8_t *pData, int iLen) {
   tpWriteData(pData,iLen);
}

//
// Checksum
//
static uint8_t CheckSum(uint8_t *pData, int iLen)
{
int i;
uint8_t cs = 0;

    for (i=0; i<iLen; i++)
        cs = cChecksumTable[(cs ^ pData[i])];
    return cs;
} /* CheckSum() */

// Compose command for cat printer
// 0x51 0x78 -> prefix (STX)
// CC -> command
// 00 -> from PC to printer
// 01 -> one byte of data
// 00 -> upper byte for one byte
// DD -> data
// CRC -> checksum of data
// 0xFF -> suffix (ETX)

// call for one byte data
void tpWriteCatCommandD8(uint8_t command, uint8_t data)
{
// prepare blank command:
uint8_t ucTemp[9] = {0x51, 0x78, 0xCC, 0x00, 0x01, 0x00, 0xDD, 0xC0, 0xFF};
                   // prefix      cmd   dir    length    data  crc   suffix
    ucTemp[2] = command;				// add requested command
    ucTemp[6] = data;					// add requested data
    ucTemp[7] = cChecksumTable[data];	// add CRC
   tpWriteData(ucTemp,9);
}

// same call for two bytes data
void tpWriteCatCommandD16(uint8_t command, uint16_t data)
{
// prepare blank command:
uint8_t ucTemp[10] = {0x51, 0x78, 0xCC, 0x00, 0x02, 0x00, 0xDD, 0xDD, 0xC0, 0xFF};
                   // prefix      cmd   dir    length     data  data  crc   suffix
    ucTemp[2] = command;					// add requested command
    ucTemp[6] = (uint8_t)(data & 0xFF);	// add requested data
    ucTemp[7] = (uint8_t)(data >> 8);		// add requested data
    ucTemp[8] = CheckSum(ucTemp+6, 2);	// add CRC
    tpWriteData(ucTemp,10);
}




//
// Returns the BLE name of the connected printer
// as a zero terminated c-string
// Returns NULL if not connected
//
char *tpGetName(void)
{
   if (!bConnected) return NULL;
   return szPrinterName;
} /* tpGetName() */
//
// Return the printer width in pixels
// The printer needs to be connected to get this info
//
int tpGetWidth(void)
{
   if (!bConnected)
      return 0;
   return 384;
} /* tpGetWidth() */
//
// Feed the paper in scanline increments
//
void tpFeed(int iLines)
{
  if (!bConnected || iLines < 0 || iLines > 255)
    return;
  tpWriteCatCommandD16(paperFeed,iLines);
} /* tpFeed() */
//
// tpSetEnergy Set Energy - switch between eco and nice images :) 
//
void tpSetEnergy(int iEnergy)
{
  if (bConnected)
     tpWriteCatCommandD16(setEnergy,iEnergy);
} /* tpSetEnergy() */
//
// Send the preamble for transmitting graphics to X18-9556
//
static void tpPreGraphics(int iWidth, int iHeight)
{
  tpWriteCatCommandD8(setDrawingMode, 0);
  tpSendX18LatticeStart();
} /* tpPreGraphics() */

static void tpPostGraphics(void)
{
   tpSendX18LatticeEnd();
   tpWriteCatCommandD8(0xbd, 0x08);
   tpWriteCatCommandD16(0xa1, 0x0080);
   tpWriteCatCommandD8(0xa3, 0x00);
   tpFlush();
} /* tpPostGraphics() */

static void tpSendScanline(uint8_t *s, int iLen)
{
      uint8_t ucTemp[256];
      for (int i=0; i<iLen; i++) {
        ucTemp[i] = ucMirror[s[i]];
      }
      tpWriteCatCommandMulti(0xa2, ucTemp, iLen);
      delay(30);
} /* tpSendScanline() */

//
// Send the graphics to the printer (must be connected over BLE first)
//
void tpPrintBuffer(void)
{
    uint8_t *s, ucTemp[8];
    int y;
    int i;

    if (!bConnected)
        return;

    static bool bInitialized = false;
    if (!bInitialized) {
        tpInitX18_9556();
        bInitialized = true;
    }

    tpPreGraphics(bb_width, bb_height);

  // Print the graphics
  s = pBackBuffer;
  for (y=0; y<bb_height; y++) {
    tpSendScanline(s, bb_pitch);
    s += bb_pitch;
  } // for y
  
  tpPostGraphics();

} /* tpPrintBuffer() */

void tpPrintBufferSide(void)
{
uint8_t *s;
int x, y;
uint8_t line[bb_pitch] = {0};

  if (!bConnected)
    return;

  tpPreGraphics(bb_height, bb_width);
  // Print the graphics
  s = pBackBuffer;
  for (y=0; y<bb_width; y++) {
    for (x=0; x<bb_height; x++)
    {
      line[x/8] = (line[x/8] << 1) | (((*(s+((x+1)*bb_width-1-y)/8)) >> (y%8))&1);
    }
    tpSendScanline(line, bb_pitch);
  } // for y
  
  tpPostGraphics();

} /* tpPrintBufferSide() */

//
// Draw a line between 2 points
//
void tpDrawLine(int x1, int y1, int x2, int y2, uint8_t ucColor)
{
  int temp;
  int dx = x2 - x1;
  int dy = y2 - y1;
  int error;
  uint8_t *p, mask;
  int xinc, yinc;

  if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0 || x1 >= bb_width || x2 >= bb_width || y1 >= bb_height || y2 >= bb_height)
     return;

  if(abs(dx) > abs(dy)) {
    // X major case
    if(x2 < x1) {
      dx = -dx;
      temp = x1;
      x1 = x2;
      x2 = temp;
      temp = y1;
      y1 = y2;
      y2 = temp;
    }

    dy = (y2 - y1);
    error = dx >> 1;
    yinc = 1;
    if (dy < 0)
    {
      dy = -dy;
      yinc = -1;
    }
    p = &pBackBuffer[(y1 * bb_pitch) + (x1 >> 3)]; // point to current spot in back buffer
    mask = 0x80 >> (x1 & 7); // current bit offset
    for(; x1 <= x2; x1++) {
      if (ucColor)
        *p |= mask; // set pixel and increment x pointer
      else
        *p &= ~mask;
      mask >>= 1;
      if (mask == 0) {
         mask = 0x80;
         p++;
      }
      error -= dy;
      if (error < 0)
      {
        error += dx;
        if (yinc > 0)
           p += bb_pitch;
        else
           p -= bb_pitch;
      }
    } // for x1
  }
  else {
    // Y major case
    if(y1 > y2) {
      dy = -dy;
      temp = x1;
      x1 = x2;
      x2 = temp;
      temp = y1;
      y1 = y2;
      y2 = temp;
    }

    p = &pBackBuffer[(y1 * bb_pitch) + (x1 >> 3)]; // point to current spot in back buffer
    mask = 0x80 >> (x1 & 7); // current bit offset
    dx = (x2 - x1);
    error = dy >> 1;
    xinc = 1;
    if (dx < 0)
    {
      dx = -dx;
      xinc = -1;
    }
    for(; y1 <= y2; y1++) {
      if (ucColor)
         *p |= mask; // set the pixel
      else
         *p &= ~mask;
      p += bb_pitch; // y++
      error -= dx;
      if (error < 0)
      {
        error += dy;
        x1 += xinc;
        if (xinc > 0)
        {
          mask >>= 1;
          if (mask == 0) // change the byte
          {
             p++;
             mask = 0x80;
          }
        } // positive delta x
        else // negative delta x
        {
          mask <<= 1;
          if (mask == 0)
          {
             p--;
             mask = 1;
          }
        }
      }
    } // for y
  } // y major case
} /* tpDrawLine() */
