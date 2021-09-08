//===========================================================================
//
//  Code written for Seeeduino v4.2 set to 3.3v (important!)
//
//  CRYSTALFONTZ CFAF320240F-035T 320x240 COLOR 3.5" TFT
//    https://www.crystalfontz.com/product/cfaf320240f035t-lcd-graphical-tft-display-module-320x240
//
//  +Touch Screen
//     https://www.crystalfontz.com/product/cfaf320240f035tts-lcd-module-tft-320x240-graphic-touch-screen
//
//  This code uses the 4-wire SPI mode of the display.
//
//  The controller is a Sitronix ST2119V:
//    https://www.crystalfontz.com/controllers/Solomon%20Systech/SSD2119
//
//  Seeeduino v4.2, an open-source 3.3v capable Arduino clone.
//    https://www.seeedstudio.com/Seeeduino-V4.2-p-2517.html
//    https://github.com/SeeedDocument/SeeeduinoV4/raw/master/resources/Seeeduino_v4.2_sch.pdf
//
//===========================================================================
//This is free and unencumbered software released into the public domain.
//
//Anyone is free to copy, modify, publish, use, compile, sell, or
//distribute this software, either in source code form or as a compiled
//binary, for any purpose, commercial or non-commercial, and by any
//means.
//
//In jurisdictions that recognize copyright laws, the author or authors
//of this software dedicate any and all copyright interest in the
//software to the public domain. We make this dedication for the benefit
//of the public at large and to the detriment of our heirs and
//successors. We intend this dedication to be an overt act of
//relinquishment in perpetuity of all present and future rights to this
//software under copyright law.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
//OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
//OTHER DEALINGS IN THE SOFTWARE.
//
//For more information, please refer to <http://unlicense.org/>
//============================================================================
//#include <avr/io.h>

#include <SPI.h>
// C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\SPI\src\SPI.cpp
// C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\SPI\src\SPI.h

//A CFA10112 micro SD card adapter can be used: https://www.crystalfontz.com/product/cfa10112
#include <SD.h>
// C:\Program Files (x86)\Arduino\libraries\SD\src\SD.cpp
// C:\Program Files (x86)\Arduino\libraries\SD\src\SD.h

#include <util/delay.h>
#include <avr/pgmspace.h>

//============================================================================
// LCD SPI & control lines
//   ARD      | Port  | LCD
// -----------+-------+-------------------------
//  #2/D2     |  PD2  | SD_CS    
//  #3/D3     |  PD3  | Backlight
//  #3/D3     |  PD3  | PS0
//  #3/D3     |  PD3  | PS1
//  #3/D3     |  PD3  | PS2
//  #3/D3     |  PD3  | PS3
//  #8/D8     |  PB0  | LCD_RS
//  #9/D9     |  PB1  | LCD_RESET
// #10/D10    |  PB2  | LCD_CS_NOT (or SPI SS)
// #11/D11    |  PB3  | LCD_MOSI   (hardware SPI)
// #12/D12    |  PB4  | LCD_MISO   (hardware SPI)
// #13/D13    |  PB5  | LCD_SCK    (hardware SPI)
// #23/D14/A0 |  PC0  | Touch XL   (only used on TS modules)
// #24/D15/A1 |  PC1  | Touch XR   (only used on TS modules)
// #25/D16/A2 |  PC2  | Touch YD   (only used on TS modules)
// #26/D17/A3 |  PC3  | Touch YU   (only used on TS modules)
//
//============================================================================
//Interface possibilities table
//			| 6800 – 8 Bit | 6800 – 9 Bit | 6800 – 16 Bit | 6800 – 18 bit | 8080 – 8 Bit | 8080 – 9 Bit | 8080 – 16 Bit | 8080 – 18 Bit | SPI – 4 Wire | SPI – 3 Wire | RGB – 262K | RGB – 64K |
//  PS3 | 0            | 1            | 0             | 1             | 0            | 1            | 0             | 1             | 1            | 1            | 0          | 0         |
//  PS2 | 0            | 0            | 0             | 0             | 0            | 0            | 0             | 0             | 1            | 1            | 1          | 1         |
//  PS1 | 0            | 0            | 0             | 0             | 1            | 1            | 1             | 1             | 1            | 1            | 1          | 0         |
//  PS0 | 1            | 1            | 0             | 0             | 1            | 1            | 0             | 0             | 1            | 0            | 0          | 1         |
//
//============================================================================
//set the macros to control the pins by port manipulation - much faster digitalwrite()
#define CLR_RS    (PORTB &= ~(0x01))  //CLR_RS      digitalWrite(8,HIGH)
#define SET_RS    (PORTB |=  (0x01))  //CLR_RS      digitalWrite(8, LOW)
#define CLR_RESET (PORTB &= ~(0x02))  //CLR_RESET   digitalWrite(9,HIGH)
#define SET_RESET (PORTB |=  (0x02))  //SET_RESET   digitalWrite(9, LOW)
#define CLR_CS    (PORTB &= ~(0x04))  //CLR_CS      digitalWrite(10,HIGH)
#define SET_CS    (PORTB |=  (0x04))  //SET_CS      digitalWrite(10, LOW)
#define CLR_MOSI  (PORTB &= ~(0x08))  //CLR_MOSI    digitalWrite(11,HIGH)
#define SET_MOSI  (PORTB |=  (0x08))  //SET_MOSI    digitalWrite(11, LOW)
#define CLR_SCK   (PORTB &= ~(0x20))  //CLR_SCK     digitalWrite(13,HIGH)
#define SET_SCK   (PORTB |=  (0x20))  //SET_SCK     digitalWrite(13, LOW)

#define BL_PIN  (0x08)
#define PS0_PIN (0x10)
#define PS1_PIN (0x20)
#define PS2_PIN (0x40)
#define PS3_PIN (0x80)

#define CLR_BL     (PORTD &= ~BL_PIN)
#define SET_BL     (PORTD |=  BL_PIN)
#define CLR_PS0    (PORTD &= ~PS0_PIN)
#define SET_PS0    (PORTD |=  PS0_PIN)
#define CLR_PS1    (PORTD &= ~PS1_PIN)
#define SET_PS1    (PORTD |=  PS1_PIN)
#define CLR_PS2    (PORTD &= ~PS2_PIN)
#define SET_PS2    (PORTD |=  PS2_PIN)
#define CLR_PS3    (PORTD &= ~PS3_PIN)
#define SET_PS3    (PORTD |=  PS3_PIN)

#define TS_XL (14)
#define TS_XR (15)
#define TS_YD (16)
#define TS_YU (17)

//#define SYNC_PIN (6)
#define SD_CS    (2)

//============================================================================
void SPI_sendCommand(uint8_t command)
  {
  // Select the LCD's command register
  CLR_RS;
  // Select the LCD controller
  CLR_CS;
  //Send the command via SPI:
  SPI.transfer(command);
  // Deselect the LCD controller
  SET_CS;
  }
//----------------------------------------------------------------------------
void SPI_sendData(uint16_t data)
  {
  // Select the LCD's data register
  SET_RS;
  // Select the LCD controller
  CLR_CS;
  //Send the MSB data via SPI:
  SPI.transfer(data>>8);
  //Send the LSB data via SPI:
  SPI.transfer(data&0x00FF);
  // Deselect the LCD controller
  SET_CS;
  }
//----------------------------------------------------------------------------
// init ref: https://www.crystalfontz.com/controllers/Solomon%20Systech/SSD2119
//----------------------------------------------------------------------------
void Initialize_LCD(void)
  {
    Serial.println("Initialize_LCD");
  //Reset the LCD controller
  CLR_RESET;
  delay(1);//10µS min
  SET_RESET;
  delay(150);//120mS max

  SPI_sendCommand(0x0028);    // VCOM OTP
  SPI_sendData(0x0006);       // Page 55-56 of SSD2119 datasheet

  SPI_sendCommand(0x0000);    // start Oscillator
  SPI_sendData(0x0001);       // Page 36 of SSD2119 datasheet

  SPI_sendCommand(0x0010);    // Sleep mode
  SPI_sendData(0x0000);       // Page 49 of SSD2119 datasheet

  SPI_sendCommand(0x0001);    // Driver Output Control
  //SPI_sendData(0x32EF);       // Page 36-39 of SSD2119 datasheet
  //Mirror RL, so 0,0 is in lower-left (quadrant 1)
  SPI_sendData(0x72EF);       // Page 36-39 of SSD2119 datasheet

  SPI_sendCommand(0x0002);    // LCD Driving Waveform Control
  SPI_sendData(0x0600);       // Page 40-42 of SSD2119 datasheet

  SPI_sendCommand(0x0003);    // Power Control 1
  SPI_sendData(0x6A38);       // Page 43-44 of SSD2119 datasheet

  SPI_sendCommand(0x0011);    // Entry Mode
  SPI_sendData(0x6870);       // Page 50-52 of SSD2119 datasheet

  SPI_sendCommand(0X000F);    // Gate Scan Position
  SPI_sendData(0x0000);       // Page 49 of SSD2119 datasheet

  SPI_sendCommand(0X000B);    // Frame Cycle Control
  SPI_sendData(0x5308);       // Page 45 of SSD2119 datasheet

  SPI_sendCommand(0x000C);    // Power Control 2
  SPI_sendData(0x0003);       // Page 47 of SSD2119 datasheet

  SPI_sendCommand(0x000D);    // Power Control 3
  SPI_sendData(0x000A);       // Page 48 of SSD2119 datasheet

  SPI_sendCommand(0x000E);    // Power Control 4
  SPI_sendData(0x2E00);       // Page 48 of SSD2119 datasheet

  SPI_sendCommand(0x001E);    // Power Control 5
  SPI_sendData(0x00B7);       // Page 55 of SSD2119 datasheet

  SPI_sendCommand(0x0025);    // Frame Frequency Control
  SPI_sendData(0x8000);       // Page 53 of SSD2119 datasheet

  SPI_sendCommand(0x0026);    // Analog setting
  SPI_sendData(0x3800);       // Page 54 of SSD2119 datasheet
  
  SPI_sendCommand(0x0027);    // Critical setting to avoid pixel defect
  SPI_sendData(0x0078);       // per solomon systech, apparently undocumented.

  SPI_sendCommand(0x004E);    // Ram Address Set
  SPI_sendData(0x0000);       // Page 58 of SSD2119 datasheet

  SPI_sendCommand(0x004F);    // Ram Address Set
  SPI_sendData(0x0000);       // Page 58 of SSD2119 datasheet

  SPI_sendCommand(0x0012);    // Sleep mode
  SPI_sendData(0x0D99);       // Page 49 of SSD2119 datasheet

  // Gamma Control (R30h to R3Bh) -- Page 56 of SSD2119 datasheet
  SPI_sendCommand(0x0030);
  SPI_sendData(0x0000);

  SPI_sendCommand(0x0031);
  SPI_sendData(0x0104);

  SPI_sendCommand(0x0032);
  SPI_sendData(0x0100);

  SPI_sendCommand(0x0033);
  SPI_sendData(0x0305);

  SPI_sendCommand(0x0034);
  SPI_sendData(0x0505);

  SPI_sendCommand(0x0035);
  SPI_sendData(0x0305);

  SPI_sendCommand(0x0036);
  SPI_sendData(0x0707);

  SPI_sendCommand(0x0037);
  SPI_sendData(0x0300);

  SPI_sendCommand(0x003A);
  SPI_sendData(0x1200);

  SPI_sendCommand(0x003B);
  SPI_sendData(0x0800);    

  SPI_sendCommand(0x0007);    // Display Control 
  SPI_sendData(0x0033);       // Page 45 of SSD2119 datasheet

  SPI_sendCommand(0x0044);    // Vertical RAM address position
  SPI_sendData(0xEF00);       // Page 57 of SSD2119 datasheet
  SPI_sendCommand(0x0045);    // Horizontal RAM address position 
  SPI_sendData(0x0000);       // Page 57 of SSD2119 datasheet
  SPI_sendCommand(0x0046);    // Horizontal RAM address position
  SPI_sendData(0x013F);       // Page 57 of SSD2119 datasheet

  SPI_sendCommand(0x0022);    // RAM data write/read

  SPI_sendCommand(0x10);    // set Sleep mode
  SPI_sendData(0x0000);//sleep OUT 
     delay(200);
  SPI_sendCommand(0x07);    // Display Control 
  SPI_sendData(0x0033);  // Display on
}
//============================================================================
void Set_LCD_for_write_at_X_Y(uint16_t x, uint16_t y)
  {
  SPI_sendCommand(0x4E);    // RAM address set, X
  SPI_sendData(x);          // Page 59 of SSD2119 datasheet
  SPI_sendCommand(0x4F);    // RAM address set, Y
  SPI_sendData(y);          // Page 59 of SSD2119 datasheet
  SPI_sendCommand(0x0022);    // RAM data write/read
  }
//============================================================================
void Fill_LCD(uint8_t R, uint8_t G, uint8_t B)
  {
    Serial.println("Filling LCD");
  uint32_t
    i;
  Set_LCD_for_write_at_X_Y(0, 0);

  //Pre-calculate the two bytes for this color of pixel
  uint8_t
    first_half;
  uint8_t
    second_half;
  //The display takes two bytes (565) RRRRR GGGGGG BBBBB 
  //to show one pixel.
  first_half=(R&0xF8) | (G >> 5);
  second_half=((G&0xFC)<<3) | (B >> 3);

  // Select the LCD controller
  CLR_CS;
  // Select the LCD controller's data register
  SET_RS;

  //Fill display with a given RGB value
  for (i = 0; i < (320UL * 240UL); i++)
    {
    SPI.transfer(first_half);
    SPI.transfer(second_half);
    }
  // Deselect the OLED controller
  SET_CS;      
  }
//============================================================================
void Put_Pixel(uint16_t x, uint16_t y, uint8_t R, uint8_t G, uint8_t B)
  {
  Set_LCD_for_write_at_X_Y(x, y);
  //Write the single, 16-bit pixel's worth of data
  SPI_sendData(((R&0xF8) << 8) | ((G&0xFC) << 3) | (B >> 3));
  }
//============================================================================
// From: http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
void LCD_Circle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t R, uint16_t G, uint16_t B)
  {
  uint16_t x = radius;
  uint16_t y = 0;
  int16_t radiusError = 1 - (int16_t) x;

  while (x >= y)
    {
    //11 O'Clock
    Put_Pixel(x0 - y, y0 + x, R, G, B);
    //1 O'Clock
    Put_Pixel(x0 + y, y0 + x, R, G, B);
    //10 O'Clock
    Put_Pixel(x0 - x, y0 + y, R, G, B);
    //2 O'Clock
    Put_Pixel(x0 + x, y0 + y, R, G, B);
    //8 O'Clock
    Put_Pixel(x0 - x, y0 - y, R, G, B);
    //4 O'Clock
    Put_Pixel(x0 + x, y0 - y, R, G, B);
    //7 O'Clock
    Put_Pixel(x0 - y, y0 - x, R, G, B);
    //5 O'Clock
    Put_Pixel(x0 + y, y0 - x, R, G, B);

    y++;
    if (radiusError < 0)
      radiusError += (int16_t)(2 * y + 1);
    else
      {
      x--;
      radiusError += 2 * (((int16_t) y - (int16_t) x) + 1);
      }
    }
  }
//============================================================================
#define mSwap(a,b,t)\
  {\
  t=a;\
  a=b;\
  b=t;\
  }\
//----------------------------------------------------------------------------
void Fast_Horizontal_Line(uint16_t x0, uint16_t y, uint16_t x1,
                          uint8_t R, uint8_t G, uint8_t B)
  {
  uint16_t
    temp;
  uint8_t
    first_half;
  uint8_t
    second_half;    
  if(x1 < x0)
    mSwap(x0, x1, temp);
  Set_LCD_for_write_at_X_Y(x0, y);

  // Select the LCD's data register
  SET_RS;
  // Select the LCD controller
  CLR_CS;
  
  //Pre-calculate the two bytes for this color of pixel
  //The display takes two bytes (565) RRRRR GGGGGG BBBBB 
  //to show one pixel.
  first_half=(R&0xF8) | (G >> 5);
  second_half=((G&0xFC)<<3) | (B >> 3);
    
  while(x0 <= x1)
    {
    //Write the single pixel's worth of data
    SPI.transfer(first_half);
    x0++;
    SPI.transfer(second_half);
    }
  }
//============================================================================
// From: http://rosettacode.org/wiki/Bitmap/Bresenham's_line_algorithm#C
void LCD_Line(uint16_t x0, uint16_t y0,
              uint16_t x1, uint16_t y1,
              uint8_t r, uint8_t g, uint8_t b)
  {
  int16_t
    dx;
  int16_t
    sx;
  int16_t
    dy;
  int16_t
    sy;
  int16_t
    err;
  int16_t
   e2;

  //General case
  if (y0 != y1)
    {
    dx = abs((int16_t )x1 - (int16_t )x0);
    sx = x0 < x1 ? 1 : -1;
    dy = abs((int16_t )y1 - (int16_t )y0);
    sy = y0 < y1 ? 1 : -1;
    err = (dx > dy ? dx : -dy) / 2;

    for (;;)
      {
      Put_Pixel(x0, y0, r,g,b);
      if ((x0 == x1) && (y0 == y1))
        break;
      e2 = err;
      if (e2 > -dx)
        {
        err -= dy;
        x0 = (uint16_t)((int16_t) x0 + sx);
        }
      if (e2 < dy)
        {
        err += dx;
        y0 = (uint16_t)((int16_t) y0 + sy);
        }
      }
    }
  else
    {
    //Optimized for LCD
    Fast_Horizontal_Line(x0, y0, x1,r,g,b);
    }
  }  
//============================================================================
// This function transfers data, in one stream. Slightly
// optimized to do index operations during SPI transfers.
void SPI_send_pixels(uint8_t pixel_count, uint8_t *data_ptr)
  {
  uint8_t
    r;
  uint8_t
    g;
  uint8_t
    b;
  uint8_t
    first_half;
  uint8_t
    second_half;
    
  // Select the LCD's data register
  SET_RS;
  // Select the LCD controller
  CLR_CS;

  //Load the first pixel. BMPs BGR format
  b=*data_ptr;
  data_ptr++;
  g=*data_ptr;
  data_ptr++;
  r=*data_ptr;
  data_ptr++;  

  //The display takes two bytes (565) RRRRR GGGGGG BBBBB 
  //to show one pixel.

  first_half = (r & 0xF8) | (g >> 5);
  second_half = ((g & 0xFC) << 3) | (b >> 3);

  while(pixel_count)
    {
    //Send the first half of this pixel out
    SPDR = first_half;
    //Load the next pixel while that is transmitting
    b=*data_ptr;
    data_ptr++;
    g=*data_ptr;
    data_ptr++;
    r=*data_ptr;
    data_ptr++;
    //Calculate the next first half while that is transmitting
    // ~1.9368us -0.1256 us = 1.8112uS
    first_half = (r & 0xF8) | (g >> 5);
    //Make sure the transfer is complete.
    while (!(SPSR & _BV(SPIF))) ;
    //Send the second half of the this pixel out
    SPDR = second_half;
    //Calculate the next first half while that is transmitting
    // ~1.9368us -0.1256 us = 1.8112uS
    second_half = ((g & 0xFC) << 3) | (b >> 3);

    //Done with this pixel
    pixel_count--;
    while (!(SPSR & _BV(SPIF))) ;
    }
  //Wait for the final transfer to complete before we bang on CS.
  
  // Deselect the LCD controller
  SET_CS;
  }
//----------------------------------------------------------------------------
void show_BMPs_in_root(void)
  {
  File
    root_dir;
  root_dir = SD.open("/");
  if(0 == root_dir)
    {
    Serial.println("show_BMPs_in_root: Can't open \"root\"");
    }
  File
    bmp_file;

  while(1)
    {
    bmp_file = root_dir.openNextFile();
    if (0 == bmp_file)
      {
      // no more files, break out of while()
      // root_dir will be closed below.
      break;
      }
    //Skip directories (what about volume name?)
    if(0 == bmp_file.isDirectory())
      {
      //The file name must include ".BMP"
      if(0 != strstr(bmp_file.name(),".BMP"))
        {
        //The BMP must be exactly 230456 long
        //(this is correct for 320x240, 24-bit)
        Serial.println(bmp_file.size());
        if(230456 == bmp_file.size())
          {
          //Jump over BMP header. BMP must be 320x240 24-bit
          bmp_file.seek(54);
          
    
          //Since we are limited in memory, break the line up from
          // 320*3 = 960 bytes into four chunks of 80 pixels
          // each 80*3 = 240 bytes.
          //Making this static speeds it up slightly (10ms)
          //Reduces flash size by 114 bytes, and uses 225 bytes.
          static uint8_t
            fourth_of_a_line[80*3];
          for(uint16_t line=0;line<240;line++)
            {
            //Set the LCD to the left of this line. BMPs store data
            //lowest line first -- bottom up.
            Set_LCD_for_write_at_X_Y(0,239-line);
            for(uint8_t line_section=0;line_section<4;line_section++)
              {
              //Get a fourth of the line
              bmp_file.read(fourth_of_a_line,80*3);
              //Now write this third to the TFT, doing the BGR -> RGB
              //color fixup interlaced with the SPI transfers.
              SPI_send_pixels(80,fourth_of_a_line);
              }
            }
           }
         }
       }
    if(0 != strstr(bmp_file.name(),"Z_POUNCE.BMP"))
      {
      delay(10000);
      }
    //Release the BMP file handle
    bmp_file.close();
    delay(2000);
    }
  //Release the root directory file handle
  root_dir.close();
  }
//============================================================================
#define FIND_MIN_MAX 0
#if(FIND_MIN_MAX)
  uint16_t Xmin=1023;
  uint16_t Xmax=0;
  uint16_t Ymin=1023;
  uint16_t Ymax=0;
#else
  //Copied from the serial console window
  uint16_t Xmin=105;
  uint16_t Xmax=928;
  uint16_t Ymin=110;
  uint16_t Ymax=860;
#endif
//----------------------------------------------------------------------------
uint8_t Read_Touch_Screen(uint16_t *x, uint16_t *y)
  {
  //See if there is a touch.
  //Let YU float, make YD tug high, drive X1 and X2 low.
  //Read Y1, if it is near 5v (~1024), then the screen is not
  //touched. If it is near ground (~50) then the screen is
  //touched.
  uint16_t
    touched;
  pinMode(TS_YU,INPUT);
  digitalWrite(TS_YU,HIGH);
  pinMode(TS_YD,INPUT_PULLUP);  
  digitalWrite(TS_YD,HIGH);
  pinMode(TS_XL,OUTPUT);
  digitalWrite(TS_XL,LOW);
  pinMode(TS_XR,OUTPUT);
  digitalWrite(TS_XR,LOW);
  touched = analogRead(TS_YU);

  //Idle YD as an input
  pinMode(TS_YD,INPUT);  
  if(touched < 512)
    {
    //We are touched.
    uint32_t
      X;
    uint32_t
      Y;
    //Read X. Set a gradient from 0v to 5v on X, then
    //read the Y line to get the X contact point.
    //pinMode(TS_YU,INPUT);    //Already set
    //pinMode(TS_YD,INPUT);    //Already set
    //pinMode(TS_XL,OUTPUT);   //Already set
    digitalWrite(TS_XR,HIGH);
    //pinMode(TS_XR,OUTPUT);   //Already set
    //digitalWrite(TS_XL,LOW); //Already set
    X = analogRead(TS_YD);     //Could use YU

    //Read Y. Set a gradient from 0v to 5v on Y, then
    //read the X line to get the Y contact point.
    pinMode(TS_XL,INPUT);
    pinMode(TS_XR,INPUT);
    pinMode(TS_YU,OUTPUT);
    digitalWrite(TS_YU,HIGH);
    pinMode(TS_YD,OUTPUT);
    digitalWrite(TS_YD,LOW);
    Y = analogRead(TS_XL);     //Could use XR
    
    Serial.print("X: ");
    Serial.println(X);
    Serial.print("Y: ");
    Serial.println(Y);
    //Idle the Y pins
    pinMode(TS_YU,INPUT);
    pinMode(TS_YD,INPUT);

    //Calculate the pixel values, store in the user's pointers.
    *x=((X-(uint32_t)Xmin)*320)/((uint32_t)Xmax-(uint32_t)Xmin);
    *y=240-((Y-(uint32_t)Ymin)*240)/((uint32_t)Ymax-(uint32_t)Ymin);
   
    //Return touched flag.
    return(1);
    }
  else
    {    
    //Not touched. Idle the pins that were set to output
    //to detect the touch.
    pinMode(TS_XL,INPUT);
    pinMode(TS_XR,INPUT);
    return(0);
    }
  }  
//============================================================================
void setup()
{
  DDRB |= 0x2F;
  DDRD |= (PS0_PIN) | (PS1_PIN) | (PS2_PIN) | (PS3_PIN);

  // Initialize SPI. By default the clock is 4MHz.
  SPI.begin();

  //Bump the clock to 8MHz. Appears to be the maximum.
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //Drive the ports to a reasonable starting state.
  CLR_RESET;
  CLR_RS;
  SET_CS;
  CLR_MOSI;

  // Set the PS pins to the desired interface
  SET_PS0;
  SET_PS1;
  SET_PS2;
  SET_PS3;
  SET_BL;

  //debug console
  Serial.begin(9600);
  Serial.println("setup()");

  // Initialize SPI. By default the clock is 4MHz.
  SPI.begin();

  //Bump the clock to 8MHz. Appears to be the maximum.
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //Initialize the LCD controller
  Initialize_LCD();

  // For the Seeduino I am using, the default speed of SPI_HALF_SPEED
  // set in C:\Program Files (x86)\Arduino\libraries\SD\src\SD.cpp
  // results in a 4MHz clock.
  //
  // If you change this function call in SDClass::begin() of SD.cpp
  // from:
  //
  //  return card.init(SPI_HALF_SPEED, csPin) &&
  //         volume.init(card) &&
  //         root.openRoot(volume);
  //
  // to:
  //
  //  return card.init(SPI_FULL_SPEED, csPin) &&
  //         volume.init(card) &&
  //         root.openRoot(volume);
  //
  // That appears to make the SD library talk at 8MHz.
  //

  Serial.println("Initialized");
  if (!SD.begin(SD_CS))
  {
    Serial.println("Card failed to initialize, or not present");
    //Reset the SPI clock to fast. SD card library does not clean up well.
    //Bump the clock to 8MHz. Appears to be the maximum.
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  }
  else
  {
    Serial.println("Card initialized.");
  }

#if(FIND_MIN_MAX)
  //Cheesy touch screen calibration
  //Enable this loop to find limits of the touch screen.
  //Stroke with a stylus from center to outside in every
  //direction.
  //Record the values from the serial monitor into the code above.
  while (1)
  {
    uint16_t
      X;
    uint16_t
      Y;
    //Read X. Set a gradient from 0v to 5v on X, then
    //read the Y line to get the X contact point.
    pinMode(TS_YU, INPUT);
    pinMode(TS_YD, INPUT);
    pinMode(TS_XL, OUTPUT);
    digitalWrite(TS_XL, HIGH);
    pinMode(TS_XR, OUTPUT);
    digitalWrite(TS_XR, LOW);
    X = analogRead(TS_YU);

    //Read Y. Set a gradient from 0v to 5v on Y, then
    //read the X line to get the Y contact point.
    pinMode(TS_XL, INPUT);
    pinMode(TS_XR, INPUT);
    pinMode(TS_YU, OUTPUT);
    digitalWrite(TS_YU, HIGH);
    pinMode(TS_YD, OUTPUT);
    digitalWrite(TS_YD, LOW);
    Y = analogRead(TS_XL);

    if (X < Xmin)
      Xmin = X;
    if (Xmax < X)
      Xmax = X;
    if (Y < Ymin)
      Ymin = Y;
    if (Ymax < Y)
      Ymax = Y;
    //Display X and Y on Serial Monitor
    Serial.print("Xmin = ");
    Serial.print(Xmin);
    Serial.print(" X = ");
    Serial.print(X);
    Serial.print(" Xmax = ");
    Serial.print(Xmax);
    Serial.print("| Ymin = ");
    Serial.print(Ymin);
    Serial.print(" Y = ");
    Serial.print(Y);
    Serial.print(" Ymax = ");
    Serial.println(Ymax);
  }
#endif

}
//============================================================================
#define drawcircles   0
#define cheesylines   1
#define checkerboard  0
#define graycode      0
#define touchscreen   1
#define showBMPs      1


void loop()
  {
    Serial.println("Top of the Loop");
    // while(1)
    // {
    //   //Go to Sleep
    //   Serial.println("Sleep");
    //   SPI_sendCommand(0x10);    // set Sleep mode
    //   SPI_sendData(0x0000);//sleep OUT 
    //   delay(500);

    //   //Wake up
    //   Serial.println("Wake");
    //   SPI_sendCommand(0x10);    // set Sleep mode
    //   SPI_sendData(0x0000);//sleep OUT 
    //   delay(200);
    //   SPI_sendCommand(0x07);    // Display Control 
    //   SPI_sendData(0x0033);  // Display on
    //   delay(500);
    // }



#if drawcircles
  //Draw a cyan circle
  //LCD_Circle(160,  120,  119,0x00,0xFF,0xFF);
  //Draw a cyan circle
  LCD_Circle(160, 120, 119, 0x00, 0xFF, 0xFF);
  //Draw a white circle
  LCD_Circle(160, 120, 40, 0xFF, 0xFF, 0xFF);
  //Draw a green circle
  LCD_Circle(160 - 80, 120, 37, 0x00, 0xFF, 0x00);
  //Draw a red circle
  LCD_Circle(160 + 80, 120, 37, 0xFF, 0x00, 0x00);
  //Draw a purple circle
  LCD_Circle(160, 120 - 80, 32, 0xFF, 0x00, 0xFF);
  //Draw a orange circle
  LCD_Circle(160, 120 + 80, 28, 0xFF, 0xA5, 0x00);
  delay(1000);
#endif

      
#if touchscreen
    Fill_LCD(0xFF,0x00,0x00);
  
  



  //Enable this secton for simple touch screen demo
  while (1)
  {
    uint16_t
      x;
    uint16_t
      y;

    if (Read_Touch_Screen(&x, &y))
    {
      //touch in upper right corner gets a clear
      if ((300 < x) && (y < 20))
      {
        Fill_LCD(0x00, 0x00, 0xFF);
      }
      //touch in upper left corner exits
      if ((x < 20) && (y < 20))
      {
        break;
      }
      //Otherwise draw
      LCD_Circle(x, y, 5, 0xFF, 0x00, 0xFF);
    }
    delay(10);

#endif

#if graycode
  //Gray code fill demo
  Fill_LCD(0x00,0xFF,0xFF); delay(200);
  Fill_LCD(0x00,0xFF,0xBF); delay(200); //transition
  Fill_LCD(0x00,0xFF,0x7F); delay(200); //transition
  Fill_LCD(0x00,0xFF,0x3F); delay(200); //transition
  Fill_LCD(0x00,0xFF,0x00); delay(200);
  Fill_LCD(0x3F,0xFF,0x00); delay(200); //transition
  Fill_LCD(0x7F,0xFF,0x00); delay(200); //transition
  Fill_LCD(0xBF,0xFF,0x00); delay(200); //transition
  Fill_LCD(0xFF,0xFF,0x00); delay(200);
  Fill_LCD(0xFF,0xFF,0x3F); delay(200); //transition
  Fill_LCD(0xFF,0xFF,0x7F); delay(200); //transition
  Fill_LCD(0xFF,0xFF,0xBF); delay(200); //transition
  Fill_LCD(0xFF,0xFF,0xFF); delay(200);
  Fill_LCD(0xFF,0xBF,0xFF); delay(200); //transition
  Fill_LCD(0xFF,0x7F,0xFF); delay(200); //transition
  Fill_LCD(0xFF,0x3F,0xFF); delay(200); //transition
  Fill_LCD(0xFF,0x00,0xFF); delay(200);
  Fill_LCD(0xFF,0x00,0xBF); delay(200); //transition
  Fill_LCD(0xFF,0x00,0x7F); delay(200); //transition
  Fill_LCD(0xFF,0x00,0x3F); delay(200); //transition
  Fill_LCD(0xFF,0x00,0x00); delay(200);
  Fill_LCD(0xBF,0x00,0x00); delay(200); //transition
  Fill_LCD(0x7F,0x00,0x00); delay(200); //transition
  Fill_LCD(0x3F,0x00,0x00); delay(200); //transition
  Fill_LCD(0x00,0x00,0x00); delay(200);
  Fill_LCD(0x00,0x00,0x3F); delay(200); //transition
  Fill_LCD(0x00,0x00,0x7F); delay(200); //transition
  Fill_LCD(0x00,0x00,0xBF); delay(200); //transition
  Fill_LCD(0x00,0x00,0xFF); delay(200);
  delay(2500);
#endif

#if cheesylines
//Cheesy lines
  uint8_t
    i;
  uint16_t
    x;
  uint16_t
    sub_x;
  uint16_t
    y;
  uint16_t
    sub_y;
  uint8_t
    r;
  uint8_t
    g;
  uint8_t
    b;

  r = 0xff;
  g = 0x00;
  b = 0x80;
  for(x=0;x<320;x++)
    {
    LCD_Line(160,120,
             x,0,
             r++,g--,b+=2);
    }
  for(y=0;y<240;y++)
    {
    LCD_Line(160,120,
             319,y,
             r++,g+=4,b+=2);
    }
  for(x=319;0!=x;x--)
    {
    LCD_Line(160,120,
             x,239,
             r-=3,g-=2,b-=1);
    }
  for(y=239;0!=y;y--)
    {
    LCD_Line(160,120,
             0,y,
             r+-3,g--,b++);
    }
  delay(1000);

  Fill_LCD(0x00,0x00,0x00);
  for(i=2;i<120;i+=2)
    {
    LCD_Circle(i+2+40, 120, i,i<<2,0xff-(i<<2),0xFF);
    }
  delay(1000);
#endif

#if checkerboard
  //Write a 16x16 checkerboard
  for(x=0;x<(320/16);x++)
    {
    for(y=0;y<(240/16);y++)
      {
      for(sub_y=0;sub_y<=15;sub_y++)
        {
        if(((x&0x01)&&!(y&0x01)) || (!(x&0x01)&&(y&0x01)))
          {
          Fast_Horizontal_Line((x<<4)+sub_x, (y<<4)+sub_y, (x<<4)+sub_x+15,
                               0x00, 0x00, 0x00);
          }
        else
          {
          Fast_Horizontal_Line((x<<4)+sub_x, (y<<4)+sub_y, (x<<4)+sub_x+15,
                                0xFF,
                                0xFF-(x<<8)/20,
                                (y<<8)/15);
          }
        }
      }
    }  
  delay(1000);
#endif

  //Slideshow of bitmap files on uSD card.
#if showBMPs
  show_BMPs_in_root();
#endif
  } // void loop()
//============================================================================
