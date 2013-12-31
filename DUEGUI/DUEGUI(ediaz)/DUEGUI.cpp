/*
  UTFT.cpp - Arduino/chipKit library support for Color TFT LCD Boards
  Copyright (C)2010-2012 Henning Karlsen. All right reserved
  
  This library is the continuation of my ITDB02_Graph, ITDB02_Graph16
  and RGB_GLCD libraries for Arduino and chipKit. As the number of 
  supported display modules and controllers started to increase I felt 
  it was time to make a single, universal library as it will be much 
  easier to maintain in the future.

  Basic functionality of this library was origianlly based on the 
  demo-code provided by ITead studio (for the ITDB02 modules) and 
  NKC Electronics (for the RGB GLCD module/shield).
  This library supports a number of 8bit, 16bit and seribyte DUEGUI::oriental graphic 
  displays, and will work with both Arduino and chipKit boards. For a 
  full list of tested display modules and controllers, see the 
  document UTFT_Supported_display_modules_&_controllers.pdf.

  When using 8bit and 16bit display modules there are some 
  requirements you must adhere to. These requirements can be found 
  in the document UTFT_Requirements.pdf.
  There are no special requirements when using serial displays.

  You can always find the latest version of the library at 
  http://electronics.henningkarlsen.com/

  If you make any modifications or improvements to the code, I would 
  appreciate that you share the code with me so that I might include 
  it in the next release. I can be contacted through 
  http://electronics.henningkarlsen.com/contact.php.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#define USE_ARDUINO_SPI_LIBRARY 0
#define  USE_NATIVE_SAM3X_SPI 1



#include "DUEGUI.h"

#include <pins_arduino.h>
#if defined(__AVR__)
	#include <avr/pgmspace.h>
	#include "HW_AVR.h" 
#elif defined(__SAM3X8E__)
	#include "Arduino.h"
        //#include "HW_SAM.h" 
#else
	#include "HW_PIC32.h"
#endif


char  keymap[]={'Q','W','E','R','T','Y','U','I','O','P','7','8','9'
               ,'A','S','D','F','G','H','J','K','L',' ','4','5','6'
               ,'Z','X','C','V','B','N','M',',',' ',' ','1','2','3'
               ,' ',' ',' ',' ',' ',' ',' ','/','*','-','+','.','0'
               
               ,'q','w','e','r','t','y','u','i','o','p','7','8','9'
               ,'a','s','d','f','g','h','j','k','l',' ','4','5','6'
               ,'z','x','c','v','b','n','m','?',' ',' ','1','2','3'
               ,' ',' ',' ',' ',' ',' ',' ',':',';',' ','+','.','0'
               
               ,'!','@','£','$','%','^','&','*',(char)254,(char)255,'7','8','9'
               ,'{','}','[',']','(',')','#',(char)252,(char)253,' ','4','5','6'
               ,(char)251,(char)250,(char)249,(char)248,(char)247,(char)246,(char)245,(char)244,' ',' ','1','2','3'
               ,' ',' ',' ',' ',' ',' ',' ',(char)243,(char)241,(char)240,'+','.','0'
};


DUEGUI::DUEGUI()
{
}

DUEGUI::DUEGUI(UTFT * lcd, UTouch * touch) {
  mylcd = (UTFT *) lcd;
  mytouch = (UTouch *) touch;
}


void DUEGUI::SPI_Flash_init(int CS_pin, int Rate)
{
	pinMode(CS_pin,OUTPUT);
	digitalWrite(CS_pin,HIGH);
	CSpin=CS_pin;
	spiBegin();
	spiInit(Rate);
}

void  DUEGUI::READ_ID()
{
				digitalWrite(CSpin,LOW);
                spiSend(0x90);
				spiSend(0x00);
				spiSend(0x00);
				spiSend(0x00);
				delayMicroseconds(10);
                //Serial1.write(spiRec());
                //Serial1.write(spiRec());
                digitalWrite(CSpin,HIGH);
}

	unsigned char font_height;
	unsigned int font_size;
	unsigned long font_address;
	unsigned int PositionX, PositionY;
	unsigned char Charspace=1;

void DUEGUI::Set_character_spacing(unsigned char space)
{
	Charspace=space;
}

void DUEGUI::Send_Flash_information_to_UART()
{
						digitalWrite(CSpin,LOW);
 						spiSend(0x03);
 						spiSend(0);
 						spiSend(0);
 						spiSend(0);
						for (int i=0; i<1000; i++)
						 {
						 Serial1.write(spiRec());
						 }
  			 			digitalWrite(CSpin,HIGH);

}

void DUEGUI::Put_Text(String st, int x, int y, int font_number)
{
  mylcd->print(st,x,y,0);
}


void DUEGUI::show_color_bar()
{

}

void DUEGUI::Convertto8bit(char VH, char VL)
{

}

void  DUEGUI::Load_image(int X, int Y, int location)
{ 

}
			

void DUEGUI::InitLCD(byte orientation)
{
  mylcd->InitLCD(orientation);
}

void DUEGUI::drawRect(int x1, int y1, int x2, int y2)
{
  mylcd->drawRect(x1, y1, x2, y2);
}

void DUEGUI::drawRoundRect(int x1, int y1, int x2, int y2)
{
  mylcd->drawRoundRect(x1, y1, x2, y2);
}

void DUEGUI::fillRect(int x1, int y1, int x2, int y2)
{
  mylcd->fillRect(x1, y1, x2, y2);
}

void DUEGUI::fillRoundRect(int x1, int y1, int x2, int y2)
{
  mylcd->fillRoundRect(x1, y1, x2, y2);
}

void DUEGUI::drawCircle(int x, int y, int radius)
{
  mylcd->drawCircle(x, y, radius);
}

void DUEGUI::fillCircle(int x, int y, int radius)
{
  mylcd->fillCircle(x, y, radius);
}

void DUEGUI::clrScr()
{
  mylcd->clrScr();
}

void DUEGUI::fillScr(byte r, byte g, byte b)
{
  mylcd->fillScr(r, g, b);
}

void DUEGUI::setColor(byte r, byte g, byte b)
{
  mylcd->setColor(r, g, b);
}

void DUEGUI::setBackColor(byte r, byte g, byte b)
{
  mylcd->setBackColor(r, g, b);
}

void DUEGUI::drawPixel(int x, int y)
{
  mylcd->drawPixel(x, y);
}

void DUEGUI::drawLine(int x1, int y1, int x2, int y2)
{
  mylcd->drawLine(x1, y1, x2, y2);
}

void DUEGUI::print(char *st, int x, int y, int deg)
{
  mylcd->print(st, x, y, deg);
}

void DUEGUI::print(String st, int x, int y, int deg)
{
  mylcd->print(st, x, y, deg);
}

void DUEGUI::printNumI(long num, int x, int y, int length, char filler)
{
  mylcd->printNumI(num, x, y, length, filler);
}

void DUEGUI::printNumF(double num, byte dec, int x, int y, char divider, int length, char filler)
{
  mylcd->printNumF(num, dec, x, y, divider, length, filler);
}

void DUEGUI::setFont(uint8_t* font)
{
  mylcd->setFont(font);
}

void DUEGUI::drawBitmap(int x, int y, int sx, int sy, bitmapdatatype data, int scale)
{
  mylcd->drawBitmap(x, y, sx, sy, data, scale);
}

void DUEGUI::drawBitmap(int x, int y, int sx, int sy, bitmapdatatype data, int deg, int rox, int roy)
{
  mylcd->drawBitmap(x, y, sx, sy, data, deg, rox, roy);
}

void DUEGUI::lcdOff()
{
  mylcd->lcdOff();
}

void DUEGUI::lcdOn()
{
  mylcd->lcdOn();
}

void DUEGUI::setContrast(char c)
{
  mylcd->setContrast(c);
}

int DUEGUI::getDisplayXSize()
{
  mylcd->getDisplayXSize();
}

int DUEGUI::getDisplayYSize()
{
  mylcd->getDisplayYSize();
}

// SPI functions
//==============================================================================
#if USE_ARDUINO_SPI_LIBRARY
#include <SPI.h>
//------------------------------------------------------------------------------
static void spiBegin() {
  SPI.begin();
}
//------------------------------------------------------------------------------
static void spiInit(uint8_t spiRate) {
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  int v;
#ifdef SPI_CLOCK_DIV128
  switch (spiRate/2) {
    case 0: v = SPI_CLOCK_DIV2; break;
    case 1: v = SPI_CLOCK_DIV4; break;
    case 2: v = SPI_CLOCK_DIV8; break;
    case 3: v = SPI_CLOCK_DIV16; break;
    case 4: v = SPI_CLOCK_DIV32; break;
    case 5: v = SPI_CLOCK_DIV64; break;
    default: v = SPI_CLOCK_DIV128; break;
  }
#else  // SPI_CLOCK_DIV128
  if (spiRate > 13) {
    v = 255;
  } else {
    v = (2 | (spiRate & 1)) << (spiRate/2);
  }
#endif  // SPI_CLOCK_DIV128
  SPI.setClockDivider(spiRate);  // thd
}
//------------------------------------------------------------------------------
/** SPI receive a byte */
static  uint8_t spiRec() {
  return SPI.transfer(0XFF);
}
//------------------------------------------------------------------------------
/** SPI receive multiple bytes */
static uint8_t spiRec(uint8_t* buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0XFF);
  }
  return 0;
}
//------------------------------------------------------------------------------
/** SPI send a byte */
static void spiSend(uint8_t b) {
  SPI.transfer(b);
}
//------------------------------------------------------------------------------
/** SPI send multiple bytes */
static void spiSend(const uint8_t* buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    SPI.transfer(buf[i]);
  }
}
//==============================================================================
#elif  USE_NATIVE_SAM3X_SPI
/** Use SAM3X DMAC if nonzero */
#define USE_SAM3X_DMAC 1
/** Use extra Bus Matrix arbitration fix if nonzero */
#define USE_SAM3X_BUS_MATRIX_FIX 0
/** Time in ms for DMA receive timeout */
#define SAM3X_DMA_TIMEOUT 100
/** chip select register number */
#define SPI_CHIP_SEL 3
/** DMAC receive channel */
#define SPI_DMAC_RX_CH  1
/** DMAC transmit channel */
#define SPI_DMAC_TX_CH  0
/** DMAC Channel HW Interface Number for SPI TX. */
#define SPI_TX_IDX  1
/** DMAC Channel HW Interface Number for SPI RX. */
#define SPI_RX_IDX  2
//------------------------------------------------------------------------------
/** Disable DMA Controller. */
static void dmac_disable() {
  DMAC->DMAC_EN &= (~DMAC_EN_ENABLE);
}
/** Enable DMA Controller. */
static void dmac_enable() {
  DMAC->DMAC_EN = DMAC_EN_ENABLE;
}
/** Disable DMA Channel. */
static void dmac_channel_disable(uint32_t ul_num) {
  DMAC->DMAC_CHDR = DMAC_CHDR_DIS0 << ul_num;
}
/** Enable DMA Channel. */
static void dmac_channel_enable(uint32_t ul_num) {
  DMAC->DMAC_CHER = DMAC_CHER_ENA0 << ul_num;
}
/** Poll for transfer complete. */
static bool dmac_channel_transfer_done(uint32_t ul_num) {
  return (DMAC->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul_num)) ? false : true;
}
//------------------------------------------------------------------------------
static void spiBegin() {
  PIO_Configure(
      g_APinDescription[PIN_SPI_MOSI].pPort,
      g_APinDescription[PIN_SPI_MOSI].ulPinType,
      g_APinDescription[PIN_SPI_MOSI].ulPin,
      g_APinDescription[PIN_SPI_MOSI].ulPinConfiguration);
  PIO_Configure(
      g_APinDescription[PIN_SPI_MISO].pPort,
      g_APinDescription[PIN_SPI_MISO].ulPinType,
      g_APinDescription[PIN_SPI_MISO].ulPin,
      g_APinDescription[PIN_SPI_MISO].ulPinConfiguration);
  PIO_Configure(
      g_APinDescription[PIN_SPI_SCK].pPort,
      g_APinDescription[PIN_SPI_SCK].ulPinType,
      g_APinDescription[PIN_SPI_SCK].ulPin,
      g_APinDescription[PIN_SPI_SCK].ulPinConfiguration);
  pmc_enable_periph_clk(ID_SPI0);
#if USE_SAM3X_DMAC
  pmc_enable_periph_clk(ID_DMAC);
  dmac_disable();
  DMAC->DMAC_GCFG = DMAC_GCFG_ARB_CFG_FIXED;
  dmac_enable();
#if USE_SAM3X_BUS_MATRIX_FIX
  MATRIX->MATRIX_WPMR = 0x4d415400;
  MATRIX->MATRIX_MCFG[1] = 1;
  MATRIX->MATRIX_MCFG[2] = 1;
  MATRIX->MATRIX_SCFG[0] = 0x01000010;
  MATRIX->MATRIX_SCFG[1] = 0x01000010;
  MATRIX->MATRIX_SCFG[7] = 0x01000010;
#endif  // USE_SAM3X_BUS_MATRIX_FIX
#endif  // USE_SAM3X_DMAC
}
//------------------------------------------------------------------------------
// start RX DMA
void spiDmaRX(uint8_t* dst, uint16_t count) {
  dmac_channel_disable(SPI_DMAC_RX_CH);
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_SADDR = (uint32_t)&SPI0->SPI_RDR;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_DADDR = (uint32_t)dst;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_DSCR =  0;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CTRLA = count |
    DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CTRLB = DMAC_CTRLB_SRC_DSCR |
    DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_PER2MEM_DMA_FC |
    DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_INCREMENTING;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CFG = DMAC_CFG_SRC_PER(SPI_RX_IDX) |
    DMAC_CFG_SRC_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG;
  dmac_channel_enable(SPI_DMAC_RX_CH);
}
//------------------------------------------------------------------------------
// start TX DMA
void spiDmaTX(const uint8_t* src, uint16_t count) {
  static uint8_t ff = 0XFF;
  uint32_t src_incr = DMAC_CTRLB_SRC_INCR_INCREMENTING;
  if (!src) {
    src = &ff;
    src_incr = DMAC_CTRLB_SRC_INCR_FIXED;
  }
  dmac_channel_disable(SPI_DMAC_TX_CH);
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_SADDR = (uint32_t)src;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DADDR = (uint32_t)&SPI0->SPI_TDR;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DSCR =  0;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLA = count |
    DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;

  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLB =  DMAC_CTRLB_SRC_DSCR |
    DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_MEM2PER_DMA_FC |
    src_incr | DMAC_CTRLB_DST_INCR_FIXED;

  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CFG = DMAC_CFG_DST_PER(SPI_TX_IDX) |
      DMAC_CFG_DST_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ALAP_CFG;

  dmac_channel_enable(SPI_DMAC_TX_CH);
}
//------------------------------------------------------------------------------
//  initialize SPI controller
static void spiInit(uint8_t spiRate) {
  Spi* pSpi = SPI0;
  uint8_t scbr = 255;
  if (spiRate < 14) {
    scbr = (2 | (spiRate & 1)) << (spiRate/2);
  }
  scbr = spiRate;  //thd
  //  disable SPI
  pSpi->SPI_CR = SPI_CR_SPIDIS;
  // reset SPI
  pSpi->SPI_CR = SPI_CR_SWRST;
  // no mode fault detection, set master mode
  pSpi->SPI_MR = SPI_PCS(SPI_CHIP_SEL) | SPI_MR_MODFDIS | SPI_MR_MSTR;
  // mode 0, 8-bit,
  pSpi->SPI_CSR[SPI_CHIP_SEL] = SPI_CSR_SCBR(scbr) | SPI_CSR_NCPHA;
  // enable SPI
  pSpi->SPI_CR |= SPI_CR_SPIEN;
}
//------------------------------------------------------------------------------
static inline uint8_t spiTransfer(uint8_t b) {
  Spi* pSpi = SPI0;

  pSpi->SPI_TDR = b;
  while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0) {}
  b = pSpi->SPI_RDR;
  return b;
}
//------------------------------------------------------------------------------
/** SPI receive a byte */
static inline uint8_t spiRec() {
  return spiTransfer(0XFF);
}
//------------------------------------------------------------------------------
/** SPI receive multiple bytes */
static uint8_t spiRec(uint8_t* buf, size_t len) {
  Spi* pSpi = SPI0;
  int rtn = 0;
#if USE_SAM3X_DMAC
  // clear overrun error
  uint32_t s = pSpi->SPI_SR;

  spiDmaRX(buf, len);
  spiDmaTX(0, len);

  uint32_t m = millis();
  while (!dmac_channel_transfer_done(SPI_DMAC_RX_CH)) {
    if ((millis() - m) > SAM3X_DMA_TIMEOUT)  {
      dmac_channel_disable(SPI_DMAC_RX_CH);
      dmac_channel_disable(SPI_DMAC_TX_CH);
      rtn = 2;
      break;
    }
  }
  if (pSpi->SPI_SR & SPI_SR_OVRES) rtn |= 1;
#else  // USE_SAM3X_DMAC
  for (size_t i = 0; i < len; i++) {
    pSpi->SPI_TDR = 0XFF;
    while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0) {}
    buf[i] = pSpi->SPI_RDR;
  }
#endif  // USE_SAM3X_DMAC
  return rtn;
}
//------------------------------------------------------------------------------
/** SPI send a byte */
static inline void spiSend(uint8_t b) {
  spiTransfer(b);
}
//------------------------------------------------------------------------------
static void spiSend(const uint8_t* buf, size_t len) {
  Spi* pSpi = SPI0;
#if USE_SAM3X_DMAC
  spiDmaTX(buf, len);
  while (!dmac_channel_transfer_done(SPI_DMAC_TX_CH)) {}
#else  // #if USE_SAM3X_DMAC
  while ((pSpi->SPI_SR & SPI_SR_TXEMPTY) == 0) {}
  for (size_t i = 0; i < len; i++) {
    pSpi->SPI_TDR = buf[i];
    while ((pSpi->SPI_SR & SPI_SR_TDRE) == 0) {}
  }
#endif  // #if USE_SAM3X_DMAC
  while ((pSpi->SPI_SR & SPI_SR_TXEMPTY) == 0) {}
  // leave RDR empty
  uint8_t b = pSpi->SPI_RDR;
}
#endif

void DUEGUI::InitTouch(byte orientation)
{
        mytouch->InitTouch(orientation);

	orient			        = orientation;
	_default_orientation	= CAL_S>>31;
	
//	touch_x_left			= CAL_X_MIN;
//	touch_x_right			= CAL_X_MAX;
//	touch_y_bottom			= CAL_Y_MAX;
//	touch_y_top 			= CAL_Y_MIN;

	touch_x_left			= 130;
	touch_x_right			= 3900;
	touch_y_bottom			= 3760;
	touch_y_top 			= 250;
	
	//touch_x_left			= 130;
	//touch_x_right			= 4000;
	//touch_y_bottom			= 4000;
	//touch_y_top 			= 250;
	
//	touch_disp_x_size		= CAL_X_SIZE;
//	touch_disp_y_size		= CAL_Y_SIZE;

	touch_disp_x_size		= 400;
	touch_disp_y_size		= 240;
	
	prec				= 25;
	
	_revTouchX = false;
	_revTouchY = false;

}

void DUEGUI::read(){

  int flag,x, y;

  mytouch->read();

  flag = mytouch->dataAvailable();

  if (flag) {
    TP_X = mytouch->TP_X;
    TP_Y = mytouch->TP_Y;
  } else {
    TP_X=-1;
    TP_Y=-1;
  }
 
}


bool DUEGUI::dataAvailable(){
  bool avail;

  avail = mytouch->dataAvailable();

  return avail;
}


int DUEGUI::getX(){

  long c;

  c = (3800-TP_X)/9.5;     // 400 

  return c;

}

int DUEGUI::getY(){

  long c;

  c = (TP_Y-0)/15.8;       // 240

  return c;

}

void DUEGUI::setPrecision(byte precision){
  switch (precision){
  case PREC_LOW:
	prec=1;
  break;
  case PREC_MEDIUM:
	prec=10;
  break;
  case PREC_HI:
	prec=25;
  break;
  case PREC_EXTREME:
	prec=100;
  break;
  default:
	prec=10;
  break;
  }

  mytouch->setPrecision(precision);
}

void DUEGUI::setReverseX(boolean reversed){
  if (reversed){
	_revTouchX=true;
  } else {
	_revTouchX=false;
  } 
}

void DUEGUI::setReverseY(boolean reversed){
  if (reversed){
	_revTouchY=true;
  } else {
	_revTouchY=false;
  } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            GENERAL FUNCTIONS USED BY GUI
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern uint8_t BigFont[];


void DUEGUI::InitGUI(byte tclk, byte tcs, byte tdin, byte dout, byte irq,int CS_pin, int Rate,int SD_pin){
  InitLCD();
  SPI_Flash_init(CS_pin,Rate);
  InitTouch();
  //
  // Variables for the GUI
  //
  anyClockVisible=false;
  anyButtonPressed=false;
  GUI_firstObject=0;
  GUI_objects=0;
  clrScr();
  startTimer(TC1,0,TC3_IRQn,ticksPerSecond);
  setFont(BigFont);
  //GUI_xoffset=0;
  //GUI_yoffset=0;
  GUI_xsize=getDisplayXSize();
  GUI_ysize=getDisplayYSize();
  GUIpopup_bw=GUI_xsize/13;     // ediaz - define popup button width function of screen size
  popup_ys=GUIpopup_bw*4;       // ediaz - dynamic popup_ys based on screen height (text area not included)
  popup_xs=0;                   // ediaz - also dynamic, but currently not used.
  GUIpopupbuttonpressed=-1;
  GUI_popupactive=false;
  GUI_popupobject=0;
  GUI_ShowCalibrate=false;
  _debugging=false;
}

void DUEGUI::preserveColours(){
  GUI_br=bcolorr;
  GUI_bg=bcolorg;
  GUI_bb=bcolorb;
  GUI_fr=fcolorr;
  GUI_fg=fcolorg;
  GUI_Fb=fcolorb;
}

void DUEGUI::restoreColours(){
  setBackColor(GUI_br,GUI_bg,GUI_bb);
  setColor(GUI_fr,GUI_fg,GUI_Fb);
}

void DUEGUI::setColorLong(long newColour){
  setColor((newColour&0xFF0000)/0x10000,(newColour&0xFFFF)/0x100,(newColour&0xFF));
}


void DUEGUI::setBackColorLong(long newColour){
  setBackColor((newColour&0xFF0000)/0x10000,(newColour&0xFFFF)/0x100,(newColour&0xFF));
}

void DUEGUI::updatingScreen(){
//  stopTimer(TC3_IRQn);
  preserveColours();
}

void DUEGUI::finishedUpdatingScreen(){
  restoreColours();
//  restartTimer(TC3_IRQn);
}

void DUEGUI::startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc/2); //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

void DUEGUI::stopTimer(IRQn_Type irq) {
  NVIC_DisableIRQ(irq);
}

void DUEGUI::restartTimer(IRQn_Type irq) {
  NVIC_EnableIRQ(irq);
}

String DUEGUI::truefalse(bool booleanValue) {
  if (booleanValue==true){
    return "true";
  } else {
    return "false";
  }
}


bool DUEGUI::withinBounds(int chkX,int chkY,int X, int Y, int X2, int Y2){
  if ( (chkX>X) && (chkY>Y) && (chkX<X2) && (chkY<Y2) ){ 
    return true; 
  } else {
    return false;
  }
}

String DUEGUI::displayNumFormat(int Num,int len){
  String numeric="0000000000"+String(Num,10);
  return numeric.substring(numeric.length()-len);
}

String DUEGUI::IntegerToString(long num, int length, char filler){
  char buf[25];
  char st[27];
  boolean neg=false;
  int c=0, f=0;
  String numeric="";
  if (length==-1){
    String numeric=String(num,10);
    length=numeric.length();	
  }
  if (num==0){
	if (length!=0){
      for (c=0; c<(length-1); c++)	st[c]=filler;
	  st[c]=48;
	  st[c+1]=0;
    } else {
	  st[0]=48;
      st[1]=0;
	}
  } else {
	if (num<0){
	  neg=true;
	  num=-num;
	}
	while (num>0){
	  buf[c]=48+(num % 10);
	  c++;
	  num=(num-(num % 10))/10;
	}
	buf[c]=0;
	if (neg){
	  st[0]=45;
	}
	if (length>(c+neg)){
	  for (int i=0; i<(length-c-neg); i++){
		st[i+neg]=filler;
		f++;
	  }
	}
    for (int i=0; i<c; i++){
	  st[i+neg+f]=buf[c-i-1];
	}
	st[c+neg+f]=0;
  }
  return (String)st;
}


const int cossin[]={1000,999,999,998,998,997,995,992,991,989
                    ,986,982,978,977,972,968,965,959,954,948
                    ,940,938,929,925,918,908,905,894,886,880
                    ,872,858,851,845,838,825,814,802,788,783
		    ,774,763,745,731,720,706,693,681,654,645
                    ,632,621,615,597,580,564,545,534,525,513
                    ,487,474,463,447,423,418,395,379,369,346
                    ,330,316,298,281,261,250,233,212,204,188
                    ,166,143,129,107,90,71,60,50,20,10,0};


int DUEGUI::calculate_x(int angle,int radius){
  int xo;
  angle=(angle%360);
  if (angle<91){
    xo=(radius*cossin[90-angle])/1000;
  }
  if ((angle>90) && (angle<181)){
    xo=(radius*cossin[angle-90])/1000;
  }
  if ((angle>180) && (angle<271)){
    xo=-(radius*cossin[270-angle])/1000;
  }
  if (angle>270){
    xo=-(radius*cossin[angle-270])/1000;
  }
  return xo;
}

int DUEGUI::calculate_y(int angle,int radius){
  int yo;
  angle=(angle%360);
  if (angle<91){
    yo=-(radius*cossin[angle])/1000;
  }
  if ((angle>90) && (angle<181)){
    yo=(radius*cossin[180-angle])/1000;
  }
  if ((angle>180) && (angle<271)){
    yo=(radius*cossin[angle-180])/1000;
  }
  if (angle>270){
    yo=-(radius*cossin[360-angle])/1000;
  }
  return yo;
}

void DUEGUI::drawAngledLine(int x,int y,int angle,int radius){
  drawLine(max(x,getDisplayXSize()),max(y,getDisplayYSize()),max(x+calculate_x(angle,radius),getDisplayXSize()),max(y+calculate_y(angle,radius),getDisplayYSize()));
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            NUMBERPAD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*  
int DUEGUI::numberpad(word x,word y,long colour,long borcolour,long buttoncolour,long buttonborcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,boolean fullstop,int input,int URN){
  int currentGUIobjectnumber=GUI_objects;
  updatingScreen();
  setColorLong(colour);
  fillRoundRect_OFFSET(x,y,x+190,y+250);
  setColourLong(borcolour);
  drawRoundRect_OFFSET(x,y,x+190,y+250);
  for (byte i = 1; i < 10; i++) {
    addButton(popupbuttonstart+i,x+10+(((i-1) % 3))*60,y+10+(((9-i)/3))*60,50,50,buttoncolour,buttonborcolour,textcolour,presscolour,presstextcolour,2,"1",GUI_singlechar_center_x,GUI_singleline_center_y,GUI_buttonchar_font);
    GUIobject_top[popupbuttonstart+i] = String((char) (48 + i));
    drawButton(popupbuttonstart+i); 
  }
  addButton(popupbuttonstart,x+10,y+190,50,50,buttoncolour,buttonborcolour,textcolour,presscolour,presstextcolour,2,"0",GUI_singlechar_center_x,GUI_singleline_center_y,GUI_buttonchar_font);
  drawButton(popupbuttonstart); 
  if (fullstop){
    addButton(popupbuttonstart+11,x+70,y+190,50,50,buttoncolour,buttonborcolour,textcolour,presscolour,presstextcolour,2,".",GUI_singlechar_center_x,GUI_singleline_center_y,GUI_buttonchar_font);
    drawButton(popupbuttonstart+11); 
    addButton(popupbuttonstart+10,x+130,y+190,50,50,buttoncolour,buttonborcolour,textcolour,presscolour,presstextcolour,2,"En",9,GUI_singleline_center_y,GUI_buttonchar_font);
  } else {
    addButton(popupbuttonstart+10,x+70,y+190,110,50,buttoncolour,buttonborcolour,textcolour,presscolour,presstextcolour,2,"Enter",10,GUI_singleline_center_y,GUI_buttonchar_font);
  }
  drawButton(popupbuttonstart+10); 
  finishedUpdatingScreen();
  GUI_objects=+1;
  
  return currentGUIobjectnumber+1;
}*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            TextInput FUNCTION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// function add TextInput
//
//
// add variable		  	stored in			description
//
// word x				[x]						{ location of label x }
// word y				[y]						{ location of label y }
// word xs				[xs]					{ size of the panel x in pixels }
// word ys				[ys]					{ size of the panel y in pixels }
// long colour			[colour]				{ colour of background }
// long borcolour		[borcolour]				{ colour of the border }
// byte borwidth		[borwidth]				{ width of the border }
// long textcolour		[textcolour]			{ colour of text }
// byte inputBoxcolour	[data2]					{ colour of the input box background }
// long inputtextcolour	[data3]	         		{ colour of the input box text }
// int inputlength		[data4]					{ maximum input length }
// int popup_y			[data5]					{ y pos to scroll screen to in order to fit keyboard }
// byte options			[data6]					{ input options }
// String top			[top]					{ text of label }
// int xo				[xo]					{ location of text within the panel x }
// int yo				[yo]					{ location of text within the panel y }
// int font				[font]					{ font for text }
// bool visible   	    [visible]    			{ visibility flag }
// int URN				[data1]					{ unique reference number for this button }
// int options			[data6]					{ options }
//  					[GUIobject_Stringdata1]	{ current state of the TextInput }
// bool changed			[changed]				{ object has changed since drawn or not drawn at all }
//
int  DUEGUI::addTextInput(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long inputBoxcolour,long inputtextcolour,byte borwidth,String top,int popup_y,word xo,word yo,String initialstate,int inputlength,int options,int font,bool visible,int URN){
  addPanel(x,y,xs,ys,colour,borcolour,textcolour,borwidth,top,xo,yo,font,visible,URN);
  GUIobject_type[GUI_objects]=GUI_Type_TextInput;  // use definition of button but change type to correct value.
  GUIobject_data1[GUI_objects]=popup_y;
  GUIobject_data2[GUI_objects]=inputBoxcolour;
  GUIobject_data3[GUI_objects]=inputtextcolour;
  GUIobject_data4[GUI_objects]=inputlength;
  GUIobject_data5[GUI_objects]=popup_y;
  GUIobject_data6[GUI_objects]=options;
  GUIobject_Stringdata1[GUI_objects]=initialstate;
  return GUI_objects;
}

void DUEGUI::drawTextInputText(int objectNumber,long textColour){
  setColorLong(textColour);
  setBackColorLong(GUIobject_data2[objectNumber]);
  if ((GUIobject_data6[objectNumber]&1)==1){
    Put_Text(GUIobject_Stringdata1[objectNumber],GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_font[objectNumber]);
  } else {
    Put_Text(GUIobject_Stringdata1[objectNumber],GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-10-(16*GUIobject_data4[objectNumber]),GUIobject_y[objectNumber]+(GUIobject_ys[objectNumber]/2-15),GUIobject_font[objectNumber]);
  }
}

void DUEGUI::drawTextInput(int objectNumber){
  if (objectVisible(objectNumber)){
    updatingScreen();
    Serial.print(((GUIobject_data6[objectNumber]&1)==0));
    Serial.print(",");
    Serial.println(GUIobject_data6[objectNumber]);
    if ((GUIobject_data6[objectNumber]&1)==0){
      int bx2=GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-15;
      int bx=bx2-(16*GUIobject_data4[objectNumber]);
      int by=GUIobject_y[objectNumber]+(GUIobject_ys[objectNumber]/2-18);
      int by2=by+36;
      // Draw panel
      setColorLong(GUIobject_colour[objectNumber]);
      fillRoundRect (GUIobject_x[objectNumber],GUIobject_y[objectNumber], GUIobject_x[objectNumber]+GUIobject_xs[objectNumber],GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]);
      setColorLong(GUIobject_borcolour[objectNumber]);
      drawRoundRect (GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_x[objectNumber]+GUIobject_xs[objectNumber],GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]);
      // Draw inner input box
      setColorLong(GUIobject_data2[objectNumber]);
      fillRoundRect (bx,by,bx2 ,by2);
      setColorLong(GUIobject_borcolour[objectNumber]);
      drawRoundRect (bx,by,bx2 ,by2);
      // Add label
      setColorLong(GUIobject_textcolour[objectNumber]);
      setBackColorLong(GUIobject_colour[objectNumber]);
      Put_Text(GUIobject_top[objectNumber],GUIobject_x[objectNumber]+GUIobject_xo[objectNumber],GUIobject_y[objectNumber]+GUIobject_yo[objectNumber],GUIobject_font[objectNumber]);
    }
    // Draw input
    drawTextInputText(objectNumber,GUIobject_data3[objectNumber]);
    finishedUpdatingScreen(); 
  }
}

void DUEGUI::updateTextInput(int objectNumber,String inputText){
  if ((GUIobject_visible[objectNumber]) && (inputText!=GUIobject_Stringdata1[objectNumber])){
    updatingScreen();
    //drawTextInputText(objectNumber,GUIobject_data2[objectNumber]);
    GUIobject_Stringdata1[objectNumber]=inputText;
    //drawTextInputText(objectNumber,GUIobject_data3[objectNumber]);
    finishedUpdatingScreen();  
    GUIobject_changed[objectNumber]=false;
  } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            CYCLEBUTTON FUNCTION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// function add CYCLEBUTTON
//
//
// add variable		  	stored in			description
//
// word x				[x]					{ location of label x }
// word y				[y]					{ location of label y }
// word xs				[xs]				{ size of the panel x in pixels }
// word ys				[ys]				{ size of the panel y in pixels }
// long colour			[colour]			{ colour of background }
// long borcolour		[borcolour]			{ colour of the border }
// long textcolour		[textcolour]		{ colour of text }
// byte borwidth		[borwidth]			{ width of the border }
// long presscolour		[presscolour]		{ colour of button when pressed }
// long presstextcolour	[presstextcolour]	{ Colour of text in button when pressed }
// String top			[top]				{ text of label }
// int xo				[xo]				{ location of text within the panel x }
// int yo				[yo]				{ location of text within the panel y }
// int font				[font]				{ font for text }
// bool visible   	    [visible]    		{ visibility flag }
// int URN				[data1]				{ unique reference number for this button }
// int initialstate		[data3]				{ initial/current state of the cyclebox }
// int cyclemin			[data4]				{ min value of the cyclebox }
// int cyclemax			[data5]				{ max value of the cyclebox }
// int cyclestep		[data6]				{ max value of the cyclestep: -1 / 1 usually  }
// int cyclexo			[link]				{ x offset from right x point  }
// byte options         [linkoptions]		{ options }
//                                          { options [bit  1  ] = rocker style (up/down)            }
// bool changed			[changed]			{ object has changed since drawn or not drawn at all }
//
int  DUEGUI::addCycleButton(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,int initialstate,int cyclemin,int cyclemax,int cyclestep,int cyclexo,int options,bool visible,int URN){
  addButton(x,y,xs,ys,colour,borcolour,textcolour,presscolour,presstextcolour,borwidth,top,xo,yo,font,visible,URN);
  GUIobject_type[GUI_objects]=GUI_Type_cycleButton;  // use definition of button but change type to correct value.
  GUIobject_data3[GUI_objects]=initialstate;
  GUIobject_data4[GUI_objects]=cyclemin;
  GUIobject_data5[GUI_objects]=cyclemax;
  GUIobject_data6[GUI_objects]=cyclestep;
  GUIobject_linkoption[GUI_objects]=options;
  if (cyclexo==posAuto){
//    GUIobject_link[GUI_objects]=
  } else {
    GUIobject_link[GUI_objects]=cyclexo;
  }
  return GUI_objects;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                       CYCLETEXTBUTTON FUNCTION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// function add CYCLETEXTBUTTON
//
//
// add variable		  	stored in			description
//
// word x				[x]					{ location of label x }
// word y				[y]					{ location of label y }
// word xs				[xs]				{ size of the panel x in pixels }
// word ys				[ys]				{ size of the panel y in pixels }
// long colour			[colour]			{ colour of background }
// long borcolour		[borcolour]			{ colour of the border }
// long textcolour		[textcolour]		{ colour of text }
// byte borwidth		[borwidth]			{ width of the border }
// long presscolour		[presscolour]		{ colour of button when pressed }
// long presstextcolour	[presstextcolour]	{ Colour of text in button when pressed }
// String top			[top]				{ text of label }
// int xo				[xo]				{ location of text within the panel x }
// int yo				[yo]				{ location of text within the panel y }
// int font				[font]				{ font for text }
// bool visible   	    [visible]    		{ visibility flag }
// int URN				[URN]				{ unique reference number for this button }
// int initialstate		[data3]				{ initial/current state of the cyclebox }
// 						[data4]				{ number of entries (calculated) }
// int cyclexo			[link]				{ x offset from right x point  }
// String *elements		[StringPointer]		{ array containing the String elements }
// byte options         [linkoptions]		{ options }
//                                          { options [bit  1  ] = rocker style (up/down)            }
// bool changed			[changed]			{ object has changed since drawn or not drawn at all }
//
int DUEGUI::addCycleStringButton(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,int initialstate,String *elements,int cyclexo,int options,bool visible,int URN){
  addButton(x,y,xs,ys,colour,borcolour,textcolour,presscolour,presstextcolour,borwidth,top,xo,yo,font,visible,URN);
  GUIobject_type[GUI_objects]=GUI_Type_cycleStringbutton;  // use definition of button but change type to correct value.
  GUIobject_data3[GUI_objects]=initialstate;
  GUIobject_StringPointer[GUI_objects]=elements;
  GUIobject_linkoption[GUI_objects]=options;
  if (cyclexo==posAuto){
//    GUIobject_link[GUI_objects]=
  } else {
    GUIobject_link[GUI_objects]=cyclexo;
  } 
  int i=0;
  while (GUIobject_StringPointer[GUI_objects][i]!=""){
    i+=1;    
  } 
  GUIobject_data4[GUI_objects]=i-1; 
  return GUI_objects;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            CHECKBOX FUNCTION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// function add CHECKBOX
//
//
// add variable		  	stored in			description
//
// word x				[x]					{ location of label x }
// word y				[y]					{ location of label y }
// word xs				[xs]				{ size of the panel x in pixels }
// word ys				[ys]				{ size of the panel y in pixels }
// long colour			[colour]			{ colour of background }
// long borcolour		[borcolour]			{ colour of the border }
// long textcolour		[textcolour]		{ colour of text }
// byte borwidth		[borwidth]			{ width of the border }
// long presscolour		[presscolour]		{ colour of button when pressed }
// long presstextcolour	[presstextcolour]	{ Colour of text in button when pressed }
// String top			[top]				{ text of label }
// int xo				[xo]				{ location of text within the panel x }
// int yo				[yo]				{ location of text within the panel y }
// int font				[font]				{ font for text }
// bool visible   	    [visible]    		{ visibility flag }
// int URN				[urn]				{ unique reference number for this button }
// int initialstate		[data3]				{ initial & current state of the checkbox }
// byte options         [data4]				{ options [bits 1-3] = 1 - Y/N, 2 - YES/NO, 3 - checkbox 
// bool changed			[changed]			{ object has changed since drawn or not drawn at all }
//
int DUEGUI::addCheckBox(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,int initialstate,byte options,bool visible,int URN){
  addButton(x,y,xs,ys,colour,borcolour,textcolour,presscolour,presstextcolour,borwidth,top,xo,yo,font,visible,URN);
  GUIobject_type[GUI_objects]=GUI_Type_checkBox;  // use definition of button but change type to correct value.
//  GUIobject_data2[GUI_objects]=initialstate;
  GUIobject_data3[GUI_objects]=initialstate;
  GUIobject_data4[GUI_objects]=options;
  return GUI_objects;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            BUTTON FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// function add BUTTON
//
//
// variable			  	stored in	description
//
// word x				[x]					{ location of label x }
// word y				[y]					{ location of label y }
// word xs				[xs]				{ size of the panel x in pixels }
// word ys				[ys]				{ size of the panel y in pixels }
// long colour			[colour]			{ colour of background }
// long borcolour		[borcolour]			{ colour of the border }
// long textcolour		[textcolour]		{ colour of text }
// byte borwidth		[borwidth]			{ width of the border }
// long presscolour		[presscolour]		{ colour of button when pressed }
// long presstextcolour	[presstextcolour]	{ Colour of text in button when pressed }
// String top			[top]				{ text of label }
// int xo				[xo]				{ location of text within the panel x }
// int yo				[yo]				{ location of text within the panel y }
// int font				[font]				{ font for text }
// int URN				[data1]				{ unique reference number for this button }
// bool visible   	    [visible]    		{ visibility flag }
// bool changed			[changed]			{ object has changed since drawn or not drawn at all }
//
int DUEGUI::addButton(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,bool visible,int URN){
  //
  // "Button" is a child of "Panel" call panel then add the additional attributes.
  //
  addPanel(x,y,xs,ys,colour,borcolour,textcolour,borwidth,top,xo,yo,font,visible,URN);
  GUIobject_type[GUI_objects]=GUI_Type_button;
  GUIobject_pressedcolour[GUI_objects]=presscolour;
  GUIobject_pressedtextcolour[GUI_objects]=presstextcolour;
  GUIobject_currentstate[GUI_objects]=GUI_State_Unpressed;
  return GUI_objects;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                      IMAGEBUTTON FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// function add IMAGEBUTTON
//
//
// variable			  	stored in	description
//
// word x				[x]					{ location of label x }
// word y				[y]					{ location of label y }
// 						[xs]				{ size of the image x in pixels (calculated) }
// 						[ys]				{ size of the image y in pixels (calculated) }
// int image			[data1]				{ image number }
// int imagepressed		[data2]				{ alternative image to display when pressed (if required or 0) }
// int imagealt			[data5]				{ alternative image to display if this is an on-off-on style button or 0 }
// int size				[data4]				{ image size multiplier }
// int option			[data6]				{ options (none at the moment) }
// int URN				[data1]				{ unique reference number for this button }
// bool visible   	    [visible]    		{ visibility flag }
// bool changed			[changed]			{ object has changed since drawn or not drawn at all }
//
int DUEGUI::addImageButton(word x,word y,int image,int imagepressed,int imagealt,byte size,bool initialState,byte option,bool visible,int URN){
  GUI_objects+=1;
  GUIobject_type[GUI_objects]=GUI_Type_imagebutton;
  GUIobject_x[GUI_objects]=x;
  GUIobject_y[GUI_objects]=y;
  GUIobject_UniqueReference[GUI_objects]=URN;
  GUIobject_visible[GUI_objects]=visible;  
  GUIobject_changed[GUI_objects]=true;
  //
  // Find the size of the graphic and store is xs,ys multiplied by the size multiplier
  getImageSize(image);
  GUIobject_xs[GUI_objects] = iXsize*size;	
  GUIobject_ys[GUI_objects] = iYsize*size;	
  GUIobject_data1[GUI_objects]=image;
  GUIobject_data2[GUI_objects]=imagepressed;
  GUIobject_data3[GUI_objects]=initialState;  //this becomes the state to be checked
  GUIobject_data4[GUI_objects]=size;
  GUIobject_data5[GUI_objects]=max(imagealt,image);  //if 0 set alt image to normal image
  GUIobject_data6[GUI_objects]=option;
  GUIobject_currentstate[GUI_objects]=GUI_State_Unpressed;
  return GUI_objects;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                      GHOSTBUTTON FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// function add GHOSTBUTTON
//
//
// variable			  	stored in	description
//
// word x				[x]					{ location of label x }
// word y				[y]					{ location of label y }
// word xs				[xs]				{ size of the area x in pixels (calculated) }
// word ys				[ys]				{ size of the area y in pixels (calculated) }
// int option			[data6]				{ options (none at the moment) }
// int URN				[data1]				{ unique reference number for this button }
// bool visible   	    [visible]    		{ visibility flag }
// bool changed			[changed]			{ object has changed since drawn or not drawn at all }
//
int DUEGUI::addGhostButton(word x,word y,word xs,word ys,byte option,bool visible,int URN){
  //
  // "Ghost Button" is a child of "Panel" call panel then add the additional attributes.
  //
  addPanel(x,y,xs,ys,0,0,0,0,"",0,0,0,visible,URN);
  GUIobject_type[GUI_objects]=GUI_Type_ghostbutton;
  GUIobject_currentstate[GUI_objects]=GUI_State_Unpressed;
  GUIobject_data6[GUI_objects]=option;
  return GUI_objects;
}

void DUEGUI::drawCycleValue(int objectNumber,int colour,int backcolour){
  if (colour!=-1){
    setColorLong(colour);
    setBackColorLong(backcolour);
  }
  Put_Text((String)GUIobject_data3[objectNumber],GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-GUIobject_link[objectNumber],GUIobject_y[objectNumber]+GUIobject_yo[objectNumber],GUIobject_font[objectNumber]);
}

void DUEGUI::drawCycleText(int objectNumber,int colour,int backcolour){
  if (colour!=-1){
    setColorLong(colour);
    setBackColorLong(backcolour);
  }
  Put_Text(   GUIobject_StringPointer[objectNumber][GUIobject_data3[objectNumber]]   ,GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-GUIobject_link[objectNumber],GUIobject_y[objectNumber]+GUIobject_yo[objectNumber],GUIobject_font[objectNumber]);
}

void DUEGUI::drawButton(int objectNumber){
  if (objectVisible(objectNumber)){
    updatingScreen();
    //
    // Draw the outer filled button
    if (GUIobject_type[objectNumber]==GUI_Type_imagebutton){   
      if (GUIobject_currentstate[objectNumber]==GUI_State_Unpressed){
        if (!GUIobject_data3[objectNumber]){
          Load_image(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_data1[objectNumber]);
        } else {
          Load_image(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_data5[objectNumber]);
        }
      } else {
        Load_image(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_data2[objectNumber]);
      } 
    } else {
      if (GUIobject_currentstate[objectNumber]==GUI_State_Unpressed){
        setColorLong(GUIobject_colour[objectNumber]);
        fillRoundRect (GUIobject_x[objectNumber],GUIobject_y[objectNumber], GUIobject_x[objectNumber]+GUIobject_xs[objectNumber],GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]);
        setColorLong(GUIobject_borcolour[objectNumber]);
        drawRoundRect (GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_x[objectNumber]+GUIobject_xs[objectNumber],GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]);
        setColorLong(GUIobject_textcolour[objectNumber]);
        setBackColorLong(GUIobject_colour[objectNumber]);
      } else {
        setColorLong(GUIobject_pressedcolour[objectNumber]);
        fillRoundRect (GUIobject_x[objectNumber],GUIobject_y[objectNumber], GUIobject_x[objectNumber]+GUIobject_xs[objectNumber],GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]);
        setColorLong(GUIobject_borcolour[objectNumber]);
        drawRoundRect (GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_x[objectNumber]+GUIobject_xs[objectNumber],GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]);
        setColorLong(GUIobject_pressedtextcolour[objectNumber]);
        setBackColorLong(GUIobject_pressedcolour[objectNumber]);
      }
      //
      // Insert the text
      Put_Text(GUIobject_top[objectNumber],GUIobject_x[objectNumber]+GUIobject_xo[objectNumber],GUIobject_y[objectNumber]+GUIobject_yo[objectNumber],GUIobject_font[objectNumber]);
      //
      // Is it the Checkbox type?
      if (GUIobject_type[objectNumber]==GUI_Type_checkBox) {
        //
        // Is it option X/Y?
        if (GUIobject_data4[objectNumber]==cycYN){
          if (GUIobject_data3[objectNumber]!=1){
            Put_Text("X",GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-34,GUIobject_y[objectNumber]+GUIobject_yo[objectNumber],GUIobject_font[objectNumber]);
         } else {
           Put_Text("Y",GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-34,GUIobject_y[objectNumber]+GUIobject_yo[objectNumber],GUIobject_font[objectNumber]);
          }
        }
        //
        // Is it option NO/YES?
        if (GUIobject_data4[objectNumber]==cycYESNO){
          if (GUIobject_data3[objectNumber]!=1){
            Put_Text("  NO",GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-58,GUIobject_y[objectNumber]+GUIobject_yo[objectNumber],GUIobject_font[objectNumber]);
          } else {
            Put_Text("YES",GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-58,GUIobject_y[objectNumber]+GUIobject_yo[objectNumber],GUIobject_font[objectNumber]);  
          }
        }
        //
        // Is it option traditional checkbox?
        if (GUIobject_data4[objectNumber]==cycCHECKBOX){
          if (GUIobject_data3[objectNumber]!=1){
            Put_Text(" ",GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-33,GUIobject_y[objectNumber]+GUIobject_yo[objectNumber]-1,GUIobject_font[objectNumber]);
          } else {
            Put_Text("x",GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-33,GUIobject_y[objectNumber]+GUIobject_yo[objectNumber]-1,GUIobject_font[objectNumber]);
          }
          drawRect(GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-36,GUIobject_y[objectNumber]+GUIobject_yo[objectNumber]+5,GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]-36+18,GUIobject_y[objectNumber]+GUIobject_yo[objectNumber]+24);
        }
      }
      //
      // Is it the cycleButton
      if (GUIobject_type[objectNumber]==GUI_Type_cycleButton) {
        drawCycleValue(objectNumber,-1,-1);
      }
      // Is it the cycleTextButton
      if (GUIobject_type[objectNumber]==GUI_Type_cycleStringbutton) {
        drawCycleText(objectNumber,-1,-1);
      }
      finishedUpdatingScreen(); 
    }
  }
}

bool DUEGUI::checkButton(word objectNumber){
  if (GUIobject_visible[objectNumber]){
    read();
    bool inbounds = (withinBounds(getX(),getY(),GUIobject_x[objectNumber],GUIobject_y[objectNumber],(GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]),(GUIobject_y[objectNumber]+GUIobject_ys[objectNumber])));
    if (GUIobject_currentstate[objectNumber]==GUI_State_Unpressed){
    
      // Button currently shown as un pressed.
      // Now check current state.....
      if (inbounds){
//        Serial.println("no / yes");
        // Button is currently shown as NOT pressed BUT has been pressed
        //
        // Change state to pressed
        //
        // Now we need to check the alternative types of buttons
        //
        // First we check the single state buttons
        //
        if (GUIobject_type[objectNumber]==GUI_Type_TextInput) {
          //
          // This is an input box so display popup
          //
          makePopUp(objectNumber,GUIobject_data5[objectNumber]);
          anyButtonPressed=true;
        } else {
          if (GUIobject_type[objectNumber]==GUI_Type_ghostbutton) {
            //
            // This checking ghost button
            //
            anyButtonPressed=true;
          } else {
            //
            //  Now we check buttons that have dual states
            //
            //
            GUIobject_currentstate[objectNumber]=GUI_State_Pressed;
            //
            //   Make changes to button variables based on new state
            //
            //  CheckBox or image button
            // 
            if  ((GUIobject_type[objectNumber]==GUI_Type_checkBox)||(GUIobject_type[objectNumber]==GUI_Type_imagebutton)) {
              // This is a check box
              GUIobject_data3[objectNumber]=!GUIobject_data3[objectNumber];
            }
            if (GUIobject_type[objectNumber]==GUI_Type_cycleStringbutton){
              // remove the old value first
              drawCycleText(objectNumber,GUIobject_colour[objectNumber],GUIobject_colour[objectNumber]);
              //
              // Check if option bit set for rocker button AND first half of button pressed, if so
              // then reverse direction of increment.
              //
              int div_x=1;
              int div_y=1;
              bool rocker=false;
              if ((GUIobject_linkoption[objectNumber]&1)==1){
                div_x=2;
                div_y=1;
                rocker=true;
              } else {
                if ((GUIobject_linkoption[objectNumber]&2)==2){
                  div_x=1;
                  div_y=2;
                  rocker=true;
                }
              } 
              if ( (rocker) && ((withinBounds(getX(),getY(),GUIobject_x[objectNumber],GUIobject_y[objectNumber],(GUIobject_x[objectNumber]+(GUIobject_xs[objectNumber]/div_x)),(GUIobject_y[objectNumber]+(GUIobject_ys[objectNumber]/div_y)))))){
                if (GUIobject_data3[objectNumber]==0){
                  GUIobject_data3[objectNumber]=GUIobject_data4[objectNumber];
                } else {
                  GUIobject_data3[objectNumber]=GUIobject_data3[objectNumber]-1;
                }
              } else {       
                GUIobject_data3[objectNumber]=GUIobject_data3[objectNumber]+1;
              }
              if (GUIobject_data3[objectNumber]==GUIobject_data4[objectNumber]+1){
                GUIobject_data3[objectNumber]=0;
              }
              // set string to the value
              GUIobject_Stringdata1[objectNumber]=GUIobject_StringPointer[objectNumber][GUIobject_data3[objectNumber]];
           } else {
              //
              //  CycleButton 
              //
              if (GUIobject_type[objectNumber]==GUI_Type_cycleButton) {
                // remove the old value first
                drawCycleValue(objectNumber,GUIobject_colour[objectNumber],GUIobject_colour[objectNumber]);
                //
                // Check if option bit set for rocker button AND first half of button pressed, if so
                // then reverse direction of increment.
                //
                int div_x=1;
                int div_y=1;
                bool rocker=false;
                if ((GUIobject_linkoption[objectNumber]&1)==1){
                  div_x=2;
                  div_y=1;
                  rocker=true;
                } else {
                  if ((GUIobject_linkoption[objectNumber]&2)==2){
                    div_x=1;
                    div_y=2;
                    rocker=true;
                  }
                } 
                if ( (rocker) && ((withinBounds(getX(),getY(),GUIobject_x[objectNumber],GUIobject_y[objectNumber],(GUIobject_x[objectNumber]+(GUIobject_xs[objectNumber]/div_x)),(GUIobject_y[objectNumber]+(GUIobject_ys[objectNumber]/div_y)))))){
                  GUIobject_data3[objectNumber]=GUIobject_data3[objectNumber]-GUIobject_data6[objectNumber];
                } else {        
                  GUIobject_data3[objectNumber]=GUIobject_data3[objectNumber]+GUIobject_data6[objectNumber];
                }
                if (GUIobject_data3[objectNumber]<GUIobject_data4[objectNumber]){
                  GUIobject_data3[objectNumber]=GUIobject_data5[objectNumber];
                }
                if (GUIobject_data3[objectNumber]>GUIobject_data5[objectNumber]){
                  GUIobject_data3[objectNumber]=GUIobject_data4[objectNumber];
                }
              } else {      
                // Check button link options (not if cyclebutton though....)
                if (GUIobject_linkoption[objectNumber]=GUI_link_INC){
                  // button being pressed increases value of another object
                  //
                }
                if (GUIobject_linkoption[objectNumber]=GUI_link_DEC){
                  // button being pressed reduces value of another object
                  //
                }
              }    
            }   
            drawButton(objectNumber);
            anyButtonPressed=true;
          } 
        }
        return true;   
      } else {
//        Serial.println("no / no");
        // Button is currently show as NOT pressed AND has NOT been pressed
        //
        // No need to change anything on screen but still need to return false as it isn't pressed
        return false;   
      }
    } else {
  //    anyButtonPressed=false;
      // Button currently shown as pressed.
      // Now check current state.....
      if (inbounds){
//        Serial.println("yes / yes");
        // Button is currently shown as pressed and has been pressed
        //
        // No need to change anything on screen but still need to return true as it was pressed
        return true;   
      } else {
//        Serial.println("yes / no");
        // Button is currently show as pressed AND has NOT been pressed
        //
        // Change state to unpressed
        GUIobject_currentstate[objectNumber]=GUI_State_Unpressed;
        // Redraw button
        drawButton(objectNumber);        
        anyButtonPressed=false;
        return false;   
      }
    }
  }
}


int DUEGUI::checkAllButtons(){
  int found=-1;
  // First check if popup keyboard is active and if so deal with that

  if (GUI_popupactive){
    // popup keyboard is active.  Check if keyboard pressed:
    //
    read();
    int xpos=getX();
    int ypos=getY();

    if (ypos>0){

      String newvalue=GUIobject_Stringdata1[GUI_popupobject];
      
      //Put_Text(newvalue,0,GUI_ysize-GUIpopup_bw,BVS_28);                      //ediaz 

      if (ypos<GUI_ysize){
        // cancel popup
        clearPopUp();
      } else {
        int keynumber=-1;
        if (withinBounds(xpos,ypos,0,GUI_ysize,getDisplayXSize(),getDisplayYSize())){
          // Pressed within the popup keyboard.
          //
          byte i=(xpos)/GUIpopup_bw;
          byte k=(ypos-GUI_ysize)/GUIpopup_bw;
          keynumber=i+(k*13);

          if (GUIpopupbuttonpressed!=-1){
            anyButtonPressed=false;
            drawSingleKey(GUIpopupbuttonpressed+GUI_popupkeyoffset,clrWhite,clrBlack);
            GUIpopupbuttonpressed=-1;
          }
          anyButtonPressed=true;
          if ((keynumber==43)||(keynumber==44)){keynumber=42;}
          if (keynumber==35){keynumber=34;}
          if ((keynumber==45)||(keynumber==41)){
            keynumber=-1;
          } else {
            keynumber+=GUI_popupkeyoffset;
            if (((keynumber%52)!=39)&&((keynumber%52)!=40)){
              drawSingleKey(keynumber+GUI_popupkeyoffset,clrRed,clrWhite);
            }
            GUIpopupbuttonpressed=keynumber;
          
            // Key is pressed on popup
            
            switch (keynumber) {
            
            case 22:      // backspace keys
            case 22+52:
               if (GUIobject_Stringdata1[GUI_popupobject]!=""){
                 newvalue=GUIobject_Stringdata1[GUI_popupobject].substring(0,GUIobject_Stringdata1[GUI_popupobject].length()-1);
                 setColor(255,255,255);
                 fillRect(0,GUI_ysize-GUIpopup_bw,getDisplayXSize()-1,GUI_ysize);
               }
               break; 
               
            case 34:                                       // enter keys
            case 34+52:
              GUI_popupclearedwith=13;
              clearPopUp();
              break;
              
            case 39:      // sym lock on
            case 39+52:      
              GUI_popupkeyoffset=104;
              redrawPopUpButtons(GUI_popupobject);
              break;
              
            case 39+2*52:      // sym lock on
              GUI_popupkeyoffset=0;
              redrawPopUpButtons(GUI_popupobject);
              break;
              
            case 40:      // shift lock on
              GUI_popupkeyoffset=52;
              redrawPopUpButtons(GUI_popupobject);
              break;
              
            case 40+52:   // shift lock off
            case 40+2*52:   // shift lock off
              GUI_popupkeyoffset=0;
              redrawPopUpButtons(GUI_popupobject);
              break;
              
            default:
            
              if (GUIobject_Stringdata1[GUI_popupobject].length()<GUIobject_data4[GUI_popupobject]){
                newvalue=newvalue+(String)keymap[keynumber];
              }
              break;
            }

            Put_Text(newvalue,0,GUI_ysize-GUIpopup_bw,BVS_28);
            updateTextInput(GUI_popupobject,newvalue);
                     
          }
        }
      }
    } else {
      // nothing pressed, if button still shown as pressed show it as unpressed.
      //
      anyButtonPressed=false;
      if (GUIpopupbuttonpressed!=-1){
        drawSingleKey(GUIpopupbuttonpressed,clrWhite,clrBlack);
        GUIpopupbuttonpressed=-1;
      }
    }  
  } else {

    // Then check all normal buttons

    for (int i = GUI_firstObject; i < GUI_objects+1; i++) {
      if (((GUIobject_type[i]==GUI_Type_button)||(GUIobject_type[i]==GUI_Type_cycleStringbutton)||(GUIobject_type[i]==GUI_Type_ghostbutton)||(GUIobject_type[i]==GUI_Type_checkBox)||(GUIobject_type[i]==GUI_Type_imagebutton)||(GUIobject_type[i]==GUI_Type_cycleButton)||(GUIobject_type[i]==GUI_Type_TextInput))) { 
        if (checkButton(i)){found=i;}; 
      }
    }
  }
  return found;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            PANEL
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// function add PANEL
//
//
// variable			 stored in		  description
//
// word x			[x]				{ location of label x }
// word y			[y]				{ location of label y }
// word xs			[xs]			{ size of the panel x in pixels }
// word ys			[ys]			{ size of the panel y in pixels }
// long colour		[colour]		{ colour of background }
// long borcolour	[borcolour]		{ colour of the border }
// long textcolour	[textcolour]	{ colour of text }
// byte borwidth	[borwidth]		{ width of the border }
// String top		[top]			{ text of label }
// int xo			[xo]			{ location of text within the panel x }
// int yo			[yo]			{ location of text within the panel y }
// int font			[font]			{ font for text }
// bool visible     [visible]    	{ visibility flag }
// bool changed		[changed]		{ object has changed since drawn or not drawn at all }
//
int DUEGUI::addPanel(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,byte borwidth,String top,int xo,int yo,int font,bool visible,int URN){
  GUI_objects+=1;
  GUIobject_type[GUI_objects]=GUI_Type_panel;
  GUIobject_x[GUI_objects]=x;
  GUIobject_y[GUI_objects]=y;
  GUIobject_xs[GUI_objects]=xs;
  GUIobject_ys[GUI_objects]=ys;
  if (xo==posRight){
    GUIobject_xo[GUI_objects]=(25+xs-((top.length())*16));
  } else {
    if (xo==posCentre){
      GUIobject_xo[GUI_objects]=15+((xs-(top.length()*16))/2);
    } else {
      if (xo==posLeft){
        GUIobject_xo[GUI_objects]=15;
      } else {
        GUIobject_xo[GUI_objects]=xo+15;
      }
    }
  }
  if (yo==posCentre){
    GUIobject_yo[GUI_objects]=ys/2-3-cfont.y_size/2    ;
  } else {
    GUIobject_yo[GUI_objects]=yo;
  }
  GUIobject_colour[GUI_objects]=colour;
  GUIobject_borcolour[GUI_objects]=borcolour;
  GUIobject_textcolour[GUI_objects]=textcolour;
  GUIobject_borwidth[GUI_objects]=borwidth;
  GUIobject_top[GUI_objects]=top;
  GUIobject_font[GUI_objects]=font;

  GUIobject_UniqueReference[GUI_objects]=URN;  
  GUIobject_visible[GUI_objects]=visible;
  GUIobject_changed[GUI_objects]=true;
  return GUI_objects;
}

void DUEGUI::drawPanel(int objectNumber){
  if (objectVisible(objectNumber)){
    updatingScreen();
    setColorLong(GUIobject_colour[objectNumber]);
    fillRoundRect(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_x[objectNumber]+GUIobject_xs[objectNumber],GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]);
    setColorLong(GUIobject_borcolour[objectNumber]);
    drawRoundRect(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_x[objectNumber]+GUIobject_xs[objectNumber],GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]);
    setColorLong(GUIobject_textcolour[objectNumber]);
    setBackColorLong(GUIobject_colour[objectNumber]);
    Put_Text(GUIobject_top[objectNumber],GUIobject_x[objectNumber]+GUIobject_xo[objectNumber],GUIobject_y[objectNumber]+GUIobject_yo[objectNumber],GUIobject_font[objectNumber]);
    finishedUpdatingScreen();
    GUIobject_changed[objectNumber]=false;
  } 
}

  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                    PROGRESS BAR
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// function add PROGRESS BAR
//
//
// variable			 stored in		  description
//
// word x			[x]				{ location of label x }
// word y			[y]				{ location of label y }
// word xs			[xs]			{ size of the panel x in pixels }
// word ys			[ys]			{ size of the panel y in pixels }
// long colour		[colour]		{ colour of background }
// long borcolour	[borcolour]		{ colour of the border }
// long textcolour	[textcolour]	{ colour of text }
// byte borwidth	[borwidth]		{ width of the border }
// String top		[top]			{ text of label }
// int maxvalue		[xo]			{ max value }
// int redvalue		[yo]			{ red value }
// int initialvalue	[data1]			{ initial/current value }
// 					[data2]			{ displayed value }
// int options		[font]			{ options for progress bar }
// bool visible     [visible]    	{ visibility flag }
// bool changed		[changed]		{ object has changed since drawn or not drawn at all }
//

int DUEGUI::addProgressBar(word x,word y,word xs,word ys,word maxvalue,word redvalue,word initialvalue,word options,long colour,long borcolour,long backcolour,long redcolour,long borwidth,bool visible,int URN){
  GUI_objects+=1;
  GUIobject_type[GUI_objects]=GUI_Type_panel;
  GUIobject_x[GUI_objects]=x;
  GUIobject_y[GUI_objects]=y;
  GUIobject_xs[GUI_objects]=xs;
  GUIobject_ys[GUI_objects]=ys;
  GUIobject_borwidth[GUI_objects]=borwidth;
  GUIobject_xo[GUI_objects]=maxvalue;
  GUIobject_yo[GUI_objects]=redvalue;
  GUIobject_data1[GUI_objects]=initialvalue;
  GUIobject_data2[GUI_objects]=0;
  GUIobject_font[GUI_objects]=options;
  GUIobject_colour[GUI_objects]=colour;
  GUIobject_borcolour[GUI_objects]=borcolour;
  GUIobject_pressedcolour[GUI_objects]=redcolour;
  GUIobject_pressedtextcolour[GUI_objects]=backcolour;
  GUIobject_UniqueReference[GUI_objects]=URN;  
  GUIobject_visible[GUI_objects]=visible;
  GUIobject_changed[GUI_objects]=true;
  return GUI_objects;
}

void DUEGUI::updateProgressBar(int objectNumber,int value){
  if (objectVisible(objectNumber)){
    if (GUIobject_data2[objectNumber]!=value){
      GUIobject_data1[objectNumber]=value;
      int v2=0;
      if (value>GUIobject_yo[objectNumber]){
        v2=value-GUIobject_yo[objectNumber];
        value=GUIobject_yo[objectNumber];
      }
      if ((GUIobject_font[objectNumber]&&1)==1){
        //
        // horizontal
        //
        int h1=GUIobject_borwidth[objectNumber]+GUIobject_x[objectNumber]+(((GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])*value)/GUIobject_xo[objectNumber]);
        int h2=h1+(((GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])*v2)/GUIobject_xo[objectNumber]);
        setColorLong(GUIobject_colour[objectNumber]);
        fillRect(GUIobject_x[objectNumber]+GUIobject_borwidth[objectNumber],GUIobject_y[objectNumber]+GUIobject_borwidth[objectNumber],h1,(GUIobject_y[objectNumber]+GUIobject_ys[objectNumber])-GUIobject_borwidth[objectNumber]);
        setColorLong(GUIobject_pressedcolour[objectNumber]);
        fillRect(h1,GUIobject_y[objectNumber]+GUIobject_borwidth[objectNumber],h2,(GUIobject_y[objectNumber]+GUIobject_ys[objectNumber])-GUIobject_borwidth[objectNumber]);
        setColorLong(GUIobject_pressedtextcolour[objectNumber]);
        fillRect(h2,GUIobject_y[objectNumber]+GUIobject_borwidth[objectNumber],(GUIobject_x[objectNumber]+GUIobject_xs[objectNumber])-GUIobject_borwidth[objectNumber],(GUIobject_y[objectNumber]+GUIobject_ys[objectNumber])-GUIobject_borwidth[objectNumber]);
      } else {
        //
        // vertical
        //
        int h1=(GUIobject_y[objectNumber]+GUIobject_ys[objectNumber])-GUIobject_borwidth[objectNumber]-(((GUIobject_ys[objectNumber]-GUIobject_borwidth[objectNumber])*value)/GUIobject_xo[objectNumber]);
        int h2=h1-(((GUIobject_ys[objectNumber]-GUIobject_borwidth[objectNumber])*v2)/GUIobject_xo[objectNumber]);
        setColorLong(GUIobject_colour[objectNumber]);
        fillRect(GUIobject_x[objectNumber]+GUIobject_borwidth[objectNumber],(GUIobject_y[objectNumber]+GUIobject_ys[objectNumber])-GUIobject_borwidth[objectNumber],(GUIobject_x[objectNumber]+GUIobject_xs[objectNumber])-GUIobject_borwidth[objectNumber],h1);
        setColorLong(GUIobject_pressedcolour[objectNumber]);
        fillRect(GUIobject_x[objectNumber]+GUIobject_borwidth[objectNumber],h1,(GUIobject_x[objectNumber]+GUIobject_xs[objectNumber])-GUIobject_borwidth[objectNumber],h2);
        setColorLong(GUIobject_pressedtextcolour[objectNumber]);
        fillRect(GUIobject_x[objectNumber]+GUIobject_borwidth[objectNumber],h2,(GUIobject_x[objectNumber]+GUIobject_xs[objectNumber])-GUIobject_borwidth[objectNumber],GUIobject_y[objectNumber]+GUIobject_borwidth[objectNumber]);
      }
      GUIobject_data2[objectNumber]=GUIobject_data1[objectNumber];
      GUIobject_changed[objectNumber]=false;
    }
  }
}

void DUEGUI::drawActualProgressBar(int objectNumber){
  setColorLong(GUIobject_borcolour[objectNumber]);
  fillRect(GUIobject_x[objectNumber],GUIobject_y[objectNumber],(GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]),(GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]));          
  updateProgressBar(objectNumber,GUIobject_data1[objectNumber]);              
}

void DUEGUI::drawProgressBar(int objectNumber){
  if (objectVisible(objectNumber)){
    updatingScreen();
    drawActualProgressBar(objectNumber);
    finishedUpdatingScreen();
    GUIobject_changed[objectNumber]=false;
  } 
}

  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            Shape
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// function add Label
//
//
// variable			 stored in		  description
//
// word x			[x]				{ location of label x }
// word y			[y]				{ location of label y }
// long colour		[colour]		{ colour of background }
// byte type		[font]			{ shape type }
// int  a        	[data1]         { data 1 }
// int  b        	[data2]         { data 2 }
// int  c        	[data3]         { data 3 }
// int  d        	[data4]         { data 4 }
// int  e        	[data5]         { data 5 }
// int  f        	[data6]         { data 6 }
// int  g        	[xo]            { data 7 }
// int  h        	[yo]            { data 8 }
// int  i        	[xs]            { data 9 }
// int  j        	[ys]            { data 10 }
// bool visible     [visible]    	{ visibility flag }
// bool changed		[changed]		{ object has changed since drawn or not drawn at all }
//
//  1 = shapeRectangle     
//  2 = shapeRectangle_FILL
//  3 = line
//
//
int DUEGUI::addShape(word x,word y,long colour,int type,int a,int b,int c,int d,int e,int f,int g,int h,int i,int j,bool visible,int URN){
  GUI_objects+=1;
  GUIobject_type[GUI_objects]=GUI_Type_shape;
  GUIobject_x[GUI_objects]=x;
  GUIobject_y[GUI_objects]=y;
  GUIobject_colour[GUI_objects]=colour;
  GUIobject_font[GUI_objects]=type;
  GUIobject_data1[GUI_objects]=a;
  GUIobject_data2[GUI_objects]=b;
  GUIobject_data3[GUI_objects]=c;
  GUIobject_data4[GUI_objects]=d;
  GUIobject_data5[GUI_objects]=e;
  GUIobject_data6[GUI_objects]=f;
  GUIobject_xo[GUI_objects]=g;
  GUIobject_yo[GUI_objects]=h;
  GUIobject_xs[GUI_objects]=i;
  GUIobject_ys[GUI_objects]=j;

  GUIobject_UniqueReference[GUI_objects]=URN;
  GUIobject_visible[GUI_objects]=visible;
  GUIobject_changed[GUI_objects]=true;
  return GUI_objects;
}

void DUEGUI::drawShape(int objectNumber){
  if (objectVisible(objectNumber)){
    updatingScreen();
    setColorLong(GUIobject_colour[objectNumber]);
    switch (GUIobject_font[objectNumber]){
    
      case shapeRectangle:
        // rectangle=1
        drawLine(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_x[objectNumber],GUIobject_data2[objectNumber]);   
        drawLine(GUIobject_x[objectNumber],GUIobject_data2[objectNumber],GUIobject_data1[objectNumber],GUIobject_data2[objectNumber]);
        drawLine(GUIobject_data1[objectNumber],GUIobject_data2[objectNumber],GUIobject_data1[objectNumber],GUIobject_y[objectNumber]);
        drawLine(GUIobject_data1[objectNumber],GUIobject_y[objectNumber],GUIobject_x[objectNumber],GUIobject_y[objectNumber]);
      break;
      
      case shapeRectangle_FILL:
        // filled rectangle=2
        
      break;
              
      case shapeLine:
        // line=3
        
        Serial.print(GUIobject_x[objectNumber]);
        Serial.print(GUIobject_y[objectNumber]);
        Serial.print(GUIobject_data1[objectNumber]);
        Serial.print(GUIobject_data2[objectNumber]);
        
        drawLine(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_data1[objectNumber],GUIobject_data2[objectNumber]);   
      break;
      
    } // end of switch
    finishedUpdatingScreen();  
    GUIobject_changed[objectNumber]=false;
  } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            Label
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// function add Label
//
//
// variable			 stored in		  description
//
// word x			[x]				{ location of label x }
// word y			[y]				{ location of label y }
// long colour		[colour]		{ colour of background }
// long textcolour	[textcolour]	{ colour of text }
// String top		[top]			{ text of label }
// int font			[font]			{ font for text }
// bool visible     [visible]    	{ visibility flag }
// bool changed		[changed]		{ object has changed since drawn or not drawn at all }
//
int DUEGUI::addLabel(word x,word y,long colour,long textcolour,String top,int font,bool visible,int URN){
  GUI_objects+=1;
  GUIobject_type[GUI_objects]=GUI_Type_panel;
  GUIobject_x[GUI_objects]=x;
  GUIobject_y[GUI_objects]=y;
  GUIobject_colour[GUI_objects]=colour;
  GUIobject_textcolour[GUI_objects]=textcolour;
  GUIobject_top[GUI_objects]=top;
  GUIobject_font[GUI_objects]=font;

  GUIobject_UniqueReference[GUI_objects]=URN;
  GUIobject_visible[GUI_objects]=visible;
  GUIobject_changed[GUI_objects]=true;
  return GUI_objects;
}

void DUEGUI::drawLabel(int objectNumber){
  if (objectVisible(objectNumber)){
    updatingScreen();
    setColorLong(GUIobject_textcolour[objectNumber]);
    setBackColorLong(GUIobject_colour[objectNumber]);
    Put_Text(GUIobject_top[objectNumber],GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_font[objectNumber]);
    finishedUpdatingScreen();  
    GUIobject_changed[objectNumber]=false;
  } 
}

void DUEGUI::removeLabel(int objectNumber){
  if (objectVisible(objectNumber)){
    updatingScreen();
    setColorLong(GUIobject_colour[objectNumber]);
    setBackColorLong(GUIobject_colour[objectNumber]);
    Put_Text(GUIobject_top[objectNumber],GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_font[objectNumber]);
    finishedUpdatingScreen();  
  } 
}

void DUEGUI::updateLabel(int objectNumber,String top){
  if ((objectVisible(objectNumber)) && (top!=GUIobject_top[objectNumber])){
    updatingScreen();
    removeLabel(objectNumber);
    GUIobject_top[objectNumber]=top;
    drawLabel(objectNumber);
    finishedUpdatingScreen();  
    GUIobject_changed[objectNumber]=false;
  } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            Clock
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// function add Clock
//
//
// variable			  stored in			  description
//
// word x            [x]          		{ location of clock centre x }
// word y            [y]          		{ location of clock centre y }
// word xs           [xs]         		{ radius of whole clock }
// word centresize   [ys]         		{ radius of clock centre point }
// long facecolour   [colour]     		{ colour of clock face }
// long borcolour    [borcolour]  		{ colour of clock border }
// long borwidth     [borwidth]   		{ border width }
// long hourcolour   [textcolour] 		{ colour of hour hand }
// long hourlen      [pressedtextcolour]{ colour of hour hand }
// long mincolour    [xo]         		{ colour of minute hand }
// long minlen       [pressedcolour]    { colour of minute hand }
// long seccolour    [yo]         		{ colour of second hand }
// long seclen       [currentstate]    	{ colour of second hand }
// bool visible      [visible]    		{ visibility flag }
// bool changed		 [changed]			{ object has changed since drawn or not drawn at all }
// int  data1        [data1]            { previous time hours }
// int  data2        [data2]            { previous time mins }
// int  data3        [data3]            { previous time secs }
// byte timeoptions  [font]       		{ clock options }
//                                          bits 0,1   : 0 = no separators
//													     1 = 15 min separators
//													     2 = 5 min separators
//													     3 = 1 min separators
//											bit 2      : lines around separators
//                                          bits 3,4,5 : 1 = 12
//														 2 = 12 & 6
//														 3 = 12,9,6 & 3
//														 4 = numbers
//
int DUEGUI::addAnalogueClock(word x,word y,word clocksize,word centresize,long facecolour,long borcolour,long hourcolour,int hourlen,long mincolour,int minlen,long seccolour,int seclen,byte borwidth, int options,bool visible,int URN){
  GUI_objects+=1;
  GUIobject_type[GUI_objects]=GUI_Type_anaClock;
  GUIobject_x[GUI_objects]=x;
  GUIobject_y[GUI_objects]=y;
  GUIobject_xs[GUI_objects]=clocksize;
  GUIobject_ys[GUI_objects]=centresize;
  GUIobject_xo[GUI_objects]=mincolour;
  GUIobject_yo[GUI_objects]=seccolour;
  GUIobject_colour[GUI_objects]=facecolour;
  GUIobject_borcolour[GUI_objects]=borcolour;
  GUIobject_textcolour[GUI_objects]=hourcolour;
  GUIobject_borwidth[GUI_objects]=borwidth;
  GUIobject_font[GUI_objects]=options;
  GUIobject_pressedcolour[GUI_objects]=minlen;
  GUIobject_pressedtextcolour[GUI_objects]=hourlen;
  GUIobject_currentstate[GUI_objects]=seclen;
  GUIobject_UniqueReference[GUI_objects]=URN;
  GUIobject_visible[GUI_objects]=visible;
  GUIobject_changed[GUI_objects]=true;
  GUIobject_data1[GUI_objects]=-1;
  return GUI_objects;
}

void DUEGUI::drawAnalogueClock(int objectNumber){
  bool lines=true;
  if (objectVisible(objectNumber)){
    int cx=GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]/2;
    int cy=GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]/2;
    updatingScreen();
    setColorLong(GUIobject_borcolour[objectNumber]);
    fillCircle(cx,cy,GUIobject_xs[objectNumber]/2);
    setColorLong(GUIobject_colour[objectNumber]);
    fillCircle(cx,cy,(GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])/2);
    
    //
    //  Draw options
    //
    if ((GUIobject_font[objectNumber]&4)==4){lines=false;}; 
    switch ((GUIobject_font[objectNumber]&3)){
     case 1: drawAnalogueClock_dividers(objectNumber,cx,cy,(GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])/2-5,GUIobject_currentstate[objectNumber]+5,4,lines);  break;
     case 2: drawAnalogueClock_dividers(objectNumber,cx,cy,(GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])/2-5,GUIobject_currentstate[objectNumber]+5,12,lines);  break;
     case 3: drawAnalogueClock_dividers(objectNumber,cx,cy,(GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])/2-5,GUIobject_currentstate[objectNumber]+5,60,lines);  break;
    }
    
    if ((GUIobject_font[objectNumber]&24)==24){
      setColorLong(GUIobject_borcolour[objectNumber]);
      setBackColorLong(GUIobject_colour[objectNumber]);
      Put_Text("12",cx-5,cy-(GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])/2+5,BVS_19);
      Put_Text("6",cx-5,cy+(GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])/2-20,BVS_19);
      Put_Text("3",cx+(GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])/2-15,cy-5,BVS_19);
      Put_Text("9",cx-(GUIobject_xs[objectNumber]-GUIobject_borwidth[objectNumber])/2+5,cy-5,BVS_19);
    };

    drawHands(objectNumber);

    setColorLong(GUIobject_borcolour[objectNumber]);
    fillCircle(cx,cy,GUIobject_ys[objectNumber]/2);
    finishedUpdatingScreen();  
    GUIobject_changed[objectNumber]=false;
    anyClockVisible=true;
  }
}

int DUEGUI::addDigitalClock_Time(word x,word y,long colour,long textcolour,int options,int font,bool visible,int URN){
  addLabel(x,y,colour,textcolour,"",font,visible,URN);
  GUIobject_type[GUI_objects]=GUI_Type_digTime;
  GUIobject_currentstate[GUI_objects]=options;
  anyClockVisible=true;
  return GUI_objects;
}

void DUEGUI::drawDigitalClock_Time(int objectNumber){
  if (objectVisible(objectNumber)){
    drawLabel(objectNumber);
    anyClockVisible=true;
  }
}

int DUEGUI::addDigitalClock_Date(word x,word y,long colour,long textcolour,int options,int font,bool visible,int URN){
  addLabel(x,y,colour,textcolour,"",font,visible,URN);
  GUIobject_type[GUI_objects]=GUI_Type_digDate;
  GUIobject_currentstate[GUI_objects]=options;
  anyClockVisible=true;
  return GUI_objects;
}

void DUEGUI::drawDigitalClock_Date(int objectNumber){
  if (objectVisible(objectNumber)){
    drawLabel(objectNumber);
    anyClockVisible=true;
  }
};

void DUEGUI::drawHands(int objectNumber){
  //
  //   GUIobject_data1[objectNumber]  {old hours}
  //   GUIobject_data2[objectNumber]  {old mins}
  //   GUIobject_data3[objectNumber]  {old seconds}
  //   GUIobject_data4[objectNumber]  {hours}
  //   GUIobject_data5[objectNumber]  {mins}
  //   GUIobject_data6[objectNumber]  {seconds}
  //
  int cx=GUIobject_x[objectNumber]+GUIobject_xs[objectNumber]/2;
  int cy=GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]/2;
  if (GUIobject_data1[objectNumber]!=-1){
    //remove old hands
    setColorLong(GUIobject_colour[objectNumber]);
    drawLine(cx,cy,cx+calculate_x(GUIobject_data3[objectNumber]*6,GUIobject_currentstate[objectNumber]),cy+calculate_y(GUIobject_data3[objectNumber]*6,GUIobject_currentstate[objectNumber]));
    drawLine(cx,cy,cx+calculate_x(GUIobject_data2[objectNumber]*6,GUIobject_pressedcolour[objectNumber]),cy+calculate_y(GUIobject_data2[objectNumber]*6,GUIobject_pressedcolour[objectNumber]));
    drawLine(cx,cy,cx+calculate_x(GUIobject_data1[objectNumber]*6,GUIobject_pressedtextcolour[objectNumber]),cy+calculate_y(GUIobject_data1[objectNumber]*6,GUIobject_pressedtextcolour[objectNumber]));
  }
  setColorLong(GUIobject_yo[objectNumber]);
  drawLine(cx,cy,cx+calculate_x(GUIobject_data6[objectNumber]*6,GUIobject_currentstate[objectNumber]),cy+calculate_y(GUIobject_data6[objectNumber]*6,GUIobject_currentstate[objectNumber]));
  setColorLong(GUIobject_xo[objectNumber]);
  drawLine(cx,cy,cx+calculate_x(GUIobject_data5[objectNumber]*6,GUIobject_pressedcolour[objectNumber]),cy+calculate_y(GUIobject_data5[objectNumber]*6,GUIobject_pressedcolour[objectNumber]));
  setColorLong(GUIobject_textcolour[objectNumber]);
  drawLine(cx,cy,cx+calculate_x(GUIobject_data4[objectNumber]*6,GUIobject_pressedtextcolour[objectNumber]),cy+calculate_y(GUIobject_data4[objectNumber]*6,GUIobject_pressedtextcolour[objectNumber]));
  setColorLong(GUIobject_borcolour[objectNumber]);
  fillCircle(cx,cy,GUIobject_ys[objectNumber]/2);
  GUIobject_data1[objectNumber]=GUIobject_data4[objectNumber];
  GUIobject_data2[objectNumber]=GUIobject_data5[objectNumber];
  GUIobject_data3[objectNumber]=GUIobject_data6[objectNumber];
}

void DUEGUI::drawAnalogueClock_dividers(int objectNumber,int cx,int cy,int inner,int outer,int number,bool lines){
  setColorLong(GUIobject_borcolour[objectNumber]);
  for (int k = 0; k < number; k++) {
    drawLine(cx+calculate_x(k*(360/number),inner),cy+calculate_y(k*(360/number),inner),cx+calculate_x(k*(360/number),outer),cy+calculate_y(k*(360/number),outer));
  }
  if (lines){
    drawCircle(cx,cy,inner);
    drawCircle(cx,cy,outer);
  }
}

void DUEGUI::setObjectTime(int objectNumber,int hh, int mm, int ss){
  GUIobject_top[objectNumber]=displayNumFormat(hh,2)+":"+displayNumFormat(mm,2)+":"+displayNumFormat(ss,2)+" ";
  GUIobject_data4[objectNumber]=hh;
  GUIobject_data5[objectNumber]=mm;
  GUIobject_data6[objectNumber]=ss;
}

void DUEGUI::setObjectDate(int objectNumber,int yyyy, int month, int day, bool dateNorm, bool dateFour){
  if (dateNorm){
    GUIobject_top[objectNumber]=displayNumFormat(day,2)+"/"+displayNumFormat(month,2);
  } else {
    GUIobject_top[objectNumber]=displayNumFormat(month,2)+"/"+displayNumFormat(day,2);
  }
  if (dateFour){
    GUIobject_top[objectNumber]=GUIobject_top[objectNumber]+"/"+String(yyyy,10)+" ";
  } else {
    GUIobject_top[objectNumber]=GUIobject_top[objectNumber]+"/"+String(yyyy,10).substring(3)+" ";
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                            Image
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// function add Image
//
//
// variable			  	stored in	description
//
// word x				[x]			{ location of the image x }	
// word y				[y]			{ location of the image y }
// word xs				[x]			{ size of the image x <CALCULATED BY OBJECT INIT ROUTINE> }	
// word ys				[y]			{ size of the image y <CALCULATED BY OBJECT INIT ROUTINE> }	
// int imageNumber		[font]		{ image number from the rom }
// bool visible     	[visible]   { visibility flag }
// bool changed			[changed]	{ object has changed since drawn or not drawn at all }
//
int DUEGUI::addImage(word x,word y,int imageNumber,bool visible,int URN){
  GUI_objects+=1;
  GUIobject_type[GUI_objects]=GUI_Type_image;
  GUIobject_x[GUI_objects]=x;
  GUIobject_y[GUI_objects]=y;
  GUIobject_font[GUI_objects]=imageNumber;
  getImageSize(imageNumber);
  GUIobject_xs[GUI_objects] = iXsize;	
  GUIobject_ys[GUI_objects] = iYsize;	
  GUIobject_UniqueReference[GUI_objects]=URN;
  GUIobject_visible[GUI_objects]=visible;  
  GUIobject_changed[GUI_objects]=true;
  return GUI_objects;
}

void DUEGUI::drawImage(int objectNumber){
  if (objectVisible(objectNumber)){
    Load_image(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_font[objectNumber]);
    GUIobject_changed[objectNumber]=false;
  } 
}

void DUEGUI::getImageSize(int imageNumber){
  unsigned long address;
  unsigned char pixelH, pixelL,t1;
  imageNumber = imageNumber + 224;
  address=(unsigned long)imageNumber*4096;
  char H = address>>16;
  char M = address>>8;
  digitalWrite(CSpin,LOW);
  spiSend(0x03);
  spiSend(H);
  spiSend(M);
  spiSend(0x00);
  t1 = spiRec();
  t1 = spiRec();
  iXsize = spiRec();	
  iXsize = (iXsize << 8) | spiRec();
  iYsize = spiRec();	
  iYsize = (iYsize << 8) | spiRec();
  digitalWrite(CSpin,HIGH);
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                         WHOLE SCREEN FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DUEGUI::HandleShowButtons(int URN){
  if (URN==GUIURNexit){
    // exit show state
    ClearObjects(GUI_firstObject,GUI_objects);
    GUI_objects=GUI_firstObject;
    GUI_firstObject=0;
    clrScr();
    redrawAllObjects();
  }
  if (URN==GUIURNsave){
    // save show state

  }
}

void DUEGUI::HandleShowLoop(){
  //
  // The following section reads the raw touch X & Y locations and allows the
  // minimum and maximums to be seen by the user.  We are making use of the
  // updateLabel(int objectNumber, String new_value) function.  This function is
  // a quick way of checking if the value of the text of that object has changed
  // and if it has remove the old text, display the new text and update the value
  // all in one go.
  // 
  if (GUI_ShowCalibrate){
    GUIpoint_x=getX();
    GUIpoint_y=getY();
    if ((GUIold_x!=GUIpoint_x)||(GUIold_y!=GUIpoint_y)){
      setBackColorLong(0);
      drawPixel(GUIold_x,GUIold_y);
      setColorLong(0xFF00FF);
      drawPixel(GUIpoint_x,GUIpoint_y);  
      GUIold_x=GUIpoint_x;
      GUIold_y=GUIpoint_y;
    }
    updateLabel(findObjectByURN(GUIURNx_raw),(String)TP_X);
    updateLabel(findObjectByURN(GUIURNy_raw),(String)TP_Y);
    if ((TP_X<GUIval_x_min) && (TP_X>2)){
      GUIval_x_min=TP_X;
      updateLabel(findObjectByURN(GUIURNcal_x_min),(String)GUIval_x_min);
    };
    if ((TP_X>GUIval_x_max) && (TP_X<99999)){
      GUIval_x_max=TP_X;
      updateLabel(findObjectByURN(GUIURNcal_x_max),(String)GUIval_x_max);
    };
    if ((TP_Y<GUIval_y_min) && (TP_Y>2)){
      GUIval_y_min=TP_Y;
      updateLabel(findObjectByURN(GUIURNcal_y_min),(String)GUIval_y_min);
    };
    if ((TP_Y>GUIval_y_max) && (TP_Y<99999)){
      GUIval_y_max=TP_Y;
      updateLabel(findObjectByURN(GUIURNcal_y_max),(String)GUIval_y_max);
    };
  }
}


void DUEGUI::showCalibrate(){
  clrScr();
  Serial.println("Setup: calibrate_screen");  
  GUI_firstObject=GUI_objects;
  addPanel(0,0,799,50,clrBlue,clrWhite,clrWhite,2,"Screen calibration",280,8,BVS_34,optVisible,GUIURNnull);
  addLabel(150,80,clrBlack,clrWhite,"x min",BVS_28,optVisible,GUIURNnull);
  addLabel(150,130,clrBlack,clrWhite,"x max",BVS_28,optVisible,GUIURNnull);
  addLabel(150,180,clrBlack,clrWhite,"y min",BVS_28,optVisible,GUIURNnull);
  addLabel(150,230,clrBlack,clrWhite,"y max",BVS_28,optVisible,GUIURNnull);
  addLabel(150,280,clrBlack,clrWhite,"x now",BVS_28,optVisible,GUIURNnull);
  addLabel(150,330,clrBlack,clrWhite,"y now",BVS_28,optVisible,GUIURNnull);
  addLabel(300,80,clrBlack,clrWhite,"0",BVS_28,optVisible,GUIURNcal_x_min);
  addLabel(300,130,clrBlack,clrWhite,"0",BVS_28,optVisible,GUIURNcal_x_max);
  addLabel(300,180,clrBlack,clrWhite,"0",BVS_28,optVisible,GUIURNcal_y_min);
  addLabel(300,230,clrBlack,clrWhite,"0",BVS_28,optVisible,GUIURNcal_y_max);
  addLabel(300,280,clrBlack,clrWhite,"0",BVS_28,optVisible,GUIURNx_raw);
  addLabel(300,330,clrBlack,clrWhite,"0",BVS_28,optVisible,GUIURNy_raw);
  addButton(500,120,150,50,clrBlue,clrWhite,clrWhite,clrRed,clrWhite,2,"Save",20,posCentre,BVS_28,optVisible,GUIURNsave); 
  addButton(500,200,150,50,clrBlue,clrWhite,clrWhite,clrRed,clrWhite,2,"Exit",20,posCentre,BVS_28,optVisible,GUIURNexit); 
  addShape(700,330,clrWhite,1,700,430,0,0,0,0,0,0,0,0,optVisible,GUIURNnull);
  addShape(650,380,clrWhite,1,750,380,0,0,0,0,0,0,0,0,optVisible,GUIURNnull);
  redrawAllObjects();
  GUI_ShowCalibrate=true;
  GUIval_x_min=99999;
  GUIval_x_max=0;
  GUIval_y_min=99999;
  GUIval_y_max=0;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                         GENERAL OBJECT FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DUEGUI::ClearObjects(int first,int last){
  for (byte i = first; i < last; i++) {
    GUIobject_type[i]=0;
    GUIobject_currentstate[i]=0;
    GUIobject_x[i]=0;
    GUIobject_y[i]=0;
    GUIobject_xs[i]=0;
    GUIobject_ys[i]=0;
    GUIobject_xo[i]=0;
    GUIobject_yo[i]=0;
    GUIobject_colour[i]=0;
    GUIobject_borcolour[i]=0;
    GUIobject_textcolour[i]=0;
    GUIobject_pressedcolour[i]=0;
    GUIobject_pressedtextcolour[i]=0;
    GUIobject_borwidth[i]=0;
    GUIobject_top[i]="";
    GUIobject_font[i]=0;
    GUIobject_link[i]=0;
    GUIobject_linkoption[i]=0;
    GUIobject_visible[i]=optInvisible;
    GUIobject_changed[i]=false;
  }

}

void DUEGUI::clearAllObjects(){
  ClearObjects(0,maxobjects);
  anyClockVisible=false;
  anyButtonPressed=false;
  GUI_firstObject=0;
  GUI_objects=0;
}

void DUEGUI::makeObjectInvisible(int objectNumber,bool redraw){
  GUIobject_visible[objectNumber]=optInvisible;
  GUIobject_changed[objectNumber]=true;
  if (redraw){
    clearObjectArea(objectNumber);
  }
}

void DUEGUI::makeObjectVisible(int objectNumber,bool redraw){
  GUIobject_visible[objectNumber]=optVisible;
  GUIobject_changed[objectNumber]=true;
  if (redraw){
    redrawObject(objectNumber);
  }
}

void DUEGUI::drawActualKey(int x,int y,int xs,int ys,String Keytop,long keycolour,long textcolour){
  setColorLong(keycolour);
  fillRoundRect(x,y,x+xs,y+ys);
  setColorLong(clrBlack);
  drawRoundRect(x,y,x+xs,y+ys);
  setColorLong(textcolour);
  setBackColorLong(keycolour);
  Put_Text(Keytop,x+5,y+5,BVS_28);
}

void DUEGUI::drawSingleKey(int keynumber,long capcolour,long textcolour){

  int keypos=(keynumber%52);
  int i=keynumber%13;
  int k=keypos/13;
  switch (keypos){
    case 35: break;
    case 41: break;
    case 43: break;
    case 44: break;
    case 45: break;
    case 40: drawActualKey(5+(i*GUIpopup_bw),GUI_ysize+(k*GUIpopup_bw),25,20,"CAP",capcolour,textcolour); break;
    case 22: drawActualKey(5+(i*GUIpopup_bw),GUI_ysize+(k*GUIpopup_bw),25,20,"Del",capcolour,textcolour); break;
    case 39: drawActualKey(5+(i*GUIpopup_bw),GUI_ysize+(k*GUIpopup_bw),25,20,"##",capcolour,textcolour); break;
    case 34: drawActualKey(5+(i*GUIpopup_bw),GUI_ysize+(k*GUIpopup_bw),50,20,"Enter",capcolour,textcolour); break;
    case 42: drawActualKey(5+(i*GUIpopup_bw),GUI_ysize+(k*GUIpopup_bw),50,20,"Space",capcolour,textcolour); break;
    default: drawActualKey(5+(i*GUIpopup_bw),GUI_ysize+(k*GUIpopup_bw),25,20,(String)keymap[keynumber],capcolour,textcolour); break;
  }

}

void DUEGUI::redrawPopUpButtons(int objectNumber){
  for (byte keynumber = 0; keynumber < 52; keynumber++) {
    drawSingleKey(keynumber+GUI_popupkeyoffset,clrWhite,clrBlack);
  } 

}

void DUEGUI::makePopUp(int objectNumber,int yoffset){
  updatingScreen();

  /*
  if (GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]<getDisplayYSize()-popup_ys){
    //
    // Popup is fully below object in question so just display it...
    //
    GUI_ysize=getDisplayYSize()-popup_ys;
    setColor(85,85,85);
    fillRoundRect(0,GUI_ysize,getDisplayXSize(),getDisplayYSize());
    GUI_yoffset=0;
  } else {
    //
    // Popup is covering object in question so need to make adjustments....
    //
    GUI_yoffset=GUIobject_data5[objectNumber];  
    GUI_ysize=getDisplayYSize()-popup_ys;
    clrScr();
    redrawAllObjects();
    setColor(85,85,85);
    fillRect(0,GUI_ysize,getDisplayXSize(),getDisplayYSize());
  } 
  */

  //clrScr();                                                        // ediaz
  GUI_ysize=getDisplayYSize()-popup_ys;                              // ediaz
  setColor(255,255,255);
  fillRect(0,GUI_ysize-GUIpopup_bw,getDisplayXSize()-1,GUI_ysize);     // ediaz - make textarea
  setColor(85,85,85);                                              
  fillRect(0,GUI_ysize,getDisplayXSize()-1,getDisplayYSize()-1);       // ediaz - make keyboard area

  redrawPopUpButtons(objectNumber);
  GUI_popupactive=true;
  GUI_popupobject=objectNumber;
  GUI_popupkeyoffset=0;
  GUI_popupclearedwith=-1;
  anyButtonPressed=false;

  Put_Text(GUIobject_Stringdata1[GUI_popupobject],0,GUI_ysize-GUIpopup_bw,BVS_28);

  finishedUpdatingScreen(); 
}

void DUEGUI::clearPopUp(){
  //GUI_xoffset=0;
  //GUI_yoffset=0;
  GUI_xsize=getDisplayXSize();
  GUI_ysize=getDisplayYSize();
  GUI_popupactive=false;
  clrScr();
  redrawAllObjects();
  anyButtonPressed=false;
}


void DUEGUI::clearObjectArea(int objectNumber){
  updatingScreen();
  GUIobject_visible[objectNumber]=optInvisible;
  switch (GUIobject_type[objectNumber]){
    
    case GUI_Type_panel:
    case GUI_Type_cycleButton:
    case GUI_Type_cycleStringbutton:
    case GUI_Type_checkBox:
    case GUI_Type_button:
    case GUI_Type_image:
      // Object type is a button or panel.
      setColor(0,0,0);
      fillRoundRect(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_x[objectNumber]+GUIobject_xs[objectNumber],GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]);
    break;
      
    case GUI_Type_anaClock:
      setColor(0,0,0);
      fillCircle(GUIobject_x[objectNumber],GUIobject_y[objectNumber],GUIobject_xs[objectNumber]);
    break;
    
    case GUI_Type_label:
      removeLabel(objectNumber);
    break;
      
  } // end of switch
  finishedUpdatingScreen();  
}

bool DUEGUI::objectVisible(int objectNumber){
  //int visible_range_y1=-GUI_yoffset;
  int visible_range_y1=0;
  int visible_range_y2=visible_range_y1+GUI_ysize;
  //
  if ((GUIobject_y[objectNumber]>visible_range_y2) || (GUIobject_y[objectNumber]+GUIobject_ys[objectNumber]<visible_range_y1) 
         || (GUIobject_visible[objectNumber]==false) || (objectNumber<GUI_firstObject)){
    return false;
  } else {
    return true;
  }
}

void DUEGUI::redrawObject(int objectNumber){
  if ((objectVisible(objectNumber)) && ((objectNumber+1)>GUI_firstObject)){
    switch (GUIobject_type[objectNumber]){
    
    case GUI_Type_panel:
      // Object type is a panel.
      drawPanel(objectNumber); 
    break;
    
    case GUI_Type_shape:
      // Object type is a shape.
      drawShape(objectNumber); 
    break;
      
    case GUI_Type_button:
      // Object type is a button.
      drawButton(objectNumber); 
    break;
      
    case GUI_Type_cycleStringbutton:
    case GUI_Type_cycleButton:
    case GUI_Type_imagebutton:
    case GUI_Type_checkBox:
    case GUI_Type_label:
    case GUI_Type_digDate:
    case GUI_Type_digTime:
      // Object type is a text.
      drawButton(objectNumber); 
    break;
    
    case GUI_Type_image:
      // Object type is an image.
      drawImage(objectNumber); 
    break;
    
    case GUI_Type_TextInput:
      // Object type is an input box.
      drawTextInput(objectNumber); 
    break;
    
    case GUI_Type_anaClock:
      // Object type is a analogue clock.
      drawAnalogueClock(objectNumber); 
    break;
        
    } // end of switch
    GUIobject_changed[objectNumber]=false;
  }
}

bool DUEGUI::returnBoolValue(int objectNumber){
  switch (GUIobject_type[objectNumber]){
    case GUI_Type_checkBox:
      if (GUIobject_data3[objectNumber]==1){
        return true;
      } else {
        return false;
      }
    break;
    
  } 
}

int DUEGUI::returnIntValue(int objectNumber){

  switch (GUIobject_type[objectNumber]){         
    case GUI_Type_cycleButton:
      return (int)GUIobject_data3[objectNumber];
      
    break;
    default:Serial.print("<DEFAULT INT VALUE>");
  } 
}

String DUEGUI::returnStringValue(int objectNumber){
  return GUIobject_Stringdata1[objectNumber];
}

void DUEGUI::redrawAllObjects(){
  int objectNumber=GUI_firstObject;
  while (objectNumber<GUI_objects){
    objectNumber+=1;
    if(GUIobject_visible[objectNumber]){
      redrawObject(objectNumber);
    };
  } 
}

void DUEGUI::redrawChangedObjects(){
  int objectNumber=GUI_firstObject;
  while (objectNumber<GUI_objects){
    objectNumber+=1;    
    if(GUIobject_changed[objectNumber]){
      // object has changed
      //
      redrawObject(objectNumber);
    };
  } 
}

int DUEGUI::findObjectByURN(int URN){
  int objectNumber=-1;
  while (objectNumber<GUI_objects){
    objectNumber+=1;    
    if(GUIobject_UniqueReference[objectNumber]==URN){
      return objectNumber; break;
    };
  } 
}

void DUEGUI::debug(int onoff){
  _debugging=onoff;
}

void DUEGUI::db_varSt(int level,String out,String val){
  if (level>=_debugging){
    Serial.print(out);
    Serial.println(val);
  }
}

void DUEGUI::db_St(int level,String out){
  if (level>=_debugging){
    Serial.println(out);
  }
}

void DUEGUI::db_varInt(int level,String out,int val){
  if (level>=_debugging){
    Serial.print(out);
    Serial.println(val);
  }
}


