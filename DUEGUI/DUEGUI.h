/*
  UTFT.h - Arduino/chipKit library support for Color TFT LCD Boards
  Copyright (C)2010-2012 Henning Karlsen. All right reserved
  
  modified by coldtears electronics to suite CTE TFT Modules 
  
  This library is the continuation of my ITDB02_Graph, ITDB02_Graph16
  and RGB_GLCD libraries for Arduino and chipKit. As the number of 
  supported display modules and controllers started to increase I felt 
  it was time to make a single, universal library as it will be much 
  easier to maintain in the future.

  Basic functionality of this library was origianlly based on the 
  demo-code provided by ITead studio (for the ITDB02 modules) and 
  NKC Electronics (for the RGB GLCD module/shield).

  This library supports a number of 8bit, 16bit and serial graphic 
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

#ifndef DUEGUI_h
#define DUEGUI_h

#define DueGUI_tickHandler TC3_Handler
#define DueGUI_timer TC3_IRQn 
#define maxbuttons 100
#define ticksPerSecond 8

#define CAL_X_MIN  = 130
#define CAL_X_MAX  = 3760
#define CAL_Y_MIN  = 250
#define CAL_Y_MAX  = 3900
#define CAL_S 0x8031f1dfUL
#define CAL_X_SIZE = 800
#define CAL_Y_SIZE = 480

#define posLeft 0
#define posRight 9999
#define posCentre 9998
#define posTop 9997
#define posBottom 9996
#define posAuto 9995
#define LEFT 0
#define RIGHT 9999
#define CENTRE 9998

#define cycYN 1
#define cycYESNO 2
#define cycCHECKBOX 3

#define optVisible true
#define optInvisible false

#define PORTRAIT 0
#define LANDSCAPE 1

#define SERIAL_4PIN		4
#define SERIAL_5PIN		5

/* ediaz
#if defined(__AVR__)
	#include "Arduino.h"
	#include "HW_AVR_defines.h"
#elif defined(__SAM3X8E__)
	#include "Arduino.h"
	#include "HW_SAM_defines.h"
#else
	#include "WProgram.h"
	#include "HW_PIC32_defines.h"
#endif
*/

#define PREC_LOW			1
#define PREC_MEDIUM			2
#define PREC_HI				3
#define PREC_EXTREME		4

// Constants for the GUI
#define BVS_13 10
#define BVS_15 12
#define BVS_19 14
#define BVS_22 18
#define BVS_28 22
#define BVS_34 28
#define BVS_43 38
#define BVS_52 53
#define BVS_74 78
#define BVS_112 122

#define clrRed 0xFF0000
#define clrGreen 0x00FF00
#define clrBlue 0x0000FF
#define clrWhite 0xFFFFFF
#define clrBlack 0x000000
#define clrYellow 0xFFFF00
#define clrCyan 0x00FFFF

#define shapeRectangle 1
#define shapeRectangle_FILL 2
#define shapeLine 3

#define maxobjects 100
#define stateDrawn 1
#define statePressed 2
#define GUI_buttonchar_font BVS_34
#define GUI_singlechar_center_x 18
#define GUI_singleline_center_y 9

#define GUI_Type_void 0
#define GUI_Type_button 1
#define GUI_Type_panel 2
#define GUI_Type_shape 3  //Still to do
#define GUI_Type_image 4
#define GUI_Type_label 5
#define GUI_Type_anaClock 6
#define GUI_Type_digDate 7
#define GUI_Type_digTime 8
#define GUI_Type_checkBox 9
#define GUI_Type_cycleButton 10
#define GUI_Type_TextInput 11
#define GUI_Type_ghostbutton 12
#define GUI_Type_imagebutton 13
#define GUI_Type_cycleStringbutton 14

#define GUI_State_Unpressed false
#define GUI_State_Pressed true

#define GUI_link_INC 1
#define GUI_link_DEC 2
#define GUI_link_ZERO 3

#define GUIURNnull 60000
#define GUIURNcal_x_min 60001
#define GUIURNcal_x_max 60002
#define GUIURNcal_y_min 60003
#define GUIURNcal_y_max 60004
#define GUIURNx_raw 60005
#define GUIURNy_raw 60006
#define GUIURNexit 60007
#define GUIURNsave 60008

#include <UTouch.h>
#include <UTFT.h>

/* Done in UTFT // ediaz
struct _current_font
{
	uint8_t* font;
	uint8_t x_size;
	uint8_t y_size;
	uint8_t offset;
	uint8_t numchars;
};
*/

class DUEGUI
{
	public:
		DUEGUI();
                DUEGUI(UTFT * mylcd, UTouch * mytouch);
		void show_color_bar();
		void SPI_Flash_init(int CS_pin, int Rate);
		void READ_ID();
		void Send_Flash_information_to_UART();
		void Load_image(int X, int Y, int location);
		void Put_Text(String st, int x, int y, int font_number);
		void Put_Text_array(char *st, int x, int y, int font_number);
		void Set_character_spacing(unsigned char space);
		void InitLCD(byte orientation=LANDSCAPE);
		void clrScr();
		void drawPixel(int x, int y);
		void drawLine(int x1, int y1, int x2, int y2);
		void fillScr(byte r, byte g, byte b);
		void drawRect(int x1, int y1, int x2, int y2);
		void drawRoundRect(int x1, int y1, int x2, int y2);
		void fillRect(int x1, int y1, int x2, int y2);
		void fillRoundRect(int x1, int y1, int x2, int y2);
		void drawCircle(int x, int y, int radius);
		void fillCircle(int x, int y, int radius);
		void setColor(byte r, byte g, byte b);
		void setBackColor(byte r, byte g, byte b);
		void print(char *st, int x, int y, int deg=0);
		void print(String st, int x, int y, int deg=0);
		void printNumI(long num, int x, int y, int length=0, char filler=' ');
		void printNumF(double num, byte dec, int x, int y, char divider='.', int length=0, char filler=' ');
		void setFont(uint8_t* font);
		void drawBitmap(int x, int y, int sx, int sy, bitmapdatatype data, int scale=1);
		void drawBitmap(int x, int y, int sx, int sy, bitmapdatatype data, int deg, int rox, int roy);
		void lcdOff();
		void lcdOn();
		void setContrast(char c);
		int  getDisplayXSize();
		int	 getDisplayYSize();
		unsigned char CSpin;
		word	TP_X ,TP_Y;
		void	InitTouch(byte orientation = LANDSCAPE);
		void	read();
		bool	dataAvailable();
		int		getX();
		int		getY();
		void	setPrecision(byte precision);
		void	setReverseX(boolean reversed);
		void	setReverseY(boolean reversed);
		
// GUI functions being added (subject to change)

// Other functions		
		
  void    InitGUI(byte tclk, byte tcs, byte tdin, byte dout, byte irq,int CS_pin, int Rate,int SD_pin);

  void    setColorLong(long newColour);
  void    setBackColorLong(long newColour);
  void    preserveColours();
  void    restoreColours();

  void    updatingScreen();
  void    finishedUpdatingScreen();

  bool    withinBounds(int chkX,int chkY,int X, int Y, int XS, int YS);
  String  truefalse(bool booleanValue);
  String  displayNumFormat(int Num,int len);
  String  IntegerToString(long num, int length, char filler);
  int     calculate_x(int angle,int radius);
  int     calculate_y(int angle,int radius);
  void    drawAngledLine(int x,int y,int angle,int radius);
  void    getImageSize(int imageNumber);

  void    startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
  void    stopTimer(IRQn_Type irq);
  void    restartTimer(IRQn_Type irq);
  
  int     addTextInput(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long inputBoxcolour,long inputtextcolour,byte borwidth,String top,int popup_y,word xo,word yo,String initialstate,int inputlength,int options,int font,bool visible,int URN);
  void    drawTextInputText(int objectNumber,long textColour);
  void    drawTextInput(int buttonnumber);
  void    updateTextInput(int objectNumber,String top);
  
  int     addCheckBox(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,int initialstate,byte options,bool visible,int URN);
  int     addCycleButton(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,int initialstate,int cyclemin,int cyclemax,int cyclestep,int cyclexo,int options,bool visible,int URN);
  int     addButton(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,bool visible,int URN);
  int     addCycleStringButton(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,int initialstate,String *elements,int cyclexo,int options,bool visible,int URN);
  int     addImageButton(word x,word y,int image,int imagepressed,int imagealt,byte size,bool initialState,byte option,bool visible,int URN);
  int     addGhostButton(word x,word y,word xs,word ys,byte option,bool visible,int URN);
  void    drawButton(int buttonnumber);
  bool    checkButton(word buttonnumber);
  int     checkAllButtons();
  
  int     addPanel(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,byte borwidth,String top,int xo,int yo,int font,bool visible,int URN);
  void    drawPanel(int objectnumber);
    
  int     addLabel(word x,word y,long colour,long textcolour,String top,int font,bool visible,int URN);
  void    drawLabel(int objectnumber);
  void    removeLabel(int objectNumber);
  void    updateLabel(int objectNumber,String top);

  int     addShape(word x,word y,long colour,int type,int a,int b,int c,int d,int e,int f,int g,int h,int i,int j,bool visible,int URN);
  void    drawShape(int objectnumber);

  int     addProgressBar(word x,word y,word xs,word ys,word maxvalue,word redvalue,word initialvalue,word options,long colour,long borcolour,long backcolour,long redcolour,long borwidth,bool visible,int URN);
  void    updateProgressBar(int objectNumber,int value);
  void    drawActualProgressBar(int objectNumber);
  void    drawProgressBar(int objectNumber);

  int     addImage(word x,word y,int imageNumber,bool visible,int URN);
  void    drawImage(int objectNumber);

  int     addAnalogueClock(word x,word y,word clocksize,word centresize,long facecolour,long borcolour,long hourcolour,int hourlen,long mincolour,int minlen,long seccolour,int seclen,byte borwidth, int options,bool visible,int URN);
  void    drawAnalogueClock(int objectNumber);
  int     addDigitalClock_Time(word x,word y,long colour,long textcolour,int options,int font,bool visible,int URN);
  void    drawDigitalClock_Time(int objectNumber);
  int     addDigitalClock_Date(word x,word y,long colour,long textcolour,int options,int font,bool visible,int URN);
  void    drawDigitalClock_Date(int objectNumber);
  void    drawHands(int objectNumber);
  void    drawAnalogueClock_dividers(int objectNumber,int cx,int cy,int inner,int outer,int number,bool lines);
  void    setObjectTime(int objectNumber,int hh, int mm, int ss);
  void    setObjectDate(int objectNumber,int yyyy, int month, int day, bool dateNorm, bool dateFour);
  
  void    ClearObjects(int first,int last);
  void    clearAllObjects(); 
  void    redrawAllObjects();
  void	  redrawChangedObjects();
  void    redrawObject(int objectNumber);
  void    clearObjectArea(int objectNumber);
  bool    objectVisible(int objectNumber);
  void    makeObjectInvisible(int objectNumber,bool redraw);
  void    makeObjectVisible(int objectNumber,bool redraw);
  void    drawSingleKey(int keynumber,long capcolour,long textcolour);
  void    makePopUp(int objectNumber,int yoffset);
  void    clearPopUp();
  bool    returnBoolValue(int objectNumber);
  int     returnIntValue(int objectNumber);
  String  returnStringValue(int objectNumber);
  int     findObjectByURN(int URN);
  
  void    HandleShowButtons(int URN);
  void    HandleShowLoop();
  void    showCalibrate();
  
  void    debug(int onoff);
  void    db_varSt(int level,String out,String val);
  void    db_varInt(int level,String out,int val);
  void    db_St(int level,String out);
  
// GUI variables
        
  byte    GUIobject_type[maxobjects];
  int     GUIobject_link[maxobjects];
  byte    GUIobject_linkoption[maxobjects];
  int     GUIobject_UniqueReference[maxobjects];

  int     GUIobject_x[maxobjects];
  int     GUIobject_y[maxobjects];
  int     GUIobject_xs[maxobjects];
  int     GUIobject_ys[maxobjects];
  long    GUIobject_xo[maxobjects];
  long    GUIobject_yo[maxobjects];
  long    GUIobject_colour[maxobjects];
  long    GUIobject_borcolour[maxobjects];
  long    GUIobject_textcolour[maxobjects];
  long    GUIobject_pressedcolour[maxobjects];
  long    GUIobject_pressedtextcolour[maxobjects];
  int     GUIobject_borwidth[maxobjects];
  int     GUIobject_font[maxobjects];
  String  GUIobject_top[maxobjects];
  int     GUIobject_currentstate[maxobjects];
  bool    GUIobject_visible[maxobjects];
  bool    GUIobject_changed[maxobjects];
  int     GUIobject_data1[maxobjects];
  int     GUIobject_data2[maxobjects];
  int     GUIobject_data3[maxobjects];
  int     GUIobject_data4[maxobjects];
  int     GUIobject_data5[maxobjects];
  int     GUIobject_data6[maxobjects];
  String  GUIobject_Stringdata1[maxobjects];
  String  *GUIobject_StringPointer[maxobjects];

  int     GUI_firstObject;
  int     GUI_objects;
  byte    GUI_br,GUI_bg,GUI_bb,GUI_fr,GUI_fg,GUI_Fb;
  
  unsigned long int  GUI_prevTime;
  unsigned long int  GUI_thisTime;


// End of GUI Functions and variables  
        
        
        long	touch_x_left, touch_x_right, touch_y_top, touch_y_bottom;
		long	touch_disp_x_size, touch_disp_y_size;
		byte    set_rs,set_wr,set_rst,set_cs;
		bool	anyClockVisible;
		bool    anyButtonPressed;
		bool    GUI_popupactive;
		int     GUI_popupobject;
		int     GUI_popupkeyoffset;
		int     GUI_popupclearedwith;
		int     GUI_xsize;
		int     GUI_ysize;
                int     GUIpopup_bw;
                int     popup_xs;
                int     popup_ys;
                String  popup_string;
		bool    GUI_ShowCalibrate;
		int    _debugging;
		
        
    private:   
    	byte	T_CLK, T_CS, T_DIN, T_DOUT, T_IRQ;
		long	_default_orientation;
		byte	orient;
		byte	prec;
		byte	display_model;
		long	disp_x_size, disp_y_size;
		long	default_orientation;

		void	touch_WriteData(byte data);
        word	touch_ReadData();
		bool	_revTouchX;
		bool	_revTouchY;
        byte    GUIoldAnaClockHH;
        byte    GUIoldAnaClockMM;
        byte    GUIoldAnaClockSS;
        byte    GUIpopupbuttonpressed;
        int     iXsize,iYsize;	
        
        // variables for Show functions:
        
        int     GUIcal_x_min,GUIcal_x_max,GUIcal_y_min,GUIcal_y_max,GUIx_raw,GUIy_raw,GUIcal_x,GUIcal_y,GUIold_x,GUIold_y,GUIpoint_x,GUIpoint_y;
        unsigned long int GUIval_x_min,GUIval_x_max,GUIval_y_min,GUIval_y_max;
        

	protected:
                UTFT    * mylcd;                            // ediaz 
                UTouch  * mytouch;                          // ediaz
		byte fcolorr,fcolorg,fcolorb;
		byte bcolorr,bcolorg,bcolorb;
		byte display_transfer_mode, display_serial_mode;
		regtype *P_RS, *P_WR, *P_CS, *P_RST, *P_SDA, *P_SCL,* P_F_CS;
		regsize B_RS, B_WR, B_CS, B_RST, B_SDA, B_SCL, B_F_CS;
		_current_font	cfont;
 		void 	SPI_WriteByte(byte data);
 		byte 	SPI_ReadByte();
		void 	_set_direction_registers(byte mode);
		void 	Convertto8bit(char VH,char VL);
		
        void    redrawPopUpButtons(int objectNumber);
        void    drawActualKey(int x,int y,int xs,int ys,String Keytop,long keycolour,long textcolour);
        void    drawCycleValue(int objectNumber,int colour,int backcolour);
        void    drawCycleText(int objectNumber,int colour,int backcolour);
};

static void spiInit(uint8_t spiRate);
static void spiBegin();
static uint8_t spiRec();
static uint8_t spiRec(uint8_t* buf, size_t len);
static void spiSend(uint8_t b);
static void spiSend(const uint8_t* buf, size_t len);  
#endif
