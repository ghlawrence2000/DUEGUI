//

// Setup and initialise TFT and Touch

extern uint8_t SmallFont[];
#include "SPI.h"
#include <UTFT.h>
UTFT TFT1(CTE70,25,26,27,28); 

//
//  Setup for Timer
//
#define ticksPerSecond 20

// 
// Setup for RTC library
//
#include <rtc_clock.h>
RTC_clock rtc_clock(RC);
char* daynames[]={"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
int hh,mm,ss,dow,dd,mon,yyyy;

//Setup variables for GUI
#define maxbuttons 100
#define popupbuttonstart 50
#define stateDrawn 1
#define statePressed 2

int found;
int clockDigital,clockDate,clockAnalogue,background,pnlTitle;
int btnTB1,btnTB2,btnRemove,btnAdd,btnRefresh;

//
//  Interupt setup
//
volatile int l;
void TC3_Handler()
{
  //
  // We don't want timer called whilst it's being handled.
  //
  TFT1.stopTimer(TC3_IRQn);
  //
  // routine is called several times per second ie "ticksPerSecond".
  // l is incremented each time and by checking this against "ticksPerSecond"
  // we can make sure that once a second events are only called once per second.
  //
  l+=1;
  //
  // This is the "once per second" event routine.
  //
  if (l==ticksPerSecond){
    // second passed
    l=0;
    if (TFT1.anyClockVisible) {
      rtc_clock.get_time(&hh,&mm,&ss);
      rtc_clock.get_date(&dow,&yyyy,&mon,&dd);
      TFT1.setObjectTime(clockDigital,hh,mm,ss);
      TFT1.setObjectDate(clockDate,dd,mon,yyyy,true,true);
      TFT1.setObjectTime(clockAnalogue,hh,mm,ss);
      TFT1.redrawObject(clockDigital);
      TFT1.redrawObject(clockDate);
      TFT1.drawHands(clockAnalogue);
    }
  }
  //
  // Ok interupt handler is finished. 
  //
  TC_GetStatus(TC1, 0);
  TFT1.restartTimer(TC3_IRQn);
}

void setup(){ 
  //
  // Initialise serial on the programming port
  //
  Serial.begin(115200);
  Serial.println("\n\nSERIAL CONNECTED AT 115200\n\n");

  //
  // Initialise for RTC
  //
  rtc_clock.init();
  rtc_clock.set_time(__TIME__);
  rtc_clock.set_date(__DATE__);
  
  //
  // Initialise TFT display & Touch, clear screen and select font
  //
  delay(500);
  TFT1.InitLCD();
  TFT1.clrScr();
  TFT1.setFont(SmallFont);
  TFT1.SPI_Flash_init(52,2);

  TFT1.UTouch(6,5,32,3,2);
  TFT1.InitTouch();
  TFT1.setPrecision(PREC_MEDIUM);
 
 // background=TFT1.addImage(0,0,287,true);  
  pnlTitle=TFT1.addPanel(0,0,799,50,0x0000FF,0xFFFFFF,0xFFFFFF,2,"Board   control   menu",280,8,BVS_34,true);
  btnRefresh=TFT1.addButton(350,100,120,50,0x0000FF,0xFFFFFF,0xFFFFFF,0xFF0000,0xFFFFFF,2,"Refresh",20,12,BVS_28,true); 
  btnTB1=TFT1.addButton(50,100,249,50,0x0000FF,0xFFFFFF,0xFFFFFF,0xFF0000,0xFFFFFF,2,"Test button 1",50,12,BVS_28,true); 
  btnTB2=TFT1.addButton(50,175,249,50,0x0000FF,0xFFFFFF,0xFFFFFF,0xFF0000,0xFFFFFF,2,"Test button 2",50,12,BVS_28,true); 
  btnAdd=TFT1.addButton(50,250,249,50,0x00FFFF,0xFFFFFF,0x000000,0xFF0000,0xFFFFFF,2,"Bring back 1&2",50,12,BVS_28,false); 
  btnRemove=TFT1.addButton(50,350,249,50,0x00FF00,0xFFFFFF,0x000000,0xFF0000,0xFFFFFF,2,"Remove  1&2",50,12,BVS_28,true); 
  clockDigital=TFT1.addDigitalClock_Time(681,11,0x0000FF,0xFFFFFF,0,BVS_34,true);
  clockDate   =TFT1.addDigitalClock_Date( 15,11,0x0000FF,0xFFFFFF,0,BVS_34,true);
 
                   //addAnalogueClock(x  ,y  ,clocksize,centresize,facecolour,borcolour,hourcolour,hourlen,mincolour,minlen,seccolour,seclen,borwidth,options,visible){
  clockAnalogue=TFT1.addAnalogueClock(500,225,270      ,15        ,0x000000  ,0xFFFF00 ,0xFF0000  ,80     ,0x00FF00 ,90    ,0xFFFFFF ,100   ,15      ,24     ,true);
    
  TFT1.redrawAllObjects();
  
  //
  // Start timer
  //
  TFT1.startTimer(TC1,0,TC3_IRQn,ticksPerSecond);
  
 }


void loop(){
  //
  //  Main program control loop
  //
  
  found=TFT1.checkAllButtons();
  
  if (found==btnRefresh){
    Serial.println("Refresh");
    TFT1.redrawAllObjects();
  }
  
  if (found==btnTB1){
    Serial.println("btn TB1");
  }
  
  if (found==btnTB2){
    Serial.println("btn TB2");
  }
  
  if (found==btnRemove){
    Serial.println("btn Remove");
    TFT1.makeObjectInvisible(btnRemove,true); 
    TFT1.makeObjectInvisible(btnTB1,true);
    TFT1.makeObjectInvisible(btnTB2,true); 
    TFT1.makeObjectVisible(btnAdd,true); 
  }

  if (found==btnAdd){
    Serial.println("btn Add");
    TFT1.makeObjectInvisible(btnAdd,true); 
    TFT1.makeObjectVisible(btnTB1,true);
    TFT1.makeObjectVisible(btnTB2,true); 
    TFT1.makeObjectVisible(btnRemove,true); 
  }

}


