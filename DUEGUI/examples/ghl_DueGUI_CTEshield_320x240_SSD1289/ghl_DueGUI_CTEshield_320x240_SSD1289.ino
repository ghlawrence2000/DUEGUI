////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Due GUI Object demonstration sketch
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  (c) 2013 Darren Hill (Cowasaki)
//
#define VERSION "0.13"
//
//  This program is not as yet complete.  There are a number of missing features and features not fully implemented.
//  There is also the possibility that anything implemented in this pre-release version may be implemented in a different
//  way in any subsequent version
//
//  Many thanks to those persons that created the original UTFT library - Henning 
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Setup and initialise DUEGUI
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void DueGUI_OnButtonPress(int btnFound);
#include <UTFT.h>
#include <UTouch.h>

#include <DUEGUI.h>
//#include <UTFT_Buttons.h>
//#include <UTFT_tinyFAT.h>
//	extern uint8_t SmallFont[];
//	extern uint8_t BigFont[];
//	extern uint8_t various_symbols[];

UTFT          myGLCD(ITDB32S,25,26,27,28); // CTE Shield!
UTouch        myTouch(6,5,32,3,2);         // CTE Shield!    

DUEGUI DueGUI( &myGLCD , &myTouch); 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Setup for the external RTC library
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#include <rtc_clock.h>
RTC_clock rtc_clock(RC); 
char* daynames[]={
  "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
int hh,mm,ss,dow,dd,mon,yyyy;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                 Setup variables for Due GUI
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// variables for screens etc
int DueGUI_currentlyDisplayedScreen=0;
#define URNnull 0
// Every screen
int clockDigital,clockDate,background,pnlTitle,pnlClk;
//
// Main menu screen
#define main_menu 1
int btnTB1, btnTB2, btnRemove, btnAdd, progressbar, progressbar2;  // we only need the object number for SOME of the objects....
String mm_dropdown1[10] = {
  "Blue","Green","Red","Black","White","Yellow"};
#define URNTB1 1
#define URNTB2 2 
#define URNRemove 3 
#define URNAdd 4
#define URNRefresh 5
#define URNFanScreen 6
#define URNInputScreen 7     
#define URNClockScreen 8
#define URNCalibrate 9
#define URNProgressbar 10
#define URNProgressbar2 11
#define URNghostbutton 12
#define URNimagebutton1 13
#define URNimagebutton2 14
#define URNstringcycle 15
//
// Fan screen
#define fan_screen 2
int btnFBonoff0,btnFBspeed0,btnFBonoff1,btnFBspeed1,btnFBonoff2,btnFBspeed2,btnFBonoff3,btnFBspeed3;
#define URNFBmainmenu 209 
int valFBspeed[] = {
  4000,3000,500,0};
boolean valFBonoff[] = {
  true,false,false,true};
//
// input screen
#define input_screen 3
int inpInput1,inpInput2,inpInput3;
String valInput1="Test 1",valInput2="Test 2",valInput3="Test 2";
#define URNINPinput1 301 
#define URNINPinput2 302 
#define URNINPinput3 303 
#define URNINPmainmenu 309 
//
// clock screen
#define clock_screen 4
int clockAnalogue,clockAnalogue2,clockAnalogue3;
#define URNCLKmainmenu 409 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                 Due GUI Interupt routine and variables
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// fast ticks for Due GUI interupt
volatile int l;
volatile int btnFound;
volatile unsigned long int GUI_InterruptTime;
unsigned long int GUI_ButtonLastPressedTime;
//
//  DueGUI Interupt handler routine
//
void DueGUI_tickHandler(){
  //
  // We don't want timer called whilst it's being handled.
  //
  GUI_InterruptTime=millis();
  DueGUI.stopTimer(DueGUI_timer);
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
    if (DueGUI.anyClockVisible) {
      rtc_clock.get_time(&hh,&mm,&ss);
      rtc_clock.get_date(&dow,&yyyy,&mon,&dd);
      //
      // Insert below the set and redraw clock functions:
      //
      DueGUI.setObjectTime(clockDigital,hh,mm,ss);
      DueGUI.redrawObject(clockDigital);
      DueGUI.setObjectDate(clockDate,dd,mon,yyyy,true,false);
      DueGUI.redrawObject(clockDate);

      if (DueGUI_currentlyDisplayedScreen==main_menu){ 
        DueGUI.updateProgressBar(progressbar,ss);
        DueGUI.updateProgressBar(progressbar2,ss);
      }
      //
      //  The analogue clock is only shown on the clock screen
      if (DueGUI_currentlyDisplayedScreen==clock_screen){  
        DueGUI.setObjectTime(clockAnalogue,hh,mm,ss);
        DueGUI.setObjectTime(clockAnalogue2,((hh+5)%12),mm,ss);
        DueGUI.setObjectTime(clockAnalogue3,((hh+5)%12),((mm+30)%60),ss);
        DueGUI.drawHands(clockAnalogue);
        DueGUI.drawHands(clockAnalogue2);
        DueGUI.drawHands(clockAnalogue3);
      }
    }
  }
  if ((DueGUI.dataAvailable()==true) || (DueGUI.anyButtonPressed==true)){
    if ((GUI_ButtonLastPressedTime+200)<GUI_InterruptTime){
      GUI_ButtonLastPressedTime=GUI_InterruptTime;
      btnFound=DueGUI.checkAllButtons();
      if (btnFound!=-1){
        DueGUI_OnButtonPress(btnFound);
      }
    }
  }
  // Ok interupt handler is finished. 
  //
  TC_GetStatus(TC1, 0);
  DueGUI.restartTimer(DueGUI_timer);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                   DueGUI On button press routine
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void DueGUI_OnButtonPress(int btnFound){
  //
  //  Find the URN of the button that was found.
  //
  int URN=DueGUI.GUIobject_UniqueReference[btnFound];
  if (URN>GUIURNnull){
    DueGUI.HandleShowButtons(URN);
  } 
  else {
    //  Main Menu
    //
    if (URN==URNRefresh){
      DueGUI.redrawAllObjects();
      Serial.println("URNRefresh");
    }
    if (URN==URNTB1){
      DueGUI.makeObjectInvisible(btnTB2, true);
      
    }
    if (URN==URNTB2){
      DueGUI.makeObjectInvisible(btnTB1, true);
      
    }
    if (URN==URNRemove){
      DueGUI.makeObjectInvisible(btnRemove, true); 
      DueGUI.makeObjectInvisible(btnTB1, true);
      DueGUI.makeObjectInvisible(btnTB2, true); 
      DueGUI.makeObjectVisible(btnAdd, true); 
    }
    if (URN==URNAdd){
      DueGUI.makeObjectInvisible(btnAdd, true); 
      DueGUI.makeObjectVisible(btnTB1, true);
      DueGUI.makeObjectVisible(btnTB2, true); 
      DueGUI.makeObjectVisible(btnRemove, true); 
    }
    if (URN==URNFanScreen){
      DueGUI_createScreen(fan_screen);
    }
    if (URN==URNghostbutton){
      DueGUI_createScreen(fan_screen);
    }
    if (URN==URNimagebutton2){
      DueGUI_createScreen(fan_screen);
    }
    if (URN==URNInputScreen){
      DueGUI_createScreen(input_screen);
    }
    if (URN==URNClockScreen){
      DueGUI_createScreen(clock_screen);
    }
    if (URN==URNCalibrate){
      DueGUI.showCalibrate();
    }
    // Fan screen buttons
    //
    if (URN==URNFBmainmenu){
      // Grab the data BEFORE you switch screens.
      //
      // Note the objects were created in order so we know that btnFBonoff1 is 1 more than btnFBonoff0 etc.  We can use this
      // to calculate object values in order to place in loops etc.
      //
      // store the values of the cycBoxes in variables ready to use again later
      for (int i = 0; i < 4; i++){
        valFBspeed[i]=DueGUI.returnIntValue(btnFBspeed0+i);
        valFBonoff[i]=DueGUI.returnBoolValue(btnFBonoff0+i);
      }
      printvariables();
      DueGUI_createScreen(main_menu);
    }
    // Input screen buttons
    //
    if (URN==URNINPmainmenu){
      //
      // Store the values of the input boxes for next time.
      valInput1=DueGUI.returnStringValue(inpInput1);
      valInput2=DueGUI.returnStringValue(inpInput2);
      DueGUI_createScreen(main_menu);
    }
    // Clock screen buttons
    //
    if (URN==URNCLKmainmenu){
      DueGUI_createScreen(main_menu);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                   DueGUI display a new screen
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void DueGUI_createScreen(int screen){
  // First stop interupts whilst we build screen
  DueGUI.stopTimer(DueGUI_timer);
  // Set global variable to record which screen is in view
  DueGUI_currentlyDisplayedScreen=screen; 
  // Clear all the data and the screen
  DueGUI.clearAllObjects();
  DueGUI.clrScr();
  // Setup anything that is on EVERY screen
  // NOTE: pnlTitle's text is left blank as it is filled in by each screens functions.
  pnlTitle=     DueGUI.addPanel(0, 0,319,40,clrBlue,clrWhite,clrWhite,0,"",88-15,12,BVS_34,optVisible,URNnull); 
  pnlClk=       DueGUI.addPanel(0,50,319,40,clrBlue,clrWhite,clrWhite,2,"",280,62,BVS_34,optVisible,URNnull); 
  
  clockDigital= DueGUI.addDigitalClock_Time(15 ,62,clrBlue,clrWhite,0,BVS_34,optVisible,URNnull);
  clockDate=    DueGUI.addDigitalClock_Date(175 ,62,clrBlue,clrWhite,0,BVS_34,optVisible,URNnull);
  if (DueGUI_currentlyDisplayedScreen==main_menu){
    DueGUI.db_St(2, "Setup: main_menu");
    // As a demonstration here we are grabbing the object number for every item and setting the URN value for every button.  As can be seen below
    //  you don't actually need to do this for a lot of buttons because the library handles the updating etc automatically.  Once we get to using the
    //  settings they will be read from the objects using the object number variable.
    //
    // For the buttons we need the object number for TB1, TB2, remove and add because we will be making them visible/invisible whilst
    // running.  We could use the function findObjectByURL(URNRemove); for example but this is down to choice.
    //                   (word x,word y,word xs,word ys,word maxvalue,word redvalue,word initialvalue,word options,long colour,long borcolour,long backcolour,long redcolour,long borwidth,bool visible,int URN) 
    //    progressbar= DueGUI.addProgressBar(375   ,100   ,400    ,30     ,60           ,50           ,ss               ,1           ,clrBlue    ,clrWhite      ,clrBlack       ,clrRed        ,2            ,optVisible  ,URNProgressbar); 
    //    progressbar2=DueGUI.addProgressBar(20    ,100   ,40     ,350    ,60           ,30           ,ss               ,0           ,clrGreen   ,clrWhite      ,clrBlack       ,clrYellow     ,2            ,optVisible  ,URNProgressbar2); 
        DueGUI.addButton(280,100,39,40,clrBlue ,clrWhite ,clrWhite,clrRed,clrWhite,2,"R",posCentre,posCentre,BVS_28,optVisible ,URNRefresh); 
        btnAdd=DueGUI.addButton(0 , 100,80,40,clrCyan ,clrWhite ,clrBlack,clrRed,clrWhite,2,"+1&2",posCentre,posCentre,BVS_28,optVisible, URNAdd); 
        btnRemove=DueGUI.addButton(190 ,100,80,40,clrGreen,clrWhite ,clrBlack,clrRed,clrWhite,2,"-1&2",posCentre,posCentre,BVS_28,optVisible, URNRemove); 
        DueGUI.addButton(0  ,150,150,40,clrBlue ,clrWhite ,clrWhite,clrRed,clrWhite,2,"Fan Scn",posCentre,posCentre,BVS_28,optVisible ,URNFanScreen); 
        DueGUI.addButton(169,150,150,40,clrBlue ,clrWhite ,clrWhite,clrRed,clrWhite,2,"Clk Scn",posCentre,posCentre,BVS_28,optVisible ,URNClockScreen); 
        DueGUI.addButton(0,  199,150,40,clrBlue ,clrWhite ,clrWhite,clrRed,clrWhite,2,"Inp Scn",posCentre,posCentre,BVS_28,optVisible ,URNInputScreen); 
        DueGUI.addButton(169,199,150,40,clrBlue ,clrWhite ,clrWhite,clrRed,clrWhite,2,"Cal scn",posCentre,posCentre,BVS_28,optVisible ,URNCalibrate); 
    DueGUI.addGhostButton(0,0,319,40,0,optVisible,URNghostbutton);
    //    DueGUI.addImageButton(400,200,55,56,57,1,true,0,optVisible,URNimagebutton1);
    //    DueGUI.addImageButton(500,200,58,59, 0,1,true,0,optVisible,URNimagebutton2); 
    btnTB1=DueGUI.addButton(90 ,100,40,40,clrBlue ,clrWhite ,clrWhite,clrRed,clrWhite,2,"1",posCentre,posCentre,BVS_28,optVisible,URNTB1    ); 
    btnTB2=DueGUI.addButton(140 ,100,40,40,clrBlue ,clrWhite ,clrWhite,clrRed,clrWhite,2,"2",posCentre,posCentre,BVS_28,optVisible,URNTB2    ); 
    // For the clocks we DO need to know the object number as this will be required by the time/date setting up functions
    // Change the text in the header panel
    //    DueGUI.addCycleStringButton(350,275,299,50,clrBlue ,clrWhite ,clrWhite,clrRed,clrWhite,2,"Cycle Text",15,posCentre,BVS_28,1,mm_dropdown1,100,1,optVisible,URNstringcycle);
    DueGUI.GUIobject_top[pnlTitle]="Main Menu";
  }
  if (DueGUI_currentlyDisplayedScreen==fan_screen){
    DueGUI.db_St(2,"Setup: fan_screen");
    // NOTE the return value is not being stored for the main menu button...  Unless you need to refer to the object once created you don't actually
    //  have to store it! also the URN isn't needed either for the first 8 buttons because they are handled by the library and no user intervention 
    //  is required so they can be set to the special URNnull value. If we were actually going to use these values now we can get them via the object
    //  number.  This would normally be done as the screen is changed so in this case when the main menu button is pressed.  ie you would get them like
    //  this   fan1onoff=DueGUI.returnBoolValue(btnFBonoff1);
    //  or     fan1speed=DueGUI.returnIntValue(btnFBspeed1);
    //  Note for all these button the value are given as an initial value and then retrieved alter so that as we come and go to this screen the values
    //  are persistent.
    //
    printvariables();
    //addCycleButton(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,int initialstate,int cyclemin,int cyclemax,int cyclestep,int options,bool visible,int URN){
    btnFBspeed0=DueGUI.addCycleButton(300,100,249,50,clrBlue,clrWhite ,clrWhite,clrRed,clrWhite,2,"Speed",20,posCentre,BVS_28,valFBspeed[0],0,9000,500,75,1,optVisible,URNnull); 
    btnFBspeed1=DueGUI.addCycleButton(300,175,249,50,clrBlue,clrWhite ,clrWhite,clrRed,clrWhite,2,"Speed",20,posCentre,BVS_28,valFBspeed[1],0,9000,500,75,1,optVisible,URNnull); 
    btnFBspeed2=DueGUI.addCycleButton(300,250,249,50,clrBlue,clrWhite ,clrWhite,clrRed,clrWhite,2,"Speed",20,posCentre,BVS_28,valFBspeed[2],0,9000,500,75,0,optVisible,URNnull); 
    btnFBspeed3=DueGUI.addCycleButton(300,325,249,100,clrBlue,clrWhite ,clrWhite,clrRed,clrWhite,2,"Speed",20,posCentre,BVS_28,valFBspeed[3],0,9000,500,75,2,optVisible,URNnull); 

    btnFBonoff0=DueGUI.addCheckBox(50,100,199,50,clrBlue,clrWhite ,clrWhite,clrRed,clrWhite,2,"Fan on",20,posCentre,BVS_28,valFBonoff[0],cycCHECKBOX,optVisible,URNnull); 
    btnFBonoff1=DueGUI.addCheckBox(50,175,199,50,clrBlue,clrWhite ,clrWhite,clrRed,clrWhite,2,"Fan on",20,posCentre,BVS_28,valFBonoff[1],cycCHECKBOX,optVisible,URNnull); 
    btnFBonoff2=DueGUI.addCheckBox(50,250,199,50,clrBlue,clrWhite ,clrWhite,clrRed,clrWhite,2,"Fan on",20,posCentre,BVS_28,valFBonoff[2],cycCHECKBOX,optVisible,URNnull); 
    btnFBonoff3=DueGUI.addCheckBox(50,325,199,50,clrBlue,clrWhite ,clrWhite,clrRed,clrWhite,2,"Fan on",20,posCentre,BVS_28,valFBonoff[3],cycCHECKBOX,optVisible,URNnull); 
    DueGUI.addButton(600,400,150,50,clrBlue,clrWhite,clrWhite,clrRed,clrWhite,2,"Main Menu",20,posCentre,BVS_28,optVisible,URNFBmainmenu); 
    // Change the text in the header panel
    DueGUI.GUIobject_top[pnlTitle]="Fan  Screen";
  }
  if (DueGUI_currentlyDisplayedScreen==input_screen){  
    DueGUI.db_St(2,"Setup: input_screen");
    DueGUI.addButton(600,400,150,50,clrBlue,clrWhite,clrWhite,clrRed,clrWhite,2,"Main Menu",20,posCentre,BVS_28,optVisible,URNINPmainmenu); 
    //      int          addTextInput(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long inputboxcolour,long inputtextcolour,byte borwidth,String top,int popup_y,word xo  ,word yo  ,String initialstate,int inputlength,int options,int font,bool visible,int URN);
    inpInput1=DueGUI.addTextInput(100   ,150   ,400    ,80     ,clrBlue    ,clrWhite      ,clrWhite       ,clrBlack           ,clrWhite            ,2            ,"Enter"   ,-0         ,20       ,posCentre,valInput1          ,10             ,0          ,BVS_28  ,optVisible  ,URNINPinput1);
    inpInput2=DueGUI.addTextInput(100   ,350   ,400    ,80     ,clrBlue    ,clrWhite      ,clrWhite       ,clrBlack           ,clrWhite            ,2            ,"Enter 2" ,-200       ,20       ,posCentre,valInput2          ,15             ,0          ,BVS_28  ,optVisible  ,URNINPinput1);
    inpInput3=DueGUI.addTextInput(100   ,75    ,400    ,80     ,clrBlue    ,clrWhite      ,clrWhite       ,clrBlack           ,clrWhite            ,2            ,""        ,0          ,20       ,posCentre,valInput3          ,30             ,1          ,BVS_28  ,optVisible  ,URNINPinput1);
    // Change the text in the header panel
    DueGUI.GUIobject_top[pnlTitle]="Input  Screen";
  }
  if (DueGUI_currentlyDisplayedScreen==clock_screen){  
    DueGUI.db_St(2,"Setup: clock_screen");
    DueGUI.addButton(600,400,150,50,clrBlue,clrWhite,clrWhite,clrRed,clrWhite,2,"Main Menu",20,posCentre,BVS_28,optVisible,URNCLKmainmenu); 
    clockAnalogue=DueGUI.addAnalogueClock(65,200,270,15,clrBlack,clrYellow,clrRed,80,clrGreen,90,clrWhite,100,15,24,optVisible,URNnull);
    clockAnalogue2=DueGUI.addAnalogueClock(465,200,270,15,clrBlack,clrYellow,clrRed,80,clrGreen,90,clrWhite,100,20,2,optVisible,URNnull);
    clockAnalogue3=DueGUI.addAnalogueClock(320,360,150,15,clrBlack,clrYellow,clrBlue,80,clrGreen,90,clrWhite,100,20,2,optVisible,URNnull);
    // Change the text in the header panel
    DueGUI.GUIobject_top[pnlTitle]="Clock  Screen";
  }
  // ok draw all the new objects that are supposed to be visible
  DueGUI.redrawAllObjects();
  // we can restart the interupts now
  DueGUI.restartTimer(DueGUI_timer);
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                   Functions used for the demo only
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void  printvariables(){
  Serial.println();
  for (int i = 0; i < 4; i++){
    Serial.print("Fan ("); 
    Serial.print(i); 
    Serial.print(") is set as - ");
    Serial.print(DueGUI.truefalse(valFBonoff[i]));
    Serial.print(" at speed = "); 
    Serial.println(valFBspeed[i]);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                   DueGUI demo setup() function
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setup(){ 
  // Turn debug mode on at level 5 (normal)
  DueGUI.debug(5);
  // Initialise serial on the programming port (this section is for debugging)....
  Serial.begin(115200);
  DueGUI.db_St(1,"\n\nSERIAL CONNECTED AT 115200\nCompiled at: ");
  DueGUI.db_St(1,__TIME__);
  DueGUI.db_St(1," on: ");
  DueGUI.db_St(1,__DATE__); 
  DueGUI.db_St(1,"\nVersion number: ");
  DueGUI.db_St(1,VERSION);
  DueGUI.db_St(1,"");
  //
  // Initialise for RTC
  rtc_clock.init(); 
  rtc_clock.set_time(__TIME__); 
  rtc_clock.set_date(__DATE__);
  //
  // Initialise DueGUI
  DueGUI.InitGUI(6,5,32,3,2,52,2,51);   //DueGUI.InitGUI(6,5,32,3,2,52,2,51);
  DueGUI.setReverseX(1);
  // Display "main_menu" screen
  DueGUI_createScreen(main_menu);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                   DueGUI demo loop() function
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loop(){
  // Handle loop stuff from library:
  DueGUI.HandleShowLoop(); 
  // Do your things here:
}


