#include "simpletools.h"
#include "functions.h"
#define BUZZER 3
#define LED 2

serial *lcd;

const int ON  = 22;
const int CLR = 12;

void printIntLetter(int intersectionNum){

    if(intersectionNum>5 && intersectionNum<10){
      dprint(lcd, " A");
      intersectionNum -= 5;
    }else if(intersectionNum>10){
       dprint(lcd, " B");
       intersectionNum -= 10;
    }else{
       dprint(lcd, " i");
    }
    dprint(lcd, "%d",intersectionNum);
}

//=============================================================================================
// void LCD()
// ----- This function opens serial communication with the Parallax LCD
// ----- Then in a while Loop the function checks for the intersection detected flag, and the
// ----- obstacle in front flag
//=============================================================================================
void LCD(){
  
  lcd = serial_open(12, 12, 0, 9600);
  writeChar(lcd, ON);
  writeChar(lcd, CLR);
  pause(1000);
  
  
  while(1){
      writeChar(lcd, 17); // backlight
    
    //--------------------------------------------------------------------------------------
    //When the front Ultrasonic detects an obstacle, the LCD will display "OBJ IN FRONT"
    if(getFrontLCD()){
      writeChar(lcd, CLR);
      dprint(lcd, "OBSTACLE DETECTED\n");
      pause(1000);
      setfrontLCD(0);
    } else{
      pause(50);
      writeChar(lcd, CLR);
      pause(5);
    }       
    
    //--------------------------------------------------------------------------------------
    //When the line sensors detect an intersection the LCD will display "Intersection"
    if(getIntLCD()){
      high(26);
      high(27);  
      dprint(lcd, "Intersection:");
      writeChar(lcd, 225);        
      pause(500);
      printIntLetter(getIntersect());
      
      pause(500);//turning off the LED's after 1 second but leaving the variable on for objection detection below
      low(26);
      low(27);
      pause(500);
      setIntLCD(0);
    } else{
      pause(50);
      writeChar(lcd, CLR);
      pause(5);
    }

    
  }   

}

//=============================================================================================
// void objectIndication()
// ----- This function checks if
// ----- 1. If the bot is in the A or B lane (by checking the intersection number)
// ----- 2. If the left ultrasonic detects any object within the distance
// ----- 3. If there is not an object at B5, (to avoid double detection when going around the loop)
//            if there is an obj at B5 after the robot completes one loop of the map, then it will 
//            not double detect any object before B5. Once it gets to B5 it checks if this flag is 
//            set == 2, then it does all the other checks and blinks the LED if the obj is detected
// ----- 4. If the bot is on or has just passed an intersection
// ----- If all of these are true then the GREEN LED will blink the BUZZER will play a sound
//=============================================================================================
void objectIndication(){
  
  //--------------------------------------------------------------------------------------
    //When the robot is NOT in the middle lane the left Ultrasonic will check if there is an object
    //If all locations are checked in the common Loop and only 1 object was detected then there will be 
    //an object at B5. This means that the robot needs to drive to B5. While driving to B5 it should 
    //not double detect a previously detected object. Only when it gets to B5 should it check for this 
    while(1){

      //final object.
      if(  (checkLeftObj() && getIntersect()>5) && (isObjatB5()<1 || isObjatB5()==2)  && getIntLCD()){
        //This will only occur when the bot is on an Intersection. 
        //INCREASE THE PAUSE IN LINE 29 TO MAKE THIS WORK BETTER
          addObjCount();
          high(LED);                          // LED GREEN 
          freqout(BUZZER,300,3520);
          freqout(BUZZER,300,3729);
          pause(600);
          low(LED);
          pause(600);  
      } 
    }  
  
}  