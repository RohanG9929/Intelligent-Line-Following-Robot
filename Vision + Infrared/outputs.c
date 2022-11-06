#include "simpletools.h"
#include "functions.h"
#define RPI_ENEMY_SIGNAL 3
#define RPI_FRIEND_SIGNAL 2
#define SERVO_PIN 15

#define FRIEND_LED 13
#define ENEMEY_LED 14



//=============================================================================================
// void indicateEorF()
// ----- This function checks for a signal coming from the Raspberry Pi
// ----- If the friendly signal is HIGH then the green LED will blink
// ----- if the enemey signla is HIGH then the red LED will blink and the servo will actuate
//=============================================================================================
void indicateEorF(){
  servo_angle(SERVO_PIN,0);
  
  while(1){
  //If signal is 1 the camera detected an enemy
    if(input(RPI_ENEMY_SIGNAL)){
       //therefore knock it off with the servo and high LED
      high(ENEMEY_LED);
      servo_angle(SERVO_PIN,900);
      pause(1000);
      servo_angle(SERVO_PIN,0);
      pause(2000); 
    } else if(input(RPI_FRIEND_SIGNAL)){ 
      high(FRIEND_LED);
      pause(2000); 
    }else{
       low(FRIEND_LED);
       low(ENEMEY_LED);
    }        
      
  
  }  
}


//=============================================================================================
// void intersectionLED()
// ----- This function checks for a the intersection flag
// ----- If the flag is HIGH then the on baord LEDS will be set to HIGH for 1 second
//=============================================================================================
void intersectionLED(){
  
  while(1){
    if(intersectionFlag()){
      high(26);
      high(27);
      pause(1000);
      SETintersectionFlag();
      low(26);
      low(27);
    }
  
  }  
   
}      
  