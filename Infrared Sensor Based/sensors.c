#include "ping.h"                             // Incluide ultraSonic header

#define ULTRA_ONE_PIN 9
#define ULTRA_TWO_PIN 10
// #define ULTRA_THREE_PIN 11

static volatile int usReadingCenter;                // for distance measurement
static volatile int usReadingLeft; 
static volatile int usReadingRight; 


//=============================================================================================
// void ultraSonic() 
// ----- This function runs in its own cog and gets the cm distance from the sensors 
// ----- and saves it to usReading.
//=============================================================================================
void ultraSonic() {                         
  while(1){                                 // This is running inside a separte cog so repeat indefinitely
    usReadingCenter = ping_cm(ULTRA_ONE_PIN);                  
    pause(10);
    usReadingLeft = ping_cm(ULTRA_TWO_PIN);                  
    pause(10);
    // usReadingRight = ping_cm(ULTRA_THREE_PIN);                  
    // pause(10);
  }
 }


//=============================================================================================
// int getUSReading_____()
// ----- These function return the cm distance that was saved to one of the usReading by utraSonic().
// ----- This is done so that any cog can call this function and access the US reading without
// ----- having to have access to the usReading variable.
//=============================================================================================
int getUSReadingCenter(){
  
  return usReadingCenter;

}  

int getUSReadingLeft(){
  
  return usReadingLeft;

}  

// int getUSReadingRight(){
  
//   return usReadingRight;

// }  
  