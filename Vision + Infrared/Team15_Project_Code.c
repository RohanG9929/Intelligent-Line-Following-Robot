/*
TEAM15 Propeller Project Code
*/

//LIBRARIES
#include "simpletools.h"                      // Include simple tools

//SELF MADE FILES
#include "functions.h"                        // Header file for functions created
#include "sensors.h"                          // Header file for sensor functions created
#include "outputs.h" 


unsigned int stack[128];
unsigned int stack2[128];

unsigned int stack3[128];



int main(){
  

//Making the indication LEDS low and starting the Camera on RPI
  
  

  // Cog 1
  //So ultrasonics are always checking
  //------------------------------------------------------------------------  
  cogstart(&ultraSonic, NULL, stack, sizeof(stack));//Located in sensors.c
   

  // Cog 1
  //So Propeller is always checking for signal from RPI
  //------------------------------------------------------------------------  
  cogstart(&indicateEorF, NULL, stack2, sizeof(stack2));//Located in outputs.c
   
   
  // Cog 2
  //So Propeller indicates the intersection
  //------------------------------------------------------------------------
  cogstart(&intersectionLED, NULL, stack3, sizeof(stack3));//Located in outputs.c



  // Cog 0 
  //------------------------------------------------------------------------

  getToStart();
  // i1;
  
  
    
  //Driving until the red obstacle and then turning around
  switch(driveUntilObs()) {

    case 1: // obstacle at i2 
      obsAt2();
      break; 
	
    case 2: //obstacle at i3
      obsAt3();
      break;     
      
    case 4: //obstacle at i5
      obsAt5();
      break; 
  }
  
  

stopMotors();

  //Once the B5 has been reached the code will end

} 

