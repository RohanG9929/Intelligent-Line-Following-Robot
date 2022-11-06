#include "simpletools.h"                      // Include simple tools
#include "servo.h"                            // Include servo header
#include "sensors.h"                          // sensors header for Ultrasonics


//=============================================================================================
// PIN CONNECTIONS
//=============================================================================================
#define LEFT  4
#define RIGHT  6
#define CENTER 5
#define LEFT_SERVO 17
#define RIGHT_SERVO 16
#define LEFTMOST 7
#define RIGHTMOST 8


//=============================================================================================
// OPERATING VARIABLES
//=============================================================================================
#define MOTOR_KP 5.0
static volatile int currentIntersection = 0;
static volatile int numOfObjs = 0;
static volatile int intLCD = 0;
static volatile int frontLCD = 0;
static volatile int objAtB5 = 0;


/*
------------------------Hardware Functions----------------------
----------------------------------------------------------------
----------------------------------------------------------------
----------------------------------------------------------------
*/


//=============================================================================================
// float lineSensors(long *readings)
// ----- Function that reads the values from each of the IR sensors and saves them to a 
// ----- long array. This array is passed as a pointer.
//=============================================================================================
void lineSensors(long *readings){
    
    set_direction(CENTER, 1);
    set_output(CENTER, 1);
    pause(1);
    set_direction(CENTER,0);
    readings[1] = rc_time(CENTER,1);
    //print("centerSen = %d\n", readings[1]);
    
    
    set_direction(RIGHT, 1);
    set_output(RIGHT, 1);
    pause(1);
    set_direction(RIGHT,0);
    readings[2] = rc_time(RIGHT,1);
    //print("rightSen = %d\n", readings[2]);


    set_direction(LEFT, 1);
    set_output(LEFT, 1);
    pause(1);
    set_direction(LEFT,0);
    readings[0] = rc_time(LEFT,1);
   // print("leftSen = %d\n", readings[0]);
    pause(5); 

}


//=============================================================================================
// void drive(float LEFTPower, float RIGHTPower)
// ----- Function that moves the continuous servos depending on the power values supplied. 
// ----- 1500 represenents stop, however the values close to 1500 are too slow therefore
// ----- the power values begin incrementing with a starting value of 1500+-25. 
//=============================================================================================
void drive(float LEFTPower, float RIGHTPower){

  //1700 (1600)is maximum in one direction and 1300 (1400) is maximum in other direction
  //1500 is stop

  servo_set(LEFT_SERVO,(1525+(LEFTPower*2.5)));   //LEFT
  servo_set(RIGHT_SERVO,(1475-(RIGHTPower*2.5)));  // RIGHT

}


//=============================================================================================
// void checkFrontObj()
// ----- Function that checks if there is an object in front of the robot within 10 cm
// ----- and returns 1 if there is
//=============================================================================================
int checkFrontObj(){
  
  if(getUSReadingCenter()<35){
    return 1;
  }  else{
    return 0;
  }      
  
} 

//=============================================================================================
// void checkFLeftObj()
// ----- Function that checks if there is an object in left of the robot within 10 cm
// ----- and returns 1 if there is
//=============================================================================================
int checkLeftObj(){
  
  if(getUSReadingLeft()<12){
    return 1;
  }  else{
    return 0;
  }      
  
} 

//=============================================================================================
// void stopMotors()
// ----- Function that stops the continuous servos. 1500 represenents stop in the servo_set
// ----- function
//=============================================================================================
void stopMotors() {
  servo_set(LEFT_SERVO,1500);
  servo_set(RIGHT_SERVO,1500);
}


//=============================================================================================
// void checkForIntersect()
// ----- Function that uses the farthest LEFT and farthest RIGHT sensor on the sensor array
// ----- to check if an intersection has been reached
//=============================================================================================
int checkForIntersect(){
    long leftInt, rightInt;

    set_direction(LEFTMOST, 1);
    set_output(LEFTMOST, 1);
    pause(1);
    set_direction(LEFTMOST,0);
    leftInt = rc_time(LEFTMOST,1);

    set_direction(RIGHTMOST, 1);
    set_output(RIGHTMOST, 1);
    pause(1);
    set_direction(RIGHTMOST,0);
    rightInt = rc_time(RIGHTMOST,1);
    pause(5);

    if(leftInt > 2500 && rightInt > 2500){
      intLCD = 1;//variable for triggering the LCD to display intersection detected
      return 1;
    }else{
      return 0;
    }
}


//=============================================================================================
// void lineFollow(float Kp, int objectInFront)
// ----- This function controls the robot to follow a line until an intersection is detected.
//=============================================================================================
void lineFollow(float Kp, int objectInFront ){

  //Variables local to this function
  long leftSen;
  long midSen;
  long rightSen;
  long sensorValues[3];
  float stndMtrPwr = 10.0;
  float PosError = 0;
  float one;
  float senPos = 2.50;
  int breakCondition;

  //Checking the break condition once before the loop begins
  if(objectInFront){  
  
    breakCondition = checkForIntersect() || checkFrontObj();

  }else{
    breakCondition = checkForIntersect();
  }    
  
  
  //---------------------------Line Follwoing Loop----------------------------------

    while(!breakCondition && numOfObjs != 2){
      //Reading sensor values and applying the power values to the motor

    //-----------------------Reading Sensors Values---------------------------------
      lineSensors(sensorValues);
      leftSen = sensorValues[0];
      //print("leftSen = %0.1f\n", sensorValues[0]);
      midSen = sensorValues[1];
      //print("midSen = %0.1f\n", midSen);
      rightSen = sensorValues[2];
      //print("rightSen = %0.1f\n", rightSen);
      
      
      //Updating Motor Power
      stndMtrPwr = 10.0;
      drive(stndMtrPwr+Kp*PosError,stndMtrPwr-Kp*PosError);

      
    //-----------------------Computing Error --------------------------------

      if(midSen > 2500){
        //If the middle sensor is on the line then the robot needs to just continue straight
        senPos = 2.50;
        //2.5

      }else if (rightSen > leftSen){
        one = rightSen/2500.0;
        senPos = senPos + 2.5*one;
        //RIGHT Sensor is on the line.
        //Robot needs to turn RIGHT

        //5.0
        //Increase LEFT power
        //DECREASE RIGHT power
        
        
      } else if(rightSen < leftSen){
        one = leftSen/2500.0;
        senPos = senPos - 2.5*one;
        //LEFT Sensor is on the line.
        //Robot needs to turn LEFT

        //0
        //Increase RIGHT power
        //Decrease LEFT Power
        
      }
      
      PosError =  senPos - 2.50;
      senPos = 2.5;
      //From -2.50 to +2.5
      //From LEFT side to RIGHT Side

      /*Error is 2.500 if RIGHT side is on line
      Needs to turn RIGHT. 

      Error is -2.500 if LEFT side is on line
      Needs to turn LEFT. 
    
      */

    //-----------------------Break Conditions--------------------------------

    if(objectInFront){  
      breakCondition = checkForIntersect() || checkFrontObj();
    }else{
      breakCondition = checkForIntersect();
    }  

    //When breakCondition is 1, the loop will break and the linefollow function ends
    }

  //Once loop ends, an intersection has been detected. Therefore stop motors
  //stopMotors();
}

//=============================================================================================
// void turnLeft()
// ----- This function turns the robot left until the middle line sensor detects black
//=============================================================================================
void turnLeft(){
  long readings[3];
  if(numOfObjs !=2){
    servo_set(LEFT_SERVO,(1450));   //LEFT
    servo_set(RIGHT_SERVO,(1450));  // RIGHT
    pause(500);

    do{
      servo_set(LEFT_SERVO,(1450));   //LEFT
      servo_set(RIGHT_SERVO,(1450));  // RIGHT
      
      lineSensors(readings);
    } while(readings[1]<2200);



    stopMotors();

  }
}
  
//=============================================================================================
// void turnRight()
// ----- This function turns the robot right until the middle line sensor detects black
//=============================================================================================
void turnRight(){
  long readings[3];
  if (numOfObjs !=2){
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    pause(500);
    
    do{
      servo_set(LEFT_SERVO,(1550));   //LEFT
      servo_set(RIGHT_SERVO,(1550));  // RIGHT
      
      lineSensors(readings);
    } while(readings[1]<2200);


    stopMotors();
  }
}

//=============================================================================================
// void turn180()
// ----- This function turns the robot right until the right line sensor detects black
//=============================================================================================
void turn180(){
  long readings[3];
  if (numOfObjs !=2){
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    pause(500);
    
    do{
      servo_set(LEFT_SERVO,(1550));   //LEFT
      servo_set(RIGHT_SERVO,(1550));  // RIGHT
      
      lineSensors(readings);
    } while(readings[2]<2200);


    stopMotors();
  }
}

//=============================================================================================
// void BotForward()
// ----- This function moves the bot forward so the wheels are over the line
//=============================================================================================
void BotForward(){
  if(numOfObjs!=2){
    drive(20,20);
    pause(450);
   // stopMotors();
  }
}

//=============================================================================================
// void multLineFollow(int n, int sign)
// ----- This function calls the linefollow function multiple times. This is for when the robot
// ----- is on the A or B lanes. n is the number of times to line follow and sign is whether to
// ----- add or subtract to the current intersection counter while driving
//=============================================================================================
void multLineFollow(int n, int sign){

  for (int i= 0; i<n; i++){

    lineFollow(MOTOR_KP,0);         // repeat line follow "n" times
    if (i != n-1){
      drive(20,20);
      pause(300);
    }
    currentIntersection = currentIntersection + sign;
  }
}


/*
------------------------State Functions-------------------------
----------------------------------------------------------------
----------------------------------------------------------------
----------------------------------------------------------------
*/


//=============================================================================================
// Getters and Setters
// ----- These functions allow other cogs to access the variables that are local to the main Cog
//=============================================================================================

//------Intersection Counter
void setIntersect(int intersection){
  currentIntersection = intersection;
}

int getIntersect(){
  return currentIntersection;
} 

//------Object Detected Counter
void addObjCount(){
  numOfObjs += 1;
}

//------Intersection LCD update variable
void setIntLCD(int disp){
  intLCD = disp;
}

int getIntLCD(){
  return intLCD;
} 

//------Front Obstacle LCD update variable
void setfrontLCD(int disp){
  frontLCD = disp;
}

int getFrontLCD(){
  return frontLCD;
} 

//------Flag for if Object is at B5
int isObjatB5(){
  return objAtB5;
} 


//=============================================================================================
// void getToStart()
// ----- This function drives the robot until i1. Following the line the whole time
//=============================================================================================
void getToStart(){
  
     lineFollow(MOTOR_KP,0);//located in functions.c
     //i0
     drive(20,20);
     pause(300);
     //print("START");
     lineFollow(MOTOR_KP,0);
     currentIntersection = 1; 
    //i1 reached  
    BotForward();
    
}  


//=============================================================================================
// int driveUntilObs()
// ----- This function drives the robot until the obstacle in the path. The intersection that
// ----- was reached prior to this obstacle is returned to be checked in the main file.
//=============================================================================================
int driveUntilObs(){
    
  while(currentIntersection<5){

    //The robot drives until an intersection and then checks if there is an obstacle in front.
    //If there is then it will turn accordingly. Else is will line follow to the next intersection
    if(checkFrontObj()){
      frontLCD = 1;//variable for triggering the LCD to display obstacle detected
      switch(currentIntersection){
        case 1://obs at i2
        turnLeft();
        break;

        case 2://obs at i3
        turnRight();
        turnRight();
        break;

        case 4://obs at i5
        turnRight();
        break;
      }
      break;
      
    }else{
      lineFollow(MOTOR_KP,0);
      currentIntersection = currentIntersection +1;
      BotForward();
    }      
  }  
  
  return currentIntersection;
  
} 

//=============================================================================================
// void commonLoop()
// ----- This function drives the robot along a the loop i1 -> B1 -> B4 -> A4 - > A1 -> i1
// ----- starting from i1 and facing B1, with wheels on i1.
//=============================================================================================
void commonLoop(){
  lineFollow(MOTOR_KP,0);
  currentIntersection = 11; //B1
  BotForward();
  turnRight();            
  multLineFollow(3,1);  // B2->B3->B4
  BotForward();
  turnRight();            
  lineFollow(MOTOR_KP,0);
  currentIntersection = 4; //i4
  BotForward();
  lineFollow(MOTOR_KP,0);
  currentIntersection = 9; //A4
  BotForward();
  turnRight();
  multLineFollow(3,-1);  // A3->A2->A1
  BotForward();
  turnRight();
  lineFollow(MOTOR_KP,0);
  currentIntersection = 1;//i1
  stopMotors();
}

//=============================================================================================
// void gotoB5()
// ----- This function drives the robot along to B5
// ----- starting from i1 and facing B1, with wheels on i1.
//=============================================================================================
void gotoB5(){
  BotForward();
  lineFollow(MOTOR_KP,0); // line follow
  currentIntersection = 11; //B1
  BotForward();
  turnRight();           
  multLineFollow(3,1); //B2->B3->B4
  checkForIntersect();

  if(numOfObjs !=2){
    objAtB5 = 1;
    // gotoB5
    BotForward();
    lineFollow(MOTOR_KP,0);
    
    //AT B5 Now 
    currentIntersection = 15;
    BotForward(); 
    objAtB5 = 2;  
  }

  stopMotors();
}

//=============================================================================================
// void obsAt2()
// ----- This function drives the robot along a determined path to check all destinations, with
// ----- the knowledge that the obstacle was detected at intersection i2
//=============================================================================================
void obsAt2(){
           
  commonLoop();
  if (numOfObjs!=2){
    objAtB5 = 1;
    gotoB5();
  }
}

//=============================================================================================
// void obsAt3()
// ----- This function drives the robot along a determined path to check all destinations, with
// ----- the knowledge that the obstacle was detected at intersection i3
//=============================================================================================
void obsAt3(){

  lineFollow(MOTOR_KP,0);   
  currentIntersection = 1;  //i1
  BotForward();
  turnRight();
  commonLoop();
  if (numOfObjs!=2){
    objAtB5 = 1;
    gotoB5();
  }
  stopMotors();
}  

//=============================================================================================
// void obsAt5()
// ----- This function drives the robot along a determined path to check all destinations, with
// ----- the knowledge that the obstacle was detected at intersection i5
//=============================================================================================
void obsAt5(){

  lineFollow(MOTOR_KP,0);
  currentIntersection = 9; // A4
  BotForward();
  turnRight();
  multLineFollow(3,-1); // A4->A3->A1
  BotForward();
  turnRight();
  lineFollow(MOTOR_KP,0); // i1
  currentIntersection = 1;
  gotoB5();  
} 


