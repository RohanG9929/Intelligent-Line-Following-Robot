#include "simpletools.h"                      // Include simple tools
#include "servo.h"                            // Include servo header
#include "sensors.h"                          // sensors header for Ultrasonics


//=============================================================================================
// PIN CONNECTIONS
//=============================================================================================
#define LEFT  4
#define RIGHT  6
#define CENTER 5
#define LEFTMOST 7
#define RIGHTMOST 8
#define TAIL 0
#define LEFT_SERVO 16
#define RIGHT_SERVO 17
#define RPI_ENEMY_SIGNAL 3
#define RPI_FRIEND_SIGNAL 2


//=============================================================================================
// OPERATING VARIABLES
//=============================================================================================
#define MOTOR_KP 4.0
static volatile int currentIntersection = 0;
static volatile int numOfObjs = 0;
static volatile int intDetect = 0;

int ob = 0;
int firstB4 = 0;

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

void driveReverse(){

  //1700 (1600)is maximum in one direction and 1300 (1400) is maximum in other direction
  //1500 is stop

  servo_set(LEFT_SERVO,(1475));   //LEFT
  servo_set(RIGHT_SERVO,(1525));  // RIGHT

}

void driveSlowly(){
  servo_set(LEFT_SERVO,(1520));   //LEFT
  servo_set(RIGHT_SERVO,(1480));  // RIGHT 
}  

//=============================================================================================
// void checkFrontObj()
// ----- Function that checks if there is an object in front of the robot within 10 cm
// ----- and returns 1 if there is
//=============================================================================================
int checkFrontObj(){
  
  if(getUSReadingCenter()<5){
    return 1;
  }  else{
    return 0;
  }      
  
} 


//=============================================================================================
// void checkLeftObj()
// ----- Function that checks if there is an object in front of the robot within 10 cm
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
      intDetect = 1;
      return 1;
      
    }else{
      return 0;
    }
}


//=============================================================================================
// int checkEdge(int LorR)
// ----- Function that checks ONLY the right or left or center line sensor value
// ----- this is will be used in the turning functions so time is not wasted computing the 
// ----- other sensor values when not needed
//=============================================================================================
int checkEdge(int LorR){
  
  
      int centInt;
      set_direction(TAIL, 1);
      set_output(TAIL, 1);
      pause(1);
      set_direction(TAIL,0);
      centInt = rc_time(TAIL,1);
      pause(5);
      return centInt;     
    
}    

//=============================================================================================
// checkRPI(int *leftObjCounter)
// ----- Function that checks the signal coming from the RPI
// ----- if the signal is received, the motors stop and the functions returns 1
//=============================================================================================
int checkRPI(int *leftObjCounter){
  
  if( (input(RPI_ENEMY_SIGNAL) || input(RPI_FRIEND_SIGNAL))  && *leftObjCounter==0){
      pause(10);//Just so the bot moves forward ever so slightly
      stopMotors();
      *leftObjCounter = 1;
      //Pausing for 2 seconds to let the Rpi do its thing
      pause(2000); 
      return 1;  

    } else{ 
    return 0;
  }    
  
}  
//=============================================================================================
// void lineFollow(float Kp, int objectInFront)
// ----- This function controls the robot to follow a line until an intersection is detected.
//=============================================================================================
void lineFollow(float Kp, int objectInFront ){

  //Variables local to this function
  int leftObjCounter = 0;
  
  long leftSen;
  long midSen;
  long rightSen;
  long sensorValues[3];
  float stndMtrPwr = 4.0;
  float PosError = 0;
  float one;
  float senPos = 2.50;
  int breakCondition;
  
  if(currentIntersection == 14){
        stndMtrPwr = 15.0;
        
  } else{
        stndMtrPwr = 4.0;
  }        


  //Checking the break condition once before the loop begins
 
      if(objectInFront){  
      
        breakCondition = checkForIntersect() || checkFrontObj();
   
      }else{
        breakCondition = checkForIntersect();
      }     
  
  
  //---------------------------Line Follwoing Loop----------------------------------

    while(!breakCondition){
      //Reading sensor values and applying the power values to the motor
      

    //-----------------------Reading Sensors Values---------------------------------
      lineSensors(sensorValues);
      leftSen = sensorValues[0];
      midSen = sensorValues[1];
      rightSen = sensorValues[2];      
      
      //Updating Motor Power
      drive(stndMtrPwr+Kp*PosError,stndMtrPwr-Kp*PosError);

      
    //-----------------------Computing Error --------------------------------

      if(midSen > 2800){
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


    checkRPI(&leftObjCounter); 

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
// void BotForward()
// ----- This function moves the bot forward so the wheels are over the line
//=============================================================================================
void BotForward(){
    int leftObjCounter = 0;
    drive(5,5);
    
    for(int i =0;i<80;i++){
      pause(10);
      
      if(checkRPI(&leftObjCounter)){
        drive(5,5);
      }        
    }
   // pause(500);
   // stopMotors();
  
}

void BotForwardTurn(){
  
    int thisSpeed = 5;
    int delayCount = 40;
    
    if(currentIntersection == 4){
      thisSpeed = 15;
      delayCount = 20;
    }      
    
    int leftObjCounter = 0;
    drive(thisSpeed,thisSpeed);
    
    for(int i =0;i<delayCount;i++){
      pause(10);
      if(checkRPI(&leftObjCounter)){
        drive(thisSpeed,thisSpeed);
      } 
    }
   // stopMotors();
  
}

void cornerBotFor(){
  
   int leftObjCounter = 0;
    drive(5,5);
    
    for(int i =0;i<40;i++){
      pause(10);
      if(checkRPI(&leftObjCounter)){
        drive(5,5);
      }  
    }
    //pause(200);
   // stopMotors();
  
}  


//=============================================================================================
// void turn180()
// ----- This function turns the robot right until the tail sensor detects black becasue the bot
// ----- is not on an intersection this will result in a 180 turn
//=============================================================================================
void turn180(){
  
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    pause(900);
    
    do{
      servo_set(LEFT_SERVO,(1550));   //LEFT
      servo_set(RIGHT_SERVO,(1550));  // RIGHT
      
      //lineSensors(readings);
      
    } while(checkEdge(3)<2200);
    
     driveReverse();
     pause(150);


}

//=============================================================================================
// void turn180onInt()
// ----- This function turns the robot right until the tail sensor detects black and does the 
// ----- same again. Because the bot is on an intersection this needs to happen twice for the
// ----- bot to turn 180
//=============================================================================================
void turn180onInt(){
  
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    pause(600);
    
    do{
      servo_set(LEFT_SERVO,(1550));   //LEFT
      servo_set(RIGHT_SERVO,(1550));  // RIGHT
      
      //lineSensors(readings);
      
    } while(checkEdge(3)<2200);
    
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    pause(600);
    
    do{
      servo_set(LEFT_SERVO,(1550));   //LEFT
      servo_set(RIGHT_SERVO,(1550));  // RIGHT
      
      //lineSensors(readings);
      
    } while(checkEdge(3)<2200);


}

//=============================================================================================
// void turnRight()
// ----- This function turns the robot right until the middle line sensor detects black
//=============================================================================================
void turnRight(){
    int count = 0;
  
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    pause(500);
    
    do{
      servo_set(LEFT_SERVO,(1550));   //LEFT
      servo_set(RIGHT_SERVO,(1550));  // RIGHT
      
      pause(10);
      if(checkRPI(&count)){
        drive(15,15);
      }  
      
    } while(checkEdge(2)<2200);

    //Small Forward motion after a turn to ensure the line sensor is not on the intersection
    BotForwardTurn();
      
}

//=============================================================================================
// void turnLeft()
// ----- This function turns the robot left until the middle line sensor detects black
//=============================================================================================
void turnLeft(){
  int count = 0;
  
  servo_set(LEFT_SERVO,(1450));   //LEFT
    servo_set(RIGHT_SERVO,(1450));  // RIGHT
    pause(500);

    do{
      servo_set(LEFT_SERVO,(1450));   //LEFT
      servo_set(RIGHT_SERVO,(1450));  // RIGHT
      
      pause(10);
      if(checkRPI(&count)){
        drive(15,15);
      }
      

    } while(checkEdge(1)<2200);

    //Small Forward motion after a turn to ensure the line sensor is not on the intersection
    BotForwardTurn();

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
      BotForward();
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


int intersectionFlag(){
  
  return intDetect;
}  

void SETintersectionFlag(){
  
   intDetect = 0;
} 

//=============================================================================================
// void getToStart()
// ----- This function drives the robot until i1. Following the line the whole time
//=============================================================================================
void getToStart(){
  
    lineFollow(MOTOR_KP,0);//located in functions.c
    //i0
    BotForward();
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
    
  int count = 0;
  while(currentIntersection<5){
    
    lineFollow(MOTOR_KP,1);
     

    //The robot drives until an intersection and then checks if there is an obstacle in front.
    //If there is then it will turn 180 degrees and break thw loop. Else is will line follow to the next intersection
    if(checkFrontObj()){ 
    
      while(getUSReadingCenter()<=9){
        driveReverse();
        checkRPI(&count);

      }

      turn180();

      
      break;  
    }else{
      currentIntersection = currentIntersection +1;
      BotForward();
    }   
      
  }  
  ob = currentIntersection+1;
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
  BotForwardTurn();
  turnRight();            
  multLineFollow(3,1);  // B2->B3->B4
  //BotForwardTurn();
  cornerBotFor();
  turnRight();            
  lineFollow(MOTOR_KP,0);
  currentIntersection = 4; //i4
  BotForwardTurn();//wheels on i4


  //If the obstacle is at i3 then the bot will drive to i3 and then i5 to check for locations
  if(ob==3){
    turnRight();
    lineFollow(MOTOR_KP,1);
    
    int count =0;
    while(getUSReadingCenter()<=9){
        driveReverse();
        checkRPI(&count);  
    }

    turn180();      
    lineFollow(MOTOR_KP,0);
    BotForward();//wheels are on i4
    lineFollow(MOTOR_KP,0);
    BotForwardTurn();
    currentIntersection += 1;//i5
    turn180onInt();
    BotForward();
    lineFollow(MOTOR_KP,0);
    BotForwardTurn();
    currentIntersection -= 1;
    turnLeft();//wheels are on i4
  }

  //If the obstacle is at i2 then the bot will drive to i2 and then i5 to check for locations
  if(ob == 2){
    turnRight();
    lineFollow(MOTOR_KP,0);
    currentIntersection -= 1;//i3
    BotForward();
    lineFollow(MOTOR_KP,1);
    
    int count =0;
    while(getUSReadingCenter()<=9){
        driveReverse();
        checkRPI(&count);    
    }
      
    turn180();
    lineFollow(MOTOR_KP,0);
    BotForward();//i3
    lineFollow(MOTOR_KP,0);
    BotForward();
    currentIntersection += 1;//i4
    lineFollow(MOTOR_KP,0);
    BotForwardTurn();
    currentIntersection += 1;//i5
    turn180onInt(); 
    BotForward();
    lineFollow(MOTOR_KP,0);
    BotForwardTurn();
    currentIntersection -= 1;
    turnLeft();//wheels are on i4
  }

  lineFollow(MOTOR_KP,0);
  currentIntersection = 9; //A4
  BotForwardTurn();
  turnRight();
  multLineFollow(3,-1);  // A3->A2->A1
  //BotForwardTurn();
  cornerBotFor();
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
  BotForwardTurn();
  turnRight();           
  multLineFollow(3,1); //B2->B3->B4
  BotForward();
  lineFollow(MOTOR_KP,0);
  //AT B5 Now 
  currentIntersection = 15;
  BotForward(); 

  stopMotors();
}

//=============================================================================================
// void obsAt2()
// ----- This function drives the robot along a determined path to check all destinations, with
// ----- the knowledge that the obstacle was detected at intersection i2
//=============================================================================================
void obsAt2(){


  lineFollow(MOTOR_KP,0);   
  currentIntersection -= 1;  //i1
  BotForwardTurn();
  turnRight();         
  commonLoop();
  gotoB5();

}

//=============================================================================================
// void obsAt3()
// ----- This function drives the robot along a determined path to check all destinations, with
// ----- the knowledge that the obstacle was detected at intersection i3
//=============================================================================================
void obsAt3(){

  lineFollow(MOTOR_KP,0);   
  currentIntersection -= 1;  //i2
  BotForward();
  lineFollow(MOTOR_KP,0);   
  currentIntersection -= 1;  //i1
  BotForwardTurn();
  turnRight();
  commonLoop();
  gotoB5();
  stopMotors();
}  

//=============================================================================================
// void obsAt5()
// ----- This function drives the robot along a determined path to check all destinations, with
// ----- the knowledge that the obstacle was detected at intersection i5
//=============================================================================================
void obsAt5(){

  multLineFollow(4,-1);
  BotForwardTurn();
  turnRight();
  commonLoop();
  gotoB5();

} 


