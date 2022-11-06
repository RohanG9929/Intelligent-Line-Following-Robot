
/*
-----------------------Hardware Functions-----------------------
----------------------------------------------------------------
----------------------------------------------------------------
----------------------------------------------------------------
*/


void lineSensors(long *readings); // sensor readings function
void lineFollow(float Kp, int objectInFront); // line follow function
int checkForIntersect(); // checkForIntersection function
void drive(float LEFTPower, float RIGHTPower); //drive-bot function
void stopMotors(); //stop-bot function
int checkFrontObj();
int checkLeftObj();
void turnLeft();
void turnRight();
void turn180();
void BotForward();
void multLineFollow(int n, int sign);
int checkRPI(int *leftObjCounter);


/*
------------------------State Functions-------------------------
----------------------------------------------------------------
----------------------------------------------------------------
----------------------------------------------------------------
*/

void SETintersectionFlag();
int intersectionFlag();
void addObjCount();

void getToStart();
int driveUntilObs();
void obsAt2();
void obsAt3();
void obsAt5();
void commonLoop();
void goToB5();

