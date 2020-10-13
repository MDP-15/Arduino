#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include <EnableInterrupt.h>
#include <PID_v1.h>
#include <ArduinoSort.h>


/*
 * ==============================
 * Variables declaration
 * ==============================
 */

//Define Encoder Pins
#define LEFT_MOTOR_PIN 3
#define RIGHT_MOTOR_PIN 11

volatile unsigned int flag = 0;
unsigned long StartTimeLeft = 0;
volatile unsigned long EndTimeLeft = 0;
volatile unsigned long TimeWidthLeft = 0;
unsigned long StartTimeRight = 0;
volatile unsigned long EndTimeRight = 0;
volatile unsigned long TimeWidthRight = 0;
double RPM_L = 0;
double RPM_R = 0;
double PID_RPM_R = 0;
double tick_R = 0;                                // To Keep Track of the Number of Ticks for Right md 
double tick_L = 0; 
double ticksDiff=0;

//const double kp = 0, ki =0, kd =0;

double distance_cm; //distance in cm that the robot need to move.

volatile unsigned int leftTick = 0;
volatile unsigned int rightTick = 0;
//ticks parameters for PID
volatile word tick1 = 0;
volatile word tick2 = 0;
word ticks_moved = 0;
double currentTick1, currentTick2, oldTick1, oldTick2;

//Serial buffer for communication
char command[50];
byte count = 0;
double ideal=0;

//Operating states
bool checklist = true;
bool FASTEST_PATH = false;
bool DEBUG = false;
byte delayExplore = 2.5;
byte delayFastestPath = 500;
//For sensors median filter
#define SAMPLE 50
double speed_L=197;
volatile word sensor_reading[SAMPLE]; //for median filtering

//constructors
DualVNH5019MotorShield md;

SharpIR front_right(SharpIR:: GP2Y0A21YK0F, A0); //FRONTFORWARDRIGHT
SharpIR front_center(SharpIR:: GP2Y0A21YK0F, A1); //FRONTFORWARD
SharpIR front_left(SharpIR:: GP2Y0A21YK0F, A2); //FRONTFORWARDLEFT
SharpIR right_front(SharpIR:: GP2Y0A21YK0F, A3); //FRONTRIGHT
SharpIR long_left(SharpIR:: GP2Y0A02YK0F, A5); //FRONTLEFT
SharpIR right_back(SharpIR:: GP2Y0A21YK0F, A4); //BACKRIGHT

//Refer to end of program for explanation on PID
//PID myPID(&tick_R, &PID_RPM_R, &tick_L, kp, ki, kd, DIRECT);
//PID myPID(&tick_R, &PID_RPM_R, &tick_L, kp, ki, kd, DIRECT);
PID myPID(&ticksDiff,&speed_L,&ideal,7,0,0,DIRECT);
//PID myPID(&ticksDiff,&speed_L,&ideal,7,0,0,DIRECT);


void setupPID(){
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-350, 350);
}
/*
 * ==============================
 * Main Program
 * ==============================
 */
void setup() {
  md.init();
  pinMode(RIGHT_MOTOR_PIN, INPUT);
  pinMode(LEFT_MOTOR_PIN, INPUT);
  enableInterrupt(LEFT_MOTOR_PIN, leftMotorTime, RISING);
  enableInterrupt(RIGHT_MOTOR_PIN, rightMotorTime, RISING);
  Serial.begin(115200);
  Serial.setTimeout(50);
  if (DEBUG){
    Serial.println("Connected");
    if(!FASTEST_PATH)
      Serial.println("Exploration mode!");
  }
  StartTimeLeft = micros();
  StartTimeRight = StartTimeLeft;
  //setupPID();

  //init values
  currentTick1 = currentTick2 = oldTick1 = oldTick2 = 0;
}
/*
*This commented segment was for calibration during MDP competition run, 
*it merely helps to check if all the robot functinality works (distance
*from sensors to the blocks is read correctly, rotation and moving of robot 
*is as expected, any deviation due to battery, etc before quarantine for real run.
*
*You may uncomment this segment or just delete it as is not needed
*/


//s
//  
//while (true){
//  int front, left, right,right_front_1, right_back_1;
//  for(int i=SAMPLE; i>0; i--){
//      sensor_reading[i] = right_front.getDistance(true);
//      delay(1);
//    }
//  sortArray(sensor_reading,SAMPLE);
//  right_front_1 = sensor_reading[SAMPLE/2]; 
//
//    for(int i=SAMPLE; i>0; i--){
//      sensor_reading[i] = right_back.getDistance(true);
//      delay(1);
//    }
//
//  sortArray(sensor_reading,SAMPLE);
//  right_back_1 = sensor_reading[SAMPLE/2]; 
//
//    for(int i=SAMPLE; i>0; i--){
//      sensor_reading[i] = long_left.getDistance(true);
//      delay(1);
//    }
//
//  sortArray(sensor_reading,SAMPLE);
//  left = sensor_reading[SAMPLE/2]; 
//  Serial.print(right_front_1);Serial.print(" "); Serial.print(right_back_1); Serial.print(" "); Serial.println(left);
//  }
//Fastest();
//while(true)
//Explore();
//while (true)
//  calibrate_sensor_print();
//  delay(50);
//}
//while(true){
//  move_forward(1);
//    delay(500);
//}


/*
* Main loop of the robot is repeated infinitely, the algo should be to get 
* commands from the RPi, process them and repeat the process.
*/
void loop() {

//checklist_45();
//Serial.println(distance_long_left());
//  right_wall_calibrate();
//  move_forward(1);
//  delay(500);
//  move_forward(1);
//  delay(500);
//  rotate_right(90, 200);
//  delay(500);
//  move_forward(1);
//  delay(500);
//  move_forward(1);
//  delay(500);
//  rotate_right (90, 200);
//  delay(500);
//  rotate_left(90, 200);
//  delay(500);
//    rotate_left(90, 200);
//  delay(500);
//    rotate_left(90, 200);
//  delay(500); 
////delay(500);
//  rotate_right(90, 200);
//  delay(500);
//  rotate_right(90, 200);
//  delay(500);
//    rotate_right(90, 200);
//  delay(500);
//    rotate_right(90, 200);
//  delay(500);
//      rotate_right(90,200);
//      delay(10);
//      front_calibrate();
//      delay(20);
//      rotate_left(90,200);
//      delay(20);
//front_calibrate();
//move_forward(4);
//delay(100);
//right_wall_calibrate();
//angle_calibrate();
//rotate_right(90,200);
//Drain();
//while(1){
//  
//}
  
  get_command();

  //Debug to see all commands collected.
//  if(DEBUG){
//    print_all_commands();
//  }
  count=0;
  while (command[count] != '\0'){
    //while not end of string
    switch(command[count]){
      case 'F':
      case 'f':
        //move robot forward with stipulated distance unit (blocks)
        switch(command[count+1]){
          //check how many blocks to move
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            if(DEBUG){
              //Serial.print("Moving Forward by ");
              //Serial.print(command[count+1]);
              //Serial.println(" units"); 
            }
            delay(3);
            move_forward(byte(command[count+1])-48); //moving forward by amount stipulcated.
//            for (int i = 0; i< command[count+1]-48; i++){
//              move_forward(1);
//           }
            //offset -48 is to convert ASCII to blocks distance (cuz 0 = 48 for ascii decimal and so on so forth)
            break;
            
          default: 
            if(DEBUG) {
                Serial.println("S's ERROR INVALID COMMAND: "); 
                Serial.println(command[count]); 
             }
             break;
             
            }
        Serial.println("MC");
        count++;
        break;
        
      case 'L':
      case 'l':
        //rotate robot to the left 90 degrees
        if (DEBUG){Serial.println("Rotating Left by 90 degrees");}
        delay(3);
        rotate_left(90, 200);
        Serial.println("MC");
        break;
        
      case 'R':
      case 'r':
        //rotate robot to the right 90 degrees
        if (DEBUG){Serial.println("Rotating Right by 90 degrees");}
        delay(3);
        rotate_right(90, 200);
        Serial.println("MC");
        break;

      case 'H':
      case 'h':
        //calibrate to right wall hug
        if (DEBUG){Serial.println("Right Wall Calibration");}
        delay(3);
        right_wall_calibrate();
        //
      case 'A':
      case 'a':
        //calibrate angle
        if (DEBUG){Serial.println("Angle Calibrating");}
        delay(3);
        angle_calibrate();
        //Serial.println("CC");
        break;

      case 'S':
      case 's':
      
        read_all_sensors(15);
        delay(10);
        break;

      case 'E':
      case 'e':
        if (DEBUG){
          Serial.println("Ending and Realigning");
        }
        Serial.println("EC");
        break;

      case 'Z':
      case 'z':
        //enable flag for fastest path
        FASTEST_PATH = true;
        if(DEBUG)
          Serial.println("Get Ready Boiz");
        delay(3);
        break;
        
      case 'O':
      case 'o':
        //oa_forward(front_center.getDistance());
        break;
        
      default: //by default means error command
        if(DEBUG) {
          Serial.print("ERROR INVALID COMMAND: "); 
          Serial.println(command[count]); 
        }
        break;
    }
    count++;
  }
  command[0] = '\0';
}

/*
 * =======================================================
 * Motion Controls
 * Methods to move the robot straight, left, right, etc...
 * =======================================================
 */

 //A method to rotate robot to the right by a degree. Using 360 degree as a base line
void rotate_right(float degree,int speed) { 
  PID_RPM_R = 0;
  int speed_R = speed;
  int Ticks = getDegreeTicksR(degree);

  initMove();
  initStart();
  md.setSpeeds(speed, -speed);
      
  while (tick_R <= Ticks || tick_L <= Ticks ) {
    speed_R += (-1)* PID_RPM_R;
    md.setSpeeds(speed, -speed_R);
  }
  initEnd(); 
  delay(10);                                                 
}

void rotate_left(float degree, int speed) { 
  PID_RPM_R = 0;
  int speed_R = speed;
  int Ticks = getDegreeTicksL(degree);

//  Serial.println(Ticks);
  initMove();
  initStart();
  md.setSpeeds(-speed, speed);
  
  while (tick_R <= Ticks || tick_L <= Ticks ) {
    
    speed_R += PID_RPM_R;
    md.setSpeeds(-speed, speed_R);
  }
  initEnd(); 
  delay(10);                                                 
}

//A method to move robot forward by distance/unit of square
void move_forward(double distance){
  PID_RPM_R = 0; 
  int MoveDist = 10 *distance;
  int Ticks = getTicks(MoveDist);
  ticksDiff=0;
  initMove();
  initStart();
  float speed_R=206;
  speed_L=193;
//  for(int i=0;i<200;i+=20){
//    md.setSpeeds(i-2,i);
//    delay(30);
//  }
//  double remaining_r = Ticks-tick_R;
//  double remaining_l = Ticks-tick_L;
  md.setSpeeds(193,206);
  myPID.SetSampleTime(6.5);
  while (tick_R <= Ticks || tick_L <= Ticks ) {
//    myPID.Compute();
////    speed_R += (1)*PID_RPM_R;
////    speed_L -= (1)*PID_RPM_R;
//    md.setSpeeds(speed_L, speed_R);
            ticksDiff = tick_L - tick_R;
            myPID.Compute();
            md.setM1Speed(speed_L);
  }
  initEnd(); 
  delay(10);
}

void initMove() {                             
  tick_R = 0;                                    
  tick_L = 0;                                
  RPM_L = 0;
  RPM_R = 0;
}

void initStart() {                  
  md.setSpeeds(0, 0);
  md.setBrakes(0, 0);
}

void initEnd() {                     
  md.setSpeeds(0, 0);
  md.setBrakes(400, 400);
  delay(20);
}

int getDegreeTicksR(float degree){                // Function return ticks required for specified degree
  if(degree<170){return ceil(degree * 4.3);}
  else if(degree<200){return ceil(degree * 4.5);}
  else if(degree<=370){return ceil(degree * 4.6);}
  else if(degree<=450){return ceil(degree * 4.5);}
  else if(degree<=720){return ceil(degree * 4.6);}
  else if(degree<=810){return ceil(degree * 4.55);}
  else if(degree<=1080){return ceil(degree * 4.6);}
}

int getDegreeTicksL(float degree){                // Function return ticks required for specified degree
  if(degree<=135){return ceil(degree * 4.35);}
  else if(degree<=180){return ceil(degree * 4.5);}
  else if(degree<=360){return ceil(degree * 4.6);}
  else if(degree<=540){return ceil(degree * 4.6);}
  else if(degree<=630){return ceil(degree * 4.65);}
  else if(degree<=720){return ceil(degree * 4.6);}
  else if(degree<=810){return ceil(degree * 4.58);}
  else if(degree<=900){return ceil(degree * 4.6);}
  else if(degree<=1080){return ceil(degree * 4.6);}
}
int getTicks(double cm){  
  if(cm==10){// Function return ticks required for specified distance
  return ceil(cm * 28);}
  else if(cm==20){return ceil(cm * 28.5);}
  else if(cm==30){return ceil(cm * 28.7);}
  else if(cm==40){return ceil(cm * 29);}
  else if(cm<=100){return ceil(cm * 28.5);}
}


/*
 * ==============================================================
 * Sensors
 * Methods dealing with data acquisition and processing of sensor
 * All of them use median filtering for the sensor readings
 * This is highly dependent on your sensor placements on the robot.
 * ==============================================================
 */

int new_short_front_center(){
  int distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
      sensor_reading[i] = front_center.getDistance(true);
      delay(1);
    }

  sortArray(sensor_reading,SAMPLE);
  distance_cm = sensor_reading[SAMPLE/2];
//  if(distance_cm==9){ return 0;}
  return distance_cm-5;
  
}

int new_short_front_left(){
  int distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
      sensor_reading[i] = front_left.getDistance(true);
      delay(1);
    }

  sortArray(sensor_reading,SAMPLE);
  distance_cm = sensor_reading[SAMPLE/2];
 // if(distance_cm==9){ return -1;}
  return distance_cm-5;
  
}

int new_short_front_right(){
  int distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
      sensor_reading[i] = front_right.getDistance(true);
      delay(1);
    }

  sortArray(sensor_reading,SAMPLE);
  distance_cm = sensor_reading[SAMPLE/2];
  //if(distance_cm==9){ return -1;}
  return distance_cm-5;
  
}

int new_short_right_front(){
  int distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
      sensor_reading[i] = right_front.getDistance(true);
      delay(1);
    }

  sortArray(sensor_reading,SAMPLE);
  distance_cm = sensor_reading[SAMPLE/2];
  //if(distance_cm==9){ return -1;}
  return distance_cm-7;
  
}

int new_short_right_back(){
  int distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
      sensor_reading[i] = right_back.getDistance(true);
      delay(1);
    }

  sortArray(sensor_reading,SAMPLE);
  distance_cm = sensor_reading[SAMPLE/2];
  //if(distance_cm==9){ return -1;}
  return distance_cm-5;
  
}


int new_long_left(){
  int distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
      sensor_reading[i] = long_left.getDistance(true);
      delay(1);
    }

  sortArray(sensor_reading,SAMPLE);
  distance_cm = sensor_reading[SAMPLE/2];
  //if(distance_cm==19){ return -1;}
  return distance_cm-12;
  
}

/*
 //Methods to return blocks unit from obstacles
 //I changed the default condition to 3, according to what marcus wanted
 //need to find the sweet spot for calibration
 //issue is when obstacle is between 6-16cm from board, it returns 1 although it might be directly in front
 //result = block is printed one block further
*/

char distance_short_front_center(){
  byte distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
      sensor_reading[i] = front_center.getDistance(true);
      delay(1);
    }

  sortArray(sensor_reading,SAMPLE);
    
  //  sensor_distance_cm[0] = sensor_reading[SAMPLE/2];
  //  Serial.println(sensor_distance_cm[0]);
  
  distance_cm = sensor_reading[SAMPLE/2];
  return distance_cm;
  
}


char distance_short_front_left(){
  byte distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
      sensor_reading[i] = front_left.getDistance(true);
      delay(1);
    }
  sortArray(sensor_reading,SAMPLE);
//  sensor_distance_cm[1] = sensor_reading[SAMPLE/2];
//  if (DEBUG)
//  Serial.println(sensor_distance_cm[1]);

  distance_cm = sensor_reading[SAMPLE/2];
  return distance_cm;
}

char distance_short_front_right(){
  byte distance_cm = 0;
  
  for(byte i=SAMPLE; i>0; i--){
    sensor_reading[i] = front_right.getDistance(true);
    delay(1);
  }  

  sortArray(sensor_reading,SAMPLE);

  //sensor_distance_cm[2] = sensor_reading[SAMPLE/2];
  //if (DEBUG)
  //Serial.println(sensor_distance_cm[2]);

  distance_cm = sensor_reading[SAMPLE/2];
  return distance_cm;
  
  

}

char distance_short_right_front(){
  byte distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
    sensor_reading[i] = right_front.getDistance(true);
    delay(1);
  }

  sortArray(sensor_reading,SAMPLE);

  //sensor_distance_cm[3] = sensor_reading[SAMPLE/2];
  distance_cm = sensor_reading[SAMPLE/2];
  return distance_cm;
  
   
}

char distance_short_right_back(){
  byte distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
    sensor_reading[i] = right_back.getDistance(true);
    delay(1);
  }

  sortArray(sensor_reading,SAMPLE);

//  sensor_distance_cm[4] = sensor_reading[SAMPLE/2];
//
//  if (DEBUG)
//  Serial.println(sensor_distance_cm[4]);

  distance_cm = sensor_reading[SAMPLE/2];
  return distance_cm;
}

char distance_long_left(){
  byte distance_cm = 0;
  for(byte i=SAMPLE; i>0; i--){
    sensor_reading[i] = long_left.getDistance(true);
    delay(1);
  }

  sortArray(sensor_reading,SAMPLE);

//  sensor_distance_cm[5] = sensor_reading[SAMPLE/2];
//
//  if (DEBUG)
//  Serial.println(sensor_distance_cm[5]);

  distance_cm = sensor_reading[SAMPLE/2];
  return distance_cm;
  
}

void read_all_sensors(byte delay_time){
  delay(delay_time);
  int sensor1,sensor2,sensor3,sensor4,sensor5,sensor6;
  sensor1 = distance_short_front_center();
  sensor2 = distance_short_front_left();
  sensor3 = distance_short_front_right();
  sensor4 = distance_short_right_front();
  sensor5 = distance_short_right_back();
  sensor6 = distance_long_left();
  //Serial.print("SD|");
  Serial.print(sensor4); Serial.print(";");
  Serial.print(sensor5); Serial.print(";");
  Serial.print(sensor3); Serial.print(";");
  Serial.print(sensor1); Serial.print(";");
  Serial.print(sensor2); Serial.print(";");
  Serial.println(sensor6);
  
}

void calibrate_sensor_print(){
  //method to print out calibrate sensor value
  double difference = 0;
  double distance_front = 0;
  double distance_back = 0;
  for (byte j = 10; j>0; j--){
      distance_front += right_front.getDistance(true);
      distance_back += right_back.getDistance(true);
    }
  distance_front /= 10;
  distance_back /= 10;
  difference = distance_front - distance_back;
  Serial.print("distance_front, distance_back, difference: ");
  Serial.print(distance_front);Serial.print(";");
  Serial.print(distance_back);Serial.print(";");
  Serial.print(difference);Serial.println();
}




//+++++++++++++++++++++++++++++++++++++++++

//Methods for detect object in front of sensors, for clearance purpose 

//++++++++++++++++++++++++++++


bool has_obstacle_front_center(){
  if (distance_short_front_center() == 0 && distance_short_front_center()!=3 )
    return true;
  else
    return false;
}

bool has_obstacle_front_left(){
  if (distance_short_front_left() != 3)
    return true;
  else
    return false;
}

bool has_obstacle_front_right(){
  if (distance_short_front_right() != 3)
    return true;
  else
    return false;
}

bool has_obstacle_right_front(){
  if (distance_short_right_front() ==3)
    return false;
  else
    return true;
}

bool has_obstacle_right_back(){
  if (distance_short_right_back() ==3)
    return false;
  else
    return true;
}

//bool has_obstacle_long_left(){
//  if (distance_long_left() == 0)
//    return true;
//  else
//    return false;
//}
//+++++++++++++++++++++++++++++++++++++++++
/*
 * ==========================================================================
 * Calibration
 * Methods to align wall, calibrate by front sensors, for turning at corner
 * ==========================================================================
 */

//Method for right wall alignment
void right_wall_calibrate(){
  double difference = 0;
  double distance_front = 0;
  double distance_back = 0;
  boolean distance_calibrate_only = false;
  byte i = 100; //changed from 80 to 100, for more angle calibration

  //Serial.println("ENTERING FUNCTIONN!!!");
  //calibrate distance first by front calibrate
  if (!has_obstacle_right_front() && !has_obstacle_right_back()){
    if (DEBUG) Serial.println("No Wall");
    return;
  }
  else if(!has_obstacle_right_front() || !has_obstacle_right_back()){
    if (distance_short_right_front() !=  distance_short_right_back()){
        distance_calibrate_only = true;
        i = 0;
    }
    
    
  }
  //Serial.println(distance_calibrate_only);

   //otherwise only need to calibrate angle   
if (DEBUG){ 
    Serial.println("Right Wall Angle Calibration");
    Serial.println("Distance calibrate only");
    Serial.println(distance_calibrate_only);
  }
  
  while(i > 0 && distance_calibrate_only == false){
    for (byte j = 15; j>0; j--){
      distance_front += right_front.getDistance(true);
      distance_back += right_back.getDistance(true);
    }
    distance_front /= 15;
    distance_back /= 15;
    //Serial.print("distance_front: ");
      //Serial.print(distance_front);
      //Serial.print("distance_back: ");
      //Serial.println(distance_back);
    if(distance_front < 10 || distance_back < 10){
      //Serial.println("EXITING,TOO CLOSE");
      break;
    }
    
    difference = (distance_front - 2 - distance_back); //distance_front + 0.95  - distance_back
    
    if(DEBUG){
      Serial.println("HELLOOOOO");
      Serial.println(" ");
      Serial.print("distance_front: ");
      Serial.print(distance_front);
      Serial.print("distance_back: ");
      Serial.println(distance_back);
      Serial.print("difference");
      Serial.println(difference);
    }
    delay(1);
    i--;                                          
    

    if(difference >= 0.01){ //&& distance_back < 25){ //If the robot tilts to the left
       
      md.setSpeeds(rpm_to_speed_1(100),rpm_to_speed_2(-100));
      delay(2.5);
      md.setBrakes(400,400);
    }
    
    else if(difference <= -0.01){//&& distance_back  < 25){ //If the robot tilts to the right
      md.setSpeeds(rpm_to_speed_1(-100),rpm_to_speed_2(100));
      delay(2.5);
      md.setBrakes(400,400);
    }
    
    else{ // If difference is in between 0.035 to -0.035
      break;
    }
  }
  md.setBrakes(100,100);
  if(DEBUG) Serial.println("Done Side calibration");
  
  delay(10);
  // Se
  
  if (DEBUG) Serial.println("Right Wall distance calibration");
  for (byte j = 10; j>0; j--){
      distance_front += right_front.getDistance(true);
      distance_back += right_back.getDistance(true);
    }
  distance_front /= 10;
  distance_back /= 10;

  if(distance_front <12 || distance_back <10.5){ 
    //If robot is too close to the rightwall
      rotate_right(90,200);
      delay(10);
      front_calibrate();
      delay(20);
      rotate_left(90,200);
      delay(20);
    }

   else if (distance_front > 12 || distance_back > 10.5){
      rotate_right(90,200);
      delay(10);
      front_calibrate();
      delay(20);
      rotate_left(90,200);
      delay(20);
   }

   i=100;
   //otherwise only need to calibrate angle   
  if (DEBUG){ 
    Serial.println("Right Wall Angle Calibration");
    Serial.println("Distance calibrate only");
    Serial.println(distance_calibrate_only);
  }
  while(i > 0 && distance_calibrate_only == false){
    
    for (byte j = 15; j>0; j--){
      distance_front += right_front.getDistance(true);
      distance_back += right_back.getDistance(true);
    }
    
    distance_front /= 15;
    distance_back /= 15;
    
    difference = (distance_front - 2 - distance_back); //distance_front + 0.95  - distance_back
    
    if(DEBUG){
      Serial.println(" ");
      Serial.print("distance_front: ");
      Serial.print(distance_front);
      Serial.print("distance_back: ");
      Serial.println(distance_back);
      Serial.print("difference");
      Serial.println(difference);
    }
    delay(1);
    i--;                                          
    

    if(difference >= 0.01){ //&& distance_back < 25){ //If the robot tilts to the left
      //Serial.println("Entering difference>=0.01");
      md.setSpeeds(rpm_to_speed_1(100),rpm_to_speed_2(-100));
      delay(2.5);
      md.setBrakes(400,400);
    }
    
    else if(difference <= -0.01){//&& distance_back  < 25){ //If the robot tilts to the right
      //Serial.println("Entering difference<=0.01");
      md.setSpeeds(rpm_to_speed_1(-100),rpm_to_speed_2(100));
      delay(2.5);
      md.setBrakes(400,400);
    }
    
    else{ // If difference is in between 0.035 to -0.035
      break;
    }
  }
  md.setBrakes(100,100);
  if(DEBUG) Serial.println("Done Side calibration");
  delay(10);
  
}

//Method to calibrate using front sensors
void front_calibrate(){
  double distance_left = 0;
  double distance_right = 0;
  double difference = 0;
  double ideal = 10.5;
  byte k = 50;
  byte j = 4;
  bool only_distance_calibrate = false;

  if (!has_obstacle_front_left() && !has_obstacle_front_right())
    return;
  
  else if(!has_obstacle_front_left() || !has_obstacle_front_right()){
      //Only calibrate distance if there is an empty block in front on either side
      if(DEBUG) Serial.println("Not enough objects, distance calibration kickinng in!");
      only_distance_calibrate = true;
      k = 0;
   }
  else {//when both has obstacle in front, check how far is the obstacle
      if (distance_short_front_center() != distance_short_front_left() ||distance_short_front_center() != distance_short_front_right()){ 
          only_distance_calibrate = true;
          k = 0;
      } 
  }
   
  if(DEBUG)
    Serial.println("Front calibrating");
    
  while (j>0){
    while (k>0 && only_distance_calibrate == false){
      //calibrate in term of angles first
  
      //Step1: Collecting Data
      for (byte i = 10; i>0; i--){
        distance_left += front_left.getDistance(true);
        distance_right += front_right.getDistance(true);
      }
      distance_left /= 10;
      distance_right /= 10;
  
      //Step 2: Calculate the difference
      difference  = distance_left - (distance_right);
  
      //Some debug prints
      if (DEBUG){
        Serial.print(distance_left);
        Serial.print("|");
        Serial.print(distance_right);
        Serial.print("|");
        Serial.println(difference);
      }
      k--;
      delay(1);
      //calibrate the angle by rotate left/right
      if (difference > 0.03){
        //k++;
        md.setSpeeds(rpm_to_speed_1(100),rpm_to_speed_2(-100));
        delay(2.5);
        md.setBrakes(400,400);
      }
      else if(difference < -0.03){
        //k++;
        md.setSpeeds(rpm_to_speed_1(-100),rpm_to_speed_2(100));
        delay(2.5);
        md.setBrakes(400,400);
      }
  
      else
        break;
    }
    
//    Serial.print("K: ");
//    Serial.println(k);
//    Serial.print("J: ");
//    Serial.println(j);
//    
    if(DEBUG && only_distance_calibrate == false)
      Serial.println("Done calibrating angle front");
    
    k = 50;
    
    //calibrating distance
    while (k>0){
      //collect data
      for (byte i = 10; i>0; i--){
          distance_left += front_left.getDistance(true);
          distance_right += front_right.getDistance(true);
          }
      distance_left /= 10;
      distance_right /= 10;
      if (DEBUG){
        Serial.print(distance_left);
        Serial.print("|");
        Serial.println(distance_right);
      }  
      k--;
      if (distance_left < ideal || distance_right < ideal){
        md.setSpeeds(rpm_to_speed_1(-100),rpm_to_speed_2(-100));
        delay(2.5);
        md.setBrakes(200,200);
      }
  
      else if (distance_left > ideal || distance_right > ideal)
      {
        md.setSpeeds(rpm_to_speed_1(100),rpm_to_speed_2(100));
        delay(2.5);
        md.setBrakes(200,200);
      }
      else
        break;
  
  
    }
    if (DEBUG){
      Serial.println("Done with distance front calibration");
    }
    j--;
    k=15;
//
//    Serial.print("K: ");
//    Serial.println(k);
//    Serial.print("J: ");
//    Serial.println(j);
  }

//  Serial.print("K: ");
//  Serial.println(k);
//  Serial.print("J: ");
//  Serial.println(j);
  if (DEBUG){
    Serial.println("Done front calibration!");
  }
  delay(10);
    
}      

void angle_calibrate(){
    double difference = 0;
  double distance_front = 0;
  double distance_back = 0;
  boolean distance_calibrate_only = false;
  byte i = 100;//changed from 80 to 100, for more angle calibration

  while(i > 0 && distance_calibrate_only == false){
    
    for (byte j = 15; j>0; j--){
      distance_front += right_front.getDistance(true);
      distance_back += right_back.getDistance(true);
    }
    
    distance_front /= 15;
    distance_back /= 15;
    
    if(distance_front < 10 || distance_back < 10){
      //Serial.println("EXITING,TOO CLOSE");
      break;
    }
    
    difference = (distance_front - 2 - distance_back); //distance_front + 0.95  - distance_back
    
    if(DEBUG){
      Serial.println(" ");
      Serial.print("distance_front: ");
      Serial.print(distance_front);
      Serial.print("distance_back: ");
      Serial.println(distance_back);
      Serial.print("difference");
      Serial.println(difference);
    }
    delay(1);
    i--;                                          
    

    if(difference >= 0.01){ //&& distance_back < 25){ //If the robot tilts to the left
      //Serial.println("Entering difference>=0.01");
      md.setSpeeds(rpm_to_speed_1(100),rpm_to_speed_2(-100));
      delay(2.5);
      md.setBrakes(400,400);
    }
    
    else if(difference <= -0.01){//&& distance_back  < 25){ //If the robot tilts to the right
      //Serial.println("Entering difference<=0.01");
      md.setSpeeds(rpm_to_speed_1(-100),rpm_to_speed_2(100));
      delay(2.5);
      md.setBrakes(400,400);
    }
    
    else{ // If difference is in between 0.035 to -0.035
      break;
    }
  }
  md.setBrakes(100,100);
  if(DEBUG) Serial.println("Done Side calibration");
  delay(10);
  
  
}

//void obstacle_detected(int dir){
//  Serial.println("ENTERED FUNC");
//  if(dir==1){
//    int count = 0;
//    while(has_obstacle_right_back() || has_obstacle_right_front()){
//      Serial.print("MOVE FORWARD:");
//      
//      Serial.println(count+1);
//      move_forward(1);
//    }
//    Serial.println("TURN RIGHT");
//    rotate_right(90,200);
//    if(!has_obstacle_front_center()){
//      move_forward(1);
//    }
//    if(!has_obstacle_front_center()){
//      move_forward(1);
//    }
//    if(!has_obstacle_front_center()){
//      move_forward(1);
//    }
//    if(!has_obstacle_front_center()){
//      move_forward(1);
//    }
//    rotate_right(90,200);
//    Serial.println("EXITED FUNC");
//  }
//}
//
////to call: oa_forward(front_center.getDistance());
//void oa_forward(double distance_cm){
//
//  Serial.print("DISTANCE:");
//  Serial.println(distance_cm);
//   double rpm1, rpm2;
//   
//   //Calculate the amount of motor ticks needed to reach the distance
//   double target_tick = distanceToTicks(distance_cm);
//   double tick_travelled = 0;
//
//   if(target_tick<0) return;
//
//   // Init values
//   tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
//   currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
//   oldTick1 = oldTick2 = 0; //All ticks accumulated until the current PIDController is called.
//   rpm1 = 60;
//   rpm2 = 60;
//   speed1 = rpm_to_speed_1(rpm1); //70.75 //74.9  100
//   speed2 = rpm_to_speed_2(rpm2); //70.5 //74.5 99.5
//
//   //Implementing gradual acceleration to remove jerks
//   for (int j = 0; j < speed2; j+=50){
//     md.setSpeeds(j-15,j+5);
//     delay(5); 
//   }
//
//   //Set Final ideal speed and accomodate for the ticks we used in acceleration
//   md.setSpeeds(speed1,speed2);
//   tick_travelled = (double)tick2;
//
//   //PID stuffs
//   PIDControlStraight.SetSampleTime(6.5); //Controller is called every 25ms
//   PIDControlStraight.SetMode(AUTOMATIC); //Controller is invoked automatically.
//
//  while(tick_travelled < target_tick){
//    // if not reach destination ticks yet
//    currentTick1 = tick1 - oldTick1; //calculate the ticks travelled in this sample interval of 25ms
//    currentTick2 = tick2 - oldTick2;
//    
//    Serial.print(currentTick1); //for debug
//    Serial.print(" "); Serial.println(currentTick2);
//    PIDControlStraight.Compute();
//
//    oldTick2 += currentTick2; //update ticks
//    oldTick1 += currentTick1;
//    tick_travelled += currentTick2;
//   
//  //Wide obstacle straight in front
//    if(front_left.getDistance() < 15 && front_right.getDistance() < 15 && front_center.getDistance() < 14){
//      Serial.println("ENTERED 1");
//       //md.setSpeeds(-100, -100);
//       rotate_left(90);
//       //print_Distance();
//       break;
//    }
//    //Incoming obstacle on the left
//    else if(front_left.getDistance() < 15 && front_right.getDistance() > 15){
//      Serial.println("ENTERED 2");
//       md.setBrakes(400, 400);
//       rotate_right(90,200);
//       Serial.println("Left corner blocked - Turning right");
//       break;
//    }
//    //Incoming obstacle on the right
//    else if(front_left.getDistance() > 15 && front_right.getDistance() < 15){
//      Serial.println("ENTERED 3");
//       md.setBrakes(400, 400);
//       delay(100);
//       rotate_left(90);
//       Serial.println("Right corner blocked - Turning left");
//       delay(100);
//       obstacle_detected(1);
//       break;
//    }    
//    else if(!has_obstacle_right_back() && !has_obstacle_right_front())
//    {
//        md.setBrakes(400, 400);
////      Serial.println("ENTERED 4");
////      move_forward(1);
////      rotate_right(90);
////      move_forward(2);
////      Serial.println("Past obstacle - Turning right");
////      break;
//    }
//  }
//}

void checklist_45(){
  int count=0;
  while(new_short_front_center()>15 && new_short_front_left()>15 && new_short_front_right()>15){
    if(count%3==0){
      right_wall_calibrate();
    }
    move_forward(1);
    angle_calibrate();
    count++;
    delay(50);
  }
  if(new_short_front_center()<=15 && new_short_front_left()>15){
    rotate_left(45,200);
    delay(100);
    move_forward(4);
    delay(100);
    rotate_right(90,200);
    delay(100);
    move_forward(4);
    delay(100);
    rotate_left(45,200);
    delay(100);
    right_wall_calibrate();
    return;
  }
  else if(new_short_front_center()<=15 && new_short_front_right()>15){
    rotate_right(45,200);
    delay(100);
    move_forward(4);
    delay(100);
    rotate_right(90,200);
    delay(100);
    move_forward(4);
    delay(100);
    rotate_left(45,200);
    delay(100);
    right_wall_calibrate();
    return;
  }
  else if(new_short_front_center()>15 && new_short_front_right()<=15)
  {
    rotate_left(45,200);
    delay(100);
    move_forward(4);
    delay(100);
    rotate_right(90,200);
    delay(100);
    move_forward(4);
    delay(100);
    rotate_left(45,200);
    delay(100);
    right_wall_calibrate();
    return;
  }
   else if(new_short_front_center()>15 && new_short_front_left()<=15)
  {
    rotate_right(45,200);
    delay(100);
    move_forward(4);
    delay(100);
    rotate_left(90,200);
    delay(100);
    move_forward(4);
    delay(100);
    rotate_right(45,200);
    delay(100);
    right_wall_calibrate();
    return;
  }
  
  
}


double circumference = PI * 6;
double distanceToTicks(double distance){
  return ((0.95*distance) * 562.25)/circumference;
}

/*
 * ==================================
 * Interrupt Service Routine
 * For counting ticks
 * ================================== 
 */
void E1_Pos(){
  tick1++;
}

void E2_Pos(){
  tick2++;
}

/**
 * calculate ticks for LEFT motor
 */
void leftMotorTime() {                            
  tick_L++;
}

/**
 * calculate ticks for RIGHT motor
 */
void rightMotorTime() {                           
  tick_R++;
}

void doEncoderLeft(){
    tick_L++;
    EndTimeLeft = micros();
    TimeWidthLeft = EndTimeLeft - StartTimeLeft;
    StartTimeLeft = EndTimeLeft;
    float duration_L = 2*TimeWidthLeft;
    duration_L = duration_L/1000000;
    RPM_L = ((1/duration_L)/(562.25/60));
    
}

void doEncoderRight(){
  
  tick_R++;
  EndTimeLeft = micros();
    TimeWidthLeft = EndTimeLeft - StartTimeLeft;
    StartTimeLeft = EndTimeLeft;
    float duration_L = 2*TimeWidthLeft/(leftTick);
    duration_L = duration_L/1000000;
    RPM_L = ((1/duration_L)/(562.25/60));
    leftTick = 0;  
}

/*
 * ======================================================
 * Conversion
 * Methods to convert stuffs
 * ======================================================
 */

double rpm_to_speed_1(double RPM){
//  if (RPM>0)
//    return 2.8598*RPM + 51.582;
//  else if (RPM == 0)
//    return 0;
//  else
//    return -2.9117*(-1)*RPM - 45.197;
 
//  if (RPM>0)
//    return (RPM + 3.1127)/0.3454;
//  else if (RPM == 0)
//    return 0;
//  else
//    return (RPM*(-1) - 3.1127)/0.3454;

  if (RPM == 0)
    return 0;
  else if(RPM>0)
    return (RPM - 0.3714)/0.3514;
  else
    return (RPM+0.3714)/0.3514;
}

double rpm_to_speed_2(double RPM){
//  if (RPM>0)
//    return (RPM + 9.2249)/0.3465;
//  else if (RPM == 0)
//    return 0;
//  else
//    return (RPM*(-1) - 9.2249)/0.3465;

  if (RPM == 0)
    return 0;
  else if(RPM>0)
    return (RPM - 1.5501)/0.3511;
  else
    return (RPM+1.5501)/0.3511;
}

//convert analog reading to distance (CM)
double short_distance(int reading){
  return reading;
}

/*
 * ======================================================
 * Communication
 * Methods to help in communication with RPI
 * ======================================================
 */


//method to get command string into the buffer
void get_command(){
    byte i = 0;
    while(Serial.available()>0){
       command[i] = Serial.read();
       i++;
       delay(2); //essential delay cause of serial being too slow
    }
    command[i] = '\0';

    //Debug print command
    if(DEBUG && command[0]!='\0'){
        Serial.print("COMMAND :");
        Serial.println(command);
    }
}

//method to print all characters of string received (for debug)
void print_all_commands(){
  byte i = 0;
  Serial.println("Msg Received: ");
  while (command[i] != '\0'){
    Serial.print(command[i]);
    i++;
  }
  Serial.print("EndOfLine");
  Serial.println();
}

/*
 * ========================
 * Others
 * ========================
 *
 * PID Theory:
 * Kp: Proportional term, decide how fast and how strong the system response to a change. System should be in an oscillating state (turn left and right and left)
 * Kd: Differential term, decide how dampen the oscillation will be. Higher = more dampened
 * Ki: How strong the response is if error is accumulated. (hard turn in extreme case)
 * -------------------------------------------------
 * Differential PID with Master Slave configuration:
 * Master: Stable, slower Motor
 * Slave: Unstable, faster motor
 * We feed a speed into the system, then measure the actual speed of both motor. 
 * We then calculate the error in this speed and 
 * calculate the PID adjustment to be feed back so the error is balanced out.
 * Here, we want to control the speed of unstable motor (1) so that it is as close and stable to stable motor (2)'s speed.
 * We feedback to the system as PWM speed (speed1) of the unstable motor 1.
 * ===================================================
 */
 void PIDdebug(PID controller){
  Serial.println("PID constants (P, I, D): ");
  Serial.println(controller.GetKp());
  Serial.println(controller.GetKi());
  Serial.println(controller.GetKd());
  Serial.println(controller.GetMode());
 }

/*
 * ============================
 * More Testing and Simulation
 * ============================
 *
 * In the absence of Rpi and Algo, we can roughly simulate some activities of the robot.
 *
 */

 //Method to test fastest path
 void Fastest(){
   FASTEST_PATH = true;
 }

 //method to test normal exploration
 void Explore(){
   move_forward(3);
   delay(500);
   rotate_right(90,200);
   delay(500);
   move_forward(2);
   delay(500);
   rotate_left(90,200);
   delay(500);
   move_forward(9);
   delay(500);
   rotate_left(90,200);
   delay(500);
   move_forward(2);
 }

//This was to drain fully charge battery to a suitable voltage.
 void Drain(){
  while(true){
    md.setSpeeds(400,400);
  }
 }
//U
