#include <Servo.h>
#include <math.h>

//Servo[0]=TopArm
//Servo[1]=MiddleArm
//Servo[2]=Grip

Servo servo[3];

// setup Pins
const int servo_Pin[] = {6, 7, 4}; //top,middle,grip
//const int servoGrip_val[]= {145,105,145};  //open/max, close/min, default
const int servoGrip_val[]= {170,125,170};  //open/max, close/min, default
const int stepper_dirPin[] = {8};
const int stepper_stepPin[] = {9};

// servo Angles
double angle_current[] = {90, 90, 145};
double angle_next[] = {90, 90, 145};

// starting arm Coordinates
const double XYZ_base[] = {0, 0, -6, 1, 0, 0, 1}; //x,y,z, bool_move, bool_open, delay_to_next, type_of_action
double XYZ_current[] = {0, 0, -9, 1, 0, 1}; //x,y,z, bool_move, bool_open, delay_to_next, type_of_action
double XYZ_next[] = {0, 0, -9, 1, 0, 1}; //x,y,z, bool_move, bool_open, delay_to_next, type_of_action

// operating Constraints
const int stepper_delay[] = {27 * 28}; // 27 for sixteen, 27*22 for full step
const int stepper_maxsteps[] = {4000}; 
const double STEPS_PER_CM[] ={50.335}; //1500 total travel steps for ~29.8 cm = 50.335 steps per cm
double stepper_correction[]={0};

// make sure that the servo angle position, correctly translates to the trigonometric formulas of the arm position (see docs)
// set the servo to 90 degress, then measure the real angle of the arm.
// the calibration value is (90 - {position of arm vs reference at servo 90}), which then helps all values translate to sevo position.
const double calibrate_TopArm=90-35.7;
const double calibrate_MiddleArm=90-41.4;
//compensate for Grip Height (vs. middle arm end) on Z-axis
const double calibrate_Z=8.5;

// communication
const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;


void setup() {

  // Setup Communication
  Serial.begin(9600);
  //Serial.println("#====# Setup Start #====#");
  Serial.println("start");

  // Setup Pin Modes
  pinMode(stepper_dirPin[0], OUTPUT);
  pinMode(stepper_stepPin[0], OUTPUT);
  int i = 0;
  for (i = 0; i < 3; i++) {
    pinMode(servo_Pin[i], OUTPUT);
  }

  // Setup initial Servo Angles
  //for(i=0; i<2; i++) { servo[i].write(90); angle_current[i]=90; }
  servo[2].write(servoGrip_val[2]); angle_current[2] = servoGrip_val[2];
  // Set Coordinates a Base
  int y = 0;
  int z = -6;
  coordinate_move(0, y, z, false);

  // setup servos
  for (i = 0; i < 3; i++) {
    servo[i].attach(servo_Pin[i], 500, 2500);
  }

  //=========run tests (make sure to set "loop" bolean to false)
  //test_stepper();
  //test_servo(0);
  //test_servo(1);
  //test_servo(2);
  //test_servo_home(0);
  //test_servo_home(1);
  //test_servo_home(2);
  //test_getangles(-5,-20); free-movement
  //test_getangles(5,-20);
  //test_getangles(-5,-9);
  //test_getangles(5,-9);
  //test_getangles(0,-13);

  //coordinate_move(0,-9,0);
  //delay(500);50
  //coordinate_move(0, y, 0, false);
  //delay(500);

  //Serial.println("#====# Arduino Ready #====#");
  Serial.println("ready");

  
}

void loop() {


  // put your main code here, to run repeatedly:
  bool loop=true;
  
  recvWithStartEndMarkers();
  showNewData();
  //delay(5000);
  
  //data format <x,y,z,bool_move,bool_open,delayms,type_int> = <23,56,89,1,1,3456,3> {17}
  //X: 7.00 Y: 8.00 Z: 9.00 bool_move: 1.00 bool_open: 0.00 delay_ms: 10.00 move_type: 1.00

  //the bool_move controls if the arm moves linearly to the position or performs a pick/grab motions (move x first/y second etc)
  
  //<0,0,-9,1,1,1>

  if (newData==true && loop==true) {
    coordinate_move(XYZ_next[0],XYZ_next[1],XYZ_next[2],XYZ_next[3]);
    servo_Open(XYZ_next[4]);
    delay(XYZ_next[5]);
    newData=false;
    Serial.println("done");
  }

}


void get_angles_from_yz(double y, double z) {

  //refer to trigonometry illustration for variable description

  double H, s1, s2, aB, aA, aQ, servo1angle, servo2angle, y2, z2, y3, z3;

  //arm length in cm
  int L = 13;

  H= sqrt (pow(y,2) + pow(z,2));
  s1=H/2;
  s2=sqrt (pow(L,2) - pow(s1,2));

  aB=atan(s2/s1);
  y2=y/2;
  z2=z/2;
  aA=atan(y2/z2);

  servo1angle=aA+aB;
  servo1angle= (servo1angle/ (2 * M_PI)) * 360;

  //matrix multiplication - counterclockwise rotation
  y3 = -L*sin(aA+aB);
  z3 = -L* cos(aA+aB);

  servo2angle=atan((y-y3)/(z-z3));  

  servo2angle= (servo2angle / (2 * M_PI)) * 360;
  //tangent calculation changes when servo2 exceeds 90 degrees, correction below
  if ((z-z3)>0) {
    servo2angle=servo2angle-180;
  }

  //Absolute Top Arm Angle
  //Top Arm moves 0 to +90
  angle_next[0] = servo1angle;

  //Absolute Middle Arm Angle
  //Midle Arm moves 0 to +90
  angle_next[1] = -servo2angle;


  //Convert to SERVO Angle
  //in this case, a 90 servo position is equal to 71 degrees for Top arm
  //90 servo position is equal to 65 Middle Arm
  angle_next[0] = angle_next[0] + calibrate_TopArm;
  angle_next[1] = angle_next[1] + calibrate_MiddleArm;

}


void  coordinate_move(double xEnd, double yEnd, double zEnd, bool liftgrab_motion) {

  double xStart = XYZ_current[0];
  double yStart = XYZ_current[1];
  double zStart = XYZ_current[2];

  //Serial.println("/ Coord Move Start /");

  //calibrate stepper steps into cms (for x axis)
  
  double x_to_steps = STEPS_PER_CM[0];

  //identify if there is movement in the y Axis
  double zDelta = zEnd - zStart;
  //identify if there is movement in the z Axis
  double xDelta = xEnd - xStart;

  double x_stepper_steps = x_to_steps * abs(xDelta);

  if (xDelta != 0) {
    if (xDelta > 0) {
      stepper_advance(0, x_stepper_steps, 0);
    } else {
      stepper_advance(0, x_stepper_steps, 1);
    }
  }

  //the liftbrab_motion bool is equivalent to the bool_move paramter
  // controls if the arm moves linearly to the position or performs a pick/grab motions (move Y first/z second etc)
  

  if (liftgrab_motion == true) {
    if (zDelta < 0) {
      //arm is going to move down, move Y first

      // move arms in Y direction
      get_angles_from_yz(yEnd, zStart);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);

      // move arms in Z direction
      get_angles_from_yz(yEnd, zEnd);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);

    } else {
      //arm is moving up, perform Y movement first.

      // move arms in Z direction
      get_angles_from_yz(yStart, zEnd);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);

      // move arms in Y direction
      get_angles_from_yz(yEnd, zEnd);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);
    }

  } else {
    get_angles_from_yz(yEnd, zEnd);
    twoarm_step_coordinate(angle_next[0], angle_next[1]);
  }


  //Serial.println("/ Coord Move End /");

  //Serial.println(" //////// ");
  //Serial.print(" xStart=  "); Serial.print(xStart); Serial.print(" yStart=  "); Serial.println(yStart);
  //Serial.print("Angle Top Arm="); Serial.print(angle_TopArm); Serial.print(" Angle Middle Arm=  "); Serial.println(angle_MiddleArm);
  //Serial.print("Angle Top Arm_next="); Serial.print(angle_TopArm_next); Serial.print(" Angle Middle Arm_next=  "); Serial.println(angle_MiddleArm_next);
  //Serial.print(" xEnd=  "); Serial.print(xEnd);   Serial.print(" yEnd=  "); Serial.println(yEnd);

  XYZ_current[0] = xEnd;
  XYZ_current[1] = yEnd;
  XYZ_current[2] = zEnd;
  XYZ_current[3] = liftgrab_motion;
}


void stepper_advance(int stepper_num, double steps, int dir) {

  // generates the pulso for the Pololu controller to move the stepper.
  // this is helpful, given the delay has to change if you want to modifiy your stepper to operate at Full/Half steps etc.

  // check to see if a full step needs to be corrected
  if (abs(stepper_correction[stepper_num]) > 1) {
    if (stepper_correction[stepper_num]>1){
      //add one steps if correction is >1
      //steps++;
      //remove that step from the correction log
      //stepper_correction[stepper_num]--;
    } else {
      //steps--;
      //stepper_correction[stepper_num]++;
    }
  }

  // set direction
  if (dir == 0) {
    digitalWrite(stepper_dirPin[stepper_num], HIGH);
  } else {
    digitalWrite(stepper_dirPin[stepper_num], LOW);
  }

  // send pulse signal to stepper
  while (1) {
    digitalWrite(stepper_stepPin[stepper_num], HIGH);
    delayMicroseconds(stepper_delay[stepper_num]);

    digitalWrite(stepper_stepPin[stepper_num], LOW);
    delayMicroseconds(stepper_delay[stepper_num]);

    steps--;
    if (steps < 1) break;
  }


  // accumulate correction on the remainder of steps to avoid de-calibration
  if (steps > 0 && steps <1) {
    if (dir ==0) {
      stepper_correction[stepper_num]+=steps;
    } else {
      stepper_correction[stepper_num]-=steps;
    }
  }
  
  Serial.print("Stepper Remainder ");
  Serial.println(stepper_correction[stepper_num]);

}

void servo_steps(int servo_num, double angle_target, double incr_step = 10, int step_delay = 50) {
  // Using LD-20MG Servos with an Arduino Mega 2560, they only work smoothly on maximum 25 degree instructions at a time.
  // I haven't debugged the hardware, but this functions solves the issue.

  // This function helps you send commands to servos to move in a set of degrees at at a time.

  int set_angle;
  int angle_start = angle_current[servo_num];

  if (angle_start > angle_target) {
    //start from angle_start, and then move the servo by incr_steps into the angle_target
    //stepping down to target
    for (set_angle = angle_start; set_angle >= angle_target; set_angle -= incr_step) {
      servo[servo_num].write(set_angle);
      //Serial.println(set_angle);
      delay(step_delay);
    }
  } else {
    //stepping up to target
    for (set_angle = angle_start; set_angle <= angle_target; set_angle += incr_step) {
      servo[servo_num].write(set_angle);
      //Serial.println(set_angle);
      delay(step_delay);
    }
  }

  // make sure the servo arrives at the target
  servo[servo_num].write(angle_target);
  //update the current angle
  angle_current[servo_num] = angle_target;

}

void twoarm_step_coordinate(double toparm_target, double middlearm_target) {

  //Serial.println("--> two arm step Start /");

  double incr_steps0=1;
  double incr_steps1= 1;
  int inner_step_delay0 = 0;
  int inner_step_delay1 = 0;
  int outer_step_delay = 30;
  double i, j;
  int e0 = 0;
  int e1 = 0;

  //Reference to function:  servo_steps(int servo_num, int angle_target, int incr_step=10, int step_delay=50)

  //identify which of the arms has a greater delta in terms of degress to move
  double delta0 = abs(angle_current[0] - toparm_target);
  double delta1 = abs(angle_current[1] - middlearm_target);

  //coordinate the speed of the two access through the incremental steps, so they can move smoothly in the x/y plane
  //   (this avoids one arm finishing its movements first, and generating huge variagtionsin the real x/y position of the endpoint)
  if (delta0!=0 && delta1!=0) {
    if (delta0 >= delta1) {
      incr_steps0 = (delta0 / delta1)*incr_steps1;
      //slow down motion as steps increase (just in case big jump in steps)
      inner_step_delay0=(delta0/delta1)*0.5;
      //reduce the outer step
      outer_step_delay=outer_step_delay-inner_step_delay0;
    } else {
      incr_steps1 = (delta1 / delta0)*incr_steps0;
      //slow down motion as steps increase (just in case big jump in steps)
      inner_step_delay1=(delta1/delta0)*0.5;
      //reduce the outer step
      outer_step_delay=outer_step_delay-inner_step_delay1;
    }
  }
  //set to zero if negative value on outer delay
  if (outer_step_delay<0) {
    outer_step_delay=0;
  }
  
  //identify the direction of steps
  if (angle_current[0] > toparm_target) {
    i = -incr_steps0;
  } else {
    i = incr_steps0;
  }
  if (angle_current[1] > middlearm_target) {
    j = -incr_steps1;
  } else {
    j = incr_steps1;
  }
  
  // user the servo step functions, doing inter-twined steps until the gaps are reached.
  // we send a delay of 0 to the servo step function, given we'll control the delay in this outer loop.
  while (1) {
    // top arm moves
    if (abs(angle_current[0] - toparm_target) > incr_steps0) {
      servo_steps(0, angle_current[0] + i, incr_steps0, inner_step_delay0);
    } else {
      servo_steps(0, toparm_target, incr_steps0, inner_step_delay0);
      e0 = 1;
    }
    // middle arm moves
    if (abs(angle_current[1] - middlearm_target) > incr_steps1) {
      servo_steps(1, angle_current[1] + j, incr_steps1, inner_step_delay1);
    } else {
      servo_steps(1, middlearm_target, incr_steps1, inner_step_delay1);
      e1 = 1;
    }
    delay(outer_step_delay);
    if ((e0 + e1) >= 2) break;

  }

  //Serial.println("--> two arm step End /");

}



void servo_Open(bool openVal) {

  int servo_num = 2;
  int open_angle = servoGrip_val[0];
  int close_angle = servoGrip_val[1];

  //Reference to function:  servo_steps(int servo_num, int angle_target, int incr_step=10, int step_delay=50)

  if (openVal == true) {
    servo_steps(servo_num, open_angle, 1, 5);
  } else {
    servo_steps(servo_num, close_angle, 1, 5);
  }
  
  XYZ_current[4] = openVal;
}

void test_stepper() {

  stepper_advance(0, stepper_maxsteps[0], 0);
  delay(2500);
  stepper_advance(0, stepper_maxsteps[0], 1);

}
void test_servo(int servo_num) {

  int angle_max = 0;
  int angle_min = 0;
  int angle_default = 0;

  //ref servo_steps(int servo_num, int angle_target, int incr_step=10, int step_delay=50)

  //top servo
  if (servo_num == 0) {
    //max design free-movement limits (your actual setup limits may be different!)
    angle_max = 100;
    angle_min = 10;
    angle_default = 90;
  }
  //middle servo
  if (servo_num == 1) {
    //max design  free-movementlimits (your actual setup limits may be different!)
    angle_max = 120;
    angle_min = 30;
    angle_default = 90;
  }
  //grip servo
  if (servo_num == 2) {
    angle_max = servoGrip_val[0];
    angle_min = servoGrip_val[1];
    angle_default = servoGrip_val[2];
  }

  servo_steps(servo_num, angle_max);
  delay(1000);
  servo_steps(servo_num, angle_min);
  delay(1000);
  servo_steps(servo_num, angle_default);

}

void test_servo_home(int servo_num) {

  int angle_default = 0;

  //ref servo_steps(int servo_num, int angle_target, int incr_step=10, int step_delay=50)

  //top servo
  if (servo_num == 0) {
    angle_default = 90;
  }
  //middle servo
  if (servo_num == 1) {
    angle_default = 90;
  }
  //grip servo
  if (servo_num == 2) {
    angle_default = servoGrip_val[2];
  }

  servo_steps(servo_num, angle_default);

}

void test_getangles(double y, double z) {
   
  get_angles_from_yz(y,z);

  Serial.print("Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  Serial.print("servo1: ");
  Serial.print(angle_next[0]-calibrate_TopArm);
  Serial.print(" servo2: ");
  Serial.println(angle_next[1]-calibrate_MiddleArm);

  Serial.print("servo1 calibrated: ");
  Serial.print(angle_next[0]);
  Serial.print(" servo2 calibrated: ");
  Serial.println(angle_next[1]);


}



//from http://forum.arduino.cc/index.php?topic=288234.0

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
 // if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
   
}
void showNewData() {
    if (newData == true) {
        Serial.println(receivedChars);
        parseData();

        bool printmsg=false;
        if (printmsg == true) {
          Serial.print("X: ");
          Serial.print(XYZ_next[0]);
          Serial.print(" Y: ");
          Serial.print(XYZ_next[1]);
          Serial.print(" Z: ");
          Serial.print(XYZ_next[2]);
          Serial.print(" bool_move: ");
          Serial.print(XYZ_next[3]);
          Serial.print(" bool_open: ");
          Serial.print(XYZ_next[4]);
          Serial.print(" delay_ms: ");
          Serial.print(XYZ_next[5]);
          Serial.print(" move_type: ");
          Serial.println(XYZ_next[6]);
        }
        //newData = false;
    }
}


void parseData() {

  // split the data into its parts

  //data format <x,y,z,bool_move,bool_open,delayms,type_int> = <23,56,89,1,1,3456,3> {17}
  //X: 7.00 Y: 8.00 Z: 9.00 bool_move: 1.00 bool_open: 0.00 delay_ms: 10.00 move_type: 1.00

  char * strtokIndx; // this is used by strtok() as an index
 
  //grab X
  strtokIndx = strtok(receivedChars,",");      // get the first part - the string
  XYZ_next[0] = atof(strtokIndx);     // convert this part to a float
  //grab Y
  strtokIndx = strtok(NULL, ",");
  XYZ_next[1] = atof(strtokIndx);     // convert this part to a float
  //grab Z
  strtokIndx = strtok(NULL, ",");
  XYZ_next[2] = atof(strtokIndx);     // convert this part to a float
  //grab bool_move
  strtokIndx = strtok(NULL, ",");
  XYZ_next[3] = atoi(strtokIndx);     // convert this part to a integer
  //grab bool_open
  strtokIndx = strtok(NULL, ",");
  XYZ_next[4] = atoi(strtokIndx);     // convert this part to a integer
  //grab delayms
  strtokIndx = strtok(NULL, ",");
  XYZ_next[5] = atoi(strtokIndx);     // convert this part to a integer
  //type of action
  strtokIndx = strtok(NULL, ",");
  XYZ_next[6] = atoi(strtokIndx);     // convert this part to a integer
    
}
