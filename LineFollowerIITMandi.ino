#include <QTRSensors.h>
#define left_motor_positive 4
#define left_motor_negative 5
#define right_motor_positive 3
#define right_motor_negative 2
#define en1 10
#define en2 11

int check = 0;
int attempt = 0;
int rightState =0;

int w = 400;
int b = 800;
int c = 600;

#define led 13



QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];








int initial_motor_speed = 80;
int rotating_speed = 60;
int forward_speed = 150;
int right_motor_speed = 0; //for the speed after PID control
int left_motor_speed = 0;


int error; 
float kp = .08; //proportiona constant
float ki = 0;
float kd = 2;
float P,I,D,previousError=0;;
int pid_value;

char mode;
int Status = 0;
int buttom_reading;

int ObstaclePin=10;
int ObstacleRead;
void led_signal(int times);

void calculatePID();
void PIDmotor_control();
   uint16_t position;

void readIRvalue();     //to read sensor value and calculate error as well mode
void Set_motion();

void dryrun();

void recIntersection(char);


void forward(int spd1, int spd2);
void left(int spd);
void right(int spd);
void stop_motor();
void goAndTurnLeft();
void maze_end();
void move_inch();
void backward(int spd1, int spd2);


void setup() 
{ 
// Serial.begin(9600);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  for (int i = 2; i <= 5; i++)
  {
    pinMode(i, OUTPUT);
  }
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
  delay(500);
  pinMode(13, OUTPUT);


  
  digitalWrite(13, HIGH);  

  
 // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 300; i++)  // make the calibration take about 10 seconds
  {
    qtr.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on

  
}


void loop() 
{   
    dryrun();
 
  }


void led_signal(int times)
{
  for (int i = 0; i <= times; i=i+1)
  {
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
  }
}


//dryrun begins------------------------------------------------------------------------------------------------------------------
void dryrun()
{
     readIRvalue();
     set_motion();
  
}
//ReadIRvalue begins----------------------------------------------------------------------------------------------------------------------------------
void readIRvalue()
{
   uint16_t position = qtr.readLineBlack(sensorValues);
    error = 3500 - position;

    if (sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w && sensorValues[6] < w && sensorValues[7] < w)
  {
    mode = 'N';  //NO LINE
    error = 0;
  }
  //--------------------------------------------------------------------------------------
  else if ( sensorValues[0] > b && sensorValues[1] > b && sensorValues[2] > b && sensorValues[3] > b && sensorValues[4] > b && sensorValues[5] > b && sensorValues[6] > b && sensorValues[7] > b)
  {
    mode = 'S';//Stop Condition
    error = 0;
  }
  //--------------------------------------------------------------------------------------

  else if (sensorValues[0] > b && sensorValues[1] > (b) && sensorValues[2] > b && sensorValues[3] > b && sensorValues[4] > b && sensorValues[5] > b && sensorValues[6] < w && sensorValues[7] < w)
  {
    mode = 'L';//90 degree turn
    error = 0;
  }
  
  else if ( sensorValues[0] > b && sensorValues[1] > b && sensorValues[6] < w && sensorValues[7] < w)
  {
    mode = 'L';//90 degree turn
    error = 0;
  }
  //-------------------------------------------------------------------------------------
  else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] > b && sensorValues[3] > b && sensorValues[4] > b && sensorValues[5] > b && sensorValues[6] > b && sensorValues[7] > b)
  {
    mode = 'R'; //90 degree turn
    error = 0;
  }

    else if ( sensorValues[0] < w && sensorValues[1] < c && sensorValues[2] < w && sensorValues[3] > b && sensorValues[4] > b && sensorValues[5] > b && sensorValues[6] > b && sensorValues[7] > b)
  {
    mode = 'R'; //90 degree turn
    error = 0;
  }
    else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < c && sensorValues[3] > b && sensorValues[4] > b && sensorValues[5] > b && sensorValues[6] > b && sensorValues[7] > b)
  {
    mode = 'R'; //90 degree turn
    error = 0;
  }
   else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[6] > (b) && sensorValues[7] > b)
  {
    mode = 'R'; //90 degree turn 
    error = 0;
  }

  else{
    mode = 'F';
    }
   

}



//set motion begins------------------------------------------------------------------------------------------------------------------------------
void set_motion()
{
switch (mode)
  {
    case 'N':
      stop_motor();
      goAndTurnLeft();
      recIntersection('N');      
      break;
    case 'S':
      stop_motor();
      move_inch();
      readIRvalue();
      if (mode == 'S')
      {
        maze_end();
      }
      else
      {
        goAndTurnLeft();
        recIntersection('L');
      }
      break;
    case'R':
      stop_motor();
      move_inch();
      readIRvalue();
      if (mode == 'N')
      {
        goAndTurnRight();
        recIntersection('R');
      }
      else
      {
        recIntersection('S');
      }
      break;
    case 'L':
      stop_motor();
      goAndTurnLeft();
      recIntersection('L');
      break;
    case 'F':
      calculatePID();
      PIDmotor_control();
      break;
  }
}

//-----------------------------------------------------------------------------------------------------------
void move_inch()
{
  forward(forward_speed, forward_speed);
  delay(250);
  stop_motor();
}
//----------------------------------------------------------------------------------------------------------

void stop_motor()
{
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
}



void goAndTurnLeft()
{ 
    previousError = 0;
  left(rotating_speed);
  delay(400);
  do
  {
    left(rotating_speed);
    readIRvalue();
    Serial.print("yo ");
  } while (sensorValues[3] < 600 || sensorValues[4] < 600);
}

//------------------------------------------------------------------------------------------------
void goAndTurnRight()
  {
   previousError = 0;
  right(rotating_speed);
  delay(300);
  do
  {
    right(rotating_speed);
    readIRvalue();
    
  }while (sensorValues[3] < 600 || sensorValues[4] < 600); 
    
}
//-----------------------------------------------------------------------------------------------
void maze_end()
{
  Status++;
  stop_motor();
  led_signal(20);
}

//------------------------------------------------------------------------------------------------
void calculatePID()
{
  
  P = error;
  I = I + error;
  D = error-previousError;
  pid_value = (kp*P) + (ki*I) + (kd*D);
  previousError = error;
  
}
//-----------------------------------------------------------------------------------------------
void PIDmotor_control()
{
  right_motor_speed = initial_motor_speed - pid_value;
  left_motor_speed = initial_motor_speed + pid_value;
  right_motor_speed = constrain(right_motor_speed, 0, initial_motor_speed);
  left_motor_speed = constrain(left_motor_speed, 0, initial_motor_speed);
  forward(left_motor_speed, right_motor_speed);
  
}

//-------------------------------------------------------------------------------------------------------
void forward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
  digitalWrite(led, LOW);
}



void left(int spd)
{
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
  digitalWrite(led, HIGH);
}

void right(int spd)
{
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, HIGH);
}
//--------------------------------------------------------------------------------------


void recIntersection(char Direction)
{

      Serial.println(Direction);
}

//--------------------------------------------------------------------------------------------------
