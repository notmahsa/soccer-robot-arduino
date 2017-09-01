#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <NewPing.h>

#define TRIGGER_PIN  50
#define ECHO_PIN     51
#define TRIGGER_PIN2  52
#define ECHO_PIN2     53
#define TRIGGER_PIN3  48
#define ECHO_PIN3     49
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);

int sensor[8];
int x1,y1,z1,x_read,y_read;
int max = 0, num;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

const int motor1Pin = 36;    // H-bridge leg 1 (pin 2, 1A)
const int motor2Pin = 37;    // H-bridge leg 2 (pin 7, 2A)
const int enablePin = 44;    // H-bridge enable pin

const int motor1Pin2 = 32;    // H-bridge leg 1 (pin 2, 1A)
const int motor2Pin2 = 33;    // H-bridge leg 2 (pin 7, 2A)
const int enablePin2 = 45;    // H-bridge enable pin

const int motor1Pin3 = 28;    // H-bridge leg 1 (pin 2, 1A)
const int motor2Pin3 = 29;    // H-bridge leg 2 (pin 7, 2A)
const int enablePin3 = 46;    // H-bridge enable pin

void sensors(){
    max = 0;
    sensor[0] = analogRead(A0);
    sensor[1] = analogRead(A1);
    sensor[2] = analogRead(A2);
    sensor[3] = analogRead(A3);
    sensor[4] = analogRead(A4);
    sensor[5] = analogRead(A5);
    sensor[6] = analogRead(A6);
    sensor[7] = analogRead(A7);
    for (int i = 0; i < 8; i++){
      if (sensor[i] > max){
        max = sensor[i];
        num = i;
      }
    }
    num++;
    if (num==8) num = 0;
}

void motorController(int motor1, int motor2, int motor3){

  if (motor1 > 255) motor1 = 255;
  if (motor2 > 255) motor2 = 255;
  if (motor3 > 255) motor3 = 255;
  if (motor1 < -255) motor1 = -255;
  if (motor1 < -255) motor2 = -255;
  if (motor1 < -255) motor3 = -255;
  
  if (motor2 >= 0){
    analogWrite(enablePin, motor2);
    digitalWrite(motor1Pin, LOW);
    digitalWrite(motor2Pin, HIGH);
  }
  else if (motor2 < 0){
    analogWrite(enablePin, 0-motor2);
    digitalWrite(motor1Pin, HIGH);
    digitalWrite(motor2Pin, LOW);
  }
    
  if (motor1 >= 0){
    analogWrite(enablePin2, motor1);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  else if (motor1 < 0){
    analogWrite(enablePin2, 0-motor1);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }
    
  if (motor3 >= 0){
    analogWrite(enablePin3, motor3);
    digitalWrite(motor1Pin3, HIGH);
    digitalWrite(motor2Pin3, LOW);
  }
  else if (motor3 < 0){
    analogWrite(enablePin3, 0-motor3);
    digitalWrite(motor1Pin3, LOW);
    digitalWrite(motor2Pin3, HIGH);
  } 
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  pinMode(motor1Pin3, OUTPUT);
  pinMode(motor2Pin3, OUTPUT);
  pinMode(enablePin3, OUTPUT);
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  /*Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");*/

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  x1 = headingDegrees*255/360;
  if (x1 > 128) x1-=255;
  if (x1 <-128) x1+=255;
  
}

void loop(void) 
{

  /////////////////COMPASS
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI; 
  x_read = x1 - ((headingDegrees * 255) / 360);
  if (x_read > 128) x_read -= 255;
  if (x_read <-128) x_read += 255;
  Serial.print(num);Serial.print(": ");Serial.print(max);Serial.print("  & Degrees: "); Serial.println(x_read);
  /////////////////////////

  int cmp = x_read * 2;
  //cmp = 0;

  sensors();
  

  if (max > 700){

    
    if (cmp > 20 || cmp < -15) {
      if (cmp > 0 && cmp < 600) cmp += (60 - cmp);
      if (cmp < 0 && cmp > -60) cmp -= (60 + cmp);
      motorController(0+cmp,0+cmp,0+cmp);
    }
    
    else{
      cmp = 0;
      
      if (sensor[6] > 600 && sensor[7] > 600 && sensor[0] > 600) motorController(255+cmp,-255+cmp,0+cmp);
      
      else if (num == 0) motorController(255+cmp,-255+cmp,0+cmp);
      
      else if (max > 900){
        
        if (num == 1) motorController(-255+cmp,0+cmp,255+cmp);
        else if (num == 2) motorController(-255+cmp,255+cmp,0+cmp);
        else if (num == 3) motorController(0+cmp,255+cmp,-255+cmp);
        else if (num == 4) motorController(-128+cmp,-128+cmp,255+cmp);
        else if (num == 5) motorController(-255+cmp,128+cmp,128+cmp);
        else if (num == 6) motorController(-255+cmp,255+cmp,0+cmp);
        else if (num == 7) motorController(0+cmp,255+cmp,-255+cmp);
        
      }
      else if (max>700){
        
        if (num == 1) motorController(0+cmp,-255+cmp,255+cmp); //2
        else if (num == 2) motorController(-128+cmp,-128+cmp,255+cmp); //3
        else if (num == 3) motorController(-255+cmp,0+cmp,255+cmp); //4
        else if (num == 4) motorController(-255+cmp,255+cmp,0+cmp);
        else if (num == 5) motorController(0+cmp,255+cmp,-255+cmp);
        else if (num == 6) motorController(128+cmp,128+cmp,-255+cmp);
        else if (num == 7) motorController(255+cmp,0+cmp,-255+cmp);
        
      }
    }
  }
  
  else{

    if (cmp > 20 || cmp < -15) cmp = 0;
    if (cmp > 0 && cmp < 60) cmp += (60 - cmp);
    if (cmp < 0 && cmp > -60) cmp -= (60 + cmp);
    motorController(0+cmp,0+cmp,0+cmp);
  }
  
  //delay(20);
}
