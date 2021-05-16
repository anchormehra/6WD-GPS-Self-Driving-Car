#include "Coordinates.h" //GPS Coordinates

#include <Wire.h> //I2C library
#include "./Adafruit_PWMServoDriver.h" //PWM motor driver
#include <SoftwareSerial.h> //Software Serial for GPS
#include <TinyGPS++.h> //TinyGPS library
#include <QMC5883LCompass.h> //QMC5883L magnetic compass library.

//motor speeds
int Speed = 500; //Straight speed
int tSpeed = 1000; //Turn Speed

//GPS neo6M
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4,3); //GPS RX is connected to D4 and TX is connected to D3

//Obstacle avoidance using ultrasonic sensor
int trigPin =8; //Trigger pin of ultrasonic sensor
int echoPin =9; //Echo pin of ultrasonic sensor
long duration, obstacle;

//Motor Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

//Compass
QMC5883LCompass qmc;

//Compass Variables
int x, y, z;
int azimuth;


//GPS Variables
double latitude , longitude;
String lat_str , lng_str; //for debugging
int couarseChange;
int coarseToDestination;
double distanceToDestination;
double prevDistanceToDestination;

//Obstacle avoidance function
void findObstacle(){ 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);   
  digitalWrite(trigPin, HIGH);     // send waves for 10 us
  delayMicroseconds(10);
  duration = pulseIn(echoPin, HIGH); // receive reflected waves
  obstacle = duration / 58.2;   // convert to distance
  delay(100);
}

//Function that finds the Azimuth
void getAzimuth(){
  int sum;
  sum = 0;
  for(int i=0; i<10; i++){
    qmc.read();
    sum += qmc.getAzimuth();
  }
  azimuth = (sum/10);

}


//Go straight 
void goForward()
{

    pwm.setPWM(8, 0, Speed);
    pwm.setPWM(9, 0, 0);
    pwm.setPWM(11, 0, Speed);
    pwm.setPWM(10, 0, 0);

    pwm.setPWM(0, 0, Speed);
    pwm.setPWM(1, 0, 0);
    pwm.setPWM(3, 0, Speed);
    pwm.setPWM(2, 0, 0);

    pwm.setPWM(12, 0, Speed);
    pwm.setPWM(13, 0, 0);
    pwm.setPWM(15, 0, Speed);
    pwm.setPWM(14, 0, 0);
    delay(1000);
}

//go back
void goBackward()
{
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, Speed);
    pwm.setPWM(11, 0, 0);
    pwm.setPWM(10, 0, Speed);

    pwm.setPWM(0, 0, 0);
    pwm.setPWM(1, 0, Speed);
    pwm.setPWM(3, 0, 0);
    pwm.setPWM(2, 0, Speed);

    pwm.setPWM(12, 0, 0);
    pwm.setPWM(13, 0, Speed);
    pwm.setPWM(15, 0, 0);
    pwm.setPWM(14, 0, Speed);
    delay(500);
}

//brake function
void brake()
{
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, 0);
    pwm.setPWM(11, 0, 0);
    pwm.setPWM(10, 0, 0);

    pwm.setPWM(0, 0, 0);
    pwm.setPWM(1, 0, 0);
    pwm.setPWM(3, 0, 0);
    pwm.setPWM(2, 0, 0);

    pwm.setPWM(12, 0, 0);
    pwm.setPWM(13, 0, 0);
    pwm.setPWM(15, 0, 0);
    pwm.setPWM(14, 0, 0);
}

//left turn. Left wheels rools while right wheels don't move
void turnLeft()
{
    pwm.setPWM(8, 0, tSpeed);
    pwm.setPWM(9, 0, 0);
    pwm.setPWM(11, 0, 0);
    pwm.setPWM(10, 0, tSpeed);

    pwm.setPWM(0, 0, tSpeed);
    pwm.setPWM(1, 0, 0);
    pwm.setPWM(3, 0, 0);
    pwm.setPWM(2, 0, tSpeed);

    pwm.setPWM(12, 0, tSpeed);
    pwm.setPWM(13, 0, 0);
    pwm.setPWM(15, 0, 0);
    pwm.setPWM(14, 0, tSpeed);
    delay(100);

}

//right turn. Right wheels rools while left wheels don't move
void turnRight()
{
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, tSpeed);
    pwm.setPWM(11, 0, tSpeed);
    pwm.setPWM(10, 0, 0);

    pwm.setPWM(0, 0, 0);
    pwm.setPWM(1, 0, tSpeed);
    pwm.setPWM(3, 0, tSpeed);
    pwm.setPWM(2, 0, 0);

    
    pwm.setPWM(12, 0, 0);
    pwm.setPWM(13, 0, tSpeed);
    pwm.setPWM(15, 0, tSpeed);
    pwm.setPWM(14, 0, 0);
    delay(100);

}

//Left wheels rools font while right wheels rolls back
void spinLeft()
{
    pwm.setPWM(8, 0, tSpeed);
    pwm.setPWM(9, 0, 0);
    pwm.setPWM(11, 0, 0);
    pwm.setPWM(10, 0, tSpeed);

    pwm.setPWM(0, 0, tSpeed);
    pwm.setPWM(1, 0, 0);
    pwm.setPWM(3, 0, 0);
    pwm.setPWM(2, 0, tSpeed);


    pwm.setPWM(12, 0, tSpeed);
    pwm.setPWM(13, 0, 0);
    pwm.setPWM(15, 0, 0);
    pwm.setPWM(14, 0, tSpeed);
    delay(100);
}

//Right wheels rools font while left wheels rolls back
void spinRight()
{
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, tSpeed);
    pwm.setPWM(11, 0, tSpeed);
    pwm.setPWM(10, 0, 0);

    pwm.setPWM(0, 0, 0);
    pwm.setPWM(1, 0, tSpeed);
    pwm.setPWM(3, 0, tSpeed);
    pwm.setPWM(2, 0, 0);

    
    pwm.setPWM(12, 0, 0);
    pwm.setPWM(13, 0, tSpeed);
    pwm.setPWM(15, 0, tSpeed);
    pwm.setPWM(14, 0, 0);
    delay(100);
}

//Setup
void setup()
{
   Serial.begin(19200);
   pwm.begin();
   pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
   gpsSerial.begin(9600);
   Wire.begin();
   qmc.init();
   getAzimuth();
   pinMode(trigPin, OUTPUT);       
   pinMode(echoPin, INPUT);            
}


void loop()
{
  Goola();
}

void Goola(){
  Serial.println("Entered Goola");
  getAzimuth(); //Find azimuth
  findObstacle(); //Find obstracle distance
  Serial.print("Azimuth : ");
  Serial.println(azimuth);
  Serial.print("Number of GPS satellites: ");
  Serial.println(gps.satellites.value()); 
  Serial.print(gps.location.lat());
  Serial.print(" , ");
  Serial.println(gps.location.lng());
  Serial.print("Obstacle Distance: ");
  Serial.println(obstacle);

  while (gpsSerial.available() > 0){ //While MCU is receiving data from GPS module
    
    if (gps.encode(gpsSerial.read()))
    {
      Serial.println("Reading GPS Data");
      if (gps.location.isValid())
      {
  Serial.println("GPS OK");
  getAzimuth();
  Serial.print("Azimuth : ");
  Serial.println(azimuth);
        //For GPS debugging. IF GPS works, following lines will return course and distance. You can change coordinates to your desired point
  //Serial.print("Course to : ");
  //Serial.println(TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), 33.676772, -117.913715));
  //Serial.print("Distance to : ");
  //Serial.println(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),33.676772, -117.913715));
  Serial.print(gps.location.lat());
  Serial.print(" , ");
  Serial.println(gps.location.lng());

  for (int i=0; i<wayPointsSize; i++){
    do{
      while (gpsSerial.available() > 0){
        if (gps.encode(gpsSerial.read()))
    {
      if (gps.location.isValid())
      {


  //do{
    distanceToDestination=TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),wayPoints[i][0], wayPoints[i][1]);
  //}while(distanceToDestination!=prevDistanceToDestination);
  //do{
  coarseToDestination=TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), wayPoints[i][0], wayPoints[i][1]);
  couarseChange=coarseToDestination-azimuth;
  if(couarseChange<0)
    couarseChange+=360;
    findObstacle();
  if(obstacle>50){
    if(couarseChange>10 && couarseChange<180){
      spinRight();
      brake();
    Serial.println("Turning Right");
    Serial.println(i);
    }
    else if(couarseChange>179 && couarseChange<350){
      spinLeft();
      brake();
      Serial.println("Turning Left");
      Serial.println(i);
    }

  else{
    goForward();
    brake();
    Serial.println("Go Forward");
    Serial.println(i);
  }
  }
  else{
    Serial.println("Obstacle avoidance");
    do{
    spinLeft();
    findObstacle();
    }while(obstacle<50);
    if(obstacle>50){
      goForward();
      brake();
    }
  }
getAzimuth();

//}while(abs(couarseChange<5));

//goForward();
//Serial.println("Going Forward");
if(distanceToDestination<2.5)
  i++;
  if (i>wayPointsSize-1)
    i=0;
Serial.print("Azimuth : ");
Serial.println(azimuth);
Serial.print("Distance to Destination : ");
Serial.println(distanceToDestination);
Serial.print("Coarse to Destination : ");
Serial.println(coarseToDestination);
Serial.print("Coarse change: ");
Serial.println(couarseChange);
Serial.println(gps.location.lat());
Serial.println(gps.location.lng());
delay(500);
      }
      }
      }
}while(distanceToDestination>2.5);
}    
}
}
}
prevDistanceToDestination=distanceToDestination;
delay(500);
}


