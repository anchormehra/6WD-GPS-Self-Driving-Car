//#include "AdamsLotCoordinates.h"//Adams Lot Coordinates
//#include "LotECoordinates.h"
#include "HomeFront.h"

#include <Wire.h>
#include "./Adafruit_PWMServoDriver.h"
#include <SoftwareSerial.h> 
#include <TinyGPS++.h> 
#include <QMC5883LCompass.h>


//motor speeds
int Speed = 500;
int tSpeed = 1000;

//GPS
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4,3);

//Obstacle avoidance 
int trigPin =8;
int echoPin =9;
long duration;
long obstacle;

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

void findObstacle(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);   
  digitalWrite(trigPin, HIGH);     // send waves for 10 us
  delayMicroseconds(10);
  duration = pulseIn(echoPin, HIGH); // receive reflected waves
  obstacle = duration / 58.2;   // convert to obstacle
  if(obstacle == 16 || obstacle == 0){
    obstacle+=51;
  }
  delay(100);
}

void getAzimuth(){
  int sum;
  sum = 0;
  for(int i=0; i<10; i++){
    qmc.read();
    sum += qmc.getAzimuth();
  }
  azimuth = (sum/10);

}


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

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do{
    while(gpsSerial.available())
      gps.encode(gpsSerial.read());
  }while(millis()-start<ms);
}
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
  getAzimuth();
  findObstacle();
  Serial.print("Azimuth : ");
  Serial.println(azimuth);
  Serial.print("Number of GPS satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print(gps.location.lat());
  Serial.print(" , ");
  Serial.println(gps.location.lng());
  Serial.print("Obstacle Distance: ");
  Serial.println(obstacle);
  smartDelay(1000);
  //while (gpsSerial.available() > 0){
    while(gps.location.isValid()){
    
  //  if (gps.encode(gpsSerial.read()))
  //  {
  //    Serial.println("Reading GPS Data");
  //    if (gps.location.isValid())
  //    {
  Serial.println("GPS OK");
  getAzimuth();
  Serial.print("Azimuth : ");
  Serial.println(azimuth);
  Serial.print("Course to : ");
  Serial.println(TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), 33.676772, -117.913715));
  Serial.print("Distance to : ");
  Serial.println(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),33.676772, -117.913715));
  Serial.print(gps.location.lat());
  Serial.print(" , ");
  Serial.println(gps.location.lng());

  for (int i=0; i<wayPointsSize; i++){
    do{ 
      //while (gpsSerial.available() > 0){
        //if (gps.encode(gpsSerial.read()))
    //{
    //  if (gps.location.isValid())
    // {

 
  //do{
    distanceToDestination=TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),wayPoints[i][0], wayPoints[i][1]);
  //}while(distanceToDestination!=prevDistanceToDestination);
  
  do{
    findObstacle();
  coarseToDestination=TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), wayPoints[i][0], wayPoints[i][1]);
  couarseChange=coarseToDestination-azimuth;
  if(couarseChange<0)
    couarseChange+=360;
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
//    findObstacle();
    Serial.println("Obstacle avoidance");
    Serial.print("Obstacle :");
    Serial.println(obstacle);
    do{
    spinLeft();
    findObstacle();
    }while(obstacle<50);
    delay(250);
    brake();
    delay(250);
  }
  Serial.println("Exited the loop");
getAzimuth();

}while(abs(couarseChange<10));
  Serial.println("Exited the loop 2");
//goForward();
//Serial.println("Going Forward");
if(distanceToDestination<2.5){
  i++;
}
  if (i>wayPointsSize-1){
    i=0;
  }
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
//delay(500);
 //     }
//      }
  smartDelay(1000);    
}while(distanceToDestination>2.5);
  Serial.println("Exited the loop 3");
}    
//}
//}
}
prevDistanceToDestination=distanceToDestination;
delay(500);
}


