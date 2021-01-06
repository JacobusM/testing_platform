//Micro-controller Code for VECTR Stair-climbing Robot
#define RC_CH1_INPUT 12 
#define RC_CH2_INPUT 11
#define RC_CH3_INPUT 13
 
//Pin numbers as wired onto micro-controller
const int IN1 = 8;
const int IN2 = 7;
const int IN3 = 5;
const int IN4 = 4;
const int ENM1 = 9;
const int ENM2 = 3;
 
int trigPin1=30;
int echoPin1=31;
int trigPin2=32;
int echoPin2=33;
int trigPin3=34;
int echoPin3=35;
int trigPin4=36;
int echoPin4=37;
int trigPin5=38;
int echoPin5=39;
 
long duration1, distance1;
long duration2, distance2;
long duration3, distance3;
long duration4, distance4;
long duration5, distance5;
 
float m1; //mapped speed for left motor from -200 to 200
float m2; //mapped speed for right motor from -200 to 200
float M1; //mapped speed for left motor from -100 to 100
float M2; //mapped speed for right motor from -100 to 100
float a;
float b;
 
void setup() {
  Serial.begin(9600);
 
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENM1, OUTPUT);
  pinMode (ENM2, OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin5, INPUT);
}
 
void loop() {
  //Read channels from transmitter
  int CH1 = pulseIn(RC_CH1_INPUT, HIGH, 50000);
  int CH2 = pulseIn(RC_CH2_INPUT, HIGH, 25000);
  int CH3 = pulseIn(RC_CH3_INPUT, HIGH, 25000);
  
  //If transmitter is turned off, motors should not move
  if (CH1 || CH2 < 800) {
      m1 = 0;
      m2 = 0;
    }
  //If manual mode is engaged
  if (CH3 < 1500) {
    a = map(CH1,988,2012,-100,100);
    b = map(CH2,988,2012,-100,100);
    m1 = 2*a+0.8*b;
    m2 = 2*a-0.8*b;
    
    M1 = map(m1,-200,200,-100,100);
    M2 = map(m2,-200,200,-100,100);
    if (M1<=0){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      m1 = 2*a-b;
      M1 = map(m1,-200,200,-100,100);
    }
    else{
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }
    
    if (M2<=0){
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      m2 = 2*a+b;
      M2 = map(m2,-200,200,-100,100);
    }
    else{
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
  }
  //If autonomous mode is engaged
  else {
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin1, LOW);
    duration1 = pulseIn(echoPin1, HIGH);
    distance1 = (duration1/2) / 29.1;
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin2, LOW);
    duration2 = pulseIn(echoPin2, HIGH);
    distance2= (duration2/2) / 29.1;
    digitalWrite(trigPin3, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin3, LOW);
    duration3 = pulseIn(echoPin3, HIGH);
    distance3= (duration3/2) / 29.1;
    digitalWrite(trigPin4, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin4, LOW);
    duration4 = pulseIn(echoPin4, HIGH);
    distance4= (duration4/2) / 29.1;
    digitalWrite(trigPin5, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin5, LOW);
    duration5 = pulseIn(echoPin5, HIGH);
    distance5= (duration5/2) / 29.1;
 
    //Give initial speed to both motors
    m1 = 100;
    m2 = 100;
    
    if (distance1 < 30) { //If something is directly ahead of the platform and less that 30cm away: 
      m1 = m1 - m1*((30 - distance1)/11.25);
      m2 = m2 - m2*((30 - distance1)/11.25);
    }
    else { //Add to speed of both motors:
      m1 = m1 + 10;
      m2 = m2 + 10;
    }
    
    if (distance2 < 40) { //If something is left and ahead (diagonally) of the platform and less that 40cm away: 
      m1 = m1 + m1*((40 - distance2)/15);
      m2 = m2 - m2*((40 - distance2)/15);
    }
    else { //Add to speed of both motors:
      m1 = m1 + 10;
      m2 = m2 + 10; 
    }
    if (distance3 < 40) { //If something is right and ahead (diagonally) of the platform and less that 40cm away:
      m1 = m1 - m1*((40 - distance3)/15);
      m2 = m2 + m2*((40 - distance3)/15);
    }
    else { //Add to speed of both motors:
      m1 = m1 + 10;
      m2 = m2 + 10;
    }
    if (distance4 < 20) { //If something is left of the platform and less that 20cm away: 
      m1 = m1 + m1*((20 - distance4)/7.5);
      m2 = m2 - m2*((20 - distance4)/7.5);
    }
    else { //Add to speed of both motors:
      m1 = m1 + 10;
      m2 = m2 + 10;
    }
    if (distance5 < 20) { //If something is right of the platform and less that 20cm away: 
      m1 = m1 - m1*((20 - distance5)/7.5);
      m2 = m2 + m2*((20 - distance5)/7.5);
    }
    else { //Add to speed of both motors:
      m1 = m1 + 10;
      m2 = m2 + 10;
    }
    //If motor velocity exceeds bounds of mapping, set to maximum possible velocity magnitude
    if (m1 > 200) { m1 = 200; }
    if (m1 < -200) { m1 = -200; }
 
    if (m2 > 200) { m2 = 200; }
    if (m2 < -200) { m2 = -200; }
    
    M1 = map(m1,-200,200,-100,100);
    M2 = map(m2,-200,200,-100,100);
 
    if (M1<=0){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
    else{
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }
    if (M2<=0){
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    else{
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
  }
  //If motor velocity exceeds bounds of mapping, set to maximum possible velocity magnitude
  if (M1 > 100) { M1 = 100; }
  if (M1 < -100) { M1 = -100; }
  if (M2 > 100) { M2 = 100; }
  if (M2 < -100) { M2 = -100; }
  //Output velocity to motors
  analogWrite(ENM1, map(abs(M1),0,100,0,255));
  analogWrite(ENM2, map(abs(M2),0,100,0,255)); 
  delay(50);
}
