#include <ros.h>
#include <std_msgs/Int32.h>
#define IN1 9
#define IN2 4
#define IN3 7
#define IN4 8
#define PWM1 5
#define PWM2 6
#define dirPin 2
#define stepPin 3





int J1Val;
int J2Val;
int J3Val;

ros::NodeHandle nh;


void j1Callback(const std_msgs::Int32& msg) {
  J1Val = msg.data;}

void j2Callback(const std_msgs::Int32& msg) {
  J2Val = msg.data;}
  
void j3Callback(const std_msgs::Int32& msg) {
  J3Val = msg.data;}
  
ros::Subscriber<std_msgs::Int32> j1_sub("/J1", &j1Callback);
ros::Subscriber<std_msgs::Int32> j2_sub("/J2", &j2Callback);
ros::Subscriber<std_msgs::Int32> j3_sub("/J3", &j3Callback);


void setup() {
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (PWM1, OUTPUT);
  pinMode (PWM2, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);

  
  nh.initNode();
  nh.subscribe(j1_sub);
  nh.subscribe(j2_sub);
  nh.subscribe(j3_sub);
}

void loop() {

  if(J1Val == 1){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(PWM1,255);
   }
   else if(J1Val == 2){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(PWM1,255);
    }
    else analogWrite(PWM1,0);

   if(J2Val == 3){
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(PWM2,60);
    }
    else if(J2Val == 4){
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(PWM2,255);
    }
    else analogWrite(PWM2,0);


  if(J3Val == 5){
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(40);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(40);
   }
   else if(J3Val == 6){
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(40);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(40);
    }
    else{
     
    }
  nh.spinOnce(); 
}
