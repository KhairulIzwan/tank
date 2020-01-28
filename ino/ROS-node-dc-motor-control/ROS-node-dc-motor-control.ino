/*
 * Title: The ROS node that controls the DC motor
 * Author: https://wiki.dfrobot.com/Arduino_Motor_Shield__L298N___SKU_DRI0009_
 * Description: PWM Speed Control
 * 
 * Re-arrange the code in order to make it works! [Khairul Izwan (24-01-2020)]
 */

//include necessary library
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;
//Arduino PWM Speed Control:
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

#define wheelSep 0.14 // mm
#define wheelRadius 0.05; // mm

float transVelocity;
float rotVelocity;
float velDiff;

float leftPWM;
float rightPWM;

//char transVel[10];
//char rotVel[10];

void messageCb(const geometry_msgs::Twist &msg)
{
////  convert float to char
//  dtostrf(msg.linear.x, 0, 2, transVel);
//  dtostrf(msg.angular.z, 0, 2, rotVel);
//  
//  nh.loginfo("Linear Velocity:\t");
//  nh.loginfo(transVel);
//  nh.loginfo("Angular Velocity:\t");
//  nh.loginfo(rotVel);

  transVelocity = msg.linear.x;
  rotVelocity = msg.angular.z;

  velDiff = (wheelSep * rotVelocity) / 2.0;
  leftPWM = (transVelocity + velDiff) / wheelRadius;
  rightPWM = (transVelocity - velDiff) / wheelRadius;

//  Convert to PWM DC (0 ~ 100)
  leftPWM = (leftPWM - 0) * (100 - 0) / (4.4 - 0) + 0;
  rightPWM = (rightPWM - 0) * (100 - 0) / (4.4 - 0) + 0;
  
  motorDirection();
}

void motorDirection()
{
  if (leftPWM > 0 and rightPWM > 0)
  {
//    tank forward
      nh.loginfo("[WARN] Forward...");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, abs(leftPWM));
      analogWrite(E2, abs(rightPWM));
  } 
  else if (leftPWM < 0 and rightPWM < 0)
  {
//    tank backward
      nh.loginfo("[WARN] Backward...");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, abs(leftPWM));
      analogWrite(E2, abs(rightPWM));
  }
  else if (leftPWM < 0 and rightPWM > 0)
  {
//    tank turn left
      nh.loginfo("[WARN] Left...");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, abs(leftPWM));
      analogWrite(E2, abs(rightPWM));
  }
  else if (leftPWM > 0 and rightPWM < 0)
  {
//    tank turn right
      nh.loginfo("[WARN] Right...");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, abs(leftPWM));
      analogWrite(E2, abs(rightPWM));
  }
  else
  {
//    tank stalled
      nh.loginfo("[INFO] Stop!...");
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", messageCb);

//put your setup code here, to run once:
void setup()
{
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
}

//put your main code here, to run repeatedly:
void loop()
{
  nh.spinOnce();
  delay(1);
}
