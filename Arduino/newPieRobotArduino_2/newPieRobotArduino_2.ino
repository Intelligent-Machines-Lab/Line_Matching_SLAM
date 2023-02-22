/* Arduino Code for new PieRobot
 * Uses rosserial for comunicating with ROS via USB
 * Author: Luiz Eugenio S. A. Filho
 */
//Included Libraries---------------------------------
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <Encoder.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//---------------------------------------------------
//Variables and objects instantiation----------------
Encoder ENC1(19, 18);//Best Performance both pins have interrupt capability
const int EN_R = 9;//connected to Arduino's port 6 (PWM Speed Regulation)
const int IN1 = 7;//connected to Arduino's port 5 (Direction of Rotation)
const int IN2 = 8;//connected to Arduino's port 4 (Direction of Rotation)
Encoder ENC2(3, 2);//Best Performance both pins have interrupt capability
const int EN_L = 6;//connected to Arduino's port 6 (PWM Speed Regulation)
const int IN3 = 5;//connected to Arduino's port 5 (Direction of Rotation)
const int IN4 = 4;//connected to Arduino's port 4 (Direction of Rotation)

LiquidCrystal_I2C lcd(0x27,16,2);

long oldPosition  = -999;

long encM1, encM2; 

char hello[30];

ros::NodeHandle  nh;

std_msgs::String str_msg;

tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_msg;
geometry_msgs::TransformStamped t_base;


char odom[] = "/odom";
char base_link[] = "/base_link";

int defaultPWM = 200;
const int maxPWM = 255;
int pwmL = 0, pwmR = 0;
const double wheel_rad = 0.05, wheel_sep = 0.275;
double ang_z = 0, lin_x = 0;

struct Quaternion{
    double w, x, y, z;
};

struct Pose{
    double x, y, th;
};

Quaternion quat;
Pose pose;

unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0, readyToGo = 0;
double angle[3] = {0, 0, 0};
double headDegrees = 0;

const double tickL = 2*3.141592/2017, tickR = 2*3.141592/2062;
double WR_measured, WL_measured, lin_x_measured, ang_z_measured;
double tic;

const int delayTime = 200;
double dt = 0.1;

const double deg2rad = 3.141592/180; // Converts degrees to radians

unsigned long beginTime = 0, loopTime = 0;

unsigned char control = 0;
//---------------------------------------------------
//Functions definitions------------------------------
void codeCb( const std_msgs::Int32& code_msg ){
  int code = code_msg.data;
  switch(code){
    case 0:
      //calibDone = 0;
      break;
    case 2:
      pose.x = 0;
      pose.y = 0;
      break;
    case 3:
      resetEncoders(ENC1, ENC2);
      break;
    case 4:
      control = 1;
      break;
    case 5:
      control = 0;
      break;
    default:
      break;
  }
}


long readEncoder(Encoder& theEnc){
  long newPosition = theEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    return(newPosition);
  }
}

void resetEncoders(Encoder& theEnc1, Encoder& theEnc2){
  theEnc1.write(0);
  theEnc2.write(0);
}

void cmdCb(const geometry_msgs::Twist& msg){
  ang_z = msg.angular.z;
  lin_x = msg.linear.x;
  if(lin_x > 1) lin_x = 1;
  if(lin_x < -1) lin_x = -1;
  if(ang_z > 1) ang_z = 1;
  if(ang_z < -1) ang_z = -1;
  //w_r = (lin_x/wheel_rad) + ((ang_z*wheel_sep)/(2.0*wheel_rad));
  //w_l = (lin_x/wheel_rad) - ((ang_z*wheel_sep)/(2.0*wheel_rad));
}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, HIGH);
 }
 if (Pulse_Width1 < 0){
     Pulse_Width1 = abs(Pulse_Width1);
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN3, HIGH);
     digitalWrite(IN4, LOW);
 }
 if (Pulse_Width1 == 0){
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
 }
}

void MotorR(int Pulse_Width2){
 if (Pulse_Width2 > 0){
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1, HIGH);
     digitalWrite(IN2, LOW);
 }
 if (Pulse_Width2 < 0){
     Pulse_Width2 = abs(Pulse_Width2);
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, HIGH);
 }
 if (Pulse_Width2 == 0){
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, LOW);
 }
}

void serialEvent2() {
  while(Serial2.available()) {
    Re_buf[counter] = (unsigned char)Serial2.read();
    if(counter == 0 && Re_buf[0] != 0x55) 
      return;
    counter++;
    if(counter == 11){
      //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      counter = 0;
      sign = 1;
    }
  }
}

// Params expected to be in degrees (from IMU)
struct Quaternion ToQuaternion(double yaw, double pitch, double roll){
    // Abbreviations for the various angular functions
    double cy = cos(yaw * deg2rad * 0.5);
    double sy = sin(yaw * deg2rad * 0.5);
    double cp = cos(pitch * deg2rad * 0.5);
    double sp = sin(pitch * deg2rad * 0.5);
    double cr = cos(roll * deg2rad * 0.5);
    double sr = sin(roll * deg2rad * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

// After calling decodeIMU, one should have updated values for Acc/Gyro/Angle in globals a[]/w[]/angle[] respectively
void decodeIMU(){
  if(sign){
     sign = 0;
     if(Re_buf[0] == 0x55){
        switch(Re_buf[1]){
          case 0x53:
            angle[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0 * 180;
            angle[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0 * 180;
            angle[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0 * 180;
            readyToGo = 1;
            break;
        }
     }
  }
}

//---------------------------------------------------
//ROS pubs and subs----------------------------------
ros::Publisher chatter("chatter", &str_msg);
//ros::Publisher odom_pub("odom", &odom_msg);

//ros::Subscriber<std_msgs::Int32> code_sub("code_sub", &codeCb );
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &cmdCb );
//---------------------------------------------------
//Setup and loop start from here---------------------
void setup(){
  pinMode(EN_R,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);  
  pinMode(EN_L,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW);
  analogWrite(EN_R, 0);
  analogWrite(EN_L, 0);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

  Serial2.begin(115200);
  
  resetEncoders(ENC1, ENC2);

  nh.getHardware()->setBaud(115200);
  nh.initNode();  //Initiate rosserial node
  
  nh.advertise(chatter);  //For publishers
  //nh.advertise(odom_pub);  //For publishers

  nh.subscribe(cmd_vel);  //For subscribers
  //nh.subscribe(code_sub);  //For subscribers
  
  //broadcaster.init(nh);

  pose.x = 0.0;
  pose.y = 0.0;
  pose.th = 0.0;
}

void loop()
{
  beginTime = millis();
  
  // IMU stuff
  decodeIMU();
//  readyToGo = 1;
  if(readyToGo){
    // Motor lowlevel control. Using Walber's equation
    pwmL = defaultPWM * lin_x - defaultPWM * (-ang_z) + ((maxPWM - defaultPWM) / 2) * lin_x * sq(ang_z) + ((3*defaultPWM - maxPWM) / 2) * (-ang_z) * sq(lin_x);
    pwmR = defaultPWM * lin_x + defaultPWM * (-ang_z) + ((maxPWM - defaultPWM) / 2) * lin_x * sq(ang_z) - ((3*defaultPWM - maxPWM) / 2) * (-ang_z) * sq(lin_x);

    // Reading first encoder measurement before movement
    encM1 = readEncoder(ENC1);
    encM2 = readEncoder(ENC2);

    tic = millis(); // Start tic to measure instantaneous angular velocity via encoder
    
    MotorL(pwmL);
    MotorR(pwmR);

    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print(String("E1:") + String(encM1));
    lcd.setCursor(8,1);
    lcd.print(String("E2:") + String(encM2));
    lcd.setCursor(0,0);
    lcd.print(String("x") + String(pose.x));
    lcd.setCursor(4,0);
    lcd.print(String("y") + String(pose.y));
    lcd.setCursor(8,0);
    lcd.print(String("t") + String(headDegrees));
    lcd.setCursor(15,0);
    lcd.print(F("\xDF"));

    //if(abs(angle[2]-headDegrees) <= 50)
    headDegrees = angle[2];  
    quat = ToQuaternion(headDegrees, 0, 0);
/*
    // Base link frame transformation
    t_base.header.frame_id = odom;
    t_base.child_frame_id = base_link;
    t_base.transform.translation.x = pose.x;
    t_base.transform.translation.y = pose.y;
    t_base.transform.translation.z = 0.0;
    t_base.transform.rotation.x = quat.x;
    t_base.transform.rotation.y = quat.y;
    t_base.transform.rotation.z = quat.z;
    t_base.transform.rotation.w = quat.w;
    t_base.header.stamp = nh.now();
*/
    // Sending transformations
    //broadcaster.sendTransform(t_base);
/*
    // Preparing Odom message
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = pose.x;
    odom_msg.pose.pose.position.y = pose.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = quat.x;
    odom_msg.pose.pose.orientation.y = quat.y;
    odom_msg.pose.pose.orientation.z = quat.z;
    odom_msg.pose.pose.orientation.w = quat.w;
    odom_msg.pose.covariance[0] = 0.1;
    odom_msg.pose.covariance[7] = 0.1;
    odom_msg.pose.covariance[14] = 0.1;
    odom_msg.pose.covariance[21] = 0.1;
    odom_msg.pose.covariance[28] = 0.1;
    odom_msg.pose.covariance[35] = 0.1;
    odom_msg.twist.twist.linear.x = lin_x_measured;
    odom_msg.twist.twist.angular.z = ang_z_measured;
    odom_msg.header.stamp = nh.now();
*/
    // Preparing chatter message
    (String(pose.x, 3) + String(",") + String(pose.y, 3) + String(",") + String(pose.th, 3)).toCharArray(hello, 30);
    str_msg.data = hello;
    
    // Sending the messages through pubs
    chatter.publish( &str_msg );
//    odom_pub.publish( &odom_msg );
   
    nh.spinOnce();
    while(((millis() -  beginTime) <= (delayTime - loopTime)) && (loopTime < delayTime)); // Hodor until 300 ms has passed
    readyToGo = 0;
    loopTime = 0;
  
  } //end if(readyToGo)
  
  // Odometry stuff.
  WR_measured = (readEncoder(ENC1) - encM1) * (tickR/(millis() - tic)) * 1000; // End tic for right angular velocity
  WL_measured = (readEncoder(ENC2) - encM2) * (tickR/(millis() - tic)) * 1000; // End tic for left angular velocity
  
  lin_x_measured = wheel_rad * (WR_measured + WL_measured) / 2;
  ang_z_measured = wheel_rad * (WR_measured - WL_measured) / wheel_sep;

  dt = (millis() - tic)/1000;
  
  pose.x += (lin_x_measured * 0.01 *  cos(pose.th * deg2rad)) * dt;
  pose.y += (lin_x_measured * 0.01 *  sin(pose.th * deg2rad)) * dt;
  pose.th = headDegrees;
  
  loopTime += (millis() - beginTime); // End tic for ROS loop
}
//---------------------------------------------------
