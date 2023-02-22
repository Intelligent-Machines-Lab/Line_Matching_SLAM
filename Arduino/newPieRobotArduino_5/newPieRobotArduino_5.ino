/* Arduino Code for new PieRobot
 * Uses rosserial for comunicating with ROS via USB
 * Author: Luiz Eugenio S. A. Filho
 */
//Included Libraries---------------------------------
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//---------------------------------------------------
//Variables and objects instantiation----------------
Encoder ENC1(19, 18);//Best Performance both pins have interrupt capability
const int EN_R = 9;//connected to Arduino's port 9 (PWM Speed Regulation)
const int IN1 = 7;//connected to Arduino's port 7 (Direction of Rotation)
const int IN2 = 8;//connected to Arduino's port 8 (Direction of Rotation)
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

int defaultPWM_L = 250, defaultPWM_R = 250;
const int maxPWM = 255;
int pwmL = 0, pwmR = 0;
const double wheel_rad = 0.05, wheel_sep = 0.275;
double ang_z = 0, lin_x = 0;

struct Pose{
    double x, y, th;
};

Pose pose;

const double tick2rad_L = 2*3.141592/2017, tick2rad_R = 2*3.141592/2062;
double WR_measured, WL_measured, lin_x_measured, ang_z_measured;
double tic;

double dt = 0.0;

const double deg2rad = 3.141592/180; // Converts degrees to radians
unsigned long beginTime = 0, loopTime = 0;
//---------------------------------------------------
//Functions definitions------------------------------
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
  if(lin_x > 0.4) lin_x = 0.85;
  if(lin_x < -0.4) lin_x = -0.85;
  if(ang_z > 0.5) ang_z = 0.85;
  if(ang_z < -0.5) ang_z = -0.85;
  /*
  if(lin_x > 0.4 && ang_z > 0.5){
    lin_x = 1.5;
    ang_z = -1.5;
    }
  if(lin_x > 0.4 && ang_z < -0.5){
    lin_x = 1.5;
    ang_z = 1.5;
    }
  if(lin_x < -0.4 && ang_z > 0.5){
    lin_x = -1.5;
    ang_z = -1.5;
    }
  if(lin_x < -0.4 && ang_z < -0.5){
    lin_x = -1.5;
    ang_z = 1.5;
    }
    */
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

double wrap180(double a){  
  if (a >  180) a -= 360;
  if (a < -180) a += 360;
  return a;
}
//---------------------------------------------------
//ROS pubs and subs----------------------------------
ros::Publisher xy("xy", &str_msg);

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
 
  
  resetEncoders(ENC1, ENC2);

  nh.getHardware()->setBaud(115200);
  nh.initNode();  //Initiate rosserial node
  
  nh.advertise(xy);  //For publishers

  nh.subscribe(cmd_vel);  //For subscribers
  
  pose.x = 0.0;
  pose.y = 0.0;
  pose.th = 0.0;
  tic = millis();
}

void loop()
{
  beginTime = millis();

  // Motor lowlevel control. Using Walber's equation
  pwmL = defaultPWM_L * lin_x - defaultPWM_L * (-ang_z) + ((maxPWM - defaultPWM_L) / 2) * lin_x * sq(ang_z) + ((3*defaultPWM_L - maxPWM) / 2) * (-ang_z) * sq(lin_x);
  pwmR = defaultPWM_R * lin_x + defaultPWM_R * (-ang_z) + ((maxPWM - defaultPWM_R) / 2) * lin_x * sq(ang_z) - ((3*defaultPWM_R - maxPWM) / 2) * (-ang_z) * sq(lin_x);
  
  // Odometry stuff.
  //
  // Angular velocities of wheels computed from encoder ticks
  // omega = Delta Tick * tick2rad / Delta t
  WR_measured = (readEncoder(ENC1) - encM1) * (tick2rad_R/(millis() - tic)) * 1000; // End tic for right angular velocity
  WL_measured = (readEncoder(ENC2) - encM2) * (tick2rad_L/(millis() - tic)) * 1000; // End tic for left angular velocity
  
  // linear on x, and angular on z calculated from w_l and w_r
  lin_x_measured = wheel_rad * (WR_measured + WL_measured) / 2;
  ang_z_measured = wheel_rad * (WR_measured - WL_measured) / wheel_sep;
  
  dt = (millis() - tic)/1000;
  // position calculated from linear on x and angular on z 
  pose.x += (lin_x_measured * 0.01 * cos(pose.th)) * dt;
  pose.y += (lin_x_measured * 0.01 * sin(pose.th)) * dt;
  pose.th += ang_z_measured * dt;
  //pose.th = wrap180(pose.th);
  
  // Reading first encoder measurement before movement
  encM1 = readEncoder(ENC1);
  encM2 = readEncoder(ENC2);
  
  // Tic for computing time between encoder measurements
  tic = millis(); // Start tic to measure instantaneous angular velocity via encoder
  
  // Send cmd to motor
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
  lcd.print(String("t") + String(pose.th));
  lcd.setCursor(15,0);
  lcd.print(F("\xDF"));


  // Preparing chatter message
  //(String(pose.x, 3) + String(",") + String(pose.y, 3) + String(",") + String(pose.th, 3) + String(",") + String(loopTime, 3)).toCharArray(hello, 30);
  (String(pose.x*100, 3) + String(",") + String(pose.y*100, 3) + String(",") + String(pose.th, 3) + String(",") + String(loopTime, 3)).toCharArray(hello, 60);
  //(String(pose.x*100, 3) + String(",") + String(pose.y*100, 3) + String(",") + String(pose.th, 3) + String(",") + String(loopTime, 3) + String(pwmL, 3) + String(",") + String(pwmR, 3)).toCharArray(hello, 60);
  str_msg.data = hello;
  
  // Sending the messages through pubs
  xy.publish( &str_msg );
 
  nh.spinOnce();
  loopTime = (millis() - beginTime); // End tic for loop
}
//---------------------------------------------------
