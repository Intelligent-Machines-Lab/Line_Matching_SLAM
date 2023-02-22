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
Encoder ENC1(18, 19);//Best Performance both pins have interrupt capability
const int EN_R = 9;//connected to Arduino's port 6 (PWM Speed Regulation)
const int IN1 = 7;//connected to Arduino's port 5 (Direction of Rotation)
const int IN2 = 8;//connected to Arduino's port 4 (Direction of Rotation)
Encoder ENC2(2, 3);//Best Performance both pins have interrupt capability
const int EN_L = 6;//connected to Arduino's port 6 (PWM Speed Regulation)
const int IN3 = 5;//connected to Arduino's port 5 (Direction of Rotation)
const int IN4 = 4;//connected to Arduino's port 4 (Direction of Rotation)

LiquidCrystal_I2C lcd(0x27,16,2);

long oldPosition  = -999;

long encM1, encM2; 

char hello[30], enc_str[15];

ros::NodeHandle  nh;

std_msgs::String str_msg, encm_msg;
std_msgs::Float32 encr_msg, encl_msg, ukl_msg, wl_msg, ukr_msg, wr_msg;

geometry_msgs::TransformStamped t_imu;
geometry_msgs::TransformStamped t_laser;
tf::TransformBroadcaster broadcaster;
tf::TransformBroadcaster broadcaster2;

tf::TransformBroadcaster broadcaster3;
nav_msgs::Odometry odom_msg;
geometry_msgs::TransformStamped t_base;


char odom[] = "/odom";
char base_link[] = "/base_link";
char imu[] = "/imu";
char laser[] = "/laser";

int defaultPWM = 200;
const int maxPWM = 255;
int pwmL, pwmR;
const double wheel_rad = 0.05, wheel_sep = 0.275;
double ang_z = 0, lin_x = 0, lin_y = 0;
double w_r = 0, w_l = 0;

struct Quaternion{
    double w, x, y, z;
};

struct Pose{
    double x, y, th;
};

Quaternion quat, quatOdom;
Pose pose;

unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0, readyToGo = 0;
//double a[3] = {0, 0, 0};
//double w[3] = {0, 0, 0};
double angle[3] = {0, 0, 0};
//double mag[3] = {0, 0, 0};
//double quaternion[4];
double headDegrees = 0;

const double tickL = 2*3.141592/2017, tickR = 2*3.141592/2062;
double WR_measured, WL_measured, lin_x_measured, ang_z_measured;
double tic;

const int delayTime = 200;
double dt = 0.1;

const double deg2rad = 3.141592/180; // Converts degrees to radians

unsigned long beginTime = 0, loopTime = 0;

// Parameters initialized with prior training on MATLAB
/*
double mx_bias = -100, my_bias = 20, mx_ganhoescala = 34, my_ganhoescala = 34;
double A[3][3] = {{0.0187, 0.0068, -0.5854}, {0, 0.0112, -0.3680}, {0, 0, 0.0309}};
double v[3] = {-60.1715, -38.4551, 77.0811};
double max_mx = -999, min_mx = 999, max_my = -999, min_my = 999;
*/
unsigned char calibDone = 1;
unsigned int iter = 0;

double uk, uk1, uk2, ek1, ek2, ukk, ukk1, ukk2, ekk1, ekk2, Ta;

unsigned char control = 0;
//---------------------------------------------------
//Functions definitions------------------------------
void messageCb( const std_msgs::Empty& toggle_msg ){
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void codeCb( const std_msgs::Int32& code_msg ){
  int code = code_msg.data;
  switch(code){
    case 0:
      calibDone = 0;
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
  w_r = (lin_x/wheel_rad) + ((ang_z*wheel_sep)/(2.0*wheel_rad));
  w_l = (lin_x/wheel_rad) - ((ang_z*wheel_sep)/(2.0*wheel_rad));
}

void motorRCb(const std_msgs::UInt8& msg){
  pwmR = msg.data;
}

void motorLCb(const std_msgs::UInt8& msg){
  pwmL = msg.data;
}
/*
void kpCb(const std_msgs::Float32& msg){
  Kp = msg.data;
}
void kiCb(const std_msgs::Float32& msg){
  Ki = msg.data;
}
*/

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
    
    //char inChar = (char)Serial.read(); Serial.print(inChar); //Output Original Data, use this code 

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
          /*
          case 0x51:
            a[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0 * 16 * 9.8;
            a[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0 * 16 * 9.8;
            a[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0 * 16 * 9.8;
            //T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0 + 36.25;
            break;
          case 0x52:
            w[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0 * 2000;
            w[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0 * 2000;
            w[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0 * 2000;
            //T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0 + 36.25;
            break;
            */
          case 0x53:
            angle[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0 * 180;
            angle[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0 * 180;
            angle[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0 * 180;
            //T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0 + 36.25;
            readyToGo = 1;
            break;
            /*
          case 0x59:
            quaternion[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0;
            quaternion[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0;
            quaternion[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0;
            quaternion[3] = (short(Re_buf [9]<<8| Re_buf [8]))/32768.0;
            readyToGo = 1;
            break;
            */
        }
     }
  }
}
/*
double headingFromMag(double magX, double magY){
  magX = ((magX - mx_bias) * 100) / mx_ganhoescala;
  magY = ((magY - my_bias) * 100) / my_ganhoescala;
  
  double heading = atan2(magY, magX);

  //double declinacao = -21.3;
  //heading = heading + declinacao;
  
  if(heading < 0)
    heading = heading + 2*3.14159;
  if(heading > 2*3.14159)
    heading = heading - 2*3.14159;
    
  double headingGraus = 360 - heading * 180/3.14159;
  
  return headingGraus;
}

double calib2(void){
  double x, y, yaw;
  x = A[0][0]*(mag[0] - v[0]) + A[0][1]*(mag[1] - v[1]) + A[0][2]*(mag[2] - v[2]);
  y = A[1][1]*(mag[1] - v[1]) + A[1][2]*(mag[2] - v[2]);

  yaw = atan2(-y, x);
  yaw /= deg2rad;
  return yaw;
}

void calibrationIMU(){
  
  double magnetom_x, magnetom_y;
  
  MotorR(defaultPWM); MotorL(-defaultPWM);

  magnetom_x = mag[0];
  magnetom_y = mag[1];

  if(magnetom_x > max_mx) max_mx = magnetom_x;
  if(magnetom_x < min_mx) min_mx = magnetom_x;
  if(magnetom_y > max_my) max_my = magnetom_y;
  if(magnetom_y < min_my) min_my = magnetom_y;

  iter++;
  
  if(iter > 300){
    mx_bias = (max_mx + min_mx) / 2;
    my_bias = (max_my + min_my) / 2;
    mx_ganhoescala = (max_mx - min_mx) / 2;
    my_ganhoescala = (max_my - min_my) / 2;  
    calibDone = 1;
    iter = 0;
  }
}
*/
/*
double PIdigital(double ekk){
  //Using backward Euler
  ek = ekk;
  uk = (Kp + Ki*Ta)*ek - Kp*ek1 + uk1;
  uk = min(maxPWM, uk);
  uk = max(-maxPWM, uk);
  ek1 = ek; uk1 = uk;
  
  return uk1;
}

double PID_Tustin(double ek){
  // Using Tustin's approximation
  uk = uk2 + Kp*(ek - ek2) + (Ta*Ki/2) * (ek + 2*ek1 + ek2) + (2*Kd/Ta) * (ek - 2*ek1 + ek2);
  uk = min(maxPWM, max(-maxPWM, uk));
  ek2 = ek1;
  ek1 = ek;
  uk2 = uk1;
  uk1 = uk;

  return uk;
}
*/


double PID_backward_r(double ekk, double Kp, double Ki, double Kd){
  // Using backwards euler
  ukk = ukk1 + Kp*(ekk - ekk1) + ekk*Ta*Ki + (Kd/Ta) * (ekk - 2*ekk1 + ekk2);
  ukk = min(maxPWM, max(-maxPWM, ukk));
  ekk2 = ekk1;
  ekk1 = ekk;
  ukk1 = ukk;

  return ukk;  
}
double PID_backward_l(double ek, double Kp, double Ki, double Kd){
  // Using backwards euler
  uk = uk1 + Kp*(ek - ek1) + ek*Ta*Ki + (Kd/Ta) * (ek - 2*ek1 + ek2);
  uk = min(maxPWM, max(-maxPWM, uk));
  ek2 = ek1;
  ek1 = ek;
  uk1 = uk;

  return uk;  
}

double PI_backward_l(double ek, double Kp, double Ki){
  // Using backwards euler
  uk = uk1 + Kp*(ek - ek1) + ek*Ta*Ki;
  uk = min(maxPWM, max(-maxPWM, uk));
  ek1 = ek;
  uk1 = uk;

  return uk;  
}

double PI_backward_r(double ekk, double Kp, double Ki){
  // Using backwards euler
  ukk = ukk1 + Kp*(ekk - ekk1) + ekk*Ta*Ki;
  ukk = min(maxPWM, max(-maxPWM, ukk));
  ekk1 = ekk;
  ukk1 = ukk;

  return ukk;  
}

double PI_discretized_r(double ekk, double Kp, double Ki){
  // Using c2d on MATLAB with zoh as method
  ukk = ukk1 + Kp*(ekk - ekk1) + ekk1*Ta*Ki;
  ukk = min(maxPWM, max(-maxPWM, ukk));
  ekk1 = ekk;
  ukk1 = ukk;

  return ukk;  
}

double PI_discretized_l(double ek, double Kp, double Ki){
  // Using c2d on MATLAB with zoh as method
  uk = uk1 + Kp*(ek - ek1) + ek1*Ta*Ki;
  uk = min(maxPWM, max(-maxPWM, uk));
  ek1 = ek;
  uk1 = uk;

  return uk;  
}


//---------------------------------------------------
//ROS pubs and subs----------------------------------
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher enc_r("enc_r", &encr_msg);
ros::Publisher enc_l("enc_l", &encl_msg);
ros::Publisher uk_l("uk_l", &ukl_msg);
ros::Publisher wl_calc("wl_calc", &wl_msg);
ros::Publisher uk_r("uk_r", &ukr_msg);
ros::Publisher wr_calc("wr_calc", &wr_msg);
ros::Publisher odom_pub("odom", &odom_msg);
//ros::Publisher enc_m("encoder_measured", &encm_msg);

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
ros::Subscriber<std_msgs::Int32> code_sub("code_sub", &codeCb );
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &cmdCb );
ros::Subscriber<std_msgs::UInt8> motor_l("motor_l", &motorLCb );
ros::Subscriber<std_msgs::UInt8> motor_r("motor_r", &motorRCb );
/*
ros::Subscriber<std_msgs::Float32> k_p("k_p", &kpCb );
ros::Subscriber<std_msgs::Float32> k_i("k_i", &kiCb );
*/
//---------------------------------------------------
//Setup and loop start from here---------------------
void setup()
{
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
  //Serial2.begin(921600);
  
  resetEncoders(ENC1, ENC2);

  nh.getHardware()->setBaud(115200);
  nh.initNode();  //Initiate rosserial node
  nh.advertise(chatter);  //For publishers
  nh.advertise(enc_r);  //For publishers
  nh.advertise(enc_l);  //For publishers
  nh.advertise(uk_l);  //For publishers
  nh.advertise(wl_calc);  //For publishers
  nh.advertise(uk_r);  //For publishers
  nh.advertise(wr_calc);  //For publishers
  nh.advertise(odom_pub);  //For publishers
  //nh.advertise(enc_m);  //For publishers
  
  nh.subscribe(sub);  //For subscribers
  nh.subscribe(cmd_vel);  //For subscribers
  nh.subscribe(code_sub);  //For subscribers
  nh.subscribe(motor_l);  //For subscribers
  nh.subscribe(motor_r);  //For subscribers
  /*
  nh.subscribe(k_p);  //For subscribers
  nh.subscribe(k_i);  //For subscribers
  */
  broadcaster.init(nh);
  broadcaster2.init(nh);
  broadcaster3.init(nh);

  pose.x = 0.0;
  pose.y = 0.0;
  pose.th = 0.0;

  ek1 = 0;
  ek2 = 0;
  uk1 = 0;
  uk2 = 0;
  ekk1 = 0;
  ekk2 = 0;
  ukk1 = 0;
  ukk2 = 0;
  Ta = 0.1;

  
}

void loop()
{
  beginTime = millis();
  
  // IMU stuff
  decodeIMU();
//  readyToGo = 1;
  if(readyToGo){
    /*
    if(!calibDone){
      calibrationIMU();
    }
    */
    // Preparing encoder message
    encM1 = readEncoder(ENC1);
    encM2 = readEncoder(ENC2);
    encr_msg.data = WR_measured;
    encl_msg.data = WL_measured;
    wl_msg.data = w_l;
    wr_msg.data = w_r;
    
    if(control){
      // PID Controller
      pwmL = PID_backward_l(w_l - WL_measured, 3.1822, 124.7961, 0.020285); //10.8, 36, 0.81
      pwmR = PID_backward_r(w_r - WR_measured, 14.4774, 124.4512, 0.42104); //12.5, 41.6, 0.935

      // PI Controller
      //pwmL = PI_backward_l(w_l - WL_measured, 47.2, 89.9);
      //pwmR = PI_backward_r(w_r - WR_measured, 22.9, 126);
      
      // PI Controller discretized
      //pwmL = PI_discretized_l(w_l - WL_measured, 57.1199, 713.1154);
      //pwmR = PI_discretized_r(w_r - WR_measured, 56.1549, 466.2291);
    } else{  
      // Motor lowlevel control. Using Walber's equation
      pwmL = defaultPWM * lin_x - defaultPWM * ang_z + ((maxPWM - defaultPWM) / 2) * lin_x * sq(ang_z) + ((3*defaultPWM - maxPWM) / 2) * ang_z * sq(lin_x);
      pwmR = defaultPWM * lin_x + defaultPWM * ang_z + ((maxPWM - defaultPWM) / 2) * lin_x * sq(ang_z) - ((3*defaultPWM - maxPWM) / 2) * ang_z * sq(lin_x);
    }
    
    ukl_msg.data = (((double)pwmL)/((double)maxPWM))*6.0;
    ukr_msg.data = (((double)pwmR)/((double)maxPWM))*6.0;
    
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

    // Preparing encoder_measured message
    //(String(WL_measured, 3) + String(",") + String(WR_measured, 3)).toCharArray(enc_str, 15);
    //encm_msg.data = enc_str;
    
    if(calibDone){
      //headDegrees = headingFromMag(mag[0], mag[1]);
      //headDegrees = calib2();
      if(abs(angle[2]-headDegrees) <= 50)
        headDegrees = angle[2];
        
      quat = ToQuaternion(headDegrees, 0, 0);
      /*
      quat.w = quaternion[0];
      quat.x = quaternion[1];
      quat.y = quaternion[2];
      quat.z = quaternion[3];
      */
    }else{
      quat = ToQuaternion(0, 0, 0);
    }
    // IMU frame transformation
    t_imu.header.frame_id = base_link;
    t_imu.child_frame_id = imu;
    t_imu.transform.translation.z = 0.203; // ATENTION HERE
    t_imu.transform.rotation.x = 0;
    t_imu.transform.rotation.y = 0;
    t_imu.transform.rotation.z = 0;
    t_imu.transform.rotation.w = 1;
    t_imu.header.stamp = nh.now();

    // Lidar frame transformation
    
    //quatLidar = ToQuaternion(-90, 0, 0);
    /*
    t_laser.header.frame_id = base_link;
    t_laser.child_frame_id = laser;
    t_laser.transform.translation.z = 0.3208; // ATENTION HERE
    t_laser.transform.rotation.x = 0;
    t_laser.transform.rotation.y = 0;
    t_laser.transform.rotation.z = -0.7071;
    t_laser.transform.rotation.w = 0.7071;
    t_laser.header.stamp = nh.now();
    */
    // Base link frame transformation
    t_base.header.frame_id = odom;
    t_base.child_frame_id = base_link;
    t_base.transform.translation.x = pose.x;
    t_base.transform.translation.y = pose.y;
    t_base.transform.translation.z = 0.0;
    t_base.transform.rotation.x = quat.x;//quaternion[1];
    t_base.transform.rotation.y = quat.y;//quaternion[2];
    t_base.transform.rotation.z = quat.z;//quaternion[3];
    t_base.transform.rotation.w = quat.w;//quaternion[0];
    t_base.header.stamp = nh.now();

    // Sending transformations
    broadcaster.sendTransform(t_imu);
    //broadcaster2.sendTransform(t_laser);
    broadcaster3.sendTransform(t_base);

    // Preparing Odom message
    //quatOdom = ToQuaternion((pose.th/deg2rad), 0, 0);
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


    // Preparing chatter message
    //(String(mag[0], 3) + String(",") + String(mag[1], 3) + String(",") + String(mag[2], 3)).toCharArray(hello, 30);
    (String(loopTime)+ String(",") + String(lin_x_measured, 3) + String(",") + String(ang_z_measured, 3) + String(",") + String(pose.x, 3) + String(",") + String(pose.y, 3)).toCharArray(hello, 30);
    str_msg.data = hello;
    
    
    // Sending the messages through pubs
    chatter.publish( &str_msg );
    enc_r.publish( &encr_msg );
    enc_l.publish( &encl_msg );
    //enc_m.publish( &encm_msg );
    wl_calc.publish( &wl_msg );
    uk_l.publish( &ukl_msg );
    wr_calc.publish( &wr_msg );
    uk_r.publish( &ukr_msg );
    
    odom_pub.publish( &odom_msg );
   
    nh.spinOnce();
    while(((millis() -  beginTime) <= (delayTime - loopTime)) && (loopTime < delayTime)); // Hodor until 300 ms has passed
    readyToGo = 0;
    loopTime = 0;
  
  } else{
    
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
