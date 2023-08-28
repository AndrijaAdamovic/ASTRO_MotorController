#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <Adafruit_INA260.h>

#define ENC1_A 23
#define ENC1_B 22

#define ENC2_A 21 
#define ENC2_B 20

#define INA 12
#define INA_ 11
#define PWMA 10

#define INB 4
#define INB_ 2
#define PWMB 3

#define CPR 6533
#define RPM_COUNTER_INTERVAL_MS 10
#define UART Serial
#define UART_BAUD 115200
#define PWM_FREQ 50000

#define MAX_RPM 100
#define WHEEL_RADIUS 0.0360
#define WHEEL_SEPARATION 0.2932

#define ODOM_INTERVAL_MS 10
#define CMD_VEL_TIMEOUT_MS 5000

#define pi 3.14159265

//For PID calculation
double targetRPM1, outPWM1, targetRPM2, outPWM2;

//Timer for RPM calculation
IntervalTimer rpmTimer;

//Encoder variables
volatile int64_t encCnt[] = {0, 0};
volatile int64_t encCntPub[] = {0, 0};
volatile int64_t lastCnt[] = {0, 0};

//Current RPMs
double currentRPM1, currentRPM2;

//Wheel angular velocities
double w_r, w_l;

//Odometry variables
double o_x, o_y, o_th, o_vx, o_vy, o_vth;
int64_t lastCntOdom[] = {0, 0}; //Last encoder ticks, for odometry
int64_t lastMsOdom;

int64_t lastCmdVelMs;

//PID constants
double Kp = 2;
double Ki = 100;
double Kd = 0;

PID mPID1(&currentRPM1, &outPWM1, &targetRPM1, Kp, Ki, Kd, DIRECT);
PID mPID2(&currentRPM2, &outPWM2, &targetRPM2, Kp, Ki, Kd, DIRECT);

ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
sensor_msgs::JointState jstate;

char *joint_name[] = {"left_wheel_joint", "right_wheel_joint"};
float vel[]={0,0};
float pos[]={0,0};
float eff[]={0,0};

Adafruit_INA260 ina260_left = Adafruit_INA260();
Adafruit_INA260 ina260_right = Adafruit_INA260();

//assigning the arrays to the message

void messageCb(const geometry_msgs::Twist& msg)
{
  float angular_speed = msg.angular.z;
  float linear_speed = msg.linear.x;

  w_r = (linear_speed/WHEEL_RADIUS) + ((angular_speed*WHEEL_SEPARATION)/(2.0*WHEEL_RADIUS));
  w_l = (linear_speed/WHEEL_RADIUS) - ((angular_speed*WHEEL_SEPARATION)/(2.0*WHEEL_RADIUS));

  lastCmdVelMs = millis();
}

ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", &messageCb);
ros::Publisher odom_pub("odom", &odom_msg);
ros::Publisher js_pub("joint_states", &jstate);
tf::TransformBroadcaster odom_broadcaster;

//angular velocity(rad/s) to RPM
inline double wToRPM (double w) { return (w*(60/(2*pi))); }

void readEnc1A()
{
  if(abs(encCntPub[0]) >= CPR)
    encCntPub[0] = 0;
  if(digitalRead(ENC1_A) != digitalRead(ENC1_B))
  {
    encCnt[0]++;
    encCntPub[0]++;
  }
  else
  {
    encCnt[0]--;
    encCntPub[0]--;
  }
}



void readEnc1B()
{
  if(abs(encCntPub[0]) >= CPR)
    encCntPub[0] = 0;
  if(digitalRead(ENC1_A) == digitalRead(ENC1_B))
  {
    encCnt[0]++;
    encCntPub[0]++;
  }
  else
  {
    encCnt[0]--;
    encCntPub[0]--;
  }
}

void readEnc2A()
{
  if(abs(encCntPub[1]) >= CPR)
    encCntPub[1] = 0;
  if(digitalRead(ENC2_A) != digitalRead(ENC2_B))
  {
    encCnt[1]++;
    encCntPub[1]++;
  }
  else
  {
    encCnt[1]--;
    encCntPub[1]--;
  }
}

void readEnc2B()
{
  if(abs(encCntPub[1]) >= CPR)
    encCntPub[1] = 0;
  if(digitalRead(ENC2_A) == digitalRead(ENC2_B))
  {
    encCnt[1]++;
    encCntPub[1]++;
  }
  else
  {
    encCnt[1]--;
    encCntPub[1]--;
  }
}


void motorDrive(int pwm1, int pwm2)
{
  pwm1 = constrain(pwm1, -255, 255);
  pwm2 = constrain(pwm2, -255, 255);
  
  if (pwm1 > 0)
  {
    digitalWrite(INA, 0);
    digitalWrite(INA_, 1);
    analogWrite(PWMA, abs(pwm1));
  }
  else if (pwm1 <= 0)
  {
    digitalWrite(INA, 1);
    digitalWrite(INA_, 0);
    analogWrite(PWMA, abs(pwm1));
  }
  if (pwm2 > 0)
  {
    digitalWrite(INB, 1);
    digitalWrite(INB_, 0);
    analogWrite(PWMB, abs(pwm2));
  }
  else if (pwm2 <= 0)
  {
    digitalWrite(INB, 0);
    digitalWrite(INB_, 1);
    analogWrite(PWMB, abs(pwm2));
  }
}

void getRPM()
{
  currentRPM1 = (int64_t)abs(encCnt[0] - lastCnt[0]);
  currentRPM1 /= 0.000167;
  currentRPM1 /= CPR;
  lastCnt[0] = encCnt[0];

  currentRPM2 = (int64_t)abs(encCnt[1] - lastCnt[1]);
  currentRPM2 /= 0.000167;
  currentRPM2 /= CPR;
  lastCnt[1] = encCnt[1];
}


void setup() {
  // put your setup code here, to run once:
  pinMode(ENC1_A, 0);
  pinMode(ENC1_B, 0);
  pinMode(ENC2_A, 0);
  pinMode(ENC2_B, 0);
  pinMode(INA, 1);
  pinMode(INA_, 1);
  pinMode(PWMA, 1);
  pinMode(INB, 1);
  pinMode(INB_, 1);
  pinMode(PWMB, 1);
  pinMode(13, 1);

  analogWriteFrequency(PWMA, PWM_FREQ);
  analogWriteFrequency(PWMB, PWM_FREQ);

  digitalWrite(13, 1);
  Serial.begin(115200);
  UART.begin(UART_BAUD);

  attachInterrupt(ENC1_A, readEnc1A, CHANGE);
  attachInterrupt(ENC1_B, readEnc1B, CHANGE);
  attachInterrupt(ENC2_A, readEnc2A, CHANGE);
  attachInterrupt(ENC2_B, readEnc2B, CHANGE);

  rpmTimer.begin(getRPM, RPM_COUNTER_INTERVAL_MS*1000);

  mPID1.SetOutputLimits(0, 255);
  mPID1.SetMode(AUTOMATIC);

  mPID2.SetOutputLimits(0, 255);
  mPID2.SetMode(AUTOMATIC);

  ina260_left.begin();
  ina260_right.begin(0x41);
  ina260_left.setAveragingCount(INA260_COUNT_16);
  ina260_right.setAveragingCount(INA260_COUNT_16);
  ina260_left.setCurrentConversionTime(INA260_TIME_332_us);
  ina260_right.setCurrentConversionTime(INA260_TIME_332_us);

  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(vel_sub);
  odom_broadcaster.init(nh);
  nh.advertise(odom_pub);
  nh.advertise(js_pub);
}

void motorDriveRPM(double rpm1, double rpm2)
{
  rpm1 = constrain(rpm1, -MAX_RPM, MAX_RPM);
  rpm2 = constrain(rpm2, -MAX_RPM, MAX_RPM);

  targetRPM1 = abs(rpm1);
  targetRPM2 = abs(rpm2);

  mPID1.Compute();
  mPID2.Compute();

  motorDrive(rpm1 > 0 ? outPWM1 : -outPWM1, rpm2 > 0 ? outPWM2 : -outPWM2);
}

void jointStatesPublisher(int64_t cnt1, int64_t cnt2, float w1, float w2)
{

  pos[0] = cnt1 * ((2*PI)/CPR);
  pos[1] = cnt2 * ((2*PI)/CPR);

  vel[0] = w1;
  vel[1] = w2;
  
  eff[0] = ina260_left.readCurrent();
  eff[1] = ina260_right.readCurrent();
  
  jstate.header.stamp = nh.now();

  //assigning the arrays to the message
  jstate.name=joint_name;
  jstate.position=pos;
  jstate.velocity=vel;
  jstate.effort=eff;

  //setting the length
  jstate.name_length=2;
  jstate.position_length=2;
  jstate.velocity_length=2;
  jstate.effort_length=2;

  js_pub.publish(&jstate);
}

void odometry_and_jstates()
{
  if(millis() - lastMsOdom >= ODOM_INTERVAL_MS)
  {
    double dt = ODOM_INTERVAL_MS / 1000.0;

    double w_l_o = ((encCnt[0] - lastCntOdom[0])*((2*PI)/CPR))/dt;
    double w_r_o = ((encCnt[1] - lastCntOdom[1])*((2*PI)/CPR))/dt;

    lastCntOdom[0] = encCnt[0];
    lastCntOdom[1] = encCnt[1];

    o_vx = (WHEEL_RADIUS / 2) * (w_r_o + w_l_o);
    o_vy = 0;
    o_vth = (WHEEL_RADIUS/WHEEL_SEPARATION)*(w_r_o - w_l_o);

    double delta_x = o_vx * cos(o_th) * dt;
    double delta_y = o_vx * sin(o_th) * dt;
    double delta_th = o_vth * dt;

    o_x += delta_x;
    o_y += delta_y;
    o_th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(o_th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = nh.now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = o_x;
    odom_trans.transform.translation.y = o_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x = o_x;
    odom_msg.pose.pose.position.y = o_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.child_frame_id = "base_footprint";
    odom_msg.twist.twist.linear.x = o_vx;
    odom_msg.twist.twist.angular.z = o_vth;

    odom_pub.publish(&odom_msg);

    jointStatesPublisher(encCnt[0], encCnt[1], w_l_o, w_r_o);

    lastMsOdom = millis();
  }  
}

void loop() 
{
  #ifdef CMD_VEL_TIMEOUT_MS
    if(millis() - lastCmdVelMs >= CMD_VEL_TIMEOUT_MS)
      motorDrive(0, 0);
    else
      motorDriveRPM(wToRPM(w_l), wToRPM(w_r));
  #else
    motorDriveRPM(wToRPM(w_l), wToRPM(w_r));
  #endif

  odometry_and_jstates();
  nh.spinOnce();
}
