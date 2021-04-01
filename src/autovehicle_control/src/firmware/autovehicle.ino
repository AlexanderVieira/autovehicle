#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <WiFi.h>
//#include "L298N.h"
#include <Robojax_L298N_DC_motor.h>
#include <Ultrasonic.h>
#include "Adafruit_VL53L0X.h"
#include "ESP32_ISR_Servo.h"
#include "math.h"

// motor 2 settings
#define IN3 26
#define IN4 27
#define ENB 14// this pin must be PWM enabled pin if Arduino board is used
#define CHB 1
// motor 1 settings
#define ENA 25 // this pin must be PWM enabled pin if Arduino board is used
#define IN1 33  //18 motore collegato al contrario
#define IN2 32 //5
#define CHA 0
//conexão dos pinos para o sensor ultrasonico
#define PIN_TRIGGER   4
#define PIN_ECHO      5
//conexão dos pinos para o sensores velocidade
#define PIN_ENCODER_LEFT 17   
#define PIN_ENCODER_RIGHT 15

// Define the servos pin:
#define PIN_SERVO_DIRECAO 12
#define PIN_SERVO_LASER_SCAN 13

#define MOTOR1 1
#define MOTOR2 2

// Interrupções
#define DEBOUNCETIME 10

// ************** ESP32_ISR_Servo ***************

#ifndef ESP32
  #error This code is designed to run on ESP32 platform, not Arduino nor ESP8266! Please check your Tools->Board setting.
#endif

#define TIMER_INTERRUPT_DEBUG       1
#define ISR_SERVO_DEBUG             1

// Select different ESP32 timer number (0-3) to avoid conflict
#define USE_ESP32_TIMER_NO          3

// Published values for SG90 servos; adjust if needed
#define MIN_MICROS      800  //544
#define MAX_MICROS      2400 //2400

//***************** FIM ESP32_ISR_Servo ************

const char* ssid     = "Alexander";
const char* password = "@lerCor#";

// Set the rosserial socket server port
const uint16_t serverPort = 11411;
const float R = 0.032;                    // Wheel Radius 
const int TICK = 20;                      // Encoder total tick
const float RL_WHEEL_DIST = 0.165;        // Distance between two wheels left and right
const float RF_WHEEL_DIST = 0.115;        // Distance between two wheels front and rear
const float LIMITE_ANGULAR_VELOCITY_Z_MAX = 1.0;
const float LIMITE_ANGULAR_VELOCITY_Z_MIN = -1.0;
const int CCW = 2; // do not change
const int CW  = 1; // do not change
float leftWheelSpeed = 0.0;
float rightWheelSpeed = 0.0;
double steeringAngle = 0.0;
float steeringRadians = 0.0;
float vel_back_wheel = 0.0;
unsigned long range_timer = 0;
unsigned long publisher_timer = 0;
unsigned long timeold = 0;
float x_pos = 0.0;                         // Initial X position
float y_pos = 0.0;                         // Initial Y position
float theta = 0.0;                       // Initial Z orientation
float linear_velocity_x = 0.0;
float linear_velocity_y = 0.0;
float angular_velocity_z = 0.0;
unsigned long  deltaTime = 0;
unsigned int num_readings = 5;
int laser_frequency = 40;
//float ranges[num_readings];
//float intensities[num_readings] = {0.0};
int count = 0;

//variáveis para controle dentro da funcção ISR
volatile bool CLOCKWISE = true;
volatile unsigned long debounceTimeoutEncoder = 0;
volatile signed int countEncoder = 0;           // This variable will increase or decrease depending on the rotation of encoder
byte encoderLast;
boolean direcao;
//variáveis para controle dentro do loop
bool saveCW = true;
unsigned long saveDebounceTimeoutEncoder = 0;
unsigned long startDebounceTimeoutEncoder = 0;
int saveCountEncoder = 0;
int totalInterruptCounter;
unsigned long start_time = 0; 

int servoIndex1  = -1;
int servoIndex2  = -1;
bool servoDisable = false;
int posServo = 0;
float posServoLidarRadians = 0.0;

//variaveis que indicam o núcleo
static uint8_t taskCoreZero = 0;
static uint8_t taskCoreOne  = 1;

// Set the rosserial socket server IP address
IPAddress server(192,168,43,119);

// Inicializa H-Bridge
//L298N m1(IN3, IN4, ENB, PWM_CHB, false);
//L298N m2(IN1, IN2, ENA, PWM_CHA, false);
Robojax_L298N_DC_motor motor(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB);

//Inicializa o sensor nos pinos definidos acima
Ultrasonic ultrasonic(PIN_TRIGGER, PIN_ECHO);

Adafruit_VL53L0X laserScan = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

hw_timer_t * timer = NULL;
// usado para desabilitar e interromper interrupções
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

ros::NodeHandle  nh;

//************** FUNCOES CONTROLE ROBOT *************

int toDegrees(double _theta){

  double _thetaDegrees = _theta * 180/PI;
  //Serial.printf("toDegrees(): %lf\n", _thetaDegrees);
  return (int)_thetaDegrees;
}

float toRadians(double _angle){

  double _radians = _angle * PI/180;
  //Serial.printf("toRadians(): %lf\n", _radians);
  return (float)_radians;
}

int toPercent(float _vx){
  _vx = _vx / 0.8;  
  return int(_vx * 100);
}

void forward(float _vx)
{
  Serial.printf("Velocidade Motor: %f\n", _vx);  
  Serial.println("Servo LIDAR desligado.");
  ESP32_ISR_Servos.disable(servoIndex2);

  motor.rotate(MOTOR1, toPercent(_vx), CW); //RIGHT
  motor.rotate(MOTOR2, toPercent(_vx), CCW); //LEFT
    
//  m1.setSpeed(toPercent(_vx)); m2.setSpeed(toPercent(_vx));  
//  m1.forward(); m2.forward();
       
}

void reverse(float _vx){

  Serial.printf("Velocidade Motor: %d\n", _vx);  
  Serial.println("Servo LIDAR desligado.");
  ESP32_ISR_Servos.disable(servoIndex2);
  int _percent_vx = (-1) * toPercent(_vx);

  motor.rotate(MOTOR1, _percent_vx, CCW); //RIGHT
  motor.rotate(MOTOR2, _percent_vx, CW); //LEFT
  
//  m1.setSpeed(_percent_vx); m2.setSpeed(_percent_vx); 
//  m1.reverse(); m2.reverse(); 
    
}

void turnLeft(double _theta){
  
  int _thetaDegress = toDegrees(_theta);  
  Serial.printf("theta: %d\n", _thetaDegress);
  if(_thetaDegress > 30){
    _thetaDegress = 60;
    Serial.printf("turnLeft(): > 30: %d\n", _thetaDegress);   
  }
  else if(_thetaDegress > 0 && _thetaDegress <= 30)
  {
    _thetaDegress = _thetaDegress + 30;
    Serial.printf("turnLeft(): <= 30: %d\n", _thetaDegress);
  }
  else if(_thetaDegress < 0){
    _thetaDegress = fabs(_thetaDegress - 30);
    Serial.printf("turnLeft(): < 0: %d\n", _thetaDegress);
  }
  ESP32_ISR_Servos.setPosition(servoIndex1, _thetaDegress);  
  
}

void turnRight(double _theta){
  
  int _thetaDegress = toDegrees(_theta);
  Serial.printf("theta: %d\n", _thetaDegress);
  if(_thetaDegress < -30){
    _thetaDegress = 0;
    Serial.printf("turnRight(): < -30: %d\n", _thetaDegress);   
  }
  else if(_thetaDegress <= -1 && _thetaDegress >= -30){
    _thetaDegress = _thetaDegress + 30;
    Serial.printf("turnRight(): >= -30: %d\n", _thetaDegress);
  }
  else if(_thetaDegress > 0){
    _thetaDegress = fabs(_thetaDegress - 30);
    Serial.printf("turnRight(): > 0: %d\n", _thetaDegress);
  }
  ESP32_ISR_Servos.setPosition(servoIndex1, _thetaDegress);
  
}

void brake_motor(){

  Serial.println("Freiando...");
  motor.brake(3);   
  
  //m1.brake(); m2.brake();
  
}

void stop_motor(){
  Serial.println("Ponto morto.");
  motor.brake(3);   
  
  //m1.stop(); m2.stop();
  
}

void corrigirDirecao(double _theta){
   int _thetaDegress = toDegrees(_theta);
   _thetaDegress = _thetaDegress + 30;
   ESP32_ISR_Servos.setPosition(servoIndex1, _thetaDegress);
   steeringRadians = _theta;
   Serial.printf("corrigirDirecao(): %d\n", _thetaDegress);
}

double steering(double w, double x) {
  double head = atan2(w, x); // Slope Y, Slope X
  return head;
}

//****************** FIM FUNCOES MOVER **********************

void cmd_velCallback(const geometry_msgs::Twist &cmd_vel)
{
  linear_velocity_x = cmd_vel.linear.x;
  linear_velocity_y = cmd_vel.linear.y;
  angular_velocity_z = cmd_vel.angular.z;

  if(linear_velocity_x > 0.8){

    linear_velocity_x = 0.8;
    
    
  }else if(linear_velocity_x < -0.8){

    linear_velocity_x = -0.8;
  }

  if(angular_velocity_z > LIMITE_ANGULAR_VELOCITY_Z_MAX){

    angular_velocity_z = LIMITE_ANGULAR_VELOCITY_Z_MAX;
    
  }else if(angular_velocity_z < LIMITE_ANGULAR_VELOCITY_Z_MIN){

    angular_velocity_z = LIMITE_ANGULAR_VELOCITY_Z_MIN;
  }
  
  if(angular_velocity_z != 0.0){    
        
    //steeringAngle = atan(double((2 * RF_WHEEL_DIST * sin(angular_velocity_z)) / (2 * RF_WHEEL_DIST * cos(angular_velocity_z)) - (RL_WHEEL_DIST * sin(angular_velocity_z))));
    steeringAngle = steering(double(RF_WHEEL_DIST*angular_velocity_z), double(abs(linear_velocity_x)));
         
  }else{
    
    steeringAngle = 0.0;
  }   
  
  if (linear_velocity_x > 0.0 && steeringAngle == 0.0 )
  {
    Serial.println("Mover para frente.");
    corrigirDirecao(steeringAngle);    
    forward(linear_velocity_x);    
  }
  else if (linear_velocity_x < 0.0 && steeringAngle == 0.0 )
  {
    corrigirDirecao(steeringAngle);       
    reverse(linear_velocity_x);
    Serial.println("Mover para traz.");
  }
  else if (linear_velocity_x > 0.0 && steeringAngle > 0.0 )
  {    
    turnLeft(steeringAngle);
    forward(linear_velocity_x);    
    Serial.println("Mover para frente e girar para esquerda.");
  }
  else if (linear_velocity_x > 0.0 && steeringAngle < 0.0 )
  {
    turnRight(steeringAngle);
    forward(linear_velocity_x);
    Serial.println("Mover para frente e girar para direita.");
  }
  else if (linear_velocity_x < 0.0 && steeringAngle > 0.0)
  {    
    turnRight(steeringAngle);
    reverse(linear_velocity_x);
    Serial.println("Mover para traz e girar para direita.");    
  }
  else if (linear_velocity_x < 0.0 && steeringAngle < 0.0)
  {    
    turnLeft(steeringAngle);
    reverse(linear_velocity_x);
    Serial.println("Mover para traz e girar para esquerda.");
  }  
  else if(linear_velocity_x == 0.0 && steeringAngle > 0.0 )
  {
    Serial.println("Girar para esquerda.");
    angular_velocity_z = 0.0;
    turnLeft(steeringAngle);  
  }
  else if(linear_velocity_x == 0.0 && steeringAngle < 0.0 )
  {
    Serial.println("Girar para direita.");
    angular_velocity_z = 0.0;
    turnRight(steeringAngle);
  }
  else
  {
    Serial.println("Veículo parado...");
    corrigirDirecao(steeringAngle);      
    stop_motor();
    Serial.println("Servo LIDAR ligado."); 
    //ESP32_ISR_Servos.setPosition(servoIndex2, 90);
    delay(200);   
    ESP32_ISR_Servos.enable(servoIndex2);            
          
  }
  
}

ros::Subscriber <geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", cmd_velCallback);

sensor_msgs::LaserScan laser_msg;
ros::Publisher pub_laser("/scan", &laser_msg);
//char frameid_laser_scan[] = "scan";
char laser_scan_link[] = "laser_scan_link";
char pantilt_link[] = "pantilt_link";

sensor_msgs::Range range_msg;
ros::Publisher pub_ultrasound( "/sonar", &range_msg);
//char frameid_ultrasound[] = "sonar";
char front_sonar_link[] = "front_sonar_link";

sensor_msgs::Range ir_range_msg;
ros::Publisher pub_ir_range( "/infrared", &ir_range_msg);
char irFrameid[] = "infrared";

nav_msgs::Odometry odom_msg;
ros::Publisher pub_odometry("/odom", &odom_msg);
char odometry[] = "odom";
char main_mass[] = "main_mass";
char base_link[] = "base_link";
char back_right_wheel_link[] = "back_right_wheel_link";
char back_left_wheel_link[] = "back_left_wheel_link";
char front_right_wheel_link[] = "front_right_wheel_link";
char front_left_wheel_link[] = "front_left_wheel_link";
char front_right_steering_link[] = "front_right_steering_link";
char front_left_steering_link[] = "front_left_steering_link";

geometry_msgs::TransformStamped back_right_wheel_trans;
geometry_msgs::TransformStamped back_left_wheel_trans;
geometry_msgs::TransformStamped front_right_wheel_trans;
geometry_msgs::TransformStamped front_left_wheel_trans;
geometry_msgs::TransformStamped front_right_steering_trans;
geometry_msgs::TransformStamped front_left_steering_trans;
geometry_msgs::TransformStamped sonar_trans;
geometry_msgs::TransformStamped pantilt_trans;
geometry_msgs::TransformStamped laserscan_trans;
geometry_msgs::TransformStamped odom_trans;
geometry_msgs::TransformStamped main_mass_trans;
tf::TransformBroadcaster odom_broadcaster;

void setupMsgSensorUltrassonico(){

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  front_sonar_link;
  range_msg.field_of_view = 0.15f;  // fake
  range_msg.min_range = 0.02f;  // 2 cm
  range_msg.max_range = 4.0f;  // 400 cm

}

void publicarMsgSensorUltrassonico()
{  
  float distancia = obterDistanciaSensorUltrassonico();
  //Serial.print("Distancia Medida Sensor Ultrasonico: ");
  //Serial.println(distancia);
  range_msg.range = distancia/100.0f;
  range_msg.header.stamp = nh.now();  
  pub_ultrasound.publish(&range_msg);
  delay(5);
  
}

float obterDistanciaSensorUltrassonico()
{
    //faz a leitura das informacoes do sensor (em cm)
    float distanciaCM;
    long microsec = ultrasonic.timing();
    distanciaCM = ultrasonic.convert(microsec, Ultrasonic::CM);

    return distanciaCM;
}

void inicializarSensorIRrange(){

  // if initialization failed - write message and freeze
  if (!laserScan.begin()) {
    nh.logwarn("Failed to setup VL53L0X sensor");
    while(1);
  }
  nh.loginfo("VL53L0X API serial node started");
  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  laserScan.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  laserScan.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  laserScan.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  #if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  laserScan.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  laserScan.setMeasurementTimingBudget(200000);
  #endif
}

void setupMsgLaserScan(){

  // Set LaserScan Definition  
  laser_msg.header.frame_id = laser_scan_link;
  laser_msg.angle_min = -1.57;               // start angle of the scan [rad]
  laser_msg.angle_max = 1.57;            // end angle of the scan [rad]
  laser_msg.angle_increment = float(3.14/36);  // angular distance between measurements [rad]
  laser_msg.time_increment = float((1/laser_frequency)/num_readings);  
  //laser_msg.scan_time = float(1/laser_frequency); 
  laser_msg.range_min = 0.03f;               // minimum range value [m]
  laser_msg.range_max = 2.00f;  
  laser_msg.ranges_length = num_readings;
  //laser_msg.ranges.resize(num_readings);
  //laser_msg.intensities.resize(num_readings);  

}

void setupMsgIRrange(){

  ir_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_range_msg.header.frame_id =  irFrameid;
  ir_range_msg.field_of_view = 0.44f; 
  ir_range_msg.min_range = 0.03f; 
  ir_range_msg.max_range = 2.00f; // testar com 1.00m

}

float obterDistanciaIRrange(){

  float distanciaMetro;  
  if (millis() - range_timer){
    
    laserScan.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        distanciaMetro = (float)measure.RangeMilliMeter/1000.0f; // convert mm to m        
    } else {
      nh.logwarn("Out of range"); // if out of range, don't send message      
    }
    range_timer =  millis();    
  }
  return distanciaMetro;
}

void publicarMsgSensorIRrange(){
  
  float distancia = obterDistanciaIRrange();  
  //Serial.print("Distancia Medida Sensor IRrange: ");
  //Serial.println(distancia);
  ir_range_msg.range = distancia;
  ir_range_msg.header.stamp = nh.now();
  pub_ir_range.publish(&ir_range_msg);
  delay(5);  
  
}

void publicarMsgSensorLaserScan(){

  //if (millis() - publisher_timer)
  //{
    //Serial.printf("Delta T publisher Laser Scan: %f\n", millis() - publisher_timer);
    float ranges[num_readings];
    unsigned long scan_start = millis();   
    //generate some fake data for our laser scan
    for(int i = 0; i < num_readings; i++){
      
      ranges[i] = obterDistanciaIRrange();         

    }
    //publisher_timer = millis();  
   
    laser_msg.scan_time = (millis()-scan_start)/1000.0;   
    laser_msg.ranges = ranges;    
    laser_msg.header.stamp = nh.now();    
    pub_laser.publish(&laser_msg);
    memset(ranges, 0, sizeof(ranges));
    delay(5);         
 //}          
  
}

void setupServos(){  
  
  //Serial.print(F("\nStarting ESP32_ISR_MultiServos on ")); Serial.println(ARDUINO_BOARD);
  //Serial.println(ESP32_ISR_SERVO_VERSION);
  
  //Select ESP32 timer USE_ESP32_TIMER_NO
  ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);

  servoIndex1 = ESP32_ISR_Servos.setupServo(PIN_SERVO_DIRECAO, MIN_MICROS, MAX_MICROS);
  servoIndex2 = ESP32_ISR_Servos.setupServo(PIN_SERVO_LASER_SCAN, MIN_MICROS, MAX_MICROS);   

  if (servoIndex1 != -1){
    Serial.println(F("Setup Servo1 OK"));
    ESP32_ISR_Servos.setPosition(servoIndex1, 30);
  }      
  else{
    Serial.println(F("Setup Servo1 failed"));      
  }
  
  if (servoIndex2 != -1){
    
    Serial.println(F("Setup Servo2 OK"));    
    ESP32_ISR_Servos.setPosition(servoIndex2, 90);
    posServoLidarRadians = toRadians(double(90));
    Serial.printf("setupServos() | posServo em radiano: %lf\n", posServoLidarRadians);
    delay(100);
    Serial.println("Servo LIDAR desligado.");    
    ESP32_ISR_Servos.disable(servoIndex2);     
  }
  else{
    Serial.println(F("Setup Servo2 failed")); 
  }
  
}

void inicializarServoLaserScan(){
 
  if ( ( servoIndex2 != -1) && (ESP32_ISR_Servos.isEnabled(servoIndex2)) )
  {
    
    for (posServo = 0; posServo <= 180; posServo += 5)
    {       
      posServoLidarRadians = toRadians(double(posServo));
      //Serial.printf("posServoLidarRadians positivo: %lf\n", posServoLidarRadians);  
          
      ESP32_ISR_Servos.setPosition(servoIndex2, posServo);
      delay(50);     
    }    
    
    delay(1000);

    for (posServo = 180; posServo >= 0; posServo -= 5)
    {
       posServoLidarRadians = toRadians(double(posServo));
      //Serial.printf("posServoLidarRadians positivo: %lf\n", posServoLidarRadians);   
                        
      ESP32_ISR_Servos.setPosition(servoIndex2, posServo);
      delay(50);      
    }
     
    delay(1000);   
      
  }
   
}

void conexaoRosserial(){

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  
  // initializa comunicacao serial:
  nh.initNode();
  odom_broadcaster.init(nh);
  nh.advertise(pub_laser);       
  nh.advertise(pub_ultrasound);
  //nh.advertise(pub_ir_range);
  nh.advertise(pub_odometry);  
  nh.subscribe(sub_cmd_vel);    
  
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
  
}

void IRAM_ATTR handerEncoder_ISR(){

  portENTER_CRITICAL_ISR(&mux);
     
    int leftState = digitalRead(PIN_ENCODER_LEFT);
    if ((encoderLast == LOW) && leftState == HIGH)
    {
      int val = digitalRead(PIN_ENCODER_RIGHT);
      if (val == LOW && digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW)
      {
        direcao = true; //Reverse
      }
      else if (val == HIGH && digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH)
      {
        direcao = false;  //Forward
      }
    }
    encoderLast = leftState;
    if (!direcao)  countEncoder++;
    else  countEncoder--;
    
    debounceTimeoutEncoder = xTaskGetTickCount(); //versão do millis () que funciona a partir da interrupção 
   
  portEXIT_CRITICAL_ISR(&mux);
  
}

//void inicializaTimerInterrupt(){
//  timer = timerBegin(0, 80, true);
//  timerAttachInterrupt(timer, &handerEncoder_ISR, true);
//  timerAlarmWrite(timer, 1000000, true);
//  timerAlarmEnable(timer);
//}

void interrupcaoSensorEncoder(){
  
  //Interrupcao - pinos digitais 15 e 17
  //Aciona o contador a cada pulso
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), handerEncoder_ISR, RISING); 
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), handerEncoder_ISR, RISING);  
 
}

void publicarMsgOdom(){ 

  portENTER_CRITICAL_ISR(&mux); // início da seção crítica
      saveCountEncoder  = countEncoder;
      saveDebounceTimeoutEncoder = debounceTimeoutEncoder;       
      //saveCW = CLOCKWISE;               
  portEXIT_CRITICAL_ISR(&mux); // fim da seção crítica
   
  if((millis() - timeold) >= 1000)
  {      
    detachInterrupt(PIN_ENCODER_LEFT);
    detachInterrupt(PIN_ENCODER_RIGHT);             
    
    //Serial.printf("dt: %lf\n", dt);
    Serial.printf("linear_velocity_x: %lf\n", linear_velocity_x);
    Serial.printf("linear_velocity_y: %lf\n", linear_velocity_y);
    Serial.printf("angular_velocity_z: %lf\n", angular_velocity_z);
    Serial.printf("Angulo de esterçamento: %lf\n", float(steeringAngle));
    Serial.print("SaveDebounceTimeoutEncoder = ");
    Serial.println(saveDebounceTimeoutEncoder);     
    
    
    unsigned long deltaTimeDebounce = (millis() - saveDebounceTimeoutEncoder)/1000;
    deltaTime = (millis() - saveDebounceTimeoutEncoder) % 60;
    Serial.print("deltaTimeDebounce = ");
    Serial.println(deltaTimeDebounce);
    Serial.print("deltaTime = ");
    Serial.println(deltaTime);

    // perímetro da roda trazeira
//    float dist_percorrida = (2 * PI * R) * (saveCountEncoder / (float)TICK);
    
    // velocidade roda trazeira
   if(deltaTime != 0){
     vel_back_wheel = (2 * PI * R) * (saveCountEncoder / (float)TICK) / deltaTime;
   }else{
     vel_back_wheel = 0.0;
   }    
        
//    Serial.printf("Perimetro roda trazeira: %lf\n", travelledDistance);
    Serial.printf("Velocidade roda trazeira: %lf\n", vel_back_wheel);
    Serial.printf("Contador encoder: %d\n", saveCountEncoder);    
          
    float delta_theta = (vel_back_wheel / RF_WHEEL_DIST) * tan(float(steeringAngle));
    float delta_x = vel_back_wheel * cos(theta);    
    float delta_y = vel_back_wheel * sin(theta);             

    //calculate current position of the robot
    x_pos += delta_x;
    y_pos += delta_y;
    theta += delta_theta;    

    Serial.printf("delta_theta: %lf\n", delta_theta);    
    Serial.printf("delta_x: %lf\n", delta_x);
    Serial.printf("delta_y: %lf\n", delta_y);
    Serial.printf("x_pos: %lf\n", x_pos);
    Serial.printf("y_pos: %lf\n", y_pos);
    Serial.printf("theta: %lf\n", theta);

    timeold = millis();
    interrupcaoSensorEncoder();    
    
    portENTER_CRITICAL_ISR(&mux);  //início da seção crítica
     countEncoder = 0; // reseta o contador de interrupção     
    portEXIT_CRITICAL_ISR(&mux); //fim da seção crítica 
            
    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle

    //Massa principal
    main_mass_trans.header.stamp = nh.now();
    main_mass_trans.header.frame_id = base_link;
    main_mass_trans.child_frame_id = main_mass;
    //robot's position in x,y, and z
    main_mass_trans.transform.translation.x = 0.0;
    main_mass_trans.transform.translation.y = 0.0;
    main_mass_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    main_mass_trans.transform.rotation = tf::createQuaternionFromYaw(0); 
    
    //Sonar
    sonar_trans.header.stamp = nh.now();
    sonar_trans.header.frame_id = base_link;
    sonar_trans.child_frame_id = front_sonar_link;
    //robot's position in x,y, and z
    sonar_trans.transform.translation.x = 0.27;
    sonar_trans.transform.translation.y = 0.0;
    sonar_trans.transform.translation.z = 0.025;
    //robot's heading in quaternion
    sonar_trans.transform.rotation = tf::createQuaternionFromYaw(0.0);    

    //LaserScan    
    laserscan_trans.header.stamp = nh.now();
    laserscan_trans.header.frame_id = pantilt_link;
    laserscan_trans.child_frame_id = laser_scan_link;
    //robot's position in x,y, and z
    laserscan_trans.transform.translation.x = 0.0;
    laserscan_trans.transform.translation.y = 0.0;
    laserscan_trans.transform.translation.z = 0.025;
    //robot's heading in quaternion
    laserscan_trans.transform.rotation = tf::createQuaternionFromYaw(0.0);

    //Pantilt       
    pantilt_trans.header.stamp = nh.now();
    pantilt_trans.header.frame_id = base_link;
    pantilt_trans.child_frame_id = pantilt_link;
    //robot's position in x,y, and z
    pantilt_trans.transform.translation.x = 0.0;
    pantilt_trans.transform.translation.y = 0.0;
    pantilt_trans.transform.translation.z = 0.16;
    //robot's heading in quaternion
    pantilt_trans.transform.rotation = tf::createQuaternionFromYaw(posServoLidarRadians);

    //Roda traseira direita
    back_right_wheel_trans.header.stamp = nh.now();
    back_right_wheel_trans.header.frame_id = base_link;
    back_right_wheel_trans.child_frame_id = back_right_wheel_link;
    //robot's position in x,y, and z
    back_right_wheel_trans.transform.translation.x = -0.12;
    back_right_wheel_trans.transform.translation.y = -0.08;
    back_right_wheel_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    back_right_wheel_trans.transform.rotation = tf::createQuaternionFromYaw(0.0);

    //Roda traseira esquerda
    back_left_wheel_trans.header.stamp = nh.now();
    back_left_wheel_trans.header.frame_id = base_link;
    back_left_wheel_trans.child_frame_id = back_left_wheel_link;
    //robot's position in x,y, and z
    back_left_wheel_trans.transform.translation.x = -0.12;
    back_left_wheel_trans.transform.translation.y = 0.08;
    back_left_wheel_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    back_left_wheel_trans.transform.rotation = tf::createQuaternionFromYaw(0.0);

    //Roda dianteira direita
    front_right_wheel_trans.header.stamp = nh.now();
    front_right_wheel_trans.header.frame_id = front_right_steering_link;
    front_right_wheel_trans.child_frame_id = front_right_wheel_link;
    //robot's position in x,y, and z
    front_right_wheel_trans.transform.translation.x = 0.0;
    front_right_wheel_trans.transform.translation.y = 0.0;
    front_right_wheel_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    front_right_wheel_trans.transform.rotation = tf::createQuaternionFromYaw(0.0);

    //Roda dianteira esquerda
    front_left_wheel_trans.header.stamp = nh.now();
    front_left_wheel_trans.header.frame_id = front_left_steering_link;
    front_left_wheel_trans.child_frame_id = front_left_wheel_link;
    //robot's position in x,y, and z
    front_left_wheel_trans.transform.translation.x = 0.0;
    front_left_wheel_trans.transform.translation.y = 0.0;
    front_left_wheel_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    front_left_wheel_trans.transform.rotation = tf::createQuaternionFromYaw(0.0);    

    //Direção direita    
    front_right_steering_trans.header.stamp = nh.now();
    front_right_steering_trans.header.frame_id = base_link;
    front_right_steering_trans.child_frame_id = front_right_steering_link;
    //robot's position in x,y, and z
    front_right_steering_trans.transform.translation.x = 0.17;
    front_right_steering_trans.transform.translation.y = -0.0825;
    front_right_steering_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    front_right_steering_trans.transform.rotation = tf::createQuaternionFromYaw(float(steeringAngle));    

    //Direção esquerda
    front_left_steering_trans.header.stamp = nh.now();
    front_left_steering_trans.header.frame_id = base_link;
    front_left_steering_trans.child_frame_id = front_left_steering_link;
    //robot's position in x,y, and z
    front_left_steering_trans.transform.translation.x = 0.17;
    front_left_steering_trans.transform.translation.y = 0.0825;
    front_left_steering_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    front_left_steering_trans.transform.rotation = tf::createQuaternionFromYaw(float(steeringAngle));    
    
    //Transformação Hodometria
    odom_trans.header.stamp = nh.now();
    odom_trans.header.frame_id = odometry;
    odom_trans.child_frame_id = base_link;
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation = tf::createQuaternionFromYaw(theta);           

    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odometry;
    odom_msg.child_frame_id = base_link;
    //robot's position in x,y, and z
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
    odom_msg.pose.covariance[0] = 0.001;
    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[35] = 0.001;

    //Mensagem Hodometria
    //linear speed from encoders
    odom_msg.twist.twist.linear.x = delta_x;
    odom_msg.twist.twist.linear.y = delta_y;
    odom_msg.twist.twist.linear.z = 0.0;
    //angular speed from encoders
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;    
    odom_msg.twist.twist.angular.z = delta_theta;
    
    odom_msg.twist.covariance[0] = 0.0001;
    odom_msg.twist.covariance[7] = 0.0001;
    odom_msg.twist.covariance[35] = 0.0001;

    odom_broadcaster.sendTransform(back_right_wheel_trans);    
    odom_broadcaster.sendTransform(back_left_wheel_trans);    
    odom_broadcaster.sendTransform(front_right_wheel_trans);   
    odom_broadcaster.sendTransform(front_left_wheel_trans);    
    odom_broadcaster.sendTransform(front_right_steering_trans);    
    odom_broadcaster.sendTransform(front_left_steering_trans);    
    odom_broadcaster.sendTransform(sonar_trans);    
    odom_broadcaster.sendTransform(pantilt_trans);    
    odom_broadcaster.sendTransform(laserscan_trans);    
    odom_broadcaster.sendTransform(main_mass_trans);    
    odom_broadcaster.sendTransform(odom_trans);    
    pub_odometry.publish(&odom_msg); 

    steeringRadians = 0.0;

  }
}

void configurarPinos()
{   
  pinMode(PIN_ENCODER_LEFT, INPUT);
  pinMode(PIN_ENCODER_RIGHT, INPUT);        
}

void conectarWifi(){
  
  Serial.print("Conectando em ");
  Serial.println(ssid);
  // Connect the ESP32 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  
  Serial.begin(115200);  
  configurarPinos();     
  setupServos();
  interrupcaoSensorEncoder();  

  //cria uma tarefa que será executada na função coreTaskServoMotor, com prioridade 1 e execução no núcleo 1
  //coreTaskZero: 
  xTaskCreatePinnedToCore(
                    coreTaskServoMotor,   /* função que implementa a tarefa */
                    "coreTaskServoMotor", /* nome da tarefa */
                    10000,      /* número de palavras a serem alocadas para uso com a pilha da tarefa */
                    NULL,       /* parâmetro de entrada para a tarefa (pode ser NULL) */
                    1,          /* prioridade da tarefa (0 a N) */
                    NULL,       /* referência para a tarefa (pode ser NULL) */
                    taskCoreOne);         /* Núcleo que executará a tarefa */
                    
  delay(250); //tempo para a tarefa iniciar

  //cria uma tarefa que será executada na função coreTaskPub, com prioridade 2 e execução no núcleo 0
  //coreTaskOne: 
  xTaskCreatePinnedToCore(
                    coreTaskPub,   /* função que implementa a tarefa */
                    "coreTaskPub", /* nome da tarefa */
                    10000,      /* número de palavras a serem alocadas para uso com a pilha da tarefa */
                    NULL,       /* parâmetro de entrada para a tarefa (pode ser NULL) */
                    2,          /* prioridade da tarefa (0 a N) */
                    NULL,       /* referência para a tarefa (pode ser NULL) */
                    taskCoreZero);         /* Núcleo que executará a tarefa */

  delay(250); //tempo para a tarefa iniciar

   //cria uma tarefa que será executada na função coreTaskSensor, com prioridade 2 e execução no núcleo 0
   //coreTaskTwo: 
   xTaskCreatePinnedToCore(
                    coreTaskSensor,   /* função que implementa a tarefa */
                    "coreTaskSensor", /* nome da tarefa */
                    10000,      /* número de palavras a serem alocadas para uso com a pilha da tarefa */
                    NULL,       /* parâmetro de entrada para a tarefa (pode ser NULL) */
                    2,          /* prioridade da tarefa (0 a N) */
                    NULL,       /* referência para a tarefa (pode ser NULL) */
                    taskCoreZero);         /* Núcleo que executará a tarefa */
       
}

void loop() {

}

void coreTaskServoMotor( void * pvParameters ){
 
  String taskMessage = "Task running on core: ServoMotor ";
  taskMessage = taskMessage + xPortGetCoreID();

//  m1.begin();
//  m2.begin();
  motor.begin(); 
  
  while(true){
    Serial.println(taskMessage);     
    inicializarServoLaserScan();                   
    delay(500);
  }   
}

void coreTaskPub( void * pvParameters ){

  String taskMessage = "Task running on core: Publisher ";
  taskMessage = taskMessage + xPortGetCoreID(); 
 
  conectarWifi();
  conexaoRosserial();   
  
  while(true){
    if (nh.connected()) {    
      
      Serial.println("Conectado."); 
      Serial.println(taskMessage);   
      publicarMsgSensorUltrassonico();
      //publicarMsgSensorIRrange();      
      publicarMsgSensorLaserScan();
      publicarMsgOdom();           
      //nh.spinOnce();      
      //delay(250);
                                       
    } else {       
      
      Serial.println("Não conectado.");    
      
      if(ESP32_ISR_Servos.isEnabled(servoIndex2)){
        ESP32_ISR_Servos.setPosition(servoIndex2, 90);
        delay(100);
        posServoLidarRadians = toRadians(double(90));
        Serial.printf("posServoLidarRadians zero: %lf\n", posServoLidarRadians);
        ESP32_ISR_Servos.disable(servoIndex2);
        Serial.println("Servo LIDAR desligado.");
        //delay(500); 
      }
      //publicarMsgSensorLaserScan();     
      stop_motor();
      //delay(250);    
    }                
      nh.spinOnce();     
      delay(500);
  } 
}

void coreTaskSensor( void * pvParameters ){

  String taskMessage = "Task running on core: Sensores ";
  taskMessage = taskMessage + xPortGetCoreID();  

  setupMsgSensorUltrassonico();  
  //setupMsgIRrange();
  setupMsgLaserScan();
  inicializarSensorIRrange();   
//  interrupcaoSensorEncoder();
//  inicializaTimerInterrupt();
   
  while(true){
    Serial.println(taskMessage);
    //interrupcaoSensorEncoder();    
    delay(500);
  }  
}
