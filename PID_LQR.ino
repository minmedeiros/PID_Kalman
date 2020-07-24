#include "TimerOne.h"
#include "PID_v1.h"

#define IN_ENCODER 2 // A1
#define OUT_MOTOR 6
#define INIT_POS 1.5708 // 90 degrees
#define KP 58
#define KI 172
#define KD 3.44

// Converted A,B,C,D from continuous to discrete on Matlab for s_time=10ms
float A[3][3] ={{-0.002018,-0.06761,0},{0.02873,0.9628,0},{0.0002757,0.0098,1}};
float B[3] = {0.06655,0.03243,0.0001542};
float Q[3][3] = {{0.02,0,0},{0,0.02,0},{0,0,0.02}};
float R = 0.1;
float LQR[3] = {0.4115,2.484,3.1623}; // calculated via MATLAB

float x_current[3] = {0, 0, 0};
float x_next[3];
float p_current[3][3] = {{0.01,0,0},{0,0.01,0},{0,0,0.01}};
float p_next[3][3];
float K[3];
float temp_p;
float temp_k;
int temp_i;

String strcompare;
bool read_f=false;
int temp=0;
volatile int pulses=0;
bool control = false;
bool pulsed = false;
double pos;
double Output = 0;
uint16_t angle[50];
double Sp[10] = {1.5708*0.1, 1.5708*0.2, 1.5708*0.3,1.5708*0.4, 1.5708*0.5,1.5708*0.6, 1.5708*0.7,1.5708*0.8, 1.5708*0.9,1.5708};
double Setpoint=0;
PID myPID(&pos, &Output, &Setpoint,KP,KI,KD, DIRECT);

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(OUT_MOTOR,OUTPUT);
  pinMode(IN_ENCODER,INPUT);
  attachInterrupt(digitalPinToInterrupt(IN_ENCODER), pulse, RISING);
  Timer1.initialize(10000); // Sets period to 10 mseconds
  Timer1.attachInterrupt(clk); // Calls function "clk"
  digitalWrite(OUT_MOTOR, LOW);
  myPID.SetSampleTime(10); // 10ms
}

void pulse(){
  //pulsed = true;
  if (read_f) pulses++;
}

void clk()
{
  if((read_f)&&(temp<50)){ // temp < 0.5 second
    control=true;    
    /*
    if(temp<10) Setpoint=Sp[temp]; else Setpoint=Sp[9];
    angle[temp]= pulses;
    temp++;
    pos = pulses*0.07854;
    kalman_iteration(Output,pos);
    myPID.Compute();
    Output = Output - (LQR[0]*x_current[0] + LQR[1]*x_current[1] + LQR[2]*x_current[2]);
    Serial.println(Output);
    Output = abs(Output) < 20 ? abs(Output) : 20;
    analogWrite(OUT_MOTOR, Output*255/20); // convert to 0~20V
    */
  } else if(!read_f){
    digitalWrite(OUT_MOTOR, LOW);
    pulses=0;
  }
}

// use kalman for current, velocity and position
float kalman_iteration(float input, float input_pos){
  float input_data[3] = {0,0,input_pos};
  temp_k = 0;
  
  // propagate previous data
  for (int i=0; i<3; i++){
    x_next[i] = A[i][0]*x_current[0] + A[i][1]*x_current[1] + A[i][2]*x_current[2] + B[i]*input;
    // p_next[i] = A*p_current*A' + Q[i]; 
    p_next[i][0]=Q[i][0];
    p_next[i][1]=Q[i][1];
    p_next[i][2]=Q[i][2];

    for (int j=0; j<3; j++){
      temp_p = A[i][0]*p_current[0][j] + A[i][1]*p_current[1][j] + A[i][2]*p_current[2][j];
      p_next[i][0] += temp_p*A[0][j]; // would be A[j][0] but it is transpose so A[0][j]
      p_next[i][1] += temp_p*A[1][j];
      p_next[i][2] += temp_p*A[2][j];
    }

    // calculate forward K = P*H'/(H*P*H'+R) where H=[1; 1; 1], temp_k = H*P*H'
    K[i] = p_next[i][0] + p_next[i][1] + p_next[i][2];
    temp_k += K[i];
  }
  
  // update variables    
  for (int i=0; i<3; i++){
    K[i] = K[i]/(temp_k + R);
    x_current[i] = x_next[i] + K[i]*(input_data[i] - x_next[i]); 
    //p_current[i] = p_next[i] - K[i]*p_next[i];

    for (int j=0; j<3; j++){
      if (i == j) temp_i=1; else temp_i=0; // TESTARRRR
      p_current[i][j] = temp_i - K[i]*(p_next[0][j] + p_next[1][j] + p_next[2][j]);
    }
  }
}

void loop() {
  // flag when there is a pulse on encoder
  /*if (pulsed && read_f) {
    pulses++;
    pulsed = false;   
  }*/

  // flag when should activate controller
  if (control) {
    if(temp<10) Setpoint=Sp[temp]; else Setpoint=Sp[9];
    angle[temp]= pulses;
    temp++;
    pos = pulses*0.0628;
    kalman_iteration(Output,pos);
    myPID.Compute();
    Output = Output - (LQR[0]*x_current[0] + LQR[1]*x_current[1] + LQR[2]*x_current[2]);
    Serial.println(Setpoint);
    Output = Output < 5 ? Output : 5;
    Output = Output > 0 ? Output : 0;
    Serial.println(Output);
    analogWrite(OUT_MOTOR, Output*255/5); // convert to 0~20V
    control=false;
  }

  // case should START control run
  if(Serial.available()>0){
    strcompare = Serial.readStringUntil('\n');
    Serial.println(strcompare);
    if (strcompare == "start"){
      read_f=true;
      myPID.SetMode(AUTOMATIC);
    }
  }
  
  // case control run has ENDED
  if (temp == 50) {
    read_f=false;
    Serial.println("position: ");
    for (int i=0; i<50; i++){
      Serial.println(angle[i]);
    }
    temp=0;
  }  
}
