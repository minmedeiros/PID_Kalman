#include "TimerOne.h"
#include "PID_v1.h"

#define IN_ENCODER 2 // A1
#define OUT_MOTOR 6
#define INIT_POS 1.5708
#define KP 53
#define KI 152.7
#define KD -0.057

// Converted A,B,C,D from continuous to discrete on Matlab for s_time=10ms
float A[3][3] ={{-0.0007,-0.0078,0},{0.085,0.97,0},{0.0008,0.01,1}};
float B[3] = {0.106,0.134,0.0006};
float Q[3][3] = {{0.02,0,0},{0,0.02,0},{0,0,0.02}};
float R = 0.1;
float LQR[3] = {0.7676,3.0195,3.1623}; // calculated via MATLAB

float x_atual[3] = {0, 0, 0};
float x_proximo[3];
float p_atual[3][3] = {{0.01,0,0},{0,0.01,0},{0,0,0.01}};
float p_proximo[3][3];
float K[3];
float temp_p;
float temp_k;
int temp_i;

String strcompare;
bool read_f=false;
int temp=0;
int pulses=0;
bool readmotor = true;
double pos;
double Output = 0;
uint16_t angle[100];
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
  if (read_f) pulses++;
}

void clk()
{
  if((read_f)&&(temp<50)){ // temp < 0.5 second
    angle[temp]= pulses;
    temp++;
    pos = INIT_POS - pulses*0.07854;
    kalman_iteration(Output,pos);
    myPID.Compute();
    Output = Output - (LQR[0]*x_atual[0] + LQR[1]*x_atual[1] + LQR[2]*x_atual[2]);
    Output = abs(Output) < 20 ? abs(Output) : 20;
    analogWrite(OUT_MOTOR, Output*255/20); // convert to 0~20V
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
    x_proximo[i] = A[i][0]*x_atual[0] + A[i][1]*x_atual[1] + A[i][2]*x_atual[2] + B[i]*input;
    // p_proximo[i] = A*p_atual*A' + Q[i]; 
    p_proximo[i][0]=Q[i][0];
    p_proximo[i][1]=Q[i][1];
    p_proximo[i][2]=Q[i][2];

    for (int j=0; j<3; j++){
      temp_p = A[i][0]*p_atual[0][j] + A[i][1]*p_atual[1][j] + A[i][2]*p_atual[2][j];
      p_proximo[i][0] += temp_p*A[0][j]; // would be A[j][0] but it is transpose so A[0][j]
      p_proximo[i][1] += temp_p*A[1][j];
      p_proximo[i][2] += temp_p*A[2][j];
    }

    // adiantar calculo de K = P*H'/(H*P*H'+R) mas H=[1; 1; 1], temp_k = H*P*H'
    K[i] = p_proximo[i][0] + p_proximo[i][1] + p_proximo[i][2];
    temp_k += K[i];
  }
  
  // update variables    
  for (int i=0; i<3; i++){
    K[i] = K[i]/(temp_k + R);
    x_atual[i] = x_proximo[i] + K[i]*(input_data[i] - x_proximo[i]); 
    //p_atual[i] = p_proximo[i] - K[i]*p_proximo[i];

    for (int j=0; j<3; j++){
      if (i == j) temp_i=1; else temp_i=0; // TESTARRRR
      p_atual[i][j] = temp_i - K[i]*(p_proximo[0][j] + p_proximo[1][j] + p_proximo[2][j]);
    }
  }
}

void loop() {
  if(Serial.available()>0){
    strcompare = Serial.readStringUntil('\n');
    Serial.println(strcompare);
    if (strcompare == "start"){
      read_f=true;
      myPID.SetMode(AUTOMATIC);
    }
  }
  if (temp == 50) {
    read_f=false;
    Serial.println("posicao: ");
    for (int i=0; i<50; i++){
      Serial.println(angle[i]);
    }
    temp=0;
  }  
}
