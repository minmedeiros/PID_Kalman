#include "TimerOne.h"
#include "PID_v1.h"

#define IN_ENCODER 2 // A1
#define OUT_MOTOR 6
#define INIT_POS 1.5708 // 90 degrees
#define KP 28.45
#define KI 46.91
#define KD 3.735 // -0.09

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
    myPID.Compute();
    Serial.println(Output*255/10);
    Output = abs(Output) < 10 ? abs(Output) : 10;
    analogWrite(OUT_MOTOR, Output*255/10); // convert to 0~20V
    */
  } else if(!read_f){
    digitalWrite(OUT_MOTOR, LOW);
    pulses=0;
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
    myPID.Compute();
    Serial.println(Setpoint);
    Output = Output < 5 ? Output : 5;
    Output = Output > 0 ? Output : 0;
    Serial.println(Output);
    analogWrite(OUT_MOTOR, Output*255/5); // convert to 0~5V
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
