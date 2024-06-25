#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 3;
uint16_t sensorValues[SensorCount];

int EN1 = 6;
int EN2 = 5;  //Roboduino Motor shield uses Pin 9
int IN1 = 7;
int IN2 = 4; //Latest version use pin 4 instead of pin 8

uint16_t closeWhiteValue;



/*float Kp = 0.4;
float Ki = 0;
float Kd = 0.6;*/

/*float Kp = 0.1;
float Ki = 0.3;
float Kd = 0;*/

int minSpeed = -255;
int maxSpeed = 255;

void Motor(float pwm, boolean reverse, int EN, int IN){
  analogWrite(EN, pwm); //set pwm control, 0 for stop, and 255 for maximum speed
  if (reverse)  {
    digitalWrite(IN, LOW);
  }
  else  {
    digitalWrite(IN, HIGH);
  }
}

void Motor1(float pwm, boolean reverse) {
  Motor(pwm, reverse, EN1, IN1);
}

void Motor2(float pwm, boolean reverse) {
  Motor(pwm, reverse, EN2, IN2);
}


void motorSetup(){
  for (int i = 4; i <= 7; i++){
    pinMode(i, OUTPUT); //set pin 4,5,6,7 to output mode
  }
}

void sensorCalibration(){
  // configure the sensors
  left(255);
  delay(20);
  left(100);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){8, 9, 10}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
    closeWhiteValue+=qtr.calibrationOn.minimum[i];
  }
  Serial.println();
  closeWhiteValue = closeWhiteValue/SensorCount;

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  stop();
}
//-----------------------------------------------------------------------------setup-------------------------------------------------------------------------------------------------------
void setup(){ 
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  motorSetup();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void left(int speed){
  Motor1(speed, true);
  Motor2(speed, false);  
}


void stop(){
  Motor1(0, false);
  Motor2(0, false);
}

void forward(int speed1, int speed2){
  if(speed1 >= 0){
    Motor1(speed1, false); 
  }else{
    Motor1(-speed1, true); 
  }
  
  if(speed2 >= 0){
    Motor2(speed2, false); 
  }else{
    Motor2(-speed2, true); 
  }
  /*delay(20);
  stop();
  delay(20);*/
}


float Kp = 0.2;
float Ki = 0;
float Kd = 4;

int p, i, d, lastError, error, position;
const int maxValue = 10000;

void line_follower(){
  // read calibrated sensors values and obtain a measure of the line position
  // from 0 to 2000
  position = qtr.readLineBlack(sensorValues);
  error = 1000-position; //

  p = error;
  i = error+i;
  d = error - lastError;
  lastError = error;
  
  
  if(abs(error) >= 1000){ //i increase over time if the line is not detected
    i = error + i;
  }else{
    i=0;
  }
  i = constrain(i, -maxValue, maxValue);
  
  
  Serial.println(error);
  float speedDelta = (p * Kp) + (i * Ki) + (d * Kd);
  float ci = 0; //todo test with values
  float speed = map(abs(error), 0, 1000, 255, -255);
  float speed1 = speed + speedDelta;
  float speed2 = speed - speedDelta;

  speed1 = constrain(speed1, minSpeed, maxSpeed);
  speed2 = constrain(speed2, minSpeed, maxSpeed);
  forward(speed1, speed2);

}
//------------------------------------------------------------------------------------loop------------------------------------------------------------------------------------------------
void loop()
{

  while(digitalRead(11)){}
  delay(1000);
  Serial.println("[mesure frequence sensor]");
  int t = millis();
  int i = 0;
  while(millis()-t < 1000){
    i++;
    Serial.print(i);
    Serial.println(qtr.readLineBlack(sensorValues));
  }
  Serial.println("[Fin mesure frequence sensor]");

  while(digitalRead(11)){}
  Serial.println("[sensor Calibration...]");
  delay(1000);
  sensorCalibration();
  Serial.println("[sensor Calibrated]");

  while(digitalRead(12)){
  }
  Serial.println("[Begin line follow]");
  delay(1000);

  while(digitalRead(11)){
    line_follower();
  }
  stop();
  delay(1000);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------