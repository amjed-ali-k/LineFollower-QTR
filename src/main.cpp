#include <Arduino.h>
#include <PushButton.h>
#include <QTRSensors.h>
#include <TaskScheduler.h>
#include "BluetoothSerial.h"
#include "macros.h"

QTRSensors qtr;

#define SensorCount 8
#define SCANDURATION 100

uint16_t sensorValues[SensorCount];
uint16_t position = 0;

BluetoothSerial BT;
const byte numChars = 10;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;

int a, b, m, n, x, y;
char e, f, g;

float Kp = 15;
float Ki = 0;
float Kd = 0;
int basespeed = 160;   // base speed of motor
int minspeed = (-150); // min speed of motor
int maxspeed = 250;    // max speed of motor
int T = 2;             // sample time for pid calculation

#define motlf 18 // motor pins
#define motlb 5
#define motrf 19
#define motrb 21

const int freqm = 1000; // motor pwm frequency
const int mrfc = 0;     // Motor Right Forward Channel (mrfc) pwm channel (seperate channel for each pins)
const int mrbc = 2;
const int mlfc = 4;
const int mlbc = 6;
const int mpwmr = 8; // motor pwm resolution(8bit)
int rpwm = 0;        // pwm value to motor
int lpwm = 0;
int irdig = B00000000;
int error = 0; // position on the line from ir
float P;       // pid variables
float I;
float D;
float pidval = 0;
float lasterror;
unsigned long currenttime = 0;
int dtime = 0;
unsigned long prevt = 0;

PushButton starbutton(15);

void readerror();
void readbutton();
void pid_control();
void recvWithEndMarker();
void btpidv(const String &data);
void log();

Scheduler ts;
Task readir(TASK_IMMEDIATE, TASK_FOREVER, &readerror, &ts, false);
Task pid(TASK_IMMEDIATE, TASK_FOREVER, &pid_control, &ts, false);
Task readsw(TASK_IMMEDIATE, TASK_FOREVER, &readbutton, &ts, false);
Task btread(TASK_IMMEDIATE, TASK_FOREVER, &recvWithEndMarker, &ts, false);
Task logTask(500, TASK_FOREVER, &log, &ts, false);

void log()
{
  _PP(error);
  _PP("\t");
  // Print SensorValues array
  for (int va = 0; va < SensorCount; va++)
  {
    _PP(sensorValues[va]);
    _PP("\t");
  }
  _PL();
}

void readerror()
{
  qtr.read(sensorValues);
  position = qtr.readLineBlack(sensorValues);
  error = position - 3500;
}

void pid_control()
{
  currenttime = millis();
  dtime = currenttime - prevt;
  if (dtime >= T)
  {
    P = error;
    I = I * error;
    D = error - lasterror;

    pidval = ((P * Kp) + ((Ki * T) * I) + ((Kd / T) * D)); // calculater pid val
    lasterror = error;
    prevt = currenttime;
  }

  int lmspeed = basespeed - pidval; // get motor speed difference from pid val
  int rmspeed = basespeed + pidval; // get motor speed difference from pid val

  if (lmspeed > maxspeed)
  { // if motor speed is greater than max speed set motor speed tot max speed
    lmspeed = maxspeed;
  }
  if (rmspeed > maxspeed)
  { // if motor speed is greater than max speed set motor speed tot max speed
    rmspeed = maxspeed;
  }
  if (lmspeed < minspeed)
  { // if mot speed less  than min speed set mot speed to min speed
    lmspeed = minspeed;
  }
  if (rmspeed < minspeed)
  { // if mot speed less  than min speed set mot speed to min speed
    rmspeed = minspeed;
  }

  rpwm = abs(rmspeed); // pwm value to write to motor
  lpwm = abs(lmspeed);

  if (rmspeed > 0)
  { // if right mot speed is > 0 turn in forward direction else in backward
    ledcWrite(mrfc, rpwm);
    ledcWrite(mrbc, 0);
  }
  else
  {
    ledcWrite(mrfc, 0);
    ledcWrite(mrbc, rpwm);
  }

  if (lmspeed > 0)
  { // if left mot speed is > 0 turn in forward direction else in backward
    ledcWrite(mlfc, lpwm);
    ledcWrite(mlbc, 0);
  }
  else
  {
    ledcWrite(mlfc, 0);
    ledcWrite(mlbc, lpwm);
  }
}
void readbutton()
{
  starbutton.update();
  if (starbutton.isDoubleClicked())
  {
    if (pid.isEnabled() == false)
    {
      pid.enable();
    }
    else
    {
      pid.disable();
      ledcWrite(mrfc, 0); // write all pwmm channel 0
      ledcWrite(mrbc, 0);
      ledcWrite(mlfc, 0);
      ledcWrite(mlbc, 0);
      P = 0;
      I = 0;
      D = 0;
    }
  }

  pid.isEnabled() ? btread.disable() : btread.enable();

  // _PP(Kp);
  // _PP("\t");
  // _PP(Kd);
  // _PP("\t");
  // _PP(lpwm);
  // _PP("\t");
  // _PP(rpwm);
  // _PL("");

  // Print SensorValues array
  // for (int va = 0; va < SensorCount; va++)
  // {
  //   _PP(sensorValues[va]);
  //   _PP("\t");
  // }
  // _PL();
}

String incomingData;

void recvWithEndMarker()
{

  if (BT.available() > 0)
  {
    incomingData = BT.readStringUntil('/');
    btpidv(incomingData);
  }
}

void btpidv(const String &data)
{
  _PM("Incoming Value: ");
  _PL(data);

  for (int cnt = 0; cnt <= data.length(); cnt++)
  {
    switch (data.charAt(cnt))
    {
    case 'P':
      Kp = data.substring(cnt + 1, cnt + 6).toFloat();
      _PM("Changed P value to ");
      _PL(Kp);
      cnt += 5;
      break;
    case 'I':
      Kd = data.substring(cnt + 1, cnt + 6).toFloat();
      _PM("Changed I value to ");
      _PL(Kd);
      cnt += 5;
      break;
    case 'T':
      T = data.substring(cnt + 1, cnt + 6).toInt();
      _PM("Changed T value to ");
      _PL(T);
      cnt += 5;
      break;
    case 'D':
      Ki = data.substring(cnt + 1, cnt + 6).toFloat();
      _PM("Changed D value to ");
      _PL(Ki);
      cnt += 5;
      break;
    case 'N':
      minspeed = data.substring(cnt + 1, cnt + 6).toInt();
      _PM("Changed MinSpeed value to ");
      _PL(minspeed);
      cnt += 5;
      break;
    case 'X':
      maxspeed = data.substring(cnt + 1, cnt + 6).toInt();
      _PM("Changed MaxSpeed value to ");
      _PL(maxspeed);
      cnt += 5;
      break;
    case 'S':
      basespeed = data.substring(cnt + 1, cnt + 6).toInt();
      _PM("Changed Base Speed to ");
      _PL(basespeed);
      cnt += 5;
      break;
    default:
      break;
    }
  }
  P = I = D = 0;
}

void setup()
{

#ifdef _DEBUG_
  Serial.begin(115200);
#endif

  ledcSetup(mrfc, freqm, mpwmr); // pwm channel and frequency and resolution setup
  ledcSetup(mrbc, freqm, mpwmr);
  ledcSetup(mlfc, freqm, mpwmr);
  ledcSetup(mlbc, freqm, mpwmr);
  ledcAttachPin(motrf, mrfc); // attach motor pins pwm channels
  ledcAttachPin(motrb, mrbc);
  ledcAttachPin(motlf, mlfc);
  ledcAttachPin(motlb, mlbc);
  ledcWrite(mrfc, 0); // write all pwmm channel 0
  ledcWrite(mrbc, 0);
  ledcWrite(mlfc, 0);
  ledcWrite(mlbc, 0);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){16, 14, 27, 26, 25, 33, 32, 4}, SensorCount);
  qtr.setEmitterPin(13);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < SCANDURATION; i++)
  {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    _PP("Calibrated Minimum : ");
    _PP(qtr.calibrationOn.minimum[i]);
    _PP(' ');
  }
  _PL();

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    _PP("Calibrated Maximum : ");

    _PP(qtr.calibrationOn.maximum[i]);
    _PP(' ');
  }
  _PL();
  delay(1000);

  BT.begin("LFQTR");
  starbutton.setActiveLogic(HIGH);

  ts.addTask(readir);
  ts.addTask(pid);
  ts.addTask(readsw);
  ts.addTask(btread);

  ts.addTask(logTask);
  logTask.enable();
  readir.enable();
  readsw.enable();
  // btread.enable();
}
void loop()
{
  ts.execute();
}
