#include <Wire.h>
#include <Adafruit_VL53L0X.h>

const uint8_t STBY_FRONT = 23; 
const uint8_t STBY_REAR = 25;
const uint8_t STBY_Z = 27;

const uint8_t PWM_FL = 4; const uint8_t FL_IN1 = 22; const uint8_t FL_IN2 = 24;
const uint8_t PWM_FR = 5; const uint8_t FR_IN1 = 26; const uint8_t FR_IN2 = 28;
const uint8_t PWM_RL = 6; const uint8_t RL_IN1 = 30; const uint8_t RL_IN2 = 32;
const uint8_t PWM_RR = 7; const uint8_t RR_IN1 = 35; const uint8_t RR_IN2 = 37;
const uint8_t PWM_Z = 8; const uint8_t Z_IN1 = 39; const uint8_t Z_IN2 = 41;

const uint8_t PIN_ESTOP = 18; 
const uint8_t ENC_FL_A = 19; const uint8_t ENC_FL_B = 34;
const uint8_t ENC_FR_A = 20; const uint8_t ENC_FR_B = 36;
const uint8_t ENC_RL_A = 21; const uint8_t ENC_RL_B = 38;
const uint8_t ENC_RR_A = 2;  const uint8_t ENC_RR_B = 40;
const uint8_t ENC_Z_A = 3;   const uint8_t ENC_Z_B = 42;

const uint8_t LIMIT_Z_BOTTOM = 44; const uint8_t LIMIT_Z_TOP = 46;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

volatile long ticks_fl = 0, ticks_fr = 0, ticks_rl = 0, ticks_rr = 0, ticks_z = 0;
volatile bool emergency_stopped = false;
unsigned long last_heartbeat = 0;

float target_v_left = 0.0, target_v_right = 0.0;
long target_z_ticks = 0;

float kp_xy = 1.5, ki_xy = 0.2, kd_xy = 0.1;
long prev_fl = 0, prev_fr = 0, prev_rl = 0, prev_rr = 0;
float sum_fl = 0, sum_fr = 0, sum_rl = 0, sum_rr = 0;
float last_err_fl = 0, last_err_fr = 0, last_err_rl = 0, last_err_rr = 0;

float kp_z = 2.0, kd_z = 0.5;
long last_z_err = 0;

void isr_estop() { 
  emergency_stopped = true; 
  digitalWrite(STBY_FRONT, LOW); digitalWrite(STBY_REAR, LOW); digitalWrite(STBY_Z, LOW); 
}
void isr_fl() { (digitalRead(ENC_FL_B) == HIGH) ? ticks_fl++ : ticks_fl--; }
void isr_fr() { (digitalRead(ENC_FR_B) == HIGH) ? ticks_fr++ : ticks_fr--; }
void isr_rl() { (digitalRead(ENC_RL_B) == HIGH) ? ticks_rl++ : ticks_rl--; }
void isr_rr() { (digitalRead(ENC_RR_B) == HIGH) ? ticks_rr++ : ticks_rr--; }
void isr_z()  { (digitalRead(ENC_Z_B) == HIGH)  ? ticks_z++  : ticks_z--; }

void setup() {
  Serial.begin(115200); Wire.begin();
  
  pinMode(STBY_FRONT, OUTPUT); pinMode(STBY_REAR, OUTPUT); pinMode(STBY_Z, OUTPUT);
  digitalWrite(STBY_FRONT, HIGH); digitalWrite(STBY_REAR, HIGH); digitalWrite(STBY_Z, HIGH);
  
  pinMode(PWM_FL, OUTPUT); pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(PWM_FR, OUTPUT); pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(PWM_RL, OUTPUT); pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(PWM_RR, OUTPUT); pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);
  pinMode(PWM_Z, OUTPUT);  pinMode(Z_IN1, OUTPUT);  pinMode(Z_IN2, OUTPUT);
  
  pinMode(ENC_FL_A, INPUT_PULLUP); pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP); pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_RL_A, INPUT_PULLUP); pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_RR_A, INPUT_PULLUP); pinMode(ENC_RR_B, INPUT_PULLUP);
  pinMode(ENC_Z_A, INPUT_PULLUP);  pinMode(ENC_Z_B, INPUT_PULLUP);
  
  pinMode(LIMIT_Z_BOTTOM, INPUT_PULLUP); pinMode(LIMIT_Z_TOP, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), isr_estop, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isr_fl, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isr_fr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), isr_rl, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isr_rr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_Z_A), isr_z, RISING);
  
  if (!lox.begin()) { Serial.println("ERR:VL53L0X_FAIL"); }
  last_heartbeat = millis();
}

void setMotor(uint8_t pwm_pin, uint8_t in1, uint8_t in2, int speed) {
  if (emergency_stopped) speed = 0;
  if (speed >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); } 
  else { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); }
  analogWrite(pwm_pin, constrain(abs(speed), 0, 255));
}

void loop() {
  if (millis() - last_heartbeat > 600) isr_estop();

  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if (msg.startsWith("CMD:")) {
      last_heartbeat = millis();
      int p1 = msg.indexOf(',');
      int p2 = msg.indexOf(',', p1 + 1);
      target_v_left = msg.substring(4, p1).toFloat();
      target_v_right = msg.substring(p1 + 1, p2).toFloat();
      target_z_ticks = msg.substring(p2 + 1).toInt();
    } else if (msg == "RESET") {
      emergency_stopped = false;
      digitalWrite(STBY_FRONT, HIGH); digitalWrite(STBY_REAR, HIGH); digitalWrite(STBY_Z, HIGH);
      sum_fl = 0; sum_fr = 0; sum_rl = 0; sum_rr = 0;
    }
  }

  static unsigned long last_ctrl = 0;
  if (millis() - last_ctrl >= 50) {
    last_ctrl = millis();

    long v_fl = ticks_fl - prev_fl; prev_fl = ticks_fl;
    long v_fr = ticks_fr - prev_fr; prev_fr = ticks_fr;
    long v_rl = ticks_rl - prev_rl; prev_rl = ticks_rl;
    long v_rr = ticks_rr - prev_rr; prev_rr = ticks_rr;
    
    float err_fl = target_v_left - v_fl; sum_fl = constrain(sum_fl + err_fl, -100, 100);
    int pwm_fl = (kp_xy * err_fl) + (ki_xy * sum_fl) + (kd_xy * (err_fl - last_err_fl)); last_err_fl = err_fl;
    
    float err_rl = target_v_left - v_rl; sum_rl = constrain(sum_rl + err_rl, -100, 100);
    int pwm_rl = (kp_xy * err_rl) + (ki_xy * sum_rl) + (kd_xy * (err_rl - last_err_rl)); last_err_rl = err_rl;

    float err_fr = target_v_right - v_fr; sum_fr = constrain(sum_fr + err_fr, -100, 100);
    int pwm_fr = (kp_xy * err_fr) + (ki_xy * sum_fr) + (kd_xy * (err_fr - last_err_fr)); last_err_fr = err_fr;

    float err_rr = target_v_right - v_rr; sum_rr = constrain(sum_rr + err_rr, -100, 100);
    int pwm_rr = (kp_xy * err_rr) + (ki_xy * sum_rr) + (kd_xy * (err_rr - last_err_rr)); last_err_rr = err_rr;

    setMotor(PWM_FL, FL_IN1, FL_IN2, pwm_fl);
    setMotor(PWM_FR, FR_IN1, FR_IN2, pwm_fr);
    setMotor(PWM_RL, RL_IN1, RL_IN2, pwm_rl);
    setMotor(PWM_RR, RR_IN1, RR_IN2, pwm_rr);

    long z_err = target_z_ticks - ticks_z;
    int z_pwm = (kp_z * z_err) + (kd_z * (z_err - last_z_err)); last_z_err = z_err;
    if (z_pwm < 0 && digitalRead(LIMIT_Z_BOTTOM) == LOW) z_pwm = 0;
    if (z_pwm > 0 && digitalRead(LIMIT_Z_TOP) == LOW) z_pwm = 0;
    setMotor(PWM_Z, Z_IN1, Z_IN2, z_pwm);

    VL53L0X_RangingMeasurementData_t measure; lox.rangingTest(&measure, false);
    int tof_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
    long avg_l_ticks = (ticks_fl + ticks_rl) / 2;
    long avg_r_ticks = (ticks_fr + ticks_rr) / 2;

    Serial.print("DATA:");
    Serial.print(avg_l_ticks); Serial.print(",");
    Serial.print(avg_r_ticks); Serial.print(",");
    Serial.print(ticks_z); Serial.print(",");
    Serial.print(tof_mm); Serial.print(",");
    Serial.println(emergency_stopped ? 1 : 0);
  }
}