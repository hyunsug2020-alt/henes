#include <SPI.h>
#include <MsTimer2.h>
#include <ArduinoJson.h>
#include <util/atomic.h>

#define MOTOR1_PWM 4
#define MOTOR1_DIR 22
#define MOTOR3_PWM 5
#define MOTOR3_DIR 24
#define Steering_Sensor A8
#define LEFT_STEER_ANGLE  -55
#define RIGHT_STEER_ANGLE  55
#define ENC1_ADD 3
#define ENC2_ADD 2
#define MIN_MOTOR_PWM 30
#define CMD_TIMEOUT_MS 1000

int MIN_SENSOR_VALUE = 130, MAX_SENSOR_VALUE = 740;
int NEUTRAL_ANGLE = 0, filter_size = 5;
float Kp = 2.35, Ki = 0.0, Kd = 0.0;

volatile int velocity = 0, steer_angle = 0, f_speed = 0;
int pwm_output = 0, sensorValue = 0, Steer_Angle_Measure = 0, Steering_Angle = 0;
volatile bool initialization_complete = false;
volatile bool control_tick = false;
unsigned long boot_ms = 0;

long encoder_count1 = 0, encoder_count2 = 0;
double error = 0, error_old = 0, error_s = 0, error_d = 0;
int sensorBuffer[10], bufferIndex = 0;
volatile unsigned long last_cmd_ms = 0;

#if ARDUINOJSON_VERSION_MAJOR >= 7
JsonDocument doc_rx, doc_tx;
#else
StaticJsonDocument<300> doc_rx, doc_tx;
#endif

int getFilteredSensor() {
    sensorBuffer[bufferIndex] = analogRead(Steering_Sensor);
    bufferIndex = (bufferIndex + 1) % filter_size;
    int tempArray[10];
    for (int i = 0; i < filter_size; i++) tempArray[i] = sensorBuffer[i];
    for (int i = 0; i < filter_size - 1; i++)
        for (int j = 0; j < filter_size - i - 1; j++)
            if (tempArray[j] > tempArray[j+1]) { int t=tempArray[j]; tempArray[j]=tempArray[j+1]; tempArray[j+1]=t; }
    if (filter_size % 2 == 0)
        return (tempArray[(filter_size-1)/2] + tempArray[filter_size/2]) / 2;
    else
        return tempArray[filter_size/2];
}

void setup() {
    Serial.begin(57600);
    Serial.setTimeout(50);

    pinMode(MOTOR1_PWM, OUTPUT); pinMode(MOTOR3_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT); pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(13, OUTPUT);
    analogWrite(MOTOR1_PWM, 0); analogWrite(MOTOR3_PWM, 0);
    digitalWrite(MOTOR1_DIR, LOW); digitalWrite(MOTOR3_DIR, LOW);

    boot_ms = millis();
    f_speed = 0; velocity = 0;

    for (int i = 0; i < filter_size; i++) sensorBuffer[i] = analogRead(Steering_Sensor);
    bufferIndex = 0;
    error = error_s = error_d = error_old = 0.0;
    pwm_output = 0;

    initEncoders();
    clearEncoderCount(1);
    clearEncoderCount(2);

    MsTimer2::set(60, control_callback);
    delay(2000);
    MsTimer2::start();

    // 중요: 초기화 완료 시점에 last_cmd_ms 리셋
    last_cmd_ms = millis();
    while (Serial.available() > 0) Serial.read();
    initialization_complete = true;
}

void loop() {
    processSerialCommands();
    encoder_count1 = readEncoder(1);
    encoder_count2 = readEncoder(2);
    if (control_tick) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { control_tick = false; }
        control_step();
    }
    publishData();
    delay(20);
}

void processSerialCommands() {
    if (Serial.available() <= 0) return;
    char rx_buf[192];
    size_t n = Serial.readBytesUntil('\n', rx_buf, sizeof(rx_buf)-1);
    if (n == 0) return;
    rx_buf[n] = '\0';
    DeserializationError err = deserializeJson(doc_rx, rx_buf, n);
    if (err) { Serial.print("{\"json_err\":\""); Serial.print(err.c_str()); Serial.println("\"}"); return; }
    const char* cmd = doc_rx["cmd"] | "";
    if (cmd[0] == '\0') return;

    if (strcmp(cmd, "vel") == 0) {
        int nv = constrain((int)doc_rx["linear"], -255, 255);
        int ns = doc_rx["angular"];
        unsigned long now = millis();
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            velocity    = nv;
            steer_angle = ns;
            f_speed     = nv;
            Serial.print(nv);
            Serial.println("}");
            last_cmd_ms = now;
        }
    }
    else if (strcmp(cmd, "kp") == 0)            { Kp = doc_rx["value"]; }
    else if (strcmp(cmd, "ki") == 0)            { Ki = doc_rx["value"]; }
    else if (strcmp(cmd, "kd") == 0)            { Kd = doc_rx["value"]; }
    else if (strcmp(cmd, "neutral_angle") == 0) { NEUTRAL_ANGLE = (int)doc_rx["value"]; }
    else if (strcmp(cmd, "filter_size") == 0) {
        filter_size = constrain((int)doc_rx["value"], 1, 10);
        for (int i = 0; i < filter_size; i++) sensorBuffer[i] = analogRead(Steering_Sensor);
        bufferIndex = 0;
    }
    else if (strcmp(cmd, "min_sensor") == 0) { MIN_SENSOR_VALUE = doc_rx["value"]; }
    else if (strcmp(cmd, "max_sensor") == 0) { MAX_SENSOR_VALUE = doc_rx["value"]; }
}

void publishData() {
    static unsigned long last_publish = 0;
    static unsigned long hb_counter   = 0;
    static long prev_enc1 = 0, prev_enc2 = 0;
    static float prev_steer = 0;
    if (!initialization_complete) return;
    int cv;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { cv = velocity; }
    int interval = (abs(cv)>100) ? 50 : (abs(cv)>50) ? 75 : 100;
    if (millis() - last_publish < (unsigned long)interval) return;
    bool force = (millis() - last_publish >= 500);
    bool changed = false;
    doc_tx.clear();
    JsonObject obj = doc_tx.to<JsonObject>();
    obj["hb"] = hb_counter++;
    if (encoder_count1 != prev_enc1) { obj["enc1"]=encoder_count1; prev_enc1=encoder_count1; changed=true; }
    if (encoder_count2 != prev_enc2) { obj["enc2"]=encoder_count2; prev_enc2=encoder_count2; changed=true; }
    if (abs(Steer_Angle_Measure-prev_steer)>=1.0 || abs(cv)>0 || force) {
        obj["steer"]=Steer_Angle_Measure; obj["error"]=error;
        obj["raw_sensor"]=sensorValue; obj["pwm"]=pwm_output;
        prev_steer=Steer_Angle_Measure; changed=true;
    }
    if (changed || force) { serializeJson(doc_tx, Serial); Serial.println(); last_publish=millis(); }
}

void control_step() {
    if (!initialization_complete) return;
    if (millis() - boot_ms < 3000) return;
    unsigned long cmd_ts;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { cmd_ts = last_cmd_ms; }
    if (millis() - cmd_ts > CMD_TIMEOUT_MS) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { velocity=0; f_speed=0; steer_angle=0; }
    }
    int sc, st;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sc=f_speed; st=steer_angle; }
    motor_control(sc);
    sensorValue         = getFilteredSensor();
    Steer_Angle_Measure = map(sensorValue, MIN_SENSOR_VALUE, MAX_SENSOR_VALUE, LEFT_STEER_ANGLE, RIGHT_STEER_ANGLE);
    Steering_Angle      = NEUTRAL_ANGLE + st;
    steering_control();
}

void control_callback() {
    static boolean out = HIGH;
    digitalWrite(13, out); out = !out;
    if (!initialization_complete) return;
    control_tick = true;
}

void front_motor_control(int pwm) {
    if (!initialization_complete) { analogWrite(MOTOR1_PWM,0); return; }
    int ep = abs(pwm);
    if (ep>0 && ep<MIN_MOTOR_PWM) { analogWrite(MOTOR1_PWM,0); return; }
    if      (pwm==0) { analogWrite(MOTOR1_PWM,0); }
    else if (pwm>0)  { digitalWrite(MOTOR1_DIR,HIGH); analogWrite(MOTOR1_PWM, pwm); }
    else             { digitalWrite(MOTOR1_DIR,LOW);  analogWrite(MOTOR1_PWM,-pwm); }
}

void motor_control(int spd) {
    if (spd==0) { analogWrite(MOTOR1_PWM,0); digitalWrite(MOTOR1_DIR,HIGH); }
    else        { front_motor_control(spd); }
}

void steer_motor_control(int pwm) {
    if (!initialization_complete) { analogWrite(MOTOR3_PWM,0); return; }
    if      (pwm>0) { digitalWrite(MOTOR3_DIR,LOW);  analogWrite(MOTOR3_PWM, pwm); }
    else if (pwm<0) { digitalWrite(MOTOR3_DIR,HIGH); analogWrite(MOTOR3_PWM,-pwm); }
    else            { analogWrite(MOTOR3_PWM,0); }
}

void PID_Control() {
    static int prev_sa = 0;
    error   = Steering_Angle - Steer_Angle_Measure;
    if (error*error_old<0) error_s=0;
    error_s += error; error_d = error-error_old;
    error_s  = constrain(error_s,-255,255);
    pwm_output = constrain((int)(Kp*error+Kd*error_d+Ki*error_s),-255,255);
    if (error==0) { steer_motor_control(0); error_s=0; }
    else          { steer_motor_control(pwm_output); }
    error_old = error;
    if (abs(steer_angle-prev_sa)>10) { error_s=0; pwm_output*=0.5; }
    prev_sa = steer_angle;
}

void steering_control() {
    Steering_Angle = constrain(Steering_Angle, LEFT_STEER_ANGLE+NEUTRAL_ANGLE, RIGHT_STEER_ANGLE+NEUTRAL_ANGLE);
    PID_Control();
}

int cs_pin_for_encoder(int n) { return (n==1)?ENC1_ADD:(n==2)?ENC2_ADD:-1; }

void initEncoders() {
    pinMode(ENC1_ADD,OUTPUT); pinMode(ENC2_ADD,OUTPUT);
    digitalWrite(ENC1_ADD,HIGH); digitalWrite(ENC2_ADD,HIGH);
    SPI.begin();
    digitalWrite(ENC1_ADD,LOW); SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC1_ADD,HIGH);
    digitalWrite(ENC2_ADD,LOW); SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC2_ADD,HIGH);
}

long readEncoder(int n) {
    int cs = cs_pin_for_encoder(n); if(cs<0) return 0;
    unsigned int a,b,c,d;
    digitalWrite(cs,LOW); SPI.transfer(0x60);
    a=SPI.transfer(0); b=SPI.transfer(0); c=SPI.transfer(0); d=SPI.transfer(0);
    digitalWrite(cs,HIGH);
    return ((long)a<<24)|((long)b<<16)|((long)c<<8)|(long)d;
}

void clearEncoderCount(int n) {
    int cs = cs_pin_for_encoder(n); if(cs<0) return;
    digitalWrite(cs,LOW); SPI.transfer(0x98); SPI.transfer(0); SPI.transfer(0); SPI.transfer(0); SPI.transfer(0); digitalWrite(cs,HIGH);
    delayMicroseconds(100);
    digitalWrite(cs,LOW); SPI.transfer(0xE0); digitalWrite(cs,HIGH);
    if(n==1) encoder_count1=0; else if(n==2) encoder_count2=0;
}
