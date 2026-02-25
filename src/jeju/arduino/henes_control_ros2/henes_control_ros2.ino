/*
 * HENES T870 Arduino Control - ROS 2 Compatible
 * JSON Serial Protocol Version
 * 
 * ROS 2 → Arduino: {"cmd":"vel","linear":100,"angular":30}
 * Arduino → ROS 2: {"enc1":1234,"enc2":5678,"steer":15}
 */

#include <SPI.h>
#include <MsTimer2.h>
#include <ArduinoJson.h>  // ArduinoJson 라이브러리 필요: https://arduinojson.org

#define MOTOR1_PWM 4
#define MOTOR1_DIR 22
#define MOTOR3_PWM 5
#define MOTOR3_DIR 24
#define Steering_Sensor A8
#define LEFT_STEER_ANGLE -55
#define RIGHT_STEER_ANGLE 55
#define ENC1_ADD 3
#define ENC2_ADD 2
#define MIN_MOTOR_PWM 30

// 전역 변수
int velocity = 0, steer_angle = 0, f_speed = 0;
int MIN_SENSOR_VALUE = 130, MAX_SENSOR_VALUE = 740;
int sensorBuffer[10], bufferIndex = 0;
long encoder_count1 = 0, encoder_count2 = 0;
float Kp = 2.35, Ki = 0.0, Kd = 0.0;
double error, error_old, error_s, error_d;
int pwm_output, sensorValue = 0, Steer_Angle_Measure = 0, Steering_Angle = 0;
int NEUTRAL_ANGLE = 0, filter_size = 5;
bool initialization_complete = false;

// JSON 문서 (전역 선언)
StaticJsonDocument<200> doc_rx, doc_tx;

int getFilteredSensor() {
    sensorBuffer[bufferIndex] = analogRead(Steering_Sensor);
    bufferIndex = (bufferIndex + 1) % filter_size;
    
    int tempArray[20];
    for (int i = 0; i < filter_size; i++) {
        tempArray[i] = sensorBuffer[i];
    }
    
    for (int i = 0; i < filter_size - 1; i++) {
        for (int j = 0; j < filter_size - i - 1; j++) {
            if (tempArray[j] > tempArray[j + 1]) {
                int temp = tempArray[j];
                tempArray[j] = tempArray[j + 1];
                tempArray[j + 1] = temp;
            }
        }
    }
    
    if (filter_size % 2 == 0) {
        return (tempArray[(filter_size - 1) / 2] + tempArray[filter_size / 2]) / 2;
    } else {
        return tempArray[filter_size / 2];
    }
}

void setup() {
    Serial.begin(57600);
    
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR3_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
    analogWrite(MOTOR1_PWM, 0);
    analogWrite(MOTOR3_PWM, 0);
    digitalWrite(MOTOR1_DIR, LOW);
    digitalWrite(MOTOR3_DIR, LOW);
    
    f_speed = 0;
    velocity = 0;
    
    for (int i = 0; i < filter_size; i++) {
        sensorBuffer[i] = analogRead(Steering_Sensor);
    }
    
    error = error_s = error_d = error_old = 0.0;
    pwm_output = 0;
    
    initEncoders();
    clearEncoderCount(1);
    clearEncoderCount(2);
    
    analogWrite(MOTOR1_PWM, 0);
    analogWrite(MOTOR3_PWM, 0);
    
    MsTimer2::set(60, control_callback);
    delay(2000);
    MsTimer2::start();
    
    initialization_complete = true;
}

void loop() {
    processSerialCommands();
    // Read hardware encoder counters every cycle.
    encoder_count1 = readEncoder(1);
    encoder_count2 = readEncoder(2);
    publishData();
    delay(20);  // 50Hz
}

void processSerialCommands() {
    if (Serial.available() > 0) {
        String jsonString = Serial.readStringUntil('\n');
        DeserializationError error = deserializeJson(doc_rx, jsonString);
        
        if (error) {
            return;
        }
        
        const char* cmd = doc_rx["cmd"];
        
        if (strcmp(cmd, "vel") == 0) {
            velocity = doc_rx["linear"];
            steer_angle = doc_rx["angular"];
            velocity = constrain(velocity, -255, 255);
        }
        else if (strcmp(cmd, "kp") == 0) {
            Kp = doc_rx["value"];
        }
        else if (strcmp(cmd, "ki") == 0) {
            Ki = doc_rx["value"];
        }
        else if (strcmp(cmd, "kd") == 0) {
            Kd = doc_rx["value"];
        }
        else if (strcmp(cmd, "neutral_angle") == 0) {
            NEUTRAL_ANGLE = doc_rx["value"];
        }
        else if (strcmp(cmd, "filter_size") == 0) {
            filter_size = doc_rx["value"];
            for (int i = 0; i < filter_size; i++) {
                sensorBuffer[i] = analogRead(Steering_Sensor);
            }
            bufferIndex = 0;
        }
        else if (strcmp(cmd, "min_sensor") == 0) {
            MIN_SENSOR_VALUE = doc_rx["value"];
        }
        else if (strcmp(cmd, "max_sensor") == 0) {
            MAX_SENSOR_VALUE = doc_rx["value"];
        }
    }
}

void publishData() {
    static unsigned long last_publish = 0;
    static long prev_encoder1 = 0, prev_encoder2 = 0;
    static float prev_steering = 0;
    
    if (!initialization_complete) return;
    
    int publish_interval = 100;
    if (abs(velocity) > 100) publish_interval = 50;
    else if (abs(velocity) > 50) publish_interval = 75;
    
    if (millis() - last_publish >= publish_interval) {
        bool data_changed = false;
        
        doc_tx.clear();
        
        if (encoder_count1 != prev_encoder1) {
            doc_tx["enc1"] = encoder_count1;
            prev_encoder1 = encoder_count1;
            data_changed = true;
        }
        
        if (encoder_count2 != prev_encoder2) {
            doc_tx["enc2"] = encoder_count2;
            prev_encoder2 = encoder_count2;
            data_changed = true;
        }
        
        if (abs(Steer_Angle_Measure - prev_steering) >= 1.0) {
            doc_tx["steer"] = Steer_Angle_Measure;
            doc_tx["error"] = error;
            doc_tx["raw_sensor"] = sensorValue;
            doc_tx["pwm"] = pwm_output;
            prev_steering = Steer_Angle_Measure;
            data_changed = true;
        }
        
        if (data_changed) {
            serializeJson(doc_tx, Serial);
            Serial.println();
            last_publish = millis();
        }
    }
}

void update_encoder_counts(int speed, long &encoder_count) {
    encoder_count += speed > 0 ? 1 : (speed < 0 ? -1 : 0);
}

int cs_pin_for_encoder(int encoder_no) {
    if (encoder_no == 1) return ENC1_ADD;
    if (encoder_no == 2) return ENC2_ADD;
    return -1;
}

void control_callback() {
    static boolean output = HIGH;
    static unsigned long start_time = millis();
    
    digitalWrite(13, output);
    output = !output;
    
    if (!initialization_complete) return;
    if (millis() - start_time < 3000) return;
    
    motor_control(f_speed);
    sensorValue = getFilteredSensor();
    Steer_Angle_Measure = map(sensorValue, MIN_SENSOR_VALUE, MAX_SENSOR_VALUE, LEFT_STEER_ANGLE, RIGHT_STEER_ANGLE);
    Steering_Angle = NEUTRAL_ANGLE + steer_angle;
    steering_control();
}

void front_motor_control(int motor1_pwm) {
    if (!initialization_complete) {
        analogWrite(MOTOR1_PWM, 0);
        return;
    }
    
    int effective_pwm = abs(motor1_pwm);
    
    if (effective_pwm > 0 && effective_pwm < MIN_MOTOR_PWM) {
        analogWrite(MOTOR1_PWM, 0);
        return;
    }
    
    if (motor1_pwm == 0) {
        analogWrite(MOTOR1_PWM, 0);
        return;
    }
    else if (motor1_pwm > 0) {
        digitalWrite(MOTOR1_DIR, HIGH);
        analogWrite(MOTOR1_PWM, motor1_pwm);
    }
    else {
        digitalWrite(MOTOR1_DIR, LOW);
        analogWrite(MOTOR1_PWM, -motor1_pwm);
    }
}

void motor_control(int f_speed) {
    if (f_speed == 0) {
        analogWrite(MOTOR1_PWM, 0);
        delay(10);
        digitalWrite(MOTOR1_DIR, HIGH);
    }
    else {
        front_motor_control(f_speed);
    }
}

void steer_motor_control(int motor_pwm) {
    if (!initialization_complete) {
        analogWrite(MOTOR3_PWM, 0);
        return;
    }
    
    if (motor_pwm > 0) {
        digitalWrite(MOTOR3_DIR, LOW);
        analogWrite(MOTOR3_PWM, motor_pwm);
    }
    else if (motor_pwm < 0) {
        digitalWrite(MOTOR3_DIR, HIGH);
        analogWrite(MOTOR3_PWM, -motor_pwm);
    }
    else {
        analogWrite(MOTOR3_PWM, 0);
    }
}

void PID_Control() {
    error = Steering_Angle - Steer_Angle_Measure;
    
    if (error * error_old < 0) {
        error_s = 0;
    }
    
    error_s += error;
    error_d = error - error_old;
    error_s = constrain(error_s, -255, 255);
    
    pwm_output = Kp * error + Kd * error_d + Ki * error_s;
    pwm_output = constrain(pwm_output, -255, 255);
    
    if (error == 0) {
        steer_motor_control(0);
        error_s = 0;
    }
    else {
        steer_motor_control(pwm_output);
    }
    
    error_old = error;
    
    static int prev_steer_angle = 0;
    if (abs(steer_angle - prev_steer_angle) > 10) {
        error_s = 0;
        pwm_output = pwm_output * 0.5;
    }
    prev_steer_angle = steer_angle;
}

void steering_control() {
    Steering_Angle = constrain(Steering_Angle, LEFT_STEER_ANGLE + NEUTRAL_ANGLE, RIGHT_STEER_ANGLE + NEUTRAL_ANGLE);
    PID_Control();
}

void initEncoders() {
    pinMode(ENC1_ADD, OUTPUT);
    pinMode(ENC2_ADD, OUTPUT);
    digitalWrite(ENC1_ADD, HIGH);
    digitalWrite(ENC2_ADD, HIGH);
    SPI.begin();
    
    digitalWrite(ENC1_ADD, LOW);
    SPI.transfer(0x88);
    SPI.transfer(0x03);
    digitalWrite(ENC1_ADD, HIGH);
    
    digitalWrite(ENC2_ADD, LOW);
    SPI.transfer(0x88);
    SPI.transfer(0x03);
    digitalWrite(ENC2_ADD, HIGH);
}

long readEncoder(int encoder_no) {
    unsigned int count_1, count_2, count_3, count_4;
    long count_value;

    int cs_pin = cs_pin_for_encoder(encoder_no);
    if (cs_pin < 0) return 0;

    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x60);
    count_1 = SPI.transfer(0x00);
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    
    count_value = ((long)count_1 << 24) + ((long)count_2 << 16) + ((long)count_3 << 8) + (long)count_4;
    
    return count_value;
}

void clearEncoderCount(int encoder_no) {
    int cs_pin = cs_pin_for_encoder(encoder_no);
    if (cs_pin < 0) return;

    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x98);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    
    delayMicroseconds(100);
    
    digitalWrite(cs_pin, LOW);
    SPI.transfer(0xE0);
    digitalWrite(cs_pin, HIGH);
    
    if (encoder_no == 1)
        encoder_count1 = 0;
    else if (encoder_no == 2)
        encoder_count2 = 0;
}
