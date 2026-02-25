// ROS 2 Serial Bridge Protocol - Arduino Code
// Compatible with serial_bridge_node.py

#include <SPI.h>
#include <MsTimer2.h>

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

int velocity = 0, steer_angle = 0, f_speed = 0;
int MIN_SENSOR_VALUE = 130, MAX_SENSOR_VALUE = 740;
int sensorBuffer[10], bufferIndex = 0;
long encoder_count1 = 0, encoder_count2 = 0;

float Kp = 2.35, Ki = 0.0, Kd = 0.0;
double error = 0, error_old = 0, error_s = 0, error_d = 0;
int pwm_output = 0, sensorValue = 0, Steer_Angle_Measure = 0, Steering_Angle = 0;
int NEUTRAL_ANGLE = 0, filter_size = 5;
bool initialization_complete = false;

// Serial protocol functions
void receiveCommand() {
    if (Serial.available() >= 6) {  // 'C' + int16 + int16 + '\n'
        char header = Serial.read();
        if (header == 'C') {
            byte vel_low = Serial.read();
            byte vel_high = Serial.read();
            byte steer_low = Serial.read();
            byte steer_high = Serial.read();
            char footer = Serial.read();
            
            if (footer == '\n') {
                velocity = (int16_t)((vel_high << 8) | vel_low);
                steer_angle = (int16_t)((steer_high << 8) | steer_low);
                
                if (velocity >= 255) velocity = 255;
                if (velocity <= -255) velocity = -255;
            }
        }
    }
    
    // Parameter reception
    if (Serial.available() >= 6) {  // param_id + float + '\n'
        char param_id = Serial.peek();
        
        if (param_id == 'P' || param_id == 'I' || param_id == 'D' || 
            param_id == 'N' || param_id == 'F' || param_id == 'm' || param_id == 'M') {
            
            Serial.read();  // consume param_id
            byte data[4];
            Serial.readBytes(data, 4);
            Serial.read();  // consume '\n'
            
            float value;
            memcpy(&value, data, 4);
            
            switch (param_id) {
                case 'P': Kp = value; break;
                case 'I': Ki = value; break;
                case 'D': Kd = value; break;
                case 'N': NEUTRAL_ANGLE = (int)value; break;
                case 'F': filter_size = (int)value; break;
                case 'm': MIN_SENSOR_VALUE = (int)value; break;
                case 'M': MAX_SENSOR_VALUE = (int)value; break;
            }
        }
    }
}

void sendEncoderData() {
    // Protocol: 'E' + encoder1(int32) + encoder2(int32) + '\n'
    Serial.write('E');
    Serial.write((byte*)&encoder_count1, 4);
    Serial.write((byte*)&encoder_count2, 4);
    Serial.write('\n');
}

void sendSteeringAngle() {
    // Protocol: 'S' + angle(float) + '\n'
    float angle = (float)Steer_Angle_Measure;
    Serial.write('S');
    Serial.write((byte*)&angle, 4);
    Serial.write('\n');
}

void sendRawSensor() {
    // Protocol: 'R' + sensor(float) + '\n'
    float sensor = (float)sensorValue;
    Serial.write('R');
    Serial.write((byte*)&sensor, 4);
    Serial.write('\n');
}

void sendPWMOutput() {
    // Protocol: 'W' + pwm(float) + '\n'
    float pwm = (float)pwm_output;
    Serial.write('W');
    Serial.write((byte*)&pwm, 4);
    Serial.write('\n');
}

void sendSteeringError() {
    // Protocol: 'X' + error(float) + '\n'
    float err = (float)error;
    Serial.write('X');
    Serial.write((byte*)&err, 4);
    Serial.write('\n');
}

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
    analogWrite(MOTOR1_PWM, 0);
    analogWrite(MOTOR3_PWM, 0);
    
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
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
    static unsigned long last_publish = 0;
    static unsigned long last_receive = 0;
    static long prev_encoder1 = 0, prev_encoder2 = 0;
    static float prev_steering = 0;
    
    f_speed = velocity;
    
    // Receive commands from ROS 2
    if (millis() - last_receive >= 20) {
        receiveCommand();
        encoder_count1 = readEncoder(1);
        encoder_count2 = readEncoder(2);
        last_receive = millis();
    }
    
    if (!initialization_complete) {
        return;
    }
    
    // Publish data to ROS 2
    int publish_interval;
    if (abs(velocity) > 100) {
        publish_interval = 50;
    } else if (abs(velocity) > 50) {
        publish_interval = 75;
    } else {
        publish_interval = 100;
    }
    
    if (millis() - last_publish >= publish_interval) {
        bool data_changed = false;
        
        if (encoder_count1 != prev_encoder1) {
            sendEncoderData();
            prev_encoder1 = encoder_count1;
            prev_encoder2 = encoder_count2;
            data_changed = true;
        }
        
        if (abs(Steer_Angle_Measure - prev_steering) >= 1.0) {
            sendSteeringAngle();
            prev_steering = Steer_Angle_Measure;
            data_changed = true;
        }
        
        sendRawSensor();
        sendPWMOutput();
        sendSteeringError();
        
        if (data_changed) {
            last_publish = millis();
        }
    }
}

void control_callback() {
    static boolean output = HIGH;
    static unsigned long start_time = millis();
    
    digitalWrite(13, output);
    output = !output;
    
    if (!initialization_complete) {
        return;
    }
    
    if (millis() - start_time < 3000) {
        return;
    }
    
    motor_control(f_speed);
    
    sensorValue = getFilteredSensor();
    Steer_Angle_Measure = map(sensorValue, MIN_SENSOR_VALUE, MAX_SENSOR_VALUE, LEFT_STEER_ANGLE, RIGHT_STEER_ANGLE);
    Steering_Angle = NEUTRAL_ANGLE + steer_angle;
    steering_control();
}

void update_encoder_counts(int speed, long &encoder_count) {
    encoder_count += speed > 0 ? 1 : (speed < 0 ? -1 : 0);
}

int cs_pin_for_encoder(int encoder_no) {
    if (encoder_no == 1) return ENC1_ADD;
    if (encoder_no == 2) return ENC2_ADD;
    return -1;
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
    } else if (motor1_pwm > 0) {
        digitalWrite(MOTOR1_DIR, HIGH);
        analogWrite(MOTOR1_PWM, motor1_pwm);
    } else {
        digitalWrite(MOTOR1_DIR, LOW);
        analogWrite(MOTOR1_PWM, -motor1_pwm);
    }
}

void motor_control(int f_speed) {
    if (f_speed == 0) {
        analogWrite(MOTOR1_PWM, 0);
        delay(10);
        digitalWrite(MOTOR1_DIR, HIGH);
    } else {
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
    } else if (motor_pwm < 0) {
        digitalWrite(MOTOR3_DIR, HIGH);
        analogWrite(MOTOR3_PWM, -motor_pwm);
    } else {
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
    pwm_output = (pwm_output >= 255) ? 255 : pwm_output;
    pwm_output = (pwm_output <= -255) ? -255 : pwm_output;
    
    if (error == 0) {
        steer_motor_control(0);
        error_s = 0;
    } else {
        steer_motor_control(pwm_output);
    }
    error_old = error;
    
    static int prev_steer_angle = 0;
    if ((steer_angle - prev_steer_angle) > 10 || (steer_angle - prev_steer_angle) < -10) {
        error_s = 0;
        pwm_output = pwm_output * 0.5;
    }
    prev_steer_angle = steer_angle;
}

void steering_control() {
    if (Steering_Angle <= LEFT_STEER_ANGLE + NEUTRAL_ANGLE)
        Steering_Angle = LEFT_STEER_ANGLE + NEUTRAL_ANGLE;
    if (Steering_Angle >= RIGHT_STEER_ANGLE + NEUTRAL_ANGLE)
        Steering_Angle = RIGHT_STEER_ANGLE + NEUTRAL_ANGLE;
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
