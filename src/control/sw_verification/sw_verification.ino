const unsigned int MAX_INPUT = 31;

const int STEERING_1 = 9;
const int STEERING_2 = 8;
const int FORWARD_RIGHT_1 = 4;
const int FORWARD_RIGHT_2 = 5;
const int FORWARD_LEFT_1 = 6;
const int FORWARD_LEFT_2 =7;
const int POT = A2;
const int TRIG_PIN = 10;
const int ECHO_PIN = 11;

const int STEERING_SPEED = 128;

int resistance_most_left = 460;
int resistance_most_right = 352;
int resistance_center = 406;

int current_max_step = 15;

int angle = 0, resistance = 0, mapped_resistance = 0;
int left_speed = 0, right_speed = 0;

unsigned long lastCommandTime = 0;
const unsigned int COMMAND_INTERVAL = 50;

int lastPotValue = 0;
unsigned long lastPotChangeTime = 0;
unsigned long lastPotReadTime = 0;
const int POT_READ_INTERVAL = 50;
const int STALL_DETECT_TIME = 500;
const int POT_CHANGE_THRESHOLD = 2;

const int STABILITY_CHECK_DURATION = 3000; 
int pot_min_read = 1023; 
int pot_max_read = 0;    
const int CALIBRATION_TIMEOUT = 5000;

const int PIN_FAIL_HIGH_THRESHOLD = 1000;
const int PIN_FAIL_LOW_THRESHOLD = 20;    
const int PIN_FAIL_RANGE_THRESHOLD = 100;  

bool isCalibrating = false;
bool isDriving = false;
int calibrationState = 0;
unsigned long calibrationStartTime = 0;
int calibration_mode = 0;

bool isMotorTesting = false;
int motorTestState = 0;
unsigned long motorTestStartTime = 0;
const int MOTOR_TEST_SPEED = 50; 
const int MOTOR_TEST_DURATION = 2000; 

void steerRight();
void steerLeft();
void maintainSteering();
void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);
void processIncomingByte(const byte inByte);
void processData(const char *data);
void runCalibration();
void runDriving();

void setup() {
    Serial.begin(115200);

    pinMode(POT, INPUT);
    pinMode(STEERING_1, OUTPUT);
    pinMode(STEERING_2, OUTPUT);
    pinMode(FORWARD_RIGHT_1, OUTPUT);
    pinMode(FORWARD_RIGHT_2, OUTPUT);
    pinMode(FORWARD_LEFT_1, OUTPUT);
    pinMode(FORWARD_LEFT_2, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void loop() {
    while (Serial.available() > 0) {
        processIncomingByte(Serial.read());
    }

    if (isCalibrating) {
        runCalibration(); 
    } 

    else if (isMotorTesting) {
        runMotorTest();
    }
  
    else {      
        unsigned long currentTime = millis();
        if (currentTime - lastCommandTime >= COMMAND_INTERVAL) {
            runDriving(); 
            lastCommandTime = currentTime;
        }
    }
}


void runDriving() {
   
    resistance = analogRead(POT);
    
    if (resistance <= resistance_center) {
        mapped_resistance = map(resistance, resistance_most_left, resistance_center, -current_max_step, 0);
    } else {
        mapped_resistance = map(resistance, resistance_center, resistance_most_right, 0, current_max_step);
    }

    if (!isDriving) {
        maintainSteering();
        setLeftMotorSpeed(0);
        setRightMotorSpeed(0);
        return;
    }

    if (mapped_resistance == angle) {
        maintainSteering();
    } else if (mapped_resistance > angle) {
        steerLeft();
    } else {
        steerRight();
    }

    setLeftMotorSpeed(left_speed);
    setRightMotorSpeed(right_speed);
}


void runMotorTest() {
    unsigned long currentTime = millis();

    switch (motorTestState) {
        case 0: 
            Serial.println("Motor Test: FORWARD");
            setLeftMotorSpeed(MOTOR_TEST_SPEED);
            setRightMotorSpeed(MOTOR_TEST_SPEED);
            motorTestStartTime = currentTime;
            motorTestState = 1;
            break;

        case 1: 
            if (currentTime - motorTestStartTime >= MOTOR_TEST_DURATION) {
                Serial.println("Motor Test: REVERSE");
                setLeftMotorSpeed(-MOTOR_TEST_SPEED);
                setRightMotorSpeed(-MOTOR_TEST_SPEED);
                motorTestStartTime = currentTime; 
                motorTestState = 2;
            }
            break;

        case 2: 
            if (currentTime - motorTestStartTime >= MOTOR_TEST_DURATION) {
                Serial.println("Motor Test: STOP");
                setLeftMotorSpeed(0);
                setRightMotorSpeed(0);
                Serial.println("Motor Test Complete!");
                isMotorTesting = false;
                motorTestState = 0;
            }
            break;
    }
}

void runCalibration() {
    unsigned long currentTime = millis();

    switch (calibrationState) {
        case 0:
            Serial.println("Calibration Start: Finding Left Limit...");
            steerLeft(); 
           
            lastPotValue = analogRead(POT);
            lastPotChangeTime = currentTime;
            lastPotReadTime = currentTime;
            calibrationStartTime = currentTime;
            calibrationState = 1;
            break;

        case 1:
            if (currentTime - lastPotReadTime >= POT_READ_INTERVAL) {
                lastPotReadTime = currentTime;
                int currentPotValue = analogRead(POT);
                if (abs(currentPotValue - lastPotValue) > POT_CHANGE_THRESHOLD) {
                    lastPotValue = currentPotValue;
                    lastPotChangeTime = currentTime;
                }
            }

            if ((currentTime - lastPotChangeTime >= STALL_DETECT_TIME) || 
                (currentTime - calibrationStartTime >= CALIBRATION_TIMEOUT)) 
            {
                if (currentTime - calibrationStartTime >= CALIBRATION_TIMEOUT) {
                    Serial.println("Calibration FAILED! Left-turn timeout");
                    maintainSteering();
                    isCalibrating = false; 
                    calibrationState = 0;
                } else {
                   
                    maintainSteering(); 
                    Serial.println("Left Limit Found. Checking stability...");
                    pot_min_read = 1023;
                    pot_max_read = 0;
                    calibrationStartTime = currentTime; 
                    calibrationState = 10;
                }
            }
            break;

        case 10: { 
            int currentPotValue = analogRead(POT);
            if (currentPotValue < pot_min_read) pot_min_read = currentPotValue;
            if (currentPotValue > pot_max_read) pot_max_read = currentPotValue;

            if (currentTime - calibrationStartTime >= STABILITY_CHECK_DURATION) {
                resistance_most_left = (pot_min_read + pot_max_read) / 2;
                int jitter = pot_max_read - pot_min_read;
                
                Serial.print("Left value saved: ");
                Serial.print(resistance_most_left);
                Serial.print(", Jitter (Range): "); 
                Serial.println(jitter); 
                
                calibrationState = 2; 
            }
            break;
        }

        case 2: 
            Serial.println("Finding Right Limit...");
            steerRight(); 
           
            lastPotValue = analogRead(POT);
            lastPotChangeTime = currentTime;
            lastPotReadTime = currentTime;
            calibrationStartTime = currentTime;
            calibrationState = 3;
            break;

        case 3: 
            if (currentTime - lastPotReadTime >= POT_READ_INTERVAL) {
                lastPotReadTime = currentTime;
                int currentPotValue = analogRead(POT);
                if (abs(currentPotValue - lastPotValue) > POT_CHANGE_THRESHOLD) {
                    lastPotValue = currentPotValue;
                    lastPotChangeTime = currentTime;
                }
            }

            if ((currentTime - lastPotChangeTime >= STALL_DETECT_TIME) ||
                (currentTime - calibrationStartTime >= CALIBRATION_TIMEOUT))
            {
                if (currentTime - calibrationStartTime >= CALIBRATION_TIMEOUT) {
                    Serial.println("Calibration FAILED! Right-turn timeout");
                    maintainSteering();
                    isCalibrating = false; 
                    calibrationState = 0;
                } else {
                   
                    maintainSteering();
                    Serial.println("Right Limit Found. Checking stability...");
                    pot_min_read = 1023;
                    pot_max_read = 0;
                    calibrationStartTime = currentTime; 
                    calibrationState = 20;
                }
            }
            break;

        case 20: { 
          
            int currentPotValue = analogRead(POT);
            if (currentPotValue < pot_min_read) pot_min_read = currentPotValue;
            if (currentPotValue > pot_max_read) pot_max_read = currentPotValue;

            if (currentTime - calibrationStartTime >= STABILITY_CHECK_DURATION) {
                resistance_most_right = (pot_min_read + pot_max_read) / 2;
                int jitter = pot_max_read - pot_min_read;
                
                Serial.print("Right value saved: ");
                Serial.print(resistance_most_right);
                Serial.print(", Jitter (Range): "); 
                Serial.println(jitter); 
                
            
                if (resistance_most_left > PIN_FAIL_HIGH_THRESHOLD && resistance_most_right > PIN_FAIL_HIGH_THRESHOLD) {
                    Serial.println("Calibration FAILED! GND Pin disconnected");
                    isCalibrating = false;
                    calibrationState = 0;
                }
                
                else if (resistance_most_left < PIN_FAIL_LOW_THRESHOLD && resistance_most_right < PIN_FAIL_LOW_THRESHOLD) {
                    Serial.println("Calibration FAILED! VCC Pin disconnected");
                    isCalibrating = false;
                    calibrationState = 0;
                }
                
                else if (abs(resistance_most_left - resistance_most_right) < PIN_FAIL_RANGE_THRESHOLD) {
                    Serial.println("Calibration FAILED! Analog Pin disconnected");
                    isCalibrating = false;
                    calibrationState = 0;
                }

                else{
                  resistance_center = (resistance_most_left + resistance_most_right)/2;
                  calibrationStartTime = currentTime;
                  calibrationState = 4;
                }
                
            }
            break;
        }
            
        case 4: { 

            if (currentTime - calibrationStartTime >= CALIBRATION_TIMEOUT) {
                Serial.println("Calibration FAILED! Centering timeout");
                maintainSteering();
                isCalibrating = false;
                calibrationState = 0;
                break;
            }
            
            resistance = analogRead(POT);
            if (resistance <= resistance_center) {
                mapped_resistance = map(resistance, resistance_most_left, resistance_center, -current_max_step, 0);
            } else {
                mapped_resistance = map(resistance, resistance_center, resistance_most_right, 0, current_max_step);
            }

            if (mapped_resistance == angle) {
                maintainSteering();
                Serial.println("Centering complete! Ready.");
                
                Serial.print("Final Auto Center: ");
                Serial.println(resistance_center);
                
                isCalibrating = false;
                calibrationState = 0; 
            } else if (mapped_resistance > angle) {
                steerLeft();
            } else {
                steerRight();
            }
            break;
        } 
    }
}


void steerRight() {
    analogWrite(STEERING_1, STEERING_SPEED);
    analogWrite(STEERING_2, LOW);
}

void steerLeft() {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, STEERING_SPEED);
}

void maintainSteering() {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, LOW);
}

void setLeftMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(FORWARD_LEFT_1, speed);
        analogWrite(FORWARD_LEFT_2, LOW);
    } else {
        analogWrite(FORWARD_LEFT_1, LOW);
        analogWrite(FORWARD_LEFT_2, (-1) * speed);
    }
}

void setRightMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(FORWARD_RIGHT_1, speed);
        analogWrite(FORWARD_RIGHT_2, LOW);
    } else {
        analogWrite(FORWARD_RIGHT_1, LOW);
        analogWrite(FORWARD_RIGHT_2, (-1) * speed);
    }
}

void processIncomingByte(const byte inByte) {
    static char input_line[MAX_INPUT];
    static unsigned int input_pos = 0;

    switch (inByte) {
        case '\n':
            input_line[input_pos] = 0; 
            processData(input_line);   
            input_pos = 0; 
            break;

        case '\r':
            break;

        default:
            if (input_pos < (MAX_INPUT - 1)) {
                input_line[input_pos++] = inByte;
            }
            break;
    }
}


void processData(const char *data) {

    if (data[0] == '?' && data[1] == '\0') {
        Serial.println("ARDUINO_READY");
        return;
    }
    
    if (data[0] == 'e' && data[1] == '\0') {
        if (isCalibrating) {
            maintainSteering();
            isCalibrating = false;
            calibrationState = 0;
            Serial.println("Calibration ABORTED!");
        }
   
        if (isMotorTesting) {
            setLeftMotorSpeed(0);
            setRightMotorSpeed(0);
            isMotorTesting = false;
            motorTestState = 0;
            Serial.println("Motor Test ABORTED!");
        }
        return;
    }

    if (data[0] == 'p' && data[1] == '\0') {
        isDriving = false;
        Serial.println("Driving Mode OFF");
        return;
    }

    if (data[0] == 'M') {
        if (data[1] == '7' && data[2] == '\0') {
            current_max_step = 7;
        } 
        return;
    }

    if (data[0] == 's' || data[0] == 'l' || data[0] == 'r') {
        if (isCalibrating || isMotorTesting) return;

        int sIndex = -1, lIndex = -1, rIndex = -1;
        for (int i = 0; data[i] != '\0'; i++) {
            if (data[i] == 's') sIndex = i;
            else if (data[i] == 'l') lIndex = i;
            else if (data[i] == 'r') rIndex = i;
        }

        if (sIndex != -1 && lIndex != -1 && rIndex != -1) {
            int newAngle = atoi(data + sIndex + 1);
            int newLeftSpeed = atoi(data + lIndex + 1);
            int newRightSpeed = atoi(data + rIndex + 1);
            
            if (newAngle != angle || newLeftSpeed != left_speed || newRightSpeed != right_speed) {
                angle = newAngle;
                left_speed = newLeftSpeed;
                right_speed = newRightSpeed;
                if (angle > current_max_step) angle = current_max_step;
                else if (angle < -current_max_step) angle = -current_max_step;
            }
        }
        return;
    }

    if (isCalibrating || isDriving || isMotorTesting) {
        return; 
    }

    if (data[0] == 'a' && data[1] == '\0') {
        isCalibrating = true;
        calibration_mode = 0;
        calibrationState = 0; 
        Serial.println("Starting Auto Calibration...");
        return;
    }

    if (data[0] == 'd' && data[1] == '\0') {
        isDriving = true;
        Serial.println("Driving Mode ON");
        return;
    }

    if (data[0] == 'm' && data[1] == '\0') {
        isMotorTesting = true;
        motorTestState = 0;
        Serial.println("Motor Test Started...");
        return;
    }

}
