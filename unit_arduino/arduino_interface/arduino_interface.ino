     // ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //


#define NUM_SENSOR 6  // number of sensors
#define VIB_LEVELS 3 // levels of intensity of vibration
#define MAX_LINE_LENGTH 40 // length of serial input line

// match trigger pins to all analog pins
int trigPin[6] = { A0, A1, A2, A3, A4, A5 };
int currentSensor = 0; // index of current operating sensor

// match echo pins to all non PWM digital pins
int echoPin[6] = { 2, 4, 7, 8, 12, 13 };

// match vibration pins to all PWM pins
int vibPin[6] = { 3, 5, 6, 9, 10, 11};

// Vibration pin status: 0 -> turned off, 1 -> 100(pwm input), 2 -> 150(pwm input), 3 -> 200 (pwm input)
int vibStatus[6] = {0, 0, 0, 0, 0, 0};

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int sensorDist[6] = {0, 0, 0, 0, 0, 0};
int prevDist[6] = {0, 0, 0, 0, 0, 0};

long prevTime = 0;

bool activeSensor = false;
bool vibChanged = false;
bool reportData = false;


void setup() {
  // set operating mode for all 18 pins
  for (int i = 0; i < NUM_SENSOR; i++) {
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
    pinMode(vibPin[i], OUTPUT);
  }
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("SystemOnline");
}


void loop() {

  if (Serial.available() > 0) {
    char *token;
    String data = Serial.readStringUntil('\n');
    char buf[MAX_LINE_LENGTH];
    data.toCharArray(buf, 40);
    token = strtok(buf, " ");
    String myStr = String(token);
    if (myStr == "activateSensor") {
      activeSensor = true;
      currentSensor = 0; // reset sensor pointer
      Serial.println("MSG: Sensors have been activated fresh");
    } else if (myStr == "deactivateSensor") {
      activeSensor = false;
      Serial.println("MSG: Sensors have been deactivated");
    }  else if (myStr == "reportData") {
        reportData = true;
    } else if (myStr == "stopReport") {
        reportData = false;
    } else if (myStr == "modifyVibrator") {
      // Indicate that vibration status has been changed
      vibChanged = true;
      Serial.println("msg: vibrator engaged");
      // read all tokens and modify all states
      token = strtok(NULL, " ");
      while (token != NULL) {
        String tokenStr = String(token);
        int numData = tokenStr.toInt();
        if (numData == 0) {
          Serial.println("error: Modify Vibrator received 0 or noninteger data");
          break;
        }  
        int vibIndex = numData / 10;
        if (vibIndex > NUM_SENSOR || vibIndex < 1) {
          Serial.println("error: Modify Vibrator received vibIndex > NUM_SENSOR or < 1");
          break;
        }
        int vibLevel = numData % 10;
        if (vibLevel > VIB_LEVELS) {
          Serial.println("error: Modify Vibrator received vibLevel > VIB_LEVELS");
          break;
        }
        vibStatus[vibIndex - 1] = vibLevel;
        token=strtok(NULL, " ");
      }
      Serial.print("msg: vibration have been modified");
      Serial.print(vibStatus[0]); 
      Serial.print(vibStatus[1]);
      Serial.print(vibStatus[2]);
      Serial.print(vibStatus[3]);
      Serial.print(vibStatus[4]);
      Serial.println(vibStatus[5]);
    } else {
      // maybe code for checking rest of message
      while (token != NULL) {
        // Consider checking available to write
        Serial.print("msg: Trash command received: ");
        Serial.print(token);
        token = strtok(NULL, " ");
      }
      Serial.println("");   
    }
  }

  // In a loop, check all status of vibration pins
  if (vibChanged) {
    for (int i = 0; i < NUM_SENSOR; i++) {
      // Comeback to this to check vibration levels
      if (vibStatus[i] == 0){
        analogWrite(vibPin[i], 0);
      } else if (vibStatus[i] == 1) {
        analogWrite(vibPin[i], 80);
      } else if (vibStatus[i] == 2) {
        analogWrite(vibPin[i], 155); 
      }
    }
    vibChanged = false;
    long timeDiff = micros() - prevTime;
    Serial.println(timeDiff);
  }

  // go through each sensor
  // reset currentSensor in case it goes over the limit
  if (currentSensor > 5) {
    currentSensor = 0;
  }

  if ( activeSensor ) {
    // Clears the corresponding trigPin condition
    digitalWrite(trigPin[currentSensor], LOW);
    delay(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin[currentSensor], HIGH);
    delay(10);
    digitalWrite(trigPin[currentSensor], LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin[currentSensor], HIGH, 30000);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    sensorDist[currentSensor] = distance;
    currentSensor = currentSensor + 1;
    currentSensor = currentSensor % NUM_SENSOR;
    if (currentSensor == 1) {
      prevTime = micros();
    }
    if (currentSensor == 0) {
      // Process all information in sensors and figure out intensity
      for (int i = 0; i < NUM_SENSOR; i++){
        int diff = prevDist[i] - sensorDist[i];
        int dist = sensorDist[i];
        int intensity = 0;
        if (dist == 0){
          intensity = 0;
        } else if ( dist < 40 || ( dist < 80 && diff > 25 ) ){
          intensity = 2;
        } else if ( dist < 80 || (dist < 200 && diff > 30 ) ){
          intensity = 1;
        } else{
          intensity = 0;
        }
        if (intensity != vibStatus[i]){
          vibStatus[i] = intensity;
          vibChanged = true;
        }
        prevDist[i] = sensorDist[i];
      }

      // send message through serial if prompted
      if (reportData ){
        String dataMsg = "data:";
        for (int i = 0; i < NUM_SENSOR; i++){
          dataMsg = dataMsg + " ";
          dataMsg = dataMsg + sensorDist[i];
        }
        Serial.println(dataMsg);
      }
    }
  }
}
