// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //

#define echoPin 4 // shared echo pin
#define trigPin 2 //attach pin D3 Arduino to pin Trig of HC-SR04
#define vibPin  6  // vibration unit pin (PWM only)

#define MAX_LINE_LENGTH 40 // length of serial input line

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement


bool activeSensor = false;
bool activeVib = false;

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(vibPin, OUTPUT);
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("SystemOnline");
}
void loop() {


  if (Serial.available() > 0){
    char *token;
    String data = Serial.readStringUntil('\n');
    char buf[MAX_LINE_LENGTH];
    data.toCharArray(buf, 40);
    token = strtok(buf," ");
    String myStr = String(token);
    if (myStr == "activate"){
      activeSensor = true;
      Serial.println("MSG: Sensors have been activated");
    } else if (myStr == "deactivate"){
      activeSensor = false;
      Serial.println("MSG: Sensors have been deactivated");
    } else if (myStr == "vibrationOn") {
      activeVib = true;
      Serial.println("MSG: vibration have been activated");
    } else if (myStr == "vibrationOff") {
      activeVib = false;
      Serial.println("MSG: vibration have been deactivated");
    }
    while (token != NULL){
    // Consider checking available to write
      Serial.print("DEBUG: ");
      Serial.println(token);
      token = strtok(NULL, " ");
    }
  }

  // Current HZ is 200
  if ( activeVib ){
    analogWrite(vibPin, 152);
  } else{
    analogWrite(vibPin, 0);
  }
  if ( activeSensor ){
  // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delay(200);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delay(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

  }
}
