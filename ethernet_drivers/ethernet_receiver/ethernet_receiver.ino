#include <SPI.h>
#include <Ethernet.h>
#include <Servo.h>


#define BRAKE_ANGLE 90
#define NO_BRAKE_ANGLE 120
#define RED_LED_PIN 2
#define GREEN_LED_PIN 7
#define STEER_PWM_PIN 5
#define BRAKE_PWM_PIN 6
#define ESC_PWM_PIN 3
#define MSG_SIZE 11

// The MAC-address of the arduino Shield (Written on a sticker on the shield)
byte mac[] = {0xA8, 0x61, 0x0A, 0xAF, 0x19, 0x25};

// The IP-address of the computer (MÅ ENDRES TIL ZED BOX)
IPAddress server_ip(169, 254, 222, 165);

// Set the IP-address of the arduino (The 3 first sections must be identical to the IP-address of the computer)
IPAddress ip(169, 254, 222, 2);

// Port numbers from 49152 and up are guaranteed to be unused
unsigned int server_port = 49152;

EthernetClient client;

// Variables to store received data
bool bool1, bool2, bool3;
float angle, speed;

Servo Steer;
Servo Brake;
Servo ESC;

int steerAngle = 90; // 90 = straight, 45 = max right, 135 = max left
int ESCpwm = 0; // TODO: The caracteristics of the ESC haven't been found yet 

void setOutputs(byte buffer[MSG_SIZE]);     // Set vehicle parameters
void printBuffer(byte buffer[11]);          // Print received values

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  Steer.attach(STEER_PWM_PIN,  650, 2250);
  Brake.attach(BRAKE_PWM_PIN, 650, 2250);
  Speed.attach(ESC_PWM_PIN);
  Steer.write(steerAngle);
  Brake.write(NO_BRAKE_ANGLE);
  Speed.write(ESCpwm);

  // Disable SD Card (Not sure if necessary, but read in an arduino forum that it could cause issues with ethernet comms)
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);  

  // Start serial, needs to be removed in practical application
  Serial.begin(9600);
  
  // Start Ethernet connection
  Ethernet.begin(mac, ip);
  
  while(!Serial);
  Serial.println("Ethernet initialized");
  
  // Connect to server
  if (client.connect(server_ip, server_port)) {
    Serial.println("Connected to server");
  } else {
    Serial.println("Connection to server failed");
  }
}

void loop() {

  // Receive data from computer
  if (client.connected()) {

    if (client.available() >= 11) { // 3 bytes (3 booleans) + 4x2 bytes (2 floats)

      // Read data into buffer
      byte buffer[11];
      client.read(buffer, 11);

      setOutputs(buffer); // Set vehicle parameters
      printBuffer(buffer); // Print values, comment out in practical application
    }
  }
  else {
    Serial.println("Disconnected from server");
    delay(1000);
    // Try to reconnect
    client.connect(server_ip, server_port);
    
  }
}

void setOutputs(byte buffer[11]) {

  //client.read(buffer, 11);

  memcpy(&angle, buffer + 3, sizeof(angle));
  memcpy(&speed, buffer + 7, sizeof(speed));

  digitalWrite(RED_LED_PIN, buffer[0]);
  digitalWrite(GREEN_LED_PIN, buffer[1]);

  if (buffer[2] == 1) Brake.write(BRAKE_ANGLE);
  else Brake.write(NO_BRAKE_ANGLE);

  steerAngle = angle;
  Steer.write(steerAngle);

  ESCpwm = speed;
  ESC.write(ESCpwm);
}

void printBuffer(byte buffer[11]) {
  // Unpack data
    bool1 = buffer[0];
    bool2 = buffer[1];
    bool3 = buffer[2];
    
    memcpy(&angle, buffer + 3, sizeof(angle));
    memcpy(&speed, buffer + 7, sizeof(speed));
    
    // Print received data
    Serial.print("bool1: ");
    Serial.println(bool1);
    Serial.print("bool2: ");
    Serial.println(bool2);
    Serial.print("bool3: ");
    Serial.println(bool3);
    Serial.print("angle: ");
    Serial.println(angle);
    Serial.print("speed: ");
    Serial.println(speed);
}