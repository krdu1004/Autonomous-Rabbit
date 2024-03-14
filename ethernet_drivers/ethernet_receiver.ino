#include <Ethernet.h>

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};  // Replace with the actual MAC address of your microcontroller
IPAddress ip(192, 168, 1, 2);  // Replace with the desired IP address of your microcontroller
int port = 12345;  // Replace with the desired port number

EthernetServer server(port);

void setup() {
  Ethernet.begin(mac, ip);
  server.begin();
}

void loop() {
  EthernetClient client = server.available();
  
  if (client) {
    // If a client is connected
    Serial.println("Client connected");
    
    // Read data from the client
    char data[100];
    int bytesRead = client.read(reinterpret_cast<uint8_t*>(data), sizeof(data));
    data[bytesRead] = '\0';  // Null-terminate the received data
    Serial.println("Received from computer: " + String(data));

    // Send a response to the computer
    client.print("Hello from the microcontroller!");

    // Close the connection
    client.stop();
    Serial.println("Client disconnected");
  }
}