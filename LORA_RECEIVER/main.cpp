#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <SPI.h>          // SPI library
#include <LoRa.h>         // LoRa library
#include <WiFi.h>         // WiFI library
#include <WiFiManager.h>  // WiFI library
#include <Arduino_JSON.h> // JSON library
#include <HTTPClient.h>   // HTTP library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>  // SSD1306 library for OLED
#include <AsyncTCP.h>          // ESP32 Webserver library
#include <ESPAsyncWebServer.h> // ESP32 Websever library

// Serial monitor enable/disable
#define SERIAL_MONITOR 0
// Define the pins for LoRa receiver module
#define NSS 12
#define RST 5
#define DI0 2
// Define the pins for RGB led
#define RED 16
#define GREEN 15
#define BLUE 14
// Define the pins for Oled
#define SDA_Oled 13
#define SCL_Oled 4
#define WIRE Wire
// Define the pin for Buzzer
#define BUZZER 33

#define WIFI_RESET_BUTTON 21 // button to reset (or clear all the stored wifi credentials )
#define SERVER_LISTENING_BUTTON 17 // button to configure esp32 as a server

/* DHT22 datatype */
typedef struct
{
  double temp;
  double humid;
  bool received;
} DHT22Data;

/* Connection button structure */
typedef struct
{
  bool pressed;
  bool connect;
  bool disconnect;
  int pinNumber;
} ConnectButton;

/* Global variable */
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &WIRE); // Oled display variable
unsigned long lastTimeDelay = 0;                             
unsigned long lastSendTime = 0;
unsigned long lastReceiveTime = 0;
unsigned long lastWsTime = 0;
unsigned long lastCheck = 0;
String backendIP = "";                                       // Backend IP address and its port
String jwtToken = "";                                        // JWT token variable
DHT22Data data = {0, 0, false};                              // DHT22 received data
ConnectButton wifiButton = {false, false, false, 22};        // button to connect/disconnect wifi
ConnectButton loraButton = {false, false, false, 32};        // button to connect/disconnect lora
bool wifiResetFlag = false;                                  // flag to check whether the wifi reset button was pressed     
bool sendingFlag = false;                                    // flag to check the sending/unsending data to web api command from website
bool sendingStatus[2] = {false, false};                      // flag to check the status of sending data to web api
bool loginFlag = false;                                      // flag to identify whether you have logan or not
AsyncWebServer server(80);                                   // ESP32 server listening on port 80
AsyncWebSocket ws("/ws");                                    // ESP32 websocket handle
HTTPClient http;
bool webserverButton = false;                                // flag to check whether webserverButton was pressed
bool websocketFlag = false;                                  // flag to check whether esp32 was set as websocket
bool websocketConnected = false;                             // flag to check whether websocket connection was established
bool realtimeFlag = false;                                   // flag to check the realtime mode
double temperatureRange[2] = {0, 0};                         // array containing lowest and highest temperature level
double humidityRange[2] = {0, 0};                            // array containing lowest and highest humidity level
const char *sourceId = "20";                                 // define the LoRa Id of this esp32 device
const char *seperateCharacter = ",";                         // define the seperate character in the string of LoRa data
uint8_t cmdFlag = 0;                                         // 1: STM32F1 processed a command successfully, 2: STM32F1 failed to process a command
bool alarm_status = 0;                                       // 0: Alarm mode is OFF, 1: Alarm mode is ON
bool bulb_status = 0;                                        // 0: Bulb is OFF, 1: Bulb is ON
bool safeFlag = 0;                                           // 0: the temperature/humdity range is in safe range, 1: the temperature/humidity is in dangerous range
bool postFlag = false;                                       // Flag for checking the POST process
unsigned int count = 0;                                               // counter for detecting whether STM32F1 stop sending LoRa data.

/* Function Prototype */
// LED
void RGBLedConfig(char color); // Function to change the color of RGB led
// LoRa
void LoRaConfig(int nss, int rst, int di0); // LoRa configuration
void LoRaHandle();                          // Handle with LoRa connection
void LoRaSend(const char *desID, const char *message);  // LoRa send data function
void LoRaReceive();                    // LoRa receive data
// WiFi
void WiFiHandle(); // Handle with WiFi connection
// HTTP request
double GetLatestTemperature(String jwtToken);       // Get the latest temperature value in the database
bool PostTemperature(double temp); // Add a new temperature value to the database
bool PostHumidity(double humid);   // Add a new temperature value to the database
String PostLogin();                                 // Login as an admin
void WebServerConfig();                             // ESP32 WebServer configuration
// Interrupt Service Routine
void IRAM_ATTR WiFiButtonEvent();       // Button interrupt handler
void IRAM_ATTR LoRaButtonEvent();       // Button interrupt handler
void IRAM_ATTR WiFiResetEvent();        // Button interrupt handler
void IRAM_ATTR ServerListeningEvent();  // Server listening interrupt handler

// Buzzer
void BuzzerTrigger(); // Buzzer triggerx
// Delay
void Delayms(unsigned long ms); // User defined delay milisecond
// Oled
void OledDisplay(double temp, double humid); // Oled display
void LoRaStatusDisplay();                    // LoRa status Oled display
void WiFiStatusDislay();                     // WiFI status Oled display
void WebapiStatusDisplay();                  // Web API status Oled display
// Websocket
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void initWebSocket();
// Other functions
String convertToString(char *a, int size);  // Function to convert char array to string

/******************************************************************** SETUP ************************************************************************************/
void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize Buzzer
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH); // keep the BUZZER signal as high

  // Initialize OLED
  Wire.begin(SDA_Oled, SCL_Oled);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x64
                                             // Show image buffer on the display hardware.
                                             // Since the buffer is intialized with an Adafruit splashscreen
                                             // internally, this will display the splashscreen.
  display.display();                         // Display on the OLED
  delay(1000);
  display.clearDisplay(); // Clear the buffer
  display.display();

  // Display text
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(38, 20);
  display.print("Hello");
  display.setCursor(36, 40);
  display.print("World");
  display.display(); // actually display all of the above
  display.setTextSize(1);
  delay(1000);
  display.clearDisplay();
  display.display();
  delay(1000);

  // LED RGB set up
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);

  // Initialize Interrupt Button
  pinMode(wifiButton.pinNumber, INPUT_PULLUP);
  pinMode(loraButton.pinNumber, INPUT_PULLDOWN);
  pinMode(WIFI_RESET_BUTTON, INPUT_PULLUP);
  pinMode(SERVER_LISTENING_BUTTON, INPUT_PULLUP);
  // Attach the interrupt event
  attachInterrupt(wifiButton.pinNumber, WiFiButtonEvent, FALLING);
  attachInterrupt(WIFI_RESET_BUTTON, WiFiResetEvent, FALLING);
  attachInterrupt(SERVER_LISTENING_BUTTON, ServerListeningEvent, FALLING);
  attachInterrupt(loraButton.pinNumber, LoRaButtonEvent, CHANGE);
}

/******************************************************************** LOOP *************************************************************************************/
void loop()
{
  // Check whether user has pressed the wifi reset button
  if (wifiResetFlag == true)
  {
    // WiFi manager initialization
    WiFiManager wm;     // WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    wm.resetSettings(); // reset settings - wipe stored credentials
    wifiResetFlag = false;
  }
  // Check whether user has pressed wifi connect/disconnect button
  if (wifiButton.pressed)
  {
    WiFiHandle();
  }
  // Check whether user has pressed lora connect/disconnect button
  if (loraButton.pressed)
  {
    LoRaHandle();
  }
  // Check whether user has pressed the server button 
  if (webserverButton == true)
  {
    if (websocketFlag == true)
    {
      // Webserver configuration
      initWebSocket();
      WebServerConfig();
      server.begin();
    }
    else
    {
      ws.closeAll();
      ws.cleanupClients();
    }
    webserverButton = false;
  }
  if (millis() - lastReceiveTime > 5)
  {
    // Receive data from stm32f1 over LoRa
    if (loraButton.connect)
    {
      LoRaReceive(); // receive dht22 data
    }
    OledDisplay(data.temp, data.humid); // display data on OLED
    lastReceiveTime = millis();
  }
  // Check if the temperature/humidity is in dangerous range
  if (millis() - lastCheck > 200)
  {
    if (safeFlag == 1)
    {
      BuzzerTrigger();
    }
    lastCheck = millis();
  }
  // Check if esp32 has received LoRa data
  if (data.received == true)
  { 
    // Check for WiFi connection
    if (wifiButton.connect == true)
    {
      // POST data to webserver
        if (sendingFlag == true)
        {
          if (millis() - lastSendTime > 5000)
          {
            PostTemperature(data.temp);
            Delayms(10);
            PostHumidity(data.humid);
            if (sendingStatus[0] == true && sendingStatus[1] == true)
            {
              ws.textAll("Succeeded");
              data.received = false;
            }
            else
            {
              ws.textAll("Failed");
            }
            lastSendTime = millis();
          }
        }
      // PUSH data to the website directly
      if (realtimeFlag == true)
      {
        if (millis() - lastWsTime > 50)
        {
          // Check whether the command was processed
          if (cmdFlag == 1)
          {
            // http.end();
            // Delayms(10);
            ws.textAll("message,1");
            cmdFlag = 0;
          }
          else if (cmdFlag == 2)
          {
            // http.end();
            // Delayms(10);
            ws.textAll("message,0");
            cmdFlag = 0;
          }
          String message = "data," + String(data.temp) + "," + String(data.humid) + "," + String(alarm_status) + "," + String(bulb_status);
          // http.end();
          // Delayms(10);
          ws.textAll(message);
          data.received = false;
          lastWsTime = millis();
        }
      }
    }
  }
}

/************************************************************** Function implementation ************************************************************************/

/* Get the latest temperature value in the database */
double GetLatestTemperature(String jwtToken)
{
  /* Get request url */
  String getTemperatureUrl = "https://" + backendIP + "/Api/Temperature/";
  Serial.print("GET request url: ");
  Serial.println(getTemperatureUrl);
  HTTPClient http;
  String auth = "Bearer " + jwtToken;

  /* Begin to send Get request to the server */
  http.begin(getTemperatureUrl);
  http.addHeader("Authorization", auth);
  int responseCode = http.GET();
  String responseBody = "{}"; // string variable to get the response from the server
  if (responseCode > 0)
  {
    Serial.print("responseCode: ");
    Serial.println(responseCode);
    responseBody = http.getString(); // get response from the server
  }
  else
  {
    Serial.print("Error Code: ");
    Serial.println(responseCode);
    return -1;
    http.end();
  }
  http.end();

  /* Transfer the response to JSON format */
  double temperature = 0;
  JSONVar jsonResponse = JSON.parse(responseBody); // Convert to Json format
  if (JSON.typeof_(jsonResponse) == "undefined")
  {
    Serial.println("Parsing fail");
    return -1;
  }
  temperature = jsonResponse[0]["temperatureValue"]; // extract the latest temperature value from Json format response
  return temperature;
}

/* POST a new temperature value to the database */
bool PostTemperature(double temp)
{
  /* Post request url */
  String postTemperatureUrl = "https://" + backendIP + "/Api/Temperature/ESP32/AddTemperature/";
  String strbuff = String(temp, 2);                  // Convert double type to string type temperature
  postTemperatureUrl = postTemperatureUrl + strbuff; // Add temperature string to Post Url string
  Serial.print("POST request url: ");
  Serial.println(postTemperatureUrl);

  /* Begin to send POST request to the server */
  // HTTPClient http;
  postFlag = true;
  http.begin(postTemperatureUrl);
  int responseCode = http.POST("");
  if (responseCode == 200)
  {
    Serial.print("responseCode: ");
    Serial.println(responseCode);
    sendingStatus[0] = true;
    http.end();
    postFlag = false;
    return true;
  }
  else
  {
    Serial.print("Error Code: ");
    Serial.println(responseCode);
    sendingStatus[0] = false;
    http.end();
    postFlag = false;
    return false;
  }
}

/* POST a new humidity value to the database */
bool PostHumidity(double humid)
{
  /* Post request url */
  String postHumidityUrl = "https://" + backendIP + "/Api/Humidity/ESP32/AddHumidity/";
  String strbuff = String(humid, 2);                 // Convert double type to string type humidity
  postHumidityUrl = postHumidityUrl + strbuff;       // Add humidity string to Post Url string
  Serial.print("POST request url: ");
  Serial.println(postHumidityUrl);

  /* Begin to send POST request to the server */
  // HTTPClient http;
  postFlag = true;
  http.begin(postHumidityUrl);
  int responseCode = http.POST("");
  if (responseCode == 200)
  {
    Serial.print("responseCode: ");
    Serial.println(responseCode);
    sendingStatus[1] = true;
    http.end();
    postFlag = false;
    return true;
  }
  else
  {
    Serial.print("Error Code: ");
    Serial.println(responseCode);
    sendingStatus[1] = false;
    http.end();
    postFlag = false;
    return false;
  }
}

/* Login to the application server */
String PostLogin()
{
  /* Login request url */
  String postLoginUrl = "https://" + backendIP + "/Account/Login/";
  HTTPClient http;

  /* Begin to send POST request to the server */
  http.begin(postLoginUrl);
  /* Specify content-type header */
  http.addHeader("Content-Type", "application/json");
  /* Data to send with HTTP POST */
  int responseCode = http.POST("{\"email\":\"hoangnam.ho30@gmail.com\",\"password\":\"Nam@23012001\"}");
  String responseBody = "{}"; // string variable to get the response from the server
  if (responseCode > 0)
  {
    Serial.println("Successful Login as admin!");
    Serial.print("responseCode: ");
    Serial.println(responseCode);
    responseBody = http.getString(); // get response from the server
  }
  else
  {
    Serial.print("Error Code: ");
    Serial.println(responseCode);
    http.end();
    return "";
  }
  http.end();

  /* Transfer the response to JSON format */
  JSONVar jsonResponse = JSON.parse(responseBody); // Convert to Json format
  if (JSON.typeof_(jsonResponse) == "undefined")
  {
    Serial.println("Parsing fail");
    return "";
  }
  return jsonResponse["jwtToken"]; // extract the Jwt Token
}

/* WiFi Handle */
void WiFiHandle()
{
  // Connect to WiFi using WiFi manager
  if (wifiButton.connect == false && wifiButton.disconnect == false)
  {
    // WiFi manager initialization
    WiFiManager wm; // WiFiManager, Local intialization. Once its business is done, there is no need to keep it around

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Connecting ...");
    display.display();
    res = wm.autoConnect("ESP32WiFiManager", "esp32wifimanager"); // password protected ap

    // Display WiFi connection on the OLED
    if (!res)
    {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Fail to");
      display.println("Connect to");
      display.println("ESP32 WiFi");
      display.display();
      Delayms(1000);

      // Change the status of the WiFi connection button
      wifiButton.connect = false;
      wifiButton.disconnect = false;
      wifiButton.pressed = false;

      // ESP.restart(); reset the ESP32 MCU
    }
    else
    {
      // if you get here you have connected to the WiFi
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Successful");
      display.println("Connecting ESP32");
      display.println("to WiFi");
      display.display();
      Delayms(2000);

      // Change the status of the WiFi connection button
      wifiButton.connect = true;
      wifiButton.disconnect = false;
      wifiButton.pressed = false;
    }
  }

  // Disconnect from WiFi
  else if (wifiButton.connect == true && wifiButton.disconnect == false)
  {
    // Change the status of the WiFi connection button
    wifiButton.disconnect = true;
    wifiButton.connect = false;
    wifiButton.pressed = false;
    WiFi.disconnect();
    while (WiFi.status() != WL_CONNECTED)
      ; // wait for WiFi disconnection
  }

  // Reconnect to WiFi
  else if (wifiButton.connect == false && wifiButton.disconnect == true)
  {
    // Change the status of the WiFi connection button
    wifiButton.disconnect = false;
    wifiButton.connect = true;
    wifiButton.pressed = false;
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED)
      ; // wait for WiFi reconnection
  }

  // clear the Oled screen
  display.clearDisplay();
  display.display();
}

/* LoRa configuration */
void LoRaConfig(int nss, int rst, int di0)
{
  // Setup LoRa transceiver module
  LoRa.setPins(nss, rst, di0);

  // Select the frequency accordng to your location
  // 433E6 for Asia
  // 866E6 for Europe
  // 915E6 for North America
  while (!LoRa.begin(433E6))
  {
    Serial.println(".");
    Delayms(500);
  }

  LoRa.setSignalBandwidth(250E3); // set LoRa bandwidth 250Hz
}

/* LoRa Handle */
void LoRaHandle(void)
{
  // Enable LoRa communication
  if (!loraButton.connect)
  {
    // Change the status of LoRa connection button
    loraButton.connect = true;
    loraButton.disconnect = false;
    loraButton.pressed = false;

    // Enable LoRa communication
    LoRaConfig(NSS, RST, DI0);
  }

/* Stop LoRa communication */
  else if (loraButton.connect)
  {
    // Change the status of LoRa connection button
    loraButton.disconnect = true;
    loraButton.connect = false;
    loraButton.pressed = false;

    // Stop LoRa communication
    LoRa.end();
  }

  // Clear the Oled screen
  display.clearDisplay();
  display.display();
}

/* RGB configuration function */
void RGBLedConfig(char color)
{
  if (color == 'R')
  {
    analogWrite(RED, 255);
    analogWrite(BLUE, 0);
    analogWrite(GREEN, 0);
  }
  else if (color == 'G')
  {
    analogWrite(GREEN, 255);
    analogWrite(BLUE, 0);
    analogWrite(RED, 0);
  }
  else if (color == 'B')
  {
    analogWrite(BLUE, 255);
    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
  }
  else
  {
    analogWrite(BLUE, 0);
    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
  }
}

/* LoRa button pressed event */
void IRAM_ATTR LoRaButtonEvent()
{
  Delayms(50);
  BuzzerTrigger();
  loraButton.pressed = true; // Change the status of LoRa connection button
}

/* WiFi button pressed event */
void IRAM_ATTR WiFiButtonEvent()
{
  Delayms(50);
  BuzzerTrigger();
  wifiButton.pressed = true;
}

/* WiFi reset button pressed event */
void IRAM_ATTR WiFiResetEvent()
{
  Delayms(50);
  BuzzerTrigger();
  wifiResetFlag = true;
}

/* Buzzer trigger */
void BuzzerTrigger(void)
{
  digitalWrite(BUZZER, LOW); // Buzzer is ON when its signal pin is pulled LOW
  Delayms(100);
  digitalWrite(BUZZER, HIGH); // Buzzer is OFF when its signal pin is pulled HIGH
}

/* User defined delay milisecond */
void Delayms(unsigned long ms)
{
  lastTimeDelay = millis();
  while (millis() < lastTimeDelay + ms)
    ;
}

/* Oled display information */
void OledDisplay(double temp, double humid)
{
  display.setCursor(0, 0);
  if (data.received == true && loraButton.connect)
  {
    display.clearDisplay();
    WiFiStatusDislay();
    WebapiStatusDisplay();
    LoRaStatusDisplay();
    if (safeFlag == 0)
    {
      display.println("Safe Range");
    }
    else
    {
      display.println("Dangerous Range");
    }
    display.print("Temperature: ");
    display.println(String(temp));
    display.print("Humidity: ");
    display.print(String(humid));
  }
  else
  {
    WiFiStatusDislay();
    WebapiStatusDisplay();
    LoRaStatusDisplay();
  }
  // write the buffer to the display
  display.display();
}

/* LoRa status display */
void LoRaStatusDisplay()
{
  if (loraButton.connect == true)
  {
    display.println("LoRa: Active");
  }
  else
  {
    display.println("LoRa: Sleep");
  }
}

/* Sending to webapi status display */
void WebapiStatusDisplay()
{
  if(websocketFlag == true && websocketConnected == false)
  {
    display.println("Websocket: Listening");
  }
  else if (websocketFlag == true && websocketConnected == true)
  {
    display.println("Websocket: Connected");
  }
  else
  {
    display.println("Websocket: Closed");
  }
  if (sendingStatus[0] == true && sendingStatus[1] == true && sendingFlag == true)
  {
    display.println("Sending to Webapi");
  }
  else
  {
    display.println("");
  }
}

/* WiFI status display */
void WiFiStatusDislay()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    RGBLedConfig('R');
    display.println("WiFi: Disconnected");
    display.println("");
  }
  else
  {
    RGBLedConfig('G');
    display.println("WiFi: Connected");
    display.print("IP: ");
    display.println(WiFi.localIP());
  }
}

/* LoRa receive data */
void LoRaReceive()
{
  String LoRaData;                    // LoRa data
  char *rcvSrcId;                     // pointer to source ID
  char *rcvDesId;                     // pointer to destination ID
  char *sendingType;                  // pointer to sendingType (data or status packet)
  char *temp;                         // pointer to temperature
  char *humid;                        // pointer to humidity
  char *safety;                       // pointer to the safety check
  char *cmdStatus;                    // pointer to command status
  char *alarmStatus;                  // pointer to alarm status
  char *bulbStatus;                   // pointer to bulb status
  char rcvData[50];                   // array containing String of LoRa data
  char buffer[50];                    // array containing seperate components in array of LoRa data
  int packetSize = LoRa.parsePacket();// LoRa data packet size received from LoRa sender
  // Check if the packer size is large than 0
  if (packetSize)
  {
    count = 0;
    // Receive the data from LoRa sender in String format
    while (LoRa.available())
    {
      LoRaData = LoRa.readString();
    }

      // Get information from the received string: "source_id,destination_id,temperature,humidity"
      LoRaData.toCharArray(rcvData, LoRaData.length() + 1); // Convert String format to array format
      rcvSrcId = strtok(rcvData, ",");                      // return the pointer to the source_id
      rcvDesId = strtok(NULL, ",");                         // continue to return the pointer to the destination_id
      if (rcvSrcId != NULL && rcvDesId != NULL)
      {
        // Check if the destination_id is correct
        if (!strcmp(rcvDesId, sourceId))
        {
          sendingType = strtok(NULL, ","); // continue to return the pointer to the sending type
          if (sendingType != NULL)
          {
            // Check if the packet is for data
            if (!strcmp(sendingType, "0"))
            {
              temp = strtok(NULL, ",");   // continue to return the pointer to the temperature
              humid = strtok(NULL, ",");  // continue to return the pointer to the humidity
              safety = strtok(NULL, ","); // continue to return the pointer to the safety flag
              if (temp != NULL && humid != NULL && safety != NULL)
              {
                // Convert to double type
                strcpy(buffer, temp);               // copy the temperature string to the buffer
                sscanf(buffer, "%lf", &data.temp);  // convert the temperature string to double type
                strcpy(buffer, humid);              // copy the humidity string to the buffer
                sscanf(buffer, "%lf", &data.humid); // convert the humidity string to double type
                
                // Check the safety of temperature/humidity range
                if (!strcmp(safety, "1"))
                {
                  safeFlag = 1;
                }
                else
                {
                  safeFlag = 0;
                }

                // Successful data reception
                data.received = true;
              }
              else
              {
                // Failed to receive data reception
                data.received = false;
              }
            }
            // Check if the packet is for status
            else if (!strcmp(sendingType, "1"))
            {
              cmdStatus = strtok(NULL, ",");   // continue to return the pointer to the command status
              alarmStatus = strtok(NULL, ","); // continue to return the pointer to the alarm status
              bulbStatus = strtok(NULL, ","); // continue to return the pointer to the alarm status
              if(cmdStatus != NULL && alarmStatus != NULL)
              {
                // Check the command status
                if (!strcmp(cmdStatus, "1"))
                {
                  cmdFlag = 1; // command processing succeeded
                }
                else if (!strcmp(cmdStatus, "0"))
                {
                  cmdFlag = 2; // command processing failed
                }
                // Check the alarm status
                if (!strcmp(alarmStatus, "1"))
                {
                  alarm_status = 1;
                }
                else if (!strcmp(alarmStatus, "0"))
                {
                  alarm_status = 0;
                }
                // Check the bulb status
                if (!strcmp(bulbStatus, "1"))
                {
                  bulb_status = 1;
                }
                else if (!strcmp(bulbStatus, "0"))
                {
                  bulb_status = 0;
                }
                
                // Successful data reception
                data.received = true;
              }
              else
              {
                // Failed to receive data reception
                data.received = false;
              }
            }
          }
        }
      }

// Serial Monitoring Verification
#if SERIAL_MONITOR
    Serial.print("Data received: ");
    Serial.println(data.received);
    Serial.print("Packet size: ");
    Serial.println(packetSize);
    Serial.print("Packet data: ");
    Serial.println(rcvData);
    Serial.print("Source ID: ");
    Serial.println(rcvSrcId);
    Serial.print("Destination ID: ");
    Serial.println(rcvDesId);
    Serial.print("Temperature: ");
    Serial.println(temp);
    Serial.print("Humidity: ");
    Serial.println(humid);
    Serial.print("Command status: ");
    Serial.println(cmdStatus);
    Serial.print("\n\n\n");
#endif
  }
  else
  {
    count = count + 1;
    if(count == 500)
    {
      data.temp = 0;
      data.humid = 0;
      data.received = false;
      if (wifiButton.connect == true)
      {
        if(realtimeFlag == true)
        {
          // http.end();
          // Delayms(20);
          ws.textAll("message,2");
        }
      }
      count = 0;
    }
  }
}

/* LoRa send data */
  void LoRaSend(const char *desID, const char *message)
  {
    // send packet
    LoRa.beginPacket();            // start packet
    LoRa.print(sourceId);          // add source ID to the packet
    LoRa.print(seperateCharacter); // add the seperate character to the packet
    LoRa.print(desID);             // add destination ID to the packet
    LoRa.print(seperateCharacter); // add the seperate character to the packet
    LoRa.print(message);           // add message to the packet
    LoRa.endPacket();              // finish packet and send it
  }

/* ESP32 Web Server configuration */
  void WebServerConfig()
  {
    // Listening a GET request to turn on/off the alarm
    server.on("/Alarm", HTTP_GET, [](AsyncWebServerRequest *request){
      String message;
      if (request->hasParam("Status"))
      {
        message = request->getParam("Status")->value();
        if (message == "ON")
        {
          LoRaSend("10", "1,ON");
          Delayms(10);
        }
        else if (message == "OFF") 
        {
          LoRaSend("10", "1,OFF");
          Delayms(10);
        }
        else
        {
          request->send(400, "text/plain", "Bad request");
        }
      }
      else
      {
        message = "No message sent";
      }
      request->send(200, "text/plain", "Buzzer status command: " + message); }
    );

    // Listening a GET request to turn on/off the bulb
    server.on("/Bulb", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String message;
      if (request->hasParam("Status"))
      {
        message = request->getParam("Status")->value();
        if (message == "ON")
        {
          LoRaSend("10", "4,ON");
          Delayms(10);
        }
        else if (message == "OFF") 
        {
          LoRaSend("10", "4,OFF");
          Delayms(10);
        }
        else
        {
          request->send(400, "text/plain", "Bad request");
        }
      }
      else
      {
        message = "No message sent";
      }
      request->send(200, "text/plain", "Bulb status command: " + message); 
    });

    // Listening a GET request to start sending data to asp.net webserver
    server.on("/Start", HTTP_GET, [](AsyncWebServerRequest *request){
      sendingFlag = true;
      request->send(200, "text/plain", "Start request is sent successfully"); }
    );

    // Listening a GET request to stop sending data to asp.net webserver
    server.on("/Stop", HTTP_GET, [](AsyncWebServerRequest *request){
      sendingFlag = false;
      ws.textAll("Failed");
      request->send(200, "text/plain", "Stop request is sent successfully"); }
    );

    // Listening a GET request to open/close the realtime mode
    server.on("/Realtime", HTTP_GET, [](AsyncWebServerRequest *request){
      String message;
      if (request->hasParam("Status"))
      {
        message = request->getParam("Status")->value();
        if (message == "ON")
        {
          realtimeFlag = true;
        }
        else if (message == "OFF")
        {
          realtimeFlag = false;
        }
      }
      request->send(200, "text/plain", "Realtime request is sent successfully"); }
    );

    // Listening a GET request to update the status of the bulb and alarm mode
    server.on("/Update", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      LoRaSend("10", "5");
      Delayms(10);
      request->send(200, "text/plain", "Update request is sent successfully"); });

    // Listening a GET request to set the safe range for humidity/temperature
    server.on("/TemperatureRange", HTTP_GET, [](AsyncWebServerRequest *request){
      String lowest, highest;
      double d;
      if (request->hasParam("Lowest"))
      {
        lowest = request->getParam("Lowest")->value();
        sscanf(lowest.c_str(), "%lf", &d);
        temperatureRange[0] = d;
      }
      else
      {
        request->send(400, "text/plain", "Bad request");
      }
      if (request->hasParam("Highest"))
      {
        highest = request->getParam("Highest")->value();
        sscanf(highest.c_str(), "%lf", &d);
        temperatureRange[1] = d;
      }
      else
      {
        request->send(400, "text/plain", "Bad request");
      }
      if (temperatureRange[1] < temperatureRange[0])
      {
        request->send(400, "text/plain", "Bad request");
      }
      char message[20];
      sprintf(message, "2,%0.1lf,%0.1lf", temperatureRange[0], temperatureRange[1]);
      LoRaSend("10", message);
      Delayms(10);
      request->send(200, "text/plain", "Received request successfully");}
    );

    server.on("/HumidityRange", HTTP_GET, [](AsyncWebServerRequest *request){
      String lowest, highest;
      double d;
      if (request->hasParam("Lowest"))
      {
        lowest = request->getParam("Lowest")->value();
        sscanf(lowest.c_str(), "%lf", &d);
        humidityRange[0] = d;
      }
      else
      {
        request->send(400, "text/plain", "Bad request");
      }
      if (request->hasParam("Highest"))
      {
        highest = request->getParam("Highest")->value();
        sscanf(highest.c_str(), "%lf", &d);
        humidityRange[1] = d;
      }
      else
      {
        request->send(400, "text/plain", "Bad request");
      }
      if (humidityRange[1] < humidityRange[0])
      {
        request->send(400, "text/plain", "Bad request");
      }
      char message[20];
      sprintf(message, "3,%0.1lf,%0.1lf", humidityRange[0], humidityRange[1]);
      LoRaSend("10", message);
      Delayms(10);
      request->send(200, "text/plain", "Received request successfully"); }
    );
  }

/* Websocket */
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
data[len] = 0;
char *indicator;
char *main;
indicator = strtok((char*)data, ","); // return the pointer to the indicator part
main = strtok(NULL, ",");    // continue to return the pointer to main part
if(indicator != NULL && main != NULL)\
{
  if (!strcmp(indicator, "backendIP"))
  {
    backendIP = convertToString(main, strlen(main));
    BuzzerTrigger();
  }
  // else if (!strcmp(indicator, "Alarm"))
  // {
  //   if (!strcmp(main, "ON"))
  //   {
  //     LoRaSend("10", "1,ON");
  //     Delayms(10);
  //   }
  //   else if (!strcmp(main, "OFF"))
  //   {
  //     LoRaSend("10", "1,OFF");
  //     Delayms(10);
  //   }
    
  // }
}
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    websocketConnected = true;
    // Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    websocketFlag = false;
    websocketConnected = false;
    // Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}
void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

/* Function to convert char array to string */
String convertToString(char *a, int size)
{
  int i;
  String s = "";
  for (i = 0; i < size; i++)
  {
    s = s + a[i];
  }
  return s;
}

/* Server listening handler */
void IRAM_ATTR ServerListeningEvent()
{
  Delayms(50);
  BuzzerTrigger();
  websocketFlag = !websocketFlag;
  webserverButton = true;
}
