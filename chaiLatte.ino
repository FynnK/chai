
// Sensor readings with offsets:   2       -5      16393   0       0       0
// Your offsets:   -3654   -604    794     52      -26     6
// Sensor readings with offsets:   0       0       16392   0       -1      0
// Your offsets:   -3658   -615    795     52      -27     7
// Sensor readings with offsets:   -4      -6      16384   0       0       -1
// Your offsets:   -3632   -637    745     47      -23     6

// Data is printed as: acelX acelY acelZ giroX giroY giroZ
// Check that your sensor readings are close to 0 0 16384 0 0 0

#include <Adafruit_NeoPixel.h>
#include "ESPAsyncWebServer.h"
#include "AsyncElegantOTA.h"
#include <SPIFFS.h>
#include "driver/adc.h"
#include "driver/temp_sensor.h"
#include <esp_wifi.h>
#include "ArduinoJson.h"
#include <AsyncJson.h>
#include <AsyncWebSocket.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "esp_now.h"
#include "calibration.h"


//A4:CF:12:8D:5A:48 neuball

//A4:CF:12:8D:5A:68 der dreiäugige

constexpr auto LED_EN = 10;
constexpr auto LAS_EN = 18;
constexpr auto LED_CNT = 8;
constexpr auto LED_PIN = 11;
constexpr auto BUTTON_PIN = 0;
constexpr auto MPU_INT_PIN = 14;
constexpr auto LAS1_PIN = 4;
constexpr auto MPU6050_INT_PIN_CFG = 0x37;
constexpr auto MPU6050_INT_ENABLE = 0x38;
constexpr auto MPU_ARRAY_SIZE = 50;
constexpr auto MAX_LIGHTING_MODE = 4;
constexpr auto serialEnabled = true;
constexpr auto MONITOR_EN_PIN = 17;
constexpr auto MONITOR_PIN = 16;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


constexpr auto ssid = "EspBall";
constexpr auto stat_ssid = "retaeper";
constexpr auto password = "welchespasswort";

constexpr auto accFileString = "/acc.txt";
constexpr auto logFileString = "/log.txt";


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

const char DEVICE_NAME[] = "mpu6050";

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}


int mpuCnt = 0;
int mpuIndex = 0;
int lightingMode = 0;
int buttonState = HIGH;

bool laser1 = false;

volatile bool isUpdating = false;
bool logging = false;
bool log10k = false;
volatile bool buttonHandled = true;
volatile bool buttonIsDown = false;
bool mpuLogging = true;
volatile bool updaterRunning = true;
volatile bool mpuRunning = true;
volatile bool isMaster = 0;
bool noticeableAcc = false;

uint8_t brightness = 15;
uint16_t hue = 0;

uint32_t startTime;
uint32_t endTime;
uint32_t lastDownTime;
uint32_t lastUpTime;
int lastDT = 0;
int dt;

String content = {};
float accTot;
float velZ = 0;


TaskHandle_t lightingTaskHandle;
TaskHandle_t updaterTaskHandle;
TaskHandle_t mpuTaskHandle;

Adafruit_NeoPixel leds(LED_CNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

MPU6050 mpu;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

struct struct_message {
  uint32_t pix[8];
};

void stopTimer(String name, uint32_t t){
  uint32_t dt = xTaskGetTickCount() - t;
  Serial.println(name + " took " + String(dt) + " ticks");
}

struct ImuData {
  uint32_t timestamp;
  int16_t accX, accY, accZ;
  float qW, qX, qY, qZ;
};

struct_message outMessage;
struct_message inMessage;

ImuData imuData[MPU_ARRAY_SIZE];
uint32_t nPix[8] = {};

void IRAM_ATTR buttonChanged()
{
  buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH)
  {
    lastUpTime = xTaskGetTickCountFromISR();
    buttonIsDown = false;
    buttonHandled = false;
  }
  else if (buttonState == LOW)
  {
    lastDownTime = xTaskGetTickCountFromISR();
    buttonIsDown = true;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&inMessage, incomingData, sizeof(inMessage));
  Serial.print("Bytes received: ");
  Serial.println(len);
  for(int i = 0; i < 8; i++){
    nPix[i] = inMessage.pix[i];
  }
}

void log(String text)
{
  if(serialEnabled){
    Serial.print(text);
    return;
  }
  if (logging)
  {
    File logFile = SPIFFS.open(logFileString, FILE_APPEND);
    logFile.print(text);
    logFile.close();
  }
}

void logln(String text)
{
  if(serialEnabled){
    Serial.println(text);
    return;
  }
  if (logging)
  {
    File logFile = SPIFFS.open(logFileString, FILE_APPEND);
    logFile.println(text);
    logFile.close();
  }
}

void updatePixels(){
  for(int i = 0; i < 8; i++){
    nPix[i] = leds.getPixelColor(i);
  }
}

void espNowInit(){

  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

}

void espNowSend(){
  for(int i = 0; i < 8; i++){
   outMessage.pix[i] = nPix[i];
  }
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outMessage, sizeof(outMessage));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
}

void lightingTask(void *pvParameters)
{
  log("lightingTask started on core: ");
  logln(String(xPortGetCoreID()));

  while (true)
  {    
    if(isMaster){
    //  espNowSend();
    }

    if (leds.getBrightness() != brightness)
    {
      leds.setBrightness(brightness % 256);
    }    

    if (lightingMode == 0)
    {
      for (int i = 0; i < 8; i++)
      {
        leds.setPixelColor(i, leds.ColorHSV((uint16_t)(accTot * 8192), 255, 255));
        leds.setBrightness((uint8_t)(accTot*8));
      }
    }
    else if (lightingMode == 1)
    {
      for (int i = 0; i < 8; i++)
      {
        leds.setPixelColor(i, leds.ColorHSV((xTaskGetTickCount() * 10 + i * 8192) % 65535, 255, 255));
      }
    }
    else if (lightingMode == 2)
    {
      for (int i = 0; i < 8; i++)
      {
        leds.setPixelColor(i, leds.ColorHSV((xTaskGetTickCount() * 10) % 65535, 255, 255));
      }
    }else if (lightingMode == 3)
    {
      for (int i = 0; i < 8; i++)
      {
        leds.setPixelColor(i, leds.ColorHSV((int)(((xTaskGetTickCount()%1024)*128 + (accTot * 8192))) % 65535, 255, 255));
      }
    }else if (lightingMode == 4)
    {
      for (int i = 0; i < 8; i++)
      {
        leds.setPixelColor(i, nPix[i]);
      }
    }

    
    if(buttonIsDown){
      int d = xTaskGetTickCount() - lastDownTime;
        for(int i = 0; i < d/125; i++){
        leds.setPixelColor(i, leds.Color(255,255,255,255));
      }
    }
    


    if(leds.canShow())
    leds.show();
    taskYIELD();
  }
}



void mpu_setup()
  {
    Wire.begin(8,9);
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    mpu.initialize();
    pinMode(MPU_INT_PIN, INPUT);

    // verify connection
    logln(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    logln(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    mpu.setDLPFMode(MPU6050_DLPF_BW_256);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  
    mpu.setXGyroOffset(-95);
    mpu.setYGyroOffset(55);
    mpu.setZGyroOffset(18);
    mpu.setXAccelOffset(350);
    mpu.setYAccelOffset(2783);
    mpu.setZAccelOffset(1389);
    

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      logln(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      logln(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      pinMode(MPU_INT_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      logln(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      log(F("DMP Initialization failed (code "));
      log(devStatus);
      logln(F(")"));
    }

    startTime = xTaskGetTickCount();
}

uint32_t prevImuTimestamp(){
    if(mpuIndex == 0){
        return imuData[MPU_ARRAY_SIZE-1].timestamp;
    }else{
        return imuData[mpuIndex-1].timestamp;
    }
}


void mpu_loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    logln(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
 


    while(fifoCount > packetSize){

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      imuData[mpuIndex] = {xTaskGetTickCount(), aaWorld.x, aaWorld.y, aaWorld.z ,q.w ,q.x, q.y, q.z};

      mpuCnt++;
      mpuIndex++;
      if(mpuIndex >= MPU_ARRAY_SIZE){
          mpuIndex = 0;
      }
    }


    accTot = (float)(aaWorld.getMagnitude())/1024.0f;
    velZ += aaWorld.z*0.001*(prevImuTimestamp()-imuData[mpuIndex].timestamp);


    // if(aa.getMagnitude() >= 1.2f){
    //     if(noticeableAcc == false)
    //     noticeableAcc = true;
    // }else{
    //     noticeableAcc = false;
    // }

  }
}

void mpuTask(void *pvParameters)
{
  mpuRunning = true;
  logln("mpuTask started");
  mpu_setup();
  startTime = xTaskGetTickCount();

  while (mpuRunning)
  {
    mpu_loop();
    if (mpuCnt == 9999){
        endTime = xTaskGetTickCount();
        log10k = true;
    }
    taskYIELD();   
  }
  vTaskDelete(NULL);
}

void setLightingMode(uint8_t inMode){
  lightingMode = inMode%(MAX_LIGHTING_MODE+1);
}

void setBrightness(uint8_t inBrightness){
  brightness = inBrightness%256;
}

void getAccsCSV(){
  content.clear();
  int ind = mpuIndex;
  for(int i = ind; i < (MPU_ARRAY_SIZE + ind); i++){
    int id = i%MPU_ARRAY_SIZE;
    content.concat(imuData[id].timestamp);
    content.concat(',');
    content.concat(imuData[id].accX);
    content.concat(',');
    content.concat(imuData[id].accY);
    content.concat(',');
    content.concat(imuData[id].accZ);
    content.concat(',');
    content.concat(imuData[id].qW);
    content.concat(',');
    content.concat(imuData[id].qX);
    content.concat(',');
    content.concat(imuData[id].qY);
    content.concat(',');
    content.concat(imuData[id].qZ);
    content.concat(',');
    content.concat(touchRead(4)+touchRead(4)+touchRead(4)+touchRead(4)+touchRead(4));
    content.concat("\r\n");
  }
  return;
}

void logAcc()
{
  File accFile = SPIFFS.open(accFileString, FILE_WRITE);
  getAccsCSV();
  accFile.print(content.c_str());
  accFile.close();
}


void calibrateIMU(){
      mpuRunning = false;
      vTaskDelay(100);

      mpu.setXAccelOffset(0);
      mpu.setYAccelOffset(0);
      mpu.setZAccelOffset(0);
      mpu.setXGyroOffset(0);
      mpu.setYGyroOffset(0);
      mpu.setZGyroOffset(0);

      logln("\nCalibrating IMU");
      meansensors();
      vTaskDelay(10);

      calibration();
      vTaskDelay(10);

      mpu.setXGyroOffset(gx_offset);
      mpu.setYGyroOffset(gy_offset);
      mpu.setZGyroOffset(gz_offset);
      mpu.setXAccelOffset(ax_offset);
      mpu.setYAccelOffset(ay_offset);
      mpu.setZAccelOffset(az_offset);

      logln("xGyroOffset: "+gx_offset);
      logln("yGyroOffset: "+gy_offset);
      logln("zGyroOffset: "+gz_offset);
      logln("xAccelOffset: "+ax_offset);
      logln("yAccelOffset: "+ay_offset);
      logln("zAccelOffset: "+az_offset);

      xTaskCreate(mpuTask, "mpuTask", 5000, NULL, 10, &mpuTaskHandle);

}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    logln("Websocket client connection received");

  } else if(type == WS_EVT_DISCONNECT){
    logln("Client disconnected");

  } else if(type == WS_EVT_DATA){
    if(len < 2){
      client->text("p");
        
    }else if(len < 10) {
      getAccsCSV();
      client->text(content.c_str());
    } else {
      StaticJsonDocument<1200> doc;
      DeserializationError error = deserializeJson(doc, data);
      setBrightness(doc["brightness"]);
      setLightingMode(doc["mode"]);
      auto pix = doc["pixels"];
      for(int i = 0; i < 8; i++){
        auto in = pix[i];
        nPix[i] = leds.Color(in[0], in[1], in[2], in[3]);
      }
      client->text(error.c_str());
    }
  }
}

void updaterTask(void *pvParameters)
{
  for(int i = 0; i < 8; i++){  
    leds.setPixelColor(i, leds.Color(0, 255, 30, 10));
  }
  leds.show();
  vTaskDelay(300);
  WiFi.setTxPower(WIFI_POWER_5dBm);
  WiFi.softAP(ssid, password);

  log("SoftAP started with ip: ");
  logln(WiFi.softAPIP().toString());

  log("updater running on core: ");
  logln(String(xPortGetCoreID()));

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/update");
  });


  server.on("/format", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "formatting SPIFFS");
    SPIFFS.format();
  });

  server.on("/getMac", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", WiFi.macAddress());
  });

  server.on("/getActiveOffsets", HTTP_GET, [](AsyncWebServerRequest *request) {
        String activeOffsets = "------Offsets------\r\n";
          activeOffsets += "gyXtc: " +  String(mpu.getXGyroOffsetTC());
          activeOffsets += "gyX: "   +  String(mpu.getXGyroOffset());
          activeOffsets += "\r\n";
          activeOffsets += "gyXtc: " +  String(mpu.getXGyroOffsetTC());
          activeOffsets += "gyX: "   +  String(mpu.getXGyroOffset());
          activeOffsets += "\r\n";
          activeOffsets += "gyXtc: " +  String(mpu.getXGyroOffsetTC());
          activeOffsets += "gyX: "   +  String(mpu.getXGyroOffset());
          activeOffsets += "\r\n";
          activeOffsets += "accX: "  +  String(mpu.getXAccelOffset());
          activeOffsets += "accY: "  +  String(mpu.getYAccelOffset());
          activeOffsets += "accZ: "  +  String(mpu.getZAccelOffset());

    request->send(200, "text/plain", activeOffsets);
  });


  server.on("/files", HTTP_GET, [](AsyncWebServerRequest *request) {
    String out;  
    File root = SPIFFS.open("/");
    File file = root.openNextFile();

    while(file)
    {
      out.concat(file.name());
      out.concat("\n");
      file = root.openNextFile();
    }
    request->send(200, "text/plain", out);
  });

  server.on("/logAcc", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "loggingAccs");
    logAcc();
  });

  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "restarting");
    ESP.restart();
  });

  server.on("/calibrateIMU", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "calibrating");
    calibrateIMU();
  });

  server.on("/clearAccLogs", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (SPIFFS.exists(accFileString))
      ;
    {
      SPIFFS.remove(accFileString);
    }
    request->send(200, "text/plain", "accLogs Cleared");
  });

  server.on("/availableSpiffs", HTTP_GET, [](AsyncWebServerRequest *request) {
      int freeBytes = SPIFFS.totalBytes() - SPIFFS.usedBytes();
      String msg = "there is " + (String)freeBytes + " Bytes available in SPIFFS";
      request->send(200, "text/plain", msg);
  });

  server.on("/clearLogs", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (SPIFFS.exists(logFileString))
      ;
    {
      SPIFFS.remove(logFileString);
    }
    request->send(200, "text/plain", "logs Cleared");
  });

  server.on("/accLogs", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (SPIFFS.exists(accFileString))
    {
      request->send(SPIFFS, accFileString, "text/plain");
    }
  });

  server.on("/temp", HTTP_GET, [](AsyncWebServerRequest *request) {
    temp_sensor_config_t temp_sensor = {
        .dac_offset = TSENS_DAC_L2,
        .clk_div = 6,
    };

    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
    float tsens_out;
    float cumul;
    for(int i = 0; i < 5; i++){
      temp_sensor_read_celsius(&tsens_out);
      vTaskDelay(1);
      cumul += tsens_out;
    }
    request->send(200, "text/plain", String(cumul / 5.0f));
    temp_sensor_stop();
  });

  server.on("/voltage", HTTP_GET, [](AsyncWebServerRequest *request) {
    digitalWrite(MONITOR_EN_PIN, HIGH);
    int cumul = 0;
    int nReadings = 40;

    int read_raw;
    adc2_config_channel_atten( ADC2_CHANNEL_5, ADC_ATTEN_11db );

    for(int i = 0; i < nReadings; i++){
    esp_err_t r = adc2_get_raw( ADC2_CHANNEL_5, ADC_WIDTH_BIT_13, &read_raw);
    cumul += read_raw;
    if ( r == ESP_OK ) {
        Serial.println(read_raw);
    } else if ( r == ESP_ERR_TIMEOUT ) {
        Serial.println("ADC2 used by Wi-Fi.\n");
    }
    }
    float monitor_voltage = ((float)cumul/(float)nReadings)/8192*5;

    request->send(200, "text/plain", String(monitor_voltage));
  });

  server.on("/getAccsCSV", HTTP_GET, [](AsyncWebServerRequest *request) {
    vTaskSuspend(mpuTaskHandle);
    getAccsCSV();
    vTaskResume(mpuTaskHandle);
    request->send(200, "text/plain", content.c_str());
  });

server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request){
  String json = "[";
  int n = WiFi.scanComplete();
  if(n == -2){
    WiFi.scanNetworks(true);
  } else if(n){
    for (int i = 0; i < n; ++i){
      if(i) json += ",";
      json += "{";
      json += "\"rssi\":"+String(WiFi.RSSI(i));
      json += ",\"ssid\":\""+WiFi.SSID(i)+"\"";
      json += ",\"bssid\":\""+WiFi.BSSIDstr(i)+"\"";
      json += ",\"channel\":"+String(WiFi.channel(i));
      json += ",\"secure\":"+String(WiFi.encryptionType(i));
      json += "}";
    }
    WiFi.scanDelete();
    if(WiFi.scanComplete() == -2){
      WiFi.scanNetworks(true);
    }
  }
  json += "]";
  request->send(200, "application/json", json);
  json = String();
});

  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (SPIFFS.exists(logFileString))
    {
      request->send(SPIFFS, logFileString, "text/plain");
    }else
    {
      request->send(200, "text/plain", "no logs available");
    }
  });

  server.on("/laser", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "toggle lasers");
    laser1 = ~laser1;
    pinMode(LAS1_PIN, OUTPUT);
    pinMode(LAS_EN, OUTPUT);
    digitalWrite(LAS_EN, HIGH);
    if(laser1){
    digitalWrite(LAS1_PIN, HIGH);
    }else{
    digitalWrite(LAS1_PIN, LOW);
    }
  }); 

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
    int paramsNr = request->params();

    for (int i = 0; i < paramsNr; i++)
    {
      request->send(200, "text/plain", "message received");
      AsyncWebParameter *p = request->getParam(i);
      if (p->name() == "brightness")
      {
        int val = p->value().toInt();
        if (val >= 0 && val <= 255)
        {
          brightness = val;
          leds.setBrightness(val);
          leds.show();
        }
      }

      if (p->name() == "hue")
      {
        int val = p->value().toInt();
        if (val >= 0 && val <= 65536)
        {
          leds.fill(leds.gamma32(leds.ColorHSV(val, 255, 255)));
          leds.show();
        }
      }

      if (p->name() == "mode")
      {
        setLightingMode(p->value().toInt());
      }

      if(p->name() =="master")
        {
          isMaster=(p->value().toInt()%2);
        }

      if (p->name() == "imu")
      {
        int val = p->value().toInt();
        if(!mpuRunning && val == 1){
          xTaskCreate(mpuTask, "mpuTask", 5000, NULL, 10, &mpuTaskHandle);
        }else if(val == 0){
          mpuRunning = false;
        }
      }
    }
  });
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();

  logln("HTTP server started");

  while (updaterRunning)
  {
    vTaskDelay(10);
  }


  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_wifi_stop();
  vTaskDelete(NULL);
}

void setup()
{
  Serial.begin(115200);
  for(int i = 0; i < 10; i++){
    Serial.println("...");
    vTaskDelay(200);
  }
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  else
  {
    logging = true;
    logln("----------------------System initialized and SPIFFS mounted-------------------------------");
  }

  espNowInit();

  log("TickTime is : ");
  logln((String)(1000.0f / xPortGetTickRateHz()));

  leds.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pinMode(LED_EN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(MPU_INT_PIN, INPUT);
  pinMode(LAS1_PIN, OUTPUT);
  pinMode(MONITOR_EN_PIN, OUTPUT);
  pinMode(MONITOR_PIN, INPUT);


  digitalWrite(LED_EN, HIGH);

  //pinMode(LAS1_PIN, OUTPUT);
  //digitalWrite(LAS1_PIN, LOW);

  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
   pinMode(23, OUTPUT);
  digitalWrite(23, LOW);

  //touch_pad_set_voltage(TOUCH_HVOLT_2V5, TOUCH_LVOLT_0V8, TOUCH_HVOLT_ATTEN_1V);


  gpio_set_drive_capability((gpio_num_t)LAS1_PIN, GPIO_DRIVE_CAP_3);

  leds.fill(leds.ColorHSV(0, 255, 255));
  leds.setBrightness(100);
  leds.show();

  attachInterrupt(BUTTON_PIN, buttonChanged, CHANGE);

  xTaskCreate(lightingTask, "lightingTask", 3000, NULL, 2, &lightingTaskHandle);
  xTaskCreate(mpuTask, "mpuTask", 5000, NULL, 10, &mpuTaskHandle);
  xTaskCreate(updaterTask, "updaterTask", 10000, NULL, 3, &updaterTaskHandle);

}

void handleLongPress()
{
  if (updaterRunning)
  {
    updaterRunning = false;
    logln("updater turned off");
    vTaskSuspend(lightingTaskHandle);
    for(int i = 0; i < 8; i++){  
      leds.setPixelColor(i, leds.Color(255, 0, 0, 0));
    }
    leds.setBrightness(200);
    leds.show();
    vTaskDelay(300);  
    vTaskResume(lightingTaskHandle);

  }
  else if (updaterRunning == false)
  {
    vTaskSuspend(lightingTaskHandle);
    for(int i = 0; i < 8; i++){  
      leds.setPixelColor(i, leds.Color(0, 255, 0, 0));
    }
    leds.setBrightness(200);
    leds.show();
    vTaskDelay(300);  
    vTaskResume(lightingTaskHandle);
    
    updaterRunning = true;
    xTaskCreate(updaterTask, "updaterTask", 10000, NULL, 2, &updaterTaskHandle);
    

    logln("updater turned on");
  }
}

void handleShortPress()
{
  setLightingMode(lightingMode+1);
}



void loop()
{
  vTaskDelay(11);
  dt = lastUpTime - lastDownTime;
  if (dt != lastDT && !buttonHandled)
  {
    if (dt >= 20 && dt <= 1000)
    {
      handleShortPress();
    }
    else if (dt >= 1000 && dt <= 5000)
    {
      handleLongPress();
    }
    lastDT = dt;
    buttonHandled = true;

    if (log10k)
    {
      log("10k took: ");
      log((String)(endTime - startTime));
      logln("ms");
      log10k = false;
    }
  };
}