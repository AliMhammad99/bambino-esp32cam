#include <EEPROM.h>
#include "BluetoothSerial.h"
#include "WiFi.h"
#include <Arduino.h>
#include <base64.h>
#include "FirebaseESP32.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "esp_bt.h"

#define EEPROM_SIZE 512          //Size used from the EEPROM to store ssid and password (max = 512bytes)
BluetoothSerial SerialBT;        //Object for Bluetooth
unsigned long previousTime = 0;  //Used to track elapsed time
unsigned int interval = 1000;    //Time to wait for a bluetooth connection ms (30s)

String FIREBASE_HOST = "https://bambino-4aba4-default-rtdb.firebaseio.com/";
String FIREBASE_AUTH = "AIzaSyDbKf1iJz7E5Yibr_W0UYcg_73NmbxWu-g";
FirebaseData firebaseData;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define RED_LED_GPIO_NUM 33   //Red LED
#define FLASH_LED_GPIO_NUM 4  //Flash LED

void setupLEDs() {
  // Setup LED FLash if LED pin is defined in camera_pins.h
  pinMode(RED_LED_GPIO_NUM, OUTPUT);
  pinMode(FLASH_LED_GPIO_NUM, OUTPUT);
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 5000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // init with high specs to pre-allocate larger buffers
  if (psramFound()) {

    config.frame_size = FRAMESIZE_XGA;

    config.jpeg_quality = 12;  // 0-63 lower number means higher quality

    config.fb_count = 1;
  } else {

    // Serial.println("NO PSRAM ----");    
    config.frame_size = FRAMESIZE_XGA;

    config.jpeg_quality = 12;  // 0-63 lower number means higher quality

    config.fb_count = 1;
  }
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_XGA);     // VGA|CIF|QVGA|HQVGA|QQVGA   ( UXGA? SXGA? XGA? SVGA? )
  s->set_brightness(s, 0);                  // -2 to 2
  s->set_contrast(s, 0);                    // -2 to 2
  s->set_saturation(s, 0);                  // -2 to 2
  s->set_special_effect(s, 0);              // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);                    // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);                    // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);                     // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);               // 0 = disable , 1 = enable
  s->set_aec2(s, 0);                        // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);                    // -2 to 2
  s->set_aec_value(s, 300);                 // 0 to 1200
  s->set_gain_ctrl(s, 1);                   // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);                    // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);                         // 0 = disable , 1 = enable
  s->set_wpc(s, 1);                         // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);                     // 0 = disable , 1 = enable
  s->set_lenc(s, 1);                        // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);                     // 0 = disable , 1 = enable
  s->set_vflip(s, 0);                       // 0 = disable , 1 = enable
  s->set_dcw(s, 1);                         // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);                    // 0 = disable , 1 = enable
  s->set_xclk(s, 0, 2);
}

String getPhotoBase64() {
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();

  if (!fb) {
    // Serial.println("Camera capture failed");
    return "";
  }

  String imageFile = "data:image/jpeg;base64,";
  String encrypt = base64::encode(fb->buf, fb->len);

  esp_camera_fb_return(fb);
  return encrypt;
}

boolean blueToothTimedOut() {
  digitalWrite(RED_LED_GPIO_NUM, HIGH);
  delay(1000);
  digitalWrite(RED_LED_GPIO_NUM, LOW);
  delay(1000);
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;
    return true;
  }
  return false;
}

void setupFirebase(){
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Firebase.setMaxRetry(firebaseData, 3);
  Firebase.setMaxErrorQueue(firebaseData, 30);
  Firebase.enableClassicRequest(firebaseData, true);
}

void setup() {
  // put your setup code here, to run once:
  EEPROM.begin(EEPROM_SIZE);
  // Serial.begin(115200);
  SerialBT.begin("Bambino");
  setupLEDs();

  //After power up or reset Bluetooth will start for 30s
  while (!blueToothTimedOut()) {
    //Check if any data is recieved
    if (!SerialBT.available()) {
      //Nothing is recieved yet
      continue;
    }
    //Data recieved on Bluetooth (send any data to signify connection)
    // Serial.println(SerialBT.readStringUntil('\n'));

    //Wait until recieving ssid
    while (!SerialBT.available())
      ;

    //Save ssid to EEPROM
    String enteredSSID = SerialBT.readStringUntil('\n');
    enteredSSID.remove(enteredSSID.length() - 1, 1);
    EEPROM.writeString(0, enteredSSID);
    EEPROM.commit();
    //Serial.println("SSID: "+SerialBT.readStringUntil('\n'));

    //Wait until recieving password
    while (!SerialBT.available())
      ;

    //Save password to EEPROM
    String enteredPassword = SerialBT.readStringUntil('\n');
    enteredPassword.remove(enteredPassword.length() - 1, 1);
    EEPROM.writeString(256, enteredPassword);
    EEPROM.commit();

    //Break the bluetooth loop
    break;
  }

  //Turn off bluetooth
  btStop();
  SerialBT.end();
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
  esp_bt_controller_mem_release(ESP_BT_MODE_IDLE);
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  // free();
  delay(2000);

  //Read WiFi Credentials from EEPROM
  String ssid = EEPROM.readString(0);
  String password = EEPROM.readString(256);
  // Serial.println(ssid + " " + password);

  //Try to connect to Wifi using ssid and password
  WiFi.mode(WIFI_STA);
  WiFi.begin((const char *)ssid.c_str(), (const char *)password.c_str());

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    // Serial.println("WiFi Failed, Restarting...");
    ESP.restart();  // to restart ESP32
  } else {
    // Serial.print("Wifi Connected to ");
    // Serial.println(ssid);
  }

  setupCamera();
  setupFirebase();
}

void capturePhotoUploadToFirebase() {
  String photoBase64 = getPhotoBase64();
  // Serial.println(photoBase64);
  int nbSubStrings = photoBase64.length() / 10000;  //The number of 10k substrings
  while(!Firebase.RTDB.setInt(&firebaseData, "/nbSubStrings", nbSubStrings));

  String photoPath = "/c";

  for (int i = 0; i < nbSubStrings; i++) {
    FirebaseJson json;
    json.set("p"+String(i), photoBase64.substring(0, 10000));
    Firebase.updateNode(firebaseData, photoPath, json);
    // if (Firebase.updateNode(firebaseData, photoPath, json)) {
    //   // Serial.println(firebaseData.dataPath());
    //   // Serial.println(firebaseData.pushName());
    //   // Serial.println(firebaseData.dataPath() + "/" + firebaseData.pushName());
    //   // Serial.println("Uploaded");
    // } else {
    //   // Serial.println(firebaseData.errorReason());
    // }
    // free();
    photoBase64 = photoBase64.substring(10000, photoBase64.length());
  }

  FirebaseJson json2;
  json2.set("pL", photoBase64);
  Firebase.updateNode(firebaseData, photoPath, json2);
  // if (Firebase.updateNode(firebaseData, photoPath, json2)) {
  //   // Serial.println(firebaseData.dataPath());
  //   // Serial.println(firebaseData.pushName());
  //   // Serial.println(firebaseData.dataPath() + "/" + firebaseData.pushName());
  //   // Serial.println("Uploaded L");
  // } else {
  //   // Serial.println(firebaseData.errorReason());
  // }
}

void updateStateFromFirebase() {
  if (Firebase.RTDB.getBool(&firebaseData, "/FlashLED")) {
    if (firebaseData.dataType() == "boolean") {
      bool boolValue = firebaseData.boolData();
      digitalWrite(FLASH_LED_GPIO_NUM, boolValue);
      // Serial.println(boolValue);
    }
  } else {
    // Serial.println(firebaseData.errorReason());
  }
}

void deletePreviousFrameFromFirebase(){
  Firebase.deleteNode(firebaseData, "/c");

  // if (firebaseData.success()) {
  //   Serial.println("Data deleted successfully!");
  // } else {
  //   Serial.println("Data deletion failed!");
  //   Serial.println(firebaseData.errorReason());
  // }
}

void loop() {
  // put your main code here, to run repeatedly:
  capturePhotoUploadToFirebase();
  updateStateFromFirebase();
  // deletePreviousFrameFromFirebase();
}
