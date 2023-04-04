#include <EEPROM.h>
#include "BluetoothSerial.h"
#include "WiFi.h"
#include "esp_camera.h"
#include "esp_bt.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"           //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"
#include <HTTPClient.h>

#define PART_BOUNDARY "123456789000000000000987654321"

#define EEPROM_SIZE 512  //Size used from the EEPROM to store ssid and password (max = 512bytes)
#define SSID_INDEX 0
#define PASS_INDEX 128
#define MODE_INDEX 510

BluetoothSerial SerialBT;        //Object for Bluetooth
unsigned long previousTime = 0;  //Used to track elapsed time
unsigned int interval = 1000;    //Time to wait for a bluetooth connection ms (30s)



static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

const char* server_url = "https://bambinoserver0.000webhostapp.com/upload.php";
HTTPClient http;
const int chunckSize = 16000;

String mode;  //mode 0 -> local, mode 1 -> remote
boolean localServerRunning = false;

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
  config.xclk_freq_hz = 7000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // init with high specs to pre-allocate larger buffers
  if (psramFound()) {

    config.frame_size = FRAMESIZE_HD;

    if (mode == "0") {
      config.jpeg_quality = 12;  // 0-63 lower number means higher quality
    } else {
      config.jpeg_quality = 20;
    }
    config.fb_count = 1;
  } else {

    // Serial.println("NO PSRAM ----");
    config.frame_size = FRAMESIZE_HD;

    if (mode == "0") {
      config.jpeg_quality = 12;  // 0-63 lower number means higher quality
    } else {
      config.jpeg_quality = 20;
    }

    config.fb_count = 1;
  }
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_HD);        // VGA|CIF|QVGA|HQVGA|QQVGA   ( UXGA? SXGA? XGA? SVGA? )
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

  Serial.println("Camera Setup DONE");
}

boolean blueToothTimedOut() {
  digitalWrite(RED_LED_GPIO_NUM, HIGH);
  delay(1000);
  digitalWrite(RED_LED_GPIO_NUM, LOW);
  delay(1000);
  unsigned long currentTime = millis();
  Serial.println((currentTime - previousTime) / 1000);
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;
    return true;
  }
  return false;
}

void listenToBlueTooth() {
  //Data recieved on Bluetooth (send any data to signify connection)
  Serial.println(SerialBT.readStringUntil('\n'));
  SerialBT.flush();

  //Wait until recieving ssid
  while (!SerialBT.available())
    ;

  //Save ssid to EEPROM
  String enteredSSID = SerialBT.readStringUntil('\n');
  enteredSSID.remove(enteredSSID.length() - 1, 1);
  EEPROM.writeString(SSID_INDEX, enteredSSID);
  EEPROM.commit();
  //Serial.println("SSID: "+SerialBT.readStringUntil('\n'));

  //Wait until recieving password
  while (!SerialBT.available())
    ;

  //Save password to EEPROM
  String enteredPassword = SerialBT.readStringUntil('\n');
  enteredPassword.remove(enteredPassword.length() - 1, 1);
  EEPROM.writeString(PASS_INDEX, enteredPassword);
  EEPROM.commit();

  //Wait until recieving mode
  while (!SerialBT.available())
    ;

  //Save mode to EEPROM
  String enteredMode = SerialBT.readStringUntil('\n');
  enteredMode.remove(enteredMode.length() - 1, 1);
  EEPROM.writeString(MODE_INDEX, enteredMode);
  EEPROM.commit();

  Serial.println(enteredSSID + " " + enteredPassword + " " + enteredMode);
  // SerialBT.println("Received");
}

void turnOffBlueTooth() {
  //Turn off bluetooth
  btStop();
  SerialBT.end();

  //Free up memory
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
  esp_bt_controller_mem_release(ESP_BT_MODE_IDLE);
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
}

void connectWiFiUsingEEPROM() {
  //Read WiFi Credentials from EEPROM
  String ssid = EEPROM.readString(SSID_INDEX);
  String password = EEPROM.readString(PASS_INDEX);
  mode = EEPROM.readString(MODE_INDEX);
  Serial.println(ssid + " " + password + " " + mode);


  //Try to connect to Wifi using ssid and password
  // WiFi.mode(WIFI_STA);
  WiFi.begin((const char*)ssid.c_str(), (const char*)password.c_str());

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed, Restarting...");
    ESP.restart();  // to restart ESP32
  } else {
    Serial.print("Wifi Connected to ");
    Serial.println(ssid);
  }

  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());
}

static esp_err_t stream_handler(httpd_req_t* req) {
  localServerRunning = true;
  camera_fb_t* fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t* _jpg_buf = NULL;
  char* part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->width > 400) {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char*)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char*)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  localServerRunning = false;
  return res;
}

static esp_err_t turn_flash_on(httpd_req_t* req) {

  Serial.println("Turn On");
  digitalWrite(FLASH_LED_GPIO_NUM, 1);
  return 200;
}

static esp_err_t turn_flash_off(httpd_req_t* req) {
  Serial.println("Turn Off");
  digitalWrite(FLASH_LED_GPIO_NUM, 0);
  return 200;
}

void startLocalCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_open_sockets = 2;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  httpd_uri_t flash_on_uri = {
    .uri = "/flash_on",
    .method = HTTP_GET,
    .handler = turn_flash_on,
    .user_ctx = NULL
  };

  httpd_uri_t flash_off_uri = {
    .uri = "/flash_off",
    .method = HTTP_GET,
    .handler = turn_flash_off,
    .user_ctx = NULL
  };



  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &flash_on_uri);
    httpd_register_uri_handler(stream_httpd, &flash_off_uri);
  }
}

void streamToRemoteServer() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  Serial.println("IMAGE LENGTH: " + String(fb->len));
  Serial.println("IMAGE FORMAT: " + String(fb->format));

  if (http.begin(server_url)) {
    http.addHeader("Content-Type", "image/jpeg");
    http.setTimeout(30000);


    int remaining = fb->len;
    int offset = 0;

    while (remaining > 0) {
      Serial.println("Remaining: " + String(remaining));
      Serial.println("Offset: " + String(offset));
      int chunk_size = min(chunckSize, remaining);
      Serial.println("Chunck Size: " + String(chunk_size));
      int httpResponseCode = http.sendRequest("POST", fb->buf + offset, chunk_size);
      if (httpResponseCode > 0) {
        Serial.printf("HTTP response code: %d\n", httpResponseCode);

      } else {
        Serial.printf("HTTP request failed with error %s\n", http.errorToString(httpResponseCode).c_str());
      }
      offset += chunk_size;
      remaining -= chunk_size;
    }
    http.end();
  }

  //Get flash led state
  http.begin("https://bambinoserver0.000webhostapp.com/get_flash_led.php");
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String response = http.getString();
    bool flashLED = response.toInt();
    digitalWrite(FLASH_LED_GPIO_NUM, flashLED);
  } else {
    Serial.println("Error calling PHP script");
  }

  http.end();

  // Free the photo buffer
  esp_camera_fb_return(fb);
}

void setup() {
  // put your setup code here, to run once:
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  SerialBT.begin("Bambino");
  setupLEDs();

  //After power up or reset Bluetooth will start for 30s
  while (!blueToothTimedOut()) {
    //Check if any data is recieved
    if (!SerialBT.available()) {
      //Nothing is recieved yet
      continue;
    }
    listenToBlueTooth();
    //Break the bluetooth loop
    break;
  }
  turnOffBlueTooth();
  delay(2000);
  connectWiFiUsingEEPROM();
  setupCamera();
  // if (mode == "0") {
  //Local Mode
  startLocalCameraServer();
  // }
}

void loop() {
  // put your main code here, to run repeatedly:
  // if (mode == "1") {
  //Remote Mode
  if (localServerRunning) {
    delay(5000);
    return;
  }
  streamToRemoteServer();
  // }
}
