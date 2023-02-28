// #include <EEPROM.h>
// #include "BluetoothSerial.h"
#include "WiFi.h"
#include "esp_camera.h"
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <SPIFFS.h>
#include <FS.h>
#include <Firebase_ESP_Client.h>
//Provide the token generation process info.
#include <addons/TokenHelper.h>

//Firebase user: alimhmd.engineer@gmail.com alimhmd.engineer123456
//Storage bucket id: bambino-4aba4.appspot.com
//API Key: AIzaSyDbKf1iJz7E5Yibr_W0UYcg_73NmbxWu-g

#define EEPROM_SIZE 512          //Size used from the EEPROM to store ssid and password (max = 512bytes)
// BluetoothSerial SerialBT;        //Object for Bluetooth
unsigned long previousTime = 0;  //Used to track elapsed time
unsigned int interval = 30000;   //Time to wait for a bluetooth connection ms (30s)

// Insert Firebase project API Key
#define API_KEY "AIzaSyDbKf1iJz7E5Yibr_W0UYcg_73NmbxWu-g";

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "alimhmd.engineer@gmail.com"
#define USER_PASSWORD "alimhmd.engineer123456"

// Insert Firebase storage bucket ID e.g bucket-name.appspot.com
#define STORAGE_BUCKET_ID "bambino-4aba4.appspot.com"

// Photo File Name to save in SPIFFS
#define FILE_PHOTO "/photo.jpg"

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
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

//Define Firebase Data objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig configF;

bool takeNewPhoto = true;
bool taskCompleted = false;

const char* ssid = "Ali_Mhammad";
const char* password = "asdfghjkl123456";

// const char* ssid = "\\MAHDI//";
// const char* password = "asdfghjkl123";

void setup() {
  // put your setup code here, to run once:
  // EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  // SerialBT.begin("Bambino");

  // //After power up or reset Bluetooth will start for 30s
  // while (!blueToothTimedOut()) {
  //   //Check if any data is recieved
  //   if (!SerialBT.available()) {
  //     //Nothing is recieved yet
  //     continue;
  //   }
  //   //Data recieved on Bluetooth (send any data to signify connection)
  //   Serial.println(SerialBT.readStringUntil('\n'));

  //   //Wait until recieving ssid
  //   while (!SerialBT.available())
  //     ;

  //   //Save ssid to EEPROM
  //   String enteredSSID = SerialBT.readStringUntil('\n');
  //   enteredSSID.remove(enteredSSID.length() - 1, 1);
  //   EEPROM.writeString(0, enteredSSID);
  //   EEPROM.commit();
  //   //Serial.println("SSID: "+SerialBT.readStringUntil('\n'));

  //   //Wait until recieving password
  //   while (!SerialBT.available())
  //     ;

  //   //Save password to EEPROM
  //   String enteredPassword = SerialBT.readStringUntil('\n');
  //   enteredPassword.remove(enteredPassword.length() - 1, 1);
  //   EEPROM.writeString(256, enteredPassword);
  //   EEPROM.commit();

  //   //Break the bluetooth loop
  //   break;
  // }
  // //Read WiFi Credentials from EEPROM
  // String ssid = EEPROM.readString(0);
  // String password = EEPROM.readString(256);
  // Serial.println(ssid + " " + password);

  // //Turn off bluetooth
  // btStop();
  // delay(2000);

  //Try to connect to Wifi using ssid and password
  WiFi.mode(WIFI_STA);
  // WiFi.begin((const char *)ssid.c_str(), (const char *)password.c_str());
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed, Restarting...");
    ESP.restart();  // to restart ESP32
  } else {
    Serial.print("Wifi Connected to ");
    Serial.println(ssid);
  }

  initSPIFFS();
  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  initCamera();

  //Firebase
  // Assign the api key
  configF.api_key = API_KEY;
  //Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  //Assign the callback function for the long running token generation task
  configF.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&configF, &auth);
  Firebase.reconnectWiFi(true);


  // capturePhotoSaveSpiffs();
  // if (Firebase.ready() /*&& !taskCompleted*/) {
  //   // taskCompleted = true;
  //   Serial.print("Uploading picture... ");

  //   //MIME type should be valid to avoid the download problem.
  //   //The file systems for flash and SD/SDMMC can be changed in FirebaseFS.h.
  //   if (Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID /* Firebase Storage bucket id */, FILE_PHOTO /* path to local file */, mem_storage_type_flash /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, FILE_PHOTO /* path of remote file stored in the bucket */, "image/jpeg" /* mime type */)) {
  //     Serial.printf("\nDownload URL: %s\n", fbdo.downloadURL().c_str());
  //   } else {
  //     Serial.println(fbdo.errorReason());
  //   }
  // }
}

boolean blueToothTimedOut() {
  delay(1000);
  unsigned long currentTime = millis();
  Serial.println(currentTime / 1000);
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;
    Serial.println("30 Seconds Over");
    return true;
  }
  return false;
}

// Check if photo capture was successful
bool checkPhoto(fs::FS &fs) {
  File f_pic = fs.open(FILE_PHOTO);
  unsigned int pic_sz = f_pic.size();
  return (pic_sz > 100);
}

// Capture Photo and Save it to SPIFFS
void capturePhotoSaveSpiffs(void) {
  camera_fb_t *fb = NULL;  // pointer
  bool ok = 0;             // Boolean indicating if the picture has been taken correctly
  do {
    // Take a photo with the camera
    Serial.println("Taking a photo...");

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }
    // Photo file name
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
    // Insert the data in the photo file
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    } else {
      file.write(fb->buf, fb->len);  // payload (image), payload length
      Serial.print("The picture has been saved in ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes");
      Serial.println(file);
    }
    // Close the file
    file.close();
    esp_camera_fb_return(fb);

    // check if file has been correctly saved in SPIFFS
    ok = checkPhoto(SPIFFS);
  } while (!ok);
}

void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  } else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
}

void initCamera() {
  // OV2640 camera module
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("− failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" − not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (takeNewPhoto) {
    capturePhotoSaveSpiffs();
    takeNewPhoto = false;
  }

  if (Firebase.ready() && !taskCompleted) {
    taskCompleted = true;
    listDir(SPIFFS, "/", 0);
    Serial.println("Uploading picture... ");

    //MIME type should be valid to avoid the download problem.
    //The file systems for flash and SD/SDMMC can be changed in FirebaseFS.h.
    if (Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID /* Firebase Storage bucket id */, FILE_PHOTO /* path to local file */, mem_storage_type_flash /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, FILE_PHOTO /* path of remote file stored in the bucket */, "image/jpeg" /* mime type */)) {
      Serial.printf("\nDownload URL: %s\n", fbdo.downloadURL().c_str());
    } else {
      takeNewPhoto = true;
      taskCompleted = false;
      Serial.println(fbdo.errorReason());
      listDir(SPIFFS, "/", 0);
    }
  }
}
