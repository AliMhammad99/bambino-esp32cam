#include <EEPROM.h>
#include "BluetoothSerial.h"
#include "WiFi.h"


#define EEPROM_SIZE 512          //Size used from the EEPROM to store ssid and password (max = 512bytes)
BluetoothSerial SerialBT;        //Object for Bluetooth
unsigned long previousTime = 0;  //Used to track elapsed time
unsigned int interval = 30000;   //Time to wait for a bluetooth connection ms (30s)

void setup() {
  // put your setup code here, to run once:
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  SerialBT.begin("Bambino");

  //After power up or reset Bluetooth will start for 30s
  while (!blueToothTimedOut()) {
    //Check if any data is recieved
    if (!SerialBT.available()) {
      //Nothing is recieved yet
      continue;
    }
    //Data recieved on Bluetooth (send any data to signify connection)
    Serial.println(SerialBT.readStringUntil('\n'));

    //Wait until recieving ssid
    while (!SerialBT.available());

    //Save ssid to EEPROM
    String enteredSSID = SerialBT.readStringUntil('\n');
    enteredSSID.remove(enteredSSID.length()-1, 1);
    EEPROM.writeString(0, enteredSSID);
    EEPROM.commit();
    //Serial.println("SSID: "+SerialBT.readStringUntil('\n'));

    //Wait until recieving password
    while (!SerialBT.available());

    //Save password to EEPROM
    String enteredPassword = SerialBT.readStringUntil('\n');
    enteredPassword.remove(enteredPassword.length()-1, 1);
    EEPROM.writeString(256, enteredPassword);
    EEPROM.commit();

    //Break the bluetooth loop
    break;
  }
  //Read WiFi Credentials from EEPROM
  String ssid = EEPROM.readString(0);
  String password = EEPROM.readString(256);
  Serial.println(ssid + " " + password);

  //Turn off bluetooth
  btStop();
  delay(2000);

  //Try to connect to Wifi using ssid and password
  WiFi.mode(WIFI_STA);
  WiFi.begin((const char*)ssid.c_str(), (const char*)password.c_str());

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed, Restarting...");
    ESP.restart();  // to restart ESP32
  } else {
    Serial.print("Wifi Connected to ");
    Serial.println(ssid);
  }
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

void loop() {
  // put your main code here, to run repeatedly:
}
