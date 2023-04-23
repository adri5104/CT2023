#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <espnow.h>
#include <ESP8266WiFi.h>

const uint8_t PIN_LED = 2;

Adafruit_MPU6050 mpu;


// direccion mac del recibidor
uint8_t  broadcastAddress[] = {0x18, 0xFE, 0x34, 0xF3, 0x4B, 0xAB};
String success;

// tipo del mensaje
typedef struct data_message
{
  float accelX;
  float accelY;
} struct_message;

// mensaje
struct_message message;


// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);

    Serial.begin(115200);
    while (!Serial) 
    {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
    }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);

  // Lo ponemos en modo Station
  WiFi.mode(WIFI_STA);

  // Iniciamos espNow
  if(esp_now_init())
  {
    Serial.println("ERROR initializing esp_now");
  };

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);


  
}

void loop()
 {
 
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");  
  Serial.println("");

  // Se lo metemos al mensaje
  message.accelX = a.acceleration.x;
  message.accelY = a.acceleration.y;

  // Lo enviamos
  uint8_t result = esp_now_send(broadcastAddress, (uint8_t*) &message, sizeof(message));

  delay(10);
}






