#include <ESPping.h>
#include <ping.h>

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "WiFi.h"
#include <HTTPClient.h>
#include "WiFiSecrets.h"
// #include "FreeRTOS.h"
// #include "message_buffer.h"

// *************************************************************
// LoRa Messages
#define RF_FREQUENCY 915000000 // Hz

#define TX_OUTPUT_POWER 14 // dBm

#define LORA_BANDWIDTH 2        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 8 // [SF7..SF12]
#define LORA_CODINGRATE 2       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define RX_BUFFER_SIZE 100  // Define the payload size here
#define TX_BUFFER_SIZE 5000 // Define the payload size here

#define LoRa_LED 22
#define WIFI_LED 23
#define BLE_LED 25

char txpacket[TX_BUFFER_SIZE]; // Used for Wi-Fi Transfer
char rxpacket[RX_BUFFER_SIZE];

static RadioEvents_t RadioEvents;

// x = xMessageBufferCreateStatic()

// *************************************************************
//  Used by WiFi
// Your Domain name with URL path or IP address with path

// HTTPClient http;

WiFiClient client;
HTTPClient http;

int16_t txNumber;

int16_t rssi, rxSize;

bool lora_idle = true;

// *************************************************************
// Queue for storing LoRa Messages
#define QUEUE_LENGTH 70
#define ITEM_SIZE sizeof(rxpacket)

/* The variable used to hold the queue's data structure. */
static StaticQueue_t xStaticQueue;

/* The array to use as the queue's storage area.  This must be at least
uxQueueLength * uxItemSize bytes. */
uint8_t ucQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];

QueueHandle_t xQueue;

// *************************************************************
// Timer for sending out the Wi-Fi data to the server.
hw_timer_t *Timer0_Cfg = NULL;
volatile bool sendData2Server = false;

void IRAM_ATTR Timer0_ISR()
{
  sendData2Server = true;
}

// *************************************************************

void WIFISetUp(void)
{
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.begin(WIFISSID, WIFIPASSWRD); // fill in "Your WiFi SSID","Your Password"
  delay(1000);

  byte count = 0;
  Serial.print("Connecting.");
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count++;
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\r\nConnecting...OK.");
    digitalWrite(WIFI_LED, HIGH);
  }
  else
  {
    Serial.println("Connecting...Failed");
    // while(1);
  }
  Serial.println("WIFI Setup done");
}

void WIFIScan(unsigned int value)
{
  unsigned int i;
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.mode(WIFI_MODE_NULL);
  }

  for (i = 0; i < value; i++)
  {
    Serial.println("Scan start...");

    int n = WiFi.scanNetworks();
    Serial.println("Scan done");
    delay(500);

    if (n == 0)
    {
      Serial.println("no network found");
      // while(1);
    }
    else
    {
      Serial.print(n);
      Serial.println("networks found:");
      delay(500);

      for (int i = 0; i < n; ++i)
      {
        // Print SSID and RSSI for each network found
        Serial.print((i + 1));
        Serial.print(":");
        Serial.print((String)(WiFi.SSID(i)));
        Serial.print(" (");
        Serial.print((String)(WiFi.RSSI(i)));
        Serial.println(")");
        ;
        delay(10);
      }
    }
    delay(800);
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  digitalWrite(LoRa_LED, HIGH);
  rxSize = size;
  char *rx = rxpacket;
  memcpy(rxpacket, payload, size);
  // rxpacket[size + 1] = '\0';
  sprintf(&rxpacket[size], ",%d|\0", rssi); // seperator
                                            // rxpacket[size + 1] = '\0';

  xQueueSend(xQueue, &rxpacket, (TickType_t)5);
  // enqueue(&queue, rxpacket);
  // Serial.print("Spaces in Queue: ");
  // Serial.println(uxQueueSpacesAvailable(xQueue));
  Radio.Sleep();
  // Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);
  lora_idle = true;
  digitalWrite(LoRa_LED, LOW);
}

// *************************************************************
void setup()
{
  Serial.begin(115200);
  Mcu.begin();

  Serial.print("CPU Frequency: ");
  Serial.println(getCpuFrequencyMhz());

  txNumber = 0;
  rssi = 0;

  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  pinMode(LoRa_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(BLE_LED, OUTPUT);
  digitalWrite(LoRa_LED, LOW);
  digitalWrite(WIFI_LED, LOW);
  digitalWrite(BLE_LED, LOW);

  WIFISetUp();

  // *********************************
  Timer0_Cfg = timerBegin(0, 24000, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 31000, true);
  timerAlarmEnable(Timer0_Cfg);

  // *********************************

  // initializeQueue(&queue);

  /* Create a queue capable of containing 10 uint64_t values. */
  xQueue = xQueueCreateStatic(QUEUE_LENGTH,
                              ITEM_SIZE,
                              ucQueueStorageArea,
                              &xStaticQueue);

  xTaskCreatePinnedToCore(
      LoRaReadingLoop,   // Function to implement the task
      "LoRaReadingLoop", // Name of the task
      5000,              // Stack size in words
      NULL,              // Task input parameter
      0,                 // Priority of the task
      NULL,              // Task handle.
      0                  // Core where the task should run
  );
}

// *************************************************************
void loop()
{

  if (lora_idle)
  {
    lora_idle = false;
    // Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess();
}

void LoRaReadingLoop(void *pvParameters)
{

  // Serial.println("LoRaReadingLoop on Core: ");
  // Serial.println(xPortGetCoreID());

  while (true)
  {
    if (sendData2Server)
    {
      digitalWrite(WIFI_LED, sendData2Server);
      // Serial.println("Send 2 Server on core ");
      // Serial.println(xPortGetCoreID());

      if (WiFi.status() == WL_CONNECTED)
      { // build message

        txpacket[0] = '\0';
        bool stillHaveRoom = true;
        char peekMsg[RX_BUFFER_SIZE];
        int lenTxPacket = 0;

        while (stillHaveRoom)
        {
          stillHaveRoom = false;
          // get next value - peed
          if (xQueuePeek(xQueue, &peekMsg, (TickType_t)5)) // something off the queue
          {
            // get lenght of new value
            lenTxPacket += strlen(peekMsg);
            // Check have room left on buffer  if yes then add to end of string and pop queue
            if (lenTxPacket < TX_BUFFER_SIZE)
            {
              stillHaveRoom = true;
              xQueueReceive(xQueue, peekMsg, (TickType_t)5);
              strcat(txpacket, peekMsg);
            }
          }
        }

        //    Serial.printf("\r\nSendWiFi \"%s\" length %d\r\n", txpacket, strlen(txpacket));

        // Serial.println("Inside WiFi connected");

        http.begin(DESTIATIONSERVER);
        http.addHeader("Accept", "*/*");
        http.addHeader("User-Agent", "LoRA ShoreSide");
        http.addHeader("Content-Type", "text/plain");

        // Send HTTP POST request
        int httpResponseCode = http.POST(String(txpacket));

        if (httpResponseCode != 201)
        {
          Serial.println("BADDDDDD1 PostResponse:" + String(httpResponseCode));
          // Try a second time.
          delay(100);
          httpResponseCode = http.POST(String(txpacket));
          Serial.println("BADDDDDD222 PostResponse:" + String(httpResponseCode));
        }
        else
        {
          Serial.println("PostResponse:" + String(httpResponseCode));
        }

        sendData2Server = false;
        txpacket[0] = '\0'; // reset the tx buffer

        digitalWrite(WIFI_LED, sendData2Server);
      }
      // }
    }
  }
}
