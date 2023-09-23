#include <ESPping.h>
#include <ping.h>

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "WiFi.h"
#include <HTTPClient.h>
#include "WiFiSecrets.h"

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

// *************************************************************
//  Used by WiFi
// Your Domain name with URL path or IP address with path
String serverName = "https://www.pultneyvilleyachtclub.org/api/TestJSON/create.php";
// HTTPClient http;

int16_t txNumber;

int16_t rssi, rxSize;

bool lora_idle = true;

// *************************************************************
// Queue for storing LoRa Messages
#define QUEUE_SIZE 50
// Structure for the circular queue
struct CircularQueue
{

  char data[QUEUE_SIZE][RX_BUFFER_SIZE];

  int front, rear;
};

struct CircularQueue queue;

// *************************************************************
// Timer for sending out the Wi-Fi data to the server.
hw_timer_t *Timer0_Cfg = NULL;
bool sendData2Server = false;

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

// *************************************************************
// Queue Routines

// Function to initialize the circular queue

void initializeQueue(struct CircularQueue *queue)
{

  queue->front = queue->rear = -1;
}

// Function to check if the queue is empty
int isEmpty(struct CircularQueue *queue)
{

  return (queue->front == -1);
}

// Function to check if the queue has members and there is room for data on the TX buffer.
bool Need2HaveRoom4More(struct CircularQueue *queue, int txBufferUsed)
{
  int msgLength = 0;   // lenght of message we want to send
  int bufferSpace = 0; // Left in buffer
  // Check not empty
  if (queue->front == -1)
    return false;

  // get length of next message
  msgLength = strlen(queue->data[queue->front]);

  // Check room on TX buffer to send out
  bufferSpace = TX_BUFFER_SIZE - txBufferUsed;

  if (msgLength + 2 <= bufferSpace)
    return true;
  else
    return false;
}

// Function to add an element to the queue

int enqueue(struct CircularQueue *queue, const char *item)
{

  if ((queue->rear + 1) % QUEUE_SIZE == queue->front)
  {

    // Queue is full

    return 0; // Failure
  }

  if (isEmpty(queue))
  {

    queue->front = queue->rear = 0;
  }
  else
  {

    queue->rear = (queue->rear + 1) % QUEUE_SIZE;
  }

  strcpy(queue->data[queue->rear], item);

  return 1; // Success
}

// Function to copy a string from the queue to a destination
void copyStringFromQueue(struct CircularQueue *queue, char *destination, int destSize)
{

  if (isEmpty(queue))
  {

    // Queue is empty, handle this case accordingly

    strncat(destination, "Queue is empty", destSize);
  }
  else
  {

    strncat(destination, queue->data[queue->front], destSize);
    if (queue->front == queue->rear)
    {
      queue->front = -1;
      queue->rear = -1;
    }
    else
    {
      queue->front = (queue->front + 1) % QUEUE_SIZE;
    }
  }
}

// *************************************************************
void setup()
{
  Serial.begin(115200);
  Mcu.begin();

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
  timerAlarmWrite(Timer0_Cfg, 30000, true);
  timerAlarmEnable(Timer0_Cfg);

  // *********************************

  initializeQueue(&queue);
}

// *************************************************************
void loop()
{

  if (sendData2Server)
  {
    digitalWrite(WIFI_LED, sendData2Server);
    Serial.println("Send 2 Server");
    sendData2Server = false;

    if (WiFi.status() == WL_CONNECTED)
    // if ping
    { // build message
      while (Need2HaveRoom4More(&queue, strlen(txpacket)))
      {
        copyStringFromQueue(&queue, txpacket, RX_BUFFER_SIZE);
      }

      Serial.printf("\r\nSendWiFi \"%s\" length %d\r\n", txpacket, strlen(txpacket));

      WiFiClient client;
      HTTPClient http;

      http.begin(DESTIATIONSERVER);
      http.addHeader("Accept", "*/*");
      http.addHeader("User-Agent", "LoRA ShoreSide");
      http.addHeader("Content-Type", "text/plain");
      String httpRequestData = "LoRa hi there";
      // Send HTTP POST request
      int httpResponseCode = http.POST(String(txpacket));
      Serial.println(httpRequestData); //"PostResponse:" + String(httpResponseCode));

      txpacket[0] = '\0'; // reset the tx buffer

      digitalWrite(WIFI_LED, sendData2Server);
    }
  }
  if (lora_idle)
  {
    lora_idle = false;
    // sendData2Server = false;
    Serial.println("into WiFi TX mode");

    Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess();
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
  enqueue(&queue, rxpacket);

  Radio.Sleep();
  Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);
  lora_idle = true;
  digitalWrite(LoRa_LED, LOW);
}
