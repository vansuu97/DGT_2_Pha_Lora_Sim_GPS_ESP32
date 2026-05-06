#define LED_BUILTIN 45
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define CS 10
#define LOG_FILE "/ThacoLighting_Log.txt"

#define I2C_SLAVE_ADDR 0x12
#define I2C_SDA 8
#define I2C_SCL 9
#define RX_LEN 22

unsigned long lastWriteTime = 0;
uint8_t lineCount = 0;
uint32_t I2C_receive_count = 0;

uint8_t X1 = 1, V1 = 1, GT1 = 1, X2 = 1, V2 = 1, GT2 = 1, X3 = 1, V3 = 1, GT3 = 1;
uint8_t Day = 1, Mon = 1, Year1 = 1, Year2 = 1;
uint8_t Hour = 1, Min = 1, Sec = 1;
uint8_t Volt1 = 1, Volt2 = 1, Current1 = 1, Current2 = 1;

float Power = 0.00, Utmp = 0.00, Itmp = 0.00;

// buffer nhận I2C
uint8_t i2cRxBuf[RX_LEN + 1];
volatile bool i2cDataReady = false;

void onI2CReceive(int len) {
  int i = 0;

  while (Wire.available() && i < RX_LEN) {
    i2cRxBuf[i++] = (uint8_t)Wire.read();
  }

  while (Wire.available()) {
    Wire.read();
  }

  i2cRxBuf[i] = 0;
  i2cDataReady = true;
}

void readLastLineFast() {
  File file = SD.open(LOG_FILE, FILE_READ);
  if (!file) {
    Serial.println("Khong mo duoc file de doc.");
    return;
  }

  size_t fileSize = file.size();
  if (fileSize == 0) {
    Serial.println("File rong.");
    file.close();
    return;
  }

  int32_t pos = fileSize - 1;
  char c;

  // Bỏ qua ký tự xuống dòng ở cuối file nếu có
  while (pos >= 0) {
    file.seek(pos);
    c = file.read();
    if (c != '\n' && c != '\r') {
      break;
    }
    pos--;
  }

  if (pos < 0) {
    Serial.println("File rong.");
    file.close();
    return;
  }

  // Tìm ký tự '\n' gần cuối nhất
  while (pos >= 0) {
    file.seek(pos);
    c = file.read();
    if (c == '\n') {
      pos++;
      break;
    }
    pos--;
  }

  // Nếu file chỉ có 1 dòng
  if (pos < 0) {
    pos = 0;
  }

  file.seek(pos);

  String lastLine = "";
  while (file.available()) {
    c = file.read();
    if (c == '\n' || c == '\r') {
      break;
    }
    lastLine += c;
  }

  file.close();

  Serial.println("===================================== BEGIN READ LAST LINE =======================================");
  if (lastLine.length() > 0) {
    Serial.println(lastLine);
  } else {
    Serial.println("Khong co dong hop le.");
  }
  Serial.println("===================================== END READ LAST LINE =========================================\n\n\n\n\n");
}

void appendLogLine(String str) {
  File file = SD.open(LOG_FILE, FILE_APPEND);
  if (!file) {
    Serial.println("Khong mo duoc file de ghi noi tiep.");
    return;
  }
  file.println(str);
  file.close();
  Serial.println("Data written to SD card");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) { ; }

  delay(2000);
  Serial.println("\n\n\n->===========================<-\n- Initializing SD...");

  SPI.begin(12, 13, 11, 10);
  pinMode(CS, OUTPUT);

  if (!SD.begin(CS)) {
    Serial.println("SD init failed!");
    while (1);
  }

  Serial.println("=> SD init OK.\n");

  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onI2CReceive);

  if (!SD.exists(LOG_FILE)) {
    File file = SD.open(LOG_FILE, FILE_WRITE);
    if (file) {
      file.println("===> Thaco lighting - 0988847155 - https://DenTinHieuGiaoThongVN.com/ - ThacoLighting@gmail.com <===\n\n");
      file.close();
    }
  }

  digitalWrite(LED_BUILTIN, LOW);  // Default: LED ON
  readLastLineFast();
}

void loop() {
  if (i2cDataReady) {
    i2cDataReady = false;
    I2C_receive_count = 0;

    Serial.println("\n=> Data Received! Waitting for decode...");

    if ((i2cRxBuf[0] == '{') && (i2cRxBuf[21] == '}')) {
      Day      = i2cRxBuf[1];
      Mon      = i2cRxBuf[2];
      Year1    = i2cRxBuf[3];
      Year2    = i2cRxBuf[4];
      Hour     = i2cRxBuf[5];
      Min      = i2cRxBuf[6];
      Sec      = i2cRxBuf[7];
      X1       = i2cRxBuf[8];
      V1       = i2cRxBuf[9];
      GT1      = i2cRxBuf[10];
      X2       = i2cRxBuf[11];
      V2       = i2cRxBuf[12];
      GT2      = i2cRxBuf[13];
      X3       = i2cRxBuf[14];
      V3       = i2cRxBuf[15];
      GT3      = i2cRxBuf[16];
      Volt1    = i2cRxBuf[17];
      Volt2    = i2cRxBuf[18];
      Current1 = i2cRxBuf[19];
      Current2 = i2cRxBuf[20];

      Utmp = Volt1 + (Volt2 / 100.00f);
      Itmp = Current1 + (Current2 / 100.00f);
      Power = Utmp * Itmp;

      String newLine = "=> " + String(Day) + "/" + String(Mon) + "/" + String(Year1) + String(Year2) +
                       " | " + String(Hour) + ":" + String(Min) + ":" + String(Sec) +
                       " ==> X1 = " + String(X1) + " | V1 = " + String(V1) + " | D1 = " + String(GT1) +
                       " || X2 = " + String(X2) + " | V2 = " + String(V2) + " | D2 = " + String(GT2) +
                       " || X3 = " + String(X3) + " | V3 = " + String(V3) + " | D3 = " + String(GT3) +
                       " || V = " + String(Volt1) + "." + String(Volt2) +
                       " | I = " + String(Current1) + "." + String(Current2) +
                       " || Power = " + String(Power, 2);
      Serial.println("========================================= Data Decode: ===========================================");
      Serial.println(newLine);
      appendLogLine(newLine);
      readLastLineFast();
    }
  }

  I2C_receive_count++;
  if (I2C_receive_count > 250) {
    I2C_receive_count = 250;
    digitalWrite(LED_BUILTIN, HIGH); // LED OFF
  } else {
    digitalWrite(LED_BUILTIN, LOW);  // LED ON
  }

  delay(1);
}