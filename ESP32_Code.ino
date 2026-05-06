#define LED_BUILTIN 45

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <string.h>

#define CS 10
#define LOG_FILE "/ThacoLighting_Log.txt"

#define I2C_SLAVE_ADDR 0x12
#define I2C_SDA 8
#define I2C_SCL 9

/* STM32 -> ESP32 frame format: CMD + DATA + Counter + CRC8 */
#define CMD_REALTIME        0x01U
#define CMD_NORMAL_SETTING  0x02U
#define CMD_PEAK_SETTING    0x03U

#define FRAME_REALTIME_LEN        17U
#define FRAME_NORMAL_SETTING_LEN  16U
#define FRAME_PEAK_SETTING_LEN    16U
#define FRAME_MAX_LEN             FRAME_REALTIME_LEN

#define REALTIME_COUNTER_INDEX        15U
#define REALTIME_CRC_INDEX            16U
#define NORMAL_SETTING_COUNTER_INDEX  14U
#define NORMAL_SETTING_CRC_INDEX      15U
#define PEAK_SETTING_COUNTER_INDEX    14U
#define PEAK_SETTING_CRC_INDEX        15U

static const uint8_t CRC8_TABLE[256] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
  0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
  0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
  0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
  0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
  0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
  0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
  0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
  0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
  0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
  0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
  0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
  0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
  0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
  0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
  0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
  0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

unsigned long lastWriteTime = 0;
uint8_t lineCount = 0;
uint32_t I2C_receive_count = 0;

/* Normal setting */
uint8_t X1 = 1, V1 = 1, GT1 = 1;
uint8_t X2 = 1, V2 = 1, GT2 = 1;
uint8_t X3 = 1, V3 = 1, GT3 = 1;

/* Realtime */
uint8_t Day = 1, Mon = 1, Year = 1;
uint8_t Hour = 1, Min = 1, Sec = 1;
uint8_t Volt1 = 1, Volt2 = 1, Current1 = 1, Current2 = 1;

/* Blink yellow and peak time setting */
uint8_t begin_hour1 = 0, begin_min1 = 0, end_hour1 = 0, end_min1 = 0;  // Chop vang 1
uint8_t begin_hour2 = 0, begin_min2 = 0, end_hour2 = 0, end_min2 = 0;  // Chop vang 2
uint8_t begin_hour3 = 0, begin_min3 = 0, end_hour3 = 0, end_min3 = 0;  // Cao diem

/* Peak setting */
uint8_t CaoDiem_X1 = 5, CaoDiem_V1 = 3, CaoDiem_GT1 = 2;
uint8_t CaoDiem_X2 = 5, CaoDiem_V2 = 3, CaoDiem_GT2 = 2;
uint8_t CaoDiem_X3 = 5, CaoDiem_V3 = 3, CaoDiem_GT3 = 2;

uint8_t realtimeCounter = 0;
uint8_t normalCounter = 0;
uint8_t peakCounter = 0;

uint8_t realtimeCrc = 0;
uint8_t normalCrc = 0;
uint8_t peakCrc = 0;

bool realtimeReady = false;
bool normalReady = false;
bool peakReady = false;
uint8_t lastLoggedCounter = 0xFFU;

float Power = 0.00f, Utmp = 0.00f, Itmp = 0.00f;

/* I2C receive buffer. onReceive only copies data; loop decodes later. */
volatile uint8_t i2cRxLen = 0;
volatile bool i2cDataReady = false;
uint8_t i2cRxBuf[FRAME_MAX_LEN];
uint8_t i2cWorkBuf[FRAME_MAX_LEN];
uint8_t i2cWorkLen = 0;

static uint8_t crc8_cal(const uint8_t *data, size_t length) {
  uint8_t crc = 0x00U;
  for (size_t i = 0; i < length; ++i) {
    crc = CRC8_TABLE[crc ^ data[i]];
  }
  return crc;
}

static bool checkFrameCrc(const uint8_t *frame, uint8_t len) {
  if (len < 2U) {
    return false;
  }

  const uint8_t crcIndex = (uint8_t)(len - 1U);
  const uint8_t expected = crc8_cal(frame, crcIndex);
  return (expected == frame[crcIndex]);
}

static void printFrameHex(const uint8_t *frame, uint8_t len) {
  Serial.print("RX ");
  Serial.print(len);
  Serial.print(" byte: ");
  for (uint8_t i = 0; i < len; i++) {
    if (frame[i] < 0x10U) {
      Serial.print('0');
    }
    Serial.print(frame[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void onI2CReceive(int len) {
  uint8_t i = 0;

  while (Wire.available() && (i < FRAME_MAX_LEN)) {
    i2cRxBuf[i++] = (uint8_t)Wire.read();
  }

  while (Wire.available()) {
    (void)Wire.read();
  }

  i2cRxLen = i;
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

  int32_t pos = (int32_t)fileSize - 1;
  char c;

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

  while (pos >= 0) {
    file.seek(pos);
    c = file.read();
    if (c == '\n') {
      pos++;
      break;
    }
    pos--;
  }

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

static String twoDigit(uint8_t value) {
  if (value < 10U) {
    return "0" + String(value);
  }
  return String(value);
}

static String crcHex(uint8_t value) {
  String s = "0x";
  if (value < 0x10U) {
    s += "0";
  }
  s += String(value, HEX);
  s.toUpperCase();
  return s;
}

static String buildLogLine(void) {
  Utmp = Volt1 + (Volt2 / 100.00f);
  Itmp = Current1 + (Current2 / 100.00f);
  Power = Utmp * Itmp;

  String newLine = "=> " + String(Day) + "/" + String(Mon) + "/20" + twoDigit(Year) +
                   " | " + String(Hour) + ":" + String(Min) + ":" + String(Sec) +

                   " ==> X1 = " + String(X1) + " | V1 = " + String(V1) + " | D1 = " + String(GT1) +
                   " || X2 = " + String(X2) + " | V2 = " + String(V2) + " | D2 = " + String(GT2) +
                   " || X3 = " + String(X3) + " | V3 = " + String(V3) + " | D3 = " + String(GT3) +

                   " || V = " + String(Volt1) + "." + twoDigit(Volt2) +
                   " | I = " + String(Current1) + "." + twoDigit(Current2) +
                   " || Power = " + String(Power, 2) +

                   " || ChopVang1 = " + twoDigit(begin_hour1) + ":" + twoDigit(begin_min1) +
                   "-" + twoDigit(end_hour1) + ":" + twoDigit(end_min1) +
                   " || ChopVang2 = " + twoDigit(begin_hour2) + ":" + twoDigit(begin_min2) +
                   "-" + twoDigit(end_hour2) + ":" + twoDigit(end_min2) +

                   " || CaoDiemTime = " + twoDigit(begin_hour3) + ":" + twoDigit(begin_min3) +
                   "-" + twoDigit(end_hour3) + ":" + twoDigit(end_min3) +
                   " || CaoDiem_X1 = " + String(CaoDiem_X1) +
                   " | CaoDiem_V1 = " + String(CaoDiem_V1) +
                   " | CaoDiem_D1 = " + String(CaoDiem_GT1) +
                   " || CaoDiem_X2 = " + String(CaoDiem_X2) +
                   " | CaoDiem_V2 = " + String(CaoDiem_V2) +
                   " | CaoDiem_D2 = " + String(CaoDiem_GT2) +
                   " || CaoDiem_X3 = " + String(CaoDiem_X3) +
                   " | CaoDiem_V3 = " + String(CaoDiem_V3) +
                   " | CaoDiem_D3 = " + String(CaoDiem_GT3) +

                   " || Counter RT/N/P = " + String(realtimeCounter) +
                   "/" + String(normalCounter) +
                   "/" + String(peakCounter) +
                   " || CRC RT/N/P = " + crcHex(realtimeCrc) +
                   "/" + crcHex(normalCrc) +
                   "/" + crcHex(peakCrc);

  return newLine;
}

static void tryWriteLog(void) {
  if ((realtimeReady == false) || (normalReady == false) || (peakReady == false)) {
    return;
  }

  /* STM32 AppI2CvsESP32_SendAll() sends all 3 frames with the same Counter. */
  if ((realtimeCounter != normalCounter) || (realtimeCounter != peakCounter)) {
    return;
  }

  if (lastLoggedCounter == realtimeCounter) {
    return;
  }

  lastLoggedCounter = realtimeCounter;

  String newLine = buildLogLine();
  Serial.println("========================================= Data Decode: ===========================================");
  Serial.println(newLine);
  appendLogLine(newLine);
  readLastLineFast();
}

static void decodeRealtimeFrame(const uint8_t *frame) {
  Day      = frame[1];
  Mon      = frame[2];
  Year     = frame[3];
  Hour     = frame[4];
  Min      = frame[5];
  Sec      = frame[6];
  Volt1    = frame[7];
  Volt2    = frame[8];
  Current1 = frame[9];
  Current2 = frame[10];

  begin_hour1 = frame[11];
  begin_min1  = frame[12];
  end_hour1   = frame[13];
  end_min1    = frame[14];

  realtimeCounter = frame[REALTIME_COUNTER_INDEX];
  realtimeCrc = frame[REALTIME_CRC_INDEX];
  realtimeReady = true;

  Serial.print("REALTIME OK, Counter = ");
  Serial.println(realtimeCounter);
}

static void decodeNormalSettingFrame(const uint8_t *frame) {
  X1  = frame[1];
  V1  = frame[2];
  GT1 = frame[3];

  X2  = frame[4];
  V2  = frame[5];
  GT2 = frame[6];

  X3  = frame[7];
  V3  = frame[8];
  GT3 = frame[9];

  begin_hour2 = frame[10];
  begin_min2  = frame[11];
  end_hour2   = frame[12];
  end_min2    = frame[13];

  normalCounter = frame[NORMAL_SETTING_COUNTER_INDEX];
  normalCrc = frame[NORMAL_SETTING_CRC_INDEX];
  normalReady = true;

  Serial.print("NORMAL_SETTING OK, Counter = ");
  Serial.println(normalCounter);
}

static void decodePeakSettingFrame(const uint8_t *frame) {
  begin_hour3 = frame[1];
  begin_min3  = frame[2];
  end_hour3   = frame[3];
  end_min3    = frame[4];

  CaoDiem_X1  = frame[5];
  CaoDiem_V1  = frame[6];
  CaoDiem_GT1 = frame[7];

  CaoDiem_X2  = frame[8];
  CaoDiem_V2  = frame[9];
  CaoDiem_GT2 = frame[10];

  CaoDiem_X3  = frame[11];
  CaoDiem_V3  = frame[12];
  CaoDiem_GT3 = frame[13];

  peakCounter = frame[PEAK_SETTING_COUNTER_INDEX];
  peakCrc = frame[PEAK_SETTING_CRC_INDEX];
  peakReady = true;

  Serial.print("PEAK_SETTING OK, Counter = ");
  Serial.println(peakCounter);
}

static void processI2CFrame(const uint8_t *frame, uint8_t len) {
  if (len == 0U) {
    return;
  }

  const uint8_t cmd = frame[0];

  if (((cmd == CMD_REALTIME) && (len != FRAME_REALTIME_LEN)) ||
      ((cmd == CMD_NORMAL_SETTING) && (len != FRAME_NORMAL_SETTING_LEN)) ||
      ((cmd == CMD_PEAK_SETTING) && (len != FRAME_PEAK_SETTING_LEN))) {
    Serial.print("Sai do dai frame. CMD = 0x");
    Serial.print(cmd, HEX);
    Serial.print(", len = ");
    Serial.println(len);
    printFrameHex(frame, len);
    return;
  }

  if ((cmd != CMD_REALTIME) && (cmd != CMD_NORMAL_SETTING) && (cmd != CMD_PEAK_SETTING)) {
    Serial.print("CMD khong hop le: 0x");
    Serial.println(cmd, HEX);
    printFrameHex(frame, len);
    return;
  }

  if (checkFrameCrc(frame, len) == false) {
    Serial.print("CRC sai. CMD = 0x");
    Serial.print(cmd, HEX);
    Serial.print(", len = ");
    Serial.println(len);
    printFrameHex(frame, len);
    return;
  }

  I2C_receive_count = 0;

  switch (cmd) {
    case CMD_REALTIME:
      decodeRealtimeFrame(frame);
      break;

    case CMD_NORMAL_SETTING:
      decodeNormalSettingFrame(frame);
      break;

    case CMD_PEAK_SETTING:
      decodePeakSettingFrame(frame);
      break;

    default:
      break;
  }

  tryWriteLog();
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
    while (1) { ; }
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
    noInterrupts();
    i2cWorkLen = i2cRxLen;
    if (i2cWorkLen > FRAME_MAX_LEN) {
      i2cWorkLen = FRAME_MAX_LEN;
    }
    memcpy(i2cWorkBuf, i2cRxBuf, i2cWorkLen);
    i2cDataReady = false;
    interrupts();

    Serial.println("\n=> I2C frame received. Decode...");
    processI2CFrame(i2cWorkBuf, i2cWorkLen);
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
