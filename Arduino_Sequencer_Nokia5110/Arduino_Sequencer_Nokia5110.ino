// Arduino_Sequncer_Nokia5110
//
// SPI LCDのNokia5110を使ったシーケンサー
//
// 2016.05.14
//

#include <SPI.h>
#include <TimerOne.h>
#include <LCD5110_Graph.h>
#include <Bounce2.h>

#define SAMPLING_RATE  200  // us

#define PIN_CHECK  0

// SPI
#define SPI_SCK   13
#define SPI_MISO  12
#define SPI_MOSI  11

// PSOC4_DCO
#define PSOC4_DCO_CS 10

// AD8403_DCF
#define AD8403_DCF_CS  9

// MCP4922
#define MCP4922_CS    8
#define MCP4922_LDAC  7

// LCD5110
#define LCD5110_CS  6
#define LCD5110_DC   5
#define LCD5110_RST  4

LCD5110 myGLCD(SPI_SCK, SPI_MOSI, LCD5110_DC, LCD5110_RST, LCD5110_CS);  // SCK, MOSI, DC, RST, CS
extern uint8_t SmallFont[];

// ロータリーエンコーダ
#define PIN_RE0_A  14
#define PIN_RE0_B  15
#define PIN_RE1_A  16
#define PIN_RE1_B  17

// タクトスイッチ
#define PIN_SW_RUN  3

Bounce debouncerRun = Bounce();

// シーケンサーコマンド
#define CMDM_BASE       (0b00000000)
#define CMDM_FREQ_DECI  (CMDM_BASE|0x01)
#define CMDM_PALS_WIDTH (CMDM_BASE|0x02)
#define CMDM_WAV_FORM   (CMDM_BASE|0x03)

// 大域変数
#define SEQUENCE_N  16

struct Sequence {
  int pitch;
  byte octave;
  bool noteOn;
  bool tie;
  bool accent;  
} sequence[SEQUENCE_N];

byte bpm = 120;
bool isRunning = true;
bool isDirty = false;

volatile int ticks = 0;


// PSoC 4 DCOに出力
// parameter: frequency: 周波数の10倍値
void outDCO(uint16_t frequency)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PSOC4_DCO_CS, LOW);
  SPI.transfer(CMDM_FREQ_DECI);
  SPI.transfer(frequency >> 8);
  SPI.transfer(frequency & 0xff);
  digitalWrite(PSOC4_DCO_CS, HIGH);
  SPI.endTransaction();
  SPI.end();
}

// DACに出力
// parameter: v: 出力値(0 .. 4095)
void outDAC(int16_t v)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MCP4922_LDAC, HIGH) ;
  digitalWrite(MCP4922_CS, LOW) ;
  SPI.transfer((v >> 8)| 0x30) ;
  SPI.transfer(v & 0xff) ;
  digitalWrite(MCP4922_CS, HIGH) ;
  digitalWrite(MCP4922_LDAC, LOW) ;
  SPI.endTransaction();
  SPI.end();
}

void updateWhileRun() {
  static bool flag;
  
  digitalWrite(PIN_CHECK, flag);
  flag = flag ? false : true;

  // PSoC DCOにSPI出力
  outDCO(10000);  
  // DACに出力
  outDAC(2048);
 
  //updateWhileStop();
  // ラッチ?
}

void setup() {
  int i;
  int samplingRate;
  
  for (i = 0; i < SEQUENCE_N; i++) {
    sequence[i].pitch = 12;
    sequence[i].octave = 1;
    sequence[i].noteOn = false;
    sequence[i].tie = false;
    sequence[i].accent = false;
  }

  pinMode(PIN_CHECK, OUTPUT);
 
  // ロータリー・エンコーダ
  pinMode(PIN_RE0_A, INPUT_PULLUP);
  pinMode(PIN_RE0_B, INPUT_PULLUP);
  pinMode(PIN_RE1_A, INPUT_PULLUP);
  pinMode(PIN_RE1_B, INPUT_PULLUP);

  // タクトスイッチ  
  pinMode(PIN_SW_RUN, INPUT_PULLUP);
  debouncerRun.attach(PIN_SW_RUN);
  debouncerRun.interval(5);
  
  // PSOC4_DCO
  pinMode(PSOC4_DCO_CS, OUTPUT);
  
  // MCP4922
  pinMode(MCP4922_CS, OUTPUT);
  pinMode(MCP4922_LDAC, OUTPUT);
  
  // LCD5110
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
  initLCD();  

  // TimerOneの初期化
  samplingRate = 200;
  Timer1.initialize(samplingRate);
  //Timer1.attachInterrupt(updateWhileRun);
}

void loop() {
  int tPos, tPrevPos;
  int tmp;
  
  // Run/Stopスイッチ
  debouncerRun.update();
  tmp = debouncerRun.read();
  if (tmp == LOW) {
    if (isRunning) {
      isRunning = false;
      Timer1.detachInterrupt();
      initLCD();
      isDirty = true;
    } else {
      isRunning = true;
      displayRunning();
      Timer1.attachInterrupt(updateWhileRun);
    }
    do { 
      debouncerRun.update();
    } while(debouncerRun.read() == LOW);
  }

  if (!isRunning) {  
    // ピッチの読み取り
    tmp = sequence[tPos].pitch;
    sequence[tPos].pitch += readRE(0);
    sequence[tPos].pitch = constrain(sequence[tPos].pitch, 0, 12);
    if (tmp != sequence[tPos].pitch) {
      isDirty = true;
    }
    if (isDirty) {
      updateLCD();
    }
  }
  
  delay(1);
}

//-------------------------------------------------
// LCDの描画
void initLCD() {
  int x, y;
  
  myGLCD.clrScr();
  for (x = 0; x <= 16; x++) {
    myGLCD.drawLine(x * 5, 0, x * 5, 48);
  }
  for (y = 0; y <= 14; y++) {
    myGLCD.drawLine(0, y * 3, 84, y * 3);
  }
  myGLCD.update();
}

void updateLCD() {
  static bool flag;
  byte tPos, tPrevPos;
  
  //digitalWrite(PIN_CHECK, flag);
  flag = flag ? false : true;
  
  isDirty = false;
  myGLCD.clrRect(tPrevPos * 5 + 1, 1, tPrevPos * 5 + 4, 2);
  myGLCD.drawRect(tPos * 5 + 1, 1, tPos * 5 + 4, 2);
  myGLCD.print("              ", 0, 40);
  myGLCD.printNumI(isRunning, 0, 40);
  myGLCD.printNumI(sequence[tPos].pitch, 20, 40);
  myGLCD.update();
}

void displayRunning() {
  myGLCD.clrScr();
  myGLCD.print("Now Playing", 0, 0);
  myGLCD.update();
}

//-------------------------------------------------
// ロータリーエンコーダの読み取り (Alps@akizuki)
// return: ロータリーエンコーダーの回転方向
//         0:変化なし 1:時計回り -1:反時計回り
//
int readRE(int re_n)
{
  static uint8_t index[2];
  uint8_t rd, rdA, rdB;
  int retval = 0;
  
  switch (re_n) {
  case 0:
    rdA = digitalRead(PIN_RE0_A);
    rdB = digitalRead(PIN_RE0_B);
    break;
  case 1:
    rdA = digitalRead(PIN_RE1_A);
    rdB = digitalRead(PIN_RE1_B);
    break;
  default:
    ;
  }
  rd = rdA << 1 | rdB;
  index[re_n] = (index[re_n] << 2) | rd;
  index[re_n] &= 0b1111;

  switch (index[re_n]) {
  case 0b1101:
    retval = 1;
    break;
  case 0b1000:
    retval = -1;
    break;
  }
  return retval;
}
