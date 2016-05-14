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
#include "scaleTable10.h"

#define SAMPLING_RATE  10000  // us

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
#define PIN_SW_RUN      3
#define PIN_SW_NOTE_ON  2
#define PIN_SW_TIE      1

Bounce debouncerRun    = Bounce();
Bounce debouncerNoteOn = Bounce();
Bounce debouncerTie    = Bounce();

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
int pos = 0;
bool isRunning = true;
bool isDirty = false;

volatile int ticks = 0;

void setup() {
  int i;
  
  for (i = 0; i < SEQUENCE_N; i++) {
    sequence[i].pitch = i;
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
  
  pinMode(PIN_SW_NOTE_ON, INPUT_PULLUP);
  debouncerNoteOn.attach(PIN_SW_NOTE_ON);
  debouncerNoteOn.interval(5);
  
  pinMode(PIN_SW_TIE, INPUT_PULLUP);
  debouncerTie.attach(PIN_SW_TIE);
  debouncerTie.interval(5);
  
  // PSOC4_DCO
  pinMode(PSOC4_DCO_CS, OUTPUT);
  
  // MCP4922
  pinMode(MCP4922_CS, OUTPUT);
  pinMode(MCP4922_LDAC, OUTPUT);
  
  // LCD5110
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
  //initLCD();  
  isRunning = false;
  isDirty = true;

  // TimerOneの初期化
  Timer1.initialize(SAMPLING_RATE);
  //Timer1.attachInterrupt(updateWhileRun);
}

void loop() {
  int tmp;
  
  // Run/Stopスイッチ
  debouncerRun.update();
  tmp = debouncerRun.read();
  if (tmp == LOW) {
    if (isRunning) {
      isRunning = false;
      Timer1.detachInterrupt();
      //initLCD();
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
    // NoteOnスイッチ
    debouncerNoteOn.update();
    tmp = debouncerNoteOn.read();
    if (tmp == LOW) {
      if (sequence[pos].noteOn) {
        sequence[pos].noteOn = false;
      } else {
        sequence[pos].noteOn = true;
      }
      do { 
        debouncerNoteOn.update();
      } while(debouncerNoteOn.read() == LOW);
      isDirty = true;
    }
    // Tieスイッチ
    debouncerTie.update();
    tmp = debouncerTie.read();
    if (tmp == LOW) {
      if (sequence[pos].tie) {
        sequence[pos].tie = false;
      } else {
        sequence[pos].tie = true;
      }
      do { 
        debouncerTie.update();
      } while(debouncerTie.read() == LOW);
      isDirty = true;
    }
    // ポジションの読み取り
    tmp = pos;
    pos += readRE(1);
    pos = constrain(pos, 0, SEQUENCE_N - 1);
    if (tmp != pos) {
      isDirty = true;
    } 
    // ピッチの読み取り
    tmp = sequence[pos].pitch;
    sequence[pos].pitch += readRE(0);
    sequence[pos].pitch = constrain(sequence[pos].pitch, 0, 12);
    if (tmp != sequence[pos].pitch) {
      isDirty = true;
    }
    
    if (isDirty) {
      updateLCD();
    }
  }
  delay(10);
}

// PSoC 4 DCOに出力
// parameter: frequency: 周波数の10倍値
void outDCO(uint16_t frequency)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PSOC4_DCO_CS, LOW);
  SPI.transfer(CMDM_FREQ_DECI);
  SPI.transfer(frequency >> 8);
  SPI.transfer(frequency & 0xff);
  digitalWrite(PSOC4_DCO_CS, HIGH);
  SPI.endTransaction();
  SPI.end();
/*
  myGLCD.clrScr();
  myGLCD.printNumI(pos, 0, 0);
  myGLCD.printNumI(frequency, 0, 10);
  myGLCD.printNumI(CMDM_FREQ_DECI, 0, 20);
  myGLCD.printNumI(frequency >> 8, 0, 30);
  myGLCD.printNumI(frequency & 0xff, 0, 40);
  myGLCD.update();
  */
}

// DACに出力
// parameter: v: 出力値(0 .. 4095)
void outDAC(int16_t v)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
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
  static uint16_t frequency10;
  
  digitalWrite(PIN_CHECK, flag);
  flag = flag ? false : true;
  
  ticks--;
  if (ticks <= 0) {
    ticks = 625;
    frequency10 = scaleTable10[sequence[pos].pitch + 48];
    pos++;
    if (pos == SEQUENCE_N) {
      pos = 0;
    }
  }
  
  // PSoC DCOにSPI出力
  outDCO(frequency10);  
  // DACに出力
  outDAC(2048);
 
  //updateWhileStop();
  // ラッチ?
}

//-------------------------------------------------
// LCDの描画
void updateLCD() {
  static bool flag;
  int x, y;
  int x1, x2, y1, y2;
  
  //digitalWrite(PIN_CHECK, flag);
  flag = flag ? false : true;
  
  isDirty = false;
  myGLCD.clrScr();
  
  // Pos Indicator Grid
  myGLCD.drawLine(0, 0, 80, 0); 
  // Sequence Grid
  for (x = 0; x <= 16; x++) {
    myGLCD.drawLine(x * 5, 0, x * 5, 48);
  }
  for (y = 0; y <= 13; y++) {
    myGLCD.drawLine(0, y * 3 + 3, 80, y * 3 + 3);
  }
  myGLCD.drawLine(0, 47, 80, 47);
  
  // Pos Indicator
  myGLCD.drawLine(pos * 5 + 1, 1, pos * 5 + 5, 1);
  // Sequence
  for (x = 0; x <= 15; x++) {
    myGLCD.drawRect(x * 5 + 1, 41 - (sequence[x].pitch * 3), x * 5 + 4, 41 - (sequence[x].pitch * 3 + 1));
  }
  // Note & Tie
  for (x = 0; x <= 15; x++) {
    if (sequence[x].noteOn) {
      x1 = x * 5 + 1;
      y2 = 46;
      if (sequence[x].tie)    x2 = x * 5 + 4;
      else                    x2 = x * 5 + 3;
      if (sequence[x].accent) y1 = 43;
      else                    y1 = 43;
      myGLCD.drawRect(x1, y1, x2, y2);
    }  
  }
  myGLCD.update();
}

void displayRunning() {
  myGLCD.clrScr();
  myGLCD.print(" PLAYING ", 15, 21);
  //myGLCD.clrLine(15, 20, 69, 20);
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
