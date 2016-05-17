// Arduino_Sequncer_Nokia5110
//
// SPI LCDのNokia5110を使ったシーケンサー
//
// 2016.05.17 周波数の10倍値を32bitに
// 2016.05.14
//

#include <SPI.h>
#include <TimerOne.h>
#include <LCD5110_Graph.h>
#include <Bounce2.h>
#include "scaleTable10.h"

#define SAMPLING_RATE  5000  // us
#define BPM_INIT  30

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
#define PIN_SW_TIE      9  //デバッグ用

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
  //uint16_t frequency10;
  uint32_t frequency32;
  bool noteOn;
  bool tie;
  int32_t tieDelta;
  bool accent;  
} sequence[SEQUENCE_N];

byte bpm = BPM_INIT;
uint16_t noteLen;
int pos = 0;
bool isRunning = true;
bool isDirty = false;

volatile int ticks = 0;

void dump() {
  struct Sequence *seq;
  char buff[200];
  int i;
  Serial.println("i\tpitch\toctave\tfrequency32\tnoteOn\ttie\ttieDelta\taccent");
  for (i = 0; i < SEQUENCE_N; i++) {
    seq = &sequence[i];
    sprintf(buff, "%d\t%d\t%d\t%lu\t%d\t%d\t%ld\t%d",
      i, seq->pitch, seq->octave, seq->frequency32, seq->noteOn, seq->tie, seq->tieDelta, seq->accent);
    Serial.println(buff);
  }
}  
    
uint16_t calcFrequency10(struct Sequence& seq) {
  return scaleTable10[seq.pitch + seq.octave * 12 + 24];
}

void setup() {
  int i;
  
  Serial.begin(9600);
  
  for (i = 0; i < SEQUENCE_N; i++) {
    sequence[i].pitch = i;
    sequence[i].octave = 1;
    //sequence[i].frequency10 = scaleTable10[sequence[i].pitch + sequence[i].octave * 12 + 24];
    sequence[i].frequency32 = (uint32_t)calcFrequency10(sequence[i]) << 16;
    sequence[i].noteOn = false;
    sequence[i].tie = true;
    sequence[i].tieDelta = 0;
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
      prepareToRun();
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
    if (toggleSW(debouncerNoteOn, sequence[pos].noteOn)) {
      isDirty = true;
    }
    // Tieスイッチ
    if (toggleSW(debouncerTie, sequence[pos].tie)) {
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
      sequence[pos].frequency32 = (uint32_t)calcFrequency10(sequence[pos]) << 16;
      isDirty = true;
      dump();
    }
    // LCDの更新
    if (isDirty) {
      updateLCD();
    }
  }
  delay(10);
}

void prepareToRun() {
  int i;
  
  noteLen = 15000 / ((long)bpm * SAMPLING_RATE / 1000);
  for (i = 0; i < SEQUENCE_N - 1; i++) {
    if (sequence[i].tie) {
      sequence[i].tieDelta = ((int64_t)sequence[i + 1].frequency32 - sequence[i].frequency32) / noteLen;
    } else {
      sequence[i].tieDelta = 0;
    }
  }
}

void updateWhileRun() {
  static bool flag;
  static uint32_t freq;
  static int32_t f_delta;
  static struct Sequence *seq;
  char buff[200];
  
  digitalWrite(PIN_CHECK, flag);
  flag = flag ? false : true;
 
  ticks--;
  if (ticks <= 0) {
    ticks = noteLen;
    seq = &sequence[pos];
    freq    = sequence[pos].frequency32;
    f_delta = sequence[pos].tieDelta;
    pos++;
    if (pos == SEQUENCE_N) {
      pos = 0;
    }
  }

  Serial.println("next pos\tfreq\tf_delta");
  sprintf(buff, "%d\t%lu\t%ld", pos, freq, f_delta);
  Serial.println(buff);
  
  SPI.begin();
  // PSoC DCOにSPI出力
  outDCO((uint16_t)(freq >> 16));
  freq += f_delta;
  // AD8403 DCFにSPI出力  
  outDCF(128, 128);
  // DACにSPI出力
  outDAC(2048);
  SPI.end();
  
  /*
  myGLCD.clrScr();
  myGLCD.printNumI(freq >> 16, 0, 0);
  myGLCD.printNumI(f_delta, 0, 10);
  myGLCD.printNumI(seq->pitch, 0, 20);
  myGLCD.printNumI(seq->octave, 0, 30);
  myGLCD.update();
  */
  // ラッチ?
}

// PSoC 4 DCOに出力
// parameter: frequency: 周波数の10倍値
void outDCO(uint16_t frequency) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PSOC4_DCO_CS, LOW);
  SPI.transfer(CMDM_FREQ_DECI);
  SPI.transfer(frequency >> 8);
  SPI.transfer(frequency & 0xff);
  digitalWrite(PSOC4_DCO_CS, HIGH);
  SPI.endTransaction();
/*
  SPI.end();
  myGLCD.clrScr();
  myGLCD.printNumI(pos, 0, 0);
  myGLCD.printNumI(frequency, 0, 10);
  myGLCD.printNumI(CMDM_FREQ_DECI, 0, 20);
  myGLCD.printNumI(frequency >> 8, 0, 30);
  myGLCD.printNumI(frequency & 0xff, 0, 40);
  myGLCD.update();
  SPI.begin();
*/
}

void outDCF(byte cutOff, byte q) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(AD8403_DCF_CS, LOW);
  SPI.transfer(0);
  SPI.transfer(q);
  SPI.transfer(1);
  SPI.transfer(cutOff);
  SPI.transfer(3);
  SPI.transfer(cutOff);
  digitalWrite(AD8403_DCF_CS, HIGH);
  SPI.endTransaction();
}

// DACに出力
// parameter: v: 出力値(0 .. 4095)
void outDAC(int16_t v) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MCP4922_LDAC, HIGH) ;
  digitalWrite(MCP4922_CS, LOW) ;
  SPI.transfer((v >> 8)| 0x30) ;
  SPI.transfer(v & 0xff) ;
  digitalWrite(MCP4922_CS, HIGH) ;
  digitalWrite(MCP4922_LDAC, LOW) ;
  SPI.endTransaction();
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

//-------------------------------------------------
// スイッチのトグル動作
// return: true 変化あり false:変化なし
//
// Todo: Bounce::fell()が使えないか？
//
bool toggleSW(Bounce& debouncer, bool& flag) {
  bool tmp;
  
  debouncer.update();
  tmp = debouncer.read();
  if (tmp == LOW) {
    flag = flag ? false : true;
    do { 
      debouncer.update();
    } while(debouncer.read() == LOW);
    return true;
  }
  return false;
}
