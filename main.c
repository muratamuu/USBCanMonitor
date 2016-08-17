#pragma config PLLEN = ON
#pragma config FCMEN = ON

#include <xc.h>
#include "usb_cdc.h"
#include <string.h>

// SPIコマンド
#define SPI_RESET       (0xC0) // リセット
#define SPI_REG_READ    (0x03) // レジスタ読み込み
#define SPI_REG_WRITE   (0x02) // レジスタ書き込み
#define SPI_BIT_MOD     (0x05) // レジスタビット変更
#define SPI_RXB0_READ   (0x90) // 受信バッファ0読み込み
#define SPI_RXB1_READ   (0x94) // 受信バッファ1読み込み
#define SPI_STAT_READ   (0xA0) // 状態読み込み
#define SPI_RXSTAT_READ (0xB0) // RX状態読み込み

// チップセレクト
#define SPI_CS0 LATCbits.LATC6
#define SPI_CS1 LATCbits.LATC4
#define SPI_CS2 LATCbits.LATC3
#define SPI_CS3 LATCbits.LATC5

// MCP2515レジスタ
#define BFPCTRL  (0x0C) // BIT_MOD可能
#define CANSTAT  (0x0E)
#define CANCTRL  (0x0F)
#define CNF3     (0x28) // BIT_MOD可能
#define CNF2     (0x29) // BIT_MOD可能
#define CNF1     (0x2A) // BIT_MOD可能
#define CANINTE  (0x2B) // BIT_MOD可能
#define CANINTF  (0x2C) // BIT_MOD可能
#define EFLG     (0x2D) // BIT_MOD可能
#define RXB0CTRL (0x60) // BIT_MOD可能
#define RXB1CTRL (0x70) // BIT_MOD可能

// MCP2515ビットタイミング定義(CNF1,CNF2,CNF3) 24MHz用
// 全定義SJW=4, SampleTime=1回
#define BITRATE_10K  0xFB, 0xAD, 0x06 // BRP=60, PRSEG=6, PS1=6, PS2=7, SamplePt=65%
#define BITRATE_20K  0xDD, 0xAD, 0x06 // BRP=30, PRSEG=6, PS1=6, PS2=7, SamplePt=65%
#define BITRATE_50K  0xCB, 0xAD, 0x06 // BRP=12, PRSEG=6, PS1=6, PS2=7, SamplePt=65%
#define BITRATE_100K 0xC5, 0xAD, 0x06 // BRP=6,  PRSEG=6, PS1=6, PS2=7, SamplePt=65%
#define BITRATE_125K 0xC5, 0xA4, 0x04 // BRP=6,  PRSEG=5, PS1=5, PS2=5, SamplePt=68.75%
#define BITRATE_200K 0xC2, 0xAD, 0x06 // BRP=3,  PRSEG=6, PS1=6, PS2=7, SamplePt=65%
#define BITRATE_250K 0xC2, 0xA4, 0x04 // BRP=3,  PRSEG=5, PS1=5, PS2=5, SamplePt=68.75%
#define BITRATE_500K 0xC1, 0x9A, 0x03 // BRP=2,  PRSEG=3, PS1=4, PS2=4, SamplePt=66.67%
#define BITRATE_800K 0xC0, 0xA3, 0x04 // BRP=1,  PRSEG=4, PS1=5, PS2=5, SamplePt=66.67%
#define BITRATE_1M   0xC0, 0x9A, 0x03 // BRP=1,  PRSEG=4, PS1=3, PS2=4, SamplePt=66.67%

// システムクロック48MHz (__delay_msマクロ用)
#define _XTAL_FREQ 48000000

typedef unsigned char  BYTE;
typedef unsigned short WORD;

// タイマ
unsigned long time_msec;

// CANメッセージ構造体
#define MSG_STR_MAX (38) // 00h00m00s000-0-000-0-0000000000000000 (37+1)
                         // 00h00m00s000-N-0000.0000-E-00000.0000 (37+1)
struct canmsg_t
{
  char str[MSG_STR_MAX];
  BYTE str_idx;
  BYTE str_sz;
};

// CANメッセージバッファ
#define MSG_MAX (6)
struct canmsg_t msgbuffer[MSG_MAX];

// 文字列変換テーブル
const char* to_ascii = "0123456789ABCDEF";

// 処理進行状態変数
BYTE recv_count = 0;
BYTE recv_idx = 0;
BYTE send_idx = 0;
BYTE is_sending = 0;

// GPS受信バッファ
#define GPS_LINE_MAX (64)
BYTE gpsline[GPS_LINE_MAX];
BYTE gpslinepos = 0;

// USB受信バッファ
#define LINE_MAX (4)
BYTE line[LINE_MAX];
BYTE linepos = 0;

// アプリケーションモード
#define MODE_MCP2515 (0x0F) // MCP2515モード
#define MODE_CONFIG  (0x01) // MCP2515コンフィグモード
#define MODE_NORMAL  (0x02) // MCP2515通常受信モード(ACK送信有)
#define MODE_LISTEN  (0x04) // MCP2515リスンオンリモード
#define MODE_DATA    (0xF0) // 受信データ送信モード
#define MODE_ASCII   (0x10) // 受信データを文字列化送信
#define MODE_BINARY  (0x20) // 受信データをバイナリ送信
BYTE mode = MODE_CONFIG | MODE_ASCII;

void spi_enable(BYTE ch);
void spi_disable(BYTE ch);
BYTE spi_transmit(BYTE c);
void mcp2515_reset(BYTE ch);
void mcp2515_init(BYTE ch);
void mcp2515_bitrate(BYTE ch, BYTE cnf1, BYTE cnf2, BYTE cnf3);
void mcp2515_bitrate_allch(BYTE cnf1, BYTE cnf2, BYTE cnf3);
void mcp2515_writereg(BYTE ch, BYTE addr, BYTE data);
BYTE mcp2515_readreg(BYTE ch, BYTE addr);
void mcp2515_modreg(BYTE ch, BYTE addr, BYTE mask, BYTE data);
BYTE mcp2515_recv(BYTE ch, struct canmsg_t* msg);
BYTE is_received(BYTE ch);
BYTE gps_recv();
BYTE gps_parse(struct canmsg_t* msg);
void parse_line(char* line);

void main(void)
{
  // アナログピンをデジタルに設定
  ANSEL  = 0x00;
  ANSELH = 0x00;

  // 受信割り込みピンを入力に設定
  TRISCbits.TRISC2 = 1;   // 14番ピン(INT-ch0)を入力に設定
  TRISCbits.TRISC0 = 1;   // 16番ピン(INT-ch1)を入力に設定
  TRISCbits.TRISC1 = 1;   // 15番ピン(INT-ch2)を入力に設定
  TRISAbits.TRISA4 = 1;   //  3番ピン(INT-ch3)を入力に設定

  // SPI設定
  SSPSTATbits.CKE   = 1;  // クロックがアクティブからアイドルで送信する
  SSPCON1bits.SSPM  = 1;  // SPIクロック 1:FOSC/16 48MHz/16=3MHz
  SSPCON1bits.SSPEN = 1;  // MSSP有効

  // SPIバッファクリア
  BYTE clear = SSPBUF;
  clear = 0;

  // SPIピン設定
  TRISCbits.TRISC6 = 0;   // 8番ピン(CS-ch0)を出力に設定
  TRISCbits.TRISC4 = 0;   // 6番ピン(CS-ch1)を出力に設定
  TRISCbits.TRISC3 = 0;   // 7番ピン(CS-ch2)を出力に設定
  TRISCbits.TRISC5 = 0;   // 5番ピン(CS-ch3)を出力に設定
  TRISCbits.TRISC7 = 0;   // 9番ピン(SDO)を出力に設定
  TRISBbits.TRISB4 = 1;   // 13番ピン(SDI)を入力に設定
  TRISBbits.TRISB6 = 0;   // 11番ピン(SCK)を出力に設定

  // SPI_CS OFF
  SPI_CS0 = 1;
  SPI_CS1 = 1;
  SPI_CS2 = 1;
  SPI_CS3 = 1;

  // MCP2515リセット
  mcp2515_reset(0);
  mcp2515_reset(1);
  mcp2515_reset(2);
  mcp2515_reset(3);

  // MCP2515初期化
  mcp2515_init(0);
  mcp2515_init(1);
  mcp2515_init(2);
  mcp2515_init(3);

  OSCCON = 0x30;

  // USB初期化
  usb_init();

  // タイマー初期化(10msに1回割り込みを発生させる)
  TMR0 = -7500;
  T0CON = 0x83;          // 1 / 16 prescalse
  INTCONbits.GIE    = 1; // 割り込み許可
  INTCONbits.TMR0IE = 1; // タイマ0割り込み許可

  // UART(GPSシリアル受信)設定
  TRISBbits.TRISB5 = 1;  // 12番ピン(RX/DT)を入力に設定する
  RCSTA   = 0x90; // シリアル有効,連続受信,8bit,パリティなし
  TXSTA   = 0x04; // BRGH:1
  BAUDCON = 0x08; // BRG16:1
  SPBRGH  = 1249 / 256; // 9600bps(1249:上位)
  SPBRG   = 1249 % 256; // 9600bps(1249:下位)

  ////////////////////////
  /// メインループ開始 ///
  ////////////////////////

  // 受信チャネル番号(0〜3)
  BYTE ch = 0;

  while (1) {
    usb_process();

    // 受信処理 (1Loopで1chずつスライドしてチェックする)
    ch = (ch + 1) % 4;

    if (is_received(ch)) { // 受信/エラーチェック
      // 割り込み要因チェック
      BYTE reason = mcp2515_readreg(ch, CANINTF);

      if (reason & 0x20) { // エラー割り込み
        mcp2515_modreg(ch, CANINTF, 0x20, 0x00); // エラーフラグを落とす
        if (mcp2515_readreg(ch, EFLG) & 0xC0)
          // MCP2515受信バッファオーバー
          mcp2515_modreg(ch, BFPCTRL, 0x20, 0x20); // MCP2515の10番ピン点灯
      }

      if (reason & 0x03) { // 受信割り込み
        if (recv_count < MSG_MAX) {
          if (mcp2515_recv(ch, &msgbuffer[recv_idx]) > 0) {
            recv_count++;
            recv_idx = (recv_idx + 1) % MSG_MAX;
          }
        } else {
          // PIC内受信バッファオーバー
          mcp2515_modreg(ch, BFPCTRL, 0x20, 0x20); // MCP2515の10番ピン点灯
        }
      }
    }
    else if (PIR1bits.RCIF) { // GPSシリアル受信チェック
      if (gps_recv()) { // 解析対象受信データあり
        if (recv_count < MSG_MAX) {
          if (gps_parse(&msgbuffer[recv_idx]) > 0) {
            recv_count++;
            recv_idx = (recv_idx + 1) % MSG_MAX;
          }
        } else {
          // PIC内受信バッファオーバー(全部点灯)
          mcp2515_modreg(0, BFPCTRL, 0x20, 0x20);
          mcp2515_modreg(1, BFPCTRL, 0x20, 0x20);
          mcp2515_modreg(2, BFPCTRL, 0x20, 0x20);
          mcp2515_modreg(3, BFPCTRL, 0x20, 0x20);
        }
      }
    }

    // USB送信処理
    while (usb_ep1_ready() && recv_count > 0) {
      is_sending = 1;
      struct canmsg_t* msg = &msgbuffer[send_idx];
      usb_putch(msg->str[msg->str_idx++]);
      if (msg->str_idx == msg->str_sz) {
        send_idx = (send_idx + 1) % MSG_MAX;
        recv_count--;
        is_sending = 0;
      }
    }

    // USB受信処理(メッセージ送信中は処理しない)
    while (usb_chReceived() && !is_sending) {
      BYTE c = usb_getch();
      if (c == '\r') {
        line[linepos] = 0;
        parse_line(line);
        linepos = 0;
      } else if (c != '\n') {
        line[linepos] = c;
        if (linepos < LINE_MAX - 1)
          linepos++;
      }
    }

    // リセット判定
    if (!PORTAbits.RA3) { // 4番ピンGND
      UCON = 0;
      _delay(1000);
      RESET();
    }
  }

  return;
}

void spi_enable(BYTE ch)
{
  if (ch == 0)
    SPI_CS0 = 0;
  else if (ch == 1)
    SPI_CS1 = 0;
  else if (ch == 2)
    SPI_CS2 = 0;
  else if (ch == 3)
    SPI_CS3 = 0;
}

void spi_disable(BYTE ch)
{
  if (ch == 0)
    SPI_CS0 = 1;
  else if (ch == 1)
    SPI_CS1 = 1;
  else if (ch == 2)
    SPI_CS2 = 1;
  else if (ch == 3)
    SPI_CS3 = 1;
}

BYTE spi_transmit(BYTE c)
{
  SSPBUF = c;
  while (!SSPSTATbits.BF);
  return SSPBUF;
}

void mcp2515_reset(BYTE ch)
{
  BYTE data = 0;
  while (++data);
  spi_enable(ch);
  spi_transmit(SPI_RESET);
  spi_disable(ch);
  while (++data);
}

void mcp2515_init(BYTE ch)
{
  // コンフィグモードに移行
  // クロック出力 Fosc/2 24MHz/2=12MHz
  mcp2515_writereg(ch, CANCTRL, 0x85);
  // RXB0,RXB1でフィルタマスクを使用しない
  mcp2515_modreg(ch, RXB0CTRL, 0x60, 0x60);
  mcp2515_modreg(ch, RXB1CTRL, 0x60, 0x60);
  // エラー及びRXB0,RXB1で受信割り込み設定
  mcp2515_writereg(ch, CANINTE, 0x23);
  // CANビットレート設定(default)
  mcp2515_bitrate(ch, BITRATE_500K);
  // RXB0割り込みピン(11番)を有効化(受信時にLEDを点滅させる)
  // RXB1割り込みピン(10番)をデジタル出力に設定
  mcp2515_modreg(ch, BFPCTRL, 0x0F, 0x0D);
}

void mcp2515_bitrate(BYTE ch, BYTE cnf1, BYTE cnf2, BYTE cnf3)
{
  mcp2515_writereg(ch, CNF1, cnf1);
  mcp2515_writereg(ch, CNF2, cnf2);
  mcp2515_writereg(ch, CNF3, cnf3);
}

void mcp2515_bitrate_allch(BYTE cnf1, BYTE cnf2, BYTE cnf3)
{
  for (BYTE ch = 0; ch < 4; ch++)
    mcp2515_bitrate(ch, cnf1, cnf2, cnf3);
}

void mcp2515_writereg(BYTE ch, BYTE addr, BYTE data)
{
  spi_enable(ch);
  spi_transmit(SPI_REG_WRITE);
  spi_transmit(addr);
  spi_transmit(data);
  spi_disable(ch);
}

BYTE mcp2515_readreg(BYTE ch, BYTE addr)
{
  spi_enable(ch);
  spi_transmit(SPI_REG_READ);
  spi_transmit(addr);
  BYTE data = spi_transmit(0xFF);
  spi_disable(ch);
  return data;
}

void mcp2515_modreg(BYTE ch, BYTE addr, BYTE mask, BYTE data)
{
  spi_enable(ch);
  spi_transmit(SPI_BIT_MOD);
  spi_transmit(addr);
  spi_transmit(mask);
  spi_transmit(data);
  spi_disable(ch);
}

BYTE mcp2515_recv(BYTE ch, struct canmsg_t* msg)
{
  WORD id;
  BYTE dlc;
  BYTE data[8];
  unsigned long t = time_msec;

  spi_enable(ch);
  spi_transmit(SPI_RXB0_READ); // RXB0受信

  id = (WORD)spi_transmit(0xFF) << 3;
  id |= (WORD)spi_transmit(0xFF) >> 5;
  spi_transmit(0xFF);
  spi_transmit(0xFF);
  dlc = spi_transmit(0xFF) & 0x0F;
  for (int i = 0; i < 8; i++)
    data[i] = (i < dlc) ? spi_transmit(0xFF) : 0;

  spi_disable(ch);

  if (mode & MODE_ASCII) {
    // 文字列で送信
    WORD msec = t % 1000;
    BYTE sec  = (t / 1000) % 60;
    BYTE min  = (t / 60000) % 60;
    BYTE hour = (t / 3600000) % 24;

    msg->str[0]  = to_ascii[hour / 10];
    msg->str[1]  = to_ascii[hour % 10];
    msg->str[2]  = 'h';
    msg->str[3]  = to_ascii[min / 10];
    msg->str[4]  = to_ascii[min % 10];
    msg->str[5]  = 'm';
    msg->str[6]  = to_ascii[sec / 10];
    msg->str[7]  = to_ascii[sec % 10];
    msg->str[8]  = 's';
    msg->str[9]  = to_ascii[msec / 100];
    msg->str[10] = to_ascii[(msec % 100) / 10];
    msg->str[11] = to_ascii[msec % 10];
    msg->str[12] = '-';

    msg->str[13] = to_ascii[ch+1]; // 0-3 -> 1-4
    msg->str[14] = '-';

    msg->str[15] = to_ascii[(id >> 8) & 0x0F];
    msg->str[16] = to_ascii[(id >> 4) & 0x0F];
    msg->str[17] = to_ascii[(id >> 0) & 0x0F];
    msg->str[18] = '-';
    msg->str[19] = to_ascii[dlc];
    msg->str[20] = '-';
    for (int i = 0; i < dlc; i++) {
      msg->str[21+(i*2)]   = to_ascii[(data[i] >> 4) & 0x0F];
      msg->str[21+(i*2)+1] = to_ascii[(data[i] >> 0) & 0x0F];
    }
    msg->str[21+(dlc*2)] = '\r';
    msg->str_idx = 0;
    msg->str_sz = 21 + (dlc * 2) + 1;
  }
  else {
    // バイナリで送信 (時刻:4Byte + チャネル:1Byte + id:2Byte + dlc:1Byte + data:8Byte)
    msg->str[0] = (t >> 24) & 0xFF;
    msg->str[1] = (t >> 16) & 0xFF;
    msg->str[2] = (t >> 8)  & 0xFF;
    msg->str[3] = t & 0xFF;
    msg->str[4] = ch+1; // 0-3 -> 1-4
    msg->str[5] = (id >> 8) & 0xFF;
    msg->str[6] = id & 0xFF;
    msg->str[7] = dlc;
    for (int i = 0; i < 8; i++)
      msg->str[8+i] = data[i];
    msg->str_idx = 0;
    msg->str_sz = 16; // 固定長
  }

  return 1;
}

BYTE is_received(BYTE ch)
{
  // 受信割り込みピンを確認 GNDならMCP2515エラーor受信割り込み
  if (ch == 0)
    return !PORTCbits.RC2; // 14番ピン
#if 0 // 4ch対応時に有効化する
  else if (ch == 1)
    return !PORTCbits.RC0; // 16番ピン
  else if (ch == 2)
    return !PORTCbits.RC1; // 15番ピン
  else if (ch == 3)
    return !PORTAbits.RA4; // 3番ピン
#endif
  else
    return 0;
}

BYTE gps_recv()
{
  // シリアル受信データ読み込み
  BYTE c = RCREG;
  if (c == '$') // データ開始位置
    gpslinepos = 0;

  // バッファに追加
  gpsline[gpslinepos] = c;

  // データ終端チェック
  if (c == '\n' || gpslinepos == GPS_LINE_MAX-1) {
    gpsline[gpslinepos] = 0; // null止め
    gpslinepos = 0;
    // GPSデータチェック($GPGLLで始まり、$GPGLL,,でないこと)
    if (!strncmp(gpsline, "$GPGLL", 6) && gpsline[7] != ',')
      return 1; // データあり
  }
  gpslinepos++;
  return 0; // データなし
}

BYTE gps_parse(struct canmsg_t* msg)
{
  unsigned long t = time_msec;

  if (mode & MODE_ASCII) {
    // 文字列で送信
    WORD msec = t % 1000;
    BYTE sec  = (t / 1000) % 60;
    BYTE min  = (t / 60000) % 60;
    BYTE hour = (t / 3600000) % 24;

    msg->str[0]  = to_ascii[hour / 10];
    msg->str[1]  = to_ascii[hour % 10];
    msg->str[2]  = 'h';
    msg->str[3]  = to_ascii[min / 10];
    msg->str[4]  = to_ascii[min % 10];
    msg->str[5]  = 'm';
    msg->str[6]  = to_ascii[sec / 10];
    msg->str[7]  = to_ascii[sec % 10];
    msg->str[8]  = 's';
    msg->str[9]  = to_ascii[msec / 100];
    msg->str[10] = to_ascii[(msec % 100) / 10];
    msg->str[11] = to_ascii[msec % 10];
    msg->str[12] = '-';

    // gpslineのデータ例
    // $GPGLL,3421.7686,N,13222.3345,E,073132,A,A*49

    // 緯度 (N-3723.2475)
    msg->str[13] = gpsline[17]; // N or S
    msg->str[14] = '-';
    for (int i = 0; i < 9; i++) // 9桁
      msg->str[15+i] = gpsline[7+i];
    msg->str[24] = '-';

    // 経度 (E-12158.3416)
    msg->str[25] = gpsline[30]; // E or W
    msg->str[26] = '-';
    for (int i = 0; i < 10; i++) // 10桁
      msg->str[27+i] = gpsline[19+i];
    msg->str[37] = '\r';
    msg->str_idx = 0;
    msg->str_sz = 38; // 固定長
    return 1;
  }
  else {
    // バイナリで送信モードは非サポート
    return 0;
  }
}

void parse_line(char* line)
{
  if (line[0] == 'O' && mode & MODE_CONFIG) {
    // 通常受信モード(ACKを返す)
    mcp2515_modreg(0, CANCTRL, 0xE0, 0x00);
    mcp2515_modreg(1, CANCTRL, 0xE0, 0x00);
    mcp2515_modreg(2, CANCTRL, 0xE0, 0x00);
    mcp2515_modreg(3, CANCTRL, 0xE0, 0x00);
    mode = (mode & MODE_DATA) | MODE_NORMAL;
    LATBbits.LATB5 = 1; // Main-LED点灯
    time_msec = 0;
  }
  else if (line[0] == 'L' && mode & MODE_CONFIG) {
    // リスンオンリーモード
    mcp2515_modreg(0, CANCTRL, 0xE0, 0x60);
    mcp2515_modreg(1, CANCTRL, 0xE0, 0x60);
    mcp2515_modreg(2, CANCTRL, 0xE0, 0x60);
    mcp2515_modreg(3, CANCTRL, 0xE0, 0x60);
    mode = (mode & MODE_DATA) | MODE_LISTEN;
    LATBbits.LATB5 = 1; // Main-LED点灯
    time_msec = 0;
  }
  else if (line[0] == 'A' && mode & MODE_CONFIG) {
    mode = (mode & MODE_MCP2515) | MODE_ASCII;
  }
  else if (line[0] == 'B' && mode & MODE_CONFIG) {
    mode = (mode & MODE_MCP2515) | MODE_BINARY;
  }
  else if (line[0] == 'C') {
    // 受信停止(コンフィグモードに戻す)
    mcp2515_modreg(0, CANCTRL, 0xE0, 0x80);
    mcp2515_modreg(1, CANCTRL, 0xE0, 0x80);
    mcp2515_modreg(2, CANCTRL, 0xE0, 0x80);
    mcp2515_modreg(3, CANCTRL, 0xE0, 0x80);
    mode = (mode & MODE_DATA) | MODE_CONFIG;
    recv_count = 0;
    recv_idx = 0;
    send_idx = 0;
    is_sending = 0;
    LATBbits.LATB5 = 0; // Main-LED消灯
  }
  else if (line[0] == 'R') {
    // リセット
    UCON = 0;
    _delay(1000);
    RESET();
  }
  else if (line[0] == 'S') {
    // ビットレート設定
    if (line[1] == '0') mcp2515_bitrate_allch(BITRATE_10K);
    if (line[1] == '1') mcp2515_bitrate_allch(BITRATE_20K);
    if (line[1] == '2') mcp2515_bitrate_allch(BITRATE_50K);
    if (line[1] == '3') mcp2515_bitrate_allch(BITRATE_100K);
    if (line[1] == '4') mcp2515_bitrate_allch(BITRATE_125K);
    if (line[1] == '5') mcp2515_bitrate_allch(BITRATE_250K);
    if (line[1] == '6') mcp2515_bitrate_allch(BITRATE_500K);
    if (line[1] == '7') mcp2515_bitrate_allch(BITRATE_800K);
    if (line[1] == '8') mcp2515_bitrate_allch(BITRATE_1M);
    if (line[1] == '9') mcp2515_bitrate_allch(BITRATE_200K);
  }
  usb_putch('\r');
}

void interrupt isr()
{
  if (INTCONbits.TMR0IF) {
    // 10msec毎に割り込み発生
    TMR0 = -7500;
    INTCONbits.TMR0IF = 0;
    time_msec += 10;
  }
}
