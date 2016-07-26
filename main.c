#pragma config PLLEN = ON
#pragma config FCMEN = ON

#include <xc.h>
#include "usb_cdc.h"

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
#define SPI_CS LATCbits.LATC6

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

// システムクロック48MHz (__delay_msマクロ用)
#define _XTAL_FREQ 48000000

typedef unsigned char  BYTE;
typedef unsigned short WORD;

#define MSG_STR_MAX (23) // 11*2+1
struct canmsg_t
{
  WORD id;
  BYTE dlc;
  BYTE data[8];
  char str[MSG_STR_MAX];
  int str_idx;
};

#define MSG_MAX (8)
struct canmsg_t msgbuffer[MSG_MAX];
int recv_count = 0;
int recv_idx = 0;
int send_idx = 0;

const char* to_ascii = "0123456789ABCDEF";

BYTE spi_transmit(BYTE c);
void mcp2515_reset();
void mcp2515_init();
void mcp2515_writereg(BYTE addr, BYTE data);
BYTE mcp2515_readreg(BYTE addr);
void mcp2515_modreg(BYTE addr, BYTE mask, BYTE data);
int mcp2515_recv(struct canmsg_t* msg);

void main(void)
{
  // アナログピンをデジタルに設定
  ANSEL  = 0x00;
  ANSELH = 0x00;

  // 12番ピン(LED)を出力に設定
  TRISBbits.TRISB5 = 0;

  // SPI設定
  SSPSTATbits.CKE   = 1;  // クロックがアクティブからアイドルで送信する
  SSPCON1bits.SSPM  = 1;  // SPIクロック 1:FOSC/16 48MHz/16=3MHz
  SSPCON1bits.SSPEN = 1;  // MSSP有効

  // SPIバッファクリア
  BYTE clear = SSPBUF;
  clear = 0;

  // SPIピン設定
  TRISCbits.TRISC6 = 0;   // 8番ピン(CS)を出力に設定
  TRISCbits.TRISC7 = 0;   // 9番ピン(SDO)を出力に設定
  TRISBbits.TRISB4 = 1;   // 13番ピン(SDI)を入力に設定
  TRISBbits.TRISB6 = 0;   // 11番ピン(SCK)を出力に設定

   // SPI_CS OFF
  SPI_CS = 1;

  // MCP2515リセット
  mcp2515_reset();

  // MCP2515初期化
  mcp2515_init();

  OSCCON = 0x30;

  // USB初期化
  usb_init();

  // メインループ開始
  LATBbits.LATB5 = 1;

  while (1) {
    usb_process();

    // 受信処理
    if (!PORTCbits.RC2) { // 14番ピンGND (MCP2515エラーor受信割り込み)
      // 割り込み要因チェック
      BYTE reason = mcp2515_readreg(CANINTF);

      if (reason & 0x20) { // エラー割り込み
        mcp2515_modreg(CANINTF, 0x20, 0x00); // エラーフラグを落とす
        if (mcp2515_readreg(EFLG) & 0xC0)
          // MCP2515受信バッファオーバー
          mcp2515_modreg(BFPCTRL, 0x14, 0x14);
      }

      if (reason & 0x03) { // 受信割り込み
        if (recv_count < MSG_MAX) {
          if (mcp2515_recv(&msgbuffer[recv_idx]) > 0) {
            recv_count++;
            recv_idx = (recv_idx + 1) % MSG_MAX;
          }
        } else {
          // PIC内受信バッファオーバー
          mcp2515_modreg(BFPCTRL, 0x14, 0x14);
        }
      }

    } // 受信処理END

    // USB送信処理
    if (usb_ep1_ready() && recv_count > 0) {
      struct canmsg_t* msg = &msgbuffer[send_idx];
      usb_putch(msg->str[msg->str_idx++]);
      if (msg->str_idx == MSG_STR_MAX) {
        send_idx = (send_idx + 1) % MSG_MAX;
        recv_count--;
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

BYTE spi_transmit(BYTE c)
{
  SSPBUF = c;
  while (!SSPSTATbits.BF);
  return SSPBUF;
}

void mcp2515_reset()
{
  BYTE data = 0;
  while (++data);
  SPI_CS = 0;
  spi_transmit(SPI_RESET);
  SPI_CS = 1;
  while (++data);
}

void mcp2515_init()
{
  // コンフィグモードに移行
  // クロック出力 Fosc/2 24MHz/2=12MHz
  mcp2515_writereg(CANCTRL, 0x85);
  // RXB0,RXB1でフィルタマスクを使用しない
  mcp2515_modreg(RXB0CTRL, 0x60, 0x60);
  mcp2515_modreg(RXB1CTRL, 0x60, 0x60);
  // エラー及びRXB0,RXB1で受信割り込み設定
  mcp2515_writereg(CANINTE, 0x23);
  // CANビットレート設定
  mcp2515_writereg(CNF1, 0xC1);
  mcp2515_writereg(CNF2, 0x9A);
  mcp2515_writereg(CNF3, 0x03);
  // ノーマルモードに移行
  mcp2515_modreg(CANCTRL, 0xE0, 0x00);
}

void mcp2515_writereg(BYTE addr, BYTE data)
{
  SPI_CS = 0;
  spi_transmit(SPI_REG_WRITE);
  spi_transmit(addr);
  spi_transmit(data);
  SPI_CS = 1;
}

BYTE mcp2515_readreg(BYTE addr)
{
  SPI_CS = 0;
  spi_transmit(SPI_REG_READ);
  spi_transmit(addr);
  BYTE data = spi_transmit(0xFF);
  SPI_CS = 1;
  return data;
}

void mcp2515_modreg(BYTE addr, BYTE mask, BYTE data)
{
  SPI_CS = 0;
  spi_transmit(SPI_BIT_MOD);
  spi_transmit(addr);
  spi_transmit(mask);
  spi_transmit(data);
  SPI_CS = 1;
}

int mcp2515_recv(struct canmsg_t* msg)
{
  SPI_CS = 0;
  spi_transmit(SPI_RXB0_READ); // RXB0受信

  msg->id = (WORD)spi_transmit(0xFF) << 3;
  msg->id |= (WORD)spi_transmit(0xFF) >> 5;
  spi_transmit(0xFF);
  spi_transmit(0xFF);
  msg->dlc = spi_transmit(0xFF) & 0x0F;
  for (int i = 0; i < msg->dlc && i < 8; i++)
    msg->data[i] = spi_transmit(0xFF);

  SPI_CS = 1;

  // 文字列化
  msg->str[0] = to_ascii[(msg->id  >> 12) & 0x0F];
  msg->str[1] = to_ascii[(msg->id  >> 8)  & 0x0F];
  msg->str[2] = to_ascii[(msg->id  >> 4)  & 0x0F];
  msg->str[3] = to_ascii[(msg->id  >> 0)  & 0x0F];
  msg->str[4] = to_ascii[(msg->dlc >> 4)  & 0x0F];
  msg->str[5] = to_ascii[(msg->dlc >> 0)  & 0x0F];
  for (int i = 0; i < 8; i++) {
    msg->str[6+(i*2)]   = to_ascii[(msg->data[i] >> 4) & 0x0F];
    msg->str[6+(i*2)+1] = to_ascii[(msg->data[i] >> 0) & 0x0F];
  }
  msg->str[22] = '\r';
  //msg->str[23] = '\n';
  msg->str_idx = 0;

  return 1;
}
