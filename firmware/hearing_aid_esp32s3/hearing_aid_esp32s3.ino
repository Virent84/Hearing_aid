/*
  Smart Hearing Aid - ESP32-S3 4-microphone capture firmware
  ----------------------------------------------------------
  Captures 4x INMP441 (16 kHz, 4 channels, time-aligned) and streams them to the
  Raspberry Pi 5 over the ESP32-S3 NATIVE USB port (the connector marked "USB").

  WIRING (matches docs/WIRING.md)
    GPIO4   SCK net  -> SCK of all 4 mics   (I2S1 slave BCLK input)
    GPIO15  jumper to GPIO4                 (I2S0 master BCLK output)
    GPIO5   WS  net  -> WS  of all 4 mics   (I2S1 slave WS input)
    GPIO16  jumper to GPIO5                 (I2S0 master WS output)
    GPIO6   SD of M1 + M2  (data line A, I2S0)   M1: L/R=GND (left)  M2: L/R=3V3 (right)
    GPIO7   SD of M3 + M4  (data line B, I2S1)   M3: L/R=GND (left)  M4: L/R=3V3 (right)
    3V3 -> VDD of all mics (+ L/R of M2, M4);  GND -> GND of all mics (+ L/R of M1, M3)

  HOW IT WORKS
    I2S0 is the clock MASTER: it generates BCLK/WS on GPIO15/16 and reads GPIO6.
    The jumpers carry those clocks to the mics (GPIO4/5 net) and back into I2S1,
    which runs as a SLAVE on the very same clocks and reads GPIO7.
    Both ports therefore sample on identical clock edges (no drift). I2S1 is enabled
    first so it never misses the first WS edge; any constant start offset between the
    ports (0..1 sample) is measured by tools/esp32_check.py and corrected on the Pi.

  OUTPUT PACKET (little-endian), 128 frames = 8 ms, 125 packets/s, ~128 kB/s
    u16 magic   0xA55A  (bytes 5A A5)
    u16 seq     increments every packet (wraps)  -> Pi detects lost packets
    u16 nframes 128
    u16 flags   bit0 = I2S DMA overflow happened since last packet
                bit1 = USB TX buffer was full -> previous packet(s) dropped
    i16 pcm[nframes][4]   channel order M1, M2, M3, M4  (16-bit, from the mic's 24-bit data)
    u16 crc16   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over seq..end of pcm

  ARDUINO IDE SETTINGS (Arduino-ESP32 core 3.x, i.e. ESP-IDF 5)
    Board: "ESP32S3 Dev Module"
    USB CDC On Boot: "Enabled"      <-- makes `Serial` the native USB port
    USB Mode: "Hardware CDC and JTAG" (default)  - appears on the Pi as 303a:1001 /dev/ttyACM0
    Upload with the cable in either port; for running, use the native "USB" port to the Pi.
    Do NOT open the Arduino Serial Monitor while the Pi is reading: the stream is binary.
*/

#include <Arduino.h>
#include "driver/i2s_std.h"

// ---------------- pins ----------------
static const gpio_num_t PIN_CLK_NET_BCLK = GPIO_NUM_4;   // slave BCLK in  (mics' SCK net)
static const gpio_num_t PIN_CLK_NET_WS   = GPIO_NUM_5;   // slave WS in    (mics' WS net)
static const gpio_num_t PIN_MASTER_BCLK  = GPIO_NUM_15;  // master BCLK out (jumper to GPIO4)
static const gpio_num_t PIN_MASTER_WS    = GPIO_NUM_16;  // master WS out   (jumper to GPIO5)
static const gpio_num_t PIN_SD_A         = GPIO_NUM_6;   // M1 (L) + M2 (R)
static const gpio_num_t PIN_SD_B         = GPIO_NUM_7;   // M3 (L) + M4 (R)

// ---------------- settings ----------------
static const uint32_t FS      = 16000;
static const int      FRAMES  = 128;      // frames per packet (8 ms)
static const int      CHANNELS = 4;
static const uint16_t MAGIC   = 0xA55A;

static i2s_chan_handle_t rxA = NULL;      // I2S0 master  (M1, M2)
static i2s_chan_handle_t rxB = NULL;      // I2S1 slave   (M3, M4)

static int32_t bufA[FRAMES * 2];
static int32_t bufB[FRAMES * 2];

static const int HDR_BYTES = 8;
static const int PKT_BYTES = HDR_BYTES + FRAMES * CHANNELS * 2 + 2;
static uint8_t  pkt[PKT_BYTES];

static volatile bool ovfA = false, ovfB = false;
static uint16_t seq = 0;
static bool txDropped = false;

static bool IRAM_ATTR onOvfA(i2s_chan_handle_t, i2s_event_data_t *, void *) { ovfA = true; return false; }
static bool IRAM_ATTR onOvfB(i2s_chan_handle_t, i2s_event_data_t *, void *) { ovfB = true; return false; }

static uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

static bool setupPort(i2s_port_t port, i2s_role_t role, gpio_num_t bclk, gpio_num_t ws,
                      gpio_num_t din, i2s_chan_handle_t *h, bool (*ovf)(i2s_chan_handle_t, i2s_event_data_t *, void *)) {
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(port, role);
  cc.dma_desc_num  = 8;
  cc.dma_frame_num = FRAMES;
  if (i2s_new_channel(&cc, NULL, h) != ESP_OK) return false;

  i2s_std_config_t sc = {};
  sc.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(FS);
  sc.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  sc.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  sc.gpio_cfg.bclk = bclk;
  sc.gpio_cfg.ws   = ws;
  sc.gpio_cfg.dout = I2S_GPIO_UNUSED;
  sc.gpio_cfg.din  = din;
  sc.gpio_cfg.invert_flags.mclk_inv = false;
  sc.gpio_cfg.invert_flags.bclk_inv = false;
  sc.gpio_cfg.invert_flags.ws_inv   = false;
  if (i2s_channel_init_std_mode(*h, &sc) != ESP_OK) return false;

  i2s_event_callbacks_t cb = {};
  cb.on_recv_q_ovf = ovf;
  i2s_channel_register_event_callback(*h, &cb, NULL);
  return true;
}

static void fatal(const char *msg) {
  // Text only appears if something is wrong; the Pi parser ignores non-packet bytes.
  for (;;) { Serial.print("ERR "); Serial.println(msg); delay(1000); }
}

void setup() {
  Serial.setTxBufferSize(16384);
  Serial.begin(115200);                 // baud is ignored on native USB

  // Slave first (it listens on GPIO4/5), then master (it drives GPIO15/16).
  if (!setupPort(I2S_NUM_1, I2S_ROLE_SLAVE,  PIN_CLK_NET_BCLK, PIN_CLK_NET_WS, PIN_SD_B, &rxB, onOvfB)) fatal("I2S1 slave init");
  if (!setupPort(I2S_NUM_0, I2S_ROLE_MASTER, PIN_MASTER_BCLK,  PIN_MASTER_WS,  PIN_SD_A, &rxA, onOvfA)) fatal("I2S0 master init");

  if (i2s_channel_enable(rxB) != ESP_OK) fatal("I2S1 enable");
  if (i2s_channel_enable(rxA) != ESP_OK) fatal("I2S0 enable");   // starts the clocks

  pkt[0] = MAGIC & 0xFF; pkt[1] = MAGIC >> 8;
}

void loop() {
  size_t gotA = 0, gotB = 0;
  const size_t want = sizeof(bufA);
  esp_err_t ea = i2s_channel_read(rxA, bufA, want, &gotA, 200);
  esp_err_t eb = i2s_channel_read(rxB, bufB, want, &gotB, 200);
  if (ea != ESP_OK || eb != ESP_OK || gotA != want || gotB != want) return;   // lost frame -> seq gap seen by Pi

  // header
  uint16_t flags = (ovfA || ovfB ? 1 : 0) | (txDropped ? 2 : 0);
  ovfA = ovfB = false; txDropped = false;
  pkt[2] = seq & 0xFF;            pkt[3] = seq >> 8;
  pkt[4] = FRAMES & 0xFF;         pkt[5] = FRAMES >> 8;
  pkt[6] = flags & 0xFF;          pkt[7] = flags >> 8;

  // payload: INMP441 24-bit data sits left-aligned in each 32-bit slot -> keep the top 16 bits
  int16_t *o = (int16_t *)(pkt + HDR_BYTES);
  for (int i = 0; i < FRAMES; i++) {
    o[4 * i + 0] = (int16_t)(bufA[2 * i]     >> 16);   // M1  (GPIO6, left,  L/R=GND)
    o[4 * i + 1] = (int16_t)(bufA[2 * i + 1] >> 16);   // M2  (GPIO6, right, L/R=3V3)
    o[4 * i + 2] = (int16_t)(bufB[2 * i]     >> 16);   // M3  (GPIO7, left,  L/R=GND)
    o[4 * i + 3] = (int16_t)(bufB[2 * i + 1] >> 16);   // M4  (GPIO7, right, L/R=3V3)
  }
  uint16_t crc = crc16(pkt + 2, PKT_BYTES - 4);
  pkt[PKT_BYTES - 2] = crc & 0xFF;
  pkt[PKT_BYTES - 1] = crc >> 8;
  seq++;

  // Never block the audio loop: if the host is not draining USB, drop the whole packet.
  if (Serial.availableForWrite() >= PKT_BYTES) Serial.write(pkt, PKT_BYTES);
  else txDropped = true;
}
