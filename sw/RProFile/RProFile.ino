//***********************************************************************************
//* RProFile ProFile Emulator Software (RP2350 Edition)                            *
//* Dual-Core Architecture leveraging PIO and DMA                                   *
//* 2026-02-21                                                                      *
//***********************************************************************************

// ProFile bus protocol constants
#define PROFILE_ACK_PRESENCE  0x01 // drive -> host: "i'm here"
#define PROFILE_ACK_READ      0x02 // drive -> host: read command acknowledged (cmd + 2)
#define PROFILE_HOST_CONFIRM  0x55 // host -> drive: confirmation byte
#define PROFILE_ACK_DATA      0x06 // drive -> host: block data received OK
#define PROFILE_CMD_READ      0x00
#define PROFILE_CMD_WRITE     0x01
#define PROFILE_CMD_WRITE_VFY 0x02
#define PROFILE_CMD_WRITE_FSP 0x03

#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "profile_bus.pio.h"
#include "pico/mutex.h"
#include "Adafruit_TinyUSB.h"

// Hardware Pin Definitions for the ProFile Bus (Contiguous for PIO)
#define PROFILE_DATA_BASE 0  // D0 is GPIO 0, D1 is GPIO 1, ..., D7 is GPIO 7
#define PROFILE_STRB      9  // Strobe from Host
#define PROFILE_CMD       10 // Command from Host
#define PROFILE_BSY       11 // Busy from Drive
#define PROFILE_RW        12 // Read/Write
#define PROFILE_PRES      13 // Presence
#define PROFILE_PARITY    14 // Parity bit

// buffer for DMA
#define SECTOR_SIZE       532
#define STATUS_BYTES      4
#define PROFILE_FRAME_SIZE (STATUS_BYTES + SECTOR_SIZE) // 4 status bytes + 532 data bytes = 536

// rx: PIO pushes 8-bit data in a 32-bit FIFO word; use uint32_t and extract LSB after DMA
uint32_t dma_rx_buffer[PROFILE_FRAME_SIZE] __attribute__((aligned(4)));
// tx: PIO pulls 16-bit (8 data + parity in bit 8); use uint16_t
uint16_t dma_tx_buffer[PROFILE_FRAME_SIZE] __attribute__((aligned(4)));

// DMA channels
int dma_rx_chan;
int dma_tx_chan;
dma_channel_config rx_cfg;
dma_channel_config tx_cfg;

// handshake timeout (~100 ms at ~150 MHz)
#define HANDSHAKE_TIMEOUT 15000000UL

// SD Card Pin Definitions (SPI0, starting after ProFile bus GPIO 0-14)
#define SD_CS   15 // chip select
#define SD_SCK  18 // SPI0 SCK
#define SD_MOSI 19 // SPI0 TX
#define SD_MISO 16 // SPI0 RX

SdFat32 SDCard;
File32 disk;
const char* fileName = "profile.image";

// cross-core mutex for SD card access (used when PSRAM cache is disabled)
mutex_t sd_mutex;

// PIO Configuration
PIO profile_pio = pio0;
uint sm_read;
uint sm_write;

// ==============================================================================
// PSRAM Configuration
// ==============================================================================
#define USE_PSRAM_CACHE false // set to true to enable PSRAM write-through caching

#if USE_PSRAM_CACHE
// fixed-size ring buffer for dirty sector tracking (no heap allocation)
#define DIRTY_RING_SIZE 256
static uint32_t dirty_ring[DIRTY_RING_SIZE];
static volatile uint32_t dirty_head = 0;
static volatile uint32_t dirty_tail = 0;

mutex_t cache_mutex;
uint8_t* psram_cache = NULL; // initialized to physical PSRAM address (e.g., 0x11000000)

void markSectorDirty(uint32_t sector) {
    mutex_enter_blocking(&cache_mutex);
    uint32_t next = (dirty_head + 1) % DIRTY_RING_SIZE;
    if (next != dirty_tail) {
        dirty_ring[dirty_head] = sector;
        dirty_head = next;
    } else {
        Serial.println("WARNING: dirty ring full, sector flush dropped!");
    }
    mutex_exit(&cache_mutex);
}
#endif

// ==============================================================================
// CORE 0: USB / System Management (TinyUSB MSC)
// ==============================================================================
Adafruit_USBD_MSC usb_msc;

// Callback invoked when received READ10 command.
int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
  uint32_t block_count = bufsize / 512;
#if USE_PSRAM_CACHE
  memcpy(buffer, &psram_cache[lba * 512], bufsize);
#else
  mutex_enter_blocking(&sd_mutex);
  if(!SDCard.card()) { mutex_exit(&sd_mutex); return -1; }
  SDCard.card()->readSectors(lba, (uint8_t*) buffer, block_count);
  mutex_exit(&sd_mutex);
#endif
  return bufsize;
}

// Callback invoked when received WRITE10 command.
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  uint32_t block_count = bufsize / 512;
#if USE_PSRAM_CACHE
  memcpy(&psram_cache[lba * 512], buffer, bufsize);
  // mark these sectors as dirty so Core 0's background loop flushes them to SD later
  for(uint32_t i=0; i<block_count; i++) markSectorDirty(lba + i);
#else
  mutex_enter_blocking(&sd_mutex);
  if(!SDCard.card()) { mutex_exit(&sd_mutex); return -1; }
  SDCard.card()->writeSectors(lba, buffer, block_count);
  mutex_exit(&sd_mutex);
#endif
  return bufsize;
}

// Callback invoked when received Test Unit Ready command.
bool msc_ready_cb(void) {
  return disk.isOpen();
}

// Callback invoked when received a FLUSH command.
void msc_flush_cb(void) {
#if !USE_PSRAM_CACHE
  mutex_enter_blocking(&sd_mutex);
#endif
  if (disk.isOpen()) {
      disk.sync();
      SDCard.card()->syncDevice();
  }
#if !USE_PSRAM_CACHE
  mutex_exit(&sd_mutex);
#endif
}

void setup() {
  Serial.begin(115200);
  
  // Set up the USB Mass Storage Class
  usb_msc.setID("RProFile", "SD Card", "1.0");
  usb_msc.setCapacity(0, 512); // Will normally be updated once SD is mounted
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setReadyCallback(0, msc_ready_cb); // LUN 0 ready check
  usb_msc.setUnitReady(false);
  
  usb_msc.begin();

  mutex_init(&sd_mutex);
#if USE_PSRAM_CACHE
  mutex_init(&cache_mutex);
#endif
}

void loop() {
  // allow TinyUSB to process device tasks quietly in the background
  tud_task();

#if USE_PSRAM_CACHE
  // write-through background flusher
  // note: sector addressing here is in ProFile 532-byte sectors (not SD 512-byte blocks).
  // disk I/O is byte-addressed via File32, so seekSet(sector * 532) is correct.
  uint32_t sector_to_flush = 0xFFFFFFFF;
  bool has_dirty = false;

  mutex_enter_blocking(&cache_mutex);
  if (dirty_head != dirty_tail) {
      sector_to_flush = dirty_ring[dirty_tail];
      dirty_tail = (dirty_tail + 1) % DIRTY_RING_SIZE;
      has_dirty = true;
  }
  mutex_exit(&cache_mutex);

  if (has_dirty && disk.isOpen()) {
      disk.seekSet(sector_to_flush * SECTOR_SIZE);
      disk.write(&psram_cache[sector_to_flush * SECTOR_SIZE], SECTOR_SIZE);
      disk.sync();
  }
#endif

  delay(1);
}

// ==============================================================================
// CORE 1: Main Application & Storage Emulation
// ==============================================================================

// helper: calculate odd parity for 8-bit value
static inline uint8_t getOddParity(uint8_t val) {
  // odd parity: return 1 if even number of set bits
  return (__builtin_popcount(val) & 1) ? 0 : 1;
}

// bitmask covering D0-D7 (GPIO 0-7)
#define DATA_BUS_MASK 0x000000FF
// bitmask covering D0-D7 + parity (GPIO 0-8)
#define DATA_PARITY_MASK 0x000001FF

void setBusDirectionOutput(bool isOutput) {
  for(int i=0; i<9; i++) gpio_set_dir(PROFILE_DATA_BASE + i, isOutput);
}

void takeBusForCPU() {
  for(int i=0; i<9; i++) gpio_set_function(PROFILE_DATA_BASE + i, GPIO_FUNC_SIO);
}

void giveBusToPIO() {
  for(int i=0; i<9; i++) pio_gpio_init(profile_pio, PROFILE_DATA_BASE + i);
}

void sendDataCPU(uint8_t data) {
  uint32_t parity = getOddParity(data);
  uint32_t bus_val = (uint32_t)data | (parity << 8);
  gpio_put_masked(DATA_PARITY_MASK, bus_val);
}

uint8_t receiveDataCPU() {
  return (uint8_t)(gpio_get_all() & DATA_BUS_MASK);
}

// helper: wait for a pin to reach a given level with timeout.
// returns true on success, false on timeout.
static inline bool waitForPinLevel(uint pin, bool level, uint32_t max_loops) {
  while (gpio_get(pin) != level && --max_loops) tight_loop_contents();
  return max_loops != 0;
}

// helper: wait for the host to place a specific byte on the bus with timeout.
// returns true on success, false on timeout.
static inline bool waitForBusValue(uint8_t expected, uint32_t max_loops) {
  while (receiveDataCPU() != expected && --max_loops) tight_loop_contents();
  return max_loops != 0;
}

// helper: compute max block number for the current disk image
static inline uint32_t maxBlocks() {
  return disk.isOpen() ? (uint32_t)(disk.fileSize() / SECTOR_SIZE) : 0;
}

void setup1() {
  // 1. Initialize ProFile Control Pins
  pinMode(PROFILE_CMD, INPUT_PULLUP);
  pinMode(PROFILE_RW, INPUT_PULLUP);
  pinMode(PROFILE_STRB, INPUT_PULLUP);
  
  pinMode(PROFILE_BSY, OUTPUT);
  pinMode(PROFILE_PRES, OUTPUT);
  
  digitalWrite(PROFILE_BSY, HIGH); // Inactive (Active Low)
  digitalWrite(PROFILE_PRES, LOW); // Active Low presence

  // 2. Initialize PIO State Machines
  uint offset_read = pio_add_program(profile_pio, &profile_read_program);
  uint offset_write = pio_add_program(profile_pio, &profile_write_program);
  
  sm_read = pio_claim_unused_sm(profile_pio, true);
  sm_write = pio_claim_unused_sm(profile_pio, true);
  
  profile_read_program_init(profile_pio, sm_read, offset_read, PROFILE_DATA_BASE, PROFILE_STRB);
  profile_write_program_init(profile_pio, sm_write, offset_write, PROFILE_DATA_BASE, PROFILE_STRB);

  // 3. Initialize DMA Channels
  dma_rx_chan = dma_claim_unused_channel(true);
  dma_tx_chan = dma_claim_unused_channel(true);

  // configure RX DMA — PIO pushes 8-bit data into 32-bit FIFO words
  rx_cfg = dma_channel_get_default_config(dma_rx_chan);
  channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_32);
  channel_config_set_read_increment(&rx_cfg, false);
  channel_config_set_write_increment(&rx_cfg, true);
  channel_config_set_dreq(&rx_cfg, pio_get_dreq(profile_pio, sm_read, false));

  // configure TX DMA — PIO expects 16-bit words (8 data + parity bit 8)
  tx_cfg = dma_channel_get_default_config(dma_tx_chan);
  channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&tx_cfg, true);
  channel_config_set_write_increment(&tx_cfg, false);
  channel_config_set_dreq(&tx_cfg, pio_get_dreq(profile_pio, sm_write, true));

  // initialize SD card on SPI0 with explicit pin assignments
  SPI.setRX(SD_MISO);
  SPI.setTX(SD_MOSI);
  SPI.setSCK(SD_SCK);
  SPI.setCS(SD_CS);
  if (!SDCard.begin(SD_CS, SPI_FULL_SPEED)) {
    Serial.println("SD initialization failed!");
  } else {
    Serial.println("SD initialized.");
    if (disk.open(fileName, O_RDWR)) {
      Serial.println("Disk image opened.");

      // expose the raw SD card to the USB host for imaging/diagnostics
      usb_msc.setCapacity((uint32_t)(SDCard.card()->sectorCount()), 512);
      usb_msc.setUnitReady(true);

#if USE_PSRAM_CACHE
      // RP2350 QMI maps PSRAM starting at 0x11000000
      psram_cache = (uint8_t*)0x11000000;
      Serial.println("Loading disk to PSRAM...");
      // read in 4KB chunks for faster sequential SD throughput
      #define PSRAM_PRELOAD_CHUNK 4096
      uint32_t total = disk.fileSize();
      disk.seekSet(0);
      for (uint32_t off = 0; off < total; off += PSRAM_PRELOAD_CHUNK) {
          uint32_t chunk = (total - off < PSRAM_PRELOAD_CHUNK) ? (total - off) : PSRAM_PRELOAD_CHUNK;
          disk.read(&psram_cache[off], chunk);
      }
      Serial.println("PSRAM Ready.");
#endif
    } else {
      Serial.println("Failed to open disk image.");
    }
  }
}

void loop1() {
  takeBusForCPU();
  setBusDirectionOutput(false); // input initially
  digitalWrite(PROFILE_BSY, HIGH); // inactive (active low)

  // 1. wait for host to assert CMD (no timeout — idle state)
  while (digitalRead(PROFILE_CMD) == HIGH) {
    tight_loop_contents();
  }

  // initial handshake: acknowledge presence
  setBusDirectionOutput(true);
  sendDataCPU(PROFILE_ACK_PRESENCE);
  digitalWrite(PROFILE_BSY, LOW);

  // wait for host to raise CMD (with timeout)
  if (!waitForPinLevel(PROFILE_CMD, HIGH, HANDSHAKE_TIMEOUT)) return;

  // wait for host confirmation byte 0x55 (with timeout)
  setBusDirectionOutput(false);
  if (!waitForBusValue(PROFILE_HOST_CONFIRM, HANDSHAKE_TIMEOUT)) return;

  digitalWrite(PROFILE_BSY, HIGH); // raise BSY to show ready for command

  // 3. start DMA to receive 6-byte command via PIO
  giveBusToPIO();
  pio_sm_clear_fifos(profile_pio, sm_read);
  pio_sm_restart(profile_pio, sm_read);
  dma_channel_configure(
    dma_rx_chan,
    &rx_cfg,
    dma_rx_buffer,                                // dest (uint32_t[])
    &profile_pio->rxf[sm_read],                   // src (PIO RX FIFO)
    6,                                            // 6 command bytes
    true                                          // start immediately
  );

  pio_sm_set_enabled(profile_pio, sm_read, true);
  dma_channel_wait_for_finish_blocking(dma_rx_chan);
  pio_sm_set_enabled(profile_pio, sm_read, false);

  // extract bytes from 32-bit DMA words (PIO pushes 8-bit data into LSB of each word)
  uint8_t cmd_idx = (uint8_t)dma_rx_buffer[0];
  uint32_t block_num = ((uint8_t)dma_rx_buffer[1] << 16) | ((uint8_t)dma_rx_buffer[2] << 8) | (uint8_t)dma_rx_buffer[3];

  if (cmd_idx == PROFILE_CMD_READ) {
    // READ command (drive -> host)

    // handshake: confirm command
    takeBusForCPU();
    setBusDirectionOutput(true);
    sendDataCPU(PROFILE_ACK_READ);
    digitalWrite(PROFILE_BSY, LOW);

    if (!waitForPinLevel(PROFILE_CMD, HIGH, HANDSHAKE_TIMEOUT)) return;

    setBusDirectionOutput(false);
    if (!waitForBusValue(PROFILE_HOST_CONFIRM, HANDSHAKE_TIMEOUT)) return;

    digitalWrite(PROFILE_BSY, HIGH);

    // read block data into temp buffer, then pack into 16-bit TX buffer with parity
    uint8_t temp_buf[PROFILE_FRAME_SIZE];
    memset(temp_buf, 0, STATUS_BYTES); // first 4 bytes are status (0x00 = success)

#if USE_PSRAM_CACHE
    if (block_num < maxBlocks()) {
        memcpy(&temp_buf[STATUS_BYTES], &psram_cache[block_num * SECTOR_SIZE], SECTOR_SIZE);
    } else {
        memset(&temp_buf[STATUS_BYTES], 0, SECTOR_SIZE);
    }
#else
    mutex_enter_blocking(&sd_mutex);
    if (disk.isOpen() && block_num < maxBlocks()) {
        disk.seekSet((uint32_t)block_num * SECTOR_SIZE);
        disk.read(&temp_buf[STATUS_BYTES], SECTOR_SIZE);
    } else {
        memset(&temp_buf[STATUS_BYTES], 0, SECTOR_SIZE);
    }
    mutex_exit(&sd_mutex);
#endif

    // pack 8-bit data + parity into 16-bit words: bit[7:0] = data, bit[8] = odd parity
    for (int i = 0; i < PROFILE_FRAME_SIZE; i++) {
        dma_tx_buffer[i] = (uint16_t)temp_buf[i] | ((uint16_t)getOddParity(temp_buf[i]) << 8);
    }

    giveBusToPIO();
    pio_sm_clear_fifos(profile_pio, sm_write);
    pio_sm_restart(profile_pio, sm_write);
    dma_channel_configure(
      dma_tx_chan,
      &tx_cfg,
      &profile_pio->txf[sm_write],                  // dest (PIO TX FIFO)
      dma_tx_buffer,                                // src (uint16_t[])
      PROFILE_FRAME_SIZE,                           // 536 elements
      true
    );

    pio_sm_set_enabled(profile_pio, sm_write, true);
    dma_channel_wait_for_finish_blocking(dma_tx_chan);
    pio_sm_set_enabled(profile_pio, sm_write, false);
  }
  else if (cmd_idx == PROFILE_CMD_WRITE || cmd_idx == PROFILE_CMD_WRITE_VFY || cmd_idx == PROFILE_CMD_WRITE_FSP) {
    // WRITE command (host -> drive)
    // 0x01 = write, 0x02 = write-verify, 0x03 = write-force-spare
    // all three use the same handshake echo: cmd + 2

    takeBusForCPU();
    setBusDirectionOutput(true);
    sendDataCPU(cmd_idx + 0x02); // echo cmd + 2
    digitalWrite(PROFILE_BSY, LOW);

    if (!waitForPinLevel(PROFILE_CMD, HIGH, HANDSHAKE_TIMEOUT)) {
        pio_sm_set_enabled(profile_pio, sm_read, false);
        return;
    }

    setBusDirectionOutput(false);
    if (!waitForBusValue(PROFILE_HOST_CONFIRM, HANDSHAKE_TIMEOUT)) {
        pio_sm_set_enabled(profile_pio, sm_read, false);
        return;
    }

    digitalWrite(PROFILE_BSY, HIGH); // ready to receive data

    giveBusToPIO();
    pio_sm_clear_fifos(profile_pio, sm_read);
    pio_sm_restart(profile_pio, sm_read);
    dma_channel_configure(
      dma_rx_chan,
      &rx_cfg,
      dma_rx_buffer,                                // dest (uint32_t[])
      &profile_pio->rxf[sm_read],                   // src (PIO RX FIFO)
      SECTOR_SIZE,                                  // 532 elements from host
      true
    );

    pio_sm_set_enabled(profile_pio, sm_read, true);
    dma_channel_wait_for_finish_blocking(dma_rx_chan);
    pio_sm_set_enabled(profile_pio, sm_read, false);

    // extract 8-bit data from 32-bit DMA words and write to storage
    uint8_t sector_buf[SECTOR_SIZE];
    for (int i = 0; i < SECTOR_SIZE; i++) {
        sector_buf[i] = (uint8_t)dma_rx_buffer[i];
    }

#if USE_PSRAM_CACHE
    if (block_num < maxBlocks()) {
        memcpy(&psram_cache[block_num * SECTOR_SIZE], sector_buf, SECTOR_SIZE);
        markSectorDirty(block_num);
    }
#else
    mutex_enter_blocking(&sd_mutex);
    if (disk.isOpen() && block_num < maxBlocks()) {
        disk.seekSet((uint32_t)block_num * SECTOR_SIZE);
        disk.write(sector_buf, SECTOR_SIZE);
        disk.flush();
    }
    mutex_exit(&sd_mutex);
#endif

    // post-receive handshake
    takeBusForCPU();
    setBusDirectionOutput(true);
    sendDataCPU(PROFILE_ACK_DATA);
    digitalWrite(PROFILE_BSY, LOW);

    if (!waitForPinLevel(PROFILE_CMD, HIGH, HANDSHAKE_TIMEOUT)) return;

    setBusDirectionOutput(false);
    if (!waitForBusValue(PROFILE_HOST_CONFIRM, HANDSHAKE_TIMEOUT)) return;

    digitalWrite(PROFILE_BSY, HIGH);

    // send 4 status bytes (all zero = success) with parity
    uint16_t status_word = (uint16_t)0x00 | ((uint16_t)getOddParity(0x00) << 8);
    for (int i = 0; i < STATUS_BYTES; i++) {
        dma_tx_buffer[i] = status_word;
    }

    giveBusToPIO();
    pio_sm_clear_fifos(profile_pio, sm_write);
    pio_sm_restart(profile_pio, sm_write);
    dma_channel_configure(
      dma_tx_chan,
      &tx_cfg,
      &profile_pio->txf[sm_write],
      dma_tx_buffer,
      STATUS_BYTES,                                 // 4 status elements
      true
    );

    pio_sm_set_enabled(profile_pio, sm_write, true);
    dma_channel_wait_for_finish_blocking(dma_tx_chan);
    pio_sm_set_enabled(profile_pio, sm_write, false);
  }
}
