# RProFile Architecture

**RP2350-based Apple ProFile Hard Drive Emulator**
*2026-02-21*

## Overview

RProFile emulates an Apple ProFile hard drive using a Raspberry Pi RP2350 microcontroller. It serves disk images stored on an SD card over the original ProFile parallel bus protocol, using PIO state machines and DMA for real-time bus I/O. A USB Mass Storage interface on a separate core allows the SD card to be accessed from a modern computer.

```
┌─────────────────────────────────────────────────────────┐
│                       RP2350                            │
│                                                         │
│  Core 0                          Core 1                 │
│  ┌──────────────────┐            ┌────────────────────┐ │
│  │ USB MSC (TinyUSB)│            │ ProFile Bus Engine │ │
│  │ - read/write SD  │◄──mutex───►│ - PIO + DMA I/O   │ │
│  │ - flush/sync     │            │ - SD / PSRAM R/W   │ │
│  │ - PSRAM flusher  │            │ - handshake FSM    │ │
│  └──────────────────┘            └────────────────────┘ │
│           │                              │              │
│           ▼                              ▼              │
│      ┌─────────┐                ┌────────────────┐      │
│      │ SD Card │                │  PIO0 (2 SMs)  │      │
│      │ (SPI0)  │                │  read / write   │      │
│      └─────────┘                └────────────────┘      │
└─────────────────────────────────────────────────────────┘
```

## Dual-Core Architecture

### Core 0 — USB & System Management

- Runs the TinyUSB device stack (`tud_task()`) in `loop()`.
- Exposes the raw SD card as a USB Mass Storage device for imaging and diagnostics.
- MSC callbacks (`msc_read_cb`, `msc_write_cb`, `msc_flush_cb`) access the SD card directly using 512-byte sector addressing.
- When `USE_PSRAM_CACHE` is enabled, Core 0 also runs the **write-through background flusher** (see PSRAM section below).

### Core 1 — ProFile Bus Engine

- Runs the ProFile protocol state machine in `loop1()`.
- Handles the full command lifecycle: presence handshake → command reception → data transfer → status.
- All bus I/O for bulk data uses PIO + DMA; the CPU-driven handshake phases use GPIO directly.
- SD card access is guarded by `sd_mutex` to prevent races with Core 0's MSC callbacks.

### Mutual Exclusion

USB MSC and ProFile emulation are **mutually exclusive** — they do not operate simultaneously. The `sd_mutex` provides safety at the API level, but the design intent is that USB access pauses emulation entirely.

## GPIO Pin Map

| Pin | GPIO | Function | Direction |
|---|---|---|---|
| D0–D7 | 0–7 | 8-bit data bus | bidirectional |
| Parity | 8 | 9th bit (odd parity) | bidirectional |
| STRB | 9 | strobe from host | input |
| CMD | 10 | command from host | input |
| BSY | 11 | busy to host | output |
| R/W | 12 | read/write | input |
| PRES | 13 | presence detect | output |
| SD_CS | 15 | SD chip select (SPI0) | output |
| SD_MISO | 16 | SPI0 RX | input |
| SD_SCK | 18 | SPI0 SCK | output |
| SD_MOSI | 19 | SPI0 TX | output |

> [!IMPORTANT]
> GPIO 0–8 must be contiguous. The PIO write program uses `out pins, 9` to drive all 9 bits (D0–D7 + parity) atomically in a single instruction.

## PIO State Machines

Two state machines on PIO0 handle the timed, strobe-synchronized bus transfers:

### `profile_read` — Host → Drive

Used during command reception and write-data reception. Waits for the host to assert STRB (active low), samples 8 bits from the data bus via `in pins, 8`, pushes to the RX FIFO, then waits for STRB release. The RX FIFO is joined (8 words deep) for maximum DMA throughput.

### `profile_write` — Drive → Host

Used during read-data transmission and status output. Pulls a 16-bit word from the TX FIFO (8 data bits + parity in bit 8), drives 9 pins via `out pins, 9`, then waits for the host to acknowledge with a STRB pulse. The TX FIFO is joined (8 words deep).

### DMA Buffers

- **RX:** `uint32_t dma_rx_buffer[536]` — PIO pushes 8-bit data into 32-bit FIFO words; the CPU extracts the LSB of each word after transfer.
- **TX:** `uint16_t dma_tx_buffer[536]` — each element holds `data[7:0] | (parity << 8)`, packed by the CPU before transfer.

## ProFile Protocol Flow

### Read Command (0x00)

```
Host                          RProFile (Drive)
  │                                │
  │── CMD asserted (low) ─────────►│
  │◄── BSY low + ACK (0x01) ──────│  presence
  │── CMD released (high) ────────►│
  │── 0x55 on bus ────────────────►│  confirmation
  │                                │  BSY high (ready)
  │── 6 cmd bytes via STRB ──────►│  PIO+DMA receive
  │◄── BSY low + ACK (0x02) ──────│  read acknowledged
  │── CMD released ───────────────►│
  │── 0x55 ───────────────────────►│  confirmation
  │                                │  read sector from SD/PSRAM
  │◄── 536 bytes via STRB ────────│  PIO+DMA transmit (4 status + 532 data)
```

### Write Command (0x01 / 0x02 / 0x03)

```
Host                          RProFile (Drive)
  │                                │
  │  (same presence handshake)     │
  │── 6 cmd bytes via STRB ──────►│  PIO+DMA receive
  │◄── BSY low + ACK (cmd+2) ────│  write acknowledged
  │── CMD released ───────────────►│
  │── 0x55 ───────────────────────►│  confirmation
  │── 532 data bytes via STRB ───►│  PIO+DMA receive
  │                                │  write sector to SD/PSRAM
  │◄── BSY low + ACK (0x06) ──────│  data received OK
  │── CMD released ───────────────►│
  │── 0x55 ───────────────────────►│  confirmation
  │◄── 4 status bytes via STRB ──│  PIO+DMA transmit
```

## PSRAM Write-Through Cache

### Purpose

SD card write latency (~1–5 ms per sector) can violate the ProFile protocol's tight handshake timing. When an RP2350 board with PSRAM is used, the entire disk image can be held in PSRAM to eliminate SD latency from the critical path.

### Enable / Disable

```cpp
#define USE_PSRAM_CACHE false  // set to true when hardware has PSRAM
```

All PSRAM-related code is compiled out via `#if USE_PSRAM_CACHE` when disabled.

### Memory Layout

The RP2350 QMI peripheral maps PSRAM starting at `0x11000000`. At startup (`setup1()`), the full disk image is loaded into PSRAM in `PSRAM_PRELOAD_CHUNK`-sized (4 KB) sequential reads for maximum SD throughput.

ProFile sectors are 532 bytes (not 512), so the PSRAM cache is addressed using ProFile sector geometry:

```
psram_cache[sector * 532] ... psram_cache[sector * 532 + 531]
```

### Write-Through Architecture

```
                Core 1 (ProFile Bus)              Core 0 (Background)
                ┌────────────────────┐            ┌─────────────────────┐
  host write ──►│ memcpy to PSRAM    │            │                     │
                │ markSectorDirty()  │──ring──►   │ dequeue dirty sector│
                │ (return to host    │  buffer    │ seekSet + write SD  │
                │  immediately)      │            │ sync                │
                └────────────────────┘            └─────────────────────┘
```

- **Writes** go to PSRAM instantly, and the sector number is pushed into a fixed-size ring buffer (`DIRTY_RING_SIZE = 256`).
- **Reads** are served directly from PSRAM with zero SD latency.
- **Background flusher** on Core 0 (`loop()`) dequeues one dirty sector per tick and persists it to the SD card via `File32`. The `sd_mutex` guards access to the `disk` file object.
- **`cache_mutex`** protects the ring buffer head/tail between cores.

### USB MSC Interaction

MSC callbacks always access the SD card directly using raw 512-byte sector addressing — they **never** touch the PSRAM cache. The two address spaces (512-byte SD sectors vs. 532-byte ProFile sectors) are incompatible, and USB access is mutually exclusive with ProFile emulation by design.

## Timeout & Error Handling

- All handshake waits use a loop-count timeout (`HANDSHAKE_TIMEOUT = 15,000,000` iterations, ~100 ms at 150 MHz).
- On any timeout, `loop1()` returns immediately, which resets the bus state at the top of the next iteration (BSY high, bus direction input).
- The `disk.isOpen()` guard at the top of `loop1()` prevents bus participation when no disk image is loaded.
