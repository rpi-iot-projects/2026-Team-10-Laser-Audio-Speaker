# 2026-Team-10-Laser-Audio-Speaker
## LiFi File Transfer

A OOK modulation laser file transfer system using two Raspberry Pis. Files are sent from the **TX Pi** over a laser beam using PWM encoding and received by the **RX Pi** via a photodiode, transimpedance amplifier, and comparitor. The recived packets are then verified with CRC-8 and ARQ retransmit.

## Table of Contents

- [Overview](#overview)
- [Hardware Components](#hardware-components)
- [Software and Dependencies](#software-and-dependencies)
- [Usage](#usage)
- [Results and Demonstration](#results-and-demonstration)

---

## Overview

This project implements a one way LiFi (Light Fidelity) file transfer link between two Raspberry Pis via an TLL modulated laser. Files of any type are transmitted from a TX Pi over a laser beam using PWM (pulse-width modulation) encoding, and received by an RX Pi via a photodiode (photovoltaic mode), transimpedance amplifier, and comparator. Each transmission receives a header with packet metadata and data packets verified with a CRC-8 checksum. Missing or corrupt packets are automatically retransmitted using an ARQ (Automatic Repeat Request) over a wired back channel. The primary laser then retransmits the packets until all were received, where the file is then stored in the working directory of the RX pi. 

### Protocol

- **Encoding:** PWM: every bit has a rising edge, making them reliably detectable
  - Start pulse: 350 µs HIGH + 50 µs gap
  - Bit 1: 200 µs HIGH + 50 µs gap
  - Bit 0: 100 µs HIGH + 50 µs gap
- **Packet size:** 512 bytes
- **Checksum:** CRC-8 over `[seq_hi, seq_lo, len_hi, len_lo, payload_byte_0, payload_byte_1, ..., payload_byte_(len-1)]`
- **Framing:** Byte stuffing (0xFF / 0xFE / 0xFD escaped)
- **Retransmit:** ARQ: RX NAKs any missing or corrupt packets; TX resends only those

### Transfer Flow

```
TX                                        RX
 │                                         │
 ├── session header (loop every 2s) ──────►│
 │◄───────────────────── READY (0xBB 0x01) │
 │                                         │
 ├── packet 0 ───────────────────────────► │
 ├── packet 1 ───────────────────────────► │
 │ (number of packets depends on file)     │
 ├── end marker (0xAA 0xFF 0xEE) ────────► │
 │                                         │
 │◄─── NAK for any bad pkts (back channel)─┤
 │◄─────────────────────── DONE (0xBB 0x03)│
 │                                         │
 │      (repeat round for NAK'd pkts)      │
```

---

## Hardware Components

| Component | Role |
|---|---|
| Raspberry Pi (TX) | Sends file over laser |
| Raspberry Pi (RX) | Receives file via photodiode |
| Laser diode (Quarton: VLM-520-61 LPO hardware limited to ≤10 kHz modulation) | Forward optical channel |
| Photodiode (SLD-70BG2A) | Converts light to current |
| OPA380 with 47kΩ Rf | Transimpedance amplifier |
| TLV3201 comparator | Converts analog signal to GPIO-level digital (0.0 V low / 2.2 V high) |
| Wire (GPIO 27 ↔ GPIO 27) | Back channel for ACK/NAK |

### GPIO Pinout (both Pis)

| Pin | Direction | Purpose |
|---|---|---|
| GPIO 17 | Output (TX) / Input (RX) | Forward channel: laser data |
| GPIO 27 | Input (TX) / Output (RX) | Back channel: ACK/NAK wire |

---

## Software and Dependencies

**Language:** C++

**Library:** [pigpio](https://abyz.me.uk/rpi/pigpio/): provides DMA waveform generation (TX) and GPIO interrupt callbacks (RX)

Install on both Pis:

```bash
sudo apt install libpigpio-dev
```

**Build** (run on each Pi):

```bash
g++ -o txDma txDma.cpp -lpigpio -lrt -pthread
g++ -o rxDma rxDma.cpp -lpigpio -lrt -pthread
```

### Architecture Notes

- **TX uses pigpio DMA** (`gpioWaveCreate` / `gpioWaveTxSend`): pulse timings are driven by the Pi's DMA, not the CPU, so they are accurate to ~1 µs regardless of OS scheduling
- **RX uses interrupt-driven decoding** (`gpioSetAlertFunc`): the alert callback timestamps each edge and pushes decoded bytes to a queue then a separate thread handles packet assembly
- **Back channel** (GPIO 27 wire): carries ACK/NAK using the same PWM encoding in reverse

---

## Usage

### Step 1: Start RX Pi first

```bash
# Start the pigpio daemon
sudo pigpiod

# Activate the pigpio virtual environment
source pigpio-venv/bin/activate

# Run the receiver (root required for GPIO access)
sudo ./rxDma
```

RX will print:
```
Listening for session header (interrupt-driven)...
```

### Step 2: Start TX Pi

Any file type is supported:

```bash
sudo ./txDma <file>
```

Examples:
```bash
sudo ./txDma photo.jpg
sudo ./txDma music.mp3
```

TX will loop resending the session header every 2 seconds until RX responds with READY, then begin the transfer automatically. **RX must be started before or shortly after TX.**

### Tips

- **Dark room:** less ambient light on the photodiode means fewer CRC errors and fewer retransmit rounds. Note that testing showed successful transfers even with a light shone directly at the photodiode, since it is most sensitive to the laser's 500–600 nm wavelength
- **Shorter distance:** stronger signal and cleaner pulse edges

---

## Results and Demonstration

**TX Pi output:**
```
File: photo.jpg (6940 bytes, 14 packets)
Round 1: 14 packets
  14/14  seq=13
Transfer complete in 18.4s  (0.37 KB/s)
```

**RX Pi output:**
```
Listening for session header (interrupt-driven)...
Session: 6940 bytes, 14 packets, saving as: received_photo.jpg
Sending READY...
14/14 received  missing=0
Transfer complete.
```

The received file is saved as `received_<original_filename>` in the working directory on the RX Pi.
