//rxDma.cpp LiFi File Receiver

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <set>
#include <deque>
#include <string>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <csignal>
#include <pigpio.h>

// Constants:
static const int FWD_PIN = 17; // GPIO pin to photodiode, opam, and comparitor
static const int BACK_PIN = 27; // GPIO pin to TX Pi (wire, NAK/ACK)
static const uint32_t PERIOD_US = 100; // base time unit to must match txDma.cpp
static const uint32_t START_PULSE_US = PERIOD_US * 4; // 400µs to match txDma.cpp
static const int PACKET_SIZE = 512;

// Back-channel message codes
static const uint8_t MSG_READY = 0x01; // we are ready to receive
static const uint8_t MSG_NAK = 0x02; // requesting retransmit of seq
static const uint8_t MSG_DONE = 0x03; // finished reporting NAKs
static const uint8_t MSG_MAGIC = 0xBB; // framing/sync byte before every message

static volatile bool running = true;
void handle_sigint(int) { running = false; } // Ctrl+C sets flag; main loop exits cleanly

// CRC-8: check sum
/*
Polynomial: x^8 + x^2 + x + 1.
Applied over [seq_hi, seq_lo, len_hi, len_lo, raw_payload].
*/
uint8_t crc8(const uint8_t* data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

// Forward channel RX: interrupt-based PWM decoder
/*
pigpio calls fwd_alert() on every edge of FWD_PIN from its own thread.
Measure the width of each HIGH pulse on the falling edge:

    width >= 3×PERIOD (300µs)
    width >= 3/2×PERIOD (150µs): bit 1 (200µs)
    width >= 1/2×PERIOD (50µs): bit 0 (100µs)
    width < 1/2×PERIOD: ignore
*/
enum DecState { DEC_IDLE, DEC_IN_BITS };

// Decoder state: written only from the fwd_alert() callback thread.
static struct {
    DecState state = DEC_IDLE; // if between bytes or inside one
    uint32_t t_rise = 0; // timestamp of the last rising edge
    int bit_count = 0; // bits received so far in the current byte
    uint8_t  byte_val  = 0; // byte being assembled, MSB first
} dec;

// Thread safe queue between fwd_alert() and the main thread.
static std::deque<uint8_t> byte_queue;
static std::mutex queue_mtx;
static std::condition_variable queue_cv;

// pigpio calls this on every rising/falling edge of FWD_PIN.
// On rising edge: record timestamp.
// On falling edge: measure pulse width and decode it.
void fwd_alert(int /*gpio*/, int level, uint32_t tick)
{
    if (level == 1)
    {
        dec.t_rise = tick; // record when this pulse started
        return;
    }

    // Falling edge: compute how long the line was HIGH
    uint32_t w = tick - dec.t_rise;

    // Reject < 50µs
    if (w < PERIOD_US / 2)
    {
        return;
    }

    if (dec.state == DEC_IDLE) {
        // Looking for a start pulse: anything >= 3×PERIOD (300µs)
        if (w >= PERIOD_US * 3) 
        {
            dec.state = DEC_IN_BITS;
            dec.bit_count = 0;
            dec.byte_val  = 0;
        }
        // Pulses shorter than 3× are ignore 

    } 
    else {
        // Width threshold sits between the nominal 0 (100µs) and 1 (200µs):
        //>= 150µs is a 1,  < 150µs is a 0
        int bit = (w >= PERIOD_US * 3 / 2) ? 1 : 0;

        // Bits arrive MSB first;
        dec.byte_val |= (bit << (7 - dec.bit_count));

        if (++dec.bit_count == 8) {
            // Full byte received, so push to queue and wake the main thread
            {
                std::lock_guard<std::mutex> lk(queue_mtx);
                byte_queue.push_back(dec.byte_val);
            }
            queue_cv.notify_one();
            dec.state = DEC_IDLE;// ready for next byte's start pulse
        }
    }
}

// Block until a byte arrives in the queue or timeout_us passes.
int fwd_read_byte_irq(uint32_t timeout_us)
{
    std::unique_lock<std::mutex> lk(queue_mtx);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeout_us);

    while (byte_queue.empty()) 
    {
        if (queue_cv.wait_until(lk, deadline) == std::cv_status::timeout)
        {
            return -1;
        }
    }

    uint8_t b = byte_queue.front();
    byte_queue.pop_front();
    return b;
}

// Discard any bytes that have accumulated in the queue.
void fwd_flush_queue()
{
    std::lock_guard<std::mutex> lk(queue_mtx);
    byte_queue.clear();
}

// Wait until FWD_PIN has been idle (continuously LOW) for idle_us, then flush the queue and reset the decoder state machine.
void fwd_sync_idle(uint32_t idle_us    = PERIOD_US * 50, uint32_t max_wait_us = 3000000)
{
    uint32_t last_edge = gpioTick();
    uint32_t start = last_edge;
    while (gpioTick() - last_edge < idle_us)
    {
        if (gpioTick() - start > max_wait_us)
        {
            break;
        }
        if (gpioRead(FWD_PIN) == 1)
        {
            last_edge = gpioTick(); // activity seen, reset the idle timer
        }
        gpioDelay(10);
    }
    // Reset decoder state so we start fresh from the next start pulse
    dec.state = DEC_IDLE;
    dec.bit_count = 0;
    dec.byte_val = 0;
    dec.t_rise = 0;
    fwd_flush_queue();
}

// Read exactly n bytes into buf. Returns false if any byte times out.
bool fwd_read_n(std::vector<unsigned char>& buf, int n,
                uint32_t timeout_us = PERIOD_US * 2000)
{
    for (int i = 0; i < n; ++i) {
        int b = fwd_read_byte_irq(timeout_us);
        if (b < 0) return false;
        buf.push_back((unsigned char)b);
    }
    return true;
}

// Byte de-stuffing 
/*
Reverse of the stuffing applied by TX:
    0xFE 0x00: 0xFE
    0xFE 0x01: 0xFF
    0xFE 0x02: 0xFD
*/
bool read_stuffed_data(std::vector<unsigned char>& out, int expected_len, uint32_t timeout_us)
{
    while ((int)out.size() < expected_len) 
    {
        int b = fwd_read_byte_irq(timeout_us);
        if (b < 0)
        { 
            return false;
        }

        if (b == 0xFE) {
            // Escape sequence: next byte determines the original value
            int next = fwd_read_byte_irq(timeout_us);
            if (next < 0)
            {
                return false;
            }

            if (next == 0x00)
            {
                out.push_back(0xFE);
            }
            else if (next == 0x01)
            {
                out.push_back(0xFF);
            }
            else if (next == 0x02)
            {
                out.push_back(0xFD);
            }
            else
            {
                out.push_back((unsigned char)next); // bad format
            }
        } 
        else {
            out.push_back((unsigned char)b);
        }
    }
    // Consume the trailing 0xFD end of payload marker (we don't need its value)
    fwd_read_byte_irq(timeout_us);
    return true;
}

// Back channel TX (wire)
/*
Uses the same PWM encoding as the forward channel but transmitted over a direct wire to GPIO 27 on the TX Pi.

Encoding (same as forward channel):
    Start pulse : HIGH 4×PERIOD (400µs) + LOW 1×PERIOD gap
    Bit 1 : HIGH 1×PERIOD (100µs) + LOW 1×PERIOD (total 2× = 200µs)
    Bit 0 : LOW  2×PERIOD (200µs) — no HIGH pulse for a zero
*/
std::vector<gpioPulse_t> back_pulses;

void back_add_bit(int bit)
{
    if (bit) 
    {
        // 1 bit: short HIGH pulse followed by a gap
        back_pulses.push_back({ (1u << BACK_PIN), 0, PERIOD_US });
        back_pulses.push_back({ 0, (1u << BACK_PIN), PERIOD_US });
    }
    
    else {
        // '0' bit: line stays LOW for a full 2× period
        back_pulses.push_back({ 0, (1u << BACK_PIN), PERIOD_US * 2 });
    }
}

void back_add_byte(unsigned char byte)
{
    // Start pulse: signals start of byte to TX
    back_pulses.push_back({ (1u << BACK_PIN), 0, START_PULSE_US });
    back_pulses.push_back({ 0, (1u << BACK_PIN), PERIOD_US });
    for (int i = 7; i >= 0; --i)
    {   // MSB first
        back_add_bit((byte >> i) & 1);
    }
}

// Transmit accumulated back channel pulses via DMA waveform, then clear list.
void back_flush(uint32_t flush_wait_ms = 50)
{
    // Trailing idle period so TX can detect end-of-message
    back_pulses.push_back({ 0, (1u << BACK_PIN), PERIOD_US * 100 });
    gpioWaveClear();
    gpioWaveAddGeneric(back_pulses.size(), back_pulses.data());
    int wave_id = gpioWaveCreate();
    if (wave_id < 0)
    {
        std::cerr << "Back wave create failed\n";
        return;
    }
    gpioWaveTxSend(wave_id, PI_WAVE_MODE_ONE_SHOT);
    while (gpioWaveTxBusy())
    {
        gpioDelay(1000);
    }
    gpioDelay(flush_wait_ms * 1000);
    gpioWaveDelete(wave_id);
    back_pulses.clear();
}

// Build and send a multi byte back-channel message in one call.
void back_send_msg(std::initializer_list<unsigned char> bytes)
{
    back_pulses.clear();
    for (unsigned char b : bytes)
    {
        back_add_byte(b);
    }
    back_flush();
}

// Every message starts with MSG_MAGIC (0xBB) so TX can frame them correctly.
void send_ready() { back_send_msg({ MSG_MAGIC, MSG_READY }); }
void send_done() { back_send_msg({ MSG_MAGIC, MSG_DONE  }); }

// NAK is sent 3 times per missing packet
void send_nak(uint16_t seq)
{
    for (int i = 0; i < 3; ++i)
        back_send_msg({ MSG_MAGIC, MSG_NAK, (unsigned char)((seq >> 8) & 0xFF), (unsigned char)(seq & 0xFF) });
}

// Main:

int main()
{
    if (gpioInitialise() < 0) { std::cerr << "pigpio init failed\n"; return 1; }
    gpioSetMode(FWD_PIN,  PI_INPUT);  gpioSetPullUpDown(FWD_PIN,  PI_PUD_DOWN);
    gpioSetMode(BACK_PIN, PI_OUTPUT); gpioWrite(BACK_PIN, 0);
    std::signal(SIGINT, handle_sigint);

    // Register the edge-detection callback. pigpio fires fwd_alert() on every rising or falling edge of FWD_PIN from a dedicated internal thread.
    gpioSetAlertFunc(FWD_PIN, fwd_alert);

    std::cout << "Listening for session header (interrupt-driven)...\n";
    fwd_sync_idle(); // wait before we start looking for headers

    // Session reset state
    uint32_t filesize = 0;
    uint16_t total_packets = 0;
    std::string output_filename;
    std::vector<unsigned char> filedata; // receive buffer
    std::set<uint16_t> received_seqs; // tracks which seq numbers arrived cleanly

    bool session_ready = false; // true after session headed parsed
    bool sending_ready = false; // true while we should periodically resend READY
    bool complete = false; // set when all packets received
    uint32_t last_ready = 0; // gpioTick() when we last sent READY
    std::chrono::steady_clock::time_point t_start;

    while (running) {
        // Session reset:
        // After a successful transfer, reset and wait for next header.
        if (complete) {
            complete = false;
            session_ready = false;
            sending_ready = false;
            received_seqs.clear();
            filedata.clear();
            fwd_sync_idle();
            continue;
        }
 
        // TX retransmits the session header every 2 seconds until it hears READY.
        if (session_ready && sending_ready) {
            uint32_t now = gpioTick();
            if (now - last_ready > 500000) 
            { // 500ms since last READY
                send_ready();
                last_ready = gpioTick();
            }
        }

        // Frame sync: wait for 0xAA 0xFF:
        /*
        All packet types (session header, data, end marker) begin with thesetwo sync bytes.
        */
        int b = fwd_read_byte_irq(400000); // 400ms timeout
        if (b < 0) continue;
        if (b != 0xAA) continue;

        b = fwd_read_byte_irq(PERIOD_US * 2000);
        if (b != 0xFF) continue;

        // Third byte distinguishes the packet type
        b = fwd_read_byte_irq(PERIOD_US * 2000);
        if (b < 0) continue;

        // Session header (third byte = 0x55):
        //
        // Format: 0xAA 0xFF 0x55
        //   filesize[31:24] filesize[23:16] filesize[15:8] filesize[7:0]
        //   total_packets[15:8] total_packets[7:0]
        //   fname_len fname[0..fname_len-1]
        if (b == 0x55) 
        {
            std::vector<unsigned char> hdr;
            if (!fwd_read_n(hdr, 6)) continue; // 4 bytes filesize + 2 bytes pkt count

            filesize = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) | ((uint32_t)hdr[2] <<  8) | hdr[3];
            total_packets = ((uint16_t)hdr[4] << 8) | hdr[5];

            // Reject obviously invalid sizes before allocating
            if (filesize == 0 || filesize > 10000000)
            {
                continue;
            }
            int fname_len = fwd_read_byte_irq(PERIOD_US * 2000);
            if (fname_len <= 0)
            {
                continue;
            }
            std::vector<unsigned char> fname_buf;
            if (!fwd_read_n(fname_buf, fname_len))
            {
                continue;
            }

            output_filename = "received_" + std::string(fname_buf.begin(), fname_buf.end());

            std::cout << "Session: " << filesize << " bytes, " << total_packets << " packets: " << output_filename << "\n";

            // Preallocate the receive buffer, so packets write directly into their correct offset and they can arrive in any order.
            filedata.assign(filesize, 0x00);
            received_seqs.clear();
            fwd_flush_queue(); // discard any stale bytes from before the header
            session_ready = true;
            sending_ready = true;
            last_ready = 0; // force immediate READY on next loop iteration
            t_start = std::chrono::steady_clock::now();
            continue;
        }

        // End marker (third byte = 0xEE):
        //
        // TX sends this after all packets in a round.
        if (b == 0xEE) {
            if (!session_ready) continue;
            sending_ready = false; // stop sending READY heartbeats

            // Find missing sequence numbers
            std::set<uint16_t> missing;
            for (uint16_t i = 0; i < total_packets; ++i)
                if (received_seqs.find(i) == received_seqs.end())
                    missing.insert(i);

            std::cout << "\n" << received_seqs.size() << "/" << total_packets << " received  missing=" << missing.size() << "\n";

            // Wait 2 seconds before responding to gives TX time to finish its own fwd_flush() and switch into back channel listening mode.
            gpioDelay(2000000);

            if (missing.empty()) {
                // All packets received: send DONE, write the file, and reset.
                send_done();
                std::ofstream out(output_filename, std::ios::binary);
                out.write(reinterpret_cast<const char*>(filedata.data()), filesize);
                out.close();
                auto t_end  = std::chrono::steady_clock::now();
                double secs = std::chrono::duration<double>(t_end - t_start).count();
                double rate = (filesize / 1024.0) / secs;
                std::cout << "Saved: " << output_filename
                          << "  (" << std::fixed << std::setprecision(1) << secs << "s  "
                          << std::setprecision(2) << rate << " KB/s)\n";
                session_ready = false;
                complete      = true;
            } else {
                // Send a NAK for each missing packet and then DONE.
                // TX will collect these and retransmit in the next round. Where each NAK is sent 3 times
                for (uint16_t seq : missing)
                {
                    send_nak(seq);
                }
                send_done();
                sending_ready = true; // resume READY heartbeat for next round
            }
            continue;
        }

        // Data packet (third byte = seq_hi)
        //
        // Format:  0xAA 0xFF seq_hi seq_lo len_hi len_lo stuffed payload 0xFD crc8
        if (!session_ready)
        {
            continue; // ignore stray data before we have a session
        }
        sending_ready = false;

        unsigned char seq_hi = (unsigned char)b;
        std::vector<unsigned char> rest;
        if (!fwd_read_n(rest, 3))
        {
            continue;// seq_lo, len_hi, len_lo
        }
        
        uint16_t seq = ((uint16_t)seq_hi << 8) | rest[0];
        uint16_t len = ((uint16_t)rest[1] << 8) | rest[2];

        // Validate length before allocating anything
        if (len == 0 || len > PACKET_SIZE)
        {
            std::cout << "Packet " << seq << " bad length " << (int)len << "\n";
            // Reset decoder so we re-sync to the next clean 0xAA 0xFF header
            fwd_flush_queue();
            dec.state = DEC_IDLE;
            dec.bit_count = 0;
            dec.byte_val = 0;
            continue;
        }

        // Read and destuff the payload
        std::vector<unsigned char> data;
        if (!read_stuffed_data(data, len, PERIOD_US * 5000))
        {
            std::cout << "Packet " << seq << " timeout in stuffed data\n"; 
            continue;
        }

        // Read the trailing CRC byte
        int crc_byte = fwd_read_byte_irq(PERIOD_US * 2000);
        if (crc_byte < 0) {
            std::cout << "Packet " << seq << " timeout reading CRC\n"; 
            continue;
        }

        if ((int)data.size() != len) {
            std::cout << "Packet " << seq << " length mismatch (got " << data.size() << " exp " << (int)len << ")\n"; 
            continue;
        }

        // Verify CRC over seq_hi, seq_lo, len_hi, len_lo, raw_payload
        // This must match what TX computed in send_packet().
        std::vector<uint8_t> crc_input;
        crc_input.push_back(seq_hi);
        crc_input.push_back(rest[0]); // seq_lo
        crc_input.push_back(rest[1]); // len_hi
        crc_input.push_back(rest[2]); // len_lo
        crc_input.insert(crc_input.end(), data.begin(), data.end());
        uint8_t expected_crc = crc8(crc_input.data(), crc_input.size());

        if ((uint8_t)crc_byte != expected_crc) {
            std::cout << "Packet " << seq << " CRC fail (got 0x" << std::hex << crc_byte << " exp 0x" << (int)expected_crc << std::dec << ")\n";
            // Flush and reset so we re-sync to the next 0xAA 0xFF marker.
            fwd_flush_queue();
            dec.state = DEC_IDLE;
            dec.bit_count = 0;
            dec.byte_val = 0;
            continue;
        }

        // CRC passed: write the payload directly into the receive buffer at correct offset for this sequence number.
        uint32_t offset = (uint32_t)seq * PACKET_SIZE;
        if (offset + len <= filesize) {
            for (int i = 0; i < len; ++i)
            {
                filedata[offset + i] = data[i];
            }
            received_seqs.insert(seq);
        }

        std::cout << "\rPacket " << seq << " OK  " << received_seqs.size() << "/" << total_packets << "   " << std::flush;
    }

    gpioSetAlertFunc(FWD_PIN, nullptr); // deregister interrupt before terminating
    gpioTerminate();
    return 0;
}
