// txDma.cpp: LiFi File Transmitter
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <set>
#include <string>
#include <chrono>
#include <csignal>
#include <pigpio.h>

// Constants:
static const int FWD_PIN = 17; // GPIO pin for laser driver
static const int BACK_PIN = 27; // GPIO pin for RX Pi (wire)
static const uint32_t PERIOD_US = 100; // base time unit for all pulses due to 10khz limit
/*
Start pulse is 3.5× the period: must be different then all other PWM pulses
from a '1' bit (2×) even with timing jitter on both ends.
*/
static const uint32_t START_PULSE_US = PERIOD_US * 3.5; // 350µs

/*
Each packet holds up to 512 bytes of file payload. Larger packets mean fewer headers and less overhead, but if a packet is
corrupted the whole chunk must be retransmitted.
*/
static const int PACKET_SIZE = 512;

// Back-channel message codes (always preceded by MSG_MAGIC as a framing byte)
static const uint8_t MSG_READY = 0x01; // RX is ready to receive
static const uint8_t MSG_NAK = 0x02; // RX is missing packet <seq>
static const uint8_t MSG_DONE = 0x03; // RX has finished reporting NAKs
static const uint8_t MSG_MAGIC = 0xBB; // framing/sync byte before every message

static volatile bool running = true;
void handle_sigint(int) 
{ 
    running = false; 
}

// CRC-8 Check sum:
/*
A single-byte checksum over components of packet.
Including the sequence number and length in the CRC.
Polynomial: x^8 + x^2 + x + 1  (0x07)
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

// Byte stuffing
/*
The packet framing uses 0xFF (header sync) and 0xFD (end-of-payload marker)
as special bytes. If those values appear inside the payload, they must be
escaped so the receiver doesn't mistake them for framing.

Encoding:
  0xFF to 0xFE 0x01
  0xFE to 0xFE 0x00     (escape byte itself must also be escaped)
  0xFD to 0xFE 0x02
*/
std::vector<unsigned char> stuff(const std::vector<unsigned char>& in)
{
    std::vector<unsigned char> out;
    out.reserve(in.size() + 8);   // pre-allocate with a little headroom for escapes
    for (unsigned char b : in)
    {
        if(b == 0xFF)
        {
            out.push_back(0xFE); 
            out.push_back(0x01); 
        }
        else if (b == 0xFE) 
        { 
            out.push_back(0xFE); 
            out.push_back(0x00); 
        }
        else if (b == 0xFD) 
        { 
            out.push_back(0xFE); 
            out.push_back(0x02); 
        }
        else 
        {
            out.push_back(b);
        }
    }
    return out;
}

// Forward channel TX (laser): PWM encoding
/*
Every symbol is a HIGH pulse followed by a fixed LOW gap:
  Start pulse: HIGH 3.5×PERIOD (350µs) + gap (50µs)
  Bit '1': HIGH 2×PERIOD (200µs) + gap (50µs)
  Bit '0': HIGH 1×PERIOD (100µs) + gap (50µs)

gap: (GAP_US = PERIOD_US/2 = 50µs)
*/
static const uint32_t GAP_US = PERIOD_US / 2; // 50µs: gap between symbols

std::vector<gpioPulse_t> pulses; // accumulates pulses for the current waveform

// Add a single data bit (0 or 1) to the pulse list.
void fwd_add_bit(int bit)
{
    uint32_t high_us = bit ? PERIOD_US * 2 : PERIOD_US; // 1=200µs, 0=100µs
    pulses.push_back({ (1u << FWD_PIN), 0, high_us }); // set HIGH
    pulses.push_back({ 0, (1u << FWD_PIN), GAP_US }); // set LOW
}

// Add a full byte to the pulse list: start pulse first, then 8 bits MSB first.
void fwd_add_byte(unsigned char byte)
{
    // Start pulse: lets RX know a byte is beginning
    pulses.push_back({ (1u << FWD_PIN), 0, START_PULSE_US });
    pulses.push_back({ 0, (1u << FWD_PIN), GAP_US });
    for (int i = 7; i >= 0; --i) // MSB first
    {
        fwd_add_bit((byte >> i) & 1);
    }
}

// Transmit all accumulated pulses via DMA, then clear the list.
void fwd_flush(uint32_t flush_wait_ms = 3)
{
    // Append a long LOW idle so RX can detect the end of the send
    pulses.push_back({ 0, (1u << FWD_PIN), PERIOD_US * 50 });

    // pigpio's waveform limit is around 12000 pulses.
    if (pulses.size() > 11900) {
        std::cerr << "Wave too large: " << pulses.size() << " pulses (limit ~12000) — packet dropped\n";
        pulses.clear();
        return;
    }

    gpioWaveClear();
    int added = gpioWaveAddGeneric(pulses.size(), pulses.data());
    if (added < 0) {
        std::cerr << "gpioWaveAddGeneric failed (" << pulses.size() << " pulses) — packet dropped\n";
        pulses.clear();
        return;
    }

    int wave_id = gpioWaveCreate();
    if (wave_id < 0) { std::cerr << "Wave create failed\n"; pulses.clear(); return; }

    gpioWaveTxSend(wave_id, PI_WAVE_MODE_ONE_SHOT); // start DMA transmission
    while (gpioWaveTxBusy()) 
    {
        gpioDelay(1000); // wait for DMA to finish
    }
    gpioDelay(flush_wait_ms * 1000);
    gpioWaveDelete(wave_id);
    pulses.clear();
}

// Session header
/*
Sent before any data or main packets: Need to tell RX the total file size, packet count, and
filename to set the receive buffer and name the output file.

Format (in big-endian):
    0xAA 0xFF 0x55 are sync bytes (0x55 distinguishes this from a data packet)
    filesize[31:24]: file length, 4 bytes
    filesize[23:16]
    filesize[15:8]
    filesize[7:0]
    total_packets[15:8]: packet count, 2 bytes
    total_packets[7:0]
    fname_len: filename length (0-255), 1 byte
    fname[0..fname_len-1]: filename
*/
void send_session_header(uint32_t filesize, uint16_t total_packets,
                         const std::string& filepath)
{
    std::string fname = filepath.substr(filepath.find_last_of("/\\") + 1);
    uint8_t fname_len = (uint8_t)std::min((int)fname.size(), 255);

    pulses.clear();
    fwd_add_byte( 0xAA ); // sync
    fwd_add_byte( 0xFF) ; // sync
    fwd_add_byte( 0x55 ); // packet type: session header
    fwd_add_byte( ( filesize >> 24 ) & 0xFF ); // file size
    fwd_add_byte( ( filesize >> 16 ) & 0xFF) ;
    fwd_add_byte( ( filesize >> 8 ) & 0xFF );
    fwd_add_byte( ( filesize >> 0 ) & 0xFF );
    fwd_add_byte( ( total_packets >> 8 ) & 0xFF ); // packet count
    fwd_add_byte( total_packets & 0xFF );
    fwd_add_byte( fname_len );
    for( int i = 0; i < fname_len; ++i )
    {
        fwd_add_byte( (unsigned char)fname[i] );
    }
    fwd_flush( 5 );
}

// Data packet: main data
/*
Format:
    0xAA 0xFF: sync bytes
    seq[15:8]: sequence number
    seq[7:0]
    len[15:8]: payload length
    len[7:0]
    stuffed payload: byte stuffed file data
    0xFD: end-of-payload marker
    crc8: checksum
*/
void send_packet(uint16_t seq, const std::vector<unsigned char>& filedata, int offset, int len)
{
    // Extract the raw chunk for this packet
    std::vector<unsigned char> raw(filedata.begin() + offset, filedata.begin() + offset + len);

    // Compute CRC over [seq_hi, seq_lo, len_hi, len_lo, raw_payload] 
    std::vector<uint8_t> crc_input;
    crc_input.push_back((seq >> 8) & 0xFF);
    crc_input.push_back(seq & 0xFF);
    crc_input.push_back((len >> 8) & 0xFF);
    crc_input.push_back(len & 0xFF);
    crc_input.insert(crc_input.end(), raw.begin(), raw.end());
    uint8_t check = crc8(crc_input.data(), crc_input.size());

    // Stuff the payload (escapes 0xFF, 0xFE, 0xFD)
    std::vector<unsigned char> stuffed = stuff(raw);

    pulses.clear();
    fwd_add_byte(0xAA); // sync
    fwd_add_byte(0xFF); // sync (0xFF signals data packet to RX)
    fwd_add_byte((seq >> 8) & 0xFF); // sequence number high byte
    fwd_add_byte(seq & 0xFF); // sequence number low byte
    fwd_add_byte((len >> 8) & 0xFF); // payload length high byte
    fwd_add_byte(len & 0xFF); // payload length low byte
    for (unsigned char b : stuffed)
    {
        fwd_add_byte(b); // stuffed payload bytes
    }
    fwd_add_byte(0xFD); // end-of-payload marker
    fwd_add_byte(check); // CRC-8
    fwd_flush(3);
}

// Marks the end of a transmission round. After receiving this, RX totals missing packets and sends back NAKs.
void send_end_marker()
{
    pulses.clear();
    fwd_add_byte(0xAA);
    fwd_add_byte(0xFF);
    fwd_add_byte(0xEE); // end-of-round marker
    fwd_flush(5);
}

// Back channel RX (wire): polling
/*
Uses the same PWM encoding as the forward channel but in reverse (wire from RX GPIO 27 to TX GPIO 27).

Decodes: samples by majority voting at three points around its center

Samples each bit at t: 25µs, t, t+25µs around t = center.
*/
unsigned char back_read_bits(uint32_t t_rise)
{
    // First bit centre = start of start pulse + start pulse duration + gap + half a bit width
    // = t_rise + START_PULSE_US + PERIOD_US + PERIOD_US/2
    // Using the '0' bit half-width as the half-symbol: PERIOD_US * 3/2 / 2 = PERIOD_US * 3/4?
    // Actually simplified: t_rise + (4 + 1 + 0.5) * PERIOD_US = t_rise + 5.5 * PERIOD_US
    // = t_rise + PERIOD_US * 11/2
    uint32_t centre = t_rise + PERIOD_US * 11/2;
    unsigned char byte = 0;
    for (int i = 7; i >= 0; --i) {
        // Three sample points spread across the bit window to majority-vote
        uint32_t t1 = centre - PERIOD_US/4;
        uint32_t t2 = centre;
        uint32_t t3 = centre + PERIOD_US/4;
        while (gpioTick() < t1)
        {}
        int s1 = gpioRead(BACK_PIN);
        while (gpioTick() < t2)
        {}
        int s2 = gpioRead(BACK_PIN);
        while (gpioTick() < t3)
        {}
        int s3 = gpioRead(BACK_PIN);

        // 2 of 3 majority vote= 1
        byte |= (((s1 + s2 + s3) >= 2 ? 1 : 0) << i);

        // Advance to next bit
        centre += PERIOD_US * 2;
    }
    return byte;
}

// Find valid start pulse on the back channel within timeout_us.
// A valid start pulse is high for duration of START_PULSE_US (350µs).
uint32_t back_hunt_start(uint32_t timeout_us)
{
    uint32_t deadline = gpioTick() + timeout_us;
    while (gpioTick() < deadline)
    {
        bool already_high = (gpioRead(BACK_PIN) == 1);
        uint32_t t_seen;

        if (already_high)
        {
            // mid-pulse
            t_seen = gpioTick();
        } 
        else {
            // Wait for the line to go HIGH
            while (gpioRead(BACK_PIN) == 0) 
            {
                if (gpioTick() > deadline) return 0;
            }
            t_seen = gpioTick();
        }

        // Wait for the pulse to end and measure width
        while (gpioRead(BACK_PIN) == 1)
        {
            if (gpioTick() > deadline)
            {
                return 0;
            }
        }
        uint32_t t_fall  = gpioTick();
        uint32_t visible = t_fall - t_seen;

        if (!already_high) 
        {
            // Full pulse check if width = start pulse
            if (visible >= PERIOD_US * 5 / 2 && visible <= PERIOD_US * 5){
                return t_seen;
            }
        } 
        else {
            // End of the pulse
            if (visible >= START_PULSE_US * 3 / 4)
                return t_fall - START_PULSE_US;
        }
        // If the pulse was too short or too long ignore and try again
    }
    return 0;
}

// Read one byte from the back channel. Returns the byte value or -1 on timeout.
int back_read_byte(uint32_t timeout_us)
{
    uint32_t t = back_hunt_start(timeout_us);
    if (t == 0) return -1;
    return back_read_bits(t);
}

// Handshake:
/*
TX cannot just fire one session header and hope RX was already listening.
If RX starts a late it will miss the header, so TX retransmits the session header every 2 seconds until it hears READY (0xBB 0x01) on the back channel. 
*/
bool wait_for_ready(uint32_t timeout_us, uint32_t filesize,
                    uint16_t total_packets, const std::string& filepath)
{
    uint32_t deadline = gpioTick() + timeout_us;
    while (gpioTick() < deadline) {
        send_session_header(filesize, total_packets, filepath);

        // After each header, listen for READY for up to 2 seconds
        uint32_t listen_deadline = gpioTick() + 2000000;
        while (gpioTick() < listen_deadline && gpioTick() < deadline)
        {
            int b = back_read_byte(listen_deadline - gpioTick());
            if (b != MSG_MAGIC) continue;
            b = back_read_byte(PERIOD_US * 1000);
            if (b == MSG_READY)
            {
                gpioDelay(500000); // give RX 500ms to finish initialising
                return true;
            }
        }
    }
    return false; // timed out without hearing READY
}

// NAK collection
/*
After TX sends the end marker, RX sends zero or more NAK messages, then DONE.
Each NAK formated as: 0xBB 0x02 seq_hi seq_lo
DONE is: 0xBB 0x03
*/
std::set<uint16_t> read_naks(uint32_t timeout_us)
{
    std::set<uint16_t> naks;
    uint32_t deadline = gpioTick() + timeout_us;
    while (gpioTick() < deadline)
    {
        int b = back_read_byte(deadline - gpioTick());
        if (b < 0)
        {
            break;
        }
        if (b != MSG_MAGIC)
        {
            continue; // skip anything thats not a framed message
        }
        b = back_read_byte(PERIOD_US * 1000);
        if (b < 0)
        {
            break;
        }
        if (b == MSG_DONE)
        {
            break; // RX finished listing NAKs
        }
        if (b == MSG_NAK) {
            int hi = back_read_byte(PERIOD_US * 1000);
            int lo = back_read_byte(PERIOD_US * 1000);
            if (hi >= 0 && lo >= 0) {
                uint16_t seq = ((uint16_t)hi << 8) | lo;
                naks.insert(seq);
            }
        }
    }
    return naks;
}

// Main

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file>\n";
        return 1;
    }

    // Load the entire file into memory upfront..
    std::ifstream file(argv[1], std::ios::binary);
    if (!file)
    { 
        std::cerr << "Cannot open: " << argv[1] << "\n";
        return 1; 
    }
    std::vector<unsigned char> filedata( (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    uint32_t filesize = filedata.size();
    uint16_t total_packets = (filesize + PACKET_SIZE - 1) / PACKET_SIZE; // ceiling division

    std::cout << "File: " << argv[1] << " (" << filesize << " bytes, " << total_packets << " packets)\n";

    // Initialise pigpio this sets up the DMA engine and claims GPIO resources.
    // Must be run as root.
    if (gpioInitialise() < 0) 
    {
        std::cerr << "pigpio init failed\n";
        return 1;
    }
    gpioSetMode(FWD_PIN,  PI_OUTPUT); 
    gpioWrite(FWD_PIN, 0); // laser off
    gpioSetMode(BACK_PIN, PI_INPUT);
    gpioSetPullUpDown(BACK_PIN, PI_PUD_DOWN);
    std::signal(SIGINT, handle_sigint);

    // Handshake: keep sending session header until RX replies with READY.
    // Timeout after 30 seconds
    if (!wait_for_ready(30000000, filesize, total_packets, argv[1]))
    {
        std::cerr << "Timed out waiting for READY\n";
        gpioTerminate(); return 1;
    }

    auto t_start = std::chrono::steady_clock::now();

    // to_send starts as all packets. After each round, it becomes whatever what RX sent as the NAK. When RX reports no missing packets the transfer is complete.
    std::set<uint16_t> to_send;
    for (uint16_t i = 0; i < total_packets; ++i)
    {
        to_send.insert(i);
    }

    int round = 1;
    
    while (!to_send.empty() && running)
    {
        std::cout << "Round " << round++ << ": " << to_send.size() << " packets\n";

        // Send every packet in this round in sequence-number order
        int count = 0;
        for (uint16_t seq : to_send)
        {
            int offset = seq * PACKET_SIZE;
            int len    = std::min((int)PACKET_SIZE, (int)filesize - offset);
            send_packet(seq, filedata, offset, len);
            std::cout << "\r  " << ++count << "/" << to_send.size() << "  seq=" << seq << "   " << std::flush;
        }
        std::cout << "\n";

        // Signal end of round RX will now compute missing packets.
        send_end_marker();

        // Collect NAKs
        to_send.clear();
        std::set<uint16_t> naks = read_naks(10000000);
        to_send = naks;

        if (to_send.empty()) {
            // RX sent DONE with no NAKs and file received successfully
            auto t_end  = std::chrono::steady_clock::now();
            double secs = std::chrono::duration<double>(t_end - t_start).count();
            double rate = (filesize / 1024.0) / secs;
            std::cout << "Transfer complete in " << std::fixed << std::setprecision(1) << secs << "s  (" << std::setprecision(2) << rate << " KB/s)\n";
        } else {
            std::cout << to_send.size() << " packets to resend\n";
            gpioDelay(500000); // brief pause before next round
        }
    }

    gpioWrite(FWD_PIN, 0);// Turn laser off
    gpioTerminate();
    return 0;
}
