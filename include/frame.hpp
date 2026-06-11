/**
 * @file frame.hpp
 * @brief Frame builder (TX) and parser (RX) with matched format
 *
 * Frame structure (bytes):
 *   [Sync: 8B] [Header: 12B] [Payload: variable] [Padding] [CRC32: 4B]
 *
 * Header (12 bytes):
 *   [0]     version(4b) | num_packets(4b)
 *   [1-4]   frame_number (uint32 big-endian)
 *   [5]     fec_rate(4b) | modulation(4b)
 *   [6]     stream_bitmap (bit per stream)
 *   [7-11]  reserved (zero)
 *
 * Payload packets (repeated):
 *   [stream_id: 1B] [length: 2B big-endian] [data: length bytes]
 *
 * The CRC covers everything after the sync word (header + payload + padding).
 */
#pragma once

#include "types.hpp"
#include <vector>

namespace gw {

// =========================================================================
// FrameBuilder (TX side)
// =========================================================================

class FrameBuilder {
public:
    /** @param capacity_bytes  Max payload bytes available after FEC overhead */
    explicit FrameBuilder(size_t capacity_bytes);

    /** Add a packet to the current frame.
     *  @return true if packet fit, false if frame is full */
    bool addPacket(uint8_t stream_id, const uint8_t* data, size_t len);

    /** Build the complete frame with sync, header, payload, CRC.
     *  @param frame_number  Sequence number for this frame
     *  @param fec_rate      FEC rate to encode in header
     *  @param modulation    Modulation to encode in header
     *  @return Complete frame bytes */
    ByteVec build(uint32_t frame_number, FECRate fec_rate, Modulation modulation);

    /** Reset for next frame */
    void reset();

    bool   hasData()     const { return !packets_.empty(); }
    size_t usedBytes()   const { return payload_used_; }
    size_t capacity()    const { return capacity_; }
    size_t freeBytes()   const { return (capacity_ > payload_used_) ? capacity_ - payload_used_ : 0; }

private:
    size_t capacity_;
    size_t payload_used_;

    struct Packet {
        uint8_t    stream_id;
        ByteVec    data;
    };
    std::vector<Packet> packets_;
};

// =========================================================================
// FrameParser (RX side)
// =========================================================================

struct ParsedFrame {
    bool valid      = false;
    bool crc_ok     = false;

    // Header fields
    uint8_t    version       = 0;
    uint8_t    num_packets   = 0;
    uint32_t   frame_number  = 0;
    FECRate    fec_rate      = FECRate::None;
    Modulation modulation    = Modulation::QPSK;
    uint8_t    stream_bitmap = 0;

    struct Packet {
        uint8_t stream_id;
        ByteVec data;
    };
    std::vector<Packet> packets;
};

class FrameParser {
public:
    /** Parse a frame from raw bytes.
     *  @param data  Complete frame bytes (sync + header + payload + CRC)
     *  @param out   Parsed frame output
     *  @return true if sync word found and header parseable */
    static bool parse(const uint8_t* data, size_t len, ParsedFrame& out);
    static bool parse(const ByteVec& data, ParsedFrame& out) {
        return parse(data.data(), data.size(), out);
    }
};

} // namespace gw
