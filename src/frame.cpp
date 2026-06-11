/**
 * @file frame.cpp
 * @brief Frame builder (TX) and parser (RX) — matched format
 *
 * v1 bug fix: TX and RX now use identical packet format:
 *   [stream_id: 1B] [length: 2B big-endian] [data: length bytes]
 */

#include "frame.hpp"
#include <cstring>
#include <algorithm>

namespace gw {

// =========================================================================
// FrameBuilder
// =========================================================================

FrameBuilder::FrameBuilder(size_t capacity_bytes)
    : capacity_(capacity_bytes), payload_used_(0) {}

bool FrameBuilder::addPacket(uint8_t stream_id, const uint8_t* data, size_t len) {
    // The header's num_packets field is 4 bits, so the parser only reads
    // up to 15 packets. Reject the 16th rather than serializing a packet
    // build() emits but FrameParser silently drops. (Production caps at 8
    // streams, so this is a latent guard for direct API callers.) (#30)
    if (packets_.size() >= 15) return false;
    // Each packet: 1 (stream_id) + 2 (length) + len (data)
    size_t needed = 3 + len;
    if (payload_used_ + needed > capacity_) return false;
    if (len > 65535) return false; // 2-byte length limit

    Packet pkt;
    pkt.stream_id = stream_id;
    pkt.data.assign(data, data + len);
    packets_.push_back(std::move(pkt));
    payload_used_ += needed;

    return true;
}

ByteVec FrameBuilder::build(uint32_t frame_number, FECRate fec_rate,
                             Modulation modulation) {
    ByteVec frame;
    frame.reserve(constants::FRAME_OVERHEAD + capacity_);

    // --- Sync word (8 bytes) ---
    uint64_t sync = constants::SYNC_PATTERN;
    for (int i = 7; i >= 0; --i) {
        frame.push_back(static_cast<uint8_t>((sync >> (i * 8)) & 0xFF));
    }

    // --- Header (12 bytes) ---
    // [0] version(4b) | num_packets(4b)
    uint8_t npkt = static_cast<uint8_t>(std::min(packets_.size(), size_t(15)));
    frame.push_back(static_cast<uint8_t>(0x20 | (npkt & 0x0F)));

    // [1-4] frame_number big-endian
    frame.push_back(static_cast<uint8_t>((frame_number >> 24) & 0xFF));
    frame.push_back(static_cast<uint8_t>((frame_number >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((frame_number >> 8)  & 0xFF));
    frame.push_back(static_cast<uint8_t>( frame_number        & 0xFF));

    // [5] fec_rate(4b) | modulation(4b)
    frame.push_back(static_cast<uint8_t>(
        ((static_cast<uint8_t>(fec_rate) & 0x0F) << 4) |
         (static_cast<uint8_t>(modulation) & 0x0F)));

    // [6] stream_bitmap
    uint8_t bitmap = 0;
    for (auto& p : packets_) {
        if (p.stream_id < 8) bitmap |= (1u << p.stream_id);
    }
    frame.push_back(bitmap);

    // [7-11] reserved
    for (int i = 0; i < 5; ++i) frame.push_back(0);

    // --- Payload packets ---
    for (auto& pkt : packets_) {
        frame.push_back(pkt.stream_id);
        uint16_t len = static_cast<uint16_t>(pkt.data.size());
        frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>( len       & 0xFF));
        frame.insert(frame.end(), pkt.data.begin(), pkt.data.end());
    }

    // --- Pad to capacity + overhead ---
    size_t target = constants::SYNC_BYTES + constants::HEADER_BYTES
                    + capacity_ + constants::CRC_BYTES;
    while (frame.size() < target - constants::CRC_BYTES) {
        frame.push_back(0);
    }

    // --- CRC32 over header + payload + padding (skip sync word) ---
    const uint8_t* crc_start = frame.data() + constants::SYNC_BYTES;
    size_t crc_len = frame.size() - constants::SYNC_BYTES;
    uint32_t crc = gw::crc32(crc_start, crc_len);

    frame.push_back(static_cast<uint8_t>((crc >> 24) & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8)  & 0xFF));
    frame.push_back(static_cast<uint8_t>( crc        & 0xFF));

    return frame;
}

void FrameBuilder::reset() {
    packets_.clear();
    payload_used_ = 0;
}

// =========================================================================
// FrameParser
// =========================================================================

bool FrameParser::parse(const uint8_t* data, size_t len, ParsedFrame& out) {
    out = ParsedFrame{};

    if (len < constants::FRAME_OVERHEAD) return false;

    // --- Validate sync word ---
    uint64_t sync = 0;
    for (int i = 0; i < 8; ++i) {
        sync = (sync << 8) | static_cast<uint64_t>(data[i]);
    }
    if (sync != constants::SYNC_PATTERN) return false;

    size_t off = constants::SYNC_BYTES; // 8

    // --- Parse header ---
    out.version     = (data[off] >> 4) & 0x0F;
    out.num_packets = data[off] & 0x0F;
    off++;

    out.frame_number = (static_cast<uint32_t>(data[off])   << 24) |
                       (static_cast<uint32_t>(data[off+1]) << 16) |
                       (static_cast<uint32_t>(data[off+2]) << 8)  |
                        static_cast<uint32_t>(data[off+3]);
    off += 4;

    out.fec_rate    = static_cast<FECRate>((data[off] >> 4) & 0x0F);
    out.modulation  = static_cast<Modulation>(data[off] & 0x0F);
    off++;

    out.stream_bitmap = data[off];
    off++;

    off += 5; // skip reserved

    // --- Verify CRC (last 4 bytes) ---
    if (len >= constants::SYNC_BYTES + constants::CRC_BYTES) {
        uint32_t received_crc =
            (static_cast<uint32_t>(data[len-4]) << 24) |
            (static_cast<uint32_t>(data[len-3]) << 16) |
            (static_cast<uint32_t>(data[len-2]) << 8)  |
             static_cast<uint32_t>(data[len-1]);

        const uint8_t* crc_start = data + constants::SYNC_BYTES;
        size_t crc_len = len - constants::SYNC_BYTES - constants::CRC_BYTES;
        uint32_t computed_crc = gw::crc32(crc_start, crc_len);

        out.crc_ok = (received_crc == computed_crc);
    }

    // --- Parse packets (only if CRC ok) ---
    if (out.crc_ok) {
        for (uint8_t p = 0; p < out.num_packets && off + 3 <= len - constants::CRC_BYTES; ++p) {
            uint8_t sid = data[off++];
            uint16_t plen = (static_cast<uint16_t>(data[off]) << 8) |
                             static_cast<uint16_t>(data[off+1]);
            off += 2;

            if (off + plen > len - constants::CRC_BYTES) break;

            ParsedFrame::Packet pkt;
            pkt.stream_id = sid;
            pkt.data.assign(data + off, data + off + plen);
            out.packets.push_back(std::move(pkt));
            off += plen;
        }
    }

    out.valid = true;
    return true;
}

} // namespace gw
