/**
 * @file frame_test.cpp
 * @brief Verify frame builder/parser round-trip (the v1 bug that broke everything)
 */

#include "frame.hpp"
#include <cstdio>
#include <cstring>
#include <cassert>

using namespace dsca;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) printf("  %-50s", name)
#define PASS() do { printf("[PASS]\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("[FAIL] %s\n", msg); tests_failed++; } while(0)

int main() {
    printf("=== Frame Round-Trip Test ===\n\n");

    // Test 1: Empty frame
    TEST("Empty frame round-trip");
    {
        FrameBuilder builder(256);
        ByteVec frame = builder.build(0, FECRate::Rate_1_2, Modulation::QAM16);

        ParsedFrame parsed;
        bool ok = FrameParser::parse(frame, parsed);
        if (ok && parsed.valid && parsed.crc_ok && parsed.num_packets == 0 &&
            parsed.frame_number == 0 &&
            parsed.fec_rate == FECRate::Rate_1_2 &&
            parsed.modulation == Modulation::QAM16) {
            PASS();
        } else {
            FAIL("parse failed or fields mismatch");
        }
    }

    // Test 2: Single packet
    TEST("Single packet round-trip");
    {
        FrameBuilder builder(256);
        uint8_t payload[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE };
        bool added = builder.addPacket(3, payload, sizeof(payload));
        ByteVec frame = builder.build(42, FECRate::Rate_3_4, Modulation::QAM64);

        ParsedFrame parsed;
        bool ok = FrameParser::parse(frame, parsed);
        if (!ok || !parsed.valid || !parsed.crc_ok) {
            FAIL("parse/crc failed");
        } else if (parsed.frame_number != 42) {
            FAIL("frame number mismatch");
        } else if (parsed.packets.size() != 1) {
            FAIL("packet count mismatch");
        } else if (parsed.packets[0].stream_id != 3) {
            FAIL("stream_id mismatch");
        } else if (parsed.packets[0].data.size() != sizeof(payload)) {
            FAIL("data length mismatch");
        } else if (memcmp(parsed.packets[0].data.data(), payload, sizeof(payload)) != 0) {
            FAIL("data content mismatch");
        } else if (!added) {
            FAIL("addPacket returned false");
        } else {
            PASS();
        }
    }

    // Test 3: Multiple packets
    TEST("Multiple packets round-trip");
    {
        FrameBuilder builder(512);
        uint8_t p1[] = { 1, 2, 3 };
        uint8_t p2[] = { 10, 20, 30, 40, 50 };
        uint8_t p3[] = { 0xFF };

        builder.addPacket(0, p1, sizeof(p1));
        builder.addPacket(2, p2, sizeof(p2));
        builder.addPacket(7, p3, sizeof(p3));

        ByteVec frame = builder.build(100, FECRate::Rate_2_3, Modulation::QAM256);

        ParsedFrame parsed;
        bool ok = FrameParser::parse(frame, parsed);
        if (!ok || !parsed.valid || !parsed.crc_ok) {
            FAIL("parse/crc failed");
        } else if (parsed.packets.size() != 3) {
            char msg[64];
            snprintf(msg, sizeof(msg), "got %zu packets, expected 3",
                     parsed.packets.size());
            FAIL(msg);
        } else if (parsed.packets[0].stream_id != 0 ||
                   parsed.packets[1].stream_id != 2 ||
                   parsed.packets[2].stream_id != 7) {
            FAIL("stream_id mismatch");
        } else if (parsed.packets[1].data.size() != 5 ||
                   parsed.packets[1].data[3] != 40) {
            FAIL("data content mismatch");
        } else if (parsed.stream_bitmap != ((1<<0)|(1<<2)|(1<<7))) {
            FAIL("bitmap mismatch");
        } else {
            PASS();
        }
    }

    // Test 4: CRC corruption detection
    TEST("CRC corruption detected");
    {
        FrameBuilder builder(128);
        uint8_t p[] = { 42 };
        builder.addPacket(0, p, 1);
        ByteVec frame = builder.build(1, FECRate::Rate_1_2, Modulation::QPSK);

        // Corrupt one byte in payload area
        if (frame.size() > 25) {
            frame[25] ^= 0xFF;
        }

        ParsedFrame parsed;
        FrameParser::parse(frame, parsed);
        if (parsed.valid && !parsed.crc_ok) {
            PASS();
        } else {
            FAIL("corruption not detected");
        }
    }

    // Test 5: Capacity overflow
    TEST("Capacity overflow rejected");
    {
        FrameBuilder builder(10); // only 10 bytes capacity
        uint8_t big[100];
        memset(big, 0xAA, sizeof(big));
        bool ok = builder.addPacket(0, big, sizeof(big));
        if (!ok) {
            PASS();
        } else {
            FAIL("should have rejected oversized packet");
        }
    }

    // Test 6: Header field encoding
    TEST("All header fields preserved");
    {
        FrameBuilder builder(64);
        ByteVec frame = builder.build(0xDEADBEEF, FECRate::Rate_8_9, Modulation::QAM1024);

        ParsedFrame parsed;
        FrameParser::parse(frame, parsed);
        if (parsed.valid && parsed.crc_ok &&
            parsed.frame_number == 0xDEADBEEF &&
            parsed.fec_rate == FECRate::Rate_8_9 &&
            parsed.modulation == Modulation::QAM1024 &&
            parsed.version == 2) {
            PASS();
        } else {
            FAIL("header field mismatch");
        }
    }

    TEST("CRC-32/BZIP2 known-answer (\"123456789\" == 0xFC891918)");
    {
        const char* s = "123456789";
        uint32_t c = dsca::crc32(reinterpret_cast<const uint8_t*>(s), 9);
        if (c == 0xFC891918u) {
            PASS();
        } else {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "got 0x%08X (want 0xFC891918)", c);
            FAIL(buf);
        }
    }

    printf("\n=== Results: %d passed, %d failed ===\n",
           tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
