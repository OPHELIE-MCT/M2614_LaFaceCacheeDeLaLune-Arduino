#pragma once

#include <Arduino.h>

class LDS_LDROBOT_LD19 {
   public:
    static constexpr uint32_t SERIAL_BAUD_RATE = 230400;
    static constexpr int SAMPLING_RATE_HZ = 5000;
    static constexpr float DEFAULT_SCAN_FREQ_HZ = 10.0f;

    struct ScanPoint {
        uint16_t angle_deg_x100;
        float angle_deg;
        uint16_t distance_mm;
        uint8_t quality;
        bool scan_completed;
    };

    struct PendingPoint {
        uint16_t angle_deg_x100;
        uint16_t distance_mm;
        uint8_t quality;
        bool scan_completed;
    };

    LDS_LDROBOT_LD19();

    void begin(Stream& serial);
    void reset();
    bool readPoint(ScanPoint& point);

    bool isStarted() const;
    float getCurrentScanFreqHz() const;

   private:
    static const uint8_t START_BYTE = 0x54;
    static const uint8_t POINTS_PER_PACK = 12;
    static const uint8_t VER_LEN = 0x2C;

    struct meas_sample_t {
        uint16_t distance_mm;
        uint8_t intensity;
    } __attribute__((packed));

    static const uint16_t DATA_BYTE_LEN = sizeof(meas_sample_t) * POINTS_PER_PACK;

    struct scan_packet_t {
        uint8_t start_byte;
        uint8_t ver_len;
        uint16_t speed_deg_per_sec;
        uint16_t start_angle_deg_x100;
        meas_sample_t sample[POINTS_PER_PACK];
        uint16_t end_angle_deg_x100;
        uint16_t timestamp_ms;
        uint8_t crc8;
    } __attribute__((packed));

    bool processByte(uint8_t c);
    void queuePacketPoints();
    uint16_t decodeUInt16(uint16_t value) const;
    uint8_t checkSum(uint8_t value, uint8_t crc) const;
    bool popPendingPoint(ScanPoint& point);

    Stream* serial_;
    bool started_;
    uint16_t speed_deg_per_sec_;
    scan_packet_t scan_packet_;
    uint16_t parser_idx_;
    uint8_t crc_;
    uint16_t end_angle_deg_x100_prev_;
    PendingPoint pending_points_[POINTS_PER_PACK];
    uint8_t pending_count_;
    uint8_t pending_index_;
};