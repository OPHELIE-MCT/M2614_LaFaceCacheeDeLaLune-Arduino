#include "LDS_LDROBOT_LD19.h"

LDS_LDROBOT_LD19::LDS_LDROBOT_LD19()
    : serial_(nullptr),
      started_(false),
      speed_deg_per_sec_(0),
      parser_idx_(0),
      crc_(0),
      end_angle_deg_x100_prev_(0),
      pending_count_(0),
      pending_index_(0) {
}

void LDS_LDROBOT_LD19::begin(Stream& serial) {
    serial_ = &serial;
    started_ = true;
    reset();
}

void LDS_LDROBOT_LD19::reset() {
    speed_deg_per_sec_ = 0;
    parser_idx_ = 0;
    crc_ = 0;
    end_angle_deg_x100_prev_ = 0;
    pending_count_ = 0;
    pending_index_ = 0;
}

bool LDS_LDROBOT_LD19::isStarted() const {
    return started_ && serial_ != nullptr;
}

float LDS_LDROBOT_LD19::getCurrentScanFreqHz() const {
    static constexpr float ONE_OVER_360 = 1.0f / 360.0f;
    return ONE_OVER_360 * speed_deg_per_sec_;
}

bool LDS_LDROBOT_LD19::popPendingPoint(ScanPoint& point) {
    if (pending_index_ >= pending_count_) {
        pending_index_ = 0;
        pending_count_ = 0;
        return false;
    }

    const PendingPoint& pending_point = pending_points_[pending_index_++];
    point.angle_deg_x100 = pending_point.angle_deg_x100;
    point.angle_deg = pending_point.angle_deg_x100 * 0.01f;
    point.distance_mm = pending_point.distance_mm;
    point.quality = pending_point.quality;
    point.scan_completed = pending_point.scan_completed;

    if (pending_index_ >= pending_count_) {
        pending_index_ = 0;
        pending_count_ = 0;
    }

    return true;
}

bool LDS_LDROBOT_LD19::readPoint(ScanPoint& point) {
    if (popPendingPoint(point)) {
        return true;
    }

    if (!isStarted()) {
        return false;
    }

    while (true) {
        int c = serial_->read();
        if (c < 0) {
            return false;
        }

        if (processByte((uint8_t)c) && popPendingPoint(point)) {
            return true;
        }
    }
}

uint8_t LDS_LDROBOT_LD19::checkSum(uint8_t value, uint8_t crc) const {
    static const uint8_t CRC_TABLE[256] = {
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
        0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
        0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
        0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
        0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
        0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
        0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
        0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
        0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
        0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
        0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
        0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
        0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
        0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
        0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
        0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
        0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
        0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
        0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
        0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
        0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
        0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};
    return CRC_TABLE[(crc ^ value) & 0xff];
}

bool LDS_LDROBOT_LD19::processByte(uint8_t c) {
    uint8_t* rx_buffer = reinterpret_cast<uint8_t*>(&scan_packet_);

    if (parser_idx_ >= sizeof(scan_packet_t)) {
        parser_idx_ = 0;
        return false;
    }

    if (parser_idx_ == 0) {
        crc_ = 0;
    }

    rx_buffer[parser_idx_++] = c;
    if (parser_idx_ < sizeof(scan_packet_t)) {
        crc_ = checkSum(c, crc_);
    }

    switch (parser_idx_) {
        case 1:
            if (c != START_BYTE) {
                parser_idx_ = 0;
            }
            break;

        case 2:
            if (c != VER_LEN) {
                parser_idx_ = 0;
            }
            break;

        case 3:
        case 4:
        case 5:
        case 6:
        case 43:
        case 44:
        case 45:
        case 46:
            break;

        case 47: {
            if (crc_ == scan_packet_.crc8) {
                queuePacketPoints();
                parser_idx_ = 0;
                return pending_count_ > 0;
            }

            parser_idx_ = 0;
            break;
        }

        default:
            if (parser_idx_ <= (6 + DATA_BYTE_LEN)) {
                break;
            }
            parser_idx_ = 0;
            break;
    }

    return false;
}

void LDS_LDROBOT_LD19::queuePacketPoints() {
    speed_deg_per_sec_ = decodeUInt16(scan_packet_.speed_deg_per_sec);
    uint16_t start_angle_deg_x100 = decodeUInt16(scan_packet_.start_angle_deg_x100);
    uint16_t end_angle_deg_x100 = decodeUInt16(scan_packet_.end_angle_deg_x100);

    bool scan_completed_mid_packet = end_angle_deg_x100 < start_angle_deg_x100;
    bool scan_completed_between_packets = start_angle_deg_x100 < end_angle_deg_x100_prev_;
    end_angle_deg_x100_prev_ = end_angle_deg_x100;

    float start_angle = start_angle_deg_x100 * 0.01f;
    float end_angle = end_angle_deg_x100 * 0.01f;

    if (end_angle < start_angle) {
        end_angle += 360.0f;
    }

    static constexpr float ONE_OVER_POINTS_MINUS_ONE = 1.0f / (POINTS_PER_PACK - 1);
    float step_angle = (end_angle - start_angle) * ONE_OVER_POINTS_MINUS_ONE;
    float angle_deg_prev = start_angle;

    pending_count_ = POINTS_PER_PACK;
    pending_index_ = 0;

    for (uint8_t i = 0; i < POINTS_PER_PACK; i++) {
        float angle_deg = start_angle + step_angle * i;
        bool scan_completed = false;

        if (scan_completed_mid_packet) {
            scan_completed = (angle_deg >= 360.0f && angle_deg_prev < 360.0f);
        } else if (scan_completed_between_packets) {
            scan_completed = (i == 0);
        }

        if (angle_deg >= 360.0f) {
            angle_deg -= 360.0f;
        }

        uint16_t angle_deg_x100 = static_cast<uint16_t>(angle_deg * 100.0f + 0.5f);
        if (angle_deg_x100 >= 36000) {
            angle_deg_x100 -= 36000;
        }

        pending_points_[i] = {
            angle_deg_x100,
            decodeUInt16(scan_packet_.sample[i].distance_mm),
            scan_packet_.sample[i].intensity,
            scan_completed,
        };

        angle_deg_prev = angle_deg;
    }
}

uint16_t LDS_LDROBOT_LD19::decodeUInt16(uint16_t value) const {
    union {
        uint16_t i;
        char c[2];
    } bint = {0x0201};

    return bint.c[0] == 0x01 ? value : (value << 8) + (value >> 8);
}