/****************************************************************************
 *
 *   Copyright (c) 2025 HavocAI. All rights reserved.
 *
 ****************************************************************************/

#include "AP_NMEA2K_msg.h"

#include <cmath>

namespace nmea2k {

template <typename T>
void SetBuf(T v, size_t len, uint8_t& index, uint8_t* buf) {
  // This could be improved by casting the buffer to a pointer of T and
  // doing a direct copy. That is, if unaligned data access is allowed.
  memcpy(&buf[index], &v, len);
  index += len;
}

N2KMessage::N2KMessage(uint8_t source, uint8_t priority, uint32_t pgn,
                       uint8_t length)
    : priority_(priority),
      pgn_(pgn),
      msg_source_(source),
      data_length_(length) {}

void N2KMessage::SetPriority(uint8_t priority) {
  if (priority <= 7) {
    priority_ = priority;

  } else {
    // Handle invalid priority value
    priority_ = 6;  // Default to 6 if invalid as that's the default
  }
}

void N2KMessage::SetPGN(uint32_t pgn) { pgn_ = pgn; }

void N2KMessage::SetDestination(uint8_t destination) {
  if (destination <= 251) {
    msg_destination_ = destination;

  } else {
    // Handle invalid destination value
    msg_destination_ = 255;  // Default to global address if invalid
  }
}

void N2KMessage::SetDataLength(uint8_t length) {
  if (length <= MAX_DATA_SIZE) {
    data_length_ = length;

  } else {
    // Handle invalid length value
    data_length_ = MAX_DATA_SIZE;  // Default to max size if invalid
  }
}

void N2KMessage::ManualSetSource(uint8_t source) {
  if (source <= 251) {
    msg_source_ = source;

  } else {
    // Handle invalid source value
    msg_source_ = 0;  // Default to 0 if invalid as that's the default
  }
}

void N2KMessage::SetDataFromSingleFrame(uint8_t msg_start_index,
                                        uint8_t buf_start_index, uint8_t length,
                                        uint8_t* buffer) {
  if (msg_start_index + length > MAX_DATA_SIZE) {
    // Handle invalid length value
    return;  // Do not set data if it exceeds max size
  }

  // Copy data from the buffer starting at the specified index, into the data
  // array
  std::memcpy(data_ + msg_start_index, buffer + buf_start_index, length);
}

uint8_t N2KMessage::data_length() const { return data_length_; }

uint32_t N2KMessage::pgn() const { return pgn_; }

uint8_t N2KMessage::source() const { return msg_source_; }

uint8_t N2KMessage::destination() const { return msg_destination_; }

uint8_t N2KMessage::priority() const { return priority_; }

const uint8_t* N2KMessage::DataPtrForUnpack() const { return data_; }

void N2KMessage::clear() {
  data_length_ = 0;
  std::memset(data_, 0, sizeof(data_));
  msg_source_ = 0;
  msg_destination_ = 0;
  pgn_ = 0;
  priority_ = 6;  // Default priority
}

uint32_t N2KMessage::FormatToCanId() const {
  // PGN format:
  // 1 bit rsvd | 1 bit data page | 8 bits PDU format (PF) | 8 bits PDU specific
  // (PS)

  // Check PDU format, PDU1 or PDU2. Shift by PS portion
  if (static_cast<uint8_t>(pgn_ >> 8) < 240) {
    // PDU1 Format, include destination
    return (priority_ << 26) | ((pgn_ & 0x3ffff) << 8) |
           (msg_destination_ << 8) | (msg_source_);

  } else {
    // PDU2 Format, no specific destination
    return (priority_ << 26) | ((pgn_ & 0x3ffff) << 8) | (msg_source_);
  }
}

void N2KMessage::SetInfoFromCanId(uint32_t can_id) {
  // Decompose the CAN ID into its components
  priority_ = (can_id >> 26) & 0x7;  // 3 bits for priority
  msg_source_ = can_id & 0xFF;       // last 8 bits are source

  pgn_ = (can_id >> 8) & 0x3FFFF;  // 18 bits for PGN

  // Check PDU format
  if (static_cast<uint8_t>(pgn_ >> 8) < 240) {
    // For PDU1, destination is the PS field of the PGN
    msg_destination_ = (can_id >> 8) & 0xFF;
    pgn_ = (can_id >> 8) & 0x3FF00;  // shift by source to get PGN, not extended

  } else {
    msg_destination_ = 0xff;         // No specific destination
    pgn_ = (can_id >> 8) & 0x3FFFF;  // shift by source to get PGN
  }
}

void N2KMessage::ForceCheckDestination() {
  if ((pgn_ & 0xff) != 0) {
    msg_destination_ = 0xff;
  }
}

void N2KMessage::CopyDataToBuffer(uint8_t* buffer, size_t buffer_size,
                                  size_t data_size) const {
  if (buffer_size > MAX_DATA_SIZE || data_size > MAX_DATA_SIZE) {
    // Handle invalid size value
    return;
  }

  // Clear the buffer before copying
  std::memset(buffer, 0, buffer_size);
  std::memcpy(buffer, data_, data_size);
}

void N2KMessage::SetDataFromPack(const uint8_t* packed_data, size_t size) {
  if (size > MAX_DATA_SIZE) {
    // Handle invalid size value
    return;
  }

  std::memcpy(data_, packed_data, size);
  data_length_ = static_cast<uint8_t>(size);
}

void N2KMessage::AddByte(uint8_t byte) { data_[data_length_++] = byte; }

void N2KMessage::Add2ByteInt(int16_t value) {
  SetBuf(value, 2, data_length_, data_);
}

void N2KMessage::Add2ByteUInt(uint16_t value) {
  SetBuf(value, 2, data_length_, data_);
}

void N2KMessage::Add3ByteInt(int32_t value) {
  SetBuf(value, 3, data_length_, data_);
}

void N2KMessage::Add3ByteUInt(uint32_t value) {
  SetBuf(value, 3, data_length_, data_);
}

void N2KMessage::Add4ByteInt(int32_t value) {
  SetBuf(value, 4, data_length_, data_);
}

void N2KMessage::Add4ByteUInt(uint32_t value) {
  SetBuf(value, 4, data_length_, data_);
}

void N2KMessage::Add8ByteUint(uint64_t value) {
  SetBuf(value, 8, data_length_, data_);
}

void N2KMessage::Add8ByteInt(int64_t value) {
  SetBuf(value, 8, data_length_, data_);
}

void N2KMessage::AddFloat(float value, float undefined_value) {
  if (value >= undefined_value) {
    Add4ByteUInt(kNotAvailable_Int32);

  } else {
    int32_t value_integer;
    memcpy(&value_integer, &value, 4);

    SetBuf(value_integer, 4, data_length_, data_);
  }
}

void N2KMessage::Add1ByteDouble(double value, double precision,
                                double undefined_value) {
  if (value >= undefined_value) {
    // Fill with the "not accessible" value
    Add2ByteUInt(kNotAvailable_Int8);

  } else {
    double value_double = round(value / precision);

    int8_t value_integer = (value_double >= kOutOfRangeLower_Int8 &&
                            value_double < kOutOfRangeUpper_Int8)
                               ? static_cast<int8_t>(value_double)
                               : kOutOfRangeUpper_Int8;

    SetBuf(value_integer, 1, data_length_, data_);
  }
}

void N2KMessage::Add1ByteUDouble(double value, double precision,
                                 double undefined_value) {
  if (value >= undefined_value) {
    // Fill with the "not accessible" value
    Add2ByteUInt(kNotAvailable_UInt8);

  } else {
    double value_double = round(value / precision);

    uint8_t value_integer =
        (value_double >= 0 && value_double < kOutOfRangeUpper_UInt8)
            ? static_cast<uint8_t>(value_double)
            : kOutOfRangeUpper_UInt8;

    SetBuf(value_integer, 1, data_length_, data_);
  }
}

void N2KMessage::Add2ByteDouble(double value, double precision,
                                double undefined_value) {
  if (value >= undefined_value) {
    // Fill with the "not accessible" value
    Add2ByteUInt(kNotAvailable_Int16);

  } else {
    double value_double = round(value / precision);

    int16_t value_integer = (value_double >= kOutOfRangeLower_Int16 &&
                             value_double < kOutOfRangeUpper_Int16)
                                ? static_cast<int16_t>(value_double)
                                : kOutOfRangeUpper_Int16;

    SetBuf(value_integer, 2, data_length_, data_);
  }
}

void N2KMessage::Add2ByteUDouble(double value, double precision,
                                 double undefined_value) {
  if (value >= undefined_value) {
    // Fill with the "not accessible" value
    Add2ByteUInt(kNotAvailable_UInt16);

  } else {
    double value_double = round(value / precision);

    uint16_t value_integer =
        (value_double >= 0 && value_double < kOutOfRangeUpper_UInt16)
            ? static_cast<uint16_t>(value_double)
            : kOutOfRangeUpper_UInt16;

    SetBuf(value_integer, 2, data_length_, data_);
  }
}

void N2KMessage::Add3ByteDouble(double value, double precision,
                                double undefined_value) {
  if (value >= undefined_value) {
    // Fill with the "not accessible" value
    Add3ByteUInt(kNotAvailable_Int24);

  } else {
    double value_double = round(value / precision);

    int32_t value_integer = (value_double >= kOutOfRangeLower_Int24 &&
                             value_double < kOutOfRangeUpper_Int24)
                                ? static_cast<int32_t>(value_double)
                                : kOutOfRangeUpper_Int24;

    SetBuf(value_integer, 3, data_length_, data_);
  }
}

void N2KMessage::Add3ByteUDouble(double value, double precision,
                                 double undefined_value) {
  if (value >= undefined_value) {
    // Fill with the "not accessible" value
    Add3ByteUInt(kNotAvailable_UInt24);

  } else {
    double value_double = round(value / precision);

    uint32_t value_integer =
        (value_double >= 0 && value_double < kOutOfRangeUpper_UInt24)
            ? static_cast<uint32_t>(value_double)
            : kOutOfRangeUpper_UInt24;

    SetBuf(value_integer, 3, data_length_, data_);
  }
}

void N2KMessage::Add4ByteDouble(double value, double precision,
                                double undefined_value) {
  if (value >= undefined_value) {
    // Fill with the "not accessible" value
    Add4ByteUInt(kNotAvailable_Int32);

  } else {
    double value_double = round(value / precision);

    int32_t value_integer = (value_double >= kOutOfRangeLower_Int32 &&
                             value_double < kOutOfRangeUpper_Int32)
                                ? static_cast<int32_t>(value_double)
                                : kOutOfRangeUpper_Int32;

    SetBuf(value_integer, 4, data_length_, data_);
  }
}

void N2KMessage::Add4ByteUDouble(double value, double precision,
                                 double undefined_value) {
  if (value >= undefined_value) {
    // Fill with the "not accessible" value
    Add4ByteUInt(kNotAvailable_UInt32);

  } else {
    double value_double = round(value / precision);

    uint32_t value_integer =
        (value_double >= 0 && value_double < kOutOfRangeUpper_UInt32)
            ? static_cast<uint32_t>(value_double)
            : kOutOfRangeUpper_UInt32;

    SetBuf(value_integer, 4, data_length_, data_);
  }
}

}  // namespace nmea2k