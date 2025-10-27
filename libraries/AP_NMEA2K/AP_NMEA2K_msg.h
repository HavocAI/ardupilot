/****************************************************************************
 *
 *   Copyright (c) 2025 HavocAI. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <cstring>

namespace nmea2k {

// Constants for the "not available" values for each type
static constexpr int8_t kNotAvailable_Int8 = 0x7F;
static constexpr uint8_t kNotAvailable_UInt8 = 0xFF;
static constexpr int16_t kNotAvailable_Int16 = 0x7FFF;
static constexpr uint16_t kNotAvailable_UInt16 = 0xFFFF;
static constexpr int32_t kNotAvailable_Int24 = 0x7FFFFF;
static constexpr uint32_t kNotAvailable_UInt24 = 0xFFFFFF;
static constexpr int32_t kNotAvailable_Int32 = 0x7FFFFFFF;
static constexpr int32_t kNotAvailable_UInt32 = 0xFFFFFFFF;
static constexpr uint64_t kNotAvailable_UInt64 = 0xFFFFFFFFFFFFFFFFUL;
static constexpr int64_t kNotAvailable_Int64 = 0x7FFFFFFFFFFFFFFFUL;
static constexpr double kNotAvailable_Double = -1e9;
static constexpr double kNotAvailable_Float = -1e9;

/**
 * This is for all data and message structure information related to an NMEA2000
 * message.
 *
 * A byte array is used to store the message data. The helper functions are used
 * to modify, append, and extact data both to and from the byte array.
 */

class N2KMessage {
  

 public:

  /**
   * Maximum size of the data array, which also represents the maximum size of a
   * message using fast packets. In the case of fast packet, the first frame can
   * have 6 bytes while the following 32 frames can have 7 bytes.
   */
  static const size_t MAX_DATA_SIZE = 223;

  /**
   * @brief Constructor for N2KMessage
   * @param source The source address of this message [0..251]
   * @param priority The message priority [0..7], default is 6
   * @param pgn The Parameter Group Number (PGN) [decimal]
   * @param length The message data length [0..223]
   */
  N2KMessage(uint8_t source = 0, uint8_t priority = 6, uint32_t pgn = 0,
             uint8_t length = 0);
  ~N2KMessage() = default;

  /**
   * @brief Set the message priority
   * @param priority The message priority [0..7], default is 6
   */
  void SetPriority(uint8_t priority);

  /**
   * @brief Set the Parameter Group Number (PGN)
   * @param pgn The Parameter Group Number (PGN) [decimal]
   * @note The PGN is a 24-bit number that identifies the type of data being
   * transmitted. It is used to determine how the data should be interpreted
   * and processed by the receiving device. The PGN is an important part of the
   * NMEA 2000 protocol, as it allows devices from different manufacturers to
   * communicate with each other and share data in a standardized way.
   */
  void SetPGN(uint32_t pgn);

  /**
   * @brief Set the destination address
   *
   * @param destination The destination address of this message [0..251]
   * @note  The destination address for messages where the PGN's PF (PFU Format)
   * section has a value equal to or greater than 240, then it's a broadcast
   * message, also known as a global message. There will be no specific addres
   * in the PS (PDU Specific) field of the PGN. If the PF is less than 240, then
   * the PS section will contain an address to a specific device destination.
   */
  void SetDestination(uint8_t destination);

  /**
   * @brief Set the data length for the message
   *
   * @param length The data length of the message [0..223]
   */
  void SetDataLength(uint8_t length);

  /**
   * @brief Set the source address
   *
   * @param source The source address of this message [0..251]
   */
  void ManualSetSource(uint8_t source);

  /**
   * @brief This is used to slowly recreate the message data for a message as it
   * comes over across the CAN bus. For single frame messages, this will be only
   * be called once to load in a single byte of data. However for fast-packets
   * that have more than 8 bytes of data and come across in multiple messages,
   * this will be called for each frame until the fast-packet is completed.
   *
   * For example, to copy in 5 bytes of data, starting at index 2 in the data
   * buffer (which is 0 indexed), into the message data at index 0, you would
   * call this function with: SetDataFromSingleFrame(2, 0, 5, buffer)
   *
   * If you wanted to copy in 5 bytes of data into index 12 of the message data,
   * from index 2 of the data buffer, you would call this function with:
   * SetDataFromSingleFrame(12, 2, 5, buffer);
   *
   * @param msg_start_index The index to start copying the data TO within the
   * current message data buffer
   * @param buf_start_index The index to start copying the data FROM within the
   * supplied buffer
   * @param length The length data to copy, starting from the buf_start_index
   * (i.e. how many bytes to copy)
   * @param buffer The pointer to the buffer containing the data that's being
   * copied
   */
  void SetDataFromSingleFrame(uint8_t msg_start_index, uint8_t buf_start_index,
                              uint8_t length, uint8_t* buffer);

  /**
   * @brief Set the message data based on a pointer to the data, which should of
   * been packed with one of the autogenerated "pack" functions
   *
   * This is what is generated from the DBC file, and is use to the pack the
   * data into the correct format. Data will not be packed if the size is
   * greater that the max data size. The data length will be set based on the
   * size of the packed data in the size passed through as a parameter.
   *
   * @param packed_data The pointer to the packed data
   * @param size The size of the packed data
   */
  void SetDataFromPack(const uint8_t* packed_data, size_t size);

  /**
   * @brief Get the message data pointer
   * @return The message data pointer
   */
  const uint8_t* DataPtrForUnpack() const;

  uint8_t* DataPtrForPack() { return data_; }

  /**
   * @brief Copy data into another provided buffer
   * @param buffer The buffer to copy the data into
   * @param size The size of the buffer
   * @note This will copy the data from the message data array into the
   * provided buffer. The size of the buffer must be at least the size of the
   * message data array, otherwise the data will be truncated.
   */
  void CopyDataToBuffer(uint8_t* buffer, size_t buffer_size,
                        size_t data_size) const;

  /**
   * @brief Get the message data length
   * @return The message data length
   */
  uint8_t data_length() const;

  /**
   * @brief Get the message PGN
   * @return The message PGN
   */
  uint32_t pgn() const;

  /**
   * @brief Get the message source address
   * @return The message source address
   */
  uint8_t source() const;

  /**
   * @brief Get the message destination address
   * @return The message destination address
   */
  uint8_t destination() const;

  uint8_t priority() const;

  /**
   * @brief Clears the contents of the message, resetting PGN, length and time
   * back to 0
   */
  void clear();

  /**
   * @brief Get the formatted CAN ID for the message
   * @return The formatted CAN ID, which is a 29-bit identifier
   */
  uint32_t FormatToCanId() const;

  /**
   * @brief Set the PGN/source/destination based on the
   * provided CAN ID
   * @param can_id The CAN ID to decompose
   */
  void SetInfoFromCanId(uint32_t can_id);
  /**
   * @brief Force the destination address to be checked, if the PGN is not a
   * broadcast message, then the destination address will be set to 0xFF
   */
  void ForceCheckDestination();

  /***************************************************************
   * Functions for setting the data in the message, if there's a custom
   *message to send that doesn't have pack/unpack generated from DBC
   ***************************************************************/

  /**
   * @brief Add a byte to the message data
   * @param byte The byte to add
   */
  void AddByte(uint8_t byte);

  /**
   * @brief Add a 2-byte signed integer to the message data
   * @param value The signed integer to add
   */
  void Add2ByteInt(int16_t value);

  static inline uint16_t ReadUInt16(const uint8_t* data)
  {
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
  }

  static inline uint32_t ReadUInt32(const uint8_t* data)
  {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
  }
  
  static inline int64_t ReadInt64(const uint8_t* data)
  {
    return static_cast<int64_t>(data[0]) |
           (static_cast<int64_t>(data[1]) << 8) |
           (static_cast<int64_t>(data[2]) << 16) |
           (static_cast<int64_t>(data[3]) << 24) |
           (static_cast<int64_t>(data[4]) << 32) |
           (static_cast<int64_t>(data[5]) << 40) |
           (static_cast<int64_t>(data[6]) << 48) |
           (static_cast<int64_t>(data[7]) << 56);
  }

  /**
   * @brief Add a 2-byte unsigned integer to the message data
   * @param value The unsigned integer to add
   */
  void Add2ByteUInt(uint16_t value);

  /**
   * @brief Add a 3-byte signed integer to the message data
   * @param value The signed integer to add
   */
  void Add3ByteInt(int32_t value);

  /**
   * @brief Add a 3-byte unsigned integer to the message data
   * @param value The unsigned integer to add
   */
  void Add3ByteUInt(uint32_t value);

  /**
   * @brief Add a 4-byte signed integer to the message data
   * @param value The signed integer to add
   */
  void Add4ByteInt(int32_t value);

  /**
   * @brief Add a 4-byte unsigned integer to the message data
   * @param value The unsigned integer to add
   */
  void Add4ByteUInt(uint32_t value);

  /**
   * @brief Add a 8-byte signed integer to the message data
   * @param value The signed integer to add
   */
  void Add8ByteUint(uint64_t value);

  /**
   * @brief Add a 8-byte signed integer to the message data
   * @param value The signed integer to add
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Float
   */
  void AddFloat(float value, float undefined_value = kNotAvailable_Float);

  /**
   * @brief Add a 1-byte signed double to the message data
   * @param value The signed double to add
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Double
   *
   * @note The precision is the value that is used to scale the double value
   *       before it is added to the message data as an integer.
   */
  void Add1ByteDouble(double value, double precision,
                      double undefined_value = kNotAvailable_Double);

  /**
   * @brief Add a 2-byte signed double to the message data
   * @param value The signed double to add
   * @param precision The precision of the double value
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Double
   *
   * @note The precision is the value that is used to scale the double value
   *       before it is added to the message data as an integer.
   */
  void Add2ByteDouble(double value, double precision,
                      double undefined_value = kNotAvailable_Double);

  /**
   * @brief Add a 3-byte signed double to the message data
   * @param value The signed double to add
   * @param precision The precision of the double value
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Double
   *
   * @note The precision is the value that is used to scale the double value
   *       before it is added to the message data as an integer.
   */
  void Add3ByteDouble(double value, double precision,
                      double undefined_value = kNotAvailable_Double);

  /**
   * @brief Add a 4-byte signed double to the message data
   * @param value The signed double to add
   * @param precision The precision of the double value
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Double
   *
   * @note The precision is the value that is used to scale the double value
   *       before it is added to the message data as an integer.
   */
  void Add4ByteDouble(double value, double precision,
                      double undefined_value = kNotAvailable_Double);

  /**
   * @brief Add a 8-byte signed double to the message data
   * @param value The signed double to add
   * @param precision The precision of the double value
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Double
   *
   * @note The precision is the value that is used to scale the double value
   *       before it is added to the message data as an integer.
   */
  void Add8ByteDouble(double value, double precision,
                      double undefined_value = kNotAvailable_Double);

  /**
   * @brief Add a 1-byte unsigned double to the message data
   * @param value The unsigned double to add
   * @param precision The precision of the double value
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Double
   *
   * @note The precision is the value that is used to scale the double value
   *       before it is added to the message data as an integer.
   */
  void Add1ByteUDouble(double value, double precision,
                       double undefined_value = kNotAvailable_Double);

  /**
   * @brief Add a 2-byte unsigned double to the message data
   * @param value The unsigned double to add
   * @param precision The precision of the double value
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Double
   *
   * @note The precision is the value that is used to scale the double value
   *       before it is added to the message data as an integer.
   */
  void Add2ByteUDouble(double value, double precision,
                       double undefined_value = kNotAvailable_Double);

  /**
   * @brief Add a 3-byte unsigned double to the message data
   * @param value The unsigned double to add
   * @param precision The precision of the double value
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Double
   *
   * @note The precision is the value that is used to scale the double value
   *       before it is added to the message data as an integer.
   */
  void Add3ByteUDouble(double value, double precision,
                       double undefined_value = kNotAvailable_Double);

  /**
   * @brief Add a 4-byte unsigned double to the message data
   * @param value The unsigned double to add
   * @param precision The precision of the double value
   * @param undefined_value The value to use when the value is not available,
   * default is the kNotAvailable_Double
   *
   * @note The precision is the value that is used to scale the double value
   *       before it is added to the message data as an integer.
   */
  void Add4ByteUDouble(double value, double precision,
                       double undefined_value = kNotAvailable_Double);

 private:
  // Constants for the upper bound out of ranges values for each type
  static constexpr int8_t kOutOfRangeUpper_Int8 = 0x7E;
  static constexpr uint8_t kOutOfRangeUpper_UInt8 = 0xFE;
  static constexpr int16_t kOutOfRangeUpper_Int16 = 0x7FFE;
  static constexpr uint16_t kOutOfRangeUpper_UInt16 = 0xFFFE;
  static constexpr int32_t kOutOfRangeUpper_Int24 = 0x7FFFFE;
  static constexpr uint32_t kOutOfRangeUpper_UInt24 = 0xFFFFFE;
  static constexpr int32_t kOutOfRangeUpper_Int32 = 0x7FFFFFFE;
  static constexpr uint32_t kOutOfRangeUpper_UInt32 = 0xFFFFFFFE;

  // Constants for the lower bound out of ranges values for each type
  static constexpr int8_t kOutOfRangeLower_Int8 = -128;
  static constexpr int16_t kOutOfRangeLower_Int16 = -32768;
  static constexpr int32_t kOutOfRangeLower_Int24 = -8388608;
  static constexpr int32_t kOutOfRangeLower_Int32 = -2147483648;

  // Member variables for the message contents and tracking information
  uint8_t priority_;
  uint32_t pgn_;
  uint8_t msg_source_;
  uint8_t msg_destination_;  // Either global (255) or specific (0..251)

public:
  uint8_t data_length_;      // current length of the data buffer in bytes
  mutable uint8_t data_[MAX_DATA_SIZE];

};

}  // namespace nmea2k