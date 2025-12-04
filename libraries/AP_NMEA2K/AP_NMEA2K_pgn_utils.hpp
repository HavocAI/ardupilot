/****************************************************************************
 *
 *   Copyright (c) 2025 HavocAI. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

inline bool IsSingleFrameDefaultMessage(uint32_t pgn) {
  // Check if the PGN is a single frame default message
  return (pgn >= 0xE800 && pgn <= 0xFFFF) || (pgn >= 0x1F000 && pgn <= 0x1FEFF);
}

/**
 * @brief Check if the given PGN is a fast packet default message
 *
 * In this case a default message is defined as a message that is not a system
 * specific or device/product specific message.
 *
 * @param pgn The PGN to check
 * @return true if the PGN is a fast packet default message, false otherwise
 */
inline bool IsFastPacketDefaultMessage(uint32_t pgn) {
  switch (pgn) {
    case 126983:  // Alert, pri=2, period=1000
    case 126984:  // Alert Response, pri=2, period=NA
    case 126985:  // Alert Text, pri=2, period=10000
    case 126986:  // Alert Configuration, pri=2, period=NA
    case 126987:  // Alert Threshold, pri=2, period=NA
    case 126988:  // Alert Value, pri=2, period=10000
    case 127233:  // Man Overboard Notification(MOB), pri=3, period=NA
    case 127237:  // Heading/Track control, pri=2, period=250
    case 127489:  // Engine parameters dynamic, pri=2, period=500
    case 127490:  // Electric Drive Status (Dynamic), pri=1, period=1500
    case 127491:  // Electric Energy Storage Status (Dynamic), pri=7,

    // period=1500
    case 127494:  // Electric Drive Information, pri=4, period=NA
    case 127495:  // Electric Energy Storage Information, pri=6, period=NA
    case 127496:  // Trip fuel consumption, vessel, pri=5, period=1000
    case 127497:  // Trip fuel consumption, engine, pri=5, period=1000
    case 127498:  // Engine parameters static, pri=5, period=NA
    case 127503:  // AC Input Status, pri=6, period=1500
    case 127504:  // AC Output Status, pri=6, period=1500
    case 127506:  // DC Detailed status, pri=6, period=1500
    case 127507:  // Charger status, pri=6, period=1500
    case 127509:  // Inverter status, pri=6, period=1500
    case 127510:  // Charger configuration status, pri=6, period=NA
    case 127511:  // Inverter Configuration Status, pri=6, period=NA
    case 127512:  // AGS configuration status, pri=6, period=NA
    case 127513:  // Battery configuration status, pri=6, period=NA
    case 127514:  // AGS Status, pri=6, period=1500
    case 128275:  // Distance log, pri=6, period=1000
    case 128520:  // Tracked Target Data, pri=2, period=1000
    case 128538:  // Elevator car status, pri=6, period=100
    case 129029:  // GNSS Position Data, pri=3, period=1000
    case 129038:  // AIS Class A Position Report, pri=4, period=NA
    case 129039:  // AIS Class B Position Report, pri=4, period=NA
    case 129040:  // AIS Class B Extended Position Report, pri=4, period=NA
    case 129041:  // AIS Aids to Navigation (AtoN) Report, pri=4, period=NA
    case 129044:  // Datum, pri=6, period=10000
    case 129045:  // User Datum Settings, pri=6, period=NA
    case 129284:  // Navigation info, pri=3, period=1000
    case 129285:  // Waypoint list, pri=3, period=NA
    case 129301:  // Time to/from Mark, pri=3, period=1000
    case 129302:  // Bearing and Distance between two Marks, pri=6, period=NA
    case 129538:  // GNSS Control Status, pri=6, period=NA
    case 129540:  // GNSS Sats in View, pri=6, period=1000
    case 129541:  // GPS Almanac Data, pri=6, period=NA
    case 129542:  // GNSS Pseudorange Noise Statistics, pri=6, period=1000
    case 129545:  // GNSS RAIM Output, pri=6, period=NA
    case 129547:  // GNSS Pseudorange Error Statistics, pri=6, period=NA
    case 129549:  // DGNSS Corrections, pri=6, period=NA
    case 129551:  // GNSS Differential Correction Receiver Signal, pri=6,

    // period=NA
    case 129556:  // GLONASS Almanac Data, pri=6, period=NA
    case 129792:  // AIS DGNSS Broadcast Binary Message, pri=6, period=NA
    case 129793:  // AIS UTC and Date Report, pri=7, period=NA
    case 129794:  // AIS Class A Static data, pri=6, period=NA
    case 129795:  // AIS Addressed Binary Message, pri=5, period=NA
    case 129796:  // AIS Acknowledge, pri=7, period=NA
    case 129797:  // AIS Binary Broadcast Message, pri=5, period=NA
    case 129798:  // AIS SAR Aircraft Position Report, pri=4, period=NA
    case 129799:  // Radio Frequency/Mode/Power, pri=3, period=NA
    case 129800:  // AIS UTC/Date Inquiry, pri=7, period=NA
    case 129801:  // AIS Addressed Safety Related Message, pri=5, period=NA
    case 129802:  // AIS Safety Related Broadcast Message, pri=5, period=NA
    case 129803:  // AIS Interrogation PGN, pri=7, period=NA
    case 129804:  // AIS Assignment Mode Command, pri=7, period=NA
    case 129805:  // AIS Data Link Management Message, pri=7, period=NA
    case 129806:  // AIS Channel Management, pri=7, period=NA
    case 129807:  // AIS Group Assignment, pri=7, period=NA
    case 129808:  // DSC Call Information, pri=8, period=NA
    case 129809:  // AIS Class B Static Data: Part A, pri=6, period=NA
    case 129810:  // AIS Class B Static Data Part B, pri=6, period=NA
    case 129811:  // AIS Single Slot Binary Message, pri=5, period=NA
    case 129812:  // AIS Multi Slot Binary Message, pri=5, period=NA
    case 129813:  // AIS Long-Range Broadcast Message, pri=5, period=NA
    case 129814:  // AIS single slot binary message, pri=5, period=NA
    case 129815:  // AIS multi slot binary message, pri=5, period=NA
    case 129816:  // AIS acknowledge, pri=7, period=NA
    case 130052:  // Loran-C TD Data, pri=3, period=1000
    case 130053:  // Loran-C Range Data, pri=3, period=1000
    case 130054:  // Loran-C Signal Data, pri=3, period=1000
    case 130060:  // Label, pri=7, period=NA
    case 130061:  // Channel Source Configuration, pri=7, period=NA
    case 130064:  // Route and WP Service - Database List, pri=7, period=NA
    case 130065:  // Route and WP Service - Route List, pri=7, period=NA
    case 130066:  // Route and WP Service - Route/WP-List Attributes, pri=7,

    // period=NA
    case 130067:  // Route and WP Service - Route - WP Name & Position, pri=7,

    // period=NA
    case 130068:  // Route and WP Service - Route - WP Name, pri=7, period=NA
    case 130069:  // Route and WP Service - XTE Limit & Navigation Method,

    // pri=7, period=NA
    case 130070:  // Route and WP Service - WP Comment, pri=7, period=NA
    case 130071:  // Route and WP Service - Route Comment, pri=7, period=NA
    case 130072:  // Route and WP Service - Database Comment, pri=7, period=NA
    case 130073:  // Route and WP Service - Radius of Turn, pri=7, period=NA
    case 130074:  // Route and WP Service - WP List - WP Name & Position,

    // pri=7, period=NA
    case 130320:  // Tide Station Data, pri=6, period=1000
    case 130321:  // Salinity Station Data, pri=6, period=1000
    case 130322:  // Current Station Data, pri=6, period=1000
    case 130323:  // Meteorological Station Data, pri=6, period=1000
    case 130324:  // Moored Buoy Station Data, pri=6, period=1000
    case 130330:  // Lighting system settings, pri=7, period=NA
    case 130561:  // Lighting zone, pri=7, period=NA
    case 130562:  // Lighting scene, pri=7, period=NA
    case 130563:  // Lighting device, pri=7, period=NA
    case 130564:  // Lighting device enumeration, pri=7, period=NA
    case 130565:  // Lighting color sequence, pri=7, period=NA
    case 130566:  // Lighting program, pri=7, period=NA
    case 130567:  // Watermaker Input Setting and Status, pri=6, period=2500
    case 130577:  // Direction Data PGN, pri=3, period=1000
    case 130578:  // Vessel Speed Components, pri=2, period=250

    // Entertainment PGNs
    case 130569:  // Current File and Status, pri=6, period=500
    case 130570:  // Library Data File, pri=6, period=NA
    case 130571:  // Library Data Group, pri=6, period=NA
    case 130572:  // Library Data Search, pri=6, period=NA
    case 130573:  // Supported Source Data, pri=6, period=NA
    case 130574:  // Supported Zone Data, pri=6, period=NA
    case 130580:  // System Configuration Status, pri=6, period=NA
    case 130581:  // Zone Configuration Status, pri=6, period=NA
    case 130583:  // Available Audio EQ Presets, pri=6, period=NA
    case 130584:  // Bluetooth Devices, pri=6, period=NA
    case 130586:  // Zone Configuration Status, pri=6, period=NA
      return true;
  }

  return false;
}

/**
 * Check to see if a PGN is a fast packet system, configuration or device s
 * specific message
 */
inline bool IsFastPacketSystemMessage(uint32_t pgn) {
  switch (pgn) {
    case 65240:   // Commanded Address
    case 126208:  // NMEA Request/Command/Acknowledge group function
    case 126464:  // System Time, pri=6, period=NA
    case 126996:  // Product Information, pri=6, period=NA
    case 126998:  // Configuration Information, pri=6, period=NA
      return true;
  }

  return false;
}

/**
 * Check if it's a proprietary fastpacket message
 */
inline bool IsFastPacketProprietaryMessage(uint32_t pgn) {
  // Proprietary messages are those that are not defined in the NMEA 2000
  // standard, but are used by specific manufacturers or devices.
  return (pgn >= 65280 && pgn <= 65535);
}

inline bool IsSingleFrameSystemMessage(uint32_t pgn) {
  switch (pgn) {
    case 59392:  // ISO Acknowledgement
    case 59904:  //  ISO Request
    case 60928:  // ISO Address Claim
      return true;
  }

  return false;
}

/**
 * Check to see if the first byte of a fast packet message is the first frame
 * of the message. Upper 3 bits are the sequence identifier and the lower 5 bits
 * are the sequence item number. The first frame of a fast packet message will
 * have the sequence item number set to 0.
 *
 * @param first_byte The first byte of the fast packet message
 * @return true if it is the first frame, false otherwise
 */
inline bool IsFastPacketFirstFrame(uint8_t first_byte) { return ((first_byte & 0x1F) == 0); }

/**
 * Check to see if the address is a broadcast address. The broadcast address
 * is 0xFF.
 *
 * @param address The address to check
 * @return true if it is a broadcast address, false otherwise
 */
inline bool IsBroadcast(uint8_t address) { return address == 0xFF; }