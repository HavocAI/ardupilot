#include <AP_gtest.h>

#include <AP_J1939_CAN/AP_J1939_CAN.h>

#include <cstring>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


TEST(AP_J1939_CAN, test_id)
{

    J1939::Id id(0x18EAFF00);

    EXPECT_EQ(id.priority(), 6);
    EXPECT_EQ(id.data_page(), 0);
    EXPECT_EQ(id.pgn_raw(), 59904U);
    EXPECT_EQ(id.pdu_format(), J1939::PDUFormat::PDU1);
    EXPECT_EQ(id.source_address(), 0x00);
    EXPECT_EQ(id.pdu_specific(), 255U);

}

TEST(AP_J1939_CAN, test_rx_broadcast_transport)
{
    J1939::BroadcastTransport bt;

    {
    uint8_t test_data[8] = {0x20, 0x0e, 0x00, 0x02, 0xff, 0xca, 0xfe, 0x00};
    AP_HAL::CANFrame frame(0x18ecff0b, test_data, 8);
    EXPECT_FALSE(bt.from_frame(frame));
    EXPECT_EQ(bt.packet_count(), 2);
    }

    {
    uint8_t test_data[8] = {0x01, 0x04, 0xff, 0x1d, 0x03, 0x05, 0x02, 0x1e};
    AP_HAL::CANFrame frame(0x18ebff0b, test_data, 8);
    EXPECT_FALSE(bt.from_frame(frame));
    }

    {
    uint8_t test_data[8] = {0x02, 0x03, 0x05, 0x02, 0x75, 0x02, 0x08, 0x01};
    AP_HAL::CANFrame frame(0x18ebff0b, test_data, 8);
    EXPECT_TRUE(bt.from_frame(frame));
    }

    const uint8_t expected_data[] = {0x04, 0xff, 0x1d, 0x03, 0x05, 0x02, 0x1e, 0x03, 0x05, 0x02, 0x75, 0x02, 0x08, 0x01};
    EXPECT_TRUE(std::memcmp(bt.data_ptr(), expected_data, 14) == 0);
}

TEST(AP_J1939_CAN, test_dm1_dtc)
{
    using namespace J1939;
    const uint8_t test_data[] = {0x1d, 0x03, 0x05, 0x02};
    DiagnosticMessage1::DTC dtc = DiagnosticMessage1::DTC::from_data(test_data);

    EXPECT_EQ(dtc.spn(), 797U);
    EXPECT_EQ(dtc.fmi(), 5U);
    EXPECT_EQ(dtc.oc(), 2U);
    EXPECT_FALSE(dtc.cm());
}


/*
TEST(AP_J1939_CAN, test_rx_broadcast_transport_dm1)
{
    J1939::BroadcastTransport bt;

    {
    uint8_t test_data[8] = {0x20, 0x0c, 0x00, 0x04, 0xca, 0xfe, 0x00, 0x00};
    AP_HAL::CANFrame frame(0x4ec00ef, test_data, 8);
    EXPECT_FALSE(bt.from_frame(frame));
    EXPECT_EQ(bt.packet_count(), 2);
    }

    {
    uint8_t test_data[8] = {0x01, 0x00, 0x03, 0x00, 0x1a, 0xff, 0x00, 0x00};
    AP_HAL::CANFrame frame(0x4eb00ef, test_data, 8);
    EXPECT_FALSE(bt.from_frame(frame));
    }

    {
    uint8_t test_data[8] = {0x02, 0x00, 0x07, 0x00, 0x17, 0xff, 0x00, 0x00};
    AP_HAL::CANFrame frame(0x4eb00ef, test_data, 8);
    EXPECT_FALSE(bt.from_frame(frame));
    }

    {
    uint8_t test_data[8] = {0x03, 0x00, 0x0d, 0x00, 0x1a, 0xc2, 0x00, 0x00};
    AP_HAL::CANFrame frame(0x4eb00ef, test_data, 8);
    EXPECT_FALSE(bt.from_frame(frame));
    }

    {
    uint8_t test_data[8] = {0x04, 0x00, 0x0e, 0x00, 0x1a, 0xb6, 0x00, 0x00};
    AP_HAL::CANFrame frame(0x4eb00ef, test_data, 8);
    EXPECT_TRUE(bt.from_frame(frame));
    }

    const uint8_t expected_data[] = {0x04, 0xff, 0x1d, 0x03, 0x05, 0x02, 0x1e, 0x03, 0x05, 0x02, 0x75, 0x02, 0x08, 0x01};
    EXPECT_TRUE(std::memcmp(bt.data_ptr(), expected_data, 14) == 0);
}
*/

AP_GTEST_MAIN()
