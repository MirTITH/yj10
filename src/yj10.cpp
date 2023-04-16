#include "yj10.hpp"
#include <stdexcept>
#include <cstring>

// #define DEBUG

using namespace std;

void Yj10::ThrowException()
{
    // string str;
    // str.append("Modbus connection failed: ").append(modbus_strerror(errno));
    // throw runtime_error(str);
}

Yj10::Yj10(const std::string device, int device_id, int baud, char parity, int data_bit, int stop_bit)
{
    input_regs.fill(0);
    holding_regs.fill(0);
    memset(write_buf, 0, sizeof(write_buf));

    mb = modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit);

    if (mb == nullptr)
    {
        ThrowException();
    }

    modbus_set_slave(mb, device_id);
    // modbus_set_response_timeout(mb, 10, 0);
    // modbus_set_indication_timeout(mb, 10, 0);
    // modbus_set_byte_timeout(mb, 10, 0);

#ifdef DEBUG
    modbus_set_debug(mb, true);
#endif // DEBUG
}

Yj10::~Yj10()
{
    modbus_free(mb);
}

void Yj10::Connect()
{
    if (modbus_connect(mb) != 0)
    {
        ThrowException();
    }
}

void Yj10::Close()
{
    modbus_close(mb);
}
