#include "yj10.hpp"
#include <stdexcept>
#include <cstring>

// #define DEBUG

using namespace std;

Yj10::ClamperState Yj10::Clamper() const
{
    switch (holding_regs.at(0x07))
    {
    case 0:
        return ClamperState::Middle;
        break;
    case 1:
        return ClamperState::Close;
        break;
    case 2:
        return ClamperState::Open;
        break;

    default:
        return ClamperState::Error;
        break;
    }
}

void Yj10::ThrowException()
{
    // string str;
    // str.append("Modbus connection failed: ").append(modbus_strerror(errno));
    // throw runtime_error(str);
}

const uint16_t *Yj10::ReadInputRegisters(int start_addr, int num)
{
    if (modbus_read_input_registers(mb, start_addr, num, input_regs.data() + start_addr) == -1)
    {
        ThrowException();
    }
    return input_regs.data() + start_addr;
}

const uint16_t *Yj10::ReadHoldingRegisters(int start_addr, int num)
{
    if (modbus_read_registers(mb, start_addr, num, holding_regs.data() + start_addr) == -1)
    {
        ThrowException();
    }
    return holding_regs.data() + start_addr;
}

void Yj10::WriteHoldingRegisters(int start_addr, int num, const uint16_t *data)
{
    if (modbus_write_registers(mb, start_addr, num, data) == -1)
    {
        ThrowException();
    }
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

void Yj10::WriteAllJoints(uint16_t pwm[])
{
    for (int i = 0; i < 5; i++)
    {
        write_buf[i] = pwm[i];
    }

    WriteHoldingRegisters(0, 5, write_buf);
}

void Yj10::WriteClamperInstruction(ClamperState state)
{
    uint16_t data;
    switch (state)
    {
    case ClamperState::Stop:
        data = 0;
        break;
    case ClamperState::Close:
        data = 1;
        break;
    case ClamperState::Open:
        data = 2;
        break;

    default:
        data = 0;
        break;
    }
    WriteHoldingRegister(0x6, data);
}

void Yj10::WriteAllJoints(std::array<uint16_t, 5> pwms)
{
    for (size_t i = 0; i < pwms.size(); i++)
    {
        write_buf[i] = pwms.at(i);
    }

    WriteHoldingRegisters(0, pwms.size(), write_buf);
}
