#pragma once
#include <string>
#include <modbus/modbus.h>
#include <array>

class Yj10
{
public:
    enum class ClamperState
    {
        Stop,
        Close,
        Open,
        Middle,
        Error
    };

private:
    modbus_t *mb;
    std::array<uint16_t, 35> input_regs;   // 输入寄存器组
    std::array<uint16_t, 12> holding_regs; // 保持寄存器组
    uint16_t write_buf[12];
    void ThrowException();

    const uint16_t *ReadInputRegisters(int start_addr, int num)
    {
        if (modbus_read_input_registers(mb, start_addr, num, input_regs.data() + start_addr) == -1)
        {
            ThrowException();
        }
        return input_regs.data() + start_addr;
    }

    const uint16_t *ReadHoldingRegisters(int start_addr, int num)
    {
        if (modbus_read_registers(mb, start_addr, num, holding_regs.data() + start_addr) == -1)
        {
            ThrowException();
        }
        return holding_regs.data() + start_addr;
    }

    void WriteHoldingRegisters(int start_addr, int num, const uint16_t *data)
    {
        if (modbus_write_registers(mb, start_addr, num, data) == -1)
        {
            ThrowException();
        }
    }

    void WriteHoldingRegister(int reg_addr, const uint16_t data)
    {
        if (modbus_write_register(mb, reg_addr, data) == -1)
        {
            ThrowException();
        }
    }

public:
    Yj10(const std::string device, int device_id = 0x01, int baud = 9600, char parity = 'N', int data_bit = 8, int stop_bit = 1);
    ~Yj10();

    void Connect();
    void Close();

    /**
     * @brief 从机械臂读取所有的输入寄存器
     * @note 输入寄存器只读。
     * @note 包括：各个关节的 PWM 最大最小值、夹持器最大电流等参数，不需要连续读取
     */
    void ReadAllInputRegs()
    {
        ReadInputRegisters(0x0, input_regs.size());
    }

    /**
     * @brief 从机械臂读取所有的保持寄存器
     * @note 保持寄存器大部分可读写
     * @note 包括各个关节当前 PWM 值，夹持器操作、状态、电流等参数，是主要控制对象
     */
    void ReadAllHoldingRegs()
    {
        ReadHoldingRegisters(0x0, 12);
    }

    void ReadAllJointsPwm()
    {
        ReadHoldingRegisters(0x0, 5);
    }

    /**
     * @brief 读取夹持器状态
     *
     */
    void ReadClamper()
    {
        ReadHoldingRegisters(0x6, 5);
    }

    void WriteJoint(int index, uint16_t pwm)
    {
        if (index >= 0 && index <= 5)
        {
            WriteHoldingRegister(index, pwm);
        }
    }

    void WriteAllJoints(uint16_t pwm[5])
    {
        for (int i = 0; i < 5; i++)
        {
            write_buf[i] = pwm[i];
        }

        WriteHoldingRegisters(0, 5, write_buf);
    }

    void WriteAllJoints(std::array<uint16_t, 5> pwms)
    {
        for (int i = 0; i < 5; i++)
        {
            write_buf[i] = pwms.at(i);
        }

        WriteHoldingRegisters(0, 5, write_buf);
    }

    void WriteClamperInstruction(ClamperState state)
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

    void WriteClamperClosingCurrent(uint16_t current_mA)
    {
        WriteHoldingRegister(0x8, current_mA);
    }

    /**
     * @brief 各个关节的PWM值，读取前需要使用 ReadAllJointsPwm() 更新
     *
     * @param index 范围 [0,4]（index = 5在该型号中没有）
     * @return uint16_t PWM，(500~2500)
     */
    uint16_t Joint(int index) const
    {
        if (index >= 0 && index <= 5)
        {
            return holding_regs.at(index);
        }
        else
        {
            return 0;
        }
    }

    std::array<uint16_t, 5> Joints() const
    {
        std::array<uint16_t, 5> result;
        for (size_t i = 0; i < result.size(); i++)
        {
            result.at(i) = holding_regs.at(i);
        }
        return result;
    }

    /**
     * @brief 夹持器状态，读取前需要使用 ReadClamper() 更新
     *
     * @return ClamperState
     */
    ClamperState Clamper() const
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

    /**
     * @brief 夹持器当前电流
     *
     * @return uint16_t 电流 (mA)
     */
    uint16_t ClamperCurrent() const
    {
        return holding_regs.at(0xA);
    }
};
