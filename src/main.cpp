#include <iostream>
#include <modbus/modbus.h>
#include "yj10.hpp"
#include <thread>
#include <chrono>

using namespace std;

void ReadArm(Yj10 &arm)
{
    auto startTime = chrono::steady_clock::now();
    arm.ReadAllHoldingRegs();
    auto endTime = chrono::steady_clock::now();
    chrono::duration<double, std::milli> durationMs = endTime - startTime;
    cout << "Time: " << durationMs.count() << " ";

    for (int i = 0; i < 4; i++)
    {
        cout << "Joints: " << arm.Joint(i) << " ";
    }

    cout << "Clamper: ";

    cout << "Current: " << arm.ClamperCurrent() << " State: ";
    switch (arm.Clamper())
    {
    case Yj10::ClamperState::Open:
        cout << "Open" << endl;
        break;
    case Yj10::ClamperState::Middle:
        cout << "Middle" << endl;
        break;
    case Yj10::ClamperState::Stop:
        cout << "Stop" << endl;
        break;
    case Yj10::ClamperState::Close:
        cout << "Close" << endl;
        break;
    case Yj10::ClamperState::Error:
        cout << "Error" << endl;
        break;

    default:
        break;
    }
}

int main(int, char **)
{
    Yj10 arm;

    arm.Connect("/dev/ttyUSB0");

    cout << "Connected" << endl;

    arm.ResetPose();

    try
    {
        ReadArm(arm);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    uint16_t pwm, id;

    while (true)
    {

        // ReadArm(arm);
        // this_thread::sleep_for(0.1s);

        // if (pwms.at(3) >= 1600)
        // {
        //     delta = -10;
        // }
        // else if (pwms.at(3) <= 1400)
        // {
        //     delta = 10;
        // }

        // pwms.at(3) += delta;

        cout << "Please enter id pwm:";
        cin >> id >> pwm;

        cout << "Writing id: " << id << ", pwm: " << pwm << endl;

        try
        {
            arm.WriteJoint(id, pwm);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        //     int16_t instruct = 0;
        //     Yj10::ClamperState state;

        //     cin >> instruct;

        //     switch (instruct)
        //     {
        //     case 0:
        //         state = Yj10::ClamperState::Stop;
        //         break;
        //     case 1:
        //         state = Yj10::ClamperState::Close;
        //         break;
        //     case 2:
        //         state = Yj10::ClamperState::Open;
        //         break;

        //     default:
        //         state = Yj10::ClamperState::Stop;
        //         break;
        //     }

        //     arm.WriteClamperInstruction(state);
        //     this_thread::sleep_for(0.1s);
        //     for (int i = 0; i < 5; i++)
        //     {
        //         cout << i << ": ";

        //         ReadArm(arm);
        //         this_thread::sleep_for(0.1s);
        //     }
    }

    return 0;
}
