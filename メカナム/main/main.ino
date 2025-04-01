/*
メカナム振子
*/

#include <UdonFwd.hpp>

#include <Udon/Com/Can.hpp>
#include <Udon/Com/Message.hpp>
#include <Udon/Com/Pad.hpp>
#include <Udon/Com/Driver/Encoder.hpp>

#include <Udon/Driver/Led.hpp>
#include <Udon/Driver/DipSwitch.hpp>

#include <Udon/Algorithm/ServoSpeed.hpp>
#include <Udon/Algorithm/LoopCycleController.hpp>

#include "namu.hpp"

static Udon::CanBusTeensy<CAN1> canbus;

static Udon::LoopCycleController loopctrl(10000);

static Udon::E220PadPS5 pad{ {
    .serial = Serial3,
    .m0     = 2,
    .m1     = 3,
    .aux    = 5,
} };

static Namu namu{
    std::array<Udon::MotorBy<Udon::CanWriter>, 4>{
        Udon::MotorBy<Udon::CanWriter>{ { canbus, 0x000 }, false },
        Udon::MotorBy<Udon::CanWriter>{ { canbus, 0x001 }, true },
        Udon::MotorBy<Udon::CanWriter>{ { canbus, 0x002 }, true },
        Udon::MotorBy<Udon::CanWriter>{ { canbus, 0x003 }, false } },
    { Wire },
    { 0.5, 0, 0.01, loopctrl.cycleUs() },
    { 18.0, 50.0, 0.2, loopctrl.cycleUs(), 20 },    //{20,100,0.3,loop,10}
    { 80.0, 20.0, 4.0, loopctrl.cycleUs(), 30 }    //{20,100,0.3,loop,10}
};

Udon::Led led{ LED_BUILTIN };

void setup()
{
    delay(500);
    Serial.begin(115200);
    canbus.begin();
    pad.getConfigReference().channel = Udon::DecodeDipSwitch({ 9, 10, 11, 12 });    // // flag.begin(Zoneflag);
    pad.begin();
    namu.begin();
    led.begin();
}

void loop()
{
    canbus.update();
    pad.update();

    if (pad.getTriangle().click)
        namu.gyrooffset();

    if (pad.isOperable())
    {
        namu.move(pad.getMoveInfo(), 200);
        led.flush();
    }
    else
    {
        namu.stop();
        led.flush(200, 20);
    }

    namu.show();

    Serial.println();
    loopctrl.update();
}
