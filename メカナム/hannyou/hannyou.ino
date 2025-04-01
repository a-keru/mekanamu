#include <UdonFwd.hpp>
#include <Udon/Com/Can.hpp>
#include <Udon/Com/Message.hpp>
#include <Udon/Algorithm/LoopCycleController.hpp>
#include <Udon/Driver/Led.hpp>
#include <Udon/Driver/Motor.hpp>
#include <Udon/Driver/EncoderPico.hpp>
#include <Udon/Utility/PicoWDT.hpp>
#include "sub.hpp"

Udon::CanBusSpi canbus{
    { .channel   = spi0,
      .cs        = 17,
      .interrupt = 20 }
};

MoveMotor motors[]{
    { { canbus, 0x000 }, { 0, 2, 1 } },
    { { canbus, 0x001 }, { 3, 5, 4 } },
    { { canbus, 0x002 }, { 6, 8, 7 } },
    { { canbus, 0x003 }, { 9, 11, 10 } },
};

// ReadEncoder enc[]{
//     { { canbus, 0x040 }, { 21, 22 } },
//     { { canbus, 0x041 }, { 14, 15 } },
//     { { canbus, 0x012 }, { 12, 13 } },
// };

// Readsw sws[]{
//     { { canbus, 0x020 }, 26 },
//     { { canbus, 0x021 }, 27 },
//     { { canbus, 0x022 }, 28 }
// };

Udon::LoopCycleController loopCtrl(1000);

Udon::Led pico_led(LED_BUILTIN);

Udon::PicoWDT wdt;

uint32_t loopMs = 0;

void setup()
{
    canbus.begin();
    Serial.begin(115200);

    for (auto&& encoder : motors)
        encoder.begin();

    pico_led.begin();
}

void loop()
{
    canbus.update();
    pico_led.flush();
    wdt.update();

    for (auto&& motor : motors)
    {
        motor.move();
        motor.show();
        Serial.print("\t");
    }

    // for (auto&& encoder : enc)
    // {
    //     encoder.update();
    //     encoder.show();
    // }
    // for (auto&& sw : sws)
    //     sw.read();

    Serial.println();

    // loopMs = millis();
    loopCtrl.update();
}