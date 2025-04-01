#include "SerialUSB.h"
#pragma once
#include <stdint.h>

#include <UdonFwd.hpp>
#include <Udon/Com/Can.hpp>
#include <Udon/Com/Message.hpp>
#include <Udon/Algorithm/LoopCycleController.hpp>
#include <Udon/Driver/Motor.hpp>
#include <Udon/Driver/EncoderPico.hpp>

class MoveMotor
{
    Udon::CanReader<Udon::Message::Motor> reader;
    Udon::SmoothlyMotor3<10>              motor;

public:
    MoveMotor(Udon::CanReader<Udon::Message::Motor>&& reader,
              Udon::SmoothlyMotor3<10>&&              motor)
        : reader(std::move(reader))
        , motor(motor)
    {
    }

    void begin()
    {
        motor.begin();
    }

    bool connect()
    {
        return reader ? true : false;
    }

    void move()
    {
        if (const auto message = reader.getMessage())
            motor.move(message->power);
        else
            motor.move(0);
    }

    void move(int16_t pwm)
    {
        motor.move({ pwm });
    }

    void show()
    {
        motor.show();
    }
};

class ReadEncoder
{

    Udon::CanWriter<Udon::Message::Encoder> sender;

    Udon::EncoderPico encoder;

public:
    ReadEncoder(Udon::CanWriter<Udon::Message::Encoder>&& sender,
                Udon::EncoderPico&&                       encoder)
        : sender(std::move(sender))
        , encoder(std::move(encoder))
    {
    }

    void begin()
    {
        encoder.begin();
    }

    void update()
    {
        sender.setMessage({ encoder.read() });
    }

    int32_t getCount()
    {
        return encoder.read();
    }

    void show()
    {
        Serial.print(encoder.read());
        Serial.print('\t');
    }
};

class Readsw
{
    Udon::CanWriter<Udon::Message::Switch> sender;
    uint8_t                                swpin;

public:
    Readsw(Udon::CanWriter<Udon::Message::Switch>&& sender,
           uint8_t                                  pin)
        : sender(std::move(sender))
        , swpin(pin)
    {
        pinMode(swpin, INPUT_PULLUP);
    }

    void update()
    {
        sender.setMessage({ !digitalRead(swpin) });
    }

    bool read()
    {
        return !digitalRead(swpin);
    }

    void show()
    {
        Serial.print(!digitalRead(swpin));
        Serial.print('\t');
    }
};