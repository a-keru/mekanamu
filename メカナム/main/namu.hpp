#include "wiring.h"
#include <utility>
#include <algorithm>
#include <array>
#pragma once

#include <cmath>
#include <Arduino.h>
#include <UdonFwd.hpp>

#include <Udon/Com/Can.hpp>
#include <Udon/Com/Driver/Motor.hpp>
#include <Udon/Driver/BNO055.hpp>

#include <Udon/Types/Vector2D.hpp>
#include <Udon/Types/Position.hpp>
#include <Udon/Algorithm/PidController.hpp>

class Namu
{
    std::array<Udon::MotorBy<Udon::CanWriter>, 4> motors;

    Udon::BNO055        gyro;
    Udon::PidController turnpid;
    Udon::PidController wakepid;
    Udon::PidController wakesubpid;

    uint32_t turnfinish;
    bool     changepid = true;

public:
    Namu(std::array<Udon::MotorBy<Udon::CanWriter>, 4>&& motors,
         Udon::BNO055&& gyro, Udon::PidController&& turnpid,
         Udon::PidController&& wakepid, Udon::PidController&& wakesubpid)
        : motors(std::move(motors))
        , gyro(std::move(gyro))
        , turnpid(std::move(turnpid))
        , wakepid(std::move(wakepid))
        , wakesubpid(std::move(wakesubpid))
    {
    }

    void begin()
    {
        gyro.begin();
    }

    void changeturnpid(double p, double i, double d)
    {
        turnpid.setParam({ p, i, d });
    }

    /// using gyro
    void move(Udon::Pos&& stick, int16_t maxpwm)
    {
        gyroupdate();

        if (stick.turn != 0)
        {
            gyroclear();
            turnfinish = millis();
        }
        if (millis() - turnfinish < 400)
            gyroclear();

        if ((getRealpitch() * RAD_TO_DEG) == 1.53)
        {
            wakepid.clearIntegral();
        }
        if (abs((getRealpitch() * RAD_TO_DEG) - 1.53) < 1.2)
        {
            changepid = false;
        }
        if (abs((getRealpitch() * RAD_TO_DEG) - 1.53) > 2.5)
        {
            changepid = true;
        }

        const Udon::Vec2 local = { stick.vector.x, stick.vector.y };
        // const Udon::Vec2 local     = stick.vector.rotated(getRealyaw());
        // const double turnpower = turnpid(getyaw() * RAD_TO_DEG, 0, -maxpwm, maxpwm);

        double wakepower = 0;
        if (changepid)
        {
            wakepower = wakesubpid((getRealpitch() * RAD_TO_DEG), 1.53, -maxpwm, maxpwm);
        }
        else
        {
            wakepower = wakepid((getRealpitch() * RAD_TO_DEG), 1.53, -maxpwm, maxpwm);
        }
        int16_t power[4];

        power[0] = (+local.x - local.y + stick.turn * 0.5) * 0.4 /* - turnpower*/ - wakepower;
        power[1] = (-local.x + local.y + stick.turn * 0.5) * 0.4 /* - turnpower*/ + wakepower;
        power[2] = (+local.x + local.y + stick.turn * 0.5) * 0.4 /* - turnpower*/ - wakepower;
        power[3] = (-local.x - local.y + stick.turn * 0.5) * 0.4 /* - turnpower*/ + wakepower;

        const double maxpower = max(max(abs(power[0]), abs(power[1])), max(abs(power[2]), abs(power[3])));
        double       p;
        if (maxpwm < maxpower)
        {
            p = (double)maxpwm / (double)maxpower;
        }
        else
        {
            p = 1.0;
        }

        for (uint8_t i = 0; i < 4; i++)
        {
            motors[i].move(power[i] * p);
        }
    }

    void gyrooffset()
    {
        gyroOffset();
    }

    void stop()
    {
        gyroupdate();

        for (auto& motor : motors)
        {
            motor.stop();
        }
        gyroclear();
    }

    void show()
    {
        gyro.show();
        Serial.print('\t');
        Serial.println(getRealpitch() * RAD_TO_DEG);
        motors[0].show();
        Serial.print('\t');
        motors[1].show();
        Serial.print('\t');
        motors[2].show();
        Serial.print('\t');
        motors[3].show();
    }

private:
    double yawoffset = 0;
    double yaw       = 0;
    double realyaw   = 0;

    // double pitchoffset = 0;
    // double pitch       = 0;
    double realpitch = 0;

    void gyroupdate()
    {
        gyro.update();
        realyaw   = gyro.getQuaternion().toYaw();
        realpitch = gyro.getQuaternion().toPitch();
    }

    void gyroOffset()
    {
        gyro.clear();
        yawoffset = 0;
    }

    void gyroclear()
    {
        yawoffset = realyaw;
    }

    double getRealyaw()
    {
        return realyaw;
    }

    double getRealpitch()
    {
        return realpitch;
    }

    double getyaw()
    {
        yaw = realyaw - yawoffset;
        if (yaw > PI)
        {
            yaw = yaw - 2 * PI;
        }
        if (yaw < -PI)
        {
            yaw = yaw + 2 * PI;
        }
        return yaw;
    }
};