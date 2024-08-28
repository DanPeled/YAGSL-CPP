#include "yagsl/encoders/CanCoderSwerve.h"

const units::time::second_t yagsl::CANCoderSwerve::STATUS_TIMEOUT_SECONDS{0.02};

namespace yagsl
{
    CANCoderSwerve::CANCoderSwerve(int id, const std::string &canbus)
        : encoder{id, canbus},
          magnetFieldLessThanIdeal("Encoders", fmt::format("CANCoder {} magnetic field is less than ideal.", id), Alert::AlertType::WARNING),
          readingFaulty("Encoders", fmt::format("CANCoder {} reading was faulty.", id), Alert::AlertType::WARNING),
          readingIgnored("Encoders", fmt::format("CANCoder {} reading was faulty, ignoring.", id), Alert::AlertType::WARNING),
          cannotSetOffset("Encoders", fmt::format("Failure to set CANCoder {} Absolute Encoder Offset", id), Alert::AlertType::WARNING)
    {
    }

    void CANCoderSwerve::FactoryDefault()
    {
        encoder.GetConfigurator().Apply(ctre::phoenix6::configs::CANcoderConfiguration());
    }

    void CANCoderSwerve::ClearStickyFaults()
    {
        encoder.ClearStickyFaults();
    }

    void CANCoderSwerve::Configure(bool inverted)
    {
        auto &cfg = encoder.GetConfigurator();
        auto magnetSensorConfiguration = ctre::phoenix6::configs::MagnetSensorConfigs();
        cfg.Refresh(magnetSensorConfiguration);
        cfg.Apply(
            magnetSensorConfiguration
                .WithAbsoluteSensorRange(ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1)
                .WithSensorDirection(inverted ? ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive
                                              : ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive));
    }

    double CANCoderSwerve::GetAbsolutePosition()
    {
        readingError = false;
        auto strength = encoder.GetMagnetHealth().GetValue();

        magnetFieldLessThanIdeal.Set(strength != ctre::phoenix6::signals::MagnetHealthValue::Magnet_Green);
        if (strength == ctre::phoenix6::signals::MagnetHealthValue::Magnet_Invalid || strength == ctre::phoenix6::signals::MagnetHealthValue::Magnet_Red)
        {
            readingError = true;
            readingFaulty.Set(true);
            return 0;
        }
        else
        {
            readingFaulty.Set(false);
        }

        auto angle = encoder.GetAbsolutePosition();

        for (int i = 0; i < maximumRetries; ++i)
        {
            if (angle.GetStatus() == ctre::phoenix::StatusCode::OK)
            {
                break;
            }
            angle = angle.WaitForUpdate(STATUS_TIMEOUT_SECONDS);
        }

        if (angle.GetStatus() != ctre::phoenix::StatusCode::OK)
        {
            readingError = true;
            readingIgnored.Set(true);
            return 0;
        }
        else
        {
            readingIgnored.Set(false);
        }

        return angle.GetValue().value() * 360;
    }

    void *CANCoderSwerve::GetAbsoluteEncoder() const
    {
        return (void *)&encoder;
    }

    bool CANCoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        auto &cfg = encoder.GetConfigurator();
        auto magCfg = ctre::phoenix6::configs::MagnetSensorConfigs();
        auto error = cfg.Refresh(magCfg);
        if (error != ctre::phoenix::StatusCode::OK)
        {
            return false;
        }
        error = cfg.Apply(magCfg.WithMagnetOffset(offset / 360));
        cannotSetOffset.SetText(std::string("Failure to set CANCoder ") + std::to_string(encoder.GetDeviceID()) + std::string("Absolute Encoder Offset Error: ") + std::to_string(error));
        if (error == ctre::phoenix::StatusCode::OK)
        {
            cannotSetOffset.Set(false);
            return true;
        }
        cannotSetOffset.Set(true);
        return false;
    }

    double CANCoderSwerve::GetVelocity()
    {
        return encoder.GetVelocity().GetValue().value() * 360;
    }
}
