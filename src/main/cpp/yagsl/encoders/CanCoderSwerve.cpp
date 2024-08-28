#include "yagsl/encoders/CanCoderSwerve.h"

const units::time::second_t yagsl::CANCoderSwerve::STATUS_TIMEOUT_SECONDS{0.02};

namespace yagsl
{
    CANCoderSwerve::CANCoderSwerve(int id, const std::string &canbus)
        : m_encoder{id, canbus},
          m_magnetFieldLessThanIdeal("Encoders", fmt::format("CANCoder {} magnetic field is less than ideal.", id), Alert::AlertType::WARNING),
          m_readingFaulty("Encoders", fmt::format("CANCoder {} reading was faulty.", id), Alert::AlertType::WARNING),
          m_readingIgnored("Encoders", fmt::format("CANCoder {} reading was faulty, ignoring.", id), Alert::AlertType::WARNING),
          m_cannotSetOffset("Encoders", fmt::format("Failure to set CANCoder {} Absolute Encoder Offset", id), Alert::AlertType::WARNING)
    {
    }

    void CANCoderSwerve::FactoryDefault()
    {
        m_encoder.GetConfigurator().Apply(ctre::phoenix6::configs::CANcoderConfiguration());
    }

    void CANCoderSwerve::ClearStickyFaults()
    {
        m_encoder.ClearStickyFaults();
    }

    void CANCoderSwerve::Configure(bool inverted)
    {
        auto &cfg = m_encoder.GetConfigurator();
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
        auto strength = m_encoder.GetMagnetHealth().GetValue();

        m_magnetFieldLessThanIdeal.Set(strength != ctre::phoenix6::signals::MagnetHealthValue::Magnet_Green);
        if (strength == ctre::phoenix6::signals::MagnetHealthValue::Magnet_Invalid || strength == ctre::phoenix6::signals::MagnetHealthValue::Magnet_Red)
        {
            readingError = true;
            m_readingFaulty.Set(true);
            return 0;
        }
        else
        {
            m_readingFaulty.Set(false);
        }

        auto angle = m_encoder.GetAbsolutePosition();

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
            m_readingIgnored.Set(true);
            return 0;
        }
        else
        {
            m_readingIgnored.Set(false);
        }

        return angle.GetValue().value() * 360;
    }

    void *CANCoderSwerve::GetAbsoluteEncoder() const
    {
        return (void *)&m_encoder;
    }

    bool CANCoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        auto &cfg = m_encoder.GetConfigurator();
        auto magCfg = ctre::phoenix6::configs::MagnetSensorConfigs();
        auto error = cfg.Refresh(magCfg);
        if (error != ctre::phoenix::StatusCode::OK)
        {
            return false;
        }
        error = cfg.Apply(magCfg.WithMagnetOffset(offset / 360));
        m_cannotSetOffset.SetText(std::string("Failure to set CANCoder ") + std::to_string(m_encoder.GetDeviceID()) + std::string("Absolute Encoder Offset Error: ") + std::to_string(error));
        if (error == ctre::phoenix::StatusCode::OK)
        {
            m_cannotSetOffset.Set(false);
            return true;
        }
        m_cannotSetOffset.Set(true);
        return false;
    }

    double CANCoderSwerve::GetVelocity()
    {
        return m_encoder.GetVelocity().GetValue().value() * 360;
    }
}
