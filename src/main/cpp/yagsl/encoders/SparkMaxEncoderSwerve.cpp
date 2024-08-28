#include "yagsl/encoders/SparkMaxEncoderSwerve.h"

namespace yagsl
{
    SparkMaxEncoderSwerve::SparkMaxEncoderSwerve(SwerveMotor *motor, int conversionFactor)
        : m_failureConfiguring("Encoders", "Failure configuring SparkMax Analog Encoder", Alert::AlertType::WARNING_TRACE),
          m_offsetFailure("Encoders", "Failure to set Absolute Encoder Offset", Alert::AlertType::WARNING_TRACE)
    {
        rev::CANSparkMax *sparkMax = (rev::CANSparkMax *)(motor->GetMotor());
        if (sparkMax)
        {
            m_encoder = sparkMax->GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
            ConfigureSparkMax([&]()
                              { return m_encoder.value().SetVelocityConversionFactor(conversionFactor); });
            ConfigureSparkMax([&]()
                              { return m_encoder.value().SetPositionConversionFactor(conversionFactor); });
        }
        else
        {
            throw std::runtime_error("Motor given to instantiate SparkMaxEncoder is not a CANSparkMax");
        }
    }

    void SparkMaxEncoderSwerve::FactoryDefault()
    {
        // Do nothing
    }

    void SparkMaxEncoderSwerve::ClearStickyFaults()
    {
        // Do nothing
    }

    void SparkMaxEncoderSwerve::Configure(bool inverted)
    {
        m_encoder.value().SetInverted(inverted);
    }

    double SparkMaxEncoderSwerve::GetAbsolutePosition()
    {
        return m_encoder.value().GetPosition();
    }

    void *SparkMaxEncoderSwerve::GetAbsoluteEncoder() const
    {
        return (void *)&m_encoder;
    }

    bool SparkMaxEncoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        rev::REVLibError error;
        for (int i = 0; i < maximumRetries; i++)
        {
            error = m_encoder.value().SetZeroOffset(offset);
            if (error == rev::REVLibError::kOk)
            {
                return true;
            }
        }
        m_offsetFailure.SetText("Failure to set Absolute Encoder Offset Error: " + std::to_string(static_cast<int>(error)));
        m_offsetFailure.Set(true);
        return false;
    }

    double SparkMaxEncoderSwerve::GetVelocity()
    {
        return m_encoder.value().GetVelocity();
    }

    void SparkMaxEncoderSwerve::ConfigureSparkMax(std::function<rev::REVLibError()> config)
    {
        for (int i = 0; i < maximumRetries; i++)
        {
            if (config() == rev::REVLibError::kOk)
            {
                return;
            }
        }
        m_failureConfiguring.Set(true);
    }

} // namespace yagsl
