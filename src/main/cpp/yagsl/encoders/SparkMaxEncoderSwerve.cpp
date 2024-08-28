#include "yagsl/encoders/SparkMaxEncoderSwerve.h"

namespace yagsl
{
    SparkMaxEncoderSwerve::SparkMaxEncoderSwerve(SwerveMotor *motor, int conversionFactor)
        : failureConfiguring("Encoders", "Failure configuring SparkMax Analog Encoder", Alert::AlertType::WARNING_TRACE),
          offsetFailure("Encoders", "Failure to set Absolute Encoder Offset", Alert::AlertType::WARNING_TRACE)
    {
        rev::CANSparkMax *sparkMax = (rev::CANSparkMax *)(motor->GetMotor());
        if (sparkMax)
        {
            encoder = sparkMax->GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
            ConfigureSparkMax([&]()
                              { return encoder.value().SetVelocityConversionFactor(conversionFactor); });
            ConfigureSparkMax([&]()
                              { return encoder.value().SetPositionConversionFactor(conversionFactor); });
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
        encoder.value().SetInverted(inverted);
    }

    double SparkMaxEncoderSwerve::GetAbsolutePosition()
    {
        return encoder.value().GetPosition();
    }

    void *SparkMaxEncoderSwerve::GetAbsoluteEncoder() const
    {
        return (void *)&encoder;
    }

    bool SparkMaxEncoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        rev::REVLibError error;
        for (int i = 0; i < maximumRetries; i++)
        {
            error = encoder.value().SetZeroOffset(offset);
            if (error == rev::REVLibError::kOk)
            {
                return true;
            }
        }
        offsetFailure.SetText("Failure to set Absolute Encoder Offset Error: " + std::to_string(static_cast<int>(error)));
        offsetFailure.Set(true);
        return false;
    }

    double SparkMaxEncoderSwerve::GetVelocity()
    {
        return encoder.value().GetVelocity();
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
        failureConfiguring.Set(true);
    }

} // namespace yagsl
