#include "yagsl/encoders/SparkMaxAnalogEncoderSwerve.h"

namespace yagsl
{

    SparkMaxAnalogEncoderSwerve::SparkMaxAnalogEncoderSwerve(rev::CANSparkMax *motor)
        : encoder(motor->GetAnalog(rev::SparkAnalogSensor::Mode::kAbsolute)),
          failureConfiguring("Encoders", "Failure configuring SparkMax Analog Encoder", Alert::AlertType::WARNING_TRACE),
          doesNotSupportIntegratedOffsets("Encoders", "SparkMax Analog Sensors do not support integrated offsets", Alert::AlertType::WARNING_TRACE)
    {
    }

    void SparkMaxAnalogEncoderSwerve::FactoryDefault()
    {
        // No factory default reset for SparkMax Analog Encoder
    }

    void SparkMaxAnalogEncoderSwerve::ClearStickyFaults()
    {
        // No sticky fault clearing for SparkMax Analog Encoder
    }

    void SparkMaxAnalogEncoderSwerve::Configure(bool inverted)
    {
        encoder.SetInverted(inverted);
    }

    double SparkMaxAnalogEncoderSwerve::GetAbsolutePosition()
    {
        return encoder.GetPosition();
    }

    void *SparkMaxAnalogEncoderSwerve::GetAbsoluteEncoder() const
    {
        return (void *)&encoder;
    }

    bool SparkMaxAnalogEncoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        doesNotSupportIntegratedOffsets.Set(true);
        return false; // SparkMax Analog Sensors do not support integrated offsets
    }

    double SparkMaxAnalogEncoderSwerve::GetVelocity()
    {
        return encoder.GetVelocity();
    }

    void SparkMaxAnalogEncoderSwerve::ConfigureSparkMax(std::function<rev::REVLibError()> config)
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
