#include "yagsl/encoders/SparkMaxAnalogEncoderSwerve.h"

namespace yagsl
{

    SparkMaxAnalogEncoderSwerve::SparkMaxAnalogEncoderSwerve(rev::CANSparkMax *motor)
        : m_encoder(motor->GetAnalog(rev::SparkAnalogSensor::Mode::kAbsolute)),
          m_failureConfiguring("Encoders", "Failure configuring SparkMax Analog Encoder", Alert::AlertType::WARNING_TRACE),
          m_doesNotSupportIntegratedOffsets("Encoders", "SparkMax Analog Sensors do not support integrated offsets", Alert::AlertType::WARNING_TRACE)
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
        m_encoder.SetInverted(inverted);
    }

    double SparkMaxAnalogEncoderSwerve::GetAbsolutePosition()
    {
        return m_encoder.GetPosition();
    }

    void *SparkMaxAnalogEncoderSwerve::GetAbsoluteEncoder() const
    {
        return (void *)&m_encoder;
    }

    bool SparkMaxAnalogEncoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        m_doesNotSupportIntegratedOffsets.Set(true);
        return false; // SparkMax Analog Sensors do not support integrated offsets
    }

    double SparkMaxAnalogEncoderSwerve::GetVelocity()
    {
        return m_encoder.GetVelocity();
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
        m_failureConfiguring.Set(true);
    }

} // namespace yagsl
