#include "yagsl/encoders/AnalogAbsoluteEncoderSwerve.h"

namespace yagsl
{
    AnalogAbsoluteEncoderSwerve::AnalogAbsoluteEncoderSwerve(frc::AnalogInput *encoder)
        : m_encoder(encoder),
          m_cannotSetOffset("Encoders", "Cannot Set Absolute Encoder Offset of Analog Encoders Channel #" + std::to_string(encoder->GetChannel()), Alert::AlertType::WARNING),
          m_inaccurateVelocities("Encoders", "The Analog Absolute encoder may not report accurate velocities!", Alert::AlertType::WARNING_TRACE)
    {
    }

    AnalogAbsoluteEncoderSwerve::AnalogAbsoluteEncoderSwerve(int channel)
        : AnalogAbsoluteEncoderSwerve(new frc::AnalogInput(channel))
    {
    }

    void AnalogAbsoluteEncoderSwerve::FactoryDefault()
    {
        // Do nothing
    }

    void AnalogAbsoluteEncoderSwerve::ClearStickyFaults()
    {
        // Do nothing
    }

    void AnalogAbsoluteEncoderSwerve::Configure(bool inverted)
    {
        this->m_inverted = inverted;
    }

    double AnalogAbsoluteEncoderSwerve::GetAbsolutePosition()
    {
        return (m_inverted ? -1.0 : 1.0) *
               (m_encoder->GetAverageVoltage() / frc::RobotController::GetVoltage5V()) *
               360.0;
    }

    void *AnalogAbsoluteEncoderSwerve::GetAbsoluteEncoder() const
    {
        return m_encoder;
    }

    bool AnalogAbsoluteEncoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        m_cannotSetOffset.Set(true);
        return true;
    }

    double AnalogAbsoluteEncoderSwerve::GetVelocity()
    {
        m_inaccurateVelocities.Set(true);
        return m_encoder->GetValue() * 360.0;
    }

} // namespace yagsl
