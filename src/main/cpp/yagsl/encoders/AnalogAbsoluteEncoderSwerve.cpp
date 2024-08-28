#include "yagsl/encoders/AnalogAbsoluteEncoderSwerve.h"

namespace yagsl
{
    AnalogAbsoluteEncoderSwerve::AnalogAbsoluteEncoderSwerve(frc::AnalogInput *encoder)
        : encoder(encoder),
          cannotSetOffset("Encoders", "Cannot Set Absolute Encoder Offset of Analog Encoders Channel #" + std::to_string(encoder->GetChannel()), Alert::AlertType::WARNING),
          inaccurateVelocities("Encoders", "The Analog Absolute encoder may not report accurate velocities!", Alert::AlertType::WARNING_TRACE)
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
        this->inverted = inverted;
    }

    double AnalogAbsoluteEncoderSwerve::GetAbsolutePosition()
    {
        return (inverted ? -1.0 : 1.0) *
               (encoder->GetAverageVoltage() / frc::RobotController::GetVoltage5V()) *
               360.0;
    }

    void *AnalogAbsoluteEncoderSwerve::GetAbsoluteEncoder() const
    {
        return encoder;
    }

    bool AnalogAbsoluteEncoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        cannotSetOffset.Set(true);
        return true;
    }

    double AnalogAbsoluteEncoderSwerve::GetVelocity()
    {
        inaccurateVelocities.Set(true);
        return encoder->GetValue() * 360.0;
    }

} // namespace yagsl
