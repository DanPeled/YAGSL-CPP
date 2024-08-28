#include "yagsl/encoders/PWMDutyCycleEncoderSwerve.h"

namespace yagsl
{
    PWMDutyCycleEncoderSwerve::PWMDutyCycleEncoderSwerve(frc::DutyCycleEncoder *encoder_ptr)
        : encoder(encoder_ptr),
          isInverted(false),
          inaccurateVelocities("Encoders",
                               "The PWM Duty Cycle encoder may not report accurate velocities!",
                               Alert::AlertType::WARNING_TRACE),
          offset(0.0) {}

    void PWMDutyCycleEncoderSwerve::Configure(bool inverted)
    {
        isInverted = inverted;
    }

    double PWMDutyCycleEncoderSwerve::GetAbsolutePosition()
    {
        return (isInverted ? -1.0 : 1.0) * ((encoder->Get().value() * 360) - offset);
    }

    void *PWMDutyCycleEncoderSwerve::GetAbsoluteEncoder() const
    {
        return encoder;
    }

    double PWMDutyCycleEncoderSwerve::GetVelocity()
    {
        inaccurateVelocities.Set(true);
        return encoder->Get().value();
    }

    void PWMDutyCycleEncoderSwerve::FactoryDefault()
    {
        // Do nothing
    }

    void PWMDutyCycleEncoderSwerve::ClearStickyFaults()
    {
        // Do nothing
    }

    bool PWMDutyCycleEncoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        this->offset = offset;
        return true;
    }

} // namespace yagsl
