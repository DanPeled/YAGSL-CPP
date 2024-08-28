#include "yagsl/encoders/PWMDutyCycleEncoderSwerve.h"

namespace yagsl
{
    PWMDutyCycleEncoderSwerve::PWMDutyCycleEncoderSwerve(frc::DutyCycleEncoder *encoder_ptr)
        : m_encoder(encoder_ptr),
          m_isInverted(false),
          m_inaccurateVelocities("Encoders",
                                 "The PWM Duty Cycle encoder may not report accurate velocities!",
                                 Alert::AlertType::WARNING_TRACE),
          m_offset(0.0) {}

    void PWMDutyCycleEncoderSwerve::Configure(bool inverted)
    {
        m_isInverted = inverted;
    }

    double PWMDutyCycleEncoderSwerve::GetAbsolutePosition()
    {
        return (m_isInverted ? -1.0 : 1.0) * ((m_encoder->Get().value() * 360) - m_offset);
    }

    void *PWMDutyCycleEncoderSwerve::GetAbsoluteEncoder() const
    {
        return m_encoder;
    }

    double PWMDutyCycleEncoderSwerve::GetVelocity()
    {
        m_inaccurateVelocities.Set(true);
        return m_encoder->Get().value();
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
        this->m_offset = offset;
        return true;
    }
} // namespace yagsl
