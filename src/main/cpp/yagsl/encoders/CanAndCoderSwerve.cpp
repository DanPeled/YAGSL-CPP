#include "yagsl/encoders/CanAndCoderSwerve.h"

namespace yagsl
{
    CanAndCoderSwerve::CanAndCoderSwerve(int canid)
        : m_encoder(canid)
    {
    }

    void CanAndCoderSwerve::FactoryDefault()
    {
        m_encoder.ResetFactoryDefaults(false);
    }

    void CanAndCoderSwerve::ClearStickyFaults()
    {
        m_encoder.ClearStickyFaults();
    }

    void CanAndCoderSwerve::Configure(bool inverted)
    {
        redux::sensors::canandmag::CanandmagSettings settings{};
        settings.SetInvertDirection(inverted);
        m_encoder.SetSettings(settings);
    }

    double CanAndCoderSwerve::GetAbsolutePosition()
    {
        return m_encoder.GetAbsPosition().value() * 360.0;
    }

    void *CanAndCoderSwerve::GetAbsoluteEncoder() const
    {
        return (void *)&m_encoder;
    }

    bool CanAndCoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        redux::sensors::canandmag::CanandmagSettings settings{};
        settings.SetZeroOffset(units::turn_t{offset});
        m_encoder.SetSettings(settings);

        return true;
    }

    double CanAndCoderSwerve::GetVelocity()
    {
        return m_encoder.GetVelocity().value() * 360.0;
    }

} // namespace yagsl
