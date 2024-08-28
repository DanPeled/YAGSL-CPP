#include "yagsl/encoders/CanAndCoderSwerve.h"

namespace yagsl
{
    CanAndCoderSwerve::CanAndCoderSwerve(int canid)
        : encoder(canid)
    {
    }

    void CanAndCoderSwerve::FactoryDefault()
    {
        encoder.ResetFactoryDefaults(false);
    }

    void CanAndCoderSwerve::ClearStickyFaults()
    {
        encoder.ClearStickyFaults();
    }

    void CanAndCoderSwerve::Configure(bool inverted)
    {
        redux::sensors::canandmag::CanandmagSettings settings{};
        settings.SetInvertDirection(inverted);
        encoder.SetSettings(settings);
    }

    double CanAndCoderSwerve::GetAbsolutePosition()
    {
        return encoder.GetAbsPosition().value() * 360.0;
    }

    void *CanAndCoderSwerve::GetAbsoluteEncoder() const
    {
        return (void *)&encoder;
    }

    bool CanAndCoderSwerve::SetAbsoluteEncoderOffset(double offset)
    {
        redux::sensors::canandmag::CanandmagSettings settings{};
        settings.SetZeroOffset(units::turn_t{offset});
        encoder.SetSettings(settings);

        return true;
    }

    double CanAndCoderSwerve::GetVelocity()
    {
        return encoder.GetVelocity().value() * 360.0;
    }

} // namespace yagsl
