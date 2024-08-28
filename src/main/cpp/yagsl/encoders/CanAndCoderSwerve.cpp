// #include "yagsl/encoders/CanAndCoderSwerve.h"

// namespace yagsl
// {
//     CanAndCoderSwerve::CanAndCoderSwerve(int canid)
//         : encoder(canid)
//     {
//     }

//     void CanAndCoderSwerve::FactoryDefault()
//     {
//         encoder.resetFactoryDefaults(false);
//     }

//     void CanAndCoderSwerve::ClearStickyFaults()
//     {
//         encoder.clearStickyFaults();
//     }

//     void CanAndCoderSwerve::Configure(bool inverted)
//     {
//         encoder.setSettings(Canandcoder::Settings().setInvertDirection(inverted));
//     }

//     double CanAndCoderSwerve::GetAbsolutePosition() const
//     {
//         return encoder.getAbsPosition() * 360.0;
//     }

//     void *CanAndCoderSwerve::GetAbsoluteEncoder() const
//     {
//         return &encoder;
//     }

//     bool CanAndCoderSwerve::SetAbsoluteEncoderOffset(double offset)
//     {
//         return encoder.setSettings(Canandcoder::Settings().setZeroOffset(offset));
//     }

//     double CanAndCoderSwerve::GetVelocity()
//     {
//         return encoder.getVelocity() * 360.0;
//     }

// } // namespace yagsl
