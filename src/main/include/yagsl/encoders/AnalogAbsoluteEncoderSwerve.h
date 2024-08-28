#pragma once

#include <frc/AnalogInput.h>
#include <frc/RobotController.h>
#include "SwerveAbsoluteEncoder.h"
#include "yagsl/telemetry/Alert.h"

namespace yagsl
{
    /**
     * @brief Swerve Absolute Encoder for Thrifty Encoders and other analog encoders.
     */
    class AnalogAbsoluteEncoderSwerve : public SwerveAbsoluteEncoder
    {
    public:
        /**
         * @brief Construct the Thrifty Encoder as a Swerve Absolute Encoder.
         *
         * @param encoder Encoder to construct.
         */
        AnalogAbsoluteEncoderSwerve(frc::AnalogInput *encoder);

        /**
         * @brief Construct the Encoder given the analog input channel.
         *
         * @param channel Analog Input channel of which the encoder resides.
         */
        AnalogAbsoluteEncoderSwerve(int channel);

        /**
         * @brief Reset the encoder to factory defaults.
         */
        void FactoryDefault() override;

        /**
         * @brief Clear sticky faults on the encoder.
         */
        void ClearStickyFaults() override;

        /**
         * @brief Configure the absolute encoder to read from [0, 360) per second.
         *
         * @param inverted Whether the encoder is inverted.
         */
        void Configure(bool inverted) override;

        /**
         * @brief Get the absolute position of the encoder.
         *
         * @return Absolute position in degrees from [0, 360).
         */
        double GetAbsolutePosition() override;

        /**
         * @brief Get the instantiated absolute encoder Object.
         *
         * @return Pointer to the absolute encoder object.
         */
        void *GetAbsoluteEncoder() const override;

        /**
         * @brief Cannot Set the offset of an Analog Absolute Encoder.
         *
         * @param offset The offset the Absolute Encoder uses as the zero point.
         * @return Will always be false as setting the offset is unsupported of an Analog absolute encoder.
         */
        bool SetAbsoluteEncoderOffset(double offset) override;

        /**
         * @brief Get the velocity in degrees/sec.
         *
         * @return Velocity in degrees/sec.
         */
        double GetVelocity() override;

    private:
        frc::AnalogInput *m_encoder;
        bool m_inverted = false;
        Alert m_cannotSetOffset;
        Alert m_inaccurateVelocities;
    };

} // namespace yagsl
