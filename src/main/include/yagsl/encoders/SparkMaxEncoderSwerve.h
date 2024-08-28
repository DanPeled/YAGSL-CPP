#pragma once

#include <memory>
#include <functional>
#include <rev/CANSparkMax.h>
#include <rev/AbsoluteEncoder.h>
#include "yagsl/telemetry/Alert.h"
#include "SwerveAbsoluteEncoder.h"
#include "yagsl/motors/SwerveMotor.h"

namespace yagsl
{

    /**
     * SparkMax absolute encoder, attached through the data port.
     */
    class SparkMaxEncoderSwerve : public SwerveAbsoluteEncoder
    {
    public:
        /**
         * Constructor that creates a SparkMaxEncoderSwerve object as a duty cycle from the CANSparkMax motor.
         *
         * @param motor Motor to create the encoder from.
         * @param conversionFactor The conversion factor to set if the output is not from 0 to 360.
         */
        SparkMaxEncoderSwerve(SwerveMotor *motor, int conversionFactor);

        /** Reset the encoder to factory defaults. */
        void FactoryDefault() override;

        /** Clear sticky faults on the encoder. */
        void ClearStickyFaults() override;

        /**
         * Configure the absolute encoder to read from [0, 360) per second.
         *
         * @param inverted Whether the encoder is inverted.
         */
        void Configure(bool inverted) override;

        /**
         * Get the absolute position of the encoder.
         *
         * @return Absolute position in degrees from [0, 360).
         */
        double GetAbsolutePosition() override;

        /**
         * Get the instantiated absolute encoder Object.
         *
         * @return Absolute encoder object.
         */
        void *GetAbsoluteEncoder() const override;

        /**
         * Sets the Absolute Encoder Offset inside of the SparkMax's Memory.
         *
         * @param offset the offset the Absolute Encoder uses as the zero point.
         * @return if setting Absolute Encoder Offset was successful or not.
         */
        bool SetAbsoluteEncoderOffset(double offset) override;

        /**
         * Get the velocity in degrees/sec.
         *
         * @return velocity in degrees/sec.
         */
        double GetVelocity() override;

    private:
        std::optional<rev::SparkMaxAbsoluteEncoder> m_encoder;
        Alert m_failureConfiguring;
        Alert m_offsetFailure;

        /**
         * Run the configuration until it succeeds or times out.
         *
         * @param config Lambda function returning the error state.
         */
        void ConfigureSparkMax(std::function<rev::REVLibError()> config);

        static const int maximumRetries = 5;
    };

} // namespace yagsl
