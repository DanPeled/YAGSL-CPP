#pragma once

#include <rev/CANSparkMax.h>
#include <rev/SparkAnalogSensor.h>
#include <frc/geometry/Rotation2d.h>
#include "SwerveAbsoluteEncoder.h"
#include "yagsl/telemetry/Alert.h"

namespace yagsl
{

    /** SparkMax absolute encoder, attached through the data port analog pin. */
    class SparkMaxAnalogEncoderSwerve : public SwerveAbsoluteEncoder
    {
    public:
        /**
         * Create the SparkMaxAnalogEncoderSwerve object as an analog sensor from the CANSparkMax motor data port analog pin.
         *
         * @param motor Motor to create the encoder from.
         */
        explicit SparkMaxAnalogEncoderSwerve(rev::CANSparkMax *motor);

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

        /** Get the absolute position of the encoder.
         *
         * @return Absolute position in degrees from [0, 360).
         */
        double GetAbsolutePosition() override;

        /** Get the instantiated absolute encoder Object.
         *
         * @return Pointer to the absolute encoder object.
         */
        void *GetAbsoluteEncoder() const override;

        /**
         * Sets the Absolute Encoder offset at the Encoder Level.
         *
         * @param offset The offset the Absolute Encoder uses as the zero point.
         * @return True if setting Absolute Encoder Offset was successful, false otherwise.
         */
        bool SetAbsoluteEncoderOffset(double offset) override;

        /** Get the velocity in degrees/sec.
         *
         * @return Velocity in degrees/sec.
         */
        double GetVelocity() override;

    private:
        /** The SparkAnalogSensor representing the duty cycle encoder attached to the SparkMax analog port. */
        rev::SparkAnalogSensor m_encoder;

        /** An Alert for if there is a failure configuring the encoder. */
        Alert m_failureConfiguring;

        /** An Alert for if the absolute encoder does not support integrated offsets. */
        Alert m_doesNotSupportIntegratedOffsets;

        /**
         * Run the configuration until it succeeds or times out.
         *
         * @param config Lambda supplier returning the error state.
         */
        void ConfigureSparkMax(std::function<rev::REVLibError()> config);
    };

} // namespace yagsl
