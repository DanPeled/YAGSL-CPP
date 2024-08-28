#pragma once
#include <frc/DutyCycleEncoder.h>
#include "SwerveAbsoluteEncoder.h"

#include "yagsl/telemetry/Alert.h"

namespace yagsl
{

    class PWMDutyCycleEncoderSwerve : public SwerveAbsoluteEncoder
    {
    public:
        /**
         * Constructor for the PWM duty cycle encoder.
         *
         * @param pin PWM lane for the encoder.
         */
        explicit PWMDutyCycleEncoderSwerve(frc::DutyCycleEncoder *encoder_ptr);

        /**
         * Configure the inversion state of the encoder.
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
         * Get the encoder object.
         *
         * @return DutyCycleEncoder object from the class.
         */
        void *GetAbsoluteEncoder() const override;

        /**
         * Get the velocity in degrees/sec.
         *
         * @return velocity in degrees/sec.
         */
        double GetVelocity() override;

        /** Reset the encoder to factory defaults. */
        void FactoryDefault() override;

        /** Clear sticky faults on the encoder. */
        void ClearStickyFaults() override;

        /**
         * Set the absolute encoder offset.
         *
         * @param offset The offset to set.
         * @return Whether the operation was successful.
         */
        bool SetAbsoluteEncoderOffset(double offset) override;

    private:
        /** Duty Cycle Encoder. */
        frc::DutyCycleEncoder *encoder;
        /** Inversion state. */
        bool isInverted;
        /** An Alert for if the encoder cannot report accurate velocities. */
        Alert inaccurateVelocities;
        /** The Offset in degrees of the PWM absolute encoder. */
        double offset;
    };

} // namespace yagsl
