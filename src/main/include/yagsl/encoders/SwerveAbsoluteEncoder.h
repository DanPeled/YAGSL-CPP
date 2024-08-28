#pragma once

namespace yagsl
{
    /**
     * @brief Swerve abstraction class to define a standard interface with absolute encoders for swerve modules.
     */
    class SwerveAbsoluteEncoder
    {
    public:
        /**
         * @brief The maximum amount of times the swerve encoder will attempt to configure itself if failures occur.
         */
        static constexpr int maximumRetries = 5;

        /**
         * @brief Last angle reading was faulty.
         */
        bool readingError = false;

        /**
         * @brief Virtual destructor for the base class.
         */
        virtual ~SwerveAbsoluteEncoder() = default;

        /**
         * @brief Reset the encoder to factory defaults.
         */
        virtual void FactoryDefault() = 0;

        /**
         * @brief Clear sticky faults on the encoder.
         */
        virtual void ClearStickyFaults() = 0;

        /**
         * @brief Configure the absolute encoder to read from [0, 360) per second.
         *
         * @param inverted Whether the encoder is inverted.
         */
        virtual void Configure(bool inverted) = 0;

        /**
         * @brief Get the absolute position of the encoder.
         *
         * @return Absolute position in degrees from [0, 360).
         */
        virtual double GetAbsolutePosition() = 0;

        /**
         * @brief Get the instantiated absolute encoder Object.
         *
         * @return Pointer to the absolute encoder object.
         */
        virtual void *GetAbsoluteEncoder() const = 0;

        /**
         * @brief Sets the Absolute Encoder offset at the Encoder Level.
         *
         * @param offset The offset the Absolute Encoder uses as the zero point in degrees.
         * @return True if setting the Absolute Encoder Offset was successful, false otherwise.
         */
        virtual bool SetAbsoluteEncoderOffset(double offset) = 0;

        /**
         * @brief Get the velocity in degrees/sec.
         *
         * @return Velocity in degrees/sec.
         */
        virtual double GetVelocity() = 0;
    };
} // namespace yagsl