#pragma once

#include "SwerveAbsoluteEncoder.h"
#include "yagsl/telemetry/Alert.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix/StatusCodes.h>

namespace yagsl
{
    /**
     * @brief Swerve Absolute Encoder for CTRE CANCoders.
     */
    class CANCoderSwerve : public SwerveAbsoluteEncoder
    {
    public:
        /**
         * @brief Constructor for the CANCoderSwerve class.
         *
         * @param id CAN ID.
         * @param canbus Optional CAN bus to initialize the CANCoder on.
         */
        CANCoderSwerve(int id, const std::string &canbus = "");

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
         * @brief Sets the Absolute Encoder Offset within the CANcoder's Memory.
         *
         * @param offset The offset the Absolute Encoder uses as the zero point in degrees.
         * @return True if setting the Absolute Encoder Offset was successful, false otherwise.
         */
        bool SetAbsoluteEncoderOffset(double offset) override;

        /**
         * @brief Get the velocity in degrees/sec.
         *
         * @return Velocity in degrees/sec.
         */
        double GetVelocity() override;

    private:
        static const units::time::second_t STATUS_TIMEOUT_SECONDS;

        ctre::phoenix6::hardware::CANcoder m_encoder;
        Alert m_magnetFieldLessThanIdeal;
        Alert m_readingFaulty;
        Alert m_readingIgnored;
        Alert m_cannotSetOffset;
    };
} // namespace yagsl
