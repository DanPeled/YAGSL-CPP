#pragma once

#include "SwerveAbsoluteEncoder.h"
#include <redux/sensors/Canandmag.h>

namespace yagsl
{
    /**
     * @brief HELIUM Canandcoder from ReduxRobotics absolute encoder, attached through the CAN bus.
     */
    class CanAndCoderSwerve : public SwerveAbsoluteEncoder
    {
    public:
        /**
         * @brief Create the Canandcoder.
         *
         * @param canid The CAN ID whenever the CANandCoder is operating on the CANBus.
         */
        CanAndCoderSwerve(int canid);

        /**
         * @brief Reset the encoder to factory defaults. This will not clear the stored zero offset.
         */
        void FactoryDefault() override;

        /**
         * @brief Clear sticky faults on the encoder.
         */
        void ClearStickyFaults() override;

        /**
         * @brief Configure the Canandcoder to read from [0, 360) per second.
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
         * @brief Set the offset of the Canandcoder.
         *
         * @param offset The offset the Absolute Encoder uses as the zero point.
         * @return true if setting the zero point succeeded, false otherwise.
         */
        bool SetAbsoluteEncoderOffset(double offset) override;

        /**
         * @brief Get the velocity in degrees/sec.
         *
         * @return Velocity in degrees/sec.
         */
        double GetVelocity() override;

    private:
        redux::sensors::canandmag::Canandmag m_encoder;
    };

} // namespace yagsl
