#pragma once

#include <memory>
#include "yagsl/encoders/SwerveAbsoluteEncoder.h"
#include "yagsl/parser/PIDFConfig.h"

namespace yagsl
{
    /**
     * Swerve motor abstraction that defines a standard interface for motors within a swerve module.
     */
    class SwerveMotor
    {
    public:
        /**
         * Virtual destructor for SwerveMotor.
         */
        virtual ~SwerveMotor() = default;

        /**
         * Get the maximum number of attempts to configure a motor if failures occur.
         */
        static constexpr int MaximumRetries = 5;

        /**
         * Configure the factory defaults.
         */
        virtual void FactoryDefaults() = 0;

        /**
         * Clear the sticky faults on the motor controller.
         */
        virtual void ClearStickyFaults() = 0;

        /**
         * Set the absolute encoder to be a compatible absolute encoder.
         *
         * @param encoder The encoder to use.
         * @return The SwerveMotor instance for single line configuration.
         */
        virtual SwerveMotor &SetAbsoluteEncoder(std::shared_ptr<SwerveAbsoluteEncoder> encoder) = 0;

        /**
         * Configure the integrated encoder for the swerve module. Sets the conversion factors for
         * position and velocity.
         *
         * @param positionConversionFactor The conversion factor to apply for position.
         */
        virtual void ConfigureIntegratedEncoder(double positionConversionFactor) = 0;

        /**
         * Configure the PIDF values for the closed-loop controller. 0 is disabled or off.
         *
         * @param config Configuration class holding the PIDF values.
         */
        virtual void ConfigurePIDF(const PIDFConfig &config) = 0;

        /**
         * Configure the PID wrapping for the position closed-loop controller.
         *
         * @param minInput Minimum PID input.
         * @param maxInput Maximum PID input.
         */
        virtual void ConfigurePIDWrapping(double minInput, double maxInput) = 0;

        /**
         * Set the idle mode.
         *
         * @param isBrakeMode Set the brake mode.
         */
        virtual void SetMotorBrake(bool isBrakeMode) = 0;

        /**
         * Set the motor to be inverted.
         *
         * @param inverted State of inversion.
         */
        virtual void SetInverted(bool inverted) = 0;

        /**
         * Save the configurations from flash to EEPROM.
         */
        virtual void BurnFlash() = 0;

        /**
         * Set the percentage output.
         *
         * @param percentOutput Percent output for the motor controller.
         */
        virtual void Set(double percentOutput) = 0;

        /**
         * Set the closed-loop PID controller reference point.
         *
         * @param setpoint Setpoint in meters per second or angle in degrees.
         * @param feedforward Feedforward in volt-meter-per-second or kV.
         */
        virtual void SetReference(double setpoint, double feedforward) = 0;

        /**
         * Set the closed-loop PID controller reference point.
         *
         * @param setpoint Setpoint in meters per second or angle in degrees.
         * @param feedforward Feedforward in volt-meter-per-second or kV.
         * @param position Only used on the angle motor, the position of the motor in degrees.
         */
        virtual void SetReference(double setpoint, double feedforward, double position) = 0;

        /**
         * Get the voltage output of the motor controller.
         *
         * @return Voltage output.
         */
        virtual double GetVoltage() const = 0;

        /**
         * Set the voltage of the motor.
         *
         * @param voltage Voltage to set.
         */
        virtual void SetVoltage(double voltage) = 0;

        /**
         * Get the applied duty cycle output.
         *
         * @return Applied duty cycle output to the motor.
         */
        virtual double GetAppliedOutput() const = 0;

        /**
         * Get the velocity of the integrated encoder.
         *
         * @return Velocity in meters per second or degrees per second.
         */
        virtual double GetVelocity() const = 0;

        /**
         * Get the position of the integrated encoder.
         *
         * @return Position in meters or degrees.
         */
        virtual double GetPosition() const = 0;

        /**
         * Set the integrated encoder position.
         *
         * @param position Integrated encoder position. Should be angle in degrees or meters per second.
         */
        virtual void SetPosition(double position) = 0;

        /**
         * Set the voltage compensation for the swerve module motor.
         *
         * @param nominalVoltage Nominal voltage for operation to output to.
         */
        virtual void SetVoltageCompensation(double nominalVoltage) = 0;

        /**
         * Set the current limit for the swerve drive motor, remember this may cause jumping if used in
         * conjunction with voltage compensation. This is useful to protect the motor from current
         * spikes.
         *
         * @param currentLimit Current limit in AMPS at free speed.
         */
        virtual void SetCurrentLimit(int currentLimit) = 0;

        /**
         * Set the maximum rate the open/closed loop output can change by.
         *
         * @param rampRate Time in seconds to go from 0 to full throttle.
         */
        virtual void SetLoopRampRate(double rampRate) = 0;

        /**
         * Get the motor object from the module.
         *
         * @return Motor object.
         */
        virtual void *GetMotor() const = 0;

        /**
         * Queries whether the absolute encoder is directly attached to the motor controller.
         *
         * @return Connected absolute encoder state.
         */
        virtual bool IsAttachedAbsoluteEncoder() const = 0;

    protected:
        bool isDriveMotor;
    };

} // namespace yagsl
