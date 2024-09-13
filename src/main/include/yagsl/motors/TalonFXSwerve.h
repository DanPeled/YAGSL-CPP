#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "yagsl/parser/PIDFConfig.h"
#include "yagsl/encoders/SwerveAbsoluteEncoder.h"
#include "yagsl/motors/SwerveMotor.h"

namespace yagsl
{

    class TalonFXSwerve : SwerveMotor
    {
    public:
        /** Wait time for status frames to show up. */
        static constexpr units::time::second_t STATUS_TIMEOUT_SECONDS{0.02};
        /**
         * Constructor for TalonFX swerve motor.
         *
         * @param motor Motor to use.
         * @param isDriveMotor Whether this motor is a drive motor.
         */
        TalonFXSwerve(std::shared_ptr<ctre::phoenix6::hardware::TalonFX> motor, bool isDriveMotor);
        /**
         * Construct the TalonFX swerve motor given the ID and CANBus.
         *
         * @param id ID of the TalonFX on the CANBus.
         * @param canbus CANBus on which the TalonFX is on.
         * @param isDriveMotor Whether the motor is a drive or steering motor.
         */
        TalonFXSwerve(int id, std::string canbus = "", bool isDriveMotor);
        /**
         * Construct the TalonFX swerve motor given the ID.
         *
         * @param id ID of the TalonFX on the canbus.
         * @param isDriveMotor Whether the motor is a drive or steering motor.
         */
        TalonFXSwerve(int id, bool isDriveMotor);

        /** Configure the factory defaults. */
        void FactoryDefaults() override;

        /** Clear the sticky faults on the motor controller. */
        void ClearStickyFaults() override;
        /**
         * Set the absolute encoder to be a compatible absolute encoder.
         *
         * @param encoder The encoder to use.
         */
        SwerveMotor *SetAbsoluteEncoder(std::shared_ptr<SwerveAbsoluteEncoder> encoder) override;
        /**
         * Configure the integrated encoder for the swerve module. Sets the conversion factors for
         * position and velocity.
         *
         * @param positionConversionFactor The conversion factor to apply for position.
         *     <p><br>
         *     Degrees: <br>
         *     <code>
         *                                 360 / (angleGearRatio * encoderTicksPerRotation)
         *                                 </code><br>
         *     <p><br>
         *     Meters:<br>
         *     <code>
         *                                 (Math.PI * wheelDiameter) / (driveGearRatio * encoderTicksPerRotation)
         *                                 </code>
         */
        void ConfigureIntegratedEncoder(double positionConversionFactor) override;
        /**
         * Set the CAN status frames.
         *
         * @param CANStatus1 Applied Motor Output, Fault Information, Limit Switch Information
         */
        void ConfigureCANStatusFrames(int CANStatus1);
        /**
         * Set the CAN status frames.
         *
         * @param CANStatus1 Applied Motor Output, Fault Information, Limit Switch Information
         * @param CANStatus2 Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), Brushed
         *     Supply Current Measurement, Sticky Fault Information
         * @param CANStatus3 Quadrature Information
         * @param CANStatus4 Analog Input, Supply Battery Voltage, Controller Temperature
         * @param CANStatus8 Pulse Width Information
         * @param CANStatus10 Motion Profiling/Motion Magic Information
         * @param CANStatus12 Selected Sensor Position (Aux PID 1), Selected Sensor Velocity (Aux PID 1)
         * @param CANStatus13 PID0 (Primary PID) Information
         * @param CANStatus14 PID1 (Auxiliary PID) Information
         * @param CANStatus21 Integrated Sensor Position (Talon FX), Integrated Sensor Velocity (Talon
         *     FX)
         * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless Stator Current
         *     Measurement
         */
        void ConfigureCANStatusFrames(
            int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4,
            int CANStatus8, int CANStatus10, int CANStatus12, int CANStatus13,
            int CANStatus14, int CANStatus21, int CANStatusCurrent);
        /**
         * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
         *
         * @param config Configuration class holding the PIDF values.
         */
        void ConfigurePIDF(const PIDFConfig &config) override;
        /**
         * Configure the PID wrapping for the position closed loop controller.
         *
         * @param minInput Minimum PID input.
         * @param maxInput Maximum PID input.
         */
        void ConfigurePIDWrapping(double minInput, double maxInput) override;
        /**
         * Set the idle mode.
         *
         * @param isBrakeMode Set the brake mode.
         */
        void SetMotorBrake(bool isBrakeMode) override;
        /**
         * Set the motor to be inverted.
         *
         * @param inverted State of inversion.
         */
        void SetInverted(bool inverted) override;
        /** Save the configurations from flash to EEPROM. */
        void BurnFlash() override;

        /**
         * Set the percentage output.
         *
         * @param percentOutput percent out for the motor controller.
         */
        void Set(double percentOutput) override;
        /**
         * Set the closed loop PID controller reference point.
         *
         * @param setpoint Setpoint in MPS or Angle in degrees.
         * @param feedforward Feedforward in volt-meter-per-second or kV.
         */
        void SetReference(double setpoint, units::voltage::volt_t feedforward) override;
        /**
         * Set the closed loop PID controller reference point.
         *
         * @param setpoint Setpoint in meters per second or angle in degrees.
         * @param feedforward Feedforward in volt-meter-per-second or kV.
         * @param position Only used on the angle motor, the position of the motor in degrees.
         */
        void SetReference(double setpoint, units::voltage::volt_t feedforward, units::angle::degree_t position) override;
        /**
         * Get the voltage output of the motor controller.
         *
         * @return Voltage output.
         */
        units::voltage::volt_t GetVoltage() const override;
        /**
         * Set the voltage of the motor.
         *
         * @param voltage Voltage to set.
         */
        void SetVoltage(units::voltage::volt_t voltage) override;
        /**
         * Get the applied dutycycle output.
         *
         * @return Applied dutycycle output to the motor.
         */
        units::dimensionless::scalar_t GetAppliedOutput() const override;
        /**
         * Get the velocity of the integrated encoder.
         *
         * @return velocity in Meters Per Second, or Degrees per Second.
         */
        units::angular_velocity::turns_per_second_t GetVelocity() const override;
        /**
         * Get the position of the integrated encoder.
         *
         * @return Position in Meters or Degrees.
         */
        units::angle::turn_t GetPosition() const override;
        /**
         * Set the integrated encoder position.
         *
         * @param position Integrated encoder position. Should be angle in degrees or meters.
         */
        void SetPosition(units::angle::turn_t position) override;
        /**
         * Set the voltage compensation for the swerve module motor.
         *
         * @param nominalVoltage Nominal voltage for operation to output to.
         */
        void SetVoltageCompensation(units::voltage::volt_t nominalVoltage) override;
        /**
         * Set the current limit for the swerve drive motor, remember this may cause jumping if used in
         * conjunction with voltage compensation. This is useful to protect the motor from current
         * spikes.
         *
         * @param currentLimit Current limit in AMPS at free speed.
         */
        void SetCurrentLimit(int currentLimit) override;
        /**
         * Set the maximum rate the open/closed loop output can change by.
         *
         * @param rampRate Time in seconds to go from 0 to full throttle.
         */
        void SetLoopRampRate(double rampRate) override;
        /**
         * Get the motor object from the module.
         *
         * @return Motor object.
         */
        void *GetMotor() const override;
        /**
         * Queries whether the absolute encoder is directly attached to the motor controller.
         *
         * @return connected absolute encoder state.
         */
        bool IsAttachedAbsoluteEncoder() const override;

    private:
        std::shared_ptr<ctre::phoenix6::hardware::TalonFX> m_motor;
        bool m_isDriveMotor;
        double m_conversionFactor;
        bool m_factoryDefaultOccurred;
        bool m_absoluteEncoder;

        ctre::phoenix6::configs::TalonFXConfiguration m_configuration;
        ctre::phoenix6::controls::MotionMagicVoltage m_angleVoltageSetter;
        ctre::phoenix6::controls::VelocityVoltage m_velocityVoltageSetter;
    };

} // namespace yagsl
