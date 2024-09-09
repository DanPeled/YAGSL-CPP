#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "yagsl/parser/PIDFConfig.h"
#include "yagsl/encoders/SwerveAbsoluteEncoder.h"

namespace yagsl
{

    class TalonFXSwerve
    {
    public:
        // Constructor
        TalonFXSwerve(ctre::phoenix6::hardware::TalonFX *motor, bool isDriveMotor);

        // Configuration methods
        void factoryDefaults();
        void clearStickyFaults();
        TalonFXSwerve *setAbsoluteEncoder(SwerveAbsoluteEncoder *encoder);
        void configureIntegratedEncoder(double positionConversionFactor);
        void configureCANStatusFrames(int CANStatus1);
        void configureCANStatusFrames(
            int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4,
            int CANStatus8, int CANStatus10, int CANStatus12, int CANStatus13,
            int CANStatus14, int CANStatus21, int CANStatusCurrent);
        void configurePIDF(const PIDFConfig &config);
        void configurePIDWrapping(double minInput, double maxInput);
        void setMotorBrake(bool isBrakeMode);
        void setInverted(bool inverted);
        void burnFlash();

        // Motor control methods
        void set(double percentOutput);
        void setReference(double setpoint, double feedforward);
        void setReference(double setpoint, double feedforward, double position);
        double getVoltage();
        void setVoltage(double voltage);
        double getAppliedOutput();
        double getVelocity();
        double getPosition();
        void setPosition(double position);
        void setVoltageCompensation(double nominalVoltage);
        void setCurrentLimit(int currentLimit);
        void setLoopRampRate(double rampRate);

        // Accessor methods
        void *getMotor();
        bool isAttachedAbsoluteEncoder();

    private:
        ctre::phoenix6::hardware::TalonFX *m_motor;
        bool m_isDriveMotor;
        double m_conversionFactor;
        bool m_factoryDefaultOccurred;
        bool m_absoluteEncoder;

        ctre::phoenix6::configs::TalonFXConfiguration m_configuration;
        ctre::phoenix6::controls::MotionMagicVoltage m_angleVoltageSetter;
        ctre::phoenix6::controls::VelocityVoltage m_velocityVoltageSetter;
    };

} // namespace yagsl
