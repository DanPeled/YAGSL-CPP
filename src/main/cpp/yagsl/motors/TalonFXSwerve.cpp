#include "yagsl/motors/TalonFXSwerve.h"
#include "yagsl/telemetry/SwerveDriveTelemetry.h"

namespace yagsl
{
    TalonFXSwerve::TalonFXSwerve(std::shared_ptr<ctre::phoenix6::hardware::TalonFX> motor, bool isDriveMotor) : m_motor(motor), m_isDriveMotor(isDriveMotor), m_angleVoltageSetter(0_deg), m_velocityVoltageSetter(units::angular_velocity::turns_per_second_t{0})
    {
        FactoryDefaults();
        ClearStickyFaults();
    }

    TalonFXSwerve::TalonFXSwerve(int id, std::string canbus, bool isDriveMotor) : TalonFXSwerve(std::make_shared<ctre::phoenix6::hardware::TalonFX>(id, canbus), isDriveMotor)
    {
    }

    TalonFXSwerve::TalonFXSwerve(int id, bool isDriveMotor) : TalonFXSwerve(std::make_shared<ctre::phoenix6::hardware::TalonFX>(id), isDriveMotor)
    {
    }

    void TalonFXSwerve::FactoryDefaults()
    {
        if (!m_factoryDefaultOccurred)
        {
            ctre::phoenix6::configs::TalonFXConfigurator &cfg = m_motor.get()->GetConfigurator();
            m_configuration.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            m_configuration.ClosedLoopGeneral.ContinuousWrap = true;
            cfg.Apply(m_configuration);

            m_angleVoltageSetter.UpdateFreqHz = units::hertz_t{0};
            m_velocityVoltageSetter.UpdateFreqHz = units::hertz_t{0};
        }
    }

    void TalonFXSwerve::ClearStickyFaults()
    {
        m_motor.get()->ClearStickyFaults();
    }

    SwerveMotor *TalonFXSwerve::SetAbsoluteEncoder(std::shared_ptr<SwerveAbsoluteEncoder> encoder)
    {
        // Do not support.
        return this;
    }

    void TalonFXSwerve::ConfigureIntegratedEncoder(double positionConversionFactor)
    {
        ctre::phoenix6::configs::TalonFXConfigurator &cfg = m_motor.get()->GetConfigurator();
        cfg.Refresh(m_configuration);

        positionConversionFactor = 1 / positionConversionFactor;
        if (!isDriveMotor)
        {
            positionConversionFactor *= 360;
        }

        m_conversionFactor = positionConversionFactor;

        m_configuration.MotionMagic = m_configuration.MotionMagic.WithMotionMagicCruiseVelocity(
                                                                     100.0 / positionConversionFactor)
                                          .WithMotionMagicAcceleration(100.0 / positionConversionFactor)
                                          .WithMotionMagicExpo_kV(0.12 * positionConversionFactor)
                                          .WithMotionMagicExpo_kA(0.1);

        m_configuration.Feedback.WithFeedbackSensorSource(ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor)
            .WithSensorToMechanismRatio(positionConversionFactor);

        cfg.Apply(m_configuration);

        ConfigureCANStatusFrames(250);
    }

    void TalonFXSwerve::ConfigureCANStatusFrames(int CANStatus1)
    {
        // Does nothing
        // Commented Java:  motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
    }

    void TalonFXSwerve::ConfigureCANStatusFrames(
        int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4,
        int CANStatus8, int CANStatus10, int CANStatus12, int CANStatus13,
        int CANStatus14, int CANStatus21, int CANStatusCurrent)
    {
        // Commented Java:
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, CANStatus2);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, CANStatus3);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, CANStatus4);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, CANStatus8);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, CANStatus10);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, CANStatus12);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, CANStatus13);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, CANStatus14);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated,
        // CANStatus21);
        //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current,
        // CANStatusCurrent);

        // TODO: Configure Status Frame 2 thru 21 if necessary
        // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html#setting-status-frame-periods
    }

    void TalonFXSwerve::ConfigurePIDF(const parser::PIDFConfig &config)
    {
        ctre::phoenix6::configs::TalonFXConfigurator &cfg = m_motor.get()->GetConfigurator();
        cfg.Refresh(m_configuration.Slot0);
        cfg.Apply(m_configuration.Slot0.WithKP(config.p)
                      .WithKI(config.i)
                      .WithKD(config.d)
                      .WithKS(config.f));

        // Commented Java: configuration.slot0.integralZone = config.iz;
        // configuration.slot0.closedLoopPeakOutput = config.output.max;
    }

    void TalonFXSwerve::ConfigurePIDWrapping(double minInput, double maxInput)
    {
        ctre::phoenix6::configs::TalonFXConfigurator &cfg = m_motor.get()->GetConfigurator();
        cfg.Refresh(m_configuration.ClosedLoopGeneral);
        m_configuration.ClosedLoopGeneral.ContinuousWrap = true;
        cfg.Apply(m_configuration.ClosedLoopGeneral);
    }

    void TalonFXSwerve::SetMotorBrake(bool isBreakMode)
    {
        m_motor.get()->SetNeutralMode(isBreakMode ? ctre::phoenix6::signals::NeutralModeValue::Brake
                                                  : ctre::phoenix6::signals::NeutralModeValue::Coast);
    }

    void TalonFXSwerve::SetInverted(bool inverted)
    {
        // Commented Java: Timer.delay(1);
        m_motor.get()->SetInverted(inverted);
    }

    void TalonFXSwerve::BurnFlash()
    {
        // Do nothing
    }

    void TalonFXSwerve::Set(double precentOutput)
    {
        m_motor.get()->Set(precentOutput);
    }

    void TalonFXSwerve::SetReference(double setpoint, units::voltage::volt_t feedforward)
    {
        SetReference(setpoint, feedforward, GetPosition());
    }

    void TalonFXSwerve::SetReference(double setpoint, units::voltage::volt_t feedforward, units::angle::degree_t position)
    {
        // Commented Java : if (SwerveDriveTelemetry.isSimulation)
        //    {
        //      PhysicsSim.getInstance().run();
        //    }

        if (isDriveMotor)
        {
            m_motor.get()->SetControl(m_velocityVoltageSetter
                                          .WithVelocity(units::angular_velocity::turns_per_second_t{setpoint})
                                          .WithFeedForward(feedforward));
        }
        else
        {
            m_motor.get()->SetControl(m_angleVoltageSetter.WithPosition(units::angle::turn_t{setpoint}));
        }
    }

    units::voltage::volt_t TalonFXSwerve::GetVoltage() const
    {
        return m_motor.get()->GetMotorVoltage().WaitForUpdate(STATUS_TIMEOUT_SECONDS).GetValue();
    }

    void TalonFXSwerve::SetVoltage(units::voltage::volt_t voltage)
    {
        m_motor.get()->SetVoltage(voltage);
    }

    units::dimensionless::scalar_t TalonFXSwerve::GetAppliedOutput() const
    {
        return m_motor.get()->GetDutyCycle().WaitForUpdate(STATUS_TIMEOUT_SECONDS).GetValue();
    }

    units::angular_velocity::turns_per_second_t TalonFXSwerve::GetVelocity() const
    {
        return m_motor.get()->GetVelocity().GetValue();
    }

    units::angle::turn_t TalonFXSwerve::GetPosition() const
    {
        return m_motor.get()->GetPosition().GetValue();
    }

    void TalonFXSwerve::SetPosition(units::angle::turn_t position)
    {
        if (!m_absoluteEncoder && !SwerveDriveTelemetry::isSimulation)
        {
            position = (position.value() > 0) ? units::angle::turn_t(fmod(position.value(), 360) + 360) : position;
            ctre::phoenix6::configs::TalonFXConfigurator &cfg = m_motor.get()->GetConfigurator();
            cfg.SetPosition(position / 360);
        }
    }

    void TalonFXSwerve::SetVoltageCompensation(units::voltage::volt_t nominalVoltage)
    {
        // Do not implement
    }

    void TalonFXSwerve::SetCurrentLimit(int currentLimit)
    {
        ctre::phoenix6::configs::TalonFXConfigurator &cfg = m_motor.get()->GetConfigurator();
        cfg.Refresh(m_configuration.CurrentLimits);
        cfg.Apply(
            m_configuration.CurrentLimits
                .WithStatorCurrentLimit(currentLimit)
                .WithSupplyCurrentLimitEnable(true));
    }

    void TalonFXSwerve::SetLoopRampRate(double rampRate)
    {
        ctre::phoenix6::configs::TalonFXConfigurator &cfg = m_motor.get()->GetConfigurator();
        cfg.Refresh(m_configuration.ClosedLoopRamps);
        cfg.Apply(m_configuration.ClosedLoopRamps.WithVoltageClosedLoopRampPeriod(rampRate));
    }

    void *TalonFXSwerve::GetMotor() const
    {
        return m_motor.get();
    }

    bool TalonFXSwerve::IsAttachedAbsoluteEncoder() const
    {
        return m_absoluteEncoder;
    }
} // namespace yagsl
