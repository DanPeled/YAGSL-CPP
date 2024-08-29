#pragma once

#include "Alert.h"
#include <frc/RobotBase.h>
namespace yagsl
{
    /**
     * Telemetry to describe the SwerveDrive following frc-web-components. (Which
     * follows AdvantageKit)
     */
    class SwerveDriveTelemetry
    {
    public:
        /** Verbosity of telemetry data sent back. */
        enum class TelemetryVerbosity
        {
            /** No telemetry data is sent back. */
            NONE,
            /** Low telemetry data, only post the robot position on the field. */
            LOW,
            /** Medium telemetry data, swerve directory */
            INFO,
            /** Info level + field info */
            POSE,
            /** Full swerve drive data is sent back in both human and machine readable forms. */
            HIGH,
            /** Only send the machine readable data related to swerve drive. */
            MACHINE
        };

    public:
        /** An Alert for if the CAN ID is greater than 40. */
        static const Alert canIdWarning;
        /** An Alert for if there is an I2C lockup issue on the roboRIO. */
        static const Alert i2cLockupWarning;
        /** NavX serial comm issue. */
        static const Alert serialCommsIssueWarning;
        /** The current telemetry verbosity level. */
        static TelemetryVerbosity verbosity;
        /** State of simulation of the Robot, used to optimize retrieval. */
        static bool isSimulation;
        /** The number of swerve modules */
        static std::size_t moduleCount;
        /** The locations of the swerve modules */
        static std::span<const double> wheelLocations;
        /**
         * An array of rotation and velocity values describing the measured state of each swerve module
         */
        static std::span<const double> measuredStates;
        /**
         * An array of rotation and velocity values describing the desired state of each swerve module
         */
        static std::span<const double> desiredStates;
        /** The robot's current rotation based on odometry or gyro readings */
        static double robotRotation;
        /** The maximum achievable speed of the modules, used to adjust the size of the vectors. */
        static double maxSpeed;
        /** The units of the module rotations and robot rotation */
        static std::string rotationUnit;
        /** The distance between the left and right modules. */
        static double sizeLeftRight;
        /** The distance between the front and back modules. */
        static double sizeFrontBack;
        /**
         * The direction the robot should be facing when the "Robot Rotation" is zero or blank. This
         * option is often useful to align with odometry data or match videos. 'up', 'right', 'down' or
         * 'left'
         */
        static std::string forwardDirection;
        /**
         * The maximum achievable angular velocity of the robot. This is used to visualize the angular
         * velocity from the chassis speeds properties.
         */
        static double maxAngularVelocity;
        /**
         * The maximum achievable angular velocity of the robot. This is used to visualize the angular
         * velocity from the chassis speeds properties.
         */
        static std::array<double, 3> measuredChassisSpeeds;
        /** Describes the desired forward, sideways and angular velocity of the robot. */
        static std::array<double, 3> desiredChassisSpeeds;

        /** Upload data to smartdashboard */
        static void UpdateData();
    };
} // namespace yagsl