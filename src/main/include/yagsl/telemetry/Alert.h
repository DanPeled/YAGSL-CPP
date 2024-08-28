#pragma once

#include <iostream>
#include <units/time.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <wpi/sendable/Sendable.h>
#include <networktables/NetworkTableInstance.h>

/** Class for managing persistent alerts to be sent over NetworkTables. */
class Alert
{
public:
    /** Represents an alert's level of urgency. */
    enum class AlertType
    {
        ERROR,
        ERROR_TRACE,
        WARNING,
        WARNING_TRACE,
        INFO
    };

    /**
     * Creates a new Alert in the default group - "Alerts". If this is the first to be instantiated,
     * the appropriate entries will be added to NetworkTables.
     *
     * @param text Text to be displayed when the alert is active.
     * @param type Alert level specifying urgency.
     */
    Alert(const std::string &text, AlertType type);

    /**
     * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
     * entries will be added to NetworkTables.
     *
     * @param group Group identifier, also used as NetworkTables title
     * @param text Text to be displayed when the alert is active.
     * @param type Alert level specifying urgency.
     */
    Alert(const std::string &group, const std::string &text, AlertType type);

    /**
     * Sets whether the alert should currently be displayed. When activated, the alert text will also
     * be sent to the console.
     *
     * @param active Set the alert as active and report it to the driver station.
     */
    void Set(bool active);

    /**
     * Updates current alert text.
     *
     * @param text The text for the alert.
     */
    void SetText(const std::string &text);

private:
    /** Sendable alert for advantage scope. */
    class SendableAlerts : public wpi::Sendable
    {
    public:
        /** Alert list for sendable. */
        std::vector<Alert *> alerts;

        /**
         * Get alerts based off of type.
         *
         * @param type Type of alert to fetch.
         * @return Active alert strings.
         */
        std::vector<std::string> GetStrings(AlertType type) const;

        void InitSendable(wpi::SendableBuilder &builder) override;
    };

    /** Print the alert message. */
    void PrintAlert(const std::string &text) const;

    /** Group of the alert. */
    static std::map<std::string, std::shared_ptr<SendableAlerts>> groups;

    /** Type of the Alert to raise. */
    const AlertType m_type;

    /** Activation state of alert. */
    bool m_active = false;

    /** When the alert was raised. */
    units::time::second_t m_activeStartTime = units::time::second_t(0.0);

    /** Text of the alert. */
    std::string m_text;
};
