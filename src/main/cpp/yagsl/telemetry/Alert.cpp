#include "yagsl/telemetry/Alert.h"
#include <frc/Errors.h>
#include <wpi/sendable/SendableBuilder.h>

// Initialize static member
std::map<std::string, std::shared_ptr<Alert::SendableAlerts>> Alert::groups;

Alert::Alert() : Alert("", "", AlertType::ERROR) {}

Alert::Alert(const std::string &text, AlertType type)
    : Alert("Alerts", text, type) {}

Alert::Alert(const std::string &group, const std::string &alertText, AlertType alertType)
    : m_type(alertType), m_text(alertText)
{
    if (groups.find(group) == groups.end())
    {
        groups[group] = std::make_shared<SendableAlerts>();
        frc::SmartDashboard::PutData(group, groups[group].get());
    }
    groups[group]->alerts.push_back(this);
}

void Alert::Set(bool active)
{
    if (active && !this->m_active)
    {
        m_activeStartTime = frc::Timer::GetFPGATimestamp();
        PrintAlert(m_text);
    }
    this->m_active = active;
}

void Alert::SetText(const std::string &text)
{
    if (m_active && text != this->m_text)
    {
        PrintAlert(text);
    }
    this->m_text = text;
}

void Alert::PrintAlert(const std::string &text) const
{
    switch (m_type)
    {
    case AlertType::ERROR:
        frc::ReportError(
            0, __FILE__, __LINE__, __func__, text.c_str());
        break;
    case AlertType::ERROR_TRACE:
        frc::ReportError(
            0, __FILE__, __LINE__, __func__, text.c_str(), true);
        break;
    case AlertType::WARNING:
        frc::ReportError(
            frc::warn::Warning, __FILE__, __LINE__, __func__, text.c_str());
        break;
    case AlertType::WARNING_TRACE:
        frc::ReportError(
            frc::warn::Warning, __FILE__, __LINE__, __func__, text.c_str(), true);
        break;
    case AlertType::INFO:
        std::cout << text << std::endl;
        break;
    }
}

std::vector<std::string> Alert::SendableAlerts::GetStrings(AlertType type) const
{
    std::vector<std::string> alertStrings;
    for (const auto &alert : alerts)
    {
        if (alert->m_type == type && alert->m_active)
        {
            alertStrings.push_back(alert->m_text);
        }
    }
    return alertStrings;
}

void Alert::SendableAlerts::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Alerts");
    builder.AddStringArrayProperty(
        "errors", [this]
        { return GetStrings(AlertType::ERROR); },
        nullptr);
    builder.AddStringArrayProperty(
        "errors", [this]
        { return GetStrings(AlertType::ERROR_TRACE); },
        nullptr);
    builder.AddStringArrayProperty(
        "warnings", [this]
        { return GetStrings(AlertType::WARNING); },
        nullptr);
    builder.AddStringArrayProperty(
        "warnings", [this]
        { return GetStrings(AlertType::WARNING_TRACE); },
        nullptr);
    builder.AddStringArrayProperty(
        "infos", [this]
        { return GetStrings(AlertType::INFO); },
        nullptr);
}
