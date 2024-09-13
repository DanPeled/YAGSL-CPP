#include "yagsl/parser/json/modules/AngleConversionFactorsJson.h"
#include "yagsl/telemetry/Alert.h"

namespace yagsl::parser::json::modules
{
    double AngleConversionFactorsJson::calculate()
    {
        if (factor != 0 && gearRatio != 0)
        {
            Alert{
                "Configuration",
                "The given angle conversion factor takes precedence over the composite conversion factor, please remove 'factor' if you want to use the composite factor instead.",
                Alert::AlertType::WARNING}
                .Set(true);
        }
        if (factor == 0)
        {
            // TODO: factor calulcation by gear ratio
        }
        return factor;
    }
} // namespace yagsl::parser::json::modules
