#pragma once

#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cctype>
#include <string>

namespace dynamic_gap
{
    /**
    * \brief Outcome of reading the /man_gap parameter
    */
    enum class ManualModeParam { BOOL, STRING, MISSING, INVALID };

    /**
    * \brief Interpret the /man_gap parameter. Manual selection is off unless the
    * parameter clearly says true.
    * \param hasParam whether the parameter exists
    * \param value parameter value (ignored if hasParam is false)
    * \param manualMode resulting mode
    * \return how the value was interpreted
    */
    inline ManualModeParam parseManualModeParam(const bool & hasParam,
                                                XmlRpc::XmlRpcValue value,
                                                bool & manualMode)
    {
        manualMode = false;

        if (!hasParam)
            return ManualModeParam::MISSING;

        if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
        {
            manualMode = static_cast<bool>(value);
            return ManualModeParam::BOOL;
        }

        if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            std::string text = static_cast<std::string>(value);
            std::transform(text.begin(), text.end(), text.begin(),
                           [](unsigned char c) { return std::tolower(c); });

            if (text == "true" || text == "false")
            {
                manualMode = (text == "true");
                return ManualModeParam::STRING;
            }
        }

        return ManualModeParam::INVALID;
    }

    /**
    * \brief Whether tracking of a click-to-move trajectory has ended
    * \param distToTarget distance from robot to the clicked point
    * \param trackingTime time spent tracking the trajectory
    * \param estimatedTime estimated completion time (distance / max speed)
    * \param tolerance distance at which the target counts as reached
    * \return true once the target is reached, or after twice the estimated time
    */
    inline bool isCreatedTrajDone(const float & distToTarget,
                                  const double & trackingTime,
                                  const double & estimatedTime,
                                  const float & tolerance)
    {
        return distToTarget < tolerance || trackingTime > 2.0 * estimatedTime;
    }
}
