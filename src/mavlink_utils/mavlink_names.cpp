#include "mavlink_utils/mavlink_names.hpp"

#include <ardupilotmega/mavlink.h>

namespace mavlink_names {

const char* mavResultName(uint8_t result)
{
    switch (result) {
        case MAV_RESULT_ACCEPTED:                       return "ACCEPTED";
        case MAV_RESULT_TEMPORARILY_REJECTED:           return "TEMPORARILY_REJECTED";
        case MAV_RESULT_DENIED:                         return "DENIED";
        case MAV_RESULT_UNSUPPORTED:                    return "UNSUPPORTED";
        case MAV_RESULT_FAILED:                         return "FAILED";
        case MAV_RESULT_IN_PROGRESS:                    return "IN_PROGRESS";
        case MAV_RESULT_CANCELLED:                      return "CANCELLED";
        case MAV_RESULT_COMMAND_LONG_ONLY:              return "COMMAND_LONG_ONLY";
        case MAV_RESULT_COMMAND_INT_ONLY:               return "COMMAND_INT_ONLY";
        case MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME:  return "UNSUPPORTED_MAV_FRAME";
        case MAV_RESULT_NOT_IN_CONTROL:                 return "NOT_IN_CONTROL";
        default:                                        return "UNKNOWN";
    }
}

const char* mavStateName(uint8_t state)
{
    switch (state) {
        case MAV_STATE_UNINIT:             return "UNINIT";
        case MAV_STATE_BOOT:               return "BOOT";
        case MAV_STATE_CALIBRATING:        return "CALIBRATING";
        case MAV_STATE_STANDBY:            return "STANDBY";
        case MAV_STATE_ACTIVE:             return "ACTIVE";
        case MAV_STATE_CRITICAL:           return "CRITICAL";
        case MAV_STATE_EMERGENCY:          return "EMERGENCY";
        case MAV_STATE_POWEROFF:           return "POWEROFF";
        case MAV_STATE_FLIGHT_TERMINATION: return "FLIGHT_TERMINATION";
        default:                           return "UNKNOWN";
    }
}

const char* copterModeName(uint32_t customMode)
{
    switch (customMode) {
        case COPTER_MODE_STABILIZE:    return "STABILIZE";
        case COPTER_MODE_ACRO:         return "ACRO";
        case COPTER_MODE_ALT_HOLD:     return "ALT_HOLD";
        case COPTER_MODE_AUTO:         return "AUTO";
        case COPTER_MODE_GUIDED:       return "GUIDED";
        case COPTER_MODE_LOITER:       return "LOITER";
        case COPTER_MODE_RTL:          return "RTL";
        case COPTER_MODE_CIRCLE:       return "CIRCLE";
        case COPTER_MODE_LAND:         return "LAND";
        case COPTER_MODE_DRIFT:        return "DRIFT";
        case COPTER_MODE_SPORT:        return "SPORT";
        case COPTER_MODE_FLIP:         return "FLIP";
        case COPTER_MODE_AUTOTUNE:     return "AUTOTUNE";
        case COPTER_MODE_POSHOLD:      return "POSHOLD";
        case COPTER_MODE_BRAKE:        return "BRAKE";
        case COPTER_MODE_THROW:        return "THROW";
        case COPTER_MODE_AVOID_ADSB:   return "AVOID_ADSB";
        case COPTER_MODE_GUIDED_NOGPS: return "GUIDED_NOGPS";
        case COPTER_MODE_SMART_RTL:    return "SMART_RTL";
        case COPTER_MODE_FLOWHOLD:     return "FLOWHOLD";
        case COPTER_MODE_FOLLOW:       return "FOLLOW";
        case COPTER_MODE_ZIGZAG:       return "ZIGZAG";
        case COPTER_MODE_SYSTEMID:     return "SYSTEMID";
        case COPTER_MODE_AUTOROTATE:   return "AUTOROTATE";
        case COPTER_MODE_AUTO_RTL:     return "AUTO_RTL";
        case COPTER_MODE_TURTLE:       return "TURTLE";
        default:                       return "UNKNOWN";
    }
}

}  // namespace mavlink_names