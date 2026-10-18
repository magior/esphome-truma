#include "TrumaRoomClimate.h"
#include "esphome/components/truma_inetbox/helpers.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.room_climate";
// Room target range on the CP Plus.
static constexpr float ROOM_TEMPERATURE_MIN = 5;
static constexpr float ROOM_TEMPERATURE_MAX = 30;
static constexpr float ROOM_TEMPERATURE_STEP = 1;
// Home Assistant takes the current step as precision: the room temperature comes in tenths.
static constexpr float ROOM_CURRENT_TEMPERATURE_STEP = 0.1;
// Fan levels named as on the CP Plus. ClimateCall::set_fan_mode() matches the standard fan modes first and
// ignores case, so a custom "high" (or "High") would always resolve to CLIMATE_FAN_HIGH: the High level is
// that standard mode, and only Eco and Boost are custom.
static const char *const FAN_MODE_ECO = "eco";
static const char *const FAN_MODE_BOOST = "boost";

void TrumaRoomClimate::setup() {
  this->set_supported_custom_fan_modes({FAN_MODE_ECO, FAN_MODE_BOOST});
  // Before any target since boot: the switch-on value, the lowest.
  this->last_target_temperature_ = ROOM_SWITCH_ON_TEMPERATURE;
  this->target_temperature = this->last_target_temperature_;
  this->parent_->get_heater()->add_on_message_callback([this](const StatusFrameHeater *status_heater) {
    // Publish updated state
    float target = temp_code_to_decimal(status_heater->target_temp_room);
    if (!std::isnan(target)) {
      this->last_target_temperature_ = target;
    }
    this->target_temperature = this->last_target_temperature_;
    this->current_temperature = temp_code_to_decimal(status_heater->current_temp_room);
    this->mode = std::isnan(target) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;

    switch (status_heater->heating_mode) {
      case HeatingMode::HEATING_MODE_ECO:
      case HeatingMode::HEATING_MODE_VARIO_HEAT_NIGHT:
        this->set_custom_fan_mode_(FAN_MODE_ECO);
        break;
      case HeatingMode::HEATING_MODE_HIGH:
      case HeatingMode::HEATING_MODE_VARIO_HEAT_AUTO:
        this->set_fan_mode_(climate::CLIMATE_FAN_HIGH);
        break;
      case HeatingMode::HEATING_MODE_BOOST:
        this->set_custom_fan_mode_(FAN_MODE_BOOST);
        break;
      default:
        this->set_fan_mode_(climate::CLIMATE_FAN_OFF);
        break;
    }
    this->publish_state();
  });
}

void TrumaRoomClimate::dump_config() { LOG_CLIMATE(TAG, "Truma Room Climate", this); }

void TrumaRoomClimate::control(const climate::ClimateCall &call) {
  auto heater = this->parent_->get_heater();
  if (call.get_target_temperature().has_value() && !call.get_fan_mode().has_value() &&
      !call.has_custom_fan_mode()) {
    float temp = *call.get_target_temperature();
    heater->action_heater_room(static_cast<uint8_t>(temp));
  }

  if (call.get_mode().has_value()) {
    // User requested mode change
    climate::ClimateMode mode = *call.get_mode();
    switch (mode) {
      case climate::CLIMATE_MODE_HEAT:
        // Switch on to the last target shown; a target in this call or a pending one wins.
        if (heater->pending_target_temp_room() == TargetTemp::TARGET_TEMP_OFF &&
            !call.get_target_temperature().has_value()) {
          heater->action_heater_room(static_cast<uint8_t>(this->last_target_temperature_));
        }
        break;
      default:
        heater->action_heater_room(0);
        break;
    }
  }

  // The fan level to set, or HEATING_MODE_OFF to switch off.
  // Eco/High are Night/Auto on a Vario Heat, like the fan mode select.
  optional<HeatingMode> heating_mode;
  if (call.has_custom_fan_mode()) {
    auto fan_mode = call.get_custom_fan_mode();
    if (fan_mode == FAN_MODE_ECO) {
      heating_mode = heater->fan_mode_heating_mode(false);
    } else if (fan_mode == FAN_MODE_BOOST) {
      heating_mode = HeatingMode::HEATING_MODE_BOOST;
    }
  } else if (call.get_fan_mode().has_value()) {
    heating_mode = *call.get_fan_mode() == climate::CLIMATE_FAN_HIGH ? heater->fan_mode_heating_mode(true)
                                                                     : HeatingMode::HEATING_MODE_OFF;
  }

  if (heating_mode.has_value()) {
    if (*heating_mode == HeatingMode::HEATING_MODE_OFF) {
      heater->action_heater_room(0);
      return;
    }
    // While off, switch on to the last target shown.
    float temp = temp_code_to_decimal(heater->pending_target_temp_room(), this->last_target_temperature_);
    if (call.get_target_temperature().has_value()) {
      temp = *call.get_target_temperature();
    }
    if (temp < ROOM_SWITCH_ON_TEMPERATURE) {
      temp = ROOM_SWITCH_ON_TEMPERATURE;
    }
    heater->action_heater_room(static_cast<uint8_t>(temp), *heating_mode);
  }
}

climate::ClimateTraits TrumaRoomClimate::traits() {
  // The capabilities of the climate device
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT});
  // Custom fan modes (eco, boost) are set in setup().
  traits.set_supported_fan_modes({climate::CLIMATE_FAN_OFF, climate::CLIMATE_FAN_HIGH});
  traits.set_visual_min_temperature(ROOM_TEMPERATURE_MIN);
  traits.set_visual_max_temperature(ROOM_TEMPERATURE_MAX);
  traits.set_visual_target_temperature_step(ROOM_TEMPERATURE_STEP);
  traits.set_visual_current_temperature_step(ROOM_CURRENT_TEMPERATURE_STEP);
  return traits;
}
}  // namespace truma_inetbox
}  // namespace esphome