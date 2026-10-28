#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/truma_inetbox/TrumaiNetBoxApp.h"

namespace esphome {
namespace truma_inetbox {
class TrumaRoomClimate : public Component, public climate::Climate, public Parented<TrumaiNetBoxApp> {
 public:
  void setup() override;

  void dump_config() override;

  void control(const climate::ClimateCall &call) override;

  climate::ClimateTraits traits() override;

 protected:
  // Target shown while off: MQTT would publish "nan", which Home Assistant cannot convert.
  float last_target_temperature_{NAN};
};
}  // namespace truma_inetbox
}  // namespace esphome