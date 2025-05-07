#pragma once

#include "esphome/core/automation.h"
#include "mcf8316.h"

namespace esphome {
namespace mcf8316 {

class FaultTrigger : public Trigger<MCF8316Component::FaultStatus> {
 public:
  explicit FaultTrigger(MCF8316Component *parent) {
    parent->add_on_fault_callback([this](MCF8316Component::FaultStatus value) { this->trigger(value); });
  }
};

}  // namespace mcf8316
}  // namespace esphome
