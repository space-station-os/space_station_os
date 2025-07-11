/*
 * Copyright (C) 2023 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* This file was automatically generated.
 * Do not edit this directly
 */

#include "gz/msgs/Factory.hh"
#include "gz/msgs/MessageFactory.hh"
#include "thermal_controller/MessageTypes.hh"

#include <array>

namespace {
    using NamedFactoryFn = std::pair<std::string, gz::msgs::MessageFactory::FactoryFn>;

    std::array<NamedFactoryFn, 4> kFactoryFunctions = {{
  {"thermal_controller.ThermalNodeData",
    []()->std::unique_ptr<google::protobuf::Message>{return std::make_unique<thermal_controller::ThermalNodeData>();}},
  {"thermal_controller.ThermalNodeData_V",
    []()->std::unique_ptr<google::protobuf::Message>{return std::make_unique<thermal_controller::ThermalNodeData_V>();}},
  {"thermal_controller.ThermalLinkFlow",
    []()->std::unique_ptr<google::protobuf::Message>{return std::make_unique<thermal_controller::ThermalLinkFlow>();}},
  {"thermal_controller.ThermalLinkFlow_V",
    []()->std::unique_ptr<google::protobuf::Message>{return std::make_unique<thermal_controller::ThermalLinkFlow_V>();}},
}};
}  // namespace

namespace thermal_controller {
int RegisterAll() {
  size_t registered = 0;
  for (const auto &entry: kFactoryFunctions) {
    gz::msgs::Factory::Register(entry.first, entry.second);
    registered++;
  }
  return registered;
}

static int kMessagesRegistered = RegisterAll();
}  // namespace thermal_controller
