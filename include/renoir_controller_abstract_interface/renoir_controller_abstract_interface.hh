//
// Copyright 2021,
// Olivier Stasse, CNRS
//
// CNRS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
 

#ifndef RENOIR_CONTROLLER_ABSTRACT_INTERFACE_HH
#define RENOIR_CONTROLLER_ABSTRACT_INTERFACE_HH

#include <map>
#include <string>
#include <vector>

namespace renoir_controller {

class NamedVector {

private:
  std::string name_;
  std::vector<double> values_;

public:
  NamedVector() {}
  ~NamedVector() {}

  const std::string &getName() const { return name_; }

  void setName(const std::string &aname) { name_ = aname; }

  const std::vector<double> &getValues() const { return values_; }

  void setValues(const std::vector<double> &values) { values_ = values; }
};
typedef NamedVector SensorValues;
typedef NamedVector ControlValues;

class AbstractExternalInterface {
public:
  AbstractExternalInterface() {}

  virtual ~AbstractExternalInterface() {}

  virtual void
  setupSetSensors(std::map<std::string, SensorValues> &sensorsIn) = 0;

  virtual void
  nominalSetSensors(std::map<std::string, SensorValues> &sensorsIn) = 0;

  virtual void
  cleanupSetSensors(std::map<std::string, SensorValues> &sensorsIn) = 0;

  virtual void getControl(std::map<std::string, ControlValues> &) = 0;
};
} // namespace renoir_controller

typedef renoir_controller::AbstractExternalInterface *
createExternalInterface_t();
typedef void destroyExternalInterface_t(
    renoir_controller::AbstractExternalInterface *);

#endif
