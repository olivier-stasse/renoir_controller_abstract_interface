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
 

#include <map>
#include <string>
#include <vector>
#include <iostream>

#include "renoir_controller_abstract_interface.hh"

namespace renoir_controller {

class SimpleExternalInterface: public AbstractExternalInterface {
public:
  SimpleExternalInterface() {}

  virtual ~SimpleExternalInterface() {}

  virtual void
  setupSetSensors(std::map<std::string, SensorValues> &sensorsIn)
  {
    std::cout << " setupSetSensors - Size of sensorsIn:"
              << sensorsIn.size()
              << std::endl;
  }

  virtual void
  nominalSetSensors(std::map<std::string, SensorValues> &sensorsIn)
  {
    std::cout << " nominalSetSensors - Size of sensorsIn:"
              << sensorsIn.size()
              << std::endl;
  }
      

  virtual void
  cleanupSetSensors(std::map<std::string, SensorValues> &sensorsIn)
  {
    std::cout << " nominalSetSensors - Size of sensorsIn:"
              << sensorsIn.size()
              << std::endl;
  }
  
  virtual void
  getControl(std::map<std::string, ControlValues> &controlOut)
  {
    std::cout << " getControl - Size of controlOut:"
              << controlOut.size()
              << std::endl;
  }
};
} // namespace renoir_controller

renoir_controller::AbstractExternalInterface * createExternalInterface()
{
  renoir_controller::SimpleExternalInterface *aSimpExtInt =
      new renoir_controller::SimpleExternalInterface();
  
  return dynamic_cast<renoir_controller::AbstractExternalInterface *>
      (aSimpExtInt);
}

void destroyExternalInterface(renoir_controller::AbstractExternalInterface * anExtInt)
{
  
  renoir_controller::SimpleExternalInterface *
      aSimpleExtInt =
      dynamic_cast<renoir_controller::SimpleExternalInterface *>
      (anExtInt);
  
  if (aSimpleExtInt!=nullptr)
    delete aSimpleExtInt;
}

