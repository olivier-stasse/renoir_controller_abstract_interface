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

// POSIX.1-2001
#include <dlfcn.h>

#include "renoir_controller_abstract_interface.hh"

int main(int argc, char *argv[])
{
  if (argc!=2)
  {
    std::cerr << argv[0] << " waits two arguments" <<std::endl;
    return -1;
  }
  
  createExternalInterface_t* createExtInt = reinterpret_cast<createExternalInterface_t*>(
    reinterpret_cast<long>(dlsym(argv[1], "createExternalInterface")));
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    return -1;
  }

  // Create robot-controller
  renoir_controller::AbstractExternalInterface * anExtInt;
  anExtInt = createExtInt();

  std::map<std::string,renoir_controller::SensorValues> sensorsIn;
  renoir_controller::SensorValues aSetOfSensors[5];
  for(auto i=0;i<5;i++)
    sensorsIn[std::to_string(i)]=aSetOfSensors[i];
  anExtInt->setupSetSensors(sensorsIn);
  anExtInt->nominalSetSensors(sensorsIn);
  anExtInt->cleanupSetSensors(sensorsIn);
  
  std::map<std::string,renoir_controller::ControlValues> controlOut;
  renoir_controller::ControlValues aSetOfControls[5];
  for(auto i=0;i<5;i++)
    controlOut[std::to_string(i)]=aSetOfControls[i];

  anExtInt->getControl(controlOut);
  
}
