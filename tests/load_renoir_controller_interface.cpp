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

typedef std::map<std::string, renoir_controller::SensorValues>
SetOfSensorValues_t;

void fillValue(SetOfSensorValues_t &lsensorsIn)
{
  std::size_t nbDofs = 32;
  std::size_t nbIMUs = 1;
  std::size_t nbForceSensors = 4;
  std::size_t shiftid = 0;
  std::size_t local_size = 6;
  std::vector<double> vecOfVals[10];
  std::vector<std::string> list_of_vec{"motor_angle","joint_angle","velocities",
                                       "torques","motor_currents",
                                       "temperatures","orientation",
                                       "accelerometer",
                                       "gyrometer","force_sensors"};

  // Initialize the vector for each actuator.
  local_size = 6;
  for(unsigned int idvec=0; idvec<local_size;idvec++)
  {
    vecOfVals[idvec].resize(nbDofs);
    for(unsigned int i=0;i<nbDofs;i++)
    {
      vecOfVals[idvec][i] = (double)(i+idvec*nbDofs);
    }

    lsensorsIn[list_of_vec[idvec]].setValues(vecOfVals[idvec]);
  }
  shiftid+=6;

  // Create the vector of IMUS
  for(unsigned int idvec = 3; idvec < 3; idvec++)
  {
    vecOfVals[shiftid+idvec].resize(3*nbIMUs);
    for(unsigned int idimu=0;idimu<nbIMUs;idimu++)
    {
      for (unsigned int j=0;j<3;j++)
      {
        vecOfVals[shiftid+idvec][j+idimu*nbIMUs*3] = (double)(j+idimu*nbIMUs*3);
      }
    }
    lsensorsIn[list_of_vec[shiftid+idvec]].setValues(vecOfVals[shiftid+idvec]);
  }
  shiftid +=3;

  // Create the force sensor vector
  vecOfVals[shiftid].resize(nbForceSensors*6);
  for(unsigned int idvec=0;idvec<nbForceSensors;idvec++)
  {
    for (unsigned int j=0;j<6;j++)
      vecOfVals[shiftid][j+idvec*6] =j+ idvec*6;
  }
  lsensorsIn[list_of_vec[shiftid]].setValues(vecOfVals[shiftid]);
  shiftid ++;

}

int main(int argc, char *argv[])
{
  if (argc!=2)
  {
    std::cerr << argv[0] << " waits two arguments" <<std::endl;
    return -1;
  }


  std::cout << "Calling " << argv[0]
            << " with "
            << argv[1] << std::endl;

  void * ControllerLibrary = dlopen(argv[1], RTLD_LAZY | RTLD_GLOBAL);

  createExternalInterface_t* createExtInt = reinterpret_cast<createExternalInterface_t*>(
    reinterpret_cast<long>(dlsym(ControllerLibrary, "createExternalInterface")));

  std::cout << "Tried loading " << std::endl;

  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    return -1;
  }
  std::cout << "Succedded in loading " << argv[1] << std::endl;

  // Create robot-controller
  renoir_controller::AbstractExternalInterface * anExtInt;
  anExtInt = createExtInt();

  /// This program simulates the robot therefore it provides sensor values.
  std::map<std::string,renoir_controller::SensorValues> sensorsIn;
  fillValue(sensorsIn);

  anExtInt->setupSetSensors(sensorsIn);
  anExtInt->nominalSetSensors(sensorsIn);
  anExtInt->cleanupSetSensors(sensorsIn);

  /// This programs get back the control.
  std::map<std::string,renoir_controller::ControlValues> controlOut;

  anExtInt->getControl(controlOut);

  /// Display the control value.
  const std::vector<double> & controlVec = controlOut["control"].getValues();
  for(unsigned int i=0; i < controlVec.size(); i++)
    std::cout << "control (" << i
              << ")="
              << controlVec[i] << std::endl;

}
