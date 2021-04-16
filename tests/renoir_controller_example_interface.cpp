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

struct Sensors {
  // Measured angle values at the motor side.
  std::vector<double> motor_angle_;
  // Measured angle at the joint side.
  std::vector<double> joint_angle_;
  // Measured or computed velocities.
  std::vector<double> velocities_;
  // Measured torques.
  std::vector<double> torques_;
  // Reconstructed orientation (from internal IMU).
  std::vector<double> orientation_;
  // Measured linear acceleration
  std::vector<double> accelerometer_;
  // Measured angular velocities
  std::vector<double> gyrometer_;
  // Measured 6D force sensors
  std::vector<double> force_sensors_;
  // Measured motor currents
  std::vector<double> motor_currents_;
  // Measured temperatures
  std::vector<double> temperatures_;

  std::size_t nbDofs_;
  std::size_t nbForceSensors_;
  std::size_t nbIMUs_;
  
  Sensors(std::size_t lNbDofs,
          std::size_t lNbForceSensors,
          std::size_t lNbIMUs)
  {
    nbDofs_ = lNbDofs;
    nbForceSensors_ = lNbForceSensors;
    nbIMUs_ = lNbIMUs;
  }
  
  void init()
  {
    motor_angle_.resize(nbDofs_);
    joint_angle_.resize(nbDofs_);
    velocities_.resize(nbDofs_);
    torques_.resize(nbDofs_);
    orientation_.resize(3*nbIMUs_);
    accelerometer_.resize(3*nbIMUs_);
    gyrometer_.resize(3*nbIMUs_);
    force_sensors_.resize(6*nbForceSensors_);
    motor_currents_.resize(nbDofs_);
    temperatures_.resize(nbDofs_);
  }

  void fillValue()
  {
    for(unsigned int i=0;i<nbDofs_;i++)
    {
      motor_angle_[i] = (double)i;
      joint_angle_[i] = (double)(i+nbDofs_);
      velocities_[i] = (double)(i+2*nbDofs_);
      torques_[i] = (double)(i+3*nbDofs_);
      force_sensors_[i] = (double)(i+4*nbDofs_);
      motor_currents_[i] = (double)(i+5*nbDofs_);
      temperatures_[i] = (double)(i+6*nbDofs_);
    }
    
    for(unsigned int i=0;i<nbIMUs_;i++)
    {
      for (unsigned int j=0;j<3;j++)
      {
        orientation_[j] = (double)(j+i*nbIMUs_*3);
        accelerometer_[j] = (double)(j+i*nbIMUs_*3);
        gyrometer_[j] = (double)(j+i*nbIMUs_*3);
      }
    }    
  }
};


class SimpleExternalInterface: public AbstractExternalInterface {
public:

  Sensors renoirSensorValues_;
  std::vector<double> control_;
  
  SimpleExternalInterface():
      renoirSensorValues_(30,4,1)
  {
    renoirSensorValues_.init();
  }

  virtual ~SimpleExternalInterface() {}

  void fillSensorValues(std::map<std::string, SensorValues> &sensorsIn)
  {
    renoirSensorValues_.fillValue();
    sensorsIn["motor_angle"].setValues(renoirSensorValues_.motor_angle_);
    sensorsIn["joint_angle"].setValues(renoirSensorValues_.joint_angle_);
    sensorsIn["velocities"].setValues(renoirSensorValues_.velocities_);
    sensorsIn["torques"].setValues(renoirSensorValues_.torques_);
    sensorsIn["motor_currents"].setValues(renoirSensorValues_.motor_currents_);
    sensorsIn["temperatures"].setValues(renoirSensorValues_.temperatures_);
  }
  
  virtual void
  setupSetSensors(std::map<std::string, SensorValues> &sensorsIn)
  {
    fillSensorValues(sensorsIn);
  }

  virtual void
  nominalSetSensors(std::map<std::string, SensorValues> &sensorsIn)
  {
    fillSensorValues(sensorsIn);
  }
      

  virtual void
  cleanupSetSensors(std::map<std::string, SensorValues> &sensorsIn)
  {
    fillSensorValues(sensorsIn);
  }
  
  virtual void
  getControl(std::map<std::string, ControlValues> &controlOut)
  {
    controlOut["control"];
  }
};
} // namespace renoir_controller

extern "C" {
renoir_controller::AbstractExternalInterface * createExternalInterface()
{
  return new renoir_controller::SimpleExternalInterface();
}
}

extern "C" {
void destroyExternalInterface(renoir_controller::AbstractExternalInterface * anExtInt)
{
  
  renoir_controller::SimpleExternalInterface *
      aSimpleExtInt =
      dynamic_cast<renoir_controller::SimpleExternalInterface *>
      (anExtInt);
  
  if (aSimpleExtInt!=nullptr)
    delete aSimpleExtInt;
}
}

