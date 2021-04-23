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
#include <cmath>

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
    init();
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

};


class SimpleExternalInterface: public AbstractExternalInterface {
public:

  Sensors renoirSensorValues_;
  std::vector<double> control_;
  double accumulated_time_;
  double dt_;

  SimpleExternalInterface():
      renoirSensorValues_(32,4,1),
      accumulated_time_(0.0),
      dt_(0.001)
  {
    renoirSensorValues_.init();
    control_.resize(renoirSensorValues_.motor_angle_.size());
  }

  virtual ~SimpleExternalInterface() {}

  void readSensorValues(std::map<std::string, SensorValues> &sensorsIn)
  {
    renoirSensorValues_.motor_angle_ = sensorsIn["motor-angles"].getValues();
    renoirSensorValues_.joint_angle_ = sensorsIn["joint-angles"].getValues();
    renoirSensorValues_.velocities_ = sensorsIn["velocities"].getValues();
    renoirSensorValues_.torques_ = sensorsIn["torques"].getValues();
    renoirSensorValues_.motor_currents_ = sensorsIn["currents"].getValues();
    renoirSensorValues_.temperatures_ = sensorsIn["temperatures"].getValues();
  }

  virtual void
  setupSetSensors(std::map<std::string, SensorValues> &sensorsIn)
  {
    readSensorValues(sensorsIn);
  }

  virtual void
  nominalSetSensors(std::map<std::string, SensorValues> &sensorsIn)
  {
    readSensorValues(sensorsIn);
  }


  virtual void
  cleanupSetSensors(std::map<std::string, SensorValues> &sensorsIn)
  {
    readSensorValues(sensorsIn);
  }

  virtual void
  getControl(std::map<std::string, ControlValues> &controlOut)
  {
    std::vector<double> P =
        { 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, // leg_left
          5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, // leg_right
          1000.0, 1000.0, // torso
          100.0, 200.0, 50.0, 50.0, 50.0, 50.0, 50.0, // arm_left
          100.0, 200.0, 50.0, 50.0, 50.0, 50.0, 50.0, // arm_right
          300.0, 300.0 // head
        };
    std::vector<double> D =
        { 20.0, 20.0, 20.0, 20.0, 20.0, 2.0, // leg_left
          20.0, 20.0, 20.0, 20.0, 20.0, 1.0, // leg_right
          10.0, 10.0, // torso
          50.0, 50.0, 10.0, 1.0, 1.0, 1.0, 1.0, // arm_left
          50.0, 50.0, 10.0, 1.0, 1.0, 1.0, 1.0, // arm_right
          0.1,   0.1 // head
        };
    std::vector<double> q_des =
        {
          0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708, // leg_left
          0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708, // leg_right
          0.0 ,  0.006761, // torso
          0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,   // arm_left
          0.0, // gripper_left
          -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1, // arm_right
          0.0, // gripper_right
          0.0, 0.0 // head

        };
    /// Compute a control law.
    for(unsigned i=0; i < control_.size();i++)
    {
      double error =  q_des[i] - renoirSensorValues_.motor_angle_[i] ;
      if ( i==17)
      {
        error = (q_des[i] + sin(accumulated_time_)*0.087) -
            renoirSensorValues_.motor_angle_[i];
      }
      double derror = - renoirSensorValues_.velocities_[i] ;
      control_[i] = P[i] * error + D[i] * derror;

      if ( (i==21) || (i==29) || (i==30) || (i==31))
        control_[i] = q_des[i];

      // std::cout << "ang: " << renoirSensorValues_.motor_angle_[i]
      //           << " control_ [" << i
      //           << "] = " << control_[i]
      //           << " " << error
      //           << std::endl;
    }
    controlOut["control"].setValues(control_);
    accumulated_time_ += dt_;
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
