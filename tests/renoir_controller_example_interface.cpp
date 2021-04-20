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

  SimpleExternalInterface():
      renoirSensorValues_(32,4,1)
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
        { 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, // leg_left
          200.0, 200.0, 200.0, 200.0, 200.0, 200.0, // leg_right
          100.0, 100.0, // torso
          100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, // arm_left
          100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, // arm_right
          100.0, 100.0 // head
        };
    std::vector<double> D =
        { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // leg_left
          10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // leg_right
          10.0, 10.0, // torso
          10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm_left
          10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm_right
          10.0, 10.0 // head
        };
    std::vector<double> q_des =
        {
          -0.0003719153883155583, 0.0016525642553218331, -0.0006120915316514704,
          -3.604434580233063e-05, -0.0013897980557870545, -0.0031180336465180744,
          // leg_left

          0.00030263030530882664, -0.005795739121500441, 0.000631813768097992,
          0.0005462775375800402, 0.0004480349016297518, -0.0005367511345174825,
          // leg_right

          -6.071781380518884e-05, -0.005018192485418085, // torso

          -8.070292333956153e-05, 0.08542050935318082, 0.0010527578233344504,
          -0.007906152488359682, -0.0007740858999745269, 0.07921991197398072,
          -0.0006464265245044266, // arm_left
          2.0576993624919127e-05, // gripper_left

          -0.0011334639690288304, -0.08124111133850873, -0.0009592487626945355,
          -0.011901887779979844, 0.0007488445751869997, -0.0901025291605209,
          -0.0019554149510241973, // arm_right
           0.0002111823195515247, // gripper_right

          0.00043208701086725187, -3.863858192553717e-06, // head
        };
    /// Compute a control law.
    for(unsigned i=0; i < control_.size();i++)
    {
      double error =  q_des[i] - renoirSensorValues_.motor_angle_[i] ;
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
