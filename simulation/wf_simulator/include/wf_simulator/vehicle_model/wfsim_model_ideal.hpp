/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 */

/**
 * @file wfsim_model_ideal.hpp
 * @brief wf_siulator ideal velocity model (no dynamics for desired velocity & anguler-velocity or steering)
 * @author Takamasa Horibe
 * @date 2019.08.17
 */

#ifndef WAYPOINT_FOLLOWER_WF_SIMULATOR_WFSIM_MODEL_IDEAL_H_
#define WAYPOINT_FOLLOWER_WF_SIMULATOR_WFSIM_MODEL_IDEAL_H_

#include "wf_simulator/vehicle_model/wfsim_model_interface.hpp"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

/**
 * @class wf_simulator ideal twist model
 * @brief calculate ideal twist dynamics
 */
class WFSimModelIdealTwist : public WFSimModelInterface
{
public:
  /**
   * @brief constructor
   */
  WFSimModelIdealTwist();

  /**
   * @brief destructor
   */
  ~WFSimModelIdealTwist() = default;

private:
  enum IDX
  {
    X = 0,
    Y,
    YAW,
  };
  enum IDX_U
  {
    VX_DES = 0,
    WZ_DES,
  };

  /**
   * @brief get vehicle position x
   */
  double getX() override;

  /**
   * @brief get vehicle position y
   */
  double getY() override;

  /**
   * @brief get vehicle angle yaw
   */
  double getYaw() override;

  /**
   * @brief get vehicle velocity vx
   */
  double getVx() override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  double getWz() override;

  /**
   * @brief get vehicle steering angle
   */
  double getSteer() override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double& dt) override;

  /**
   * @brief calculate derivative of states with ideal twist model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
};

/**
 * @class wf_simulator ideal steering model
 * @brief calculate ideal steering dynamics
 */
class WFSimModelIdealSteer : public WFSimModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] wheelbase vehicle wheelbase length [m]
   */
  WFSimModelIdealSteer(double wheelbase);

  /**
   * @brief destructor
   */
  ~WFSimModelIdealSteer() = default;

private:
  enum IDX
  {
    X = 0,
    Y,
    YAW,
  };
  enum IDX_U
  {
    VX_DES = 0,
    STEER_DES,
  };

  const double wheelbase_;  //!< @brief vehicle wheelbase length

  /**
   * @brief get vehicle position x
   */
  double getX() override;

  /**
   * @brief get vehicle position y
   */
  double getY() override;

  /**
   * @brief get vehicle angle yaw
   */
  double getYaw() override;

  /**
   * @brief get vehicle velocity vx
   */
  double getVx() override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  double getWz() override;

  /**
   * @brief get vehicle steering angle
   */
  double getSteer() override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double& dt) override;

  /**
   * @brief calculate derivative of states with ideal steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
};

#endif