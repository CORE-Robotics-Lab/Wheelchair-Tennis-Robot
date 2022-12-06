/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ball_localization/ball_ekf.h"
#include "ball_localization/ball_filter_common.h"

#include <XmlRpcException.h>

#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <algorithm>


namespace BallLocalization
{
  Ekf::Ekf(std::vector<double>) :
    FilterBase()  // Must initialize ball_filter base!
  {
  }

  Ekf::~Ekf()
  {
  }

  void Ekf::correct(const Measurement &measurement)
  {
    FB_DEBUG("---------------------- Ekf::correct ----------------------\n" <<
             "State is:\n" << state_ << "\n"
             "Topic is:\n" << measurement.topicName_ << "\n"
             "Measurement is:\n" << measurement.measurement_ << "\n"
             "Measurement topic name is:\n" << measurement.topicName_ << "\n\n"
             "Measurement covariance is:\n" << measurement.covariance_ << "\n");

    // We don't want to update everything, so we need to build matrices that only update
    // the measured parts of our state vector. Throughout prediction and correction, we
    // attempt to maximize efficiency in Eigen.

    // First, determine how many state vector values we're updating
    std::vector<size_t> updateIndices;
    for (size_t i = 0; i < measurement.updateVector_.size(); ++i)
    {
      if (measurement.updateVector_[i])
      {
        // Handle nan and inf values in measurements
        if (std::isnan(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was nan. Excluding from update.\n");
        }
        else if (std::isinf(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was inf. Excluding from update.\n");
        }
        else
        {
          updateIndices.push_back(i);
        }
      }
    }

    FB_DEBUG("Update indices are:\n" << updateIndices << "\n");

    size_t updateSize = updateIndices.size();

    // Now set up the relevant matrices
    Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
    Eigen::VectorXd residualSubset(updateSize);                           // x (in most literature)
    Eigen::VectorXd measurementSubset(updateSize);                        // z
    Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize);  // R
    Eigen::MatrixXd stateToMeasurementSubset(updateSize, state_.rows());  // H
    Eigen::MatrixXd kalmanGainSubset(state_.rows(), updateSize);          // K
    Eigen::VectorXd innovationSubset(updateSize);                         // z - Hx

    stateSubset.setZero();
    residualSubset.setZero();
    measurementSubset.setZero();
    measurementCovarianceSubset.setZero();
    stateToMeasurementSubset.setZero();
    kalmanGainSubset.setZero();
    innovationSubset.setZero();

    // Now build the sub-matrices from the full-sized matrices
    for (size_t i = 0; i < updateSize; ++i)
    {
      measurementSubset(i) = measurement.measurement_(updateIndices[i]);
      stateSubset(i) = state_(updateIndices[i]);

      if (!is_aekf_) {
        for (size_t j = 0; j < updateSize; ++j) {
            measurementCovarianceSubset(i, j) = measurement.covariance_(updateIndices[i], updateIndices[j]);
          }
      }

      // Handle negative (read: bad) covariances in the measurement. Rather
      // than exclude the measurement or make up a covariance, just take
      // the absolute value.
      if (measurementCovarianceSubset(i, i) < 0)
      {
        FB_DEBUG("WARNING: Negative covariance for index " << i <<
                 " of measurement (value is" << measurementCovarianceSubset(i, i) <<
                 "). Using absolute value...\n");

        measurementCovarianceSubset(i, i) = ::fabs(measurementCovarianceSubset(i, i));
      }

      // If the measurement variance for a given variable is very
      // near 0 (as in e-50 or so) and the variance for that
      // variable in the covariance matrix is also near zero, then
      // the Kalman gain computation will blow up. Really, no
      // measurement can be completely without error, so add a small
      // amount in that case.
      if (measurementCovarianceSubset(i, i) < 1e-9)
      {
        FB_DEBUG("WARNING: measurement had very small error covariance for index " << updateIndices[i] <<
                 ". Adding some noise to maintain ball_filter stability.\n");

        measurementCovarianceSubset(i, i) = 1e-9;
      }
    }

    // The state-to-measurement function, h, will now be a measurement_size x full_state_size
    // matrix, with ones in the (i, i) locations of the values to be updated
    for (size_t i = 0; i < updateSize; ++i)
    {
      stateToMeasurementSubset(i, updateIndices[i]) = 1;
    }

    FB_DEBUG("Current state subset is:\n" << stateSubset <<
             "\nMeasurement subset is:\n" << measurementSubset <<
             "\nMeasurement covariance subset is:\n" << measurementCovarianceSubset <<
             "\nState-to-measurement subset is:\n" << stateToMeasurementSubset << "\n");

    // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
    Eigen::MatrixXd pht = estimateErrorCovariance_ * stateToMeasurementSubset.transpose();
    Eigen::MatrixXd hphrInv  = (stateToMeasurementSubset * pht + measurementCovarianceSubset).inverse();
    kalmanGainSubset.noalias() = pht * hphrInv;

    innovationSubset = (measurementSubset - stateSubset);

    // Wrap angles in the innovation
    for (size_t i = 0; i < updateSize; ++i)
    {
      if (updateIndices[i] == StateMemberRoll  ||
          updateIndices[i] == StateMemberPitch ||
          updateIndices[i] == StateMemberYaw)
      {
        while (innovationSubset(i) < -PI)
        {
          innovationSubset(i) += TAU;
        }

        while (innovationSubset(i) > PI)
        {
          innovationSubset(i) -= TAU;
        }
      }
    }

    // (2) Check Mahalanobis distance between mapped measurement and state.
    if (checkMahalanobisThreshold(innovationSubset, hphrInv, measurement.mahalanobisThresh_))
    {
      // (3) Apply the gain to the difference between the state and measurement: x = x + K(z - Hx)
      state_.noalias() += kalmanGainSubset * innovationSubset;

      if (is_aekf_) {
        // (3b) Calculate residual for Position
        residualSubset(0) = measurementSubset(0) - state_(0);
        residualSubset(1) = measurementSubset(1) - state_(1);
        residualSubset(2) = measurementSubset(2) - state_(2);
        
        // (3c) Update measurement covariance
        // The measurementCovariance subset is not R_{t-1} in the AEKF equations. It is our covariance estimate.
        // Our measurement covariances are decent, I left aekf-measurement_alpha_higher.
        measurementCovarianceSubset = aekf_measurement_alpha_ * measurementCovarianceSubset + (1-aekf_measurement_alpha_) * 
                                      (residualSubset * residualSubset.transpose() + stateToMeasurementSubset * pht * stateToMeasurementSubset.transpose());
        
        // Here's the parameter that will actually help us to incorporate model uncertainty.
        // Here processNoiseCovatiance is in fact Q_{t-1}.
        // We aim to alter it, if real dynamics are different enough. So, aekf_noise_alpha is lower.
        processNoiseCovariance_ = aekf_noise_alpha_ * processNoiseCovariance_ + (1 - aekf_noise_alpha_) *
                                      (kalmanGainSubset * innovationSubset * innovationSubset.transpose() * kalmanGainSubset.transpose());
      }

      // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
      Eigen::MatrixXd gainResidual = identity_;
      gainResidual.noalias() -= kalmanGainSubset * stateToMeasurementSubset;
      estimateErrorCovariance_ = gainResidual * estimateErrorCovariance_ * gainResidual.transpose();
      estimateErrorCovariance_.noalias() += kalmanGainSubset *
                                            measurementCovarianceSubset *
                                            kalmanGainSubset.transpose();

      // Handle wrapping of angles
      wrapStateAngles();

      FB_DEBUG("Kalman gain subset is:\n" << kalmanGainSubset <<
               "\nInnovation is:\n" << innovationSubset <<
               "\nCorrected full state is:\n" << state_ <<
               "\nCorrected full estimate error covariance is:\n" << estimateErrorCovariance_ <<
               "\n\n---------------------- /Ekf::correct ----------------------\n");
    }

    state_(StateMemberZ) = std::max(state_(StateMemberZ), 0.0);

  }

  void Ekf::predict(const double referenceTime, const double delta)
  {
    FB_DEBUG("---------------------- Ekf::predict ----------------------\n" <<
             "delta is " << delta << "\n" <<
             "state is " << state_ << "\n");

    const double FLOOR = 0.00;

    if (is_ball && referenceTime >= 0) {

      // This is too small of delta
      if (delta < 1e-9) {
        return;
      }

      // Just lock orientation, I locked it in measurements as well
      state_(StateMemberRoll) = 0;
      state_(StateMemberPitch) = 0;
      state_(StateMemberYaw) = 0;
      state_(StateMemberVroll) = 0;
      state_(StateMemberVpitch) = 0;
      state_(StateMemberVyaw) = 0;
      estimateErrorCovariance_(StateMemberRoll, StateMemberRoll) = 1e-5;
      estimateErrorCovariance_(StateMemberPitch, StateMemberPitch) = 1e-5;
      estimateErrorCovariance_(StateMemberYaw, StateMemberYaw) = 1e-5;
      estimateErrorCovariance_(StateMemberVroll, StateMemberVroll) = 1e-5;
      estimateErrorCovariance_(StateMemberVpitch, StateMemberVpitch) = 1e-5;
      estimateErrorCovariance_(StateMemberVyaw, StateMemberVyaw) = 1e-5;

      // Apply gravity + air resistance
      bool gravity = true;
      if (gravity) {
        const double AIR_DENSITY = 1.225; // kgm^(-3)
        const double GRAVITY = -9.79528;  // Gravity in ATL
        const double BALL_MASS = 0.0575;  // kg
        const double BALL_RADIUS = 0.0335; // m
        
        // Gravity alone
        // state_(StateMemberAx) = 0;
        // state_(StateMemberAy) = 0;
        // state_(StateMemberAz) = -9.81;
        
        // Gravity + drag
        // A_drag = C_d * .5 * rho * A * V_i^2 / M
        // double drag_acc = coeff_drag_ * 0.5 * AIR_DENSITY * (M_PI * pow(BALL_RADIUS, 2)) / BALL_MASS;
        // state_(StateMemberAx) = -std::copysign(1, state_(StateMemberVx)) * drag_acc * pow(state_(StateMemberVx), 2);
        // state_(StateMemberAy) = -std::copysign(1, state_(StateMemberVy)) * drag_acc * pow(state_(StateMemberVy), 2);
        // state_(StateMemberAz) = -9.81 - std::copysign(1, state_(StateMemberVz)) * drag_acc * pow(state_(StateMemberVz), 2);

      // =============================
      //ported fromping pong code. Fixes the drage calculation
      // A_drag = C_d * .5 * rho * A * ||V|| / M
      double v_norm = pow(state_(StateMemberVx), 2) + pow(state_(StateMemberVy), 2) + pow(state_(StateMemberVz), 2);
      if (v_norm > 0 && (sqrt(v_norm)) < 40) {
        double drag_acc = coeff_drag_ * 0.5 * AIR_DENSITY * (M_PI * pow(BALL_RADIUS, 2)) * v_norm / BALL_MASS;
        state_(StateMemberAx) = -std::copysign(1, state_(StateMemberVx)) * drag_acc * state_(StateMemberVx) / sqrt(v_norm);
        state_(StateMemberAy) = -std::copysign(1, state_(StateMemberVy)) * drag_acc * state_(StateMemberVy) / sqrt(v_norm);
        state_(StateMemberAz) = GRAVITY - std::copysign(1, state_(StateMemberVz)) * drag_acc * state_(StateMemberVz) / sqrt(v_norm);  
      } else {
        state_(StateMemberAx) = 0;
        state_(StateMemberAy) = 0;
        state_(StateMemberAz) = GRAVITY;
      }
      //endport
      // ===================================
        
        // Gravity + magnus effect
        // A_lift = C_l * 4/3 * (4 * pi^2 * r^3 * s * rho * V) / M      (Not technically acceleration below, velocity, spin are multiplied individually below)
        // double lift_acc_ = coeff_lift_ * 1.3333 * (4 * pow(M_PI, 2), pow(BALL_RADIUS, 3) * AIR_DENSITY) / BALL_MASS;
        // state_(StateMemberAx) = (state_(StateMemberVy) * state_(StateMemberVyaw) + state_(StateMemberVz) * state_(StateMemberVpitch)) * lift_acc_;
        // state_(StateMemberAy) = (state_(StateMemberVx) * state_(StateMemberVyaw) + state_(StateMemberVz) * state_(StateMemberVroll)) * lift_acc_;
        // state_(StateMemberAz) =  -9.81 + (state_(StateMemberVx) * state_(StateMemberVpitch) + state_(StateMemberVy) * state_(StateMemberVroll)) * lift_acc_;
        

        // Gravity + drag + magnus effect
        // A_lift = C_l * 4/3 * (4 * pi^2 * r^3 * s * rho * V) / M      (Not technically acceleration below, velocity, spin are multiplied individually below)
        // double drag_acc = coeff_drag_ * 0.5 * AIR_DENSITY * (M_PI * pow(BALL_RADIUS, 2)) / BALL_MASS;
        // double lift_acc_ = coeff_lift_ * 1.3333 * (4 * pow(M_PI, 2), pow(BALL_RADIUS, 3) * AIR_DENSITY) / BALL_MASS;
        // state_(StateMemberAx) = (state_(StateMemberVy) * state_(StateMemberVyaw) + state_(StateMemberVz) * state_(StateMemberVpitch)) * lift_acc_ - std::copysign(1, state_(StateMemberVx)) * drag_acc * pow(state_(StateMemberVx), 2);
        // state_(StateMemberAy) = (state_(StateMemberVx) * state_(StateMemberVyaw) + state_(StateMemberVz) * state_(StateMemberVroll)) * lift_acc_ - std::copysign(1, state_(StateMemberVy)) * drag_acc * pow(state_(StateMemberVy), 2);
        // state_(StateMemberAz) = -9.81 + (state_(StateMemberVx) * state_(StateMemberVpitch) + state_(StateMemberVy) * state_(StateMemberVroll)) * lift_acc_ - std::copysign(1, state_(StateMemberVz)) * drag_acc * pow(state_(StateMemberVz), 2);
        
        // Add Skin friction...
      }

      // Under the floor... bounce out I guess
      if (state_(StateMemberZ) <= 0) {
        state_(StateMemberZ) = abs(state_(StateMemberZ));
        state_(StateMemberVz) = abs(state_(StateMemberVz));
      }

      // Going to go be below floor in this step
      if (state_(StateMemberZ) > 0
        && (state_(StateMemberZ) + state_(StateMemberVz)*delta + 0.5*state_(StateMemberAz)*delta*delta) < 0) {
        // Move to floor
        // Find time that it takes to get to floor (quadritic equation to find time of ballistic trajectory)
        double a = 0.5*state_(StateMemberAz);
        double b = state_(StateMemberVz);
        double c = state_(StateMemberZ);
        double floor_delta = std::max( (-b+sqrt(b*b-4*a*c))/(2*a), (-b-sqrt(b*b-4*a*c))/(2*a) );
        predict(-1, floor_delta);
        state_(StateMemberZ) = abs(state_(StateMemberZ));

        // Change velocity
        state_(StateMemberVz) = coeff_restitution_z_ * abs(state_(StateMemberVz));
        state_(StateMemberVx) = coeff_restitution_x_ * state_(StateMemberVx);
        state_(StateMemberVy) = coeff_restitution_y_ * state_(StateMemberVy);
        predict(-1, delta - floor_delta);
        state_(StateMemberZ) = abs(state_(StateMemberZ));
        return;
      }
    }

    double roll = state_(StateMemberRoll);
    double pitch = state_(StateMemberPitch);
    double yaw = state_(StateMemberYaw);
    double xVel = state_(StateMemberVx);
    double yVel = state_(StateMemberVy);
    double zVel = state_(StateMemberVz);
    double pitchVel = state_(StateMemberVpitch);
    double yawVel = state_(StateMemberVyaw);
    double xAcc = state_(StateMemberAx);
    double yAcc = state_(StateMemberAy);
    double zAcc = state_(StateMemberAz);

    // We'll need these trig calculations a lot.
    double sp = ::sin(pitch);
    double cp = ::cos(pitch);
    double cpi = 1.0 / cp;
    double tp = sp * cpi;

    double sr = ::sin(roll);
    double cr = ::cos(roll);

    double sy = ::sin(yaw);
    double cy = ::cos(yaw);

    prepareControl(referenceTime, delta);

    // Prepare the transfer function
    transferFunction_(StateMemberX, StateMemberVx) = cy * cp * delta;
    transferFunction_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
    transferFunction_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
    transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * delta;
    transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * delta;
    transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * delta;
    transferFunction_(StateMemberY, StateMemberVx) = sy * cp * delta;
    transferFunction_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
    transferFunction_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
    transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * delta;
    transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * delta;
    transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * delta;
    transferFunction_(StateMemberZ, StateMemberVx) = -sp * delta;
    transferFunction_(StateMemberZ, StateMemberVy) = cp * sr * delta;
    transferFunction_(StateMemberZ, StateMemberVz) = cp * cr * delta;
    transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * delta;
    transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * delta;
    transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * delta;
    transferFunction_(StateMemberRoll, StateMemberVroll) = delta;
    transferFunction_(StateMemberRoll, StateMemberVpitch) = sr * tp * delta;
    transferFunction_(StateMemberRoll, StateMemberVyaw) = cr * tp * delta;
    transferFunction_(StateMemberPitch, StateMemberVpitch) = cr * delta;
    transferFunction_(StateMemberPitch, StateMemberVyaw) = -sr * delta;
    transferFunction_(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta;
    transferFunction_(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta;
    transferFunction_(StateMemberVx, StateMemberAx) = delta;
    transferFunction_(StateMemberVy, StateMemberAy) = delta;
    transferFunction_(StateMemberVz, StateMemberAz) = delta;

    // Prepare the transfer function Jacobian. This function is analytically derived from the
    // transfer function.
    double xCoeff = 0.0;
    double yCoeff = 0.0;
    double zCoeff = 0.0;
    double oneHalfATSquared = 0.5 * delta * delta;

    yCoeff = cy * sp * cr + sy * sr;
    zCoeff = -cy * sp * sr + sy * cr;
    double dFx_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFR_dR = 1.0 + (cr * tp * pitchVel - sr * tp * yawVel) * delta;

    xCoeff = -cy * sp;
    yCoeff = cy * cp * sr;
    zCoeff = cy * cp * cr;
    double dFx_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFR_dP = (cpi * cpi * sr * pitchVel + cpi * cpi * cr * yawVel) * delta;

    xCoeff = -sy * cp;
    yCoeff = -sy * sp * sr - cy * cr;
    zCoeff = -sy * sp * cr + cy * sr;
    double dFx_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = sy * sp * cr - cy * sr;
    zCoeff = -sy * sp * sr - cy * cr;
    double dFy_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFP_dR = (-sr * pitchVel - cr * yawVel) * delta;

    xCoeff = -sy * sp;
    yCoeff = sy * cp * sr;
    zCoeff = sy * cp * cr;
    double dFy_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    xCoeff = cy * cp;
    yCoeff = cy * sp * sr - sy * cr;
    zCoeff = cy * sp * cr + sy * sr;
    double dFy_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = cp * cr;
    zCoeff = -cp * sr;
    double dFz_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFY_dR = (cr * cpi * pitchVel - sr * cpi * yawVel) * delta;

    xCoeff = -cp;
    yCoeff = -sp * sr;
    zCoeff = -sp * cr;
    double dFz_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFY_dP = (sr * tp * cpi * pitchVel + cr * tp * cpi * yawVel) * delta;

    // Much of the transfer function Jacobian is identical to the transfer function
    transferFunctionJacobian_ = transferFunction_;
    transferFunctionJacobian_(StateMemberX, StateMemberRoll) = dFx_dR;
    transferFunctionJacobian_(StateMemberX, StateMemberPitch) = dFx_dP;
    transferFunctionJacobian_(StateMemberX, StateMemberYaw) = dFx_dY;
    transferFunctionJacobian_(StateMemberY, StateMemberRoll) = dFy_dR;
    transferFunctionJacobian_(StateMemberY, StateMemberPitch) = dFy_dP;
    transferFunctionJacobian_(StateMemberY, StateMemberYaw) = dFy_dY;
    transferFunctionJacobian_(StateMemberZ, StateMemberRoll) = dFz_dR;
    transferFunctionJacobian_(StateMemberZ, StateMemberPitch) = dFz_dP;
    transferFunctionJacobian_(StateMemberRoll, StateMemberRoll) = dFR_dR;
    transferFunctionJacobian_(StateMemberRoll, StateMemberPitch) = dFR_dP;
    transferFunctionJacobian_(StateMemberPitch, StateMemberRoll) = dFP_dR;
    transferFunctionJacobian_(StateMemberYaw, StateMemberRoll) = dFY_dR;
    transferFunctionJacobian_(StateMemberYaw, StateMemberPitch) = dFY_dP;

    FB_DEBUG("Transfer function is:\n" << transferFunction_ <<
             "\nTransfer function Jacobian is:\n" << transferFunctionJacobian_ <<
             "\nProcess noise covariance is:\n" << processNoiseCovariance_ <<
             "\nCurrent state is:\n" << state_ << "\n");

    Eigen::MatrixXd *processNoiseCovariance = &processNoiseCovariance_;

    if (useDynamicProcessNoiseCovariance_)
    {
      computeDynamicProcessNoiseCovariance(state_, delta);
      processNoiseCovariance = &dynamicProcessNoiseCovariance_;
    }

    // (1) Apply control terms, which are actually accelerations
    state_(StateMemberVroll) += controlAcceleration_(ControlMemberVroll) * delta;
    state_(StateMemberVpitch) += controlAcceleration_(ControlMemberVpitch) * delta;
    state_(StateMemberVyaw) += controlAcceleration_(ControlMemberVyaw) * delta;

    state_(StateMemberAx) = (controlUpdateVector_[ControlMemberVx] ?
      controlAcceleration_(ControlMemberVx) : state_(StateMemberAx));
    state_(StateMemberAy) = (controlUpdateVector_[ControlMemberVy] ?
      controlAcceleration_(ControlMemberVy) : state_(StateMemberAy));
    state_(StateMemberAz) = (controlUpdateVector_[ControlMemberVz] ?
      controlAcceleration_(ControlMemberVz) : state_(StateMemberAz));

    // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
    state_ = transferFunction_ * state_;

    // Handle wrapping
    wrapStateAngles();

    FB_DEBUG("Predicted state is:\n" << state_ <<
             "\nCurrent estimate error covariance is:\n" <<  estimateErrorCovariance_ << "\n");

    // (3) Project the error forward: P = J * P * J' + Q
    estimateErrorCovariance_ = (transferFunctionJacobian_ *
                                estimateErrorCovariance_ *
                                transferFunctionJacobian_.transpose());
    estimateErrorCovariance_.noalias() += delta * (*processNoiseCovariance);

    FB_DEBUG("Predicted estimate error covariance is:\n" << estimateErrorCovariance_ <<
             "\n\n--------------------- /Ekf::predict ----------------------\n");


    // Stability Check
    for (int i = 0; i < estimateErrorCovariance_.rows(); i++) {
      for (int j = 0; j < estimateErrorCovariance_.cols(); j++) {
        if (std::isnan(estimateErrorCovariance_(i, j))) {
          estimateErrorCovariance_(i, j) = 1e-9;
          ROS_ERROR_STREAM("Something is EKF Cov broke... Resesting");
        }

        estimateErrorCovariance_(i, j) = std::min(std::max(1e-9, estimateErrorCovariance_(i, j)), 1e9);
      }
    }
    for (int i = 0; i < state_.rows(); i++) {
      if (std::isnan(state_(i))) {
        state_(i) = 0;
        ROS_ERROR_STREAM("Something is Pos EKF broke... Resesting");
      }
    }


  }

}  // namespace BallLocalization
