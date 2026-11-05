/*********************************************************************
*  Copyright (c) 2017 - for information on the respective copyright
*  owner see the NOTICE file and/or the repository
*
*      https://github.com/hbanzhaf/steering_functions.git
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
*  implied. See the License for the specific language governing
*  permissions and limitations under the License.

*  This source code is derived from the Open Motion Planning Library
*  (OMPL) V 1.3.1 (https://github.com/ompl/ompl).
*  Copyright (c) 2010, Rice University, licensed under the BSD license,
*  cf. 3rd-party-licenses.txt file in the root directory of this source
*  tree.
**********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "steering_functions/4ws_state_space/4ws_state_space.hpp"
#include <map>
#include <tuple>
#include <cmath>
#include <limits>

namespace
{
// The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.
const double RS_EPS = 1e-6;
const double RS_ZERO = 10 * numeric_limits<double>::epsilon();

inline void tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega)
{
  double delta = pify(u - v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
  double t1 = atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
  tau = (t2 < 0) ? pify(t1 + PI) : pify(t1);
  omega = pify(tau - u + v - phi);
}

// formula 8.1 in Reeds-Shepp paper
inline bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v)
{
  polar(x - sin(phi), y - 1. + cos(phi), u, t);
  if (t >= -RS_ZERO)
  {
    v = pify(phi - t);
    if (v >= -RS_ZERO)
    {
      assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
      assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(pify(t + v - phi)) < RS_EPS);
      return true;
    }
  }
  return false;
}
// formula 8.2
inline bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v)
{
  double t1, u1;
  polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.)
  {
    double theta;
    u = sqrt(u1 - 4.);
    theta = atan2(2., u);
    t = pify(t1 + theta);
    v = pify(t - phi);
    assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
    assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(pify(t - v - phi)) < RS_EPS);
    return t >= -RS_ZERO && v >= -RS_ZERO;
  }
  return false;
}
void CSC(double x, double y, double phi, FourWS_State_Space::FourWS_Path &path)
{
  double t, u, v, Lmin = path.length(), L;
  if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[14], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[14], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[15], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[15], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[12], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[12], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[13], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[13], -t, -u, -v);
}
// formula 8.3 / 8.4  *** TYPO IN PAPER ***
inline bool LpRmL(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.)
  {
    u = -2. * asin(.25 * u1);
    t = pify(theta + .5 * u + PI);
    v = pify(phi - t + u);
    assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
    assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(pify(t - u + v - phi)) < RS_EPS);
    return t >= -RS_ZERO && u <= RS_ZERO;
  }
  return false;
}
void CCC(double x, double y, double phi, FourWS_State_Space::FourWS_Path &path)
{
  double t, u, v, Lmin = path.length(), L;
  if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[0], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[0], -t, -u, -v);
    Lmin = L;
  }
  if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[1], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[1], -t, -u, -v);
    Lmin = L;
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[0], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[0], -v, -u, -t);
    Lmin = L;
  }
  if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[1], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[1], -v, -u, -t);
}
// formula 8.7
inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = .25 * (2. + sqrt(xi * xi + eta * eta));
  if (rho <= 1.)
  {
    u = acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) < RS_EPS);
    assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(pify(t - 2 * u - v - phi)) < RS_EPS);
    return t >= -RS_ZERO && v <= RS_ZERO;
  }
  return false;
}
// formula 8.8
inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = (20. - xi * xi - eta * eta) / 16.;
  if (rho >= 0 && rho <= 1)
  {
    u = -acos(rho);
    if (u >= -.5 * PI)
    {
      tauOmega(u, u, xi, eta, phi, t, v);
      assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
      assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(pify(t - v - phi)) < RS_EPS);
      return t >= -RS_ZERO && v >= -RS_ZERO;
    }
  }
  return false;
}
void CCCC(double x, double y, double phi, FourWS_State_Space::FourWS_Path &path)
{
  double t, u, v, Lmin = path.length(), L;
  if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[2], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[2], -t, -u, u, -v);
    Lmin = L;
  }
  if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[3], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[3], -t, -u, u, -v);
    Lmin = L;
  }

  if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[2], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[2], -t, -u, -u, -v);
    Lmin = L;
  }
  if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[3], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[3], -t, -u, -u, -v);
}
// formula 8.9
inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.)
  {
    double r = sqrt(rho * rho - 4.);
    u = 2. - r;
    t = pify(theta + atan2(r, -2.));
    v = pify(phi - .5 * PI - t);
    assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
    assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(pify(t + PI / 2 + v - phi)) < RS_EPS);
    return t >= -RS_ZERO && u <= RS_ZERO && v <= RS_ZERO;
  }
  return false;
}
// formula 8.10
inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.)
  {
    t = theta;
    u = 2. - rho;
    v = pify(t + .5 * PI - phi);
    assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < RS_EPS);
    assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < RS_EPS);
    assert(fabs(pify(t + PI / 2 - v - phi)) < RS_EPS);
    return t >= -RS_ZERO && u <= RS_ZERO && v <= RS_ZERO;
  }
  return false;
}
void CCSC(double x, double y, double phi, FourWS_State_Space::FourWS_Path &path)
{
  double t, u, v, Lmin = path.length() - .5 * PI, L;
  if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path =
        FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[4], t, -.5 * PI, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[4], -t, .5 * PI, -u,
                                                     -v);
    Lmin = L;
  }
  if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path =
        FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[5], t, -.5 * PI, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[5], -t, .5 * PI, -u,
                                                     -v);
    Lmin = L;
  }

  if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path =
        FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[8], t, -.5 * PI, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[8], -t, .5 * PI, -u,
                                                     -v);
    Lmin = L;
  }
  if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path =
        FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[9], t, -.5 * PI, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[9], -t, .5 * PI, -u,
                                                     -v);
    Lmin = L;
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path =
        FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[6], v, u, -.5 * PI, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[6], -v, -u, .5 * PI,
                                                     -t);
    Lmin = L;
  }
  if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path =
        FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[7], v, u, -.5 * PI, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[7], -v, -u, .5 * PI,
                                                     -t);
    Lmin = L;
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[10], v, u, -.5 * PI,
                                                     t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[10], -v, -u,
                                                     .5 * PI, -t);
    Lmin = L;
  }
  if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[11], v, u, -.5 * PI,
                                                     t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[11], -v, -u,
                                                     .5 * PI, -t);
}
// formula 8.11 *** TYPO IN PAPER ***
inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.)
  {
    u = 4. - sqrt(rho * rho - 4.);
    if (u <= RS_ZERO)
    {
      t = pify(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
      v = pify(t - phi);
      assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) < RS_EPS);
      assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(pify(t - v - phi)) < RS_EPS);
      return t >= -RS_ZERO && v >= -RS_ZERO;
    }
  }
  return false;
}
void CCSCC(double x, double y, double phi, FourWS_State_Space::FourWS_Path &path)
{
  double t, u, v, Lmin = path.length() - PI, L;
  if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[16], t, -.5 * PI, u,
                                                     -.5 * PI, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[16], -t, .5 * PI,
                                                     -u, .5 * PI, -v);
    Lmin = L;
  }
  if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[17], t, -.5 * PI, u,
                                                     -.5 * PI, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    path = FourWS_State_Space::FourWS_Path(FourWS_State_Space::fourws_path_type[17], -t, .5 * PI,
                                                     -u, .5 * PI, -v);
}

// Crab path evaluation functions
// Integration safety limits
static const int MAX_INTEGRATION_STEPS = 10000; // Prevent infinite loops
static const double MIN_INTEGRATION_STEP = 1e-6; // Minimum meaningful step size
static const double CRAB_INTEGRATION_FACTOR = 0.5; // Use smaller steps for crab motion (better collision detection)

// Angular deviation penalty factors
static const double ANGULAR_PENALTY_FACTOR = 1.0; // Penalty for large angular deviations
static const double ALIGNMENT_PENALTY_FACTOR = 1.0; // Penalty for poor angular alignment
static const double CRAB_BASE_COST_MULTIPLIER = 1.0; // Base cost multiplier for crab motion
static const double ANGULAR_EPSILON_THRESHOLD = M_PI/36.0; // 5 degrees epsilon tolerance

// Cache and call counters; only reset_crab_cache touches them.
static std::map<std::tuple<double, double, double>, double> distance_cache;
static std::map<std::tuple<double, double, double>, int> call_counts;
static const int MAX_CALLS_PER_QUERY = 100; // Max calls for same query

// Check if crab motion is kinematically feasible within admissible cone
bool is_crab_motion_feasible(double x, double y, double phi, double max_crab_angle) {
    // Handle zero displacement case
    if (fabs(x) < 1e-6 && fabs(y) < 1e-6) {
        return false;  // No movement needed
    }
    
    // Calculate required displacement angle
    double displacement_angle = atan2(y, x);
    
    // Calculate angular difference between vehicle heading and required displacement
    double heading_error = displacement_angle - phi;
    
    // Normalize to [-π, π]
    heading_error = atan2(sin(heading_error), cos(heading_error));
    
    // Check if within admissible cone
    return fabs(heading_error) <= max_crab_angle;
}

// Compute angular deviation penalty for crab motion
double compute_angular_penalty(double x, double y, double phi) {
    // Apply 5-degree epsilon tolerance - no penalty within this threshold
    if (fabs(phi) <= ANGULAR_EPSILON_THRESHOLD) {
        return CRAB_BASE_COST_MULTIPLIER;  // Only base cost, no angular penalty
    }
    
    // Angular deviation penalty based on target orientation mismatch
    double angular_cost = 1.0 + fabs(phi) * ANGULAR_PENALTY_FACTOR;
    
    // Alignment penalty - how well does crab motion align with target direction
    double displacement_angle = atan2(y, x);
    double target_alignment = cos(phi);  // Perfect alignment when phi=0
    double alignment_cost = 1.0 + (1.0 - fabs(target_alignment)) * ALIGNMENT_PENALTY_FACTOR;
    
    // Base cost multiplier for crab motion
    return CRAB_BASE_COST_MULTIPLIER * angular_cost * alignment_cost;
}

// Compute crab motion distance with vehicle constraints
double compute_realistic_crab_distance(double x, double y, double phi, double wheelbase, double max_steer_angle) {
    const double displacement_magnitude = sqrt(x*x + y*y);
    
    if (displacement_magnitude < 1e-6) {
        return std::numeric_limits<double>::infinity();  // No movement needed
    }
    
    // Calculate required displacement angle and heading error
    double displacement_angle = atan2(y, x);
    double heading_error = displacement_angle - phi;
    heading_error = atan2(sin(heading_error), cos(heading_error));
    
    // Check admissible cone - must be within vehicle's crab capability
    // max_steer_angle equals max_crab_angle
    if (fabs(heading_error) > max_steer_angle) {
        return std::numeric_limits<double>::infinity();  // Outside admissible cone
    }
    
    // Apply angular deviation penalty
    double angular_penalty = compute_angular_penalty(x, y, phi);
    double penalized_distance = displacement_magnitude * angular_penalty;
    
    // Apply wheelbase scaling: normalize to default wheelbase (11.04) for consistent scaling
    double wheelbase_scaling_factor = 11.04 / wheelbase;
    penalized_distance *= wheelbase_scaling_factor;
    
    return penalized_distance;
}

void CRAB_ONLY(double x, double y, double phi, FourWS_State_Space::FourWS_Path &path, 
               double wheelbase, double max_steer_angle)
{
    // Input validation
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(phi)) {
        return;
    }
    
    // Compute maximum feasible crab angle
    double min_turning_radius = wheelbase / tan(max_steer_angle);
    double max_crab_angle = atan(wheelbase / min_turning_radius);
    max_crab_angle = std::min(max_crab_angle, max_steer_angle);  // max_steer_angle = max_crab_angle
    
    // Check if crab motion is kinematically feasible
    if (!is_crab_motion_feasible(x, y, phi, max_steer_angle)) {
        return;  // Outside admissible cone - crab motion not possible
    }
    
    // Compute crab distance with angular penalty
    double crab_distance = compute_realistic_crab_distance(x, y, phi, wheelbase, max_steer_angle);
    
    // Validate computed distance
    if (!std::isfinite(crab_distance) || crab_distance <= 0) {
        return;
    }
    
    // Get current best path length
    double current_best = path.length();
    if (!std::isfinite(current_best)) {
        current_best = std::numeric_limits<double>::max();
    }
    
    // Only use crab motion if it is better than current best (includes penalty)
    if (crab_distance < current_best) {
        // Determine crab direction based on lateral displacement
        double displacement_angle = atan2(y, x);
        double heading_error = displacement_angle - phi;
        heading_error = atan2(sin(heading_error), cos(heading_error));
        
        bool left_crab = (heading_error > 0);
        int index = left_crab ? 18 : 19;  // Use crab primitive indices
        
        // Array bounds safety check
        if (index >= 0 && index < 22) {
            // Create crab path
            FourWS_State_Space::FourWS_Path crab_path(FourWS_State_Space::fourws_path_type[index], crab_distance);
            double crab_length = crab_path.length();
            
            // Final validation before accepting
            if (std::isfinite(crab_length) && crab_length >= 0 && crab_length < current_best) {
                path = crab_path;
            }
        }
    }
}

// Reset function to clear cache and counters between runs
void reset_crab_cache() {
  distance_cache.clear();
  call_counts.clear();
}

void CRAB_STRAIGHT(double x, double y, double phi, FourWS_State_Space::FourWS_Path &path)
{
  // Not implemented: adds no path.
  return;
}

FourWS_State_Space::FourWS_Path fourws(double x, double y, double phi)
{
  FourWS_State_Space::FourWS_Path path;
  
  // Reeds-Shepp path families
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  
  // Crab motion with the default parameters
  CRAB_ONLY(x, y, phi, path, 11.04, 0.5);
  
  return path;
}

// Overloaded version that accepts vehicle parameters
FourWS_State_Space::FourWS_Path fourws(double x, double y, double phi, double wheelbase, double delta_max)
{
  FourWS_State_Space::FourWS_Path path;
  
  // Reeds-Shepp path families
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  
  // Crab motion with the given vehicle parameters
  CRAB_ONLY(x, y, phi, path, wheelbase, delta_max);
  
  return path;
}
}

const FourWS_State_Space::FourWS_Path_Segment_Type FourWS_State_Space::fourws_path_type[22][5] = {
  // Reeds-Shepp primitives (0-17)
  { FWS_LEFT, FWS_RIGHT, FWS_LEFT, FWS_NOP, FWS_NOP },         // 0
  { FWS_RIGHT, FWS_LEFT, FWS_RIGHT, FWS_NOP, FWS_NOP },        // 1
  { FWS_LEFT, FWS_RIGHT, FWS_LEFT, FWS_RIGHT, FWS_NOP },       // 2
  { FWS_RIGHT, FWS_LEFT, FWS_RIGHT, FWS_LEFT, FWS_NOP },       // 3
  { FWS_LEFT, FWS_RIGHT, FWS_STRAIGHT, FWS_LEFT, FWS_NOP },    // 4
  { FWS_RIGHT, FWS_LEFT, FWS_STRAIGHT, FWS_RIGHT, FWS_NOP },   // 5
  { FWS_LEFT, FWS_STRAIGHT, FWS_RIGHT, FWS_LEFT, FWS_NOP },    // 6
  { FWS_RIGHT, FWS_STRAIGHT, FWS_LEFT, FWS_RIGHT, FWS_NOP },   // 7
  { FWS_LEFT, FWS_RIGHT, FWS_STRAIGHT, FWS_RIGHT, FWS_NOP },   // 8
  { FWS_RIGHT, FWS_LEFT, FWS_STRAIGHT, FWS_LEFT, FWS_NOP },    // 9
  { FWS_RIGHT, FWS_STRAIGHT, FWS_RIGHT, FWS_LEFT, FWS_NOP },   // 10
  { FWS_LEFT, FWS_STRAIGHT, FWS_LEFT, FWS_RIGHT, FWS_NOP },    // 11
  { FWS_LEFT, FWS_STRAIGHT, FWS_RIGHT, FWS_NOP, FWS_NOP },     // 12
  { FWS_RIGHT, FWS_STRAIGHT, FWS_LEFT, FWS_NOP, FWS_NOP },     // 13
  { FWS_LEFT, FWS_STRAIGHT, FWS_LEFT, FWS_NOP, FWS_NOP },      // 14
  { FWS_RIGHT, FWS_STRAIGHT, FWS_RIGHT, FWS_NOP, FWS_NOP },    // 15
  { FWS_LEFT, FWS_RIGHT, FWS_STRAIGHT, FWS_LEFT, FWS_RIGHT },  // 16
  { FWS_RIGHT, FWS_LEFT, FWS_STRAIGHT, FWS_RIGHT, FWS_LEFT },  // 17
  
  // Crab primitives (18-21)
  { FWS_CRAB_LEFT, FWS_NOP, FWS_NOP, FWS_NOP, FWS_NOP },      // 18 - Pure left crab
  { FWS_CRAB_RIGHT, FWS_NOP, FWS_NOP, FWS_NOP, FWS_NOP },     // 19 - Pure right crab
  { FWS_CRAB_LEFT, FWS_NOP, FWS_NOP, FWS_NOP, FWS_NOP },      // 20 - Backward left crab
  { FWS_CRAB_RIGHT, FWS_NOP, FWS_NOP, FWS_NOP, FWS_NOP }      // 21 - Backward right crab
};

FourWS_State_Space::FourWS_Path::FourWS_Path(const FourWS_Path_Segment_Type *type, double t,
                                                            double u, double v, double w, double x)
  : type_(type)
{
  length_[0] = t;
  length_[1] = u;
  length_[2] = v;
  length_[3] = w;
  length_[4] = x;
  total_length_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
}

FourWS_State_Space::FourWS_Path FourWS_State_Space::fourws(const CCState &state1,
                                                                               const CCState &state2) const
{
  double dx = state2.x - state1.x, dy = state2.y - state1.y, dth = state2.theta - state1.theta;
  double c = cos(state1.theta), s = sin(state1.theta);
  double x = c * dx + s * dy, y = -s * dx + c * dy;
  
  return ::fourws(x * kappa_, y * kappa_, dth, wheelbase_, delta_max_);
}

void FourWS_State_Space::set_filter_parameters(const Motion_Noise &motion_noise,
                                                    const Measurement_Noise &measurement_noise,
                                                    const Controller &controller)
{
  ekf_.set_parameters(motion_noise, measurement_noise, controller);
}

double FourWS_State_Space::get_distance(const CCState &state1, const CCState &state2) const
{
  return kappa_inv_ * this->fourws(state1, state2).length();
}

vector<Control> FourWS_State_Space::get_controls(const CCState &state1, const CCState &state2) const
{
  vector<Control> controls;
  controls.reserve(5);
  FourWS_State_Space::FourWS_Path path = this->fourws(state1, state2);
  
  for (unsigned int i = 0; i < 5; ++i)
  {
    Control control;
    
    switch (path.type_[i])
    {
      case FWS_NOP:
        return controls;
      case FWS_LEFT:
        control.delta_s = kappa_inv_ * path.length_[i];
        control.kappa = kappa_;
        control.sigma = 0.0;
        break;
      case FWS_RIGHT:
        control.delta_s = kappa_inv_ * path.length_[i];
        control.kappa = -kappa_;
        control.sigma = 0.0;
        break;
      case FWS_STRAIGHT:
        control.delta_s = kappa_inv_ * path.length_[i];
        control.kappa = 0.0;
        control.sigma = 0.0;
        break;
      case FWS_CRAB_LEFT:
        control.delta_s = kappa_inv_ * path.length_[i];  // Scale distance like other motions
        control.kappa = 0.0;  // No curvature for crab motion
        control.sigma = delta_max_;  // Use sigma to encode crab angle (positive = left)
        break;
      case FWS_CRAB_RIGHT:
        control.delta_s = kappa_inv_ * path.length_[i];  // Scale distance like other motions
        control.kappa = 0.0;  // No curvature for crab motion
        control.sigma = -delta_max_;  // Negative for right crab
        break;
    }
    controls.push_back(control);
  }
  return controls;
}

vector<CCState> FourWS_State_Space::get_path(const CCState &state1, const CCState &state2) const
{
  vector<Control> controls = get_controls(state1, state2);
  return integrate(state1, controls);
}

vector<State_With_Covariance> FourWS_State_Space::get_path_with_covariance(const State_With_Covariance &state1,
                                                                                const CCState &state2) const
{
  vector<Control> controls = get_controls(state1.state, state2);
  return integrate_with_covariance(state1, controls);
}

vector<CCState> FourWS_State_Space::integrate(const CCState &state, const vector<Control> &controls) const
{
  vector<CCState> path;
  CCState state_curr, state_next;
  // reserve capacity of path
  int n_states(0);
  for (const auto &control : controls)
  {
    double abs_delta_s(fabs(control.delta_s));
    n_states += ceil(abs_delta_s / discretization_);
  }
  path.reserve(n_states + 5);
  // get first state
  state_curr.x = state.x;
  state_curr.y = state.y;
  state_curr.theta = state.theta;

  for (const auto &control : controls)
  {
    double delta_s(control.delta_s);
    double abs_delta_s(fabs(delta_s));
    double s_seg(0.0);
    double integration_step(0.0);
    // push_back current state
    state_curr.kappa = control.kappa;
    state_curr.d = sgn(delta_s);
    path.push_back(state_curr);

    // Use finer discretization for crab motion to improve collision detection
    double effective_discretization = discretization_;
    bool is_crab_motion = (fabs(control.sigma) > get_epsilon());
    if (is_crab_motion) {
      effective_discretization *= CRAB_INTEGRATION_FACTOR;
    }
    
    // Prevent infinite loops with safety limits
    int n = ceil(abs_delta_s / effective_discretization);
    if (n > MAX_INTEGRATION_STEPS) {
      n = MAX_INTEGRATION_STEPS;
      effective_discretization = abs_delta_s / n; // Adjust discretization to fit within limits
    }
    
    for (int i = 0; i < n; ++i)
    {
      // get integration step
      s_seg += effective_discretization;
      if (s_seg > abs_delta_s)
      {
        integration_step = effective_discretization - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      }
      else
      {
        integration_step = effective_discretization;
      }
      
      // Ensure minimum meaningful integration step
      if (integration_step < MIN_INTEGRATION_STEP) {
        break; // Skip too small steps
      }
      state_next = integrate_ODE(state_curr, control, integration_step);
      path.push_back(state_next);
      state_curr = state_next;
    }
  }
  return path;
}

vector<State_With_Covariance> FourWS_State_Space::integrate_with_covariance(const State_With_Covariance &state,
                                                                                 const vector<Control> &controls) const
{
  vector<State_With_Covariance> path_with_covariance;
  State_With_Covariance state_curr, state_pred, state_next;
  // reserve capacity of path
  int n_states(0);
  for (const auto &control : controls)
  {
    double abs_delta_s(fabs(control.delta_s));
    n_states += ceil(abs_delta_s / discretization_);
  }
  path_with_covariance.reserve(n_states + 5);
  // get first state
  state_curr.state.x = state.state.x;
  state_curr.state.y = state.state.y;
  state_curr.state.theta = state.state.theta;
  for (int i = 0; i < 16; i++)
  {
    state_curr.Sigma[i] = state.Sigma[i];
    state_curr.Lambda[i] = state.Lambda[i];
    state_curr.covariance[i] = state.covariance[i];
  }

  for (const auto &control : controls)
  {
    double delta_s(control.delta_s);
    double abs_delta_s(fabs(delta_s));
    double s_seg(0.0);
    double integration_step(0.0);
    // push_back current state
    state_curr.state.kappa = control.kappa;
    state_curr.state.d = sgn(delta_s);
    path_with_covariance.push_back(state_curr);

    for (int i = 0, n = ceil(abs_delta_s / discretization_); i < n; ++i)
    {
      // get integration step
      s_seg += discretization_;
      if (s_seg > abs_delta_s)
      {
        integration_step = discretization_ - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      }
      else
      {
        integration_step = discretization_;
      }
      // predict
      state_pred.state = integrate_ODE(state_curr.state, control, integration_step);
      ekf_.predict(state_curr, control, integration_step, state_pred);
      // update
      state_next.state = state_pred.state;
      ekf_.update(state_pred, state_next);

      path_with_covariance.push_back(state_next);
      state_curr.state = state_next.state;
      for (int i = 0; i < 16; i++)
      {
        state_curr.Sigma[i] = state_next.Sigma[i];
        state_curr.Lambda[i] = state_next.Lambda[i];
        state_curr.covariance[i] = state_next.covariance[i];
      }
    }
  }
  return path_with_covariance;
}

CCState FourWS_State_Space::interpolate(const CCState &state, const vector<Control> &controls, double t) const
{
  CCState state_curr, state_next;
  // get first state
  state_curr.x = state.x;
  state_curr.y = state.y;
  state_curr.theta = state.theta;
  // get arc length at t
  double s_path(0.0);
  double s_inter(0.0);
  for (const auto &control : controls)
  {
    s_path += fabs(control.delta_s);
  }
  if (t <= 0.0)
    return state_curr;
  else if (t > 1.0)
    s_inter = s_path;
  else
    s_inter = t * s_path;

  double s(0.0);
  bool interpolated = false;
  for (const auto &control : controls)
  {
    if (interpolated)
      break;

    double delta_s(control.delta_s);
    double abs_delta_s(fabs(delta_s));
    double s_seg(0.0);
    double integration_step(0.0);
    // update current state
    state_curr.kappa = control.kappa;
    state_curr.d = sgn(delta_s);

    s += abs_delta_s;
    if (s > s_inter)
    {
      abs_delta_s = abs_delta_s - (s - s_inter);
      interpolated = true;
    }

    for (int i = 0, n = ceil(abs_delta_s / discretization_); i < n; ++i)
    {
      // get integration step
      s_seg += discretization_;
      if (s_seg > abs_delta_s)
      {
        integration_step = discretization_ - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      }
      else
      {
        integration_step = discretization_;
      }
      state_next = integrate_ODE(state_curr, control, integration_step);
      state_curr = state_next;
    }
  }
  return state_curr;
}

inline CCState FourWS_State_Space::integrate_ODE(const CCState &state, const Control &control,
                                                    double integration_step) const
{
  CCState state_next;
  double d(sgn(control.delta_s));
  
  // Check for crab motion (sigma != 0 indicates crab angle)
  if (fabs(control.sigma) > get_epsilon())
  {
    // Crab motion: use utility function similar to straight line
    double crab_angle = control.sigma;
    
    // Clamp crab angle to reasonable limits for vehicle physics
    if (fabs(crab_angle) > delta_max_) {
      crab_angle = (crab_angle > 0) ? delta_max_ : -delta_max_;
    }
    
    // Crab kinematics from the utility function end_of_crab_line
    end_of_crab_line(state.x, state.y, state.theta, crab_angle, d, integration_step, &state_next.x, &state_next.y);
    state_next.theta = state.theta; // No rotation in pure crab motion
    state_next.kappa = 0.0;
    state_next.d = d;
  }
  else if (fabs(state.kappa) > get_epsilon())
  {
    end_of_circular_arc(state.x, state.y, state.theta, state.kappa, d, integration_step, &state_next.x, &state_next.y,
                        &state_next.theta);
    state_next.kappa = state.kappa;
    state_next.d = d;
  }
  else
  {
    end_of_straight_line(state.x, state.y, state.theta, d, integration_step, &state_next.x, &state_next.y);
    state_next.theta = state.theta;
    state_next.kappa = state.kappa;
    state_next.d = d;
  }
  return state_next;
}
