#ifndef _KINEMATIC_4WS_
#define _KINEMATIC_4WS_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace oc = ompl::control;
namespace ob = ompl::base;

namespace Kinematic4WS {
// Four-wheel steering vehicle model
// State: [x, y, theta]
// Control: [v, delta_F, delta_R] where v is velocity, delta_F is front steering angle, delta_R is rear steering angle
// Dynamics: dx = v * cos(theta + delta_R)
//          dy = v * sin(theta + delta_R)
//          dtheta = (v/L) * (tan(delta_F) - tan(delta_R))
//          where L is the wheelbase
inline void kinematic4WSODE(const oc::ODESolver::StateType& q,
                           const oc::Control* control,
                           oc::ODESolver::StateType& qdot) {
  global::settings.ompl.steering_timer.resume();
  double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
  double heading = q[2];
  qdot.resize(q.size(), 0);

  // u[0] = v (velocity)
  // u[1] = delta_F (front steering angle)
  // u[2] = delta_R (rear steering angle)
  
  // Reference point: centre of the rear axle
  qdot[0] = u[0] * cos(heading + u[2]);  // dx = v * cos(theta + delta_R)
  qdot[1] = u[0] * sin(heading + u[2]);  // dy = v * sin(theta + delta_R)
  qdot[2] = u[0] * (tan(u[1]) - tan(u[2])) / global::settings.forwardpropagation.car_length;  // dtheta = (v/L) * (tan(delta_F) - tan(delta_R))
  
  global::settings.ompl.steering_timer.stop();
}

inline void kinematic4WSPostIntegration(const ob::State* /*state*/,
                                       const oc::Control* /*control*/,
                                       const double /*duration*/, ob::State* result) {
  // Normalize orientation between 0 and 2*pi
  ob::SO2StateSpace SO2;
  SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()
                        ->as<ob::SO2StateSpace::StateType>(1));
}

inline void propagate(const oc::SpaceInformation* si, const ob::State* state,
               const oc::Control* control, const double duration,
               ob::State* result) {
  global::settings.ompl.steering_timer.resume();
  static double timeStep = global::settings.forwardpropagation.dt;
  int nsteps = ceil(duration / timeStep);
  double dt = duration / nsteps;
  const double* u =
      control->as<oc::RealVectorControlSpace::ControlType>()->values;

  ob::SE2StateSpace::StateType& se2 =
      *result->as<ob::SE2StateSpace::StateType>();

  si->getStateSpace()->copyState(result, state);

  for (int i = 0; i < nsteps; i++) {
    // u[0] = v, u[1] = delta_F, u[2] = delta_R
    double v = u[0];
    double delta_F = u[1];
    double delta_R = u[2];
    double L = global::settings.forwardpropagation.car_length;

    // Reference point: centre of the rear axle
    se2.setX(se2.getX() + dt * v * cos(se2.getYaw() + delta_R));
    se2.setY(se2.getY() + dt * v * sin(se2.getYaw() + delta_R));
    se2.setYaw(se2.getYaw() + dt * v * (tan(delta_F) - tan(delta_R)) / L);

    if (!si->satisfiesBounds(result)) {
      global::settings.ompl.steering_timer.stop();
      return;
    }
  }

  ob::SO2StateSpace SO2;
  SO2.enforceBounds(se2.as<ob::SO2StateSpace::StateType>(1));
  global::settings.ompl.steering_timer.stop();
}

}  // namespace Kinematic4WS
#endif /* _KINEMATIC_4WS_ */