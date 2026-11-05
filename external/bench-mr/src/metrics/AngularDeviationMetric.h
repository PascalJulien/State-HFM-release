#pragma once

#include "TrajectoryMetric.h"
#include "PathLengthMetric.h"


class AngularDeviationMetric : public TMetric<AngularDeviationMetric> {
 public:
  /**
   * Angular deviation with respect to an explicitly given goal heading.
   *
   * The overloads below read the goal from the global settings, which is fine
   * inside a benchmark run but leaves the metric undefined when it is called
   * on a path produced elsewhere, for instance through the Python bindings.
   * Passing the goal removes that dependency.
   */
  static double evaluateAtGoal(const ompl::geometric::PathGeometric& trajectory,
                               double goalTheta) {
    if (trajectory.getStateCount() < 3) return 0.0;
    const auto* lastState = trajectory.getState(trajectory.getStateCount() - 1)
        ->as<ob::SE2StateSpace::StateType>();
    const auto* ante_lastState = trajectory.getState(trajectory.getStateCount() - 2)
        ->as<ob::SE2StateSpace::StateType>();
    const auto* ante_ante_lastState = trajectory.getState(trajectory.getStateCount() - 3)
        ->as<ob::SE2StateSpace::StateType>();
    const double lastTheta = lastState->getYaw();
    const double ante_lastTheta = ante_lastState->getYaw();
    const double ante_ante_lastTheta = ante_ante_lastState->getYaw();
    auto wrap = [](double d) {
      d = std::abs(d);
      return std::fmin(d, 2 * M_PI - d);
    };
    double thetaDeviation = wrap(lastTheta - goalTheta)
                          + wrap(ante_lastTheta - lastTheta)
                          + wrap(ante_ante_lastTheta - ante_lastTheta);
    const double length = PathLengthMetric::evaluateMetric(trajectory, 0.0);
    const double length_per_points = length / trajectory.getStateCount();
    return thetaDeviation / (3 * length_per_points);
  }

  static double evaluateMetric(const ompl::geometric::PathGeometric& trajectory,double) {
    if (trajectory.getStateCount() < 2) return 0.0;

   
    const auto* lastState = trajectory.getState(trajectory.getStateCount() - 1)->as<ob::SE2StateSpace::StateType>();
    const auto* ante_lastState = trajectory.getState(trajectory.getStateCount() - 2)->as<ob::SE2StateSpace::StateType>();
    const auto* _ante_ante_lastState = trajectory.getState(trajectory.getStateCount() - 3)->as<ob::SE2StateSpace::StateType>();
    double lastTheta = lastState->getYaw();
    double ante_lastTheta = ante_lastState->getYaw();
    double ante_ante_lastTheta = _ante_ante_lastState->getYaw();
    
    double goalTheta = global::settings.environment->goalTheta();

    double thetaDeviation = fmin(std::abs(lastTheta - goalTheta),2*M_PI-std::abs(lastTheta - goalTheta))+
                            fmin(std::abs(ante_lastTheta - lastTheta),2*M_PI-std::abs(ante_lastTheta - lastTheta))+
                            fmin(std::abs(ante_ante_lastTheta - ante_lastTheta),2*M_PI-std::abs(ante_ante_lastTheta - ante_lastTheta));

    double length = PathLengthMetric::evaluateMetric(trajectory, 0.0);
    double length_per_points = length /trajectory.getStateCount();
    thetaDeviation =  thetaDeviation / (3*length_per_points);

    return thetaDeviation;
  }

  static double evaluateMetric(const ompl::control::PathControl& trajectory,double) {
    if (trajectory.getStateCount() < 2) return 0.0;

   
    const auto* lastState = trajectory.getState(trajectory.getStateCount() - 1)->as<ob::SE2StateSpace::StateType>();
    const auto* ante_lastState = trajectory.getState(trajectory.getStateCount() - 2)->as<ob::SE2StateSpace::StateType>();
    const auto* _ante_ante_lastState = trajectory.getState(trajectory.getStateCount() - 3)->as<ob::SE2StateSpace::StateType>();
    double lastTheta = lastState->getYaw();
    double ante_lastTheta = ante_lastState->getYaw();
    double ante_ante_lastTheta = _ante_ante_lastState->getYaw();
    
    double goalTheta = global::settings.environment->goalTheta();
    double thetaDeviation = fmin(std::abs(lastTheta - goalTheta),2*M_PI-std::abs(lastTheta - goalTheta))+
                            fmin(std::abs(ante_lastTheta - lastTheta),2*M_PI-std::abs(ante_lastTheta - lastTheta))+
                            fmin(std::abs(ante_ante_lastTheta - ante_lastTheta),2*M_PI-std::abs(ante_ante_lastTheta - ante_lastTheta));

    double length = PathLengthMetric::evaluateMetric(trajectory, 0.0);
    double length_per_points = length / trajectory.getStateCount();
    thetaDeviation =  thetaDeviation / (3*length_per_points);

    return thetaDeviation;
  }

};