#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include "ompl_state_spaces/CurvatureStateSpace.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(int argc, char **argv) {
  ob::StateSpacePtr space(new hc_cc_spaces::CCReedsSheppStateSpace(30.0, 0.5));
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, 0);
  bounds.setLow(1, 0);
  bounds.setHigh(0, 20);
  bounds.setHigh(1, 20);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);
  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
  ob::StateValidityCheckerPtr validityChecker(
      new ob::AllValidStateValidityChecker(si));
  si->setStateValidityChecker(validityChecker);
  si->setup();
  auto *start = space->allocState()->as<ob::SE2StateSpace::StateType>();
  start->setX(1);
  start->setY(1);
  start->setYaw(-M_PI / 2);
  auto *goal = space->allocState()->as<ob::SE2StateSpace::StateType>();
  goal->setX(5);
  goal->setY(2);
  goal->setYaw(M_PI / 2);
  og::PathGeometric path(si, start, goal);
  path.interpolate();
  path.printAsMatrix(std::cout);
  return EXIT_SUCCESS;
}
