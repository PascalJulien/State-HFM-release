#include "base/PlannerSettings.h"

#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <utils/OptimizationObjective.h>

#include <steering_functions/include/ompl_state_spaces/CurvatureStateSpace.hpp>

#include "base/environments/GridMaze.h"
#include "base/environments/PolygonMaze.h"
#include "planners/OMPLControlPlanner.hpp"
#include "planners/OMPLPlanner.hpp"
#include "steer_functions/POSQ/POSQStateSpace.h"
//#include "steer_functions/4WS_POSQ/4WS_POSQStateSpace.h"
//#include "steering_functions/ompl_state_spaces/4WS_DubinsStateSpace.hpp"
#include "steering_functions/4ws_state_space/4ws_state_space.hpp"

#ifdef G1_AVAILABLE
#include "steer_functions/G1Clothoid/ClothoidSteering.hpp"
#endif

namespace oc = ompl::control;
#define DEBUG

PlannerSettings::GlobalSettings global::settings;

template <typename DeterministicStateSamplerT>
ob::StateSamplerPtr allocateHaltonStateSampler(
    const ob::StateSpace *space, std::vector<unsigned int> bases = {}) {
  // specify which deterministic sequence to use, here: HaltonSequence
  // optionally we can specify the bases used for generation
  // (otherwise first dim prime numbers are used)
  if (bases.size() != 0)
    return std::make_shared<DeterministicStateSamplerT>(
        space, std::make_shared<ob::HaltonSequence>(bases.size(), bases));
  else
    return std::make_shared<DeterministicStateSamplerT>(
        space, std::make_shared<ob::HaltonSequence>(space->getDimension()));
}

/**
 * Instrumented state space allows to measure time spent on computing the
 * steer function.
 */
template <typename StateSpaceT>
struct InstrumentedStateSpace : public StateSpaceT {
  using StateSpaceT::StateSpaceT;

  double distance(const ob::State *state1,
                  const ob::State *state2) const override {
    global::settings.ompl.steering_timer.resume();
    double d = StateSpaceT::distance(state1, state2);
    global::settings.ompl.steering_timer.stop();
    return d;
  }

  void interpolate(const ob::State *from, const ob::State *to, double t,
                   ob::State *state) const override {
    global::settings.ompl.steering_timer.resume();
    StateSpaceT::interpolate(from, to, t, state);
    global::settings.ompl.steering_timer.stop();
  }
};

void PlannerSettings::GlobalSettings::ForwardPropagationSettings::
    initializeForwardPropagation() const {
  // Construct the robot state space in which we're planning.
  if (global::settings.forwardpropagation.forward_propagation_type ==
      ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_CAR) {
    std::cout << "Initializing state space for forward propagation KC"
              << std::endl;
    // defining state space
    auto *state_space = new InstrumentedStateSpace<ob::SE2StateSpace>;
    state_space->setBounds(global::settings.environment->bounds());
    ob::StateSpacePtr state_space_ptr(state_space);
    // And adding the control space
    global::settings.ompl.state_space = state_space_ptr;

    // set the bounds for the control space
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1.5);
    bounds.setHigh(1.5);

    auto *control_space = new oc::RealVectorControlSpace(state_space_ptr, 3);
    control_space->setBounds(bounds);
    global::settings.ompl.control_space = oc::ControlSpacePtr(control_space);

    global::settings.ompl.control_space_info =
        std::make_shared<oc::SpaceInformation>(
            global::settings.ompl.state_space,
            global::settings.ompl.control_space);
  } else if (global::settings.forwardpropagation.forward_propagation_type ==
             ForwardPropagation::
                 FORWARD_PROPAGATION_TYPE_KINEMATIC_SINGLE_TRACK) {
    std::cout << "Initializing state space for forward propagation KST"
              << std::endl;
    // defining configuration space
    auto *state_space = new InstrumentedStateSpace<ob::CompoundStateSpace>;
    state_space->addSubspace(std::make_shared<ob::SE2StateSpace>(), 1.);
    state_space->addSubspace(std::make_shared<ob::RealVectorStateSpace>(2),
                             1.0);

    state_space->getSubspace(0)->as<ob::SE2StateSpace>()->setBounds(
        global::settings.environment->bounds());
    ob::StateSpacePtr state_space_ptr(state_space);

    // set the bounds for the control space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1.5);
    bounds.setHigh(1.5);

    // create a control space
    auto *control_space = new oc::RealVectorControlSpace(state_space_ptr, 2);
    control_space->setBounds(bounds);

    state_space->getSubspace(1)->as<ob::RealVectorStateSpace>()->setBounds(
        bounds);

    global::settings.ompl.state_space = state_space_ptr;
    global::settings.ompl.control_space = oc::ControlSpacePtr(control_space);
    global::settings.ompl.control_space_info =
        std::make_shared<oc::SpaceInformation>(
            global::settings.ompl.state_space,
            global::settings.ompl.control_space);

    global::settings.ompl.state_space->as<ob::StateSpace>()
        ->registerDefaultProjection(
            std::make_shared<
                ForwardPropagation::KinematicSingleTrackProjectionEvaluator>(
                global::settings.ompl.state_space.get()));
  } else if (global::settings.forwardpropagation.forward_propagation_type ==
             ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_4WS) {
    std::cout << "Initializing state space for forward propagation 4WS"
              << std::endl;
    // defining state space - same as kinematic car but with 3 control inputs
    auto *state_space = new InstrumentedStateSpace<ob::SE2StateSpace>;
    state_space->setBounds(global::settings.environment->bounds());
    ob::StateSpacePtr state_space_ptr(state_space);
    global::settings.ompl.state_space = state_space_ptr;

    // set the bounds for the control space (v, delta_F, delta_R)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1.5);
    bounds.setHigh(1.5);

    auto *control_space = new oc::RealVectorControlSpace(state_space_ptr, 3);
    control_space->setBounds(bounds);
    global::settings.ompl.control_space = oc::ControlSpacePtr(control_space);

    global::settings.ompl.control_space_info =
        std::make_shared<oc::SpaceInformation>(
            global::settings.ompl.state_space,
            global::settings.ompl.control_space);
  }
}

void PlannerSettings::GlobalSettings::OmplSettings::initializeSampler() const {
  const auto &space_ptr = global::settings.ompl.state_space;
  const auto &steering_type = global::settings.steer.steering_type;
  if (global::settings.ompl.sampler.value() == std::string("halton")) {
    if (steering_type == Steering::STEER_TYPE_REEDS_SHEPP ||
        steering_type == Steering::STEER_TYPE_POSQ ||
        steering_type == Steering::STEER_TYPE_4WS_POSQ ||
        steering_type == Steering::STEER_TYPE_4WS_DUBINS_EXTENDED ||
        steering_type == Steering::STEER_TYPE_4WS_STATE_SPACE ||
        steering_type == Steering::STEER_TYPE_DUBINS ||
        steering_type == Steering::STEER_TYPE_LINEAR) {
      // all of these are derived from SE2StateSpace
      const auto &se2_space_ptr = space_ptr->as<ob::SE2StateSpace>();
      se2_space_ptr->setStateSamplerAllocator(
          [](const ob::StateSpace *space) -> ob::StateSamplerPtr {
            return allocateHaltonStateSampler<ob::SE2DeterministicStateSampler>(
                space);
          });
    } else if (steering_type == Steering::STEER_TYPE_CC_DUBINS ||
               steering_type == Steering::STEER_TYPE_CC_REEDS_SHEPP ||
               steering_type == Steering::STEER_TYPE_HC_REEDS_SHEPP) {
      // all of these are instances of CurvatureStateSpace (template class)
      // which is derived from CompoundStateSpace
      const auto &compound_space_ptr = space_ptr->as<ob::CompoundStateSpace>();

      // set StateSamplerAllocator for subspaces
      // (the default CompoundStateSampler will use these automatically)
      compound_space_ptr->as<ob::RealVectorStateSpace>(0)
          ->setStateSamplerAllocator(
              [](const ob::StateSpace *space) -> ob::StateSamplerPtr {
                return allocateHaltonStateSampler<
                    ob::RealVectorDeterministicStateSampler>(space, {2, 3});
              });
      compound_space_ptr->as<ob::SO2StateSpace>(1)->setStateSamplerAllocator(
          [](const ob::StateSpace *space) -> ob::StateSamplerPtr {
            return allocateHaltonStateSampler<ob::SO2DeterministicStateSampler>(
                space, {5});
          });
      compound_space_ptr->as<ob::RealVectorStateSpace>(2)
          ->setStateSamplerAllocator(
              [](const ob::StateSpace *space) -> ob::StateSamplerPtr {
                return allocateHaltonStateSampler<
                    ob::RealVectorDeterministicStateSampler>(space, {7});
              });
      // TODO: deterministic sampler for discrete space? (is it even used?)
    } else {
      OMPL_ERROR(
          "Selected sampler not supported for selected steering function."
          " Using default sampler instead (i.i.d.).");
    }
  }
}

void PlannerSettings::GlobalSettings::SteerSettings::initializeSteering()
    const {
  // Construct the robot state space in which we're planning.
  if (steering_type == Steering::STEER_TYPE_REEDS_SHEPP)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new InstrumentedStateSpace<ob::ReedsSheppStateSpace>(
            car_turning_radius));
  else if (steering_type == Steering::STEER_TYPE_POSQ)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new InstrumentedStateSpace<POSQStateSpace>());
  //else if (steering_type == Steering::STEER_TYPE_4WS_POSQ)
  //  global::settings.ompl.state_space =
  //      ob::StateSpacePtr(new InstrumentedStateSpace<FOURWS_POSQStateSpace>());
  else if (steering_type == Steering::STEER_TYPE_4WS_DUBINS_EXTENDED)
    global::settings.ompl.state_space = ob::StateSpacePtr(
        new InstrumentedStateSpace<ob::DubinsStateSpace>(car_turning_radius));
  else if (steering_type == Steering::STEER_TYPE_DUBINS)
    global::settings.ompl.state_space = ob::StateSpacePtr(
        new InstrumentedStateSpace<ob::DubinsStateSpace>(car_turning_radius));
  else if (steering_type == Steering::STEER_TYPE_LINEAR)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new InstrumentedStateSpace<ob::SE2StateSpace>);
  else if (steering_type == Steering::STEER_TYPE_CC_DUBINS)
    global::settings.ompl.state_space = ob::StateSpacePtr(
        new InstrumentedStateSpace<hc_cc_spaces::CCDubinsStateSpace>(
            hc_cc.kappa, hc_cc.sigma, hc_cc.resolution));
  else if (steering_type == Steering::STEER_TYPE_CC_REEDS_SHEPP)
    global::settings.ompl.state_space = ob::StateSpacePtr(
        new InstrumentedStateSpace<hc_cc_spaces::CCReedsSheppStateSpace>(
            hc_cc.kappa, hc_cc.sigma, hc_cc.resolution));
  else if (steering_type == Steering::STEER_TYPE_HC_REEDS_SHEPP)
    global::settings.ompl.state_space = ob::StateSpacePtr(
        new InstrumentedStateSpace<hc_cc_spaces::HCReedsSheppStateSpace>(
            hc_cc.kappa, hc_cc.sigma, hc_cc.resolution));
  else if (steering_type == Steering::STEER_TYPE_4WS_STATE_SPACE)
    global::settings.ompl.state_space = ob::StateSpacePtr(
        new InstrumentedStateSpace<hc_cc_spaces::FourWSStateSpace>(
            1.0 / car_turning_radius, sampling_resolution, wheel_base, delta_max));
#ifdef G1_AVAILABLE
  else if (steering_type == Steering::STEER_TYPE_CLOTHOID)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new InstrumentedStateSpace<G1ClothoidStateSpace>());
#else
  else if (steering_type == Steering::STEER_TYPE_CLOTHOID) {
    OMPL_ERROR("G1 Clothoid steering is not available in this release!");
    OMPL_ERROR(
        "Select a steering type other than STEER_TYPE_CLOTHOID in the "
        "global::settings.");
    exit(1);
  }
#endif
  else {
    OMPL_ERROR(
        "Unknown steer function has been defined. The state space is invalid.");
    exit(1);
  }

  global::settings.ompl.state_space->as<ob::SE2StateSpace>()->setBounds(
      global::settings.environment->bounds());

  global::settings.ompl.initializeSampler();

  global::settings.ompl.space_info =
      std::make_shared<ob::SpaceInformation>(global::settings.ompl.state_space);
  auto opt_obj_str = global::settings.ompl.optimization_objective.value();
  if (opt_obj_str == std::string("min_pathlength")) {
    global::settings.ompl.objective =
        std::make_shared<CustomPathLengthOptimizationObjective>(
            global::settings.ompl.space_info);
    global::settings.ompl.objective->setCostThreshold(
        ob::Cost(global::settings.ompl.cost_threshold));
  } else if (opt_obj_str == std::string("max_minclearance")) {
    global::settings.ompl.objective =
        std::make_shared<ob::MaximizeMinClearanceObjective>(
            global::settings.ompl.space_info);
  } else if (opt_obj_str == std::string("min_smoothness")) {
    // note, OMPL's smoothness is better when smaller
    global::settings.ompl.objective =
        std::make_shared<SmoothnessOptimizationObjective>(
            global::settings.ompl.space_info);
  } else if (opt_obj_str == std::string("min_curvature")) {
    // note, OMPL's smoothness is better when smaller
    global::settings.ompl.objective =
        std::make_shared<CurvatureOptimizationObjective>(
            global::settings.ompl.space_info);
  }

#ifdef DEBUG
  std::cout << "global::settings.ompl.state_space->hasDefaultProjection() ? "
            << std::boolalpha
            << global::settings.ompl.state_space->hasDefaultProjection()
            << std::endl;
#endif

  OMPL_INFORM("Initialized steer function %s.",
              Steering::to_string(steering_type).c_str());
}

void PlannerSettings::GlobalSettings::EnvironmentSettings::createEnvironment() {
  if (type.value() == "grid") {
    if (grid.generator.value() == "corridor") {
      global::settings.environment = GridMaze::createRandomCorridor(
          grid.width, grid.height, grid.corridor.radius, grid.corridor.branches,
          grid.seed);
    } else if (grid.generator.value() == "random") {
      global::settings.environment = GridMaze::createRandom(
          grid.width, grid.height, grid.random.obstacle_ratio, grid.seed);
    } else if (grid.generator.value() == "image") {
      global::settings.environment = GridMaze::createFromImage(
          grid.image.source, grid.image.occupancy_threshold,
          grid.image.desired_width, grid.image.desired_height);
      if (!global::settings.environment) {
        exit(1);
      }
      global::settings.env.grid.width = global::settings.environment->width();
      global::settings.env.grid.height = global::settings.environment->height();
    } else {
      OMPL_ERROR("Unknown grid environment generator \"%s\".",
                 grid.generator.value().c_str());
      exit(1);
    }
  } else if (type.value() == "polygon") {
    global::settings.environment = PolygonMaze::loadFromSvg(polygon.source);
  } else {
    OMPL_ERROR("Unknown environment type \"%s\".", type.value().c_str());
    exit(1);
  }
  collision.initializeCollisionModel();
}

void PlannerSettings::GlobalSettings::EnvironmentSettings::CollisionSettings::
    initializeCollisionModel() {
  if (collision_model != robot::ROBOT_POLYGON) return;

  robot_shape = SvgPolygonLoader::load(robot_shape_source)[0];
  robot_shape.value().center();
  robot_shape.value().scale(global::settings.env.polygon.scaling);

  if (robot_shape_scale > 0.) {
    // Explicit placement: a uniform scale and the point of the shape, centred
    // and scaled as above, that acts as the reference point of the vehicle.
    const double s = robot_shape_scale;
    robot_shape.value().scale(s);
    robot_shape.value().translate(
        Point(-robot_reference_x * s, -robot_reference_y * s));
  } else {
    // Default placement: the scale is deduced from the half width of the
    // authored shape, and the reference point is moved to the
    // centre of the rear axle of a 3.6490 m vehicle whose axle sits 0.4845 m
    // from its rear.
    const double fact = robot_shape.value().max().y / 0.35;
    const double offsetX = fact * ((-3.6490 / 2.) + 0.4845);
    const double offsetY = 0.0;
    for (auto &point : robot_shape.value().points) {
      point.x -= offsetX;
      point.y -= offsetY;
    }
  }

  OMPL_INFORM("Loaded polygon robot model from %s with %d vertices.",
              robot_shape_source.value().c_str(),
              robot_shape.value().points.size());
  OMPL_INFORM("\tBounds: [%.2f %.2f] -- [%.2f %.2f]",
              robot_shape.value().min().x, robot_shape.value().min().y,
              robot_shape.value().max().x, robot_shape.value().max().y);
}

PlannerSettings::GlobalSettings::OmplSettings::OmplSettings(const char *name,
                                                            Group *parent)
    : Group(name, parent) {
  retrieveGeometricPlannerParams();
  retrieveControlPlannerParams();
}

void PlannerSettings::GlobalSettings::OmplSettings::retrieveGeometricPlannerParams() {
  // Create dummy state space to create dummy planners for param retrieval
  auto space_ptr = std::make_shared<ob::SE2StateSpace>();
  auto space_info_ptr = std::make_shared<ob::SpaceInformation>(space_ptr);

  // Createa all geometric planners
  std::vector<std::shared_ptr<ob::Planner>> planners;
  planners.emplace_back(std::make_shared<og::RRT>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::SST>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::RRTstar>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::RRTsharp>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::InformedRRTstar>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::SORRTstar>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::BITstar>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::FMT>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::BFMT>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::PRM>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::SBL>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::PRMstar>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::CForest>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::EST>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::KPIECE1>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::STRIDE>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::SPARS>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::SPARStwo>(space_info_ptr));
  planners.emplace_back(std::make_shared<og::PDST>(space_info_ptr));

  // Temporarily decrease log level, to suppress warnings about params not being
  // used anymore.
  ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
  for (auto planner : planners) {
    const auto &param_map = planner->params().getParams();
    for (const auto &[key, value_ptr] : param_map) {
      geometric_planner_settings.value()[planner->getName()][key] = "";
    }
  }
  ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
}

void PlannerSettings::GlobalSettings::OmplSettings::retrieveControlPlannerParams() {
  auto space_ptr = std::make_shared<ob::SE2StateSpace>();
  auto control_space_ptr_ =
      std::make_shared<oc::RealVectorControlSpace>(space_ptr, 2);
  auto control_space_info_ptr =
      std::make_shared<oc::SpaceInformation>(space_ptr, control_space_ptr_);

  // Create all control planners
  std::vector<std::shared_ptr<ob::Planner>> planners;
  planners.emplace_back(std::make_shared<oc::KPIECE1>(control_space_info_ptr));
  planners.emplace_back(std::make_shared<oc::EST>(control_space_info_ptr));
  planners.emplace_back(std::make_shared<oc::SST>(control_space_info_ptr));
  planners.emplace_back(std::make_shared<oc::RRT>(control_space_info_ptr));
  planners.emplace_back(std::make_shared<oc::PDST>(control_space_info_ptr));

  // Temporarily decrease log level, to suppress warnings about params not being
  // used anymore.
  ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
  for (auto planner : planners) {
    const auto &param_map = planner->params().getParams();
    for (const auto &[key, value_ptr] : param_map) {
      control_planner_settings.value()[planner->getName()][key] = "";
    }
  }
  ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
}