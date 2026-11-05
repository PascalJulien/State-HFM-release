#pragma once

#include <Eigen/Core>

#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

#include <steering_functions/dubins_state_space/dubins_state_space.hpp>
#include <steering_functions/hc_cc_state_space/cc00_reeds_shepp_state_space.hpp>
#include <steering_functions/hc_cc_state_space/cc_dubins_state_space.hpp>
#include <steering_functions/hc_cc_state_space/hc_cc_state_space.hpp>
#include <steering_functions/hc_cc_state_space/hc_reeds_shepp_state_space.hpp>
#include <steering_functions/reeds_shepp_state_space/reeds_shepp_state_space.hpp>
#include <steering_functions/4ws_state_space/4ws_state_space.hpp>

namespace ob = ompl::base;

namespace hc_cc_spaces {
/**
 * OMPL State Space wrapper for HC/CC state spaces (steer functions).
 */
template <class HC_CC_SPACE, unsigned TYPE>
class CurvatureStateSpace : public ob::CompoundStateSpace {
 public:
  class StateType : public CompoundStateSpace::StateType {
   public:
    StateType() = default;

    double getX() const {
      return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    }

    double getY() const {
      return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
    }

    double getYaw() const { return as<ob::SO2StateSpace::StateType>(1)->value; }

    void setX(double x) {
      as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
    }

    void setY(double y) {
      as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
    }

    void setXY(double x, double y) {
      setX(x);
      setY(y);
    }

    void setYaw(double yaw) {
      as<ob::SO2StateSpace::StateType>(1)->value = yaw;
    }

    /**
     * Curvature at position (x, y).
     */
    double getKappa() const {
      return as<ob::RealVectorStateSpace::StateType>(2)->values[0];
    }

    /**
     * Driving direction {-1, 0, 1}.
     */
    double getD() const {
      return as<ob::DiscreteStateSpace::StateType>(3)->value;
    }

    void setKappa(double v) {
      as<ob::RealVectorStateSpace::StateType>(2)->values[0] = v;
    }

    void setD(double v) {
      as<ob::DiscreteStateSpace::StateType>(3)->value = (int)v;
    }
  };

  CurvatureStateSpace(const HC_CC_SPACE *space) : space_(space) {
    type_ = ob::STATE_SPACE_TYPE_COUNT + TYPE;
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 1.0);
    addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5);
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(1), 1.0);
    addSubspace(std::make_shared<ob::DiscreteStateSpace>(-1, 1), 1.0);
    lock();
    ob::RealVectorBounds bounds(1);
    bounds.setLow(0.);
    bounds.setHigh(1.);
    as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
  }

  ~CurvatureStateSpace() = default;

  double distance(const ob::State *state1,
                  const ob::State *state2) const override {
    return space_->get_distance(fromOMPL(state1), fromOMPL(state2));
  }

  void setBounds(const ob::RealVectorBounds &bounds) {
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
  }

  const ob::RealVectorBounds &getBounds() const {
    return as<ob::RealVectorStateSpace>(0)->getBounds();
  }

  ob::State *allocState() const override {
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
  }

  void freeState(ob::State *state) const override {
    CompoundStateSpace::freeState(state);
  }

  void registerProjections() override {
    class SE2DefaultProjection : public ob::ProjectionEvaluator {
     public:
      SE2DefaultProjection(const StateSpace *space)
          : ProjectionEvaluator(space) {}

      unsigned int getDimension() const override { return 2; }

      void defaultCellSizes() override {
        cellSizes_.resize(2);
        bounds_ = space_->as<ob::SE2StateSpace>()->getBounds();
        cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) /
                        ompl::magic::PROJECTION_DIMENSION_SPLITS;
        cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) /
                        ompl::magic::PROJECTION_DIMENSION_SPLITS;
      }

      void project(const ob::State *state,
                   Eigen::Ref<Eigen::VectorXd> projection) const override {
        projection.resize(2);
        projection[0] = state->as<StateType>()->getX();
        projection[1] = state->as<StateType>()->getY();
      }
    };

    registerDefaultProjection(std::make_shared<SE2DefaultProjection>(this));
  }

  void interpolate(const ob::State *from, const ob::State *to, double t,
                   ob::State *state) const override {
    const CCState state1{fromOMPL(from)}, state2{fromOMPL(to)};
    const auto controls = space_->get_controls(state1, state2);
    CCState result = space_->interpolate(state1, controls, t);
    toOMPL(result, state);
  }

 private:
  const HC_CC_SPACE *space_;

  static CCState fromOMPL(const ob::State *state) {
    const auto *s = state->as<StateType>();
    return {s->getX(), s->getY(), s->getYaw(), s->getKappa(), s->getD()};
  }

  static void toOMPL(const CCState &state, ob::State *result) {
    result->as<StateType>()->setXY(state.x, state.y);
    result->as<StateType>()->setYaw(state.theta);
    result->as<StateType>()->setKappa(state.kappa);
    result->as<StateType>()->setD(state.d);
  }
};

class CCDubinsStateSpace
    : public CurvatureStateSpace<CC_Dubins_State_Space, 1> {
 public:
  CCDubinsStateSpace(double kappa, double sigma, double discretization = 0.1,
                     bool forwards = true)
      : CurvatureStateSpace<CC_Dubins_State_Space, 1>(
            new CC_Dubins_State_Space(kappa, sigma, discretization, forwards)) {
    setName("CC_Dubins");
  }
};

class HCReedsSheppStateSpace
    : public CurvatureStateSpace<HC_Reeds_Shepp_State_Space, 2> {
 public:
  HCReedsSheppStateSpace(double kappa, double sigma,
                         double discretization = 0.1)
      : CurvatureStateSpace<HC_Reeds_Shepp_State_Space, 2>(
            new HC_Reeds_Shepp_State_Space(kappa, sigma, discretization)) {
    setName("HC_Reeds_Shepp");
  }
};

class CCReedsSheppStateSpace
    : public CurvatureStateSpace<CC00_Reeds_Shepp_State_Space, 3> {
 public:
  CCReedsSheppStateSpace(double kappa, double sigma,
                         double discretization = 0.1)
      : CurvatureStateSpace<CC00_Reeds_Shepp_State_Space, 3>(
            new CC00_Reeds_Shepp_State_Space(kappa, sigma, discretization)) {
    setName("CC_Reeds_Shepp");
  }
};

class DubinsStateSpace : public CurvatureStateSpace<Dubins_State_Space, 4> {
 public:
  DubinsStateSpace(double kappa, double discretization = 0.1,
                   bool forwards = true)
      : CurvatureStateSpace<Dubins_State_Space, 4>(
            new Dubins_State_Space(kappa, discretization, forwards)) {
    setName("Dubins");
  }
};

class ReedsSheppStateSpace
    : public CurvatureStateSpace<Reeds_Shepp_State_Space, 5> {
 public:
  ReedsSheppStateSpace(double kappa, double discretization = 0.1)
      : CurvatureStateSpace<Reeds_Shepp_State_Space, 5>(
            new Reeds_Shepp_State_Space(kappa, discretization)) {
    setName("Reeds_Shepp");
  }
};

class FourWSStateSpace : public CurvatureStateSpace<FourWS_State_Space, 6> {
 public:
  FourWSStateSpace(double kappa, double discretization = 0.005, 
                   double wheelbase = 11.04, double delta_max = 0.5)
      : CurvatureStateSpace<FourWS_State_Space, 6>(
            new FourWS_State_Space(kappa, discretization, wheelbase, delta_max)) {
    setName("FourWS_StateSpace");
  }
};
}  // namespace hc_cc_spaces
