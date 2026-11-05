#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "metrics/PathLengthMetric.h"
#include "metrics/SweptAreaMetric.h"
#include "metrics/MaxCurvatureMetric.h"
#include "metrics/NormalizedCurvatureMetric.h"
#include "metrics/AOLMetric.h"
#include "metrics/ClearingMetric.h"
#include "utils/PlannerUtils.hpp"
#include "base/environments/GridMaze.h"  
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <fstream>
#include <metrics/EuclideanDeviationMetric.h>
#include <metrics/AngularDeviationMetric.h>


namespace py = pybind11;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace metrics {

// Fonction wrapper pour calculer la longueur du chemin
double compute_path_length(const std::vector<std::vector<double>>& path) {
    // Créer l'espace d'états SE2
    auto space(std::make_shared<ob::SE2StateSpace>());
    // Configurer les limites de l'espace
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1000);
    bounds.setHigh(1000);
    space->setBounds(bounds);
    
    auto si(std::make_shared<ob::SpaceInformation>(space));
    
    // Ajouter un state validity checker basique
    class DefaultStateValidityChecker : public ob::StateValidityChecker {
    public:
        DefaultStateValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si) {}
        bool isValid(const ob::State*) const override { return true; }
    };
    si->setStateValidityChecker(std::make_shared<DefaultStateValidityChecker>(si));
    si->setup();
    
    // Créer le chemin géométrique
    og::PathGeometric geometric_path(si);
    
    // Ajouter les états au chemin
    for (const auto& p : path) {
        auto state = si->allocState();
        state->as<ob::SE2StateSpace::StateType>()->setX(p[0]);
        state->as<ob::SE2StateSpace::StateType>()->setY(p[1]);
        state->as<ob::SE2StateSpace::StateType>()->setYaw(p[2]);
        geometric_path.append(state);
    }
    
    // Utiliser la même méthode que dans PathLengthMetric
    return PathLengthMetric::evaluateMetric(geometric_path, 0.0);
}

// Fonction wrapper pour calculer la courbure
std::pair<double, double> compute_curvature(const std::vector<std::vector<double>>& path) {
    // Créer l'espace d'états SE2
    auto space(std::make_shared<ob::SE2StateSpace>());
    
    // Configurer les limites de l'espace
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1000);
    bounds.setHigh(1000);
    space->setBounds(bounds);
    
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setup();
    
    // Créer le chemin géométrique
    og::PathGeometric geometric_path(si);
    
    // Ajouter les états au chemin
    for (const auto& p : path) {
        auto state = si->allocState();
        state->as<ob::SE2StateSpace::StateType>()->setX(p[0]);
        state->as<ob::SE2StateSpace::StateType>()->setY(p[1]);
        state->as<ob::SE2StateSpace::StateType>()->setYaw(p[2]);
        geometric_path.append(state);
    }
    
    // Pas besoin d'interpoler car le chemin est déjà interpolé
    double max_curvature = MaxCurvatureMetric::evaluate(geometric_path);
    double normalized_curvature = NormalizedCurvatureMetric::evaluate(geometric_path);
    
    return {max_curvature, normalized_curvature};
}

// Fonction wrapper pour calculer la régularité
double compute_smoothness(const std::vector<std::vector<double>>& path) {
    std::vector<Point> points;
    for (const auto& p : path) {
        points.push_back(Point(p[0], p[1]));
    }
    // TODO: Implémenter le calcul de régularité
    return 0.0;
}

// Fonction wrapper pour calculer l'AOL
double compute_aol(const std::vector<std::vector<double>>& path) {
    // Créer l'espace d'états SE2
    auto space(std::make_shared<ob::SE2StateSpace>());
    
    // Configurer les limites de l'espace
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1000);
    bounds.setHigh(1000);
    space->setBounds(bounds);
    
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setup();
    
    // Créer le chemin géométrique
    og::PathGeometric geometric_path(si);
    
    // Ajouter les états au chemin
    for (const auto& p : path) {
        auto state = si->allocState();
        state->as<ob::SE2StateSpace::StateType>()->setX(p[0]);
        state->as<ob::SE2StateSpace::StateType>()->setY(p[1]);
        state->as<ob::SE2StateSpace::StateType>()->setYaw(p[2]);
        geometric_path.append(state);
    }
    
    return AOLMetric::evaluateMetric(geometric_path, 0.0);
}

// The two one-argument forms below read the goal from global::settings,
// as a benchmark run sets it.
static og::PathGeometric _as_path(const std::vector<std::vector<double>>& path) {
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1000);
    bounds.setHigh(1000);
    space->setBounds(bounds);
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setup();
    og::PathGeometric geometric_path(si);
    for (const auto& p : path) {
        auto state = si->allocState();
        state->as<ob::SE2StateSpace::StateType>()->setX(p[0]);
        state->as<ob::SE2StateSpace::StateType>()->setY(p[1]);
        state->as<ob::SE2StateSpace::StateType>()->setYaw(p[2]);
        geometric_path.append(state);
    }
    return geometric_path;
}

double compute_euclidean_deviation(const std::vector<std::vector<double>>& path) {
    return EuclideanDeviationMetric::evaluateMetric(_as_path(path), 0.0);
}

double compute_angular_deviation(const std::vector<std::vector<double>>& path) {
    return AngularDeviationMetric::evaluateMetric(_as_path(path), 0.0);
}

// Fonction wrapper pour calculer la distance euclidiennes à l'approche
double compute_euclidean_deviation_at_goal(const std::vector<std::vector<double>>& path,
                                   double goal_x, double goal_y) {
    // Créer l'espace d'états SE2
    auto space(std::make_shared<ob::SE2StateSpace>());
    
    // Configurer les limites de l'espace
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1000);
    bounds.setHigh(1000);
    space->setBounds(bounds);
    
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setup();
    
    // Créer le chemin géométrique
    og::PathGeometric geometric_path(si);
    
    // Ajouter les états au chemin
    for (const auto& p : path) {
        auto state = si->allocState();
        state->as<ob::SE2StateSpace::StateType>()->setX(p[0]);
        state->as<ob::SE2StateSpace::StateType>()->setY(p[1]);
        state->as<ob::SE2StateSpace::StateType>()->setYaw(p[2]);
        geometric_path.append(state);
    }
    
    return EuclideanDeviationMetric::evaluateAtGoal(geometric_path, Point(goal_x, goal_y));
}

// Fonction wrapper pour calculer la distance angulaires à l'approche
double compute_angular_deviation_at_goal(const std::vector<std::vector<double>>& path,
                                 double goal_theta) {
    // Créer l'espace d'états SE2
    auto space(std::make_shared<ob::SE2StateSpace>());
    
    // Configurer les limites de l'espace
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1000);
    bounds.setHigh(1000);
    space->setBounds(bounds);
    
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setup();
    
    // Créer le chemin géométrique
    og::PathGeometric geometric_path(si);
    
    // Ajouter les états au chemin
    for (const auto& p : path) {
        auto state = si->allocState();
        state->as<ob::SE2StateSpace::StateType>()->setX(p[0]);
        state->as<ob::SE2StateSpace::StateType>()->setY(p[1]);
        state->as<ob::SE2StateSpace::StateType>()->setYaw(p[2]);
        geometric_path.append(state);
    }
    
    return AngularDeviationMetric::evaluateAtGoal(geometric_path, goal_theta);
}



std::vector<double> compute_clearing_distances(
    const std::vector<std::vector<double>>& path,
    py::array_t<bool> binary_map,
    double resolution = 1.0) {
    
    // Get binary map dimensions
    auto r = binary_map.unchecked<2>();
    int height = r.shape(0);
    int width = r.shape(1);
    
    // Create GridMaze environment from binary map
    auto environment = std::make_shared<GridMaze>(0, width, height, resolution);
    
    std::vector<std::vector<bool>> grid(height, std::vector<bool>(width));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            grid[y][x] = r(y, x);
        }
    }
    
    // Set the occupancy grid
    environment->setOccupancyGrid(grid);

    environment->computeDistances();
    
    // Write the environment and the distances along the path to debug.json
    nlohmann::json j;

    std::vector<std::map<std::string, double>> path_distances;
        for (const auto& p : path) {
            std::map<std::string, double> point;
            point["x"] = p[0];
            point["y"] = p[1];
            point["distance"] = environment->bilinearDistance(p[0], p[1], resolution);
            path_distances.push_back(point);
        }
        j["path_distances"] = path_distances;
    
    environment->to_json(j);
    std::ofstream file("debug.json");
    if (file.is_open()) {
        file << j.dump(4);  // Pretty print with indent of 4
        file.close();
    }
    
    // Store the environment globally for the metric to access
    global::settings.environment = environment;
    
    auto space(std::make_shared<ob::SE2StateSpace>());
    
    // Configure space bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1000);
    bounds.setHigh(1000);
    space->setBounds(bounds);
    
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setup();
    
    // Create geometric path
    og::PathGeometric geometric_path(si);
    
    // Add states to path
    for (const auto& p : path) {
        auto state = si->allocState();
        state->as<ob::SE2StateSpace::StateType>()->setX(p[0]);
        state->as<ob::SE2StateSpace::StateType>()->setY(p[1]);
        state->as<ob::SE2StateSpace::StateType>()->setYaw(p[2]);
        geometric_path.append(state);
    }
    
    // Calculate the clearing distances
    auto clearings = ClearingMetric::clearingDistances(geometric_path);

    return clearings;
}


double compute_swept_area(const std::vector<std::vector<double>>& path,
                          const std::vector<std::vector<double>>& footprint,
                          double metres_per_unit, int supersample, int step) {
    return SweptAreaMetric::evaluate(path, footprint, metres_per_unit,
                                     supersample, step);
}


PYBIND11_MODULE(metrics_bindings, m) {
    m.doc() = "Liaisons Python pour les métriques MPB"; 
    
    m.def("compute_path_length", &compute_path_length, "Calcule la longueur totale du chemin");
    m.def("compute_curvature", &compute_curvature, "Calcule la courbure maximale et normalisée");
    m.def("compute_smoothness", &compute_smoothness, "Calcule la régularité du chemin");
    m.def("compute_aol", &compute_aol, "Calcule la métrique AOL (Angle Over Length)");
    m.def("compute_angular_deviation", &compute_angular_deviation,
          "Calcule la deviation angulaires des positions finales");
    m.def("compute_euclidean_deviation", &compute_euclidean_deviation,
          "Calcule la distance euclidienne des positions finales");
    m.def("compute_angular_deviation_at_goal", &compute_angular_deviation_at_goal,
          "Angular deviation of the final poses with respect to a given goal heading",
          py::arg("path"), py::arg("goal_theta"));
    m.def("compute_euclidean_deviation_at_goal", &compute_euclidean_deviation_at_goal,
          "Euclidean deviation of the final poses with respect to a given goal position",
          py::arg("path"), py::arg("goal_x"), py::arg("goal_y"));
    m.def("compute_swept_area", &compute_swept_area,
          "Floor area swept by the robot footprint along the path, in m^2",
          py::arg("path"), py::arg("footprint"), py::arg("metres_per_unit"),
          py::arg("supersample") = 4, py::arg("step") = 1);

    m.def("compute_clearing_distances", &compute_clearing_distances, 
          "Compute clearing distances along the path",
          py::arg("path"),
          py::arg("binary_map"),
          py::arg("resolution") = 1.0);
}

} // namespace metrics