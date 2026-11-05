find_package(pybind11 REQUIRED)

pybind11_add_module(metrics_bindings 
    python/PythonBindings.cpp
)

target_link_libraries(metrics_bindings PRIVATE 
    mpb_core
    mpb_steering
    mpb_planners
    mpb_smoothers
)

set_target_properties(metrics_bindings PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/python"
)