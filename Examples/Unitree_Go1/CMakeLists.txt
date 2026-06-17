# Go1 SysID library — only the source files actually needed
add_library(Go1lib SHARED
    SystemIdentification/ParametersIdentification/src/FrictionParametersIdentification.cpp
    SystemIdentification/ParametersIdentification/src/EndEffectorParametersIdentification.cpp
    SystemIdentification/ParametersIdentification/src/EndEffectorParametersIdentificationMomentum.cpp
    SystemIdentification/ExcitingTrajectories/src/PayloadExcitingTrajectoryGenerator.cpp
    SystemIdentification/ExcitingTrajectories/src/ExcitingTrajectoryGenerator.cpp
    CollisionAvoidanceTrajectory/src/KinovaCustomizedConstraints.cpp) 

target_include_directories(Go1lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemIdentification/ParametersIdentification/include
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemIdentification/ExcitingTrajectories/include
    ${CMAKE_SOURCE_DIR}/Trajectories/include
    ${CMAKE_SOURCE_DIR}/Costs/include
    ${CMAKE_SOURCE_DIR}/Constraints/include
    ${CMAKE_SOURCE_DIR}/Optimization/include
    ${CMAKE_SOURCE_DIR}/Utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Armour/include
    ${CMAKE_CURRENT_SOURCE_DIR}/CollisionAvoidanceTrajectory/include
)

target_link_libraries(Go1lib PUBLIC
    trajlib
    IDlib
    Costslib
    Conslib
    Optlib
    ipopt
    coinhsl
    ${Boost_LIBRARIES}
    pinocchio::pinocchio
)

###
add_executable(Go1_SysidFriction_test
    SystemIdentification/ParametersIdentification/TestFrictionParametersIdentification.cpp)
target_include_directories(Go1_SysidFriction_test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemIdentification
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemIdentification/ParametersIdentification/include)
target_link_libraries(Go1_SysidFriction_test PUBLIC
    trajlib
    IDlib
    Conslib
    Optlib
    Go1lib
    ipopt
    coinhsl
    ${Boost_LIBRARIES}
    pinocchio::pinocchio)
target_compile_options(Go1_SysidFriction_test PUBLIC
    ${PINOCCHIO_FLAGS})

add_executable(Go1_Sysid_end_effector_traj
    SystemIdentification/ExcitingTrajectories/Go1_RegressorExample_end_effector.cpp)
target_include_directories(Go1_Sysid_end_effector_traj PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemIdentification
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemIdentification/ExcitingTrajectories/include)
target_link_libraries(Go1_Sysid_end_effector_traj PUBLIC
    trajlib
    IDlib
    Conslib
    Optlib
    Go1lib
    ipopt
    coinhsl
    ${Boost_LIBRARIES}
    pinocchio::pinocchio)
target_compile_options(Go1_Sysid_end_effector_traj PUBLIC
    ${PINOCCHIO_FLAGS})

# ==============================================================================
# Go1 Armour
# ==============================================================================

# 1. The library
add_library(Go1Armourlib SHARED
    Armour/src/PZSparse.cpp
    Armour/src/RobotInfo.cpp
    Armour/src/ParameterizedTrajectory.cpp
    Armour/src/PZDynamics.cpp
    Armour/src/ArmourOptimizer.cpp
    Armour/src/DualArmourOptimizer.cpp
    SystemIdentification/ExcitingTrajectories/src/SafePayloadExcitingTrajectoryGenerator.cpp
)

target_include_directories(Go1Armourlib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Armour/include
    ${CMAKE_SOURCE_DIR}/Trajectories/include
    ${CMAKE_SOURCE_DIR}/PZDynamics/include
    ${CMAKE_SOURCE_DIR}/Constraints/include
    ${CMAKE_SOURCE_DIR}/Optimization/include
    ${CMAKE_SOURCE_DIR}/Utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemIdentification/ExcitingTrajectories/include
)

target_link_libraries(Go1Armourlib PUBLIC
    trajlib
    IDlib
    Conslib
    Costslib
    Optlib
    ipopt
    coinhsl
    ${Boost_LIBRARIES}
    pinocchio::pinocchio
    yaml-cpp
)

# 2. The single executable example
add_executable(Go1Armour_example 
    Armour/ArmourExample.cpp)
target_include_directories(Go1Armour_example PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Armour/include)
target_link_libraries(Go1Armour_example PUBLIC
    trajlib 
    IDlib
    Conslib
    Optlib
    Go1Armourlib
    ipopt
    coinhsl
    ${BOOST_LIBRARIES} 
    pinocchio::pinocchio)
target_compile_options(Go1Armour_example PUBLIC
    ${PINOCCHIO_FLAGS})

# 3. The dual example
add_executable(Go1DualArmour_example 
    Armour/DualArmourExample.cpp)
target_include_directories(Go1DualArmour_example PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Armour/include)
target_link_libraries(Go1DualArmour_example PUBLIC
    trajlib 
    IDlib
    Conslib
    Optlib
    Go1Armourlib
    ipopt
    coinhsl
    ${BOOST_LIBRARIES} 
    pinocchio::pinocchio)
target_compile_options(Go1DualArmour_example PUBLIC
    ${PINOCCHIO_FLAGS})

# ==============================================================================
# Go1 Armour Python Bindings (Nanobind)
# ==============================================================================
# 1. Single Armour module
nanobind_add_module(go1_armour_nanobind 
    NB_SHARED LTO
    Armour/ArmourPybind.cpp
    Armour/src/ArmourPybindWrapper.cpp)
target_include_directories(go1_armour_nanobind PUBLIC
    Armour/include)
target_link_libraries(go1_armour_nanobind PUBLIC
    Go1lib
    Go1Armourlib
    ${PYTHON_LIBRARIES})
nanobind_compile_options(go1_armour_nanobind PUBLIC
    ${PINOCCHIO_FLAGS})
set_property(TARGET go1_armour_nanobind PROPERTY POSITION_INDEPENDENT_CODE ON)
# 2. Dual Armour module
nanobind_add_module(go1_dualarmour_nanobind 
    NB_SHARED LTO
    Armour/DualArmourPybind.cpp
    Armour/src/DualArmourPybindWrapper.cpp)
target_include_directories(go1_dualarmour_nanobind PUBLIC
    Armour/include)
target_link_libraries(go1_dualarmour_nanobind PUBLIC
    Go1lib
    Go1Armourlib
    ${PYTHON_LIBRARIES})
nanobind_compile_options(go1_dualarmour_nanobind PUBLIC
    ${PINOCCHIO_FLAGS})
set_property(TARGET go1_dualarmour_nanobind PROPERTY POSITION_INDEPENDENT_CODE ON)