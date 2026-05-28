# Go1 SysID library — only the source files actually needed
add_library(Go1lib SHARED
    SystemIdentification/ParametersIdentification/src/FrictionParametersIdentification.cpp
    SystemIdentification/ParametersIdentification/src/EndEffectorParametersIdentification.cpp
    SystemIdentification/ParametersIdentification/src/EndEffectorParametersIdentificationMomentum.cpp)

target_include_directories(Go1lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemIdentification/ParametersIdentification/include
    ${CMAKE_SOURCE_DIR}/Trajectories/include
    ${CMAKE_SOURCE_DIR}/Costs/include
    ${CMAKE_SOURCE_DIR}/Constraints/include
    ${CMAKE_SOURCE_DIR}/Optimization/include
    ${CMAKE_SOURCE_DIR}/Utils/include
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