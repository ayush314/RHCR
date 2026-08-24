project(lifelong)

cmake_minimum_required (VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)

IF(NOT CMAKE_BUILD_TYPE)
    #SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")

# Find Boost
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )


add_executable(lifelong ${SOURCES} )
target_link_libraries(lifelong ${Boost_LIBRARIES})

enable_testing()
add_executable(workstation_policy_tests
    tests/workstation_policy_tests.cpp
    src/BasicGraph.cpp
    src/MAPFSolver.cpp
    src/PIBT.cpp
    src/ReservationTable.cpp
    src/SIPP.cpp
    src/SingleAgentSolver.cpp
    src/StateTimeAStar.cpp
    src/States.cpp
    src/WorkstationGraph.cpp
    src/common.cpp)
target_link_libraries(workstation_policy_tests ${Boost_LIBRARIES})
add_test(NAME workstation_policy_tests COMMAND workstation_policy_tests)
