add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wrench_estimator_msgs
  geometry_msgs)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS})

add_library(WrenchEstimatorROSBridge SHARED WrenchEstimatorROSBridge.cpp)
target_link_libraries(WrenchEstimatorROSBridge
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES})
set_target_properties(WrenchEstimatorROSBridge PROPERTIES PREFIX "")

add_executable(WrenchEstimatorROSBridgeComp WrenchEstimatorROSBridgeComp.cpp)
target_link_libraries(WrenchEstimatorROSBridgeComp WrenchEstimatorROSBridge)

install(TARGETS WrenchEstimatorROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS WrenchEstimatorROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
