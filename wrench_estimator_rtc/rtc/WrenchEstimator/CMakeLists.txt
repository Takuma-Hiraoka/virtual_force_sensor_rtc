add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  )
find_package(choreonoid REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

# 相対パスを絶対パスに直す                                                                                                                                                                                  
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

rtmbuild_add_library(WrenchEstimator SHARED
  WrenchEstimator.cpp
  )
target_link_libraries(WrenchEstimator
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  )
set_target_properties(WrenchEstimator PROPERTIES PREFIX "") # libWrenchEstimator.so -> WrenchEstimator.so                                                                           
add_dependencies(WrenchEstimator RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl                                                                                                        

rtmbuild_add_executable(WrenchEstimatorComp WrenchEstimatorComp.cpp)
target_link_libraries(WrenchEstimatorComp WrenchEstimator)
