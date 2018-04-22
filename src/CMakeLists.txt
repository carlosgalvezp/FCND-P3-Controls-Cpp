add_subdirectory(Drawing)
add_subdirectory(Math)
add_subdirectory(MavlinkNode)
add_subdirectory(Simulation)
add_subdirectory(Utility)

add_executable(${PROJECT_NAME}
               BaseController.cpp
               main.cpp
               QuadControl.cpp
               Trajectory.cpp)

target_link_libraries(${PROJECT_NAME}
                      Drawing
                      Math
                      MavlinkNode
                      Simulation
                      Utility
                      glut
                      GLU
                      GL
                      pthread)
