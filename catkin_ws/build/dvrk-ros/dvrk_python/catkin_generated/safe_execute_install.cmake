execute_process(COMMAND "/home/apoorva/catkin_ws/build/dvrk-ros/dvrk_python/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/apoorva/catkin_ws/build/dvrk-ros/dvrk_python/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
