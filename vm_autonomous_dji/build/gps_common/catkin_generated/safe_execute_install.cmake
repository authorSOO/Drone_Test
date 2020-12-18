execute_process(COMMAND "/home/ethan/hector_drone/src/vm_autonomous_dji/build/gps_common/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ethan/hector_drone/src/vm_autonomous_dji/build/gps_common/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
