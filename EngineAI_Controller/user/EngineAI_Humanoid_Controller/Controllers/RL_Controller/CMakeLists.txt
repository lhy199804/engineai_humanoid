cmake_minimum_required(VERSION 3.10)
project(rl_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
        include
        "include/"
        ${EIGEN3_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(rl_controllers SHARED
        src/LearningBasedController.cpp
        )       

find_package(PkgConfig REQUIRED)

# find onnxruntime lib
pkg_check_modules(ONNXRUNTIME QUIET libonnxruntime)

# manually add the lib, if onnx is not found
if(NOT ONNXRUNTIME_FOUND)
        find_package(onnxruntime REQUIRED)
endif()

# Link specific onnxruntime version to avoid ABI mismatch on target machine
# where libonnxruntime.so.1 may point to a different version.
# Use dynacore_yaml-cpp (project-internal) instead of system yaml-cpp
# to avoid symbol conflicts with libdynacore_yaml-cpp.so.0.5 used by common_module.
target_link_libraries(rl_controllers /usr/local/lib/libonnxruntime.so.1.19.0 dynacore_yaml-cpp)

#############
## Install ##
#############

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/build/EngineAI_Humanoid/install)

install(TARGETS rl_controllers   
      LIBRARY DESTINATION lib                      
)

install(FILES config/zqsa01_rl.yaml
        DESTINATION ${CMAKE_INSTALL_PREFIX}/config
        )

install(FILES policy/zqsa01/zqsa01_policy.onnx
        DESTINATION ${CMAKE_INSTALL_PREFIX}/policy/zqsa01
        )
