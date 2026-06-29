cmake_minimum_required(VERSION 3.14)
project(unitree_guide)

set(ROBOT_TYPE Go1)         # The type of robot, support A1 Go1 Go2
set(PLATFORM amd64)         # The platform to compile, support amd64 and arm64

set(CATKIN_MAKE ON)        # Use CATKIN_MAKE or not, ON or OFF
set(SIMULATION ON)         # Use Gazebo or not, ON or OFF
set(REAL_ROBOT OFF)          # Link real robot or not, ON or OFF
set(DEBUG OFF)              # Use debug functions or not, ON or OFF
set(MOVE_BASE OFF)               # Need move_base or not, ON or OFF 

set(USE_XBOX_JOYSTICK OFF) 
#set(USE_XBOX_JOYSTICK ON) 

if(USE_XBOX_JOYSTICK)
    add_definitions(-DUSE_XBOX_JOYSTICK)
endif()

set(use_Torch ON)
#set(use_Torch OFF)

if(use_Torch)
    #set(Torch_DIR /home/rosxiaobai/software/libtorch/share/cmake/Torch)  # set libtorch path
    set(Torch_DIR /home/rosxiaobai/software/libtorch_cpu/share/cmake/Torch)  # set libtorch path

    find_package(Torch REQUIRED)
    
    set(BODY_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/model/body.jit")
    set(ADAPT_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/model/adapt.jit")
    add_definitions(-DBODY_MODEL_PATH="${BODY_MODEL_PATH}")
    add_definitions(-DADAPT_MODEL_PATH="${ADAPT_MODEL_PATH}")
endif()

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
elseif(${ROBOT_TYPE} STREQUAL "Go2")
    add_definitions(-DROBOT_TYPE_Go2)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()
# 如果是仿真模式，自动开启 ROS 宏
if(SIMULATION OR MOVE_BASE)
    add_definitions(-DRUN_ROS)
    set(CATKIN_MAKE ON)
endif()

set(CMAKE_CXX_STANDARD 14)
find_package(Boost)



if(CATKIN_MAKE)
    add_definitions(-DCOMPILE_WITH_ROS)
    if(MOVE_BASE)
        add_definitions(-DCOMPILE_WITH_MOVE_BASE)
    endif()
endif()

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy)
endif()

if(CATKIN_MAKE)
    if(SIMULATION)
        add_definitions(-DCOMPILE_WITH_SIMULATION)
        find_package(catkin REQUIRED COMPONENTS
        genmsg
        robot_state_publisher
        roscpp
        std_msgs
        tf
        tf2_ros            # <--- 新增：用于 TF 树查询
        geometry_msgs
        unitree_legged_msgs
        grid_map_ros       # <--- 新增：高程图
        grid_map_core
        grid_map_msgs      # <--- 新增：高程图消息
        cv_bridge          # <--- 新增：ROS 视觉桥接
        )
        # 明确寻找 OpenCV 库
        find_package(OpenCV REQUIRED)
    else()
        find_package(catkin REQUIRED COMPONENTS
            genmsg
            robot_state_publisher
            roscpp
            std_msgs
            tf
            geometry_msgs
            unitree_legged_msgs
        )
    endif()
    catkin_package(
        CATKIN_DEPENDS
        unitree_legged_msgs
    )
    catkin_install_python(PROGRAMS
        scripts/ground_truth_tf_publisher.py
        scripts/odom_to_pose_fastlio.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        include_directories(
            library/unitree_legged_sdk_3.2/include
        )
        link_directories(
            library/unitree_legged_sdk_3.2/lib
        )
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        include_directories(
            library/unitree_legged_sdk-3.8.0/include
        )
        if(${PLATFORM} STREQUAL "amd64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/amd64
            )
        elseif(${PLATFORM} STREQUAL "arm64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/arm64
            )
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go2")
        include_directories(
            /usr/local/include/ddscxx
            /usr/local/include/iceoryx/v2.0.2
            /usr/local/include/unitree
            ./library/unitree_sdk2/include
        )
        link_directories(/usr/local/lib)
    endif()
endif()

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"

)

add_executable(junior_ctrl src/main.cpp ${SRC_LIST} "src")

if(use_Torch)
    target_link_libraries(junior_ctrl ${TORCH_LIBRARIES})
endif()


if(CATKIN_MAKE)
    target_link_libraries(junior_ctrl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
    add_dependencies(junior_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python2::Python Python2::NumPy)
endif()
if(REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        if(${PLATFORM} STREQUAL "amd64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_amd64.so)
        elseif(${PLATFORM} STREQUAL "arm64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_arm64.so)
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
            target_link_libraries(junior_ctrl libunitree_legged_sdk.a)
    elseif(${ROBOT_TYPE} STREQUAL "Go2")
            target_link_libraries(junior_ctrl unitree_sdk2 ddsc ddscxx rt pthread)
    endif()
endif()

target_link_libraries(junior_ctrl -pthread lcm)

if(NOT CATKIN_MAKE)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
endif()