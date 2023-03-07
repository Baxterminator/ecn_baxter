cmake_minimum_required(VERSION 3.5)
project(ecn_baxter VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Qt5 COMPONENTS Core Widgets WebKit WebKitWidgets REQUIRED)

# =================================================================
#                           ROS 1
# =================================================================
#find ROS 1 packages we depend on
set(ROS1_ROOT "/opt/ros/noetic")
set(ROS1_LIBS roscpp rosconsole roscpp_serialization rostime xmlrpcpp)

# if libraries in ROS1_ROOT, explicitely give their location
if(EXISTS ${ROS1_ROOT})
    foreach(ROS1_LIB ${ROS1_LIBS})
        add_library(${ROS1_LIB} UNKNOWN IMPORTED)
        set_property(TARGET ${ROS1_LIB} PROPERTY IMPORTED_LOCATION "${ROS1_ROOT}/lib/lib${ROS1_LIB}.so")
    endforeach()
endif()

# Baxters ROS1 message
set(BAXTER1_ROOT "/opt/ecn/ROS1/install")

# =================================================================
#             LEGACY CODE (FOXY, GALACTIC COMPATIBILITY)
# =================================================================
if("$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "foxy")
    set(LEGACY_IDL TRUE)
else()
    set(LEGACY_IDL FALSE)
endif()

function(legacy_cpp files out_l)
    foreach(f ${${files}})
        if(${LEGACY_IDL})
            string(CONCAT _out "src/legacy/" ${f})
        else()
            string(CONCAT _out "src/" ${f})
        endif()
        set(${out_l} ${${out_l}} ${_out})
    endforeach()    
    set(${out_l} ${${out_l}} PARENT_SCOPE)
endfunction(legacy_cpp)


# =================================================================
#                           INTERFACE
# =================================================================
file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/msg/*.msg"
        "${CMAKE_CURRENT_LIST_DIR}/srv/*.srv"
        "${CMAKE_CURRENT_LIST_DIR}/action/*.action"
)
set(dependencies
        "geometry_msgs")
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES ${dependencies})
ament_export_dependencies(rosidl_default_runtime)

if(NOT ${LEGACY_IDL})
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

macro(add_message node_name)
    if(${LEGACY_IDL})
        rosidl_target_interfaces(${node_name} "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    else()
        target_link_libraries(${node_name} "${cpp_typesupport_target}")
    endif()
endmacro()

# =================================================================
#                        QT 5 UI Generator
# =================================================================
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS resource/ui)
set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_RELEASE} -fprofile-arcs -ftest-coverage")
set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")

# Excluding files rosidl files from autouic generation
file(GLOB_RECURSE exclude_file
        "${CMAKE_CURRENT_BINARY_DIR}/ecn*/*.[hc]"
        "${CMAKE_CURRENT_BINARY_DIR}/ecn*/*.[hc]pp"
        "${CMAKE_CURRENT_BINARY_DIR}/ros*/*.[hc]"
        "${CMAKE_CURRENT_BINARY_DIR}/ros*/*.[hc]pp"
        )
foreach(X IN LISTS exclude_file)
    set_property(SOURCE ${X} PROPERTY SKIP_AUTOUIC ON)
endforeach()

# =================================================================
#                              OUTPUT
# =================================================================
## Gripper management
ament_auto_add_executable(gripper_node src/components/gripper_node.cpp)
target_include_directories(gripper_node PRIVATE
        include
)
add_message(gripper_node)

## Setup Server
ament_auto_add_library(setup_node SHARED src/setup/setup_node.cpp)
target_include_directories(setup_node PRIVATE
    include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(setup_node PRIVATE "ECN_BAXTER_BUILDING_DLL")
rclcpp_components_register_node(setup_node PLUGIN "ecn_baxter::setup::SetupNode" EXECUTABLE setup_server)
add_message(setup_node)
install(TARGETS
    setup_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

## Game Properties
ament_auto_add_library(game_properties 
    src/game/data/local_games.cpp
    src/game/data/game_properties.cpp
)
target_include_directories(game_properties PRIVATE include)

## Main Game Master executable
set(legacy_files_gm
    "game/ros2/setup_client.cpp"
)
legacy_cpp(legacy_files_gm legacy_gm)
ament_auto_add_executable(game_master
        src/game/game_master.cpp
        src/game/ros1/game_master_1.cpp
        src/game/ros1/bridge_lookup.cpp
        src/game/ros1/tf_broadcast.cpp
        src/game/ros2/game_master_2.cpp
        src/game/properties_loader.cpp
        src/game/ros2/client_points.cpp
        src/game/master/game.cpp
        src/game/master/gui_management.cpp
        src/ui/main_wrapper.cpp
        src/ui/file_loader_wrapper.cpp
        resource/ui/main.ui
        resource/ui/game_loader.ui
        ${legacy_gm}
)
target_include_directories(game_master PRIVATE
        include
        ${ROS1_ROOT}/include
        ${BAXTER1_ROOT}/include
)
target_link_libraries(game_master ${game_propeties} ${ROS1_LIBS} Qt5::Core Qt5::Widgets Qt5::WebKit Qt5::WebKitWidgets )
add_message(game_master)

# =================================================================
#                        EXPORT OTHERS
# =================================================================
install(DIRECTORY 
        resource/launch
        resource/games
        DESTINATION share/${PROJECT_NAME})

ament_auto_package()