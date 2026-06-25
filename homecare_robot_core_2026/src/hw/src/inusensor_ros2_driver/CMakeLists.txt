cmake_minimum_required(VERSION 3.15)
project(inusensor_ros2_driver)

# ---- C++ Standard ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Compiler-specific options ----
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- MSVC Runtime 설정 ----
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
#set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} /MDd")

# ---- Windows definitions ----
if(WIN32)
  add_definitions(-D_WIN32_WINNT=0x0601)  # Windows 7 이상
  add_definitions(-DWIN32_LEAN_AND_MEAN)
endif()

# ---- InuDev SDK paths ----
if(WIN32)
  set(INUDEV_ROOT "C:/Program Files/Inuitive/InuDev" CACHE PATH "InuDev root")
else()
  set(INUDEV_ROOT "/opt/Inuitive/InuDev" CACHE PATH "InuDev root")
endif()

set(INUDEV_INC  "${INUDEV_ROOT}/include")
set(INUDEV_LIB  "${INUDEV_ROOT}/lib")
set(INUDEV_BIN  "${INUDEV_ROOT}/bin")

# ---- OpenCV ----
if(WIN32)
  set(OpenCV_DIR "C:/opencv/build/x64/vc16" CACHE PATH "OpenCV cmake dir")
  find_package(OpenCV REQUIRED core imgproc highgui videoio imgcodecs)
else()
  find_package(OpenCV REQUIRED)
endif()
  

# ---- ROS2 dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)

# ---- Source files ----
set(SOURCES
  src/main.cpp
  src/inusensor_node.cpp
  src/inusensor_manager.cpp
)

# ---- Executable ----
add_executable(${PROJECT_NAME}_node ${SOURCES})

# ---- Include directories ----
target_include_directories(${PROJECT_NAME}_node PRIVATE
  include
  ${INUDEV_INC}
  ${INUDEV_INC}/Common
  ${OpenCV_INCLUDE_DIRS}
)

# ---- InuDev libraries ----
if(WIN32)
  # Windows용 라이브러리 설정
  set(INUDEV_CORE_LIBS_DEBUG "InuStreamsd;InuCommonUtilitiesd")
  set(INUDEV_CORE_LIBS_RELEASE "InuStreams;InuCommonUtilities")

  file(GLOB _ALL_INUDEV_LIBS "${INUDEV_LIB}/*.lib")
  set(INUDEV_DEBUG_LIBS_EXTRA   "")
  set(INUDEV_RELEASE_LIBS_EXTRA "")

  foreach(L ${_ALL_INUDEV_LIBS})
    get_filename_component(_name "${L}" NAME_WE)
    if(_name MATCHES "^InuStreams(d)?$" OR _name MATCHES "^InuCommonUtilities(d)?$")
      continue()
    endif()
    if(L MATCHES "d\\.lib$")
      list(APPEND INUDEV_DEBUG_LIBS_EXTRA "${L}")
    else()
      list(APPEND INUDEV_RELEASE_LIBS_EXTRA "${L}")
    endif()
  endforeach()

  # Link directories
  target_link_directories(${PROJECT_NAME}_node PUBLIC "${INUDEV_LIB}")

else()
  # Linux용 라이브러리 설정
  if(EXISTS "${INUDEV_LIB}")
    message(STATUS "InuDev SDK found at ${INUDEV_ROOT}")
    
    # Linux에서는 보통 .so 파일 사용
    find_library(INUSTREAMS_LIB 
      NAMES InuStreams libInuStreams
      PATHS ${INUDEV_LIB}
      NO_DEFAULT_PATH
    )
    find_library(INUCOMMON_LIB 
      NAMES InuCommonUtilities libInuCommonUtilities
      PATHS ${INUDEV_LIB}
      NO_DEFAULT_PATH
    )

    set(INUDEV_LIBS ${INUSTREAMS_LIB} ${INUCOMMON_LIB})
    message(STATUS "Found InuDev libraries: ${INUDEV_LIBS}")

  else()
    message(WARNING "InuDev SDK not found at ${INUDEV_ROOT}. Building without SDK support.")
    set(INUDEV_LIBS "")

  endif()
endif()

# ---- Linking ----
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  camera_info_manager
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport  
)

message("Library Linking..........")

# ---- Platform-specific linking ----
if(WIN32)
  target_link_libraries(${PROJECT_NAME}_node
    "$<$<CONFIG:Debug>:${INUDEV_CORE_LIBS_DEBUG}>"
    "$<$<CONFIG:Release>:${INUDEV_CORE_LIBS_RELEASE}>"
    "$<$<CONFIG:Debug>:${INUDEV_DEBUG_LIBS_EXTRA}>"
    "$<$<CONFIG:Release>:${INUDEV_RELEASE_LIBS_EXTRA}>"
    ${OpenCV_LIBS}
  )
else()
  # Linux linking
  target_link_libraries(${PROJECT_NAME}_node
    ${INUDEV_LIBS}  # 이미 찾은 라이브러리들 또는 빈 리스트
    ${OpenCV_LIBS}
  )
endif()

# ---- Compiler options ----
if(MSVC)
  target_compile_options(${PROJECT_NAME}_node PRIVATE
    $<$<CONFIG:Debug>:/Od>
    $<$<CONFIG:Release>:/O2;/Oi;/Gy>
    /W3
    /utf-8
  )
  target_link_options(${PROJECT_NAME}_node PRIVATE
    /SUBSYSTEM:CONSOLE
  )
else()
  target_compile_options(${PROJECT_NAME}_node PRIVATE
    $<$<CONFIG:Debug>:-O0 -g>
    $<$<CONFIG:Release>:-O3>
    -fmessage-length=0 -Wall -Wno-unknown-pragmas -Wno-reorder -Wno-unused-result -Wno-parentheses -fPIC
  )
endif()

# ---- Windows runtime output ----
if(WIN32)
  set_target_properties(${PROJECT_NAME}_node PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release"
  )
endif()

message(STATUS "Target Install..........")

# ---- Install ----
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# ---- DLL copying (Windows only) ----
if(WIN32)
  file(GLOB INUDEV_DLLS "${INUDEV_BIN}/*.dll")
  set(OPENCV_BIN_DIR "C:/opencv/build/x64/vc16/bin" CACHE PATH "OpenCV runtime bin")
  file(GLOB OPENCV_DLLS "${OPENCV_BIN_DIR}/*.dll")
  
  foreach(DLL ${INUDEV_DLLS} ${OPENCV_DLLS})
    add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${DLL}" "$<TARGET_FILE_DIR:${PROJECT_NAME}_node>"
      COMMENT "Copying ${DLL} to output directory")
  endforeach()
endif()


# ---- Linting ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
