cmake_minimum_required(VERSION 3.14)
project(kalman_tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Fetch Eigen ---
include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Eigen)

find_package(OpenCV REQUIRED)

# --- Your executable ---
add_executable(kalman_tracker
    src/main.cpp
    src/KalmanFilter.cpp
)

target_include_directories(kalman_tracker PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(kalman_tracker PRIVATE Eigen3::Eigen ${OpenCV_LIBS})