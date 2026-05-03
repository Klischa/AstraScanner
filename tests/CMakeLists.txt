cmake_minimum_required(VERSION 3.22)
project(AstraScannerTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GoogleTest
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Qt6 Test
find_package(Qt6 REQUIRED COMPONENTS Core Test)

# PCL
find_package(PCL 1.12 REQUIRED COMPONENTS common filters)

enable_testing()

# PCL фильтры тест
add_executable(test_pcl_filters ${CMAKE_CURRENT_SOURCE_DIR}/test_pcl_filters.cpp)
target_include_directories(test_pcl_filters PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(test_pcl_filters
    GTest::gtest_main
    Qt6::Core
    ${PCL_LIBRARIES}
)
add_test(NAME PCLFiltersTests COMMAND test_pcl_filters)