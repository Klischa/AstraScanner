# Unit-тесты для AstraScanner.
# Используют Google Test (GTest), который ищется через vcpkg или системный пакет.
# Запуск: cmake --build build --target test
#    или: cd build && ctest --output-on-failure

cmake_minimum_required(VERSION 3.22)

find_package(GTest REQUIRED)
find_package(PCL 1.15 REQUIRED COMPONENTS common filters)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
add_definitions(${PCL_DEFINITIONS})

# --- test_CloudHistory ---
add_executable(test_CloudHistory
    test_CloudHistory.cpp
    ../src/history/CloudHistory.cpp
)
target_link_libraries(test_CloudHistory
    GTest::gtest GTest::gtest_main
    Qt6::Core
    ${PCL_LIBRARIES}
)
add_test(NAME CloudHistory COMMAND test_CloudHistory)

# --- test_PointCloudFilters ---
add_executable(test_PointCloudFilters
    test_PointCloudFilters.cpp
    ../src/filters/PointCloudFilters.cpp
)
target_link_libraries(test_PointCloudFilters
    GTest::gtest GTest::gtest_main
    Qt6::Core
    ${PCL_LIBRARIES}
)
add_test(NAME PointCloudFilters COMMAND test_PointCloudFilters)
