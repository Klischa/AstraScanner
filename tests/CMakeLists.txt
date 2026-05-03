cmake_minimum_required(VERSION 3.22)
project(AstraScannerTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GoogleTest (скачивается автоматически)
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Находим Qt6 для тестов
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Test Network)

# Находим PCL
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)

# Пути к исходникам главного проекта для включения заголовочных файлов
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR}/..)

enable_testing()

# Тесты фильтров - требуют заголовочный файл PointCloudFilters.h
add_executable(test_filters ${CMAKE_CURRENT_SOURCE_DIR}/unit/test_filters.cpp)
target_include_directories(test_filters PRIVATE
    ${PROJECT_ROOT}/src/filters
    ${PROJECT_ROOT}/include
)
target_link_libraries(test_filters
    GTest::gtest_main
    Qt6::Core Qt6::Widgets
    PCLCommon PCLFilters
)
add_test(NAME FiltersTests COMMAND test_filters)

# Тесты облака - требуют заголовки PCL
add_executable(test_cloud ${CMAKE_CURRENT_SOURCE_DIR}/unit/test_cloud.cpp)
target_link_libraries(test_cloud
    GTest::gtest_main
    Qt6::Core
    PCLCommon PCLIO
)
add_test(NAME CloudTests COMMAND test_cloud)

# Тесты AI клиента
add_executable(test_ai_client ${CMAKE_CURRENT_SOURCE_DIR}/unit/test_ai_client.cpp)
target_link_libraries(test_ai_client
    GTest::gtest_main
    Qt6::Core Qt6::Network
)
add_test(NAME AiClientTests COMMAND test_ai_client)

# Интеграционные тесты
add_executable(test_integration ${CMAKE_CURRENT_SOURCE_DIR}/integration/test_integration.cpp)
target_link_libraries(test_integration
    GTest::gtest_main
    Qt6::Core Qt6::Widgets Qt6::Test
)
add_test(NAME IntegrationTests COMMAND test_integration)