cmake_minimum_required(VERSION 3.22)
project(AstraScanner VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DNOMINMAX)

# Allow overriding the vcpkg toolchain via -DCMAKE_TOOLCHAIN_FILE=... or VCPKG_ROOT env var.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    if(DEFINED ENV{VCPKG_ROOT})
        set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "Vcpkg toolchain file")
    elseif(EXISTS "C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake")
        set(CMAKE_TOOLCHAIN_FILE "C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "Vcpkg toolchain file")
    endif()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGL Concurrent)
find_package(OpenCV REQUIRED)
# surface нужен для Poisson-реконструкции; остальные — как и раньше.
find_package(PCL 1.15 REQUIRED COMPONENTS common io visualization filters registration features surface)
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    FiltersSources
    InteractionStyle
    RenderingOpenGL2
    RenderingQt
    GUISupportQt
)

# OpenNI2 SDK location: override with -DOPENNI2_ROOT=... or the OPENNI2_ROOT env var.
if(NOT DEFINED OPENNI2_ROOT)
    if(DEFINED ENV{OPENNI2_ROOT})
        set(OPENNI2_ROOT "$ENV{OPENNI2_ROOT}" CACHE PATH "Path to the OpenNI2 SDK root")
    else()
        set(OPENNI2_ROOT "C:/orbbec/OpenNI_2.3.0.86_202210111950_4c8f5aa4_beta6_windows/Win64-Release/sdk"
            CACHE PATH "Path to the OpenNI2 SDK root")
    endif()
endif()
set(OPENNI2_INCLUDE_DIR "${OPENNI2_ROOT}/Include")
set(OPENNI2_LIBRARY "${OPENNI2_ROOT}/libs/OpenNI2.lib")
set(OPENNI2_DLL      "${OPENNI2_ROOT}/libs/OpenNI2.dll")
set(OPENNI2_DRIVERS  "${OPENNI2_ROOT}/libs/OpenNI2/Drivers")

# vcpkg installed prefix used for post-build DLL copies (Qt platforms, runtime DLLs).
if(NOT DEFINED VCPKG_INSTALLED_DIR)
    if(DEFINED ENV{VCPKG_ROOT})
        set(VCPKG_INSTALLED_DIR "$ENV{VCPKG_ROOT}/installed/x64-windows"
            CACHE PATH "vcpkg installed prefix")
    else()
        set(VCPKG_INSTALLED_DIR "C:/dev/vcpkg/installed/x64-windows"
            CACHE PATH "vcpkg installed prefix")
    endif()
endif()

include_directories(
    ${OPENNI2_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
add_definitions(${PCL_DEFINITIONS})

set(SOURCES
    src/main.cpp
    src/gui/MainWindow.cpp
    src/gui/LiveCloudWindow.cpp
    src/capture/AstraCamera.cpp
    src/capture/CaptureWorker.cpp
    src/calibration/CameraCalibrator.cpp
    src/filters/PointCloudFilters.cpp
    src/settings/SettingsManager.cpp
    src/export/ExportManager.cpp
    src/project/ProjectManager.cpp
)

add_executable(${PROJECT_NAME} WIN32 ${SOURCES})

target_link_libraries(${PROJECT_NAME}
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::Concurrent
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    ${OPENNI2_LIBRARY}
)

if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

# Копирование зависимостей
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "--- Copying OpenNI2 ---"
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>/OpenNI2/Drivers" || (exit 0)
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENNI2_DLL}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>" || (exit 0)
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${OPENNI2_DRIVERS}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>/OpenNI2/Drivers" || (exit 0)
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "--- Copying Qt platforms ---"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${VCPKG_INSTALLED_DIR}/$<IF:$<CONFIG:Debug>,debug,release>/Qt6/plugins/platforms"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/platforms" || (exit 0)
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "--- Copying vcpkg runtime DLLs ---"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${VCPKG_INSTALLED_DIR}/$<IF:$<CONFIG:Debug>,debug,release>/bin"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>" || (exit 0)
)