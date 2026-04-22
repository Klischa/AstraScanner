cmake_minimum_required(VERSION 3.22)
project(AstraScanner VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DNOMINMAX)

# --- GPU-ускорение (опционально) ---
# CUDA: для NVIDIA GPU — ускоряет конвертацию depth→cloud и VoxelGrid.
# На AMD (Ryzen 7 8700G / Radeon 780M) CUDA недоступна; вместо неё
# используется OpenMP-параллелизация (работает на любом CPU).
option(ASTRA_ENABLE_CUDA "Build with CUDA GPU acceleration (NVIDIA only)" OFF)

set(ASTRA_HAVE_CUDA FALSE)
if(ASTRA_ENABLE_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        find_package(CUDAToolkit REQUIRED)
        set(ASTRA_HAVE_CUDA TRUE)
        message(STATUS "CUDA enabled: ${CMAKE_CUDA_COMPILER_VERSION}")
    else()
        message(FATAL_ERROR "ASTRA_ENABLE_CUDA=ON but no CUDA compiler found. "
                            "Install CUDA Toolkit or pass -DASTRA_ENABLE_CUDA=OFF.")
    endif()
endif()

# OpenMP: ускорение на CPU (Ryzen 8700G: 8 ядер / 16 потоков).
# Используется PCL (NormalEstimationOMP) и наш код конвертации depth→cloud.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP enabled: ${OpenMP_CXX_VERSION}")
else()
    message(WARNING "OpenMP not found — parallel CPU paths will be single-threaded.")
endif()

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

# --- OpenNI2 (optional) ---
# Поддержка камеры Astra Pro требует OpenNI2 SDK и Orbbec-драйверов,
# которые официально поставляются только для Windows. На других платформах
# (или при отсутствии SDK) проект можно собрать в emulation-only режиме —
# AstraCamera будет отдавать синтетические кадры, остальная функциональность
# (фильтры, ICP, Poisson, импорт/экспорт, проект) работает без изменений.
option(ASTRA_ENABLE_OPENNI2 "Build with OpenNI2 depth-camera support" ${WIN32})

set(ASTRA_HAVE_OPENNI2 FALSE)
if(ASTRA_ENABLE_OPENNI2)
    if(NOT DEFINED OPENNI2_ROOT)
        if(DEFINED ENV{OPENNI2_ROOT})
            set(OPENNI2_ROOT "$ENV{OPENNI2_ROOT}" CACHE PATH "Path to the OpenNI2 SDK root")
        elseif(WIN32)
            set(OPENNI2_ROOT "C:/orbbec/OpenNI_2.3.0.86_202210111950_4c8f5aa4_beta6_windows/Win64-Release/sdk"
                CACHE PATH "Path to the OpenNI2 SDK root")
        endif()
    endif()

    if(DEFINED OPENNI2_ROOT AND EXISTS "${OPENNI2_ROOT}/Include/OpenNI.h")
        set(OPENNI2_INCLUDE_DIR "${OPENNI2_ROOT}/Include")
        if(WIN32)
            set(OPENNI2_LIBRARY "${OPENNI2_ROOT}/libs/OpenNI2.lib")
            set(OPENNI2_DLL     "${OPENNI2_ROOT}/libs/OpenNI2.dll")
            set(OPENNI2_DRIVERS "${OPENNI2_ROOT}/libs/OpenNI2/Drivers")
        else()
            # На Linux/macOS OpenNI2 обычно .so/.dylib; оставляем за пользователем
            # переопределить OPENNI2_LIBRARY, если он что-то собирал руками.
            find_library(OPENNI2_LIBRARY OpenNI2
                HINTS "${OPENNI2_ROOT}/libs" "${OPENNI2_ROOT}/Redist")
        endif()
        if(OPENNI2_LIBRARY)
            set(ASTRA_HAVE_OPENNI2 TRUE)
        else()
            message(FATAL_ERROR "ASTRA_ENABLE_OPENNI2=ON but OpenNI2 library not found at "
                                "'${OPENNI2_ROOT}/libs'. Please set OPENNI2_ROOT correctly or "
                                "pass -DASTRA_ENABLE_OPENNI2=OFF to build in emulation-only mode.")
        endif()
    else()
        message(FATAL_ERROR "ASTRA_ENABLE_OPENNI2=ON but OPENNI2_ROOT is not set or "
                            "OpenNI.h not found. Please set -DOPENNI2_ROOT=<path-to-sdk> or "
                            "pass -DASTRA_ENABLE_OPENNI2=OFF to build in emulation-only mode.")
    endif()
endif()

if(NOT ASTRA_HAVE_OPENNI2)
    message(STATUS "Building AstraScanner in emulation-only mode (no OpenNI2).")
endif()

# vcpkg installed prefix used for post-build DLL copies (Qt platforms, runtime DLLs).
if(NOT DEFINED VCPKG_INSTALLED_DIR)
    if(DEFINED ENV{VCPKG_ROOT})
        set(VCPKG_INSTALLED_DIR "$ENV{VCPKG_ROOT}/installed/x64-windows"
            CACHE PATH "vcpkg installed prefix")
    elseif(WIN32)
        set(VCPKG_INSTALLED_DIR "C:/dev/vcpkg/installed/x64-windows"
            CACHE PATH "vcpkg installed prefix")
    endif()
endif()

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
if(ASTRA_HAVE_OPENNI2)
    include_directories(${OPENNI2_INCLUDE_DIR})
endif()
add_definitions(${PCL_DEFINITIONS})

set(SOURCES
    src/main.cpp
    src/gui/MainWindow.cpp
    src/gui/LiveCloudWindow.cpp
    src/gui/SettingsDialog.cpp
    src/capture/AstraCamera.cpp
    src/capture/CaptureWorker.cpp
    src/calibration/CameraCalibrator.cpp
    src/filters/PointCloudFilters.cpp
    src/settings/SettingsManager.cpp
    src/export/ExportManager.cpp
    src/project/ProjectManager.cpp
    src/accel/GpuAccelerator.cpp
)

if(ASTRA_HAVE_CUDA)
    list(APPEND SOURCES src/accel/DepthToCloudKernel.cu)
endif()

# На Windows GUI-приложение скрывает консоль (WIN32_EXECUTABLE / /SUBSYSTEM:WINDOWS);
# на Linux/macOS этот флаг ничего полезного не даёт.
if(WIN32)
    add_executable(${PROJECT_NAME} WIN32 ${SOURCES})
else()
    add_executable(${PROJECT_NAME} ${SOURCES})
endif()

if(ASTRA_HAVE_OPENNI2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ASTRA_HAVE_OPENNI2)
endif()
if(ASTRA_HAVE_CUDA)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ASTRA_HAVE_CUDA)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_ARCHITECTURES "50;60;70;75;80;86;89")
endif()

target_link_libraries(${PROJECT_NAME}
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::Concurrent
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
)
if(ASTRA_HAVE_OPENNI2)
    target_link_libraries(${PROJECT_NAME} ${OPENNI2_LIBRARY})
endif()
if(ASTRA_HAVE_CUDA)
    target_link_libraries(${PROJECT_NAME} CUDA::cudart)
endif()
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

# --- Post-build: копирование рантайм-зависимостей (только Windows) ---
# На Linux/macOS пользователь обычно полагается на системный загрузчик и
# LD_LIBRARY_PATH; post-build копирование нужно только для сборки в
# переносимую папку рядом с .exe на Windows.
if(WIN32)
    if(ASTRA_HAVE_OPENNI2 AND DEFINED OPENNI2_DLL)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E echo "--- Copying OpenNI2 ---"
            COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>/OpenNI2/Drivers" || (exit 0)
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENNI2_DLL}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>" || (exit 0)
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OPENNI2_DRIVERS}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>/OpenNI2/Drivers" || (exit 0)
        )
    endif()

    if(DEFINED VCPKG_INSTALLED_DIR)
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
    endif()
endif()
