#ifndef DEPTHTOCLOUDKERNEL_H
#define DEPTHTOCLOUDKERNEL_H

// C-linkage интерфейс CUDA-ядра depth→cloud.
// Вызывается из GpuAccelerator::depthToCloudCUDA().
// Компилируется только при ASTRA_HAVE_CUDA.

#ifdef ASTRA_HAVE_CUDA

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// Структура для одной точки, возвращаемой ядром.
// Хранится в page-locked host-памяти для быстрого D→H копирования.
struct CudaPointXYZRGB {
    float x, y, z;
    uint8_t r, g, b;
    uint8_t valid;  // 1 = точка валидна, 0 = пропустить
};

// Запускает CUDA-ядро конвертации depth+color → массив точек.
// depthData: указатель на uint16_t depth-карту (host)
// colorData: указатель на BGR uint8_t (host), 3 канала
// width, height: размеры depth-карты
// fx, fy, cx, cy: интринсики камеры
// stride: шаг прореживания (1 = каждый пиксель, 3 = каждый 3-й)
// outPoints: выходной буфер (host, pre-allocated, size = (w/stride)*(h/stride))
// outCount: количество валидных точек (host, записывается ядром)
// Возвращает 0 при успехе, код ошибки CUDA при неудаче.
int cudaDepthToCloud(
    const uint16_t *depthData,
    const uint8_t  *colorData,
    int width, int height,
    float fx, float fy, float cx, float cy,
    int stride,
    CudaPointXYZRGB *outPoints,
    int *outCount);

#ifdef __cplusplus
}
#endif

#endif // ASTRA_HAVE_CUDA
#endif // DEPTHTOCLOUDKERNEL_H
