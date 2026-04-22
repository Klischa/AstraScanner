// CUDA-ядро: конвертация depth-карты + RGB → массив 3D-точек.
// Каждый CUDA-поток обрабатывает один пиксель (с учётом stride).
// Результат записывается в pre-allocated host-буфер через unified memory
// или явное D→H копирование.

#include "DepthToCloudKernel.h"

#ifdef ASTRA_HAVE_CUDA

#include <cuda_runtime.h>
#include <cstdio>

// Ядро: один поток = один выходной пиксель.
// gridDim = (outW * outH + blockSize - 1) / blockSize
__global__ void depthToCloudKernel(
    const uint16_t *__restrict__ depth,
    const uint8_t  *__restrict__ color,
    int width, int height,
    int outW, int outH,
    int stride,
    float fx, float fy, float cx, float cy,
    CudaPointXYZRGB *__restrict__ out,
    int *__restrict__ validCount)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= outW * outH) return;

    const int ov = idx / outW;
    const int ou = idx % outW;
    const int v = ov * stride;
    const int u = ou * stride;

    CudaPointXYZRGB pt;
    pt.valid = 0;

    if (v < height && u < width) {
        const uint16_t d = depth[v * width + u];
        if (d != 0) {
            const float z = d * 0.001f;
            const float x = (static_cast<float>(u) - cx) * z / fx;
            const float y = (static_cast<float>(v) - cy) * z / fy;

            if (isfinite(x) && isfinite(y) && isfinite(z)) {
                pt.x = x;
                pt.y = y;
                pt.z = z;
                // color — BGR, 3 канала
                const int ci = (v * width + u) * 3;
                pt.b = color[ci + 0];
                pt.g = color[ci + 1];
                pt.r = color[ci + 2];
                pt.valid = 1;
                atomicAdd(validCount, 1);
            }
        }
    }

    out[idx] = pt;
}

extern "C" int cudaDepthToCloud(
    const uint16_t *depthData,
    const uint8_t  *colorData,
    int width, int height,
    float fx, float fy, float cx, float cy,
    int stride,
    CudaPointXYZRGB *outPoints,
    int *outCount)
{
    const int outW = (width + stride - 1) / stride;
    const int outH = (height + stride - 1) / stride;
    const int totalOut = outW * outH;

    // Выделяем GPU-буферы
    uint16_t *d_depth = nullptr;
    uint8_t  *d_color = nullptr;
    CudaPointXYZRGB *d_out = nullptr;
    int *d_count = nullptr;

    const size_t depthBytes = static_cast<size_t>(width) * height * sizeof(uint16_t);
    const size_t colorBytes = static_cast<size_t>(width) * height * 3;
    const size_t outBytes   = static_cast<size_t>(totalOut) * sizeof(CudaPointXYZRGB);

    cudaError_t err;
    err = cudaMalloc(&d_depth, depthBytes);
    if (err != cudaSuccess) return static_cast<int>(err);
    err = cudaMalloc(&d_color, colorBytes);
    if (err != cudaSuccess) { cudaFree(d_depth); return static_cast<int>(err); }
    err = cudaMalloc(&d_out, outBytes);
    if (err != cudaSuccess) { cudaFree(d_depth); cudaFree(d_color); return static_cast<int>(err); }
    err = cudaMalloc(&d_count, sizeof(int));
    if (err != cudaSuccess) { cudaFree(d_depth); cudaFree(d_color); cudaFree(d_out); return static_cast<int>(err); }

    // H→D копирование
    cudaMemcpy(d_depth, depthData, depthBytes, cudaMemcpyHostToDevice);
    cudaMemcpy(d_color, colorData, colorBytes, cudaMemcpyHostToDevice);
    cudaMemset(d_count, 0, sizeof(int));

    // Запуск ядра
    const int blockSize = 256;
    const int gridSize = (totalOut + blockSize - 1) / blockSize;
    depthToCloudKernel<<<gridSize, blockSize>>>(
        d_depth, d_color, width, height, outW, outH, stride,
        fx, fy, cx, cy, d_out, d_count);

    err = cudaGetLastError();
    if (err != cudaSuccess) {
        cudaFree(d_depth); cudaFree(d_color); cudaFree(d_out); cudaFree(d_count);
        return static_cast<int>(err);
    }

    // D→H копирование результата
    cudaMemcpy(outPoints, d_out, outBytes, cudaMemcpyDeviceToHost);
    cudaMemcpy(outCount, d_count, sizeof(int), cudaMemcpyDeviceToHost);

    cudaFree(d_depth);
    cudaFree(d_color);
    cudaFree(d_out);
    cudaFree(d_count);

    return 0;
}

#endif // ASTRA_HAVE_CUDA
