#include "GpuAccelerator.h"
#include <QDebug>
#include <cmath>

#ifdef ASTRA_HAVE_CUDA
#include "DepthToCloudKernel.h"
#include <cuda_runtime.h>
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

GpuAccelerator::GpuAccelerator() {}

GpuAccelerator &GpuAccelerator::instance()
{
    static GpuAccelerator s;
    return s;
}

void GpuAccelerator::initialize()
{
    // --- Проверка CUDA ---
#ifdef ASTRA_HAVE_CUDA
    int deviceCount = 0;
    cudaError_t err = cudaGetDeviceCount(&deviceCount);
    if (err == cudaSuccess && deviceCount > 0) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, 0);
        m_cudaAvailable = true;
        m_gpuInfo = QString("CUDA: %1 (%2 MB, SM %3.%4, %5 cores)")
            .arg(prop.name)
            .arg(prop.totalGlobalMem / (1024 * 1024))
            .arg(prop.major).arg(prop.minor)
            .arg(prop.multiProcessorCount);
        qInfo() << "[GpuAccelerator]" << m_gpuInfo;
    } else {
        qInfo() << "[GpuAccelerator] CUDA compiled-in but no GPU found";
    }
#endif

    // --- Проверка OpenMP ---
#ifdef _OPENMP
    m_ompAvailable = true;
    if (m_ompThreads <= 0) {
        m_ompThreads = omp_get_max_threads();
    }
    omp_set_num_threads(m_ompThreads);
    qInfo() << "[GpuAccelerator] OpenMP available:" << m_ompThreads << "threads";
#else
    m_ompAvailable = false;
    qInfo() << "[GpuAccelerator] OpenMP not available";
#endif

    // --- Выбор бэкенда ---
#ifdef ASTRA_HAVE_CUDA
    if (m_cudaAvailable) {
        m_backend = Backend::CUDA;
        qInfo() << "[GpuAccelerator] Using CUDA backend";
    } else
#endif
    if (m_ompAvailable) {
        m_backend = Backend::CPU_OpenMP;
        qInfo() << "[GpuAccelerator] Using OpenMP backend (" << m_ompThreads << "threads)";
    } else {
        m_backend = Backend::CPU_SingleThread;
        qInfo() << "[GpuAccelerator] Using single-threaded CPU backend";
    }

    if (m_gpuInfo.isEmpty()) {
        m_gpuInfo = backendName();
    }
}

QString GpuAccelerator::backendName() const
{
    switch (m_backend) {
    case Backend::CUDA:             return "CUDA (NVIDIA GPU)";
    case Backend::CPU_OpenMP:       return QString("OpenMP (%1 потоков)").arg(m_ompThreads);
    case Backend::CPU_SingleThread: return "CPU (однопоточный)";
    }
    return "Unknown";
}

void GpuAccelerator::setOmpThreadCount(int threads)
{
    m_ompThreads = threads;
#ifdef _OPENMP
    if (threads <= 0) {
        m_ompThreads = omp_get_max_threads();
    }
    omp_set_num_threads(m_ompThreads);
    qInfo() << "[GpuAccelerator] OpenMP threads set to" << m_ompThreads;
#endif
}

// --- Публичный метод: выбирает бэкенд ---
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GpuAccelerator::depthToCloud(
    const cv::Mat &depth, const cv::Mat &color,
    float fx, float fy, float cx, float cy,
    int stride) const
{
#ifdef ASTRA_HAVE_CUDA
    if (m_backend == Backend::CUDA) {
        return depthToCloudCUDA(depth, color, fx, fy, cx, cy, stride);
    }
#endif
    return depthToCloudCPU(depth, color, fx, fy, cx, cy, stride);
}

// --- CPU-реализация (OpenMP или single-thread) ---
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GpuAccelerator::depthToCloudCPU(
    const cv::Mat &depth, const cv::Mat &color,
    float fx, float fy, float cx, float cy,
    int stride) const
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (depth.empty() || color.empty()) return cloud;

    cv::Mat colorResized;
    if (color.cols != depth.cols || color.rows != depth.rows) {
        cv::resize(color, colorResized, cv::Size(depth.cols, depth.rows));
    } else {
        colorResized = color;
    }

    const int width = depth.cols;
    const int height = depth.rows;
    const int outH = (height + stride - 1) / stride;
    const int outW = (width + stride - 1) / stride;

    // Каждый поток собирает свой локальный вектор, потом мержим.
    // Это избегает блокировок на push_back.
#ifdef _OPENMP
    const int numThreads = m_ompThreads > 0 ? m_ompThreads : 1;
    std::vector<std::vector<pcl::PointXYZRGB>> threadClouds(numThreads);

    #pragma omp parallel num_threads(numThreads)
    {
        const int tid = omp_get_thread_num();
        auto &local = threadClouds[tid];
        local.reserve(static_cast<size_t>(outW) * outH / numThreads);

        #pragma omp for schedule(dynamic, 4) nowait
        for (int ov = 0; ov < outH; ++ov) {
            const int v = ov * stride;
            if (v >= height) continue;
            const uint16_t *depthRow = depth.ptr<uint16_t>(v);
            const cv::Vec3b *colorRow = colorResized.ptr<cv::Vec3b>(v);

            for (int ou = 0; ou < outW; ++ou) {
                const int u = ou * stride;
                if (u >= width) continue;

                const uint16_t d = depthRow[u];
                if (d == 0) continue;

                const float z = d * 0.001f;
                const float x = (static_cast<float>(u) - cx) * z / fx;
                const float y = (static_cast<float>(v) - cy) * z / fy;

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

                pcl::PointXYZRGB pt;
                pt.x = x; pt.y = y; pt.z = z;
                const cv::Vec3b &bgr = colorRow[u];
                pt.r = bgr[2]; pt.g = bgr[1]; pt.b = bgr[0];
                local.push_back(pt);
            }
        }
    }

    // Мерж результатов
    size_t total = 0;
    for (auto &tc : threadClouds) total += tc.size();
    cloud->reserve(total);
    for (auto &tc : threadClouds) {
        cloud->insert(cloud->end(), tc.begin(), tc.end());
    }
#else
    // Single-threaded fallback
    cloud->reserve(static_cast<size_t>(outW) * outH / 2);
    for (int v = 0; v < height; v += stride) {
        const uint16_t *depthRow = depth.ptr<uint16_t>(v);
        const cv::Vec3b *colorRow = colorResized.ptr<cv::Vec3b>(v);

        for (int u = 0; u < width; u += stride) {
            const uint16_t d = depthRow[u];
            if (d == 0) continue;

            const float z = d * 0.001f;
            const float x = (static_cast<float>(u) - cx) * z / fx;
            const float y = (static_cast<float>(v) - cy) * z / fy;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            pcl::PointXYZRGB pt;
            pt.x = x; pt.y = y; pt.z = z;
            const cv::Vec3b &bgr = colorRow[u];
            pt.r = bgr[2]; pt.g = bgr[1]; pt.b = bgr[0];
            cloud->push_back(pt);
        }
    }
#endif

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

// --- CUDA-реализация ---
#ifdef ASTRA_HAVE_CUDA
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GpuAccelerator::depthToCloudCUDA(
    const cv::Mat &depth, const cv::Mat &color,
    float fx, float fy, float cx, float cy,
    int stride) const
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (depth.empty() || color.empty()) return cloud;

    cv::Mat colorResized;
    if (color.cols != depth.cols || color.rows != depth.rows) {
        cv::resize(color, colorResized, cv::Size(depth.cols, depth.rows));
    } else {
        colorResized = color;
    }

    // Убеждаемся, что color — continuous BGR
    cv::Mat colorCont;
    if (!colorResized.isContinuous()) {
        colorCont = colorResized.clone();
    } else {
        colorCont = colorResized;
    }

    const int width = depth.cols;
    const int height = depth.rows;
    const int outW = (width + stride - 1) / stride;
    const int outH = (height + stride - 1) / stride;
    const int totalOut = outW * outH;

    std::vector<CudaPointXYZRGB> outBuf(totalOut);
    int validCount = 0;

    int rc = cudaDepthToCloud(
        depth.ptr<uint16_t>(),
        colorCont.ptr<uint8_t>(),
        width, height,
        fx, fy, cx, cy,
        stride,
        outBuf.data(),
        &validCount);

    if (rc != 0) {
        qWarning() << "[GpuAccelerator] CUDA kernel failed (code" << rc
                   << "), falling back to CPU";
        return depthToCloudCPU(depth, color, fx, fy, cx, cy, stride);
    }

    cloud->reserve(validCount);
    for (int i = 0; i < totalOut; ++i) {
        if (!outBuf[i].valid) continue;
        pcl::PointXYZRGB pt;
        pt.x = outBuf[i].x;
        pt.y = outBuf[i].y;
        pt.z = outBuf[i].z;
        pt.r = outBuf[i].r;
        pt.g = outBuf[i].g;
        pt.b = outBuf[i].b;
        cloud->push_back(pt);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}
#endif // ASTRA_HAVE_CUDA
