#ifndef GPUACCELERATOR_H
#define GPUACCELERATOR_H

#include <QObject>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

// Абстракция над GPU/CPU-ускорением тяжёлых операций.
//
// Стратегия выбора бэкенда:
//   1. CUDA   — если собрано с -DASTRA_ENABLE_CUDA=ON и обнаружена NVIDIA GPU.
//              Ускоряет depth→cloud конвертацию на GPU (ядро в DepthToCloudKernel.cu).
//   2. OpenMP — CPU-параллелизация. Работает на любом процессоре (AMD Ryzen 7
//              8700G: 8C/16T, Intel i7 и т.д.). Используется как fallback при
//              отсутствии CUDA или для операций, не портированных на GPU.
//   3. Single-threaded — если ни CUDA, ни OpenMP не доступны.
//
// Пользователь может задать число потоков OpenMP через SettingsManager
// (ключ "accel/ompThreads", 0 = авто).
class GpuAccelerator : public QObject
{
    Q_OBJECT
public:
    enum class Backend {
        CPU_SingleThread,
        CPU_OpenMP,
        CUDA
    };

    static GpuAccelerator &instance();

    // Инициализация: проверяет доступные бэкенды, выбирает лучший.
    // Вызывать один раз при старте приложения.
    void initialize();

    Backend activeBackend() const { return m_backend; }
    QString backendName() const;
    bool isCudaAvailable() const { return m_cudaAvailable; }
    bool isOpenMPAvailable() const { return m_ompAvailable; }
    int ompThreadCount() const { return m_ompThreads; }

    // Задать число потоков OpenMP (0 = авто, определяется по числу ядер).
    void setOmpThreadCount(int threads);

    // --- Ускоренные операции ---

    // Конвертация depth-карты + RGB в облако точек.
    // На CUDA: выгружает depth/color на GPU, запускает ядро, скачивает результат.
    // На OpenMP: параллельный цикл по строкам.
    // На single-thread: обычный вложенный цикл (как было в CaptureWorker).
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthToCloud(
        const cv::Mat &depth, const cv::Mat &color,
        float fx, float fy, float cx, float cy,
        int stride = 3) const;

    // Информация о GPU (для вкладки «Главная» / логов).
    QString gpuInfo() const { return m_gpuInfo; }

signals:
    void backendChanged(const QString &name);

private:
    GpuAccelerator();
    ~GpuAccelerator() = default;

    // Бэкенд-специфичные реализации depthToCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthToCloudCPU(
        const cv::Mat &depth, const cv::Mat &color,
        float fx, float fy, float cx, float cy,
        int stride) const;

#ifdef ASTRA_HAVE_CUDA
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthToCloudCUDA(
        const cv::Mat &depth, const cv::Mat &color,
        float fx, float fy, float cx, float cy,
        int stride) const;
#endif

    Backend m_backend = Backend::CPU_SingleThread;
    bool    m_cudaAvailable = false;
    bool    m_ompAvailable = false;
    int     m_ompThreads = 0;
    QString m_gpuInfo;
};

#endif // GPUACCELERATOR_H
