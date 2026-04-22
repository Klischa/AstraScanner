#ifndef IDEPTHSENSOR_H
#define IDEPTHSENSOR_H

#include <opencv2/core.hpp>
#include <string>

// Абстрактный интерфейс сенсора глубины.
//
// Позволяет подменять реализацию камеры без изменения CaptureWorker и
// остального пайплайна. Текущие реализации:
//   - AstraCamera  — Orbbec Astra Pro через OpenNI2 + UVC (OpenCV)
//
// Планируемые реализации (отдельные эпики):
//   - RealSenseCamera  — Intel RealSense через librealsense2
//   - AzureKinect      — Azure Kinect через Azure Kinect SDK
//   - OrbbecSDKCamera  — Orbbec SDK v2 (Linux native)
//   - FileSensor       — воспроизведение записанных depth+color из файлов
//
// CaptureWorker работает с IDepthSensor* и не знает о конкретной реализации.
class IDepthSensor
{
public:
    virtual ~IDepthSensor() = default;

    // Инициализация сенсора. Возвращает true при успехе.
    // При неудаче текст ошибки доступен через lastError().
    // Реализация может перейти в emulation-mode и всё равно вернуть true.
    virtual bool initialize() = 0;

    // Завершение работы с сенсором. Освобождает все ресурсы.
    virtual void shutdown() = 0;

    // Запуск потоков (depth + color). Вызывается после initialize().
    virtual bool startStreams() = 0;

    // Остановка потоков.
    virtual void stopStreams() = 0;

    // Чтение одного кадра (depth + color). Блокирующий вызов.
    // colorMat: CV_8UC3 (BGR), depthMat: CV_16UC1 (мм).
    // Возвращает false при ошибке или конце потока.
    virtual bool readFrame(cv::Mat &colorMat, cv::Mat &depthMat) = 0;

    // Интринсики камеры (focal length, principal point).
    virtual bool getIntrinsics(float &fx, float &fy, float &cx, float &cy) const = 0;

    // Состояние.
    virtual bool isInitialized() const = 0;
    virtual bool isEmulationActive() const = 0;
    virtual std::string getLastError() const = 0;

    // Человекочитаемое имя сенсора (для GUI / логов).
    virtual std::string sensorName() const = 0;
};

#endif // IDEPTHSENSOR_H
