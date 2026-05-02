#ifndef AICLIENT_H
#define AICLIENT_H

#include <QObject>
#include <QString>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTimer>
#include <QMutex>
#include <QWaitCondition>
#include <QByteArray>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief AiClient - асинхронный клиент для AIService
 * 
 * Обеспечивает асинхронную обработку задач и получение результатов
 * без блокировки графического интерфейса через QNetworkAccessManager.
 * 
 * Поддерживаемые модели:
 * - NPMFF-Net: Сегментация без обучения
 * - BUFFER-X: Геометрическая регистрация
 * - DINOReg: RGB-D регистрация
 * - LightweightMR: Легкая Mesh реконструкция
 * - SuperPC: Улучшение качества
 * - RARE: Рефайнинг
 */
class AiClient : public QObject
{
    Q_OBJECT
public:
    enum class Model {
        NPMFF,       // Сегментация без обучения
        BUFFERX,      // Геометрическая регистрация
        DINOReg,      // RGB-D регистрация
        LightweightMR,// Легкая Mesh реконструкция
        SuperPC,      // Улучшение качества
        RARE          // Рефайнинг
    };
    
    enum class Strategy {
        ICPSimple,      // Только ICP
        BufferXICP,     // BUFFER-X + ICP
        DINOReg         // DINO регистрация
    };
    
    enum class MeshQuality {
        Low,
        Medium,
        High
    };
    
    explicit AiClient(QObject *parent = nullptr);
    ~AiClient();
    
    // Настройка подключения к AIService
    void setServiceUrl(const QString &url);
    QString serviceUrl() const { return m_serviceUrl; }
    
    // Проверка доступности сервиса
    bool isAvailable() const { return m_available; }
    
    // === API методы ===
    
    /**
     * @brief Сегментация NPMFF-Net
     * @param cloud Облако точек для сегментации
     * @param params Дополнительные параметры (опционально)
     */
    void segmentNPMFF(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                     const QJsonObject &params = QJsonObject());
    
    /**
     * @brief Регистрация BUFFER-X
     * @param sourceCloud Исходное облако
     * @param targetCloud Целевое облако
     * @param useICP Использовать ICP для финальной подгонки
     */
    void registerBUFFERX(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr sourceCloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr targetCloud,
                        bool useICP = true);
    
    /**
     * @brief Регистрация DINO
     * @param sourceCloud Исходное облако
     * @param targetCloud Целевое облако
     * @param colorImage Цветное изображение (опционально)
     * @param depthImage Depth изображение (опционально)
     */
    void registerDINO(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr sourceCloud,
                     pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr targetCloud,
                     const QByteArray &colorImage = QByteArray(),
                     const QByteArray &depthImage = QByteArray());
    
    /**
     * @brief Генерация легкой сетки LightweightMR
     * @param cloud Облако точек
     * @param quality Качество сетки
     * @param format Формат вывода (ply/obj)
     */
    void generateMeshLightweight(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                  MeshQuality quality = MeshQuality::Medium,
                                  const QString &format = "ply");
    
    /**
     * @brief Улучшение SuperPC
     * @param cloud Облако точек
     * @param operations Список операций (denoise, fill, densify, colorize)
     */
    void enhanceSuperPC(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                        const QStringList &operations = QStringList());
    
    /**
     * @brief Рефайнинг RARE
     * @param cloud Облако точек
     */
    void refineRARE(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    
    /**
     * @brief Полный AI конвейер
     * @param clouds Список облаков для обработки
     * @param useSegmentation Использовать сегментацию
     * @param useRegistration Использовать регистрацию
     * @param useMesh Генерировать сетку
     * @param useEnhancement Использовать улучшение
     * @param useRefinement Использовать рефайнинг
     */
    void runFullPipeline(const QList<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds,
                       bool useSegmentation = true,
                       bool useRegistration = true,
                       bool useMesh = true,
                       bool useEnhancement = true,
                       bool useRefinement = true);
    
    // === Результаты ===
    
    // Получить индексы для фильтрации (результат сегментации)
    QVector<int> segmentationIndices() const { return m_segmentationIndices; }
    
    // Получить матрицу трансформации (результат регистрации)
    Eigen::Matrix4f transformationMatrix() const { return m_transformationMatrix; }
    
    // Получить путь к сгенерированной сетке
    QString meshPath() const { return m_meshPath; }
    
    // Получить путь к улучшенному облаку
    QString enhancedCloudPath() const { return m_enhancedPath; }
    
    // Получить последнюю ошибку
    QString lastError() const { return m_lastError; }
    
    // === Утилиты ===
    
    // Кодирование облака точек в Base64
    static QByteArray encodeCloudToBase64(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    
    // Декодирование облака точек из Base64
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr decodeCloudFromBase64(const QByteArray &data);
    
    // Кодирование Eigen::Matrix4f в JSON массив
    static QJsonArray encodeMatrixToJSON(const Eigen::Matrix4f &matrix);
    
    // Декодирование Eigen::Matrix4f из JSON массива
    static Eigen::Matrix4f decodeMatrixFromJSON(const QJsonArray &json);
    
signals:
    // Сигналы о статусе операций
    void segmentationStarted();
    void segmentationFinished(const QVector<int> &indices, bool success);
    
    void registrationStarted();
    void registrationFinished(const Eigen::Matrix4f &transform, bool success);
    
    void meshGenerationStarted();
    void meshGenerationFinished(const QString &meshPath, bool success);
    
    void enhancementStarted();
    void enhancementFinished(const QString &resultPath, bool success);
    
    void refinementStarted();
    void refinementFinished(const QString &resultPath, bool success);
    
    void pipelineStarted();
    void pipelineFinished(bool success);
    
    // Сигнал ошибки
    void errorOccurred(const QString &error);
    
    // Сигнал доступности сервиса
    void serviceStatusChanged(bool available);
    
    // Прогресс (0-100)
    void progressChanged(int percent);
    
public slots:
    // Проверка доступности сервиса
    void checkServiceStatus();
    
    // Отмена текущей операции
    void cancelOperation();
    
private slots:
    void onNetworkReply();
    void onTimeout();
    
private:
    void sendRequest(const QString &endpoint, const QJsonObject &payload);
    void processResponse(const QJsonObject &response);
    QByteArray cloudToByteArray(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    
private:
    QNetworkAccessManager *m_networkManager = nullptr;
    QString m_serviceUrl;
    bool m_available = false;
    
    // Текущий запрос
    QNetworkReply *m_currentReply = nullptr;
    QString m_currentEndpoint;
    
    // Результаты
    QVector<int> m_segmentationIndices;
    Eigen::Matrix4f m_transformationMatrix;
    QString m_meshPath;
    QString m_enhancedPath;
    QString m_lastError;
    
    // Таймаут
    QTimer *m_timeoutTimer = nullptr;
    int m_timeoutMs = 30000;
    
    // Мьютекс для потокобезопасности
    QMutex m_mutex;
    
    // Флаг отмены
    std::atomic<bool> m_cancelled{false};
};

#endif // AICLIENT_H