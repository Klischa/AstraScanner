#ifndef POINTCLOUDFILTERS_H
#define POINTCLOUDFILTERS_H

#include <QObject>
#include <QPolygonF>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

class AiClient;  // Forward declaration

class PointCloudFilters : public QObject
{
    Q_OBJECT
public:
    // === AI Сегментация ===
    
    // NPMFF-Net параметры сегментации (без обучения)
    struct NPMFFParams {
        float densityThreshold = 0.5f;    // Порог плотности для сегментации
        int minClusterSize = 100;          // Минимальный размер кластера
        float smoothnessWeight = 0.5f;       // Вес сглаживания
        bool useBoundaryDetection = true;    // Использовать детекцию границ
    };
    
    // Результат сегментации NPMFF-Net
    struct SegmentationResult {
        QVector<int> foregroundIndices;  // Индексы точек объекта
        QVector<int> backgroundIndices;   // Индексы фона
        QVector<int> boundaryIndices;  // Индексы границ
        bool success = false;
        QString error;
    };
    
    // Сегментация NPMFF-Net - возвращает индексы для фильтрации
    // (аналог лассо, но автоматическая)
    SegmentationResult segmentNPMFF(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const NPMFFParams &params = {});
    
    // FilterByIndices - фильтрация по индексу (аналог лассо без ручной работы)
    // mode: true = сохранить индексы, false = удалить индексы
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterByIndices(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const QVector<int> &indices,
        bool keepIndices = true);
    
    // === Регистрация BUFFER-X ===
    
    // Параметры BUFFER-X регистрации
    struct BufferXParams {
        bool useICPRefinement = true;      // Использовать ICP для финальной подгонки
        double icpMaxDistance = 0.05;      // Макс. расстояние для ICP
        int icpMaxIterations = 50;        // Макс. итераций ICP
        float fitnessThreshold = 0.01f;   // Порог пригодности
    };
    
    // Результат BUFFER-X регистрации
    struct RegistrationResult {
        Eigen::Matrix4f transformation;     // Матрица трансформации
        Eigen::Matrix4f icpTransformation; // ICP уточнение (если включено)
        float fitness = 0.0f;           // Оценка пригодности (0-1)
        bool converged = false;         // Сошлась ли ICP
        bool success = false;
        QString error;
    };
    
    // Регистрация BUFFER-X (заглушка - требует AI сервис)
    RegistrationResult registerBufferX(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target,
        const BufferXParams &params = {});
    
    // Гибридная регистрация: BUFFER-X + ICP
    RegistrationResult registerHybrid(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target,
        const BufferXParams &params = {});
    
    // DINOReg регистрация (опционально)
    RegistrationResult registerDINO(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target,
        bool useColorImages = false);  // Использовать RGB+Depth изображения
    
    // === Легкая Mesh генерация ===
    
    // Параметры LightweightMR
    struct MeshQualityParams {
        enum Quality { Low, Medium, High };
        Quality quality = Medium;
        float avgEdgeLength = 0.01f;  // Средняя длина ребра
        int targetVertices = 50000; // Целевое число вершин
    };
    
    // Результат генерации Mesh
    struct MeshResult {
        pcl::PolygonMesh mesh;
        int vertexCount = 0;
        int faceCount = 0;
        bool success = false;
        QString error;
    };
    
    // Генерация легкой сетки LightweightMR (заглушка)
    MeshResult generateLightweightMesh(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const MeshQualityParams &params = {});
    
    // === Улучшение и рефайнинг ===
    
    // Параметры SuperPC
    struct SuperPCParams {
        bool denoise = true;           // Удаление шума
        bool fill = true;              // Заполнение дыр
        bool densify = true;            // Увеличение плотности
        bool colorize = false;         // Колоризация
        float strength = 0.5f;         // Сила эффекта (0-1)
    };
    
    // Улучшение SuperPC
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr enhanceSuperPC(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const SuperPCParams &params = {});
    
    // RARE рефайнинг
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr refineRARE(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        float qualityImprovement = 0.15f);  // Ожидаемое улучшение качества
    
    // Установить AI клиент для коммуникации с AIService
    static void setAiClient(AiClient *client);
    
    // === Параметры Poisson ===

    // Параметры Poisson-реконструкции. Значения по умолчанию выбраны под
    // объекты 0.2–0.5 м, отсканированные Astra Pro на расстоянии ~0.5–1.0 м.
    // depth — логарифм размера сетки октодерева (8 = 256^3, 9 = 512^3, …).
    // С increase depth качество растёт, но время и память — экспоненциально.
    // pointWeight (screenedPoissonWeight) >0 включает screened-вариант:
    // восстановленная поверхность ближе следует исходным точкам.
    struct PoissonParams {
        int depth = 9;
        int minDepth = 5;
        float pointWeight = 4.0f;
        float samplesPerNode = 1.5f;
        float scale = 1.1f;
        bool confidence = false;
        bool outputPolygons = false;
        // Радиус поиска соседей для оценки нормалей; 0 → k-nearest вместо
        // radius search (см. kNearest).
        double normalSearchRadius = 0.01;
        int kNearest = 20;

        // --- Ориентация нормалей ---
        // По умолчанию ориентация считается через view-point ≈ centroid - 1 м
        // по оси Z (см. computeNormals в .cpp). Для сильно замкнутых объектов
        // этот эвристический view-point может давать противоположные нормали;
        // флаги ниже позволяют задать ориентацию вручную.
        bool useCustomViewpoint = false;
        float viewpointX = 0.0f;
        float viewpointY = 0.0f;
        float viewpointZ = 0.0f;
        // Инвертировать все нормали после оценки (полезно, если Poisson-меш
        // получился «вывернут наизнанку»).
        bool flipNormals = false;
        // Согласованная ориентация через BFS-propagation по k-nearest соседям:
        // стартуем с точки, ближайшей к view-point, и разворачиваем нормали
        // так, чтобы соседние были сонаправлены. Помогает для замкнутых
        // объектов, где единый view-point не даёт корректной ориентации.
        // Работает только на одной связной компоненте облака.
        bool consistentOrientation = false;
        int orientationKNeighbors = 10;
    };

    explicit PointCloudFilters(QObject *parent = nullptr);

    // Фильтры очистки
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyStatisticalOutlierRemoval(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        int meanK = 50, double stddevMulThresh = 1.0);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyRadiusOutlierRemoval(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        double radius = 0.02, int minNeighbors = 10);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyVoxelGrid(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        float leafSize = 0.005f);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyMagicWand(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        float leafSize = 0.005f, int meanK = 50, double stddevMulThresh = 1.0);

    // Регистрация (ICP)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registerPointCloudsICP(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target,
        double maxCorrespondenceDistance = 0.05,
        int maximumIterations = 50);

    // Объединение нескольких сканов: последовательная pairwise-регистрация
    // через ICP. scans[0] используется как «якорь» (reference), каждый
    // следующий скан выравнивается относительно уже накопленного облака.
    // При сбоях ICP на отдельном скане он либо добавляется «как есть»
    // (skipNonConverged=false), либо пропускается (true). Дополнительно
    // применяется финальная воксельная децимация, если voxelLeafOut > 0,
    // чтобы объединённое облако не раздувалось в разы.
    struct MergeParams {
        double maxCorrespondenceDistance = 0.05;   // 5 см
        int maximumIterations = 50;
        bool skipNonConverged = false;
        double voxelLeafOut = 0.0;                 // 0 → без децимации
    };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeScans(
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &scans,
        const MergeParams &params = {});

    // Poisson Surface Reconstruction: строит замкнутый водонепроницаемый меш
    // из плотного облака точек. Внутри оценивает нормали (если их нет),
    // ориентирует их, затем запускает pcl::Poisson. Возвращает непустой
    // pcl::PolygonMesh при успехе; при ошибке mesh.polygons будет пустым и
    // сообщение уйдёт через qWarning. Может занимать секунды-минуты на
    // плотных облаках; вызывать из worker-потока (QtConcurrent::run и т.п.).
    pcl::PolygonMesh reconstructPoissonMesh(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const PoissonParams &params = {});

signals:
    void progressUpdated(int percentage);
    void filterCompleted(const QString &filterName, int pointsBefore, int pointsAfter);

private:
    void computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr &normals);
};

#endif // POINTCLOUDFILTERS_H