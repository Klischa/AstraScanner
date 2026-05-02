#include "AiClient.h"
#include <QDebug>
#include <QEventLoop>
#include <QCoreApplication>
#include <QBuffer>
#include <QHttpMultiPart>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

// VTK includes for writing temporary files
#include <vtkSmartPointer.h>
#include <vtkPLYWriter.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>

AiClient::AiClient(QObject *parent)
    : QObject(parent)
    , m_networkManager(new QNetworkAccessManager(this))
    , m_timeoutTimer(new QTimer(this))
{
    // По умолчанию - локальный сервис
    m_serviceUrl = "http://localhost:8000";
    
    // Настройка таймера таймаута
    m_timeoutTimer->setSingleShot(true);
    connect(m_timeoutTimer, &QTimer::timeout, this, &AiClient::onTimeout);
    
    // Проверка статуса сервиса при инициализации
    QTimer::singleShot(500, this, &AiClient::checkServiceStatus);
}

AiClient::~AiClient()
{
    cancelOperation();
}

void AiClient::setServiceUrl(const QString &url)
{
    m_serviceUrl = url;
    checkServiceStatus();
}

void AiClient::checkServiceStatus()
{
    QNetworkRequest request(QUrl(m_serviceUrl + "/"));
    QNetworkReply *reply = m_networkManager->get(request);
    
    connect(reply, &QNetworkReply::finished, [this, reply]() {
        if (reply->error() == QNetworkReply::NoError) {
            bool wasAvailable = m_available;
            m_available = true;
            
            if (!wasAvailable) {
                emit serviceStatusChanged(true);
            }
            
            qDebug() << "AIService available at:" << m_serviceUrl;
        } else {
            m_available = false;
            emit serviceStatusChanged(false);
        }
        reply->deleteLater();
    });
}

void AiClient::cancelOperation()
{
    m_cancelled = true;
    
    if (m_currentReply && m_currentReply->isRunning()) {
        m_currentReply->abort();
    }
    
    m_timeoutTimer->stop();
}

void AiClient::segmentNPMFF(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, const QJsonObject &params)
{
    if (!cloud || cloud->empty()) {
        emit errorOccurred("Empty point cloud");
        emit segmentationFinished(QVector<int>(), false);
        return;
    }
    
    emit segmentationStarted();
    emit progressChanged(10);
    
    QJsonObject payload;
    payload["cloud_data"] = QString(encodeCloudToBase64(cloud).toBase64());
    payload["model"] = "npmff";
    
    if (!params.isEmpty()) {
        payload["params"] = params;
    }
    
    sendRequest("/segment", payload);
    m_currentEndpoint = "segment";
}

void AiClient::registerBUFFERX(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr sourceCloud,
                                pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr targetCloud,
                                bool useICP)
{
    if (!sourceCloud || !targetCloud || sourceCloud->empty() || targetCloud->empty()) {
        emit errorOccurred("Empty point cloud");
        emit registrationFinished(Eigen::Matrix4f::Identity(), false);
        return;
    }
    
    emit registrationStarted();
    emit progressChanged(10);
    
    QJsonObject payload;
    payload["source_cloud"] = QString(encodeCloudToBase64(sourceCloud).toBase64());
    payload["target_cloud"] = QString(encodeCloudToBase64(targetCloud).toBase64());
    payload["model"] = "bufferx";
    payload["use_icp_finish"] = useICP;
    
    sendRequest("/register", payload);
    m_currentEndpoint = "register-bufferx";
}

void AiClient::registerDINO(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr sourceCloud,
                             pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr targetCloud,
                             const QByteArray &colorImage,
                             const QByteArray &depthImage)
{
    if (!sourceCloud || !targetCloud || sourceCloud->empty() || targetCloud->empty()) {
        emit errorOccurred("Empty point cloud");
        emit registrationFinished(Eigen::Matrix4f::Identity(), false);
        return;
    }
    
    emit registrationStarted();
    emit progressChanged(10);
    
    QJsonObject payload;
    payload["source_cloud"] = QString(encodeCloudToBase64(sourceCloud).toBase64());
    payload["target_cloud"] = QString(encodeCloudToBase64(targetCloud).toBase64());
    payload["model"] = "dino";
    
    if (!colorImage.isEmpty()) {
        payload["color_image"] = QString(colorImage.toBase64());
    }
    if (!depthImage.isEmpty()) {
        payload["depth_image"] = QString(depthImage.toBase64());
    }
    
    sendRequest("/register", payload);
    m_currentEndpoint = "register-dino";
}

void AiClient::generateMeshLightweight(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                        MeshQuality quality,
                                        const QString &format)
{
    if (!cloud || cloud->empty()) {
        emit errorOccurred("Empty point cloud");
        emit meshGenerationFinished(QString(), false);
        return;
    }
    
    emit meshGenerationStarted();
    emit progressChanged(10);
    
    QString qualityStr;
    switch (quality) {
        case MeshQuality::Low: qualityStr = "low"; break;
        case MeshQuality::Medium: qualityStr = "medium"; break;
        case MeshQuality::High: qualityStr = "high"; break;
    }
    
    QJsonObject payload;
    payload["cloud_data"] = QString(encodeCloudToBase64(cloud).toBase64());
    payload["model"] = "lightweightmr";
    payload["quality"] = qualityStr;
    payload["format"] = format;
    
    sendRequest("/mesh", payload);
    m_currentEndpoint = "mesh";
}

void AiClient::enhanceSuperPC(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                              const QStringList &operations)
{
    if (!cloud || cloud->empty()) {
        emit errorOccurred("Empty point cloud");
        emit enhancementFinished(QString(), false);
        return;
    }
    
    emit enhancementStarted();
    emit progressChanged(10);
    
    QJsonArray opsArray;
    for (const QString &op : operations) {
        opsArray.append(op);
    }
    
    QJsonObject payload;
    payload["cloud_data"] = QString(encodeCloudToBase64(cloud).toBase64());
    payload["model"] = "superpc";
    payload["operations"] = opsArray;
    
    sendRequest("/enhance", payload);
    m_currentEndpoint = "enhance";
}

void AiClient::refineRARE(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    if (!cloud || cloud->empty()) {
        emit errorOccurred("Empty point cloud");
        emit refinementFinished(QString(), false);
        return;
    }
    
    emit refinementStarted();
    emit progressChanged(10);
    
    QJsonObject payload;
    payload["cloud_data"] = QString(encodeCloudToBase64(cloud).toBase64());
    payload["model"] = "rare";
    
    sendRequest("/enhance", payload);  // RARE использует тот же endpoint
    m_currentEndpoint = "refine";
}

void AiClient::runFullPipeline(const QList<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds,
                           bool useSegmentation,
                           bool useRegistration,
                           bool useMesh,
                           bool useEnhancement,
                           bool useRefinement)
{
    if (clouds.isEmpty()) {
        emit errorOccurred("No point clouds provided");
        emit pipelineFinished(false);
        return;
    }
    
    emit pipelineStarted();
    emit progressChanged(0);
    
    // Кодируем все облака в Base64
    QJsonArray cloudsArray;
    for (const auto &cloud : clouds) {
        cloudsArray.append(QString(encodeCloudToBase64(cloud).toBase64()));
    }
    
    QJsonObject config;
    config["segmentation"] = useSegmentation;
    config["registration"] = useRegistration;
    config["mesh"] = useMesh;
    config["enhancement"] = useEnhancement;
    config["refinement"] = useRefinement;
    
    QJsonObject payload;
    payload["clouds"] = cloudsArray;
    payload["config"] = config;
    
    sendRequest("/pipeline", payload);
    m_currentEndpoint = "pipeline";
}

void AiClient::sendRequest(const QString &endpoint, const QJsonObject &payload)
{
    m_cancelled = false;
    
    QUrl url(m_serviceUrl + endpoint);
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    
    QByteArray data = QJsonDocument(payload).toJson(QJsonDocument::Compact);
    
    m_currentReply = m_networkManager->post(request, data);
    
    // Запускаем таймаут
    m_timeoutTimer->start(m_timeoutMs);
    
    connect(m_currentReply, &QNetworkReply::finished, this, &AiClient::onNetworkReply);
    connect(m_currentReply, &QNetworkReply::uploadProgress, this, [](qint64 sent, qint64 total) {
        qDebug() << "Upload progress:" << sent << "/" << total;
    });
    connect(m_currentReply, &QNetworkReply::downloadProgress, this, [](qint64 received, qint64 total) {
        qDebug() << "Download progress:" << received << "/" << total;
    });
}

void AiClient::onNetworkReply()
{
    m_timeoutTimer->stop();
    
    if (!m_currentReply) {
        return;
    }
    
    if (m_cancelled) {
        m_currentReply->deleteLater();
        m_currentReply = nullptr;
        return;
    }
    
    QByteArray data = m_currentReply->readAll();
    m_currentReply->deleteLater();
    m_currentReply = nullptr;
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    
    if (error.error != QJsonParseError::NoError) {
        m_lastError = "JSON parse error: " + error.errorString();
        emit errorOccurred(m_lastError);
        processResponse(QJsonObject());  // Signal failure
        return;
    }
    
    processResponse(doc.object());
}

void AiClient::processResponse(const QJsonObject &response)
{
    QString status = response["status"].toString();
    
    if (status != "ok") {
        m_lastError = response["error"].toString("Unknown error");
        emit errorOccurred(m_lastError);
    }
    
    // Обновляем прогресс
    emit progressChanged(50);
    
    // Обрабатываем ответ в зависимости от endpoint
    if (m_currentEndpoint == "segment") {
        m_segmentationIndices.clear();
        
        if (status == "ok" && response.contains("result")) {
            QJsonObject result = response["result"].toObject();
            QJsonArray indices = result["indices"].toArray();
            
            for (const QJsonValue &val : indices) {
                m_segmentationIndices.append(val.toInt());
            }
        }
        
        emit progressChanged(100);
        emit segmentationFinished(m_segmentationIndices, status == "ok");
        
    } else if (m_currentEndpoint == "register-bufferx" || m_currentEndpoint == "register-dino") {
        m_transformationMatrix = Eigen::Matrix4f::Identity();
        
        if (status == "ok" && response.contains("result")) {
            QJsonObject result = response["result"].toObject();
            QJsonArray transform = result["transformation"].toArray();
            
            if (transform.size() == 16) {
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        m_transformationMatrix(i, j) = transform[i*4 + j].toDouble();
                    }
                }
            }
        }
        
        emit progressChanged(100);
        emit registrationFinished(m_transformationMatrix, status == "ok");
        
    } else if (m_currentEndpoint == "mesh") {
        m_meshPath.clear();
        
        if (status == "ok" && response.contains("result")) {
            QJsonObject result = response["result"].toObject();
            m_meshPath = result["mesh_path"].toString();
        }
        
        emit progressChanged(100);
        emit meshGenerationFinished(m_meshPath, status == "ok");
        
    } else if (m_currentEndpoint == "enhance") {
        m_enhancedPath.clear();
        
        if (status == "ok" && response.contains("result")) {
            QJsonObject result = response["result"].toObject();
            m_enhancedPath = result["result_path"].toString();
        }
        
        emit progressChanged(100);
        emit enhancementFinished(m_enhancedPath, status == "ok");
        
    } else if (m_currentEndpoint == "refine") {
        m_enhancedPath.clear();
        
        if (status == "ok" && response.contains("result")) {
            QJsonObject result = response["result"].toObject();
            m_enhancedPath = result["result_path"].toString();
        }
        
        emit progressChanged(100);
        emit refinementFinished(m_enhancedPath, status == "ok");
        
    } else if (m_currentEndpoint == "pipeline") {
        emit progressChanged(100);
        emit pipelineFinished(status == "ok");
    }
}

void AiClient::onTimeout()
{
    m_lastError = "Request timeout";
    emit errorOccurred(m_lastError);
    
    // Сигнализируем о неудаче в зависимости от endpoint
    if (m_currentEndpoint == "segment") {
        emit segmentationFinished(QVector<int>(), false);
    } else if (m_currentEndpoint.startsWith("register")) {
        emit registrationFinished(Eigen::Matrix4f::Identity(), false);
    } else if (m_currentEndpoint == "mesh") {
        emit meshGenerationFinished(QString(), false);
    } else if (m_currentEndpoint == "enhance") {
        emit enhancementFinished(QString(), false);
    } else if (m_currentEndpoint == "refine") {
        emit refinementFinished(QString(), false);
    } else if (m_currentEndpoint == "pipeline") {
        emit pipelineFinished(false);
    }
    
    if (m_currentReply) {
        m_currentReply->abort();
        m_currentReply->deleteLater();
        m_currentReply = nullptr;
    }
}

// === Статические утилиты ===

QByteArray AiClient::encodeCloudToBase64(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    if (!cloud || cloud->empty()) {
        return QByteArray();
    }
    
    // Создаем временный файл для PLY
    QTemporaryFile tempFile;
    tempFile.setFileTemplate("XXXXXX.ply");
    tempFile.open();
    
    QString tempPath = tempFile.fileName();
    tempFile.close();
    
    // Сохраняем облако в PLY
    pcl::io::savePLYFile(tempPath.toStdString(), *cloud);
    
    // Читаем файл и кодируем в Base64
    QFile file(tempPath);
    if (file.open(QIODevice::ReadOnly)) {
        QByteArray data = file.readAll();
        file.remove();
        return data;
    }
    
    return QByteArray();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AiClient::decodeCloudFromBase64(const QByteArray &data)
{
    if (data.isEmpty()) {
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    }
    
    // Сохраняем в временный файл
    QTemporaryFile tempFile;
    tempFile.setFileTemplate("XXXXXX.ply");
    tempFile.open();
    tempFile.write(data);
    tempFile.flush();
    
    QString tempPath = tempFile.fileName();
    tempFile.close();
    
    // Загружаем облако
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile(tempPath.toStdString(), *cloud);
    
    // Удаляем временный файл
    QFile::remove(tempPath);
    
    return cloud;
}

QJsonArray AiClient::encodeMatrixToJSON(const Eigen::Matrix4f &matrix)
{
    QJsonArray array;
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            array.append(matrix(i, j));
        }
    }
    
    return array;
}

Eigen::Matrix4f AiClient::decodeMatrixFromJSON(const QJsonArray &json)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    
    if (json.size() != 16) {
        return matrix;
    }
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            matrix(i, j) = json[i * 4 + j].toDouble();
        }
    }
    
    return matrix;
}

QByteArray AiClient::cloudToByteArray(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    return encodeCloudToBase64(cloud);
}