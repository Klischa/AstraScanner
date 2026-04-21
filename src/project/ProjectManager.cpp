#include "ProjectManager.h"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDebug>
#include <pcl/io/ply_io.h>

namespace {
constexpr const char *kMetadataFile = "project.json";
constexpr const char *kScansSubdir  = "scans";
}

ProjectManager::ProjectManager(QObject *parent) : QObject(parent) {}

QString ProjectManager::metadataPath() const
{
    if (m_projectDir.isEmpty()) return {};
    return QDir(m_projectDir).filePath(kMetadataFile);
}

QString ProjectManager::scansDirPath() const
{
    if (m_projectDir.isEmpty()) return {};
    return QDir(m_projectDir).filePath(kScansSubdir);
}

void ProjectManager::setDirty(bool dirty)
{
    if (m_dirty != dirty) {
        m_dirty = dirty;
        emit projectChanged();
    }
}

bool ProjectManager::newProject(const QString &projectDir, const QString &name)
{
    m_lastError.clear();
    QDir dir(projectDir);
    if (!dir.exists() && !dir.mkpath(".")) {
        m_lastError = QString("Не удалось создать директорию проекта: %1").arg(projectDir);
        return false;
    }
    if (!QDir(projectDir).mkpath(kScansSubdir)) {
        m_lastError = QString("Не удалось создать поддиректорию scans/");
        return false;
    }

    m_projectDir = projectDir;
    m_projectName = name.isEmpty() ? QFileInfo(projectDir).fileName() : name;
    m_scans.clear();
    setDirty(true);
    emit scansChanged();

    if (!writeMetadata()) return false;
    setDirty(false);
    qInfo() << "[Project] Created new project at" << projectDir;
    return true;
}

bool ProjectManager::openProject(const QString &projectDir)
{
    m_lastError.clear();
    if (!QFileInfo(projectDir).isDir()) {
        m_lastError = QString("Не директория: %1").arg(projectDir);
        return false;
    }

    m_projectDir = projectDir;
    m_scans.clear();
    if (!readMetadata()) {
        m_projectDir.clear();
        return false;
    }

    setDirty(false);
    emit scansChanged();
    qInfo() << "[Project] Opened project" << m_projectName << "with" << m_scans.size() << "scans";
    return true;
}

bool ProjectManager::saveProject()
{
    if (m_projectDir.isEmpty()) {
        m_lastError = "Проект не открыт";
        return false;
    }
    if (!writeMetadata()) return false;
    setDirty(false);
    return true;
}

bool ProjectManager::saveProjectAs(const QString &projectDir)
{
    const QString oldDir = m_projectDir;
    QDir newDir(projectDir);
    if (!newDir.exists() && !newDir.mkpath(".")) {
        m_lastError = QString("Не удалось создать директорию: %1").arg(projectDir);
        return false;
    }
    if (!QDir(projectDir).mkpath(kScansSubdir)) {
        m_lastError = "Не удалось создать scans/";
        return false;
    }

    m_projectDir = projectDir;
    // Перезаписываем все облака в новом месте, чтобы проект был самодостаточен.
    for (ScanItem &item : m_scans) {
        if (!item.cloud) {
            // Нужно загрузить из старого расположения.
            ScanItem tmp = item;
            tmp.relativeFilePath = QDir(oldDir).filePath(tmp.relativeFilePath);
            if (!loadScanFromDisk(tmp)) {
                m_projectDir = oldDir;
                return false;
            }
            item.cloud = tmp.cloud;
        }
        if (!saveScanToDisk(item)) {
            m_projectDir = oldDir;
            return false;
        }
    }

    if (!writeMetadata()) {
        m_projectDir = oldDir;
        return false;
    }
    setDirty(false);
    emit projectChanged();
    qInfo() << "[Project] Saved project as" << projectDir;
    return true;
}

void ProjectManager::closeProject()
{
    m_projectDir.clear();
    m_projectName.clear();
    m_scans.clear();
    setDirty(false);
    emit projectChanged();
    emit scansChanged();
}

int ProjectManager::addScan(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                            const QString &name)
{
    m_lastError.clear();
    if (!cloud || cloud->empty()) {
        m_lastError = "Пустое облако, добавление отменено";
        return -1;
    }
    if (m_projectDir.isEmpty()) {
        m_lastError = "Проект не открыт — создайте или откройте проект перед добавлением скана";
        return -1;
    }

    ScanItem item;
    item.createdAt = QDateTime::currentDateTime();
    item.name = name.isEmpty()
                    ? QString("Scan %1").arg(m_scans.size() + 1)
                    : name;
    const QString fname = QString("scan_%1.ply")
                              .arg(item.createdAt.toString("yyyyMMdd_hhmmsszzz"));
    item.relativeFilePath = QString("%1/%2").arg(kScansSubdir, fname);
    item.cloud = cloud;
    item.pointCount = static_cast<int>(cloud->size());

    if (!saveScanToDisk(item)) {
        return -1;
    }

    m_scans.append(item);
    setDirty(true);
    writeMetadata();
    setDirty(false);
    emit scansChanged();
    qInfo() << "[Project] Added scan" << item.name << "with" << item.pointCount << "points";
    return m_scans.size() - 1;
}

bool ProjectManager::removeScan(int index)
{
    if (index < 0 || index >= m_scans.size()) return false;
    const QString fullPath = QDir(m_projectDir).filePath(m_scans[index].relativeFilePath);
    QFile::remove(fullPath);
    m_scans.removeAt(index);
    setDirty(true);
    writeMetadata();
    setDirty(false);
    emit scansChanged();
    return true;
}

bool ProjectManager::renameScan(int index, const QString &newName)
{
    if (index < 0 || index >= m_scans.size()) return false;
    if (m_scans[index].name == newName) return true;
    m_scans[index].name = newName;
    setDirty(true);
    writeMetadata();
    setDirty(false);
    emit scansChanged();
    return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProjectManager::scanCloud(int index)
{
    if (index < 0 || index >= m_scans.size()) return nullptr;
    ScanItem &item = m_scans[index];
    if (!item.cloud) {
        loadScanFromDisk(item);
    }
    return item.cloud;
}

bool ProjectManager::writeMetadata()
{
    m_lastError.clear();
    QJsonObject root;
    root["name"] = m_projectName;
    root["formatVersion"] = 1;

    QJsonArray scansArr;
    for (const ScanItem &item : m_scans) {
        QJsonObject s;
        s["name"] = item.name;
        s["file"] = item.relativeFilePath;
        s["createdAt"] = item.createdAt.toString(Qt::ISODate);
        s["pointCount"] = item.pointCount;
        scansArr.append(s);
    }
    root["scans"] = scansArr;

    QFile f(metadataPath());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        m_lastError = QString("Не удалось открыть %1 для записи").arg(metadataPath());
        qWarning() << "[Project]" << m_lastError;
        return false;
    }
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

bool ProjectManager::readMetadata()
{
    m_lastError.clear();
    QFile f(metadataPath());
    if (!f.exists()) {
        m_lastError = QString("project.json не найден в %1").arg(m_projectDir);
        return false;
    }
    if (!f.open(QIODevice::ReadOnly)) {
        m_lastError = QString("Не удалось открыть %1").arg(metadataPath());
        return false;
    }
    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError) {
        m_lastError = QString("Ошибка JSON: %1").arg(err.errorString());
        return false;
    }
    const QJsonObject root = doc.object();
    m_projectName = root.value("name").toString(QFileInfo(m_projectDir).fileName());

    m_scans.clear();
    for (const QJsonValue &v : root.value("scans").toArray()) {
        const QJsonObject s = v.toObject();
        ScanItem item;
        item.name = s.value("name").toString();
        item.relativeFilePath = s.value("file").toString();
        item.createdAt = QDateTime::fromString(s.value("createdAt").toString(), Qt::ISODate);
        item.pointCount = s.value("pointCount").toInt();
        m_scans.append(item);
    }
    return true;
}

bool ProjectManager::saveScanToDisk(ScanItem &item)
{
    if (!item.cloud) {
        m_lastError = "saveScanToDisk: облако не загружено";
        return false;
    }
    const QString fullPath = QDir(m_projectDir).filePath(item.relativeFilePath);
    // Убеждаемся, что директория существует.
    QFileInfo(fullPath).absoluteDir().mkpath(".");

    try {
        const int rc = pcl::io::savePLYFileBinary(fullPath.toStdString(), *item.cloud);
        if (rc != 0) {
            m_lastError = QString("pcl::io::savePLYFileBinary вернул %1 для %2").arg(rc).arg(fullPath);
            return false;
        }
    } catch (const std::exception &ex) {
        m_lastError = QString("Исключение при сохранении скана: %1").arg(ex.what());
        return false;
    }
    item.pointCount = static_cast<int>(item.cloud->size());
    return true;
}

bool ProjectManager::loadScanFromDisk(ScanItem &item)
{
    const QString fullPath = QFileInfo(item.relativeFilePath).isAbsolute()
                                 ? item.relativeFilePath
                                 : QDir(m_projectDir).filePath(item.relativeFilePath);
    if (!QFile::exists(fullPath)) {
        m_lastError = QString("Файл скана не найден: %1").arg(fullPath);
        return false;
    }
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    try {
        const int rc = pcl::io::loadPLYFile(fullPath.toStdString(), *cloud);
        if (rc != 0) {
            m_lastError = QString("pcl::io::loadPLYFile вернул %1 для %2").arg(rc).arg(fullPath);
            return false;
        }
    } catch (const std::exception &ex) {
        m_lastError = QString("Исключение при загрузке скана: %1").arg(ex.what());
        return false;
    }
    item.cloud = cloud;
    item.pointCount = static_cast<int>(cloud->size());
    return true;
}
