#include "ExportManager.h"
#include <QFileInfo>
#include <QDebug>
#include <QStringList>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <filesystem>

ExportManager::ExportManager(QObject *parent) : QObject(parent) {}

ExportManager::CloudFormat ExportManager::detectCloudFormat(const QString &filename)
{
    const QString suffix = QFileInfo(filename).suffix().toLower();
    if (suffix == "ply") return CloudFormat::PLY;
    if (suffix == "pcd") return CloudFormat::PCD;
    return CloudFormat::Auto;
}

ExportManager::MeshFormat ExportManager::detectMeshFormat(const QString &filename)
{
    const QString suffix = QFileInfo(filename).suffix().toLower();
    if (suffix == "ply") return MeshFormat::PLY;
    if (suffix == "stl") return MeshFormat::STL;
    if (suffix == "obj") return MeshFormat::OBJ;
    return MeshFormat::Auto;
}

bool ExportManager::savePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                                   const QString &filename,
                                   CloudFormat format)
{
    m_lastError.clear();
    emit exportStarted(filename);

    if (!cloud || cloud->empty()) {
        m_lastError = "Облако точек пустое — нечего сохранять.";
        qWarning() << "[Export]" << m_lastError;
        emit exportFinished(filename, false);
        return false;
    }

    if (format == CloudFormat::Auto) {
        format = detectCloudFormat(filename);
        if (format == CloudFormat::Auto) {
            m_lastError = QString("Не удалось определить формат по расширению: %1").arg(filename);
            qWarning() << "[Export]" << m_lastError;
            emit exportFinished(filename, false);
            return false;
        }
    }

    // Создаём родительскую папку, если её нет.
    const std::filesystem::path fsPath = filename.toStdString();
    if (fsPath.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(fsPath.parent_path(), ec);
    }

    int rc = -1;
    const std::string path = filename.toStdString();
    try {
        switch (format) {
        case CloudFormat::PLY:
            // binary = true по умолчанию даёт маленькие файлы; можно сделать
            // переключаемым через настройку, если понадобится.
            rc = pcl::io::savePLYFileBinary(path, *cloud);
            break;
        case CloudFormat::PCD:
            rc = pcl::io::savePCDFileBinaryCompressed(path, *cloud);
            break;
        case CloudFormat::Auto:
            break;
        }
    } catch (const std::exception &ex) {
        m_lastError = QString("Исключение при сохранении: %1").arg(ex.what());
        qCritical() << "[Export]" << m_lastError;
        emit exportFinished(filename, false);
        return false;
    }

    if (rc != 0) {
        m_lastError = QString("pcl::io вернул код %1 для %2").arg(rc).arg(filename);
        qWarning() << "[Export]" << m_lastError;
        emit exportFinished(filename, false);
        return false;
    }

    qInfo() << "[Export] Point cloud saved:" << filename
            << "(" << cloud->size() << "points )";
    emit exportFinished(filename, true);
    return true;
}

bool ExportManager::savePolygonMesh(const pcl::PolygonMesh &mesh,
                                    const QString &filename,
                                    MeshFormat format)
{
    m_lastError.clear();
    emit exportStarted(filename);

    if (mesh.polygons.empty()) {
        m_lastError = "Меш пустой — нечего сохранять.";
        qWarning() << "[Export]" << m_lastError;
        emit exportFinished(filename, false);
        return false;
    }

    if (format == MeshFormat::Auto) {
        format = detectMeshFormat(filename);
        if (format == MeshFormat::Auto) {
            m_lastError = QString("Не удалось определить формат меша по расширению: %1").arg(filename);
            qWarning() << "[Export]" << m_lastError;
            emit exportFinished(filename, false);
            return false;
        }
    }

    const std::filesystem::path fsPath = filename.toStdString();
    if (fsPath.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(fsPath.parent_path(), ec);
    }

    int rc = -1;
    const std::string path = filename.toStdString();
    try {
        switch (format) {
        case MeshFormat::PLY:
            rc = pcl::io::savePolygonFilePLY(path, mesh, /*binary_format=*/true);
            break;
        case MeshFormat::STL:
            rc = pcl::io::savePolygonFileSTL(path, mesh, /*binary_format=*/true);
            break;
        case MeshFormat::OBJ:
            rc = pcl::io::saveOBJFile(path, mesh);
            break;
        case MeshFormat::Auto:
            break;
        }
    } catch (const std::exception &ex) {
        m_lastError = QString("Исключение при сохранении меша: %1").arg(ex.what());
        qCritical() << "[Export]" << m_lastError;
        emit exportFinished(filename, false);
        return false;
    }

    // savePolygonFilePLY/STL возвращают количество записанных элементов (>0),
    // saveOBJFile — 0 при успехе. Нормализуем к bool.
    const bool ok = (format == MeshFormat::OBJ) ? (rc == 0) : (rc > 0);
    if (!ok) {
        m_lastError = QString("pcl::io вернул %1 для меша %2").arg(rc).arg(filename);
        qWarning() << "[Export]" << m_lastError;
        emit exportFinished(filename, false);
        return false;
    }

    qInfo() << "[Export] Mesh saved:" << filename
            << "(" << mesh.polygons.size() << "polygons )";
    emit exportFinished(filename, true);
    return true;
}

QStringList ExportManager::cloudFileFilters()
{
    return {
        "Stanford Polygon (*.ply)",
        "PCD — PCL native (*.pcd)",
    };
}

QStringList ExportManager::meshFileFilters()
{
    return {
        "Stanford Polygon (*.ply)",
        "STL — STereoLithography (*.stl)",
        "Wavefront OBJ (*.obj)",
    };
}
