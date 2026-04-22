#include "SettingsManager.h"
#include <QCoreApplication>
#include <QDir>
#include <QStandardPaths>

namespace {
constexpr const char *kScanTimeout       = "scan/timeoutSec";
constexpr const char *kVoxelLeaf         = "scan/voxelLeafSize";
constexpr const char *kFrameSkip         = "scan/frameSkip";
constexpr const char *kSorMeanK          = "filters/sorMeanK";
constexpr const char *kSorStddev         = "filters/sorStddevMul";
constexpr const char *kRorRadius         = "filters/rorRadius";
constexpr const char *kRorMinNeighbors   = "filters/rorMinNeighbors";
constexpr const char *kIcpMaxCorrDist    = "icp/maxCorrespondenceDistance";
constexpr const char *kIcpMaxIter        = "icp/maxIterations";
constexpr const char *kIcpSkipNonConv    = "icp/skipNonConverged";
constexpr const char *kIcpVoxelOut       = "icp/voxelLeafOut";
constexpr const char *kPoissonDepth      = "poisson/depth";
constexpr const char *kPoissonPointWt    = "poisson/pointWeight";
constexpr const char *kPoissonSamples    = "poisson/samplesPerNode";
constexpr const char *kPoissonNormalRad  = "poisson/normalRadius";
constexpr const char *kPoissonKNearest   = "poisson/kNearest";
constexpr const char *kProjectsDir       = "paths/projectsDirectory";
constexpr const char *kLastExportDir     = "paths/lastExportDirectory";
}

SettingsManager &SettingsManager::instance()
{
    static SettingsManager s;
    return s;
}

SettingsManager::SettingsManager()
    : m_settings("AstraScanner", "AstraScanner")
{
    // Убеждаемся, что QCoreApplication заполнен: QSettings без явных
    // organization/application будет ругаться иначе. На всякий случай выставим.
    if (QCoreApplication::organizationName().isEmpty()) {
        QCoreApplication::setOrganizationName("AstraScanner");
    }
    if (QCoreApplication::applicationName().isEmpty()) {
        QCoreApplication::setApplicationName("AstraScanner");
    }
}

template <typename T>
void SettingsManager::setValue(const QString &key, const T &value)
{
    m_settings.setValue(key, value);
    emit settingsChanged(key);
}

int SettingsManager::scanTimeoutSec() const
{
    return m_settings.value(kScanTimeout, 300).toInt();
}

void SettingsManager::setScanTimeoutSec(int seconds)
{
    setValue(kScanTimeout, seconds);
}

double SettingsManager::voxelLeafSize() const
{
    return m_settings.value(kVoxelLeaf, 0.002).toDouble();
}

void SettingsManager::setVoxelLeafSize(double meters)
{
    setValue(kVoxelLeaf, meters);
}

int SettingsManager::frameSkip() const
{
    return m_settings.value(kFrameSkip, 3).toInt();
}

void SettingsManager::setFrameSkip(int n)
{
    setValue(kFrameSkip, n);
}

int SettingsManager::sorMeanK() const
{
    return m_settings.value(kSorMeanK, 50).toInt();
}

void SettingsManager::setSorMeanK(int k)
{
    setValue(kSorMeanK, k);
}

double SettingsManager::sorStddevMul() const
{
    return m_settings.value(kSorStddev, 1.0).toDouble();
}

void SettingsManager::setSorStddevMul(double m)
{
    setValue(kSorStddev, m);
}

double SettingsManager::rorRadius() const
{
    return m_settings.value(kRorRadius, 0.02).toDouble();
}

void SettingsManager::setRorRadius(double meters)
{
    setValue(kRorRadius, meters);
}

int SettingsManager::rorMinNeighbors() const
{
    return m_settings.value(kRorMinNeighbors, 5).toInt();
}

void SettingsManager::setRorMinNeighbors(int n)
{
    setValue(kRorMinNeighbors, n);
}

double SettingsManager::icpMaxCorrespondenceDistance() const
{
    return m_settings.value(kIcpMaxCorrDist, 0.05).toDouble();
}

void SettingsManager::setIcpMaxCorrespondenceDistance(double meters)
{
    setValue(kIcpMaxCorrDist, meters);
}

int SettingsManager::icpMaxIterations() const
{
    return m_settings.value(kIcpMaxIter, 50).toInt();
}

void SettingsManager::setIcpMaxIterations(int n)
{
    setValue(kIcpMaxIter, n);
}

bool SettingsManager::icpSkipNonConverged() const
{
    return m_settings.value(kIcpSkipNonConv, false).toBool();
}

void SettingsManager::setIcpSkipNonConverged(bool skip)
{
    setValue(kIcpSkipNonConv, skip);
}

double SettingsManager::icpVoxelLeafOut() const
{
    return m_settings.value(kIcpVoxelOut, 0.0).toDouble();
}

void SettingsManager::setIcpVoxelLeafOut(double meters)
{
    setValue(kIcpVoxelOut, meters);
}

int SettingsManager::poissonDepth() const
{
    return m_settings.value(kPoissonDepth, 9).toInt();
}

void SettingsManager::setPoissonDepth(int depth)
{
    setValue(kPoissonDepth, depth);
}

double SettingsManager::poissonPointWeight() const
{
    return m_settings.value(kPoissonPointWt, 4.0).toDouble();
}

void SettingsManager::setPoissonPointWeight(double w)
{
    setValue(kPoissonPointWt, w);
}

double SettingsManager::poissonSamplesPerNode() const
{
    return m_settings.value(kPoissonSamples, 1.5).toDouble();
}

void SettingsManager::setPoissonSamplesPerNode(double n)
{
    setValue(kPoissonSamples, n);
}

double SettingsManager::poissonNormalRadius() const
{
    return m_settings.value(kPoissonNormalRad, 0.01).toDouble();
}

void SettingsManager::setPoissonNormalRadius(double meters)
{
    setValue(kPoissonNormalRad, meters);
}

int SettingsManager::poissonKNearest() const
{
    return m_settings.value(kPoissonKNearest, 20).toInt();
}

void SettingsManager::setPoissonKNearest(int k)
{
    setValue(kPoissonKNearest, k);
}

QString SettingsManager::projectsDirectory() const
{
    // По умолчанию — ./projects относительно exe. Полный путь вычисляется
    // при первом обращении, чтобы пользователь мог переопределить через GUI.
    QString defaultDir = QDir(QCoreApplication::applicationDirPath()).filePath("projects");
    return m_settings.value(kProjectsDir, defaultDir).toString();
}

void SettingsManager::setProjectsDirectory(const QString &path)
{
    setValue(kProjectsDir, path);
}

QString SettingsManager::lastExportDirectory() const
{
    const QString defaultDir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    return m_settings.value(kLastExportDir, defaultDir).toString();
}

void SettingsManager::setLastExportDirectory(const QString &path)
{
    setValue(kLastExportDir, path);
}

void SettingsManager::sync()
{
    m_settings.sync();
}
