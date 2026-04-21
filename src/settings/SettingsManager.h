#ifndef SETTINGSMANAGER_H
#define SETTINGSMANAGER_H

#include <QObject>
#include <QSettings>
#include <QString>

// Тонкая обёртка над QSettings с типизированными геттерами/сеттерами для
// пользовательских параметров приложения. Все значения хранятся в реестре
// (Windows) или в ~/.config/AstraScanner/AstraScanner.conf (Unix) через
// QSettings с organization "AstraScanner" и application "AstraScanner".
//
// Используется как singleton: SettingsManager::instance(). Сигнал
// settingsChanged эмитится после каждой записи — GUI может подписаться, чтобы
// обновить UI без перезапуска.
class SettingsManager : public QObject
{
    Q_OBJECT
public:
    static SettingsManager &instance();

    // --- Сканирование ---
    int scanTimeoutSec() const;
    void setScanTimeoutSec(int seconds);

    double voxelLeafSize() const;           // метры
    void setVoxelLeafSize(double meters);

    int frameSkip() const;                  // каждый N-й кадр добавляется в облако
    void setFrameSkip(int n);

    // --- Фильтры (дефолты) ---
    int sorMeanK() const;
    void setSorMeanK(int k);

    double sorStddevMul() const;
    void setSorStddevMul(double m);

    double rorRadius() const;               // метры
    void setRorRadius(double meters);

    int rorMinNeighbors() const;
    void setRorMinNeighbors(int n);

    // --- Poisson-реконструкция ---
    int poissonDepth() const;               // typical 8..10
    void setPoissonDepth(int depth);

    double poissonPointWeight() const;      // screenedPoissonWeight
    void setPoissonPointWeight(double w);

    double poissonSamplesPerNode() const;
    void setPoissonSamplesPerNode(double n);

    double poissonNormalRadius() const;     // метры; 0 → k-nearest
    void setPoissonNormalRadius(double meters);

    int poissonKNearest() const;            // если normalRadius == 0
    void setPoissonKNearest(int k);

    // --- Пути ---
    QString projectsDirectory() const;
    void setProjectsDirectory(const QString &path);

    QString lastExportDirectory() const;
    void setLastExportDirectory(const QString &path);

    // Принудительная синхронизация на диск (QSettings делает это сам, но
    // иногда нужно вызвать явно перед выходом).
    void sync();

signals:
    void settingsChanged(const QString &key);

private:
    SettingsManager();
    SettingsManager(const SettingsManager &) = delete;
    SettingsManager &operator=(const SettingsManager &) = delete;

    template <typename T>
    void setValue(const QString &key, const T &value);

    QSettings m_settings;
};

#endif // SETTINGSMANAGER_H
