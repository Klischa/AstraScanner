#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include <QDialog>

QT_BEGIN_NAMESPACE
class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;
class QLineEdit;
class QPushButton;
class QDialogButtonBox;
QT_END_NAMESPACE

// Единый GUI-диалог для редактирования параметров приложения, которые
// хранятся в SettingsManager (QSettings). До этого диалога все параметры
// правились через разрозненные UI-группы на вкладках «Сканирование»,
// «Обработка» и т.п. — это было разбросано и часть значений (например,
// частота кадров или кол-во ключевых параметров Poisson) вообще нигде в
// UI не светилась и читалась только из QSettings.
//
// Диалог — модальный, OK/Cancel/Apply/Reset. OK и Apply вызывают save()
// и эмитят SettingsManager::settingsChanged на все затронутые ключи.
// Reset возвращает дефолты прямо в UI (не трогает QSettings, пока не
// нажат OK/Apply).
class SettingsDialog : public QDialog
{
    Q_OBJECT
public:
    explicit SettingsDialog(QWidget *parent = nullptr);

private slots:
    void onApply();
    void onResetDefaults();
    void onAccepted();
    void onBrowseProjectsDir();
    void onBrowseExportDir();

private:
    void buildUi();
    void loadFromSettings();
    void saveToSettings();

    // Сканирование / захват
    QSpinBox       *m_scanTimeout = nullptr;
    QDoubleSpinBox *m_voxelLeaf = nullptr;
    QSpinBox       *m_frameSkip = nullptr;

    // Фильтры
    QSpinBox       *m_sorMeanK = nullptr;
    QDoubleSpinBox *m_sorStddev = nullptr;
    QDoubleSpinBox *m_rorRadius = nullptr;
    QSpinBox       *m_rorMinNb = nullptr;

    // ICP
    QDoubleSpinBox *m_icpMaxCorr = nullptr;
    QSpinBox       *m_icpMaxIter = nullptr;
    QCheckBox      *m_icpSkipNonConv = nullptr;
    QDoubleSpinBox *m_icpVoxelOut = nullptr;

    // Poisson
    QSpinBox       *m_poissonDepth = nullptr;
    QDoubleSpinBox *m_poissonPointWeight = nullptr;
    QDoubleSpinBox *m_poissonSamples = nullptr;
    QDoubleSpinBox *m_poissonNormalRadius = nullptr;
    QSpinBox       *m_poissonKNearest = nullptr;

    // Ускорение
    QSpinBox       *m_ompThreads = nullptr;

    // Пути
    QLineEdit      *m_projectsDir = nullptr;
    QLineEdit      *m_exportDir = nullptr;
    QPushButton    *m_browseProjects = nullptr;
    QPushButton    *m_browseExport = nullptr;

    QDialogButtonBox *m_buttons = nullptr;
};

#endif // SETTINGSDIALOG_H
