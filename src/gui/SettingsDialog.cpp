#include "SettingsDialog.h"
#include "../settings/SettingsManager.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>

namespace {

// Дефолты продублированы здесь, чтобы «Reset to defaults» не зависел от
// состояния QSettings. При изменении дефолтов в SettingsManager.cpp эти
// значения нужно обновить синхронно.
constexpr int    kDefaultScanTimeout       = 300;
constexpr double kDefaultVoxelLeaf         = 0.002;
constexpr int    kDefaultFrameSkip         = 3;
constexpr int    kDefaultSorMeanK          = 50;
constexpr double kDefaultSorStddev         = 1.0;
constexpr double kDefaultRorRadius         = 0.02;
constexpr int    kDefaultRorMinNeighbors   = 5;
constexpr double kDefaultIcpMaxCorr        = 0.05;
constexpr int    kDefaultIcpMaxIter        = 50;
constexpr bool   kDefaultIcpSkipNonConv    = false;
constexpr double kDefaultIcpVoxelOut       = 0.0;
constexpr int    kDefaultPoissonDepth      = 9;
constexpr double kDefaultPoissonPointWt    = 4.0;
constexpr double kDefaultPoissonSamples    = 1.5;
constexpr double kDefaultPoissonNormalRad  = 0.01;
constexpr int    kDefaultPoissonKNearest   = 20;

} // namespace

SettingsDialog::SettingsDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle("Настройки → Параметры");
    setModal(true);
    buildUi();
    loadFromSettings();
    resize(640, 560);
}

void SettingsDialog::buildUi()
{
    auto *outer = new QVBoxLayout(this);
    auto *tabs = new QTabWidget(this);
    outer->addWidget(tabs);

    // ---- Вкладка «Сканирование» ----
    auto *scanPage = new QWidget(this);
    auto *scanForm = new QFormLayout(scanPage);

    m_scanTimeout = new QSpinBox(this);
    m_scanTimeout->setRange(10, 3600);
    m_scanTimeout->setSuffix(" сек");
    m_scanTimeout->setToolTip("Жёсткий лимит длительности сессии сканирования.");
    scanForm->addRow("Таймаут сканирования:", m_scanTimeout);

    m_voxelLeaf = new QDoubleSpinBox(this);
    m_voxelLeaf->setRange(0.0001, 0.05);
    m_voxelLeaf->setDecimals(4);
    m_voxelLeaf->setSingleStep(0.0005);
    m_voxelLeaf->setSuffix(" м");
    m_voxelLeaf->setToolTip("Размер ячейки voxel-grid (используется в Magic Wand и «большой» децимации).");
    scanForm->addRow("Voxel leaf size:", m_voxelLeaf);

    m_frameSkip = new QSpinBox(this);
    m_frameSkip->setRange(1, 30);
    m_frameSkip->setToolTip("Каждый N-й кадр добавляется в облако. Больше = меньше точек, но быстрее.");
    scanForm->addRow("Frame skip:", m_frameSkip);

    tabs->addTab(scanPage, "Сканирование");

    // ---- Вкладка «Фильтры» ----
    auto *filtersPage = new QWidget(this);
    auto *filtersLayout = new QVBoxLayout(filtersPage);

    auto *sorGroup = new QGroupBox("SOR (Statistical Outlier Removal)", filtersPage);
    auto *sorForm = new QFormLayout(sorGroup);
    m_sorMeanK = new QSpinBox(this);
    m_sorMeanK->setRange(5, 500);
    sorForm->addRow("meanK:", m_sorMeanK);
    m_sorStddev = new QDoubleSpinBox(this);
    m_sorStddev->setRange(0.1, 10.0);
    m_sorStddev->setSingleStep(0.1);
    sorForm->addRow("stddev mul:", m_sorStddev);
    filtersLayout->addWidget(sorGroup);

    auto *rorGroup = new QGroupBox("ROR (Radius Outlier Removal)", filtersPage);
    auto *rorForm = new QFormLayout(rorGroup);
    m_rorRadius = new QDoubleSpinBox(this);
    m_rorRadius->setRange(0.001, 0.5);
    m_rorRadius->setDecimals(4);
    m_rorRadius->setSingleStep(0.001);
    m_rorRadius->setSuffix(" м");
    rorForm->addRow("Radius:", m_rorRadius);
    m_rorMinNb = new QSpinBox(this);
    m_rorMinNb->setRange(1, 200);
    rorForm->addRow("Min neighbors:", m_rorMinNb);
    filtersLayout->addWidget(rorGroup);

    filtersLayout->addStretch();
    tabs->addTab(filtersPage, "Фильтры");

    // ---- Вкладка «ICP-регистрация» ----
    auto *icpPage = new QWidget(this);
    auto *icpForm = new QFormLayout(icpPage);
    m_icpMaxCorr = new QDoubleSpinBox(this);
    m_icpMaxCorr->setRange(0.001, 1.0);
    m_icpMaxCorr->setDecimals(4);
    m_icpMaxCorr->setSingleStep(0.005);
    m_icpMaxCorr->setSuffix(" м");
    m_icpMaxCorr->setToolTip("maxCorrespondenceDistance — порог для сопоставления точек.");
    icpForm->addRow("Max correspondence distance:", m_icpMaxCorr);

    m_icpMaxIter = new QSpinBox(this);
    m_icpMaxIter->setRange(1, 500);
    icpForm->addRow("Max iterations:", m_icpMaxIter);

    m_icpSkipNonConv = new QCheckBox("Пропускать сканы, на которых ICP не сошёлся", this);
    icpForm->addRow("", m_icpSkipNonConv);

    m_icpVoxelOut = new QDoubleSpinBox(this);
    m_icpVoxelOut->setRange(0.0, 0.05);
    m_icpVoxelOut->setDecimals(4);
    m_icpVoxelOut->setSingleStep(0.001);
    m_icpVoxelOut->setSuffix(" м");
    m_icpVoxelOut->setToolTip("Размер ячейки voxel-grid для финальной децимации. 0 = без децимации.");
    icpForm->addRow("Voxel leaf (output):", m_icpVoxelOut);

    tabs->addTab(icpPage, "ICP");

    // ---- Вкладка «Poisson» ----
    auto *poissonPage = new QWidget(this);
    auto *poissonForm = new QFormLayout(poissonPage);

    m_poissonDepth = new QSpinBox(this);
    m_poissonDepth->setRange(5, 12);
    m_poissonDepth->setToolTip("Глубина октодерева. 8≈256^3, 9≈512^3, 10≈1024^3.");
    poissonForm->addRow("Depth:", m_poissonDepth);

    m_poissonPointWeight = new QDoubleSpinBox(this);
    m_poissonPointWeight->setRange(0.0, 20.0);
    m_poissonPointWeight->setSingleStep(0.5);
    poissonForm->addRow("Point weight:", m_poissonPointWeight);

    m_poissonSamples = new QDoubleSpinBox(this);
    m_poissonSamples->setRange(1.0, 20.0);
    m_poissonSamples->setSingleStep(0.5);
    poissonForm->addRow("Samples/node:", m_poissonSamples);

    m_poissonNormalRadius = new QDoubleSpinBox(this);
    m_poissonNormalRadius->setRange(0.0, 0.1);
    m_poissonNormalRadius->setDecimals(4);
    m_poissonNormalRadius->setSingleStep(0.001);
    m_poissonNormalRadius->setSuffix(" м");
    m_poissonNormalRadius->setToolTip("0 → использовать k-nearest (см. поле k).");
    poissonForm->addRow("Normal radius:", m_poissonNormalRadius);

    m_poissonKNearest = new QSpinBox(this);
    m_poissonKNearest->setRange(5, 200);
    poissonForm->addRow("k (nearest):", m_poissonKNearest);

    tabs->addTab(poissonPage, "Poisson");

    // ---- Вкладка «Пути» ----
    auto *pathsPage = new QWidget(this);
    auto *pathsLayout = new QFormLayout(pathsPage);

    auto *projectsRow = new QHBoxLayout();
    m_projectsDir = new QLineEdit(this);
    m_browseProjects = new QPushButton("Обзор…", this);
    projectsRow->addWidget(m_projectsDir);
    projectsRow->addWidget(m_browseProjects);
    pathsLayout->addRow("Директория проектов:", projectsRow);

    auto *exportRow = new QHBoxLayout();
    m_exportDir = new QLineEdit(this);
    m_browseExport = new QPushButton("Обзор…", this);
    exportRow->addWidget(m_exportDir);
    exportRow->addWidget(m_browseExport);
    pathsLayout->addRow("Директория экспорта:", exportRow);

    tabs->addTab(pathsPage, "Пути");

    // ---- Кнопки ----
    m_buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel |
        QDialogButtonBox::Apply | QDialogButtonBox::RestoreDefaults, this);
    outer->addWidget(m_buttons);

    connect(m_buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked,
            this, &SettingsDialog::onApply);
    connect(m_buttons->button(QDialogButtonBox::RestoreDefaults), &QPushButton::clicked,
            this, &SettingsDialog::onResetDefaults);
    connect(m_buttons, &QDialogButtonBox::accepted, this, &SettingsDialog::onAccepted);
    connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    connect(m_browseProjects, &QPushButton::clicked,
            this, &SettingsDialog::onBrowseProjectsDir);
    connect(m_browseExport, &QPushButton::clicked,
            this, &SettingsDialog::onBrowseExportDir);
}

void SettingsDialog::loadFromSettings()
{
    const SettingsManager &s = SettingsManager::instance();

    m_scanTimeout->setValue(s.scanTimeoutSec());
    m_voxelLeaf->setValue(s.voxelLeafSize());
    m_frameSkip->setValue(s.frameSkip());

    m_sorMeanK->setValue(s.sorMeanK());
    m_sorStddev->setValue(s.sorStddevMul());
    m_rorRadius->setValue(s.rorRadius());
    m_rorMinNb->setValue(s.rorMinNeighbors());

    m_icpMaxCorr->setValue(s.icpMaxCorrespondenceDistance());
    m_icpMaxIter->setValue(s.icpMaxIterations());
    m_icpSkipNonConv->setChecked(s.icpSkipNonConverged());
    m_icpVoxelOut->setValue(s.icpVoxelLeafOut());

    m_poissonDepth->setValue(s.poissonDepth());
    m_poissonPointWeight->setValue(s.poissonPointWeight());
    m_poissonSamples->setValue(s.poissonSamplesPerNode());
    m_poissonNormalRadius->setValue(s.poissonNormalRadius());
    m_poissonKNearest->setValue(s.poissonKNearest());

    m_projectsDir->setText(s.projectsDirectory());
    m_exportDir->setText(s.lastExportDirectory());
}

void SettingsDialog::saveToSettings()
{
    SettingsManager &s = SettingsManager::instance();

    s.setScanTimeoutSec(m_scanTimeout->value());
    s.setVoxelLeafSize(m_voxelLeaf->value());
    s.setFrameSkip(m_frameSkip->value());

    s.setSorMeanK(m_sorMeanK->value());
    s.setSorStddevMul(m_sorStddev->value());
    s.setRorRadius(m_rorRadius->value());
    s.setRorMinNeighbors(m_rorMinNb->value());

    s.setIcpMaxCorrespondenceDistance(m_icpMaxCorr->value());
    s.setIcpMaxIterations(m_icpMaxIter->value());
    s.setIcpSkipNonConverged(m_icpSkipNonConv->isChecked());
    s.setIcpVoxelLeafOut(m_icpVoxelOut->value());

    s.setPoissonDepth(m_poissonDepth->value());
    s.setPoissonPointWeight(m_poissonPointWeight->value());
    s.setPoissonSamplesPerNode(m_poissonSamples->value());
    s.setPoissonNormalRadius(m_poissonNormalRadius->value());
    s.setPoissonKNearest(m_poissonKNearest->value());

    s.setProjectsDirectory(m_projectsDir->text());
    s.setLastExportDirectory(m_exportDir->text());

    s.sync();
}

void SettingsDialog::onApply()
{
    saveToSettings();
}

void SettingsDialog::onAccepted()
{
    saveToSettings();
    accept();
}

void SettingsDialog::onResetDefaults()
{
    m_scanTimeout->setValue(kDefaultScanTimeout);
    m_voxelLeaf->setValue(kDefaultVoxelLeaf);
    m_frameSkip->setValue(kDefaultFrameSkip);

    m_sorMeanK->setValue(kDefaultSorMeanK);
    m_sorStddev->setValue(kDefaultSorStddev);
    m_rorRadius->setValue(kDefaultRorRadius);
    m_rorMinNb->setValue(kDefaultRorMinNeighbors);

    m_icpMaxCorr->setValue(kDefaultIcpMaxCorr);
    m_icpMaxIter->setValue(kDefaultIcpMaxIter);
    m_icpSkipNonConv->setChecked(kDefaultIcpSkipNonConv);
    m_icpVoxelOut->setValue(kDefaultIcpVoxelOut);

    m_poissonDepth->setValue(kDefaultPoissonDepth);
    m_poissonPointWeight->setValue(kDefaultPoissonPointWt);
    m_poissonSamples->setValue(kDefaultPoissonSamples);
    m_poissonNormalRadius->setValue(kDefaultPoissonNormalRad);
    m_poissonKNearest->setValue(kDefaultPoissonKNearest);
    // Пути не сбрасываем — это пользовательский выбор, а не «дефолт»,
    // сохранённый в исходниках.
}

void SettingsDialog::onBrowseProjectsDir()
{
    const QString dir = QFileDialog::getExistingDirectory(
        this, "Директория проектов", m_projectsDir->text());
    if (!dir.isEmpty()) m_projectsDir->setText(dir);
}

void SettingsDialog::onBrowseExportDir()
{
    const QString dir = QFileDialog::getExistingDirectory(
        this, "Директория экспорта", m_exportDir->text());
    if (!dir.isEmpty()) m_exportDir->setText(dir);
}
