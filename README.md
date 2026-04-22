# AstraScanner — 3D-сканер на базе камеры Orbbec Astra Pro

[![GitHub repo](https://img.shields.io/badge/GitHub-Klischa/AstraScanner-blue?logo=github)](https://github.com/Klischa/AstraScanner)

**AstraScanner** — настольное Windows-приложение для 3D-сканирования на камере **Orbbec Astra Pro** (и совместимых с OpenNI2 устройствах). Захватывает RGB + depth, собирает облако точек в реальном времени, чистит его набором фильтров PCL, склеивает несколько сканов через ICP, строит меш Poisson-реконструкцией и экспортирует результат в **PLY / PCD / STL / OBJ**. Проекты (облака + настройки) сохраняются в виде директории с `project.json` и отдельными `.ply`-сканами.

---

## Основные возможности

| Функция | Описание |
|---------|----------|
| Захват | Одновременный вывод RGB и depth с Astra Pro через OpenNI2 + UVC (OpenCV). Emulation-режим при отсутствии камеры. |
| Калибровка | По шахматной доске (OpenCV). Интринсики сохраняются вместе с `image_size` и масштабируются под реальное разрешение depth-кадра. |
| Накопление облака | В реальном времени, с паузой, сбросом и индикатором расстояния. |
| Фильтры | SOR (Statistical Outlier Removal), ROR (Radius Outlier Removal), Voxel Grid, «Magic Wand» (Voxel + SOR). |
| Нормали | `NormalEstimationOMP` + опциональная переориентация под view-point. |
| Poisson-реконструкция | `pcl::Poisson` с настраиваемыми depth / pointWeight / samplesPerNode; меш рисуется в том же VTK-виджете. |
| ICP-регистрация | `pcl::IterativeClosestPoint` попарно по сканам проекта, c параметрами `maxCorrespondenceDistance` / `maxIterations`, опциональной децимацией результата через voxel-grid и режимом «skip non-converged». |
| Проект | `ProjectManager`: директория с `project.json` + `scans/*.ply`, add / remove / rename / ленивая загрузка сканов. |
| Экспорт | `ExportManager`: облака — PLY (бинарный, с цветом) или PCD; меш — PLY / STL / OBJ. Автодетект формата по расширению. |
| Настройки | `SettingsManager` (QSettings) — хранит параметры ICP / Poisson / фильтров / директории экспорта и проектов между запусками. |
| Асинхронные операции | Poisson и ICP-merge выполняются в `QtConcurrent::run` + `QFutureWatcher`, GUI не блокируется. |
| Логирование | Qt `messageHandler` → `logs/scanner.log` + отдельная вкладка «Логи» в GUI. |

---

## Системные требования

| Компонент | Минимум |
|-----------|---------|
| ОС | Windows 10 / 11 (x64) |
| Тулчейн | Visual Studio 2022 (MSVC v143), CMake 3.22+ |
| CPU | Intel Core i5 / AMD Ryzen 5 |
| ОЗУ | 8 ГБ (16 ГБ рекомендуется для Poisson-реконструкции) |
| GPU | OpenGL 3.2+ |
| Камера | Orbbec Astra Pro (или другая OpenNI2-совместимая) |
| OpenNI2 | 2.3+ (в составе Orbbec SDK) |

---

## Зависимости и сборка

### 1. vcpkg

```bat
git clone https://github.com/Microsoft/vcpkg.git C:\dev\vcpkg
C:\dev\vcpkg\bootstrap-vcpkg.bat
C:\dev\vcpkg\vcpkg integrate install
```

### 2. Библиотеки через vcpkg

```bat
vcpkg install qt6-base qt6-opengl qt6-concurrent opencv4 pcl[core,visualization,surface,registration] vtk
```

> Нужны PCL-компоненты `common io visualization filters registration features surface` (последний — для Poisson) и Qt-модули `Core Widgets OpenGL Concurrent`.

### 3. OpenNI2 SDK

Скачайте Orbbec OpenNI2 SDK и распакуйте, например, в `C:\orbbec\OpenNI_2.3.0.86_...\Win64-Release\sdk`.

### 4. Сборка

Все захардкоженные Windows-пути из предыдущих версий убраны — CMake принимает переопределения через `-D` или переменные окружения:

| Переменная | Назначение | Значение по умолчанию |
|------------|------------|-----------------------|
| `CMAKE_TOOLCHAIN_FILE` или `VCPKG_ROOT` | Тулчейн vcpkg | `C:/dev/vcpkg/...` |
| `OPENNI2_ROOT` | Корень OpenNI2 SDK | `C:/orbbec/OpenNI_2.3.0.86_.../sdk` |
| `VCPKG_INSTALLED_DIR` | Префикс установленных пакетов vcpkg (для post-build копирования DLL) | `C:/dev/vcpkg/installed/x64-windows` |

```bat
git clone https://github.com/Klischa/AstraScanner.git
cd AstraScanner
cmake -B build -S . ^
  -DCMAKE_TOOLCHAIN_FILE=C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake ^
  -DOPENNI2_ROOT=C:/orbbec/OpenNI_2.3.0.86_.../Win64-Release/sdk ^
  -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

Исполняемый файл окажется в `build/Release/AstraScanner.exe`. Post-build-шаги CMake копируют рядом `OpenNI2.dll`, драйверы OpenNI2, Qt-плагин `platforms/` и runtime-DLL из vcpkg.

---

## Структура исходников

```
src/
├── main.cpp                     — точка входа, настройка message handler-а
├── capture/                     — AstraCamera (OpenNI2+UVC), CaptureWorker (поток захвата)
├── calibration/                 — CameraCalibrator (шахматная доска → intrinsics)
├── filters/                     — PointCloudFilters (SOR/ROR/Voxel/MagicWand, ICP merge, Poisson)
├── settings/                    — SettingsManager (QSettings-обёртка, singleton)
├── export/                      — ExportManager (PLY/PCD/STL/OBJ)
├── project/                     — ProjectManager (project.json + scans/*.ply)
└── gui/                         — MainWindow, LiveCloudWindow (QVTKOpenGLNativeWidget)
```

---

## Использование

### Вкладки GUI

Главное окно содержит **6 вкладок**:

1. **Главная** — информация о камере, настройки отображения, индикатор расстояния (read-only).
2. **Сканирование** — preview / scan / pause / stop / clear, индикатор расстояния, кнопки экспорта текущего облака.
3. **Калибровка** — захват кадров шахматной доски, калибровка, сохранение интринсик в `data/camera_calibration.xml`.
4. **Обработка** — фильтры (SOR / ROR / Voxel / Magic Wand), Poisson-реконструкция, ICP-мерж всех сканов проекта.
5. **Проект** — список сканов проекта, контекст-меню (загрузить, переименовать, экспорт, удалить), кнопка «Добавить текущее облако как скан».
6. **Логи** — живой вывод Qt message handler-а (тот же, что пишется в `logs/scanner.log`).

### Меню

- **Файл** → Новый проект / Открыть / Сохранить / Сохранить как.
- **Экспорт** → Экспорт текущего облака (PLY / PCD), Экспорт меша (PLY / STL / OBJ).

### Первый запуск и калибровка

1. Подключите Astra Pro к USB 3.0.
2. Запустите `AstraScanner.exe`.
3. Вкладка «Калибровка» → `Preview` → выставите шахматную доску, «Захватить кадр» минимум 5 раз с разных ракурсов.
4. «Калибровать». При RMS-ошибке ≤ 2 px интринсики сохраняются в `data/camera_calibration.xml`; при бо́льшей ошибке выводится предупреждение.

### Сканирование

- **Preview** — только видеопотоки.
- **Scan** — начать накопление облака. Двигайте камеру / объект.
- **Pause / Stop / Clear** — соответственно.

### Работа с проектом

1. **Файл → Новый проект**, выберите директорию.
2. На вкладке «Проект» добавьте текущее облако кнопкой «Добавить текущее облако как скан».
3. Каждый скан лежит в `project_dir/scans/<name>.ply`, метаданные — в `project.json`.

### Фильтрация

На вкладке «Обработка»:

- **SOR** — удаляет выбросы (mean_k, std_mul).
- **ROR** — требует min_neighbors соседей в радиусе.
- **Voxel Grid** — прореживает облако.
- **Magic Wand** — Voxel + SOR одним нажатием.

Параметры по умолчанию читаются из `SettingsManager` и перезаписываются при каждом изменении.

### ICP-мерж сканов

1. Добавьте в проект **минимум 2** скана.
2. Вкладка «Обработка» → группа «Регистрация сканов (ICP)». Параметры:
   - `maxCorrespondenceDistance` (м) — максимальное расстояние между соответствующими точками.
   - `maxIterations` — число итераций ICP на паре.
   - `skipNonConverged` — если ICP не сошёлся, пропустить скан (иначе добавляется без трансформации).
   - `voxelLeafOut` (м) — опциональная децимация результата voxel-grid'ом; `0` — без децимации.
3. «Объединить все сканы проекта». ICP попарно выравнивает каждый последующий скан с накопленным результатом; прогресс логируется.
4. «Сохранить результат в проект…» — создать новый скан из склеенного облака.

### Poisson-реконструкция

1. Облако должно быть предварительно очищено (SOR / Voxel) и иметь нормали (считаются автоматически).
2. Группа «Реконструкция поверхности (Poisson)»: `depth` (6–12, обычно 8–10), `pointWeight`, `samplesPerNode`.
3. «Построить меш» → меш появляется в VTK-вьюере. Реконструкция идёт в фоновом потоке с прогресс-баром.
4. **Экспорт меша** (меню «Экспорт» или кнопка) → PLY / STL / OBJ. До построения меша кнопка сообщит, что меш пуст.

---

## Проверка исходников

```powershell
.\test.ps1
```

Скрипт проверяет наличие исходных файлов и базовую структуру репозитория. Прогон сборки и тесты железа — только на Windows с подключённой Astra Pro.

---

## Известные ограничения

- **Полная поддержка камеры Astra Pro — только Windows.** Стэк MSVC / OpenNI2 SDK / проприетарные драйверы Orbbec работает только на Windows. На Linux / macOS проект собирается и запускается в **emulation-only режиме** (синтетические кадры) через `-DASTRA_ENABLE_OPENNI2=OFF`; все фильтры, ICP, Poisson, экспорт и работа с проектом полностью функциональны.
- **Поворотный стол / авто-сохранение по таймеру.** Реализован упрощённый режим на вкладке «Сканирование» (группа «Режим поворотный стол»): задаёте интервал и число сканов, после старта система периодически сохраняет текущее облако в проект и очищает накопитель. Калиброванной платформы по-прежнему нет — ICP-мерж из произвольных ракурсов запускается отдельной кнопкой на вкладке «Обработка».
- **Диалог настроек GUI.** Доступен через меню «Настройки → Параметры…» (ярлык `Ctrl+,`). Собирает все параметры `QSettings` (сканирование, фильтры, ICP, Poisson, пути) в одно окно с OK / Cancel / Apply / Reset to defaults.
- **Ориентация нормалей.** На группе «Реконструкция поверхности (Poisson)» добавлена подсекция «Ориентация нормалей (при проблемах с Poisson)» — чекбоксы «Инвертировать нормали», «Согласованная ориентация (BFS по k-соседям)», а также ручной ввод `custom view-point (X/Y/Z, м)`. Для сильно замкнутых объектов эти флаги позволяют исправить «вывернутый наизнанку» меш без пересканирования.

---

## Лицензия

Проект распространяется под **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)**.

- ✅ Можно использовать, модифицировать и распространять бесплатно в личных, образовательных, исследовательских целях.
- ❌ Запрещено любое коммерческое использование (продажа ПО, платные услуги 3D-сканирования, интеграция в коммерческие продукты) без письменного разрешения автора.

Полный текст: [LICENSE.MD](https://github.com/Klischa/AstraScanner/blob/main/LICENSE.MD).

---

## Вклад в проект

1. Форкните репозиторий.
2. Создайте ветку: `git checkout -b feature/my-change`.
3. Закоммитьте изменения: `git commit -m 'Add my change'`.
4. Откройте pull request.

---

## Контакты

- Автор: klischa@yandex.ru
- Репозиторий: <https://github.com/Klischa/AstraScanner>
- Баги / предложения: <https://github.com/Klischa/AstraScanner/issues>
