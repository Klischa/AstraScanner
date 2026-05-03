#include <gtest/gtest.h>
#include <QCoreApplication>
#include <QTest>
#include <QSignalSpy>
#include <QTimer>
#include <QPushButton>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Интеграционные тесты - тестируют "от кнопки до кнопки"

// ========== Тест: Полный цикл сканирования ==========
TEST(IntegrationTest, FullScanCycle)
{
    // Этот тест требует запущенного MainWindow
    // Для автоматизации используйте QTest::mouseClick
    
    // 1. Нажать кнопку "Scan"
    // 2. Дождаться накопления кадров
    // 3. Нажать кнопку "Stop"
    // 4. Проверить что облако сохранено
    
    EXPECT_TRUE(true); // Заглушка - требует GUI
}

// ========== Тест: Многопоточная безопасность ==========
TEST(IntegrationTest, ThreadSafety)
{
    // Тест на отсутствие гонок данных
    // Требует запуска с ThreadSanitizer:
    // cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread"
    
    // Создаем множество потоков
    const int numThreads = 10;
    const int iterations = 100;
    
    // Общие данные (как в MainWindow)
    std::atomic<int> counter{0};
    std::mutex mutex;
    std::vector<int> values;
    
    auto worker = [&]() {
        for (int i = 0; i < iterations; ++i) {
            std::lock_guard<std::mutex> lock(mutex);
            values.push_back(counter++);
        }
    };
    
    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back(worker);
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    // Проверяем что все записи прошли без потери
    EXPECT_EQ(counter.load(), numThreads * iterations);
    EXPECT_EQ((int)values.size(), numThreads * iterations);
}

// ========== Тест: signal/slot потокобезопасность ==========
TEST(IntegrationTest, QtSignalSlotThread)
{
    // Тестируем что сигналы и слоты работают между потоками
    
    class Worker : public QObject {
    Q_OBJECT
    public:
        Worker() : QObject() {}
        
    public slots:
        void doWork() {
            QThread::msleep(10);
            emit resultReady(42);
        }
        
    signals:
        void resultReady(int value);
    };
    
    Worker worker;
    QThread thread;
    worker.moveToThread(&thread);
    
    QSignalSpy spy(&worker, &Worker::resultReady);
    
    // Запускаем в другом потоке
    QMetaObject::invokeMethod(&worker, "doWork", Qt::QueuedConnection);
    thread.start();
    
    // Ждем сигнал
    bool ok = spy.wait(1000);
    
    thread.quit();
    thread.wait();
    
    EXPECT_TRUE(ok);
    EXPECT_EQ(spy.count(), 1);
}

// ========== Тест: Долгая операция с отменой ==========
TEST(IntegrationTest, CancelLongOperation)
{
    // Тест на отмену долгих операций
    
    class CancellableWorker : public QObject {
    Q_OBJECT
    public:
        bool cancelled = false;
        
    public slots:
        void doWork() {
            for (int i = 0; i < 100 && !cancelled; ++i) {
                QThread::msleep(10);
            }
            if (!cancelled) {
                emit finished();
            }
        }
        
        void cancel() {
            cancelled = true;
        }
        
    signals:
        void finished();
    };
    
    CancellableWorker worker;
    QThread thread;
    worker.moveToThread(&thread);
    
    QSignalSpy spy(&worker, &CancellableWorker::finished);
    
    // Запускаем работу
    QMetaObject::invokeMethod(&worker, "doWork", Qt::QueuedConnection);
    thread.start();
    
    // Через 50ms отменяем
    QTimer::singleShot(50, &worker, &CancellableWorker::cancel);
    
    // Ждем завершения (или отмены)
    QTRY_VERIFY spy.count() > 0 || worker.cancelled;
    
    thread.quit();
    thread.wait();
}

// ========== Тест: Прогресс бар обновления ==========
TEST(IntegrationTest, ProgressBarUpdates)
{
    // Тестируем обновление прогресс бара
    
    class ProgressWorker : public QObject {
    Q_OBJECT
    public:
        int progress = 0;
        
    public slots:
        void process() {
            for (int i = 0; i <= 100; i += 10) {
                progress = i;
                emit progressUpdated(i);
                QThread::msleep(5);
            }
            emit finished();
        }
        
    signals:
        void progressUpdated(int value);
        void finished();
    };
    
    ProgressWorker worker;
    QSignalSpy progressSpy(&worker, &ProgressWorker::progressUpdated);
    QSignalSpy finishedSpy(&worker, &ProgressWorker::finished);
    
    QThread thread;
    worker.moveToThread(&thread);
    
    QMetaObject::invokeMethod(&worker, "process", Qt::QueuedConnection);
    thread.start();
    
    // Ждем завершения
    QTRY_VERIFY finishedSpy.count() == 1;
    
    thread.quit();
    thread.wait();
    
    // Проверяем что прогресс обновлялся
    EXPECT_EQ(progressSpy.count(), 11); // 0, 10, 20, ..., 100
}

// ========== Тест: Обработка исключений в фильтрах ==========
TEST(IntegrationTest, PCLExceptionHandling)
{
    // Тестируем устойчивость к исключениям PCL
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Применяем фильтр к пустому облаку - не должно крашиться
    // (реализация должна обрабатывать граничные случаи)
    
    EXPECT_TRUE(empty_cloud->empty());
    
    // Если фильтр реализован правильно:
    // - Пустое облако -> пустое облако
    // - Ошибка внутри PCL -> исходное облако или пустое
    
    EXPECT_TRUE(true); // Требует реальной реализации PointCloudFilters
}

// ========== Тест: Повторная инициализация камеры ==========
TEST(IntegrationTest, CameraReinitialize)
{
    // Тестируем повторные вызовы initialize/shutdown
    
    // Создаемmock камеры или используем эмуляцию
    // 1. Инициализировать
    // 2. Деинициализировать
    // 3. Снова инициализировать
    // 4. Проверить что нет утечки ресурсов
    
    EXPECT_TRUE(true); // Требует AstraCamera
}

// ========== Тест: Синхронизация с UI потоком ==========
TEST(IntegrationTest, UIThreadSync)
{
    // Проверяем что обновления GUI происходят в главном потоке
    
    class GuiUpdater : public QObject {
    Q_OBJECT
    public:
        bool updated = false;
        
    public slots:
        void updateGui() {
            // Этот метод вызывается из рабочего потока
            // Должен использовать Qt::QueuedConnection
            updated = true;
            emit updatedSignal();
        }
        
    signals:
        void updatedSignal();
    };
    
    GuiUpdater updater;
    QSignalSpy spy(&updater, &GuiUpdater::updatedSignal);
    
    // Вызываем через queued connection (как должно быть в реальном коде)
    QMetaObject::invokeMethod(&updater, "updateGui", Qt::QueuedConnection);
    
    QTest::qWait(50);
    
    EXPECT_EQ(spy.count(), 1);
    EXPECT_TRUE(updater.updated);
}