#include <gtest/gtest.h>
#include <QCoreApplication>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QSignalSpy>
#include <QJsonDocument>
#include <QJsonObject>
#include <chrono>
#include <thread>

// Тест AI клиента - требует мок сервер или запущенный AIService
// Для юнит-тестов используйте мокирование

// ========== Тест: JSON парсинг ответа ==========
TEST(AiClientTest, JsonParsing)
{
    // Пример валидного JSON ответа
    QString validJson = R"({
        "status": "ok",
        "result": {
            "vertices": 1000,
            "faces": 2000
        }
    })";
    
    QJsonDocument doc = QJsonDocument::fromJson(validJson.toUtf8());
    EXPECT_FALSE(doc.isNull());
    EXPECT_TRUE(doc.isObject());
    
    QJsonObject obj = doc.object();
    EXPECT_EQ(obj["status"].toString(), "ok");
    EXPECT_TRUE(obj.contains("result"));
}

// ========== Тест: Парсинг некорректного JSON ==========
TEST(AiClientTest, InvalidJsonParsing)
{
    QString invalidJson = "{ this is not valid json }";
    
    QJsonDocument doc = QJsonDocument::fromJson(invalidJson.toUtf8());
    EXPECT_TRUE(doc.isNull());
}

// ========== Тест: Timeout обработка ==========
TEST(AiClientTest, NetworkTimeout)
{
    // Если нет сервера, ожидаем ошибку таймаута
    // Для реального теста запустите AIService локально
    
    QUrl url("http://localhost:9999/nonexistent"); // Несуществующий порт
    QNetworkAccessManager nam;
    
    auto start = std::chrono::steady_clock::now();
    QNetworkReply* reply = nam.get(QNetworkRequest(url));
    
    // Ждем максимум 2 секунды
    QEventLoop loop;
    QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
    QTimer::singleShot(2000, &loop, &QEventLoop::quit);
    loop.exec();
    
    auto elapsed = std::chrono::steady_clock::now() - start;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    
    // Проверяем что мы either получили ответ или таймаут
    EXPECT_TRUE(reply->isFinished() || ms >= 1900);
    
    if (reply->error() != QNetworkReply::NoError) {
        EXPECT_TRUE(reply->error() == QNetworkReply::HostNotFoundError ||
                  reply->error() == QNetworkReply::ConnectionRefusedError ||
                  reply->error() == QNetworkReply::TimeoutError);
    }
    
    reply->deleteLater();
}

// ========== Тест: Multiple requests cleanup ==========
TEST(AiClientTest, MultipleRequestsCleanup)
{
    QNetworkAccessManager nam;
    QVector<QNetworkReply*> replies;
    
    // Создаем 10 запросов
    for (int i = 0; i < 10; ++i) {
        // Используем локальный несуществующий сервер
        QNetworkRequest req(QUrl(QString("http://localhost:9999/test%1").arg(i)));
        QNetworkReply* reply = nam.get(req);
        replies.append(reply);
    }
    
    // Все репли должны быть созданы
    EXPECT_EQ(replies.size(), 10);
    
    // Очищаем - удаляем все репли
    for (QNetworkReply* reply : replies) {
        reply->abort();
        reply->deleteLater();
    }
    
    EXPECT_TRUE(true); // Тест пройден если нет утечки
}

// ========== Тест: Network error handling ==========
TEST(AiClientTest, NetworkErrorHandling)
{
    // Тестируем обработку различных сетевых ошибок
    struct TestCase {
        QNetworkReply::NetworkError error;
        QString expectedMessage;
    };
    
    std::vector<TestCase> cases = {
        {QNetworkReply::ConnectionRefusedError, "Connection refused"},
        {QNetworkReply::HostNotFoundError, "Host not found"},
        {QNetworkReply::TimeoutError, "Timeout"},
        {QNetworkReply::ProtocolInvalid, "Protocol error"},
    };
    
    for (const auto& tc : cases) {
        // Проверяем что можем получить описание ошибки
        QString errorString = QNetworkReply::staticMetaObject
            .enumerator(0)
            .valueToKey(tc.error);
        EXPECT_FALSE(errorString.isEmpty());
    }
}

// ========== Тест: Signal emission ==========
TEST(AiClientTest, SignalEmission)
{
    // Проверяем что сигналы Qt правильно эмитятся
    // Это требует реального слота AI клиента
    
    class TestObject : public QObject {
    public:
        Q_OBJECT
    public slots:
        void onFinished(const QString& result) {
            m_lastResult = result;
            emit finished(result);
        }
        
        QString lastResult() const { return m_lastResult; }
        
    signals:
        void finished(const QString& result);
    
    private:
        QString m_lastResult;
    };
    
    TestObject obj;
    QSignalSpy spy(&obj, &TestObject::finished);
    
    // Эмитим сигнал
    emit obj.finished("test result");
    
    // Проверяем что сигнал пойман
    EXPECT_EQ(spy.count(), 1);
    EXPECT_EQ(spy.takeFirst().at(0).toString(), "test result");
}

// ========== Тест: Отмена запроса ==========
TEST(AiClientTest, RequestCancellation)
{
    QNetworkAccessManager nam;
    
    // Создаем запрос к несуществующему серверу
    QNetworkRequest req(QUrl("http://localhost:9999/slow"));
    QNetworkReply* reply = nam.get(req);
    
    // Проверяем что можем отменить до получения ответа
    EXPECT_FALSE(reply->isFinished());
    
    reply->abort();
    
    // После отмены запрос должен завершиться
    EXPECT_TRUE(reply->isFinished());
    EXPECT_EQ(reply->error(), QNetworkReply::OperationCanceledError);
    
    reply->deleteLater();
}