#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QDir>
#include <QSurfaceFormat>
#include <QMetaObject>
#include <cstdlib>
#include "gui/MainWindow.h"

static QFile g_logFile;

void fileMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    Q_UNUSED(context);

    QString level;
    switch (type) {
    case QtDebugMsg:    level = "DEBUG"; break;
    case QtInfoMsg:     level = "INFO";  break;
    case QtWarningMsg:  level = "WARN";  break;
    case QtCriticalMsg: level = "CRIT";  break;
    case QtFatalMsg:    level = "FATAL"; break;
    default:            level = "LOG";   break;
    }
    QString formatted = QString("[%1] %2").arg(level, msg);

    // Вывод в консоль
    QTextStream(stderr) << formatted << Qt::endl;

    // Запись в файл
    if (g_logFile.isOpen()) {
        QTextStream out(&g_logFile);
        out << formatted << Qt::endl;
        out.flush();
    }

    // Безопасная отправка в GUI
    if (MainWindow *mw = g_mainWindow.load(std::memory_order_acquire)) {
        QMetaObject::invokeMethod(mw, "appendLog", Qt::QueuedConnection,
                                  Q_ARG(QString, formatted));
    }
}

int main(int argc, char *argv[])
{
    // Отключаем Media Foundation до того, как OpenCV VideoIO будет впервые
    // проинициализирован. Переменная обязана быть выставлена до первого
    // использования cv::VideoCapture.
#ifdef _WIN32
    _putenv("OPENCV_VIDEOIO_PRIORITY_MSMF=0");
#else
    setenv("OPENCV_VIDEOIO_PRIORITY_MSMF", "0", 1);
#endif

    QSurfaceFormat fmt;
    fmt.setRenderableType(QSurfaceFormat::OpenGL);
    fmt.setVersion(3, 2);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);

    QDir().mkpath("logs");
    g_logFile.setFileName("logs/scanner.log");
    if (!g_logFile.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream(stderr) << "Failed to open log file" << Qt::endl;
    } else {
        QTextStream out(&g_logFile);
        out << Qt::endl << "=== Session started at " << QDateTime::currentDateTime().toString() << " ===" << Qt::endl;
        out.flush();
    }

    qInstallMessageHandler(fileMessageHandler);

    MainWindow w;
    w.show();
    int result = app.exec();

    g_logFile.close();
    return result;
}