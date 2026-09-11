#ifndef WIDGET_H
#define WIDGET_H
#include <opencv5/opencv2/opencv.hpp>
#include <opencv5/opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <QWidget>
#include <QByteArray>
#include <QHostAddress>
#include <QUdpSocket>
using namespace cv;
using namespace std;
QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();   
    void initSocket();
private slots:
    void receive();
    void receive2();
    void receive3();
    void on_horizontalSlider_low1_valueChanged(int value);

    void on_horizontalSlider_low2_valueChanged(int value);

    void on_horizontalSlider_low3_valueChanged(int value);

    void on_horizontalSlider_high1_valueChanged(int value);

    void on_horizontalSlider_high2_valueChanged(int value);

    void on_horizontalSlider_high3_valueChanged(int value);



    void on_saveBtn_clicked();

    void on_sendBtn_clicked();

private:
    Ui::Widget *ui;
    QUdpSocket *udpSocket;
    QUdpSocket *udpSocket2;
    QUdpSocket *udpSocket3;
    QByteArray colorThreadhold;
//    void MatToQimageAndShowOriginal(Mat mat);
//    void MatToQimageAndShowBinarization(Mat mat);
};
#endif // WIDGET_H
