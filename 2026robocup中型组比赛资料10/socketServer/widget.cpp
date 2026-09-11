#include "widget.h"
#include "ui_widget.h"
#include <QNetworkInterface>
#define nativeIp "10.0.0.30"
#define goalIp "10.0.0.34"
Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    ui->lineEdit_low1->setText("0");
    ui->lineEdit_low2->setText("0");
    ui->lineEdit_low3->setText("0");
    ui->lineEdit_high1->setText("0");
    ui->lineEdit_high2->setText("0");
    ui->lineEdit_high3->setText("0");
    colorThreadhold[0]=1;//mode is set colorthreadhold
    initSocket();
}

Widget::~Widget()
{
    delete ui;
}
//receive Original image
void Widget::receive(){
    QByteArray ba;
    vector<unsigned char> data(50000);
    Mat image;
    while(udpSocket->hasPendingDatagrams())
    {
        ba.resize(udpSocket->pendingDatagramSize());
        udpSocket->readDatagram(ba.data(), ba.size());
        for(int i=0;i<ba.size();i++){
            data[i]=ba[i];
        }
        image = imdecode(data,IMREAD_COLOR);//图像解码
        //MatToQimageAndShowOriginal(image);
        imshow("原始图像",image);

        waitKey(1);
        //ui->receiveEdit->append(ba);
    }
}
//receive Binarization image
void Widget::receive2(){
    QByteArray ba;
    vector<unsigned char> data(50000);
    Mat image;
    while(udpSocket2->hasPendingDatagrams())
    {
        ba.resize(udpSocket2->pendingDatagramSize());
        udpSocket2->readDatagram(ba.data(), ba.size());
        for(int i=0;i<ba.size();i++){
            data[i]=ba[i];
        }
        image = imdecode(data,IMREAD_COLOR);//图像解码
       // MatToQimageAndShowBinarization(image);
        imshow("二值化后图像",image);
        waitKey(1);
        //ui->receiveEdit->append(ba);
    }
}
//receive first threadhold
void Widget::receive3(){
    QByteArray ba;
    unsigned char bb[6];
    while(udpSocket3->hasPendingDatagrams()){
        ba.resize(udpSocket3->pendingDatagramSize());
        udpSocket3->readDatagram(ba.data(), ba.size());
        for(int i=0;i<=5;i++){
            bb[i]=ba[i];
             colorThreadhold[i+1]=ba[i];
        }
        ui->horizontalSlider_low1->setValue(bb[0]);
        ui->horizontalSlider_low2->setValue(bb[1]);
        ui->horizontalSlider_low3->setValue(bb[2]);
        ui->horizontalSlider_high1->setValue(bb[3]);
        ui->horizontalSlider_high2->setValue(bb[4]);
        ui->horizontalSlider_high3->setValue(bb[5]);

    }
}
//void Widget::MatToQimageAndShowOriginal(Mat mat){
//    QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_ARGB32);
//    ui->original->setPixmap(QPixmap::fromImage(image));

//}
//void Widget::MatToQimageAndShowBinarization(Mat mat){
//    QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888);
//    ui->binarization->setPixmap(QPixmap::fromImage(image));
//}
void Widget::initSocket(){
    udpSocket=new QUdpSocket;
    udpSocket2=new QUdpSocket;
    udpSocket3=new QUdpSocket;
    udpSocket->bind(QHostAddress(nativeIp), 30015);
    udpSocket2->bind(QHostAddress(nativeIp), 30016);
    udpSocket3->bind(QHostAddress(nativeIp), 30017);
    connect(udpSocket, SIGNAL(readyRead()), this, SLOT(receive()));
    connect(udpSocket2, SIGNAL(readyRead()), this, SLOT(receive2()));
    connect(udpSocket3, SIGNAL(readyRead()), this, SLOT(receive3()));
}

void Widget::on_horizontalSlider_low1_valueChanged(int value)
{
    ui->lineEdit_low1->setText(QString::number(value));
    colorThreadhold[1]= value;
    udpSocket->writeDatagram(colorThreadhold,QHostAddress(goalIp),8000);
}

void Widget::on_horizontalSlider_low2_valueChanged(int value)
{
    ui->lineEdit_low2->setText(QString::number(value));
    colorThreadhold[2]= value;
    udpSocket->writeDatagram(colorThreadhold,QHostAddress(goalIp),8000);
}

void Widget::on_horizontalSlider_low3_valueChanged(int value)
{
    ui->lineEdit_low3->setText(QString::number(value));
    colorThreadhold[3]= value;
    udpSocket->writeDatagram(colorThreadhold,QHostAddress(goalIp),8000);
}

void Widget::on_horizontalSlider_high1_valueChanged(int value)
{
    ui->lineEdit_high1->setText(QString::number(value));
    colorThreadhold[4]= value;
    udpSocket->writeDatagram(colorThreadhold,QHostAddress(goalIp),8000);
}

void Widget::on_horizontalSlider_high2_valueChanged(int value)
{
    ui->lineEdit_high2->setText(QString::number(value));
    colorThreadhold[5]= value;
    udpSocket->writeDatagram(colorThreadhold,QHostAddress(goalIp),8000);
}

void Widget::on_horizontalSlider_high3_valueChanged(int value)
{
    ui->lineEdit_high3->setText(QString::number(value));
    colorThreadhold[6]= value;
    udpSocket->writeDatagram(colorThreadhold,QHostAddress(goalIp),8000);
}

void Widget::on_saveBtn_clicked()
{
    QByteArray msg;
    msg[0]=2;//2 is save;
    udpSocket->writeDatagram(msg,QHostAddress(goalIp),8000);
}

void Widget::on_sendBtn_clicked()
{
    int index; //0=yellow 1=blue 2=green 3=violet 4=brown,5=red,6=orange
    index=ui->chooseBox->currentIndex();
    QByteArray msg;
    msg[0]=0;//0 is color;
    msg[1]=index;
    udpSocket->writeDatagram(msg,QHostAddress(goalIp),8000);
}
