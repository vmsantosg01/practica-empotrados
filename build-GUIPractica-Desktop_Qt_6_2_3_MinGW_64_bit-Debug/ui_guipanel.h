/********************************************************************************
** Form generated from reading UI file 'guipanel.ui'
**
** Created by: Qt User Interface Compiler version 6.2.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GUIPANEL_H
#define UI_GUIPANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>
#include "analogwidgets/counter.h"
#include "analogwidgets/led.h"
#include "analogwidgets/thermometer.h"
#include "qdigitalgauge.h"
#include "qwt_compass.h"
#include "qwt_dial.h"
#include "qwt_text_label.h"

QT_BEGIN_NAMESPACE

class Ui_GUIPanel
{
public:
    QLabel *statusLabel;
    QPushButton *statusButton;
    QTabWidget *tabWidget;
    QWidget *tab;
    QSlider *verticalSlider;
    QSlider *verticalSlider_2;
    QDigitalGauge *qdigitalgauge;
    Counter *counter;
    QwtCompass *Compass;
    QwtTextLabel *Motor_Izquierdo;
    QwtTextLabel *Motor_Izquierdo_2;
    QwtTextLabel *TextLabel;
    QwtTextLabel *TextLabel_2;
    QwtTextLabel *TextLabel_3;
    QwtTextLabel *TextLabel_4;
    QwtTextLabel *TextLabel_5;
    Led *ledIzq;
    Led *ledDer;
    Led *ledMid;
    QwtTextLabel *ENERGIA_TOTAL;
    Counter *Energy_dial;
    ThermoMeter *thermometer;
    QCheckBox *checkBoxEnergy;
    QCheckBox *checkBoxCaida;
    QCheckBox *checkBoxQuemaMotores;
    QCheckBox *checkBoxEnergyb4k;
    QWidget *tab_2;
    QGroupBox *groupBoxCnx;
    QSplitter *splitter;
    QLabel *serialPortLabel;
    QComboBox *serialPortComboBox;
    QPushButton *runButton;
    QPushButton *pingButton;

    void setupUi(QWidget *GUIPanel)
    {
        if (GUIPanel->objectName().isEmpty())
            GUIPanel->setObjectName(QString::fromUtf8("GUIPanel"));
        GUIPanel->resize(734, 624);
        statusLabel = new QLabel(GUIPanel);
        statusLabel->setObjectName(QString::fromUtf8("statusLabel"));
        statusLabel->setGeometry(QRect(80, 590, 281, 20));
        statusButton = new QPushButton(GUIPanel);
        statusButton->setObjectName(QString::fromUtf8("statusButton"));
        statusButton->setGeometry(QRect(20, 590, 51, 21));
        tabWidget = new QTabWidget(GUIPanel);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(20, 60, 681, 521));
        tabWidget->setLayoutDirection(Qt::RightToLeft);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalSlider = new QSlider(tab);
        verticalSlider->setObjectName(QString::fromUtf8("verticalSlider"));
        verticalSlider->setGeometry(QRect(10, 20, 31, 251));
        verticalSlider->setMinimum(-100);
        verticalSlider->setMaximum(100);
        verticalSlider->setOrientation(Qt::Vertical);
        verticalSlider_2 = new QSlider(tab);
        verticalSlider_2->setObjectName(QString::fromUtf8("verticalSlider_2"));
        verticalSlider_2->setGeometry(QRect(630, 20, 31, 251));
        verticalSlider_2->setMinimum(-100);
        verticalSlider_2->setMaximum(100);
        verticalSlider_2->setOrientation(Qt::Vertical);
        qdigitalgauge = new QDigitalGauge(tab);
        qdigitalgauge->setObjectName(QString::fromUtf8("qdigitalgauge"));
        qdigitalgauge->setGeometry(QRect(410, 20, 151, 131));
        qdigitalgauge->setMinValue(-10.000000000000000);
        qdigitalgauge->setMaxValue(10.000000000000000);
        counter = new Counter(tab);
        counter->setObjectName(QString::fromUtf8("counter"));
        counter->setGeometry(QRect(250, 20, 121, 40));
        Compass = new QwtCompass(tab);
        Compass->setObjectName(QString::fromUtf8("Compass"));
        Compass->setGeometry(QRect(90, 20, 141, 131));
        Compass->setLowerBound(0.000000000000000);
        Compass->setUpperBound(360.000000000000000);
        Compass->setScaleMaxMajor(20);
        Compass->setScaleStepSize(0.000000000000000);
        Compass->setValue(0.000000000000000);
        Compass->setSingleSteps(1u);
        Compass->setLineWidth(4);
        Compass->setMinScaleArc(0.000000000000000);
        Compass->setMaxScaleArc(360.000000000000000);
        Motor_Izquierdo = new QwtTextLabel(tab);
        Motor_Izquierdo->setObjectName(QString::fromUtf8("Motor_Izquierdo"));
        Motor_Izquierdo->setGeometry(QRect(10, 280, 100, 20));
        Motor_Izquierdo_2 = new QwtTextLabel(tab);
        Motor_Izquierdo_2->setObjectName(QString::fromUtf8("Motor_Izquierdo_2"));
        Motor_Izquierdo_2->setGeometry(QRect(560, 280, 100, 20));
        TextLabel = new QwtTextLabel(tab);
        TextLabel->setObjectName(QString::fromUtf8("TextLabel"));
        TextLabel->setGeometry(QRect(110, 150, 100, 20));
        TextLabel_2 = new QwtTextLabel(tab);
        TextLabel_2->setObjectName(QString::fromUtf8("TextLabel_2"));
        TextLabel_2->setGeometry(QRect(430, 150, 100, 20));
        TextLabel_3 = new QwtTextLabel(tab);
        TextLabel_3->setObjectName(QString::fromUtf8("TextLabel_3"));
        TextLabel_3->setGeometry(QRect(260, 70, 100, 20));
        TextLabel_4 = new QwtTextLabel(tab);
        TextLabel_4->setObjectName(QString::fromUtf8("TextLabel_4"));
        TextLabel_4->setGeometry(QRect(40, 140, 21, 20));
        TextLabel_5 = new QwtTextLabel(tab);
        TextLabel_5->setObjectName(QString::fromUtf8("TextLabel_5"));
        TextLabel_5->setGeometry(QRect(610, 140, 21, 20));
        ledIzq = new Led(tab);
        ledIzq->setObjectName(QString::fromUtf8("ledIzq"));
        ledIzq->setGeometry(QRect(140, 210, 16, 16));
        ledIzq->setChecked(false);
        ledDer = new Led(tab);
        ledDer->setObjectName(QString::fromUtf8("ledDer"));
        ledDer->setGeometry(QRect(200, 210, 16, 16));
        ledDer->setChecked(false);
        ledMid = new Led(tab);
        ledMid->setObjectName(QString::fromUtf8("ledMid"));
        ledMid->setGeometry(QRect(170, 260, 16, 16));
        ledMid->setChecked(false);
        ledMid->setColor(QColor(0, 0, 255));
        ENERGIA_TOTAL = new QwtTextLabel(tab);
        ENERGIA_TOTAL->setObjectName(QString::fromUtf8("ENERGIA_TOTAL"));
        ENERGIA_TOTAL->setGeometry(QRect(260, 170, 100, 20));
        Energy_dial = new Counter(tab);
        Energy_dial->setObjectName(QString::fromUtf8("Energy_dial"));
        Energy_dial->setGeometry(QRect(240, 120, 151, 40));
        Energy_dial->setValue(100000);
        Energy_dial->setDigits(6);
        thermometer = new ThermoMeter(tab);
        thermometer->setObjectName(QString::fromUtf8("thermometer"));
        thermometer->setGeometry(QRect(450, 240, 40, 160));
        thermometer->setMaximum(70.000000000000000);
        checkBoxEnergy = new QCheckBox(tab);
        checkBoxEnergy->setObjectName(QString::fromUtf8("checkBoxEnergy"));
        checkBoxEnergy->setEnabled(true);
        checkBoxEnergy->setGeometry(QRect(7, 340, 91, 22));
        checkBoxCaida = new QCheckBox(tab);
        checkBoxCaida->setObjectName(QString::fromUtf8("checkBoxCaida"));
        checkBoxCaida->setGeometry(QRect(7, 370, 91, 22));
        checkBoxQuemaMotores = new QCheckBox(tab);
        checkBoxQuemaMotores->setObjectName(QString::fromUtf8("checkBoxQuemaMotores"));
        checkBoxQuemaMotores->setGeometry(QRect(7, 400, 91, 22));
        checkBoxEnergyb4k = new QCheckBox(tab);
        checkBoxEnergyb4k->setObjectName(QString::fromUtf8("checkBoxEnergyb4k"));
        checkBoxEnergyb4k->setGeometry(QRect(15, 310, 81, 20));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        tabWidget->addTab(tab_2, QString());
        groupBoxCnx = new QGroupBox(GUIPanel);
        groupBoxCnx->setObjectName(QString::fromUtf8("groupBoxCnx"));
        groupBoxCnx->setGeometry(QRect(20, 0, 461, 71));
        splitter = new QSplitter(groupBoxCnx);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setGeometry(QRect(20, 30, 221, 22));
        splitter->setOrientation(Qt::Horizontal);
        serialPortLabel = new QLabel(splitter);
        serialPortLabel->setObjectName(QString::fromUtf8("serialPortLabel"));
        splitter->addWidget(serialPortLabel);
        serialPortComboBox = new QComboBox(splitter);
        serialPortComboBox->setObjectName(QString::fromUtf8("serialPortComboBox"));
        splitter->addWidget(serialPortComboBox);
        runButton = new QPushButton(groupBoxCnx);
        runButton->setObjectName(QString::fromUtf8("runButton"));
        runButton->setGeometry(QRect(270, 30, 98, 27));
        pingButton = new QPushButton(groupBoxCnx);
        pingButton->setObjectName(QString::fromUtf8("pingButton"));
        pingButton->setGeometry(QRect(390, 30, 61, 27));
        QWidget::setTabOrder(serialPortComboBox, pingButton);
        QWidget::setTabOrder(pingButton, runButton);

        retranslateUi(GUIPanel);

        QMetaObject::connectSlotsByName(GUIPanel);
    } // setupUi

    void retranslateUi(QWidget *GUIPanel)
    {
        GUIPanel->setWindowTitle(QCoreApplication::translate("GUIPanel", "GUIPanel", nullptr));
        statusLabel->setText(QCoreApplication::translate("GUIPanel", "Detenido", nullptr));
        statusButton->setText(QCoreApplication::translate("GUIPanel", "Estado:", nullptr));
        qdigitalgauge->setUnits(QCoreApplication::translate("GUIPanel", "cm/s", nullptr));
#if QT_CONFIG(tooltip)
        counter->setToolTip(QString());
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        counter->setWhatsThis(QString());
#endif // QT_CONFIG(whatsthis)
        Motor_Izquierdo->setPlainText(QCoreApplication::translate("GUIPanel", "Motor izquierdo", nullptr));
        Motor_Izquierdo_2->setPlainText(QCoreApplication::translate("GUIPanel", "Motor derecho", nullptr));
        TextLabel->setPlainText(QCoreApplication::translate("GUIPanel", "Direcci\303\263n", nullptr));
        TextLabel_2->setPlainText(QCoreApplication::translate("GUIPanel", "Velocidad", nullptr));
        TextLabel_3->setPlainText(QCoreApplication::translate("GUIPanel", "Distancia recorrida", nullptr));
        TextLabel_4->setPlainText(QCoreApplication::translate("GUIPanel", "0", nullptr));
        TextLabel_5->setPlainText(QCoreApplication::translate("GUIPanel", "0", nullptr));
#if QT_CONFIG(tooltip)
        ledIzq->setToolTip(QCoreApplication::translate("GUIPanel", "Color Led component", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        ledIzq->setWhatsThis(QCoreApplication::translate("GUIPanel", "Led indicator", nullptr));
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(tooltip)
        ledDer->setToolTip(QCoreApplication::translate("GUIPanel", "Color Led component", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        ledDer->setWhatsThis(QCoreApplication::translate("GUIPanel", "Led indicator", nullptr));
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(tooltip)
        ledMid->setToolTip(QCoreApplication::translate("GUIPanel", "Color Led component", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        ledMid->setWhatsThis(QCoreApplication::translate("GUIPanel", "Led indicator", nullptr));
#endif // QT_CONFIG(whatsthis)
        ENERGIA_TOTAL->setPlainText(QCoreApplication::translate("GUIPanel", "Energia Total", nullptr));
#if QT_CONFIG(tooltip)
        Energy_dial->setToolTip(QString());
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        Energy_dial->setWhatsThis(QString());
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(tooltip)
        thermometer->setToolTip(QCoreApplication::translate("GUIPanel", "Shows the pressure", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        thermometer->setWhatsThis(QCoreApplication::translate("GUIPanel", "The bar meter widget displays the pressure attached to it", nullptr));
#endif // QT_CONFIG(whatsthis)
        checkBoxEnergy->setText(QCoreApplication::translate("GUIPanel", "Low Energy", nullptr));
        checkBoxCaida->setText(QCoreApplication::translate("GUIPanel", "Robot Fallen", nullptr));
        checkBoxQuemaMotores->setText(QCoreApplication::translate("GUIPanel", "Hot motors", nullptr));
        checkBoxEnergyb4k->setText(QCoreApplication::translate("GUIPanel", "Energy b4k", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("GUIPanel", "Tab 1", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("GUIPanel", "Tab 2", nullptr));
        groupBoxCnx->setTitle(QCoreApplication::translate("GUIPanel", "Conexi\303\263n USB", nullptr));
        serialPortLabel->setText(QCoreApplication::translate("GUIPanel", "Puerto Serie:", nullptr));
        runButton->setText(QCoreApplication::translate("GUIPanel", "Conectar", nullptr));
        pingButton->setText(QCoreApplication::translate("GUIPanel", "Ping", nullptr));
    } // retranslateUi

};

namespace Ui {
    class GUIPanel: public Ui_GUIPanel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GUIPANEL_H
