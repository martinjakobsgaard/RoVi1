/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QPushButton *_btn_im;
    QPushButton *_btn_scan;
    QPushButton *_btn0;
    QPushButton *_btn1;
    QSpinBox *_spinBox;
    QSlider *_slider;
    QPushButton *_btn_home;
    QPushButton *_btn_place;
    QPushButton *_btn_sparse;
    QLabel *_label;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QStringLiteral("SamplePlugin"));
        SamplePlugin->resize(476, 479);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        _btn_im = new QPushButton(dockWidgetContents);
        _btn_im->setObjectName(QStringLiteral("_btn_im"));

        verticalLayout->addWidget(_btn_im);

        _btn_scan = new QPushButton(dockWidgetContents);
        _btn_scan->setObjectName(QStringLiteral("_btn_scan"));

        verticalLayout->addWidget(_btn_scan);

        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QStringLiteral("_btn0"));

        verticalLayout->addWidget(_btn0);

        _btn1 = new QPushButton(dockWidgetContents);
        _btn1->setObjectName(QStringLiteral("_btn1"));

        verticalLayout->addWidget(_btn1);

        _spinBox = new QSpinBox(dockWidgetContents);
        _spinBox->setObjectName(QStringLiteral("_spinBox"));

        verticalLayout->addWidget(_spinBox);

        _slider = new QSlider(dockWidgetContents);
        _slider->setObjectName(QStringLiteral("_slider"));
        _slider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(_slider);

        _btn_home = new QPushButton(dockWidgetContents);
        _btn_home->setObjectName(QStringLiteral("_btn_home"));

        verticalLayout->addWidget(_btn_home);

        _btn_place = new QPushButton(dockWidgetContents);
        _btn_place->setObjectName(QStringLiteral("_btn_place"));

        verticalLayout->addWidget(_btn_place);

        _btn_sparse = new QPushButton(dockWidgetContents);
        _btn_sparse->setObjectName(QStringLiteral("_btn_sparse"));

        verticalLayout->addWidget(_btn_sparse);

        _label = new QLabel(dockWidgetContents);
        _label->setObjectName(QStringLiteral("_label"));

        verticalLayout->addWidget(_label);


        verticalLayout_2->addLayout(verticalLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "Doc&kWidget", Q_NULLPTR));
        _btn_im->setText(QApplication::translate("SamplePlugin", "Get Image", Q_NULLPTR));
        _btn_scan->setText(QApplication::translate("SamplePlugin", "Get Scan", Q_NULLPTR));
        _btn0->setText(QApplication::translate("SamplePlugin", "Calculate Path", Q_NULLPTR));
        _btn1->setText(QApplication::translate("SamplePlugin", "Run Path", Q_NULLPTR));
        _btn_home->setText(QApplication::translate("SamplePlugin", "Home Position", Q_NULLPTR));
        _btn_place->setText(QApplication::translate("SamplePlugin", "Place Bottle", Q_NULLPTR));
        _btn_sparse->setText(QApplication::translate("SamplePlugin", "Sparse Stereo", Q_NULLPTR));
        _label->setText(QApplication::translate("SamplePlugin", "Label", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
