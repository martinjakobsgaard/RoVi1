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
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QPushButton *_btn_home;
    QPushButton *_btn_place;
    QPushButton *_btn_sparse;
    QPushButton *_btn_pose;
    QPushButton *_performTask;
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
        _btn_home = new QPushButton(dockWidgetContents);
        _btn_home->setObjectName(QStringLiteral("_btn_home"));

        verticalLayout->addWidget(_btn_home);

        _btn_place = new QPushButton(dockWidgetContents);
        _btn_place->setObjectName(QStringLiteral("_btn_place"));

        verticalLayout->addWidget(_btn_place);

        _btn_sparse = new QPushButton(dockWidgetContents);
        _btn_sparse->setObjectName(QStringLiteral("_btn_sparse"));

        verticalLayout->addWidget(_btn_sparse);

        _btn_pose = new QPushButton(dockWidgetContents);
        _btn_pose->setObjectName(QStringLiteral("_btn_pose"));

        verticalLayout->addWidget(_btn_pose);

        _performTask = new QPushButton(dockWidgetContents);
        _performTask->setObjectName(QStringLiteral("_performTask"));

        verticalLayout->addWidget(_performTask);

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
        _btn_home->setText(QApplication::translate("SamplePlugin", "Home Position", Q_NULLPTR));
        _btn_place->setText(QApplication::translate("SamplePlugin", "Place Bottle", Q_NULLPTR));
        _btn_sparse->setText(QApplication::translate("SamplePlugin", "Sparse Stereo", Q_NULLPTR));
        _btn_pose->setText(QApplication::translate("SamplePlugin", "Pose Estimation", Q_NULLPTR));
        _performTask->setText(QApplication::translate("SamplePlugin", "Perform Pick and Place", Q_NULLPTR));
        _label->setText(QApplication::translate("SamplePlugin", "Label", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
