/********************************************************************************
** Form generated from reading UI file 'optimization_viewer_dialog.ui'
**
** Created: Fri Jun 3 11:29:43 2011
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OPTIMIZATION_VIEWER_DIALOG_H
#define UI_OPTIMIZATION_VIEWER_DIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_OptimizationViewerDialogBase
{
public:
    QPushButton *mPrevButton;
    QPushButton *mNextButton;
    QLabel *mRankLabel;
    QLabel *mMagnitudeLabel;
    QPushButton *mDoneButton;
    QPushButton *mLoadButton;
    QPushButton *mOptimizationButton;

    void setupUi(QDialog *OptimizationViewerDialogBase)
    {
        if (OptimizationViewerDialogBase->objectName().isEmpty())
            OptimizationViewerDialogBase->setObjectName(QString::fromUtf8("OptimizationViewerDialogBase"));
        OptimizationViewerDialogBase->resize(243, 123);
        mPrevButton = new QPushButton(OptimizationViewerDialogBase);
        mPrevButton->setObjectName(QString::fromUtf8("mPrevButton"));
        mPrevButton->setGeometry(QRect(80, 10, 31, 27));
        mNextButton = new QPushButton(OptimizationViewerDialogBase);
        mNextButton->setObjectName(QString::fromUtf8("mNextButton"));
        mNextButton->setGeometry(QRect(120, 10, 31, 27));
        mRankLabel = new QLabel(OptimizationViewerDialogBase);
        mRankLabel->setObjectName(QString::fromUtf8("mRankLabel"));
        mRankLabel->setGeometry(QRect(10, 40, 151, 17));
        mMagnitudeLabel = new QLabel(OptimizationViewerDialogBase);
        mMagnitudeLabel->setObjectName(QString::fromUtf8("mMagnitudeLabel"));
        mMagnitudeLabel->setGeometry(QRect(10, 60, 161, 17));
        mDoneButton = new QPushButton(OptimizationViewerDialogBase);
        mDoneButton->setObjectName(QString::fromUtf8("mDoneButton"));
        mDoneButton->setGeometry(QRect(140, 90, 92, 27));
        mLoadButton = new QPushButton(OptimizationViewerDialogBase);
        mLoadButton->setObjectName(QString::fromUtf8("mLoadButton"));
        mLoadButton->setGeometry(QRect(10, 10, 51, 27));
        mOptimizationButton = new QPushButton(OptimizationViewerDialogBase);
        mOptimizationButton->setObjectName(QString::fromUtf8("mOptimizationButton"));
        mOptimizationButton->setGeometry(QRect(10, 90, 121, 27));

        retranslateUi(OptimizationViewerDialogBase);

        QMetaObject::connectSlotsByName(OptimizationViewerDialogBase);
    } // setupUi

    void retranslateUi(QDialog *OptimizationViewerDialogBase)
    {
        OptimizationViewerDialogBase->setWindowTitle(QApplication::translate("OptimizationViewerDialogBase", "Optimization viewer", 0, QApplication::UnicodeUTF8));
        mPrevButton->setText(QApplication::translate("OptimizationViewerDialogBase", "<", 0, QApplication::UnicodeUTF8));
        mNextButton->setText(QApplication::translate("OptimizationViewerDialogBase", ">", 0, QApplication::UnicodeUTF8));
        mRankLabel->setText(QApplication::translate("OptimizationViewerDialogBase", "Rank:", 0, QApplication::UnicodeUTF8));
        mMagnitudeLabel->setText(QApplication::translate("OptimizationViewerDialogBase", "Magnitude:", 0, QApplication::UnicodeUTF8));
        mDoneButton->setText(QApplication::translate("OptimizationViewerDialogBase", "Done", 0, QApplication::UnicodeUTF8));
        mLoadButton->setText(QApplication::translate("OptimizationViewerDialogBase", "Load", 0, QApplication::UnicodeUTF8));
        mOptimizationButton->setText(QApplication::translate("OptimizationViewerDialogBase", "Run optimization", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class OptimizationViewerDialogBase: public Ui_OptimizationViewerDialogBase {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OPTIMIZATION_VIEWER_DIALOG_H
