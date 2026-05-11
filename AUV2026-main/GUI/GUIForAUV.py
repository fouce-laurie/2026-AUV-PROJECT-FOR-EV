# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'GUIForAUV.ui'
##
## Created by: Qt User Interface Compiler version 6.10.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QListWidget,
    QListWidgetItem, QMainWindow, QMenu, QMenuBar,
    QPushButton, QSizePolicy, QStatusBar, QVBoxLayout,
    QWidget)

class Ui_AUV_UI(object):
    def setupUi(self, AUV_UI):
        if not AUV_UI.objectName():
            AUV_UI.setObjectName(u"AUV_UI")
        AUV_UI.resize(1285, 720)
        self.action = QAction(AUV_UI)
        self.action.setObjectName(u"action")
        self.centralwidget = QWidget(AUV_UI)
        self.centralwidget.setObjectName(u"centralwidget")
        self.mainLayout = QVBoxLayout(self.centralwidget)
        self.mainLayout.setSpacing(10)
        self.mainLayout.setObjectName(u"mainLayout")
        self.mainLayout.setContentsMargins(12, 8, 12, 10)
        self.statement = QLabel(self.centralwidget)
        self.statement.setObjectName(u"statement")
        self.statement.setMinimumSize(QSize(0, 48))
        self.statement.setMaximumSize(QSize(16777215, 48))
        font = QFont()
        font.setPointSize(18)
        font.setBold(True)
        self.statement.setFont(font)
        self.statement.setAlignment(Qt.AlignVCenter|Qt.AlignLeft)

        self.mainLayout.addWidget(self.statement)

        self.statusRow = QWidget(self.centralwidget)
        self.statusRow.setObjectName(u"statusRow")
        self.statusRow.setMinimumSize(QSize(0, 40))
        self.statusRow.setMaximumSize(QSize(16777215, 40))
        self.statusLayout = QHBoxLayout(self.statusRow)
        self.statusLayout.setObjectName(u"statusLayout")
        self.statusLayout.setContentsMargins(0, 0, 0, 0)
        self.lable_node = QLabel(self.statusRow)
        self.lable_node.setObjectName(u"lable_node")
        font1 = QFont()
        font1.setPointSize(15)
        self.lable_node.setFont(font1)
        self.lable_node.setAlignment(Qt.AlignVCenter|Qt.AlignLeft)

        self.statusLayout.addWidget(self.lable_node)

        self.label_motion = QLabel(self.statusRow)
        self.label_motion.setObjectName(u"label_motion")
        self.label_motion.setFont(font1)
        self.label_motion.setAlignment(Qt.AlignVCenter|Qt.AlignLeft)

        self.statusLayout.addWidget(self.label_motion)

        self.statusLayout.setStretch(0, 1)
        self.statusLayout.setStretch(1, 4)

        self.mainLayout.addWidget(self.statusRow)

        self.contentLayout = QHBoxLayout()
        self.contentLayout.setSpacing(12)
        self.contentLayout.setObjectName(u"contentLayout")
        self.contentLayout.setContentsMargins(0, 0, 0, 0)
        self.leftPane = QWidget(self.centralwidget)
        self.leftPane.setObjectName(u"leftPane")
        self.leftPane.setMinimumSize(QSize(280, 0))
        self.leftPane.setMaximumSize(QSize(280, 16777215))
        self.leftPaneLayout = QVBoxLayout(self.leftPane)
        self.leftPaneLayout.setSpacing(10)
        self.leftPaneLayout.setObjectName(u"leftPaneLayout")
        self.leftPaneLayout.setContentsMargins(0, 0, 0, 0)
        self.lisr_node = QListWidget(self.leftPane)
        self.lisr_node.setObjectName(u"lisr_node")
        self.lisr_node.setMinimumSize(QSize(220, 320))

        self.leftPaneLayout.addWidget(self.lisr_node)

        self.controlButtonsLayout = QVBoxLayout()
        self.controlButtonsLayout.setObjectName(u"controlButtonsLayout")
        self.button_sim_lauch = QPushButton(self.leftPane)
        self.button_sim_lauch.setObjectName(u"button_sim_lauch")
        font2 = QFont()
        font2.setPointSize(14)
        self.button_sim_lauch.setFont(font2)

        self.controlButtonsLayout.addWidget(self.button_sim_lauch)

        self.real_lauch_button = QPushButton(self.leftPane)
        self.real_lauch_button.setObjectName(u"real_lauch_button")
        self.real_lauch_button.setFont(font2)

        self.controlButtonsLayout.addWidget(self.real_lauch_button)


        self.leftPaneLayout.addLayout(self.controlButtonsLayout)


        self.contentLayout.addWidget(self.leftPane)

        self.widget = QWidget(self.centralwidget)
        self.widget.setObjectName(u"widget")
        self.widget.setMinimumSize(QSize(540, 420))

        self.contentLayout.addWidget(self.widget)

        self.contentLayout.setStretch(1, 1)

        self.mainLayout.addLayout(self.contentLayout)

        AUV_UI.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(AUV_UI)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1285, 27))
        self.menu = QMenu(self.menubar)
        self.menu.setObjectName(u"menu")
        AUV_UI.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(AUV_UI)
        self.statusbar.setObjectName(u"statusbar")
        AUV_UI.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menu.menuAction())
        self.menu.addAction(self.action)

        self.retranslateUi(AUV_UI)

        QMetaObject.connectSlotsByName(AUV_UI)
    # setupUi

    def retranslateUi(self, AUV_UI):
        AUV_UI.setWindowTitle(QCoreApplication.translate("AUV_UI", u"MainWindow", None))
        self.action.setText(QCoreApplication.translate("AUV_UI", u"\u8bbe\u7f6e\u4eff\u771f\u542f\u52a8", None))
        self.statement.setText(QCoreApplication.translate("AUV_UI", u"statement", None))
        self.lable_node.setText(QCoreApplication.translate("AUV_UI", u"active nodes:", None))
        self.label_motion.setText(QCoreApplication.translate("AUV_UI", u"TextLabel", None))
        self.button_sim_lauch.setText(QCoreApplication.translate("AUV_UI", u"Lauch in sim envirenment", None))
        self.real_lauch_button.setText(QCoreApplication.translate("AUV_UI", u"Lauch in real envirenment", None))
        self.menu.setTitle(QCoreApplication.translate("AUV_UI", u"\u8bbe\u7f6e", None))
    # retranslateUi

