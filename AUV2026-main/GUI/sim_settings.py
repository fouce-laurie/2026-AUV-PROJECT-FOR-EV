# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'sim_settings.ui'
##
## Created by: Qt User Interface Compiler version 6.10.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractButton, QApplication, QCheckBox, QComboBox,
    QDialog, QDialogButtonBox, QFormLayout, QLabel,
    QSizePolicy, QSpacerItem, QVBoxLayout, QWidget)

class Ui_SimSettingsDialog(object):
    def setupUi(self, SimSettingsDialog):
        if not SimSettingsDialog.objectName():
            SimSettingsDialog.setObjectName(u"SimSettingsDialog")
        SimSettingsDialog.resize(400, 250)
        self.verticalLayout = QVBoxLayout(SimSettingsDialog)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.formLayout = QFormLayout()
        self.formLayout.setObjectName(u"formLayout")
        self.label = QLabel(SimSettingsDialog)
        self.label.setObjectName(u"label")

        self.formLayout.setWidget(0, QFormLayout.ItemRole.LabelRole, self.label)

        self.combo_scenario = QComboBox(SimSettingsDialog)
        self.combo_scenario.addItem("")
        self.combo_scenario.addItem("")
        self.combo_scenario.addItem("")
        self.combo_scenario.setObjectName(u"combo_scenario")

        self.formLayout.setWidget(0, QFormLayout.ItemRole.FieldRole, self.combo_scenario)

        self.label_2 = QLabel(SimSettingsDialog)
        self.label_2.setObjectName(u"label_2")

        self.formLayout.setWidget(1, QFormLayout.ItemRole.LabelRole, self.label_2)

        self.check_enable_ai = QCheckBox(SimSettingsDialog)
        self.check_enable_ai.setObjectName(u"check_enable_ai")

        self.formLayout.setWidget(1, QFormLayout.ItemRole.FieldRole, self.check_enable_ai)

        self.label_3 = QLabel(SimSettingsDialog)
        self.label_3.setObjectName(u"label_3")

        self.formLayout.setWidget(2, QFormLayout.ItemRole.LabelRole, self.label_3)

        self.check_enable_depth = QCheckBox(SimSettingsDialog)
        self.check_enable_depth.setObjectName(u"check_enable_depth")

        self.formLayout.setWidget(2, QFormLayout.ItemRole.FieldRole, self.check_enable_depth)

        self.verticalLayout.addLayout(self.formLayout)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)

        self.buttonBox = QDialogButtonBox(SimSettingsDialog)
        self.buttonBox.setObjectName(u"buttonBox")
        self.buttonBox.setOrientation(Qt.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)

        self.verticalLayout.addWidget(self.buttonBox)


        self.retranslateUi(SimSettingsDialog)
        self.buttonBox.accepted.connect(SimSettingsDialog.accept)
        self.buttonBox.rejected.connect(SimSettingsDialog.reject)

        QMetaObject.connectSlotsByName(SimSettingsDialog)
    # setupUi

    def retranslateUi(self, SimSettingsDialog):
        SimSettingsDialog.setWindowTitle(QCoreApplication.translate("SimSettingsDialog", u"\u4eff\u771f\u8bbe\u7f6e", None))
        self.label.setText(QCoreApplication.translate("SimSettingsDialog", u"\u4eff\u771f\u573a\u666f:", None))
        self.combo_scenario.setItemText(0, QCoreApplication.translate("SimSettingsDialog", u"test", None))
        self.combo_scenario.setItemText(1, QCoreApplication.translate("SimSettingsDialog", u"qualification", None))
        self.combo_scenario.setItemText(2, QCoreApplication.translate("SimSettingsDialog", u"finals", None))

        self.label_2.setText(QCoreApplication.translate("SimSettingsDialog", u"\u542f\u7528 AI \u63a7\u5236:", None))
        self.check_enable_ai.setText("")
        self.label_3.setText(QCoreApplication.translate("SimSettingsDialog", u"\u542f\u7528\u6df1\u5ea6\u5bfc\u51fa/\u70b9\u4ea7\u8bb0\u5f55:", None))
        self.check_enable_depth.setText("")
    # retranslateUi

