# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'armlab_gui.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
	_fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
	def _fromUtf8(s):
		return s

try:
	_encoding = QtGui.QApplication.UnicodeUTF8
	def _translate(context, text, disambig):
		return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
	def _translate(context, text, disambig):
		return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
	def setupUi(self, MainWindow):
		MainWindow.setObjectName(_fromUtf8("MainWindow"))
		MainWindow.resize(1230, 900)
		self.centralwidget = QtGui.QWidget(MainWindow)
		self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
		self.kinectFrame = QtGui.QFrame(self.centralwidget)
		self.kinectFrame.setGeometry(QtCore.QRect(240, 40, 640, 480))
		self.kinectFrame.setFrameShape(QtGui.QFrame.StyledPanel)
		self.kinectFrame.setFrameShadow(QtGui.QFrame.Raised)
		self.kinectFrame.setObjectName(_fromUtf8("kinectFrame"))
		self.videoDisplay = QtGui.QLabel(self.kinectFrame)
		self.videoDisplay.setGeometry(QtCore.QRect(0, 0, 640, 480))
		self.videoDisplay.setObjectName(_fromUtf8("videoDisplay"))
		self.OutputFrame = QtGui.QFrame(self.centralwidget)
		self.OutputFrame.setGeometry(QtCore.QRect(10, 40, 221, 311))
		self.OutputFrame.setFrameShape(QtGui.QFrame.StyledPanel)
		self.OutputFrame.setFrameShadow(QtGui.QFrame.Raised)
		self.OutputFrame.setObjectName(_fromUtf8("OutputFrame"))
		self.JointCoordLabel = QtGui.QLabel(self.OutputFrame)
		self.JointCoordLabel.setGeometry(QtCore.QRect(40, 10, 141, 17))
		font = QtGui.QFont()
		font.setBold(True)
		font.setWeight(75)
		self.JointCoordLabel.setFont(font)
		self.JointCoordLabel.setObjectName(_fromUtf8("JointCoordLabel"))
		self.WorldCoordLabel = QtGui.QLabel(self.OutputFrame)
		self.WorldCoordLabel.setGeometry(QtCore.QRect(30, 160, 161, 17)) #(30, 160, 161, 17)
		font = QtGui.QFont()
		font.setBold(True)
		font.setWeight(75)
		self.WorldCoordLabel.setFont(font)
		self.WorldCoordLabel.setObjectName(_fromUtf8("WorldCoordLabel"))
		self.layoutWidget_2 = QtGui.QWidget(self.OutputFrame)
		self.layoutWidget_2.setGeometry(QtCore.QRect(80, 30, 131, 130)) #(80, 30, 131, 120)
		self.layoutWidget_2.setObjectName(_fromUtf8("layoutWidget_2"))
		self.verticalLayout_8 = QtGui.QVBoxLayout(self.layoutWidget_2)
		self.verticalLayout_8.setObjectName(_fromUtf8("verticalLayout_8"))
		self.rdoutBaseJC = QtGui.QLabel(self.layoutWidget_2)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutBaseJC.setFont(font)
		self.rdoutBaseJC.setObjectName(_fromUtf8("rdoutBaseJC"))
		self.verticalLayout_8.addWidget(self.rdoutBaseJC, QtCore.Qt.AlignLeft)
		self.rdoutShoulderJC = QtGui.QLabel(self.layoutWidget_2)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutShoulderJC.setFont(font)
		self.rdoutShoulderJC.setObjectName(_fromUtf8("rdoutShoulderJC"))
		self.verticalLayout_8.addWidget(self.rdoutShoulderJC, QtCore.Qt.AlignLeft)
		self.rdoutElbowJC = QtGui.QLabel(self.layoutWidget_2)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutElbowJC.setFont(font)
		self.rdoutElbowJC.setObjectName(_fromUtf8("rdoutElbowJC"))
		self.verticalLayout_8.addWidget(self.rdoutElbowJC, QtCore.Qt.AlignLeft)
		self.rdoutWristJC = QtGui.QLabel(self.layoutWidget_2)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutWristJC.setFont(font)
		self.rdoutWristJC.setObjectName(_fromUtf8("rdoutWristJC"))
		self.verticalLayout_8.addWidget(self.rdoutWristJC, QtCore.Qt.AlignLeft)
		self.rdoutWrist2JC = QtGui.QLabel(self.layoutWidget_2)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutWrist2JC.setFont(font)
		self.rdoutWrist2JC.setObjectName(_fromUtf8("rdoutWrist2JC"))
		self.verticalLayout_8.addWidget(self.rdoutWrist2JC, QtCore.Qt.AlignLeft)
		self.rdoutWrist3JC = QtGui.QLabel(self.layoutWidget_2)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutWrist3JC.setFont(font)
		self.rdoutWrist3JC.setObjectName(_fromUtf8("rdoutWrist3JC"))
		self.verticalLayout_8.addWidget(self.rdoutWrist3JC, QtCore.Qt.AlignLeft)
		self.layoutWidget_3 = QtGui.QWidget(self.OutputFrame)
		self.layoutWidget_3.setGeometry(QtCore.QRect(10, 30, 66, 130)) #(10, 30, 66, 120)
		self.layoutWidget_3.setObjectName(_fromUtf8("layoutWidget_3"))
		self.verticalLayout_9 = QtGui.QVBoxLayout(self.layoutWidget_3)
		self.verticalLayout_9.setObjectName(_fromUtf8("verticalLayout_9"))
		self.BLabel = QtGui.QLabel(self.layoutWidget_3)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.BLabel.setFont(font)
		self.BLabel.setObjectName(_fromUtf8("BLabel"))
		self.verticalLayout_9.addWidget(self.BLabel, QtCore.Qt.AlignRight)
		self.SLabel = QtGui.QLabel(self.layoutWidget_3)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.SLabel.setFont(font)
		self.SLabel.setObjectName(_fromUtf8("SLabel"))
		self.verticalLayout_9.addWidget(self.SLabel, QtCore.Qt.AlignRight)
		self.ELabel = QtGui.QLabel(self.layoutWidget_3)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.ELabel.setFont(font)
		self.ELabel.setObjectName(_fromUtf8("ELabel"))
		self.verticalLayout_9.addWidget(self.ELabel, QtCore.Qt.AlignRight)
		self.WLabel = QtGui.QLabel(self.layoutWidget_3)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.WLabel.setFont(font)
		self.WLabel.setObjectName(_fromUtf8("WLabel"))
		self.verticalLayout_9.addWidget(self.WLabel, QtCore.Qt.AlignRight)
		self.W2Label = QtGui.QLabel(self.layoutWidget_3)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.W2Label.setFont(font)
		self.W2Label.setObjectName(_fromUtf8("W2Label"))
		self.verticalLayout_9.addWidget(self.W2Label, QtCore.Qt.AlignRight)
		self.W3Label = QtGui.QLabel(self.layoutWidget_3)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.W3Label.setFont(font)
		self.W3Label.setObjectName(_fromUtf8("W3Label"))
		self.verticalLayout_9.addWidget(self.W3Label, QtCore.Qt.AlignRight)
		self.layoutWidget_4 = QtGui.QWidget(self.OutputFrame)
		self.layoutWidget_4.setGeometry(QtCore.QRect(80, 179, 131, 121))
		self.layoutWidget_4.setObjectName(_fromUtf8("layoutWidget_4"))
		self.verticalLayout_12 = QtGui.QVBoxLayout(self.layoutWidget_4)
		self.verticalLayout_12.setObjectName(_fromUtf8("verticalLayout_12"))
		self.rdoutX = QtGui.QLabel(self.layoutWidget_4)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutX.setFont(font)
		self.rdoutX.setObjectName(_fromUtf8("rdoutX"))
		self.verticalLayout_12.addWidget(self.rdoutX, QtCore.Qt.AlignLeft)
		self.rdoutY = QtGui.QLabel(self.layoutWidget_4)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutY.setFont(font)
		self.rdoutY.setObjectName(_fromUtf8("rdoutY"))
		self.verticalLayout_12.addWidget(self.rdoutY, QtCore.Qt.AlignLeft)
		self.rdoutZ = QtGui.QLabel(self.layoutWidget_4)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutZ.setFont(font)
		self.rdoutZ.setObjectName(_fromUtf8("rdoutZ"))
		self.verticalLayout_12.addWidget(self.rdoutZ, QtCore.Qt.AlignLeft)
		self.rdoutT = QtGui.QLabel(self.layoutWidget_4)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutT.setFont(font)
		self.rdoutT.setObjectName(_fromUtf8("rdoutT"))
		self.verticalLayout_12.addWidget(self.rdoutT, QtCore.Qt.AlignLeft)
		self.rdoutG = QtGui.QLabel(self.layoutWidget_4)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutG.setFont(font)
		self.rdoutG.setObjectName(_fromUtf8("rdoutG"))
		self.verticalLayout_12.addWidget(self.rdoutG, QtCore.Qt.AlignLeft)
		self.rdoutP = QtGui.QLabel(self.layoutWidget_4)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.rdoutP.setFont(font)
		self.rdoutP.setObjectName(_fromUtf8("rdoutP"))
		self.verticalLayout_12.addWidget(self.rdoutP, QtCore.Qt.AlignLeft)
		self.layoutWidget_5 = QtGui.QWidget(self.OutputFrame)
		self.layoutWidget_5.setGeometry(QtCore.QRect(11, 180, 66, 121))
		self.layoutWidget_5.setObjectName(_fromUtf8("layoutWidget_5"))
		self.verticalLayout_13 = QtGui.QVBoxLayout(self.layoutWidget_5)
		self.verticalLayout_13.setObjectName(_fromUtf8("verticalLayout_13"))
		self.XLabel = QtGui.QLabel(self.layoutWidget_5)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.XLabel.setFont(font)
		self.XLabel.setObjectName(_fromUtf8("XLabel"))
		self.verticalLayout_13.addWidget(self.XLabel, QtCore.Qt.AlignRight)
		self.YLabel = QtGui.QLabel(self.layoutWidget_5)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.YLabel.setFont(font)
		self.YLabel.setObjectName(_fromUtf8("YLabel"))
		self.verticalLayout_13.addWidget(self.YLabel, QtCore.Qt.AlignRight)
		self.ZLabel = QtGui.QLabel(self.layoutWidget_5)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.ZLabel.setFont(font)
		self.ZLabel.setObjectName(_fromUtf8("ZLabel"))
		self.verticalLayout_13.addWidget(self.ZLabel, QtCore.Qt.AlignRight)
		self.TLabel = QtGui.QLabel(self.layoutWidget_5)
		self.TLabel.setEnabled(True)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.TLabel.setFont(font)
		self.TLabel.setObjectName(_fromUtf8("TLabel"))
		self.verticalLayout_13.addWidget(self.TLabel, QtCore.Qt.AlignRight)
		self.GLabel = QtGui.QLabel(self.layoutWidget_5)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.GLabel.setFont(font)
		self.GLabel.setObjectName(_fromUtf8("GLabel"))
		self.verticalLayout_13.addWidget(self.GLabel, QtCore.Qt.AlignRight)
		self.PLabel = QtGui.QLabel(self.layoutWidget_5)
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(14)
		self.PLabel.setFont(font)
		self.PLabel.setObjectName(_fromUtf8("PLabel"))
		self.verticalLayout_13.addWidget(self.PLabel, QtCore.Qt.AlignRight)
		self.btn_estop = QtGui.QPushButton(self.centralwidget)
		self.btn_estop.setGeometry(QtCore.QRect(900, 40, 311, 71))
		font = QtGui.QFont()
		font.setPointSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.btn_estop.setFont(font)
		self.btn_estop.setObjectName(_fromUtf8("btn_estop"))
		self.btn_exec = QtGui.QPushButton(self.centralwidget)
		self.btn_exec.setGeometry(QtCore.QRect(900, 120, 311, 71))
		font = QtGui.QFont()
		font.setPointSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.btn_exec.setFont(font)
		self.btn_exec.setObjectName(_fromUtf8("btn_exec"))
		self.radioVideo = QtGui.QRadioButton(self.centralwidget)
		self.radioVideo.setGeometry(QtCore.QRect(240, 10, 117, 22))
		self.radioVideo.setChecked(True)
		self.radioVideo.setAutoExclusive(True)
		self.radioVideo.setObjectName(_fromUtf8("radioVideo"))
		self.radioDepth = QtGui.QRadioButton(self.centralwidget)
		self.radioDepth.setGeometry(QtCore.QRect(360, 10, 117, 22))
		self.radioDepth.setObjectName(_fromUtf8("radioDepth"))
		self.radioUsr1 = QtGui.QRadioButton(self.centralwidget)
		self.radioUsr1.setGeometry(QtCore.QRect(480, 10, 117, 22))
		self.radioUsr1.setObjectName(_fromUtf8("radioUsr1"))
		self.radioUsr2 = QtGui.QRadioButton(self.centralwidget)
		self.radioUsr2.setGeometry(QtCore.QRect(610, 10, 117, 22))
		self.radioUsr2.setObjectName(_fromUtf8("radioUsr2"))
		self.radioUsr3 = QtGui.QRadioButton(self.centralwidget)
		self.radioUsr3.setGeometry(QtCore.QRect(710, 10, 117, 22))
		self.radioUsr3.setObjectName(_fromUtf8("radioUsr3"))
		self.radioUsr4 = QtGui.QRadioButton(self.centralwidget)
		self.radioUsr4.setGeometry(QtCore.QRect(810, 10, 117, 22))
		self.radioUsr4.setObjectName(_fromUtf8("radioUsr4"))
		self.SliderFrame = QtGui.QGroupBox(self.centralwidget)
		self.SliderFrame.setGeometry(QtCore.QRect(240, 590, 641, 191))
		self.SliderFrame.setObjectName(_fromUtf8("SliderFrame"))
		self.layoutWidget = QtGui.QWidget(self.SliderFrame)
		self.layoutWidget.setGeometry(QtCore.QRect(0, 20, 65, 150)) #136
		self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
		self.verticalLayout = QtGui.QVBoxLayout(self.layoutWidget)
		self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
		self.BLabelS = QtGui.QLabel(self.layoutWidget)
		self.BLabelS.setObjectName(_fromUtf8("BLabelS"))
		self.verticalLayout.addWidget(self.BLabelS, QtCore.Qt.AlignRight)
		self.SLabelS = QtGui.QLabel(self.layoutWidget)
		self.SLabelS.setObjectName(_fromUtf8("SLabelS"))
		self.verticalLayout.addWidget(self.SLabelS, QtCore.Qt.AlignRight)
		self.ELabelS = QtGui.QLabel(self.layoutWidget)
		self.ELabelS.setObjectName(_fromUtf8("ELabelS"))
		self.verticalLayout.addWidget(self.ELabelS, QtCore.Qt.AlignRight)
		self.WLabelS = QtGui.QLabel(self.layoutWidget)
		self.WLabelS.setObjectName(_fromUtf8("WLabelS"))
		self.verticalLayout.addWidget(self.WLabelS, QtCore.Qt.AlignRight)
		self.W2LabelS = QtGui.QLabel(self.layoutWidget)
		self.W2LabelS.setObjectName(_fromUtf8("W2LabelS"))
		self.verticalLayout.addWidget(self.W2LabelS, QtCore.Qt.AlignRight)
		self.W3LabelS = QtGui.QLabel(self.layoutWidget)
		self.W3LabelS.setObjectName(_fromUtf8("W3LabelS"))
		self.verticalLayout.addWidget(self.W3LabelS, QtCore.Qt.AlignRight)
		self.layoutWidget_6 = QtGui.QWidget(self.SliderFrame)
		self.layoutWidget_6.setGeometry(QtCore.QRect(70, 20, 371, 150)) #70, 20, 371, 136
		self.layoutWidget_6.setObjectName(_fromUtf8("layoutWidget_6"))
		self.verticalLayout_2 = QtGui.QVBoxLayout(self.layoutWidget_6)
		self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
		self.sldrBase = QtGui.QSlider(self.layoutWidget_6)
		self.sldrBase.setMinimum(-179)
		self.sldrBase.setMaximum(180)
		self.sldrBase.setOrientation(QtCore.Qt.Horizontal)
		self.sldrBase.setObjectName(_fromUtf8("sldrBase"))
		self.verticalLayout_2.addWidget(self.sldrBase)
		self.sldrShoulder = QtGui.QSlider(self.layoutWidget_6)
		self.sldrShoulder.setMinimum(-179)
		self.sldrShoulder.setMaximum(180)
		self.sldrShoulder.setOrientation(QtCore.Qt.Horizontal)
		self.sldrShoulder.setObjectName(_fromUtf8("sldrShoulder"))
		self.verticalLayout_2.addWidget(self.sldrShoulder)
		self.sldrElbow = QtGui.QSlider(self.layoutWidget_6)
		self.sldrElbow.setMinimum(-179)
		self.sldrElbow.setMaximum(180)
		self.sldrElbow.setOrientation(QtCore.Qt.Horizontal)
		self.sldrElbow.setObjectName(_fromUtf8("sldrElbow"))
		self.verticalLayout_2.addWidget(self.sldrElbow)
		self.sldrWrist = QtGui.QSlider(self.layoutWidget_6)
		self.sldrWrist.setMinimum(-179)
		self.sldrWrist.setMaximum(180)
		self.sldrWrist.setOrientation(QtCore.Qt.Horizontal)
		self.sldrWrist.setObjectName(_fromUtf8("sldrWrist"))
		self.verticalLayout_2.addWidget(self.sldrWrist)
		self.sldrWrist2 = QtGui.QSlider(self.layoutWidget_6)
		self.sldrWrist2.setMinimum(-179)
		self.sldrWrist2.setMaximum(180)
		self.sldrWrist2.setOrientation(QtCore.Qt.Horizontal)
		self.sldrWrist2.setObjectName(_fromUtf8("sldrWrist2"))
		self.verticalLayout_2.addWidget(self.sldrWrist2)
		self.sldrWrist3 = QtGui.QSlider(self.layoutWidget_6)
		self.sldrWrist3.setMinimum(-179)
		self.sldrWrist3.setMaximum(180)
		self.sldrWrist3.setOrientation(QtCore.Qt.Horizontal)
		self.sldrWrist3.setObjectName(_fromUtf8("sldrWrist3"))
		self.verticalLayout_2.addWidget(self.sldrWrist3)
		self.layoutWidget_7 = QtGui.QWidget(self.SliderFrame)
		self.layoutWidget_7.setGeometry(QtCore.QRect(462, 20, 51, 147)) #(462, 20, 51, 137)
		self.layoutWidget_7.setObjectName(_fromUtf8("layoutWidget_7"))
		self.verticalLayout_6 = QtGui.QVBoxLayout(self.layoutWidget_7)
		self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
		self.rdoutBase = QtGui.QLabel(self.layoutWidget_7)
		self.rdoutBase.setObjectName(_fromUtf8("rdoutBase"))
		self.verticalLayout_6.addWidget(self.rdoutBase)
		self.rdoutShoulder = QtGui.QLabel(self.layoutWidget_7)
		self.rdoutShoulder.setObjectName(_fromUtf8("rdoutShoulder"))
		self.verticalLayout_6.addWidget(self.rdoutShoulder)
		self.rdoutElbow = QtGui.QLabel(self.layoutWidget_7)
		self.rdoutElbow.setObjectName(_fromUtf8("rdoutElbow"))
		self.verticalLayout_6.addWidget(self.rdoutElbow)
		self.rdoutWrist = QtGui.QLabel(self.layoutWidget_7)
		self.rdoutWrist.setObjectName(_fromUtf8("rdoutWrist"))
		self.verticalLayout_6.addWidget(self.rdoutWrist)
		self.rdoutWrist2 = QtGui.QLabel(self.layoutWidget_7)
		self.rdoutWrist2.setObjectName(_fromUtf8("rdoutWrist2"))
		self.verticalLayout_6.addWidget(self.rdoutWrist2)
		self.rdoutWrist3 = QtGui.QLabel(self.layoutWidget_7)
		self.rdoutWrist3.setObjectName(_fromUtf8("rdoutWrist3"))
		self.verticalLayout_6.addWidget(self.rdoutWrist3)
		self.sldrGrip1 = QtGui.QSlider(self.SliderFrame)
		self.sldrGrip1.setGeometry(QtCore.QRect(77, 166, 133, 29)) #77
		self.sldrGrip1.setMinimum(-179)
		self.sldrGrip1.setMaximum(180)
		self.sldrGrip1.setOrientation(QtCore.Qt.Horizontal)
		self.sldrGrip1.setObjectName(_fromUtf8("sldrGrip1"))
		self.G1LableS = QtGui.QLabel(self.SliderFrame)
		self.G1LableS.setGeometry(QtCore.QRect(7, 171, 50, 17)) #24, 171, 41, 17
		self.G1LableS.setObjectName(_fromUtf8("G1LableS"))
		self.rdoutGrip1 = QtGui.QLabel(self.SliderFrame)
		self.rdoutGrip1.setGeometry(QtCore.QRect(215, 166, 49, 28))
		self.rdoutGrip1.setObjectName(_fromUtf8("rdoutGrip1"))
		self.sldrMaxTorque = QtGui.QSlider(self.SliderFrame)
		self.sldrMaxTorque.setGeometry(QtCore.QRect(530, 40, 30, 115))
		self.sldrMaxTorque.setMaximum(100)
		self.sldrMaxTorque.setProperty("value", 50)
		self.sldrMaxTorque.setOrientation(QtCore.Qt.Vertical)
		self.sldrMaxTorque.setObjectName(_fromUtf8("sldrMaxTorque"))
		self.rdoutTorq = QtGui.QLabel(self.SliderFrame)
		self.rdoutTorq.setGeometry(QtCore.QRect(534, 162, 50, 28))
		self.rdoutTorq.setObjectName(_fromUtf8("rdoutTorq"))
		self.sldrSpeed = QtGui.QSlider(self.SliderFrame)
		self.sldrSpeed.setGeometry(QtCore.QRect(600, 40, 29, 115))
		self.sldrSpeed.setMaximum(100)
		self.sldrSpeed.setProperty("value", 25)
		self.sldrSpeed.setSliderPosition(25)
		self.sldrSpeed.setOrientation(QtCore.Qt.Vertical)
		self.sldrSpeed.setObjectName(_fromUtf8("sldrSpeed"))
		self.TqLabel = QtGui.QLabel(self.SliderFrame)
		self.TqLabel.setGeometry(QtCore.QRect(520, 10, 52, 20))
		self.TqLabel.setObjectName(_fromUtf8("TqLabel"))
		self.rdoutSpeed = QtGui.QLabel(self.SliderFrame)
		self.rdoutSpeed.setGeometry(QtCore.QRect(604, 162, 49, 28))
		self.rdoutSpeed.setObjectName(_fromUtf8("rdoutSpeed"))
		self.SpLabel = QtGui.QLabel(self.SliderFrame)
		self.SpLabel.setGeometry(QtCore.QRect(590, 10, 41, 21))
		self.SpLabel.setObjectName(_fromUtf8("SpLabel"))
		self.chk_directcontrol = QtGui.QCheckBox(self.centralwidget)
		self.chk_directcontrol.setGeometry(QtCore.QRect(750, 570, 131, 22))
		self.chk_directcontrol.setChecked(False)
		self.chk_directcontrol.setObjectName(_fromUtf8("chk_directcontrol"))
		self.rdoutMousePixels = QtGui.QLabel(self.centralwidget)
		self.rdoutMousePixels.setGeometry(QtCore.QRect(429, 530, 131, 20))
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(12)
		font.setBold(True)
		font.setWeight(75)
		self.rdoutMousePixels.setFont(font)
		self.rdoutMousePixels.setTextFormat(QtCore.Qt.AutoText)
		self.rdoutMousePixels.setObjectName(_fromUtf8("rdoutMousePixels"))
		self.rdoutMouseWorld = QtGui.QLabel(self.centralwidget)
		self.rdoutMouseWorld.setGeometry(QtCore.QRect(700, 530, 121, 20))
		font = QtGui.QFont()
		font.setFamily(_fromUtf8("Ubuntu Mono"))
		font.setPointSize(12)
		font.setBold(True)
		font.setWeight(75)
		self.rdoutMouseWorld.setFont(font)
		self.rdoutMouseWorld.setTextFormat(QtCore.Qt.AutoText)
		self.rdoutMouseWorld.setObjectName(_fromUtf8("rdoutMouseWorld"))
		self.PixelCoordLabel_2 = QtGui.QLabel(self.centralwidget)
		self.PixelCoordLabel_2.setGeometry(QtCore.QRect(570, 530, 180, 17))
		font = QtGui.QFont()
		font.setBold(True)
		font.setWeight(75)
		self.PixelCoordLabel_2.setFont(font)
		self.PixelCoordLabel_2.setObjectName(_fromUtf8("PixelCoordLabel_2"))
		self.PixelCoordLabel = QtGui.QLabel(self.centralwidget)
		self.PixelCoordLabel.setGeometry(QtCore.QRect(268, 530, 144, 17))
		font = QtGui.QFont()
		font.setBold(True)
		font.setWeight(75)
		self.PixelCoordLabel.setFont(font)
		self.PixelCoordLabel.setObjectName(_fromUtf8("PixelCoordLabel"))
		self.btn_task1 = QtGui.QPushButton(self.centralwidget)
		self.btn_task1.setGeometry(QtCore.QRect(900, 200, 311, 71))
		font = QtGui.QFont()
		font.setPointSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.btn_task1.setFont(font)
		self.btn_task1.setObjectName(_fromUtf8("btn_task1"))
		self.btn_task2 = QtGui.QPushButton(self.centralwidget)
		self.btn_task2.setGeometry(QtCore.QRect(900, 280, 311, 71))
		font = QtGui.QFont()
		font.setPointSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.btn_task2.setFont(font)
		self.btn_task2.setObjectName(_fromUtf8("btn_task2"))
		self.btn_task3 = QtGui.QPushButton(self.centralwidget)
		self.btn_task3.setGeometry(QtCore.QRect(900, 360, 311, 71))
		font = QtGui.QFont()
		font.setPointSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.btn_task3.setFont(font)
		self.btn_task3.setObjectName(_fromUtf8("btn_task3"))
		self.btn_task4 = QtGui.QPushButton(self.centralwidget)
		self.btn_task4.setGeometry(QtCore.QRect(900, 440, 311, 71))
		font = QtGui.QFont()
		font.setPointSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.btn_task4.setFont(font)
		self.btn_task4.setObjectName(_fromUtf8("btn_task4"))
		self.btn_exec_6 = QtGui.QPushButton(self.centralwidget)
		self.btn_exec_6.setGeometry(QtCore.QRect(900, 520, 311, 71))
		font = QtGui.QFont()
		font.setPointSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.btn_exec_6.setFont(font)
		self.btn_exec_6.setObjectName(_fromUtf8("btn_exec_6"))
		self.layoutWidget_8 = QtGui.QWidget(self.centralwidget)
		self.layoutWidget_8.setGeometry(QtCore.QRect(10, 360, 221, 410))
		self.layoutWidget_8.setObjectName(_fromUtf8("layoutWidget_8"))
		self.Group2 = QtGui.QVBoxLayout(self.layoutWidget_8)
		self.Group2.setMargin(10)
		self.Group2.setObjectName(_fromUtf8("Group2"))
		self.btnUser1 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser1.setObjectName(_fromUtf8("btnUser1"))
		self.Group2.addWidget(self.btnUser1)
		self.btnUser2 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser2.setObjectName(_fromUtf8("btnUser2"))
		self.Group2.addWidget(self.btnUser2)
		self.btnUser3 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser3.setObjectName(_fromUtf8("btnUser3"))
		self.Group2.addWidget(self.btnUser3)
		self.btnUser4 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser4.setObjectName(_fromUtf8("btnUser4"))
		self.Group2.addWidget(self.btnUser4)
		self.btnUser5 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser5.setObjectName(_fromUtf8("btnUser5"))
		self.Group2.addWidget(self.btnUser5)
		self.btnUser6 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser6.setObjectName(_fromUtf8("btnUser6"))
		self.Group2.addWidget(self.btnUser6)
		self.btnUser7 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser7.setObjectName(_fromUtf8("btnUser7"))
		self.Group2.addWidget(self.btnUser7)
		self.btnUser8 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser8.setObjectName(_fromUtf8("btnUser8"))
		self.Group2.addWidget(self.btnUser8)
		self.btnUser9 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser9.setObjectName(_fromUtf8("btnUser9"))
		self.Group2.addWidget(self.btnUser9)
		self.btnUser10 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser10.setAutoRepeatDelay(300)
		self.btnUser10.setObjectName(_fromUtf8("btnUser10"))
		self.Group2.addWidget(self.btnUser10)
		self.btnUser11 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser11.setAutoRepeatDelay(300)
		self.btnUser11.setObjectName(_fromUtf8("btnUser11"))
		self.Group2.addWidget(self.btnUser11)
		self.btnUser12 = QtGui.QPushButton(self.layoutWidget_8)
		self.btnUser12.setAutoRepeatDelay(300)
		self.btnUser12.setObjectName(_fromUtf8("btnUser12"))
		self.Group2.addWidget(self.btnUser12)
		self.groupBox = QtGui.QGroupBox(self.centralwidget)
		self.groupBox.setGeometry(QtCore.QRect(10, 820, 1211, 51))
		self.groupBox.setObjectName(_fromUtf8("groupBox"))
		self.rdoutStatus = QtGui.QLabel(self.groupBox)
		self.rdoutStatus.setGeometry(QtCore.QRect(70, 0, 905, 51))
		font = QtGui.QFont()
		font.setPointSize(12)
		self.rdoutStatus.setFont(font)
		self.rdoutStatus.setTextFormat(QtCore.Qt.AutoText)
		self.rdoutStatus.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignTop)
		self.rdoutStatus.setObjectName(_fromUtf8("rdoutStatus"))
		MainWindow.setCentralWidget(self.centralwidget)
		self.statusbar = QtGui.QStatusBar(MainWindow)
		self.statusbar.setObjectName(_fromUtf8("statusbar"))
		MainWindow.setStatusBar(self.statusbar)

		self.retranslateUi(MainWindow)
		QtCore.QMetaObject.connectSlotsByName(MainWindow)

	def retranslateUi(self, MainWindow):
		MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
		self.videoDisplay.setText(_translate("MainWindow", "Video Display", None))
		self.JointCoordLabel.setText(_translate("MainWindow", "Joint Coordinates", None))
		self.WorldCoordLabel.setText(_translate("MainWindow", "End Effector Location", None))
		self.rdoutBaseJC.setText(_translate("MainWindow", "0", None))
		self.rdoutShoulderJC.setText(_translate("MainWindow", "0", None))
		self.rdoutElbowJC.setText(_translate("MainWindow", "0", None))
		self.rdoutWristJC.setText(_translate("MainWindow", "0", None))
		self.rdoutWrist2JC.setText(_translate("MainWindow", "0", None))
		self.rdoutWrist3JC.setText(_translate("MainWindow", "0", None))
		self.BLabel.setText(_translate("MainWindow", "B:", None))
		self.SLabel.setText(_translate("MainWindow", "S:", None))
		self.ELabel.setText(_translate("MainWindow", "E:", None))
		self.WLabel.setText(_translate("MainWindow", "W1:", None))
		self.W2Label.setText(_translate("MainWindow", "W2:", None))
		self.W3Label.setText(_translate("MainWindow", "W3:", None))
		self.rdoutX.setText(_translate("MainWindow", "0", None))
		self.rdoutY.setText(_translate("MainWindow", "0", None))
		self.rdoutZ.setText(_translate("MainWindow", "0", None))
		self.rdoutT.setText(_translate("MainWindow", "0", None))
		self.rdoutG.setText(_translate("MainWindow", "0", None))
		self.rdoutP.setText(_translate("MainWindow", "0", None))
		self.XLabel.setText(_translate("MainWindow", "X   :", None))
		self.YLabel.setText(_translate("MainWindow", "Y   :", None))
		self.ZLabel.setText(_translate("MainWindow", "Z   :", None))
		self.TLabel.setText(_translate("MainWindow", "Phi :", None))
		self.GLabel.setText(_translate("MainWindow", "Beta:", None))
		self.PLabel.setText(_translate("MainWindow", "Psi :", None))
		self.btn_estop.setText(_translate("MainWindow", "EMERGENCY STOP", None))
		self.btn_exec.setText(_translate("MainWindow", "EXECUTE PLAN", None))
		self.radioVideo.setText(_translate("MainWindow", "Video", None))
		self.radioDepth.setText(_translate("MainWindow", "Depth", None))
		self.radioUsr1.setText(_translate("MainWindow", "User 1", None))
		self.radioUsr2.setText(_translate("MainWindow", "User 2", None))
		self.radioUsr3.setText(_translate("MainWindow", "User 3", None))
		self.radioUsr4.setText(_translate("MainWindow", "User 4", None))
		self.SliderFrame.setTitle(_translate("MainWindow", "Joint Sliders", None))
		self.BLabelS.setText(_translate("MainWindow", "Base", None))
		self.SLabelS.setText(_translate("MainWindow", "Shoulder", None))
		self.ELabelS.setText(_translate("MainWindow", "Elbow", None))
		self.WLabelS.setText(_translate("MainWindow", "Wrist1", None))
		self.W2LabelS.setText(_translate("MainWindow", "Wrist2", None))
		self.W3LabelS.setText(_translate("MainWindow", "Wrist3", None))
		self.rdoutBase.setText(_translate("MainWindow", "0", None))
		self.rdoutShoulder.setText(_translate("MainWindow", "0", None))
		self.rdoutElbow.setText(_translate("MainWindow", "0", None))
		self.rdoutWrist.setText(_translate("MainWindow", "0", None))
		self.rdoutWrist2.setText(_translate("MainWindow", "0", None))
		self.rdoutWrist3.setText(_translate("MainWindow", "0", None))
		self.G1LableS.setText(_translate("MainWindow", "Gripper", None))
		self.rdoutGrip1.setText(_translate("MainWindow", "0", None))
		# self.rdoutGrip2.setText(_translate("MainWindow", "0", None))
		self.rdoutTorq.setText(_translate("MainWindow", "0", None))
		self.TqLabel.setText(_translate("MainWindow", "Torque", None))
		self.rdoutSpeed.setText(_translate("MainWindow", "0", None))
		self.SpLabel.setText(_translate("MainWindow", "Speed", None))
		self.chk_directcontrol.setText(_translate("MainWindow", "Direct Control", None))
		self.rdoutMousePixels.setText(_translate("MainWindow", "(U,V,D)", None))
		self.rdoutMouseWorld.setText(_translate("MainWindow", "(X,Y,Z)", None))
		self.PixelCoordLabel_2.setText(_translate("MainWindow", "World Coord [mm]:", None))
		self.PixelCoordLabel.setText(_translate("MainWindow", "Mouse Coordinates:", None))
		self.btn_task1.setText(_translate("MainWindow", "RECORD WAYPOINT", None))
		self.btn_task2.setText(_translate("MainWindow", "CLEAR WAYPOINTS", None))
		self.btn_task3.setText(_translate("MainWindow", "TOGGLE GRIPPER", None))
		self.btn_task4.setText(_translate("MainWindow", "TASK 4", None))
		self.btn_exec_6.setText(_translate("MainWindow", "TASK 5", None))
		self.btnUser1.setText(_translate("MainWindow", "USER 1", None))
		self.btnUser2.setText(_translate("MainWindow", "USER 2", None))
		self.btnUser3.setText(_translate("MainWindow", "USER 3", None))
		self.btnUser4.setText(_translate("MainWindow", "USER 4", None))
		self.btnUser5.setText(_translate("MainWindow", "USER 5", None))
		self.btnUser6.setText(_translate("MainWindow", "USER 6", None))
		self.btnUser7.setText(_translate("MainWindow", "USER 7", None))
		self.btnUser8.setText(_translate("MainWindow", "USER 8", None))
		self.btnUser9.setText(_translate("MainWindow", "USER 9", None))
		self.btnUser10.setText(_translate("MainWindow", "USER 10", None))
		self.btnUser11.setText(_translate("MainWindow", "USER 11", None))
		self.btnUser12.setText(_translate("MainWindow", "USER 12", None))
		self.groupBox.setTitle(_translate("MainWindow", "STATUS: ", None))
		self.rdoutStatus.setText(_translate("MainWindow", "Waiting for Inputs", None))

