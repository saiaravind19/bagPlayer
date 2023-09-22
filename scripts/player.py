import sys
import os 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import time
from ros2 import bagfile_helper

class Qtapplication(QMainWindow):
	def __init__(self,recorder_obj):
		super().__init__()
		self.bag_player = recorder_obj
		self.currentPlaylist = ''
		self.userAction = -1
		self.statusBar().showMessage('Load ROS bags to play')
		self.homeScreen()
		self.timer = QTimer(self)
		self.timer.timeout.connect(self.updateslider)	
		self.slider_position = 0
		self.seek_requested = False
			
	def homeScreen(self):
		#Set title of the MainWindow
		self.setWindowTitle('ROS bag player')
		
		#Create Menubar
		self.createMenubar()
				
		#Add Control Bar
		controlBar = self.addControls()
		
		#need to add both infoscreen and control bar to the central widget.
		centralWidget = QWidget()
		centralWidget.setLayout(controlBar)
		self.setCentralWidget(centralWidget)
		
		#Set Dimensions of the MainWindow
		self.resize(400,100)
		
		#show everything.
		self.show()
		
	def createMenubar(self):
		menubar = self.menuBar()
		filemenu = menubar.addMenu('File')
		filemenu.addAction(self.fileOpen())
		filemenu.addAction(self.exitAction())

	def addControls(self):
		controlArea = QVBoxLayout()		#centralWidget
		seekSliderLayout = QHBoxLayout()
		controls = QHBoxLayout()
		playlistCtrlLayout = QHBoxLayout()
		
		#creating buttons
		self.playBtn = QPushButton('Play')		#play button
		self.pauseBtn = QPushButton('Pause')		#pause button
		self.stopBtn = QPushButton('Stop')		#stop button
		

		#creating playlist controls
		self.plynxtBtn = QPushButton('Play Next Msg')
	
		self.pauseBtn.setEnabled(False)
		self.stopBtn.setEnabled(False)
		self.plynxtBtn.setEnabled(False)
		#creating seek slider
		self.seekSlider = QSlider()
		self.seekSlider.setMinimum(0)
		self.seekSlider.setMaximum(100)
		self.seekSlider.setOrientation(Qt.Horizontal)
		self.seekSlider.setTracking(False)
		self.seekSlider.valueChanged.connect(self.seekPositionValueUpdate)
		self.seekSlider.sliderReleased.connect(self.seekPosition)
		
		seekSliderLabel1 = QLabel('0.00')
		seekSliderLabel2 = QLabel('0.00')
		seekSliderLayout.addWidget(seekSliderLabel1)
		seekSliderLayout.addWidget(self.seekSlider)
		seekSliderLayout.addWidget(seekSliderLabel2)
		
		#Add handler for each button. Not using the default slots.
		self.playBtn.clicked.connect(self.playHandler)
		self.pauseBtn.clicked.connect(self.pauseHandler)
		self.stopBtn.clicked.connect(self.stopHandler)
		self.plynxtBtn.clicked.connect(self.playnextmsg)
		
		#Adding to the horizontal layout
		controls.addWidget(self.playBtn)
		controls.addWidget(self.pauseBtn)
		controls.addWidget(self.stopBtn)
		
		playlistCtrlLayout.addWidget(self.plynxtBtn)
		
		#Adding to the vertical layout
		controlArea.addLayout(seekSliderLayout)
		controlArea.addLayout(controls)
		controlArea.addLayout(playlistCtrlLayout)
		return controlArea
	
	def playHandler(self):
		if self.userAction != -1:
			if self.userAction == 0: 
				self.statusBar().showMessage('Playing bag file')
				print("[player][playHandler]",self.currentPlaylist)
				self.userAction = 1
				
				self.seekSlider.setValue(0)			
				self.pauseBtn.setEnabled(True)
				self.stopBtn.setEnabled(True)
				self.plynxtBtn.setEnabled(True)
				self.playBtn.setText("Resume")
				#play the recording and start the timer
				self.bag_player.play_recording(self.currentPlaylist)
				self.timer.start(1000)

				#self.player.setPlaylist(self.currentPlaylist)
			elif self.userAction == 2:
				self.userAction = 1
				self.timer.start()
				self.statusBar().showMessage('Resuming playback')
				self.bag_player.resume_playback(self.currentPlaylist,self.slider_position)
			elif self.userAction == 1:
				self.statusBar().showMessage('Pause the ros bag file')
		else :
			self.statusBar().showMessage('Please select a bag file to play')
	
	def updateslider(self):
		try:
			if int(self.duration) > self.slider_position:
				self.seekSlider.setValue(self.slider_position+1)
			elif int(self.duration) == self.slider_position:
				time.sleep(0.2)
				self.stopHandler()
		except Exception as e:
			print("[player][updateslider]Exception in updateslider"+str(e))


	def pauseHandler(self):
		if self.userAction != -1:
			if self.userAction != 2:
				self.userAction = 2
				self.bag_player.pause_playback()
				self.timer.stop()
				self.statusBar().showMessage('Pausing ROS bag player')
		else:
			self.statusBar().showMessage('ROS bag player is not active')

			
	def stopHandler(self):
		if self.userAction != -1:
			self.userAction = -1
			self.bag_player.stop_player()
			self.timer.stop()
			
			self.playBtn.setText("Play")
			self.pauseBtn.setEnabled(False)
			self.stopBtn.setEnabled(False)
			self.plynxtBtn.setEnabled(False)
			self.statusBar().showMessage('Stopping rosbag player')			
		else :
			self.statusBar().showMessage('ROS bag player is not active')

	def playnextmsg(self):
		try:
			if self.userAction == 2:
				self.bag_player.playnextmsg()	
			else :
				self.statusBar().showMessage('Pause the bag player to play next msg')

		except Exception as e:
			print("[player][plynxtbtn]Exception"+str(e))

	def seekPosition(self):
		self.seek_requested = True


	def seekPositionValueUpdate(self):
		self.slider_position = self.seekSlider.value()

		if self.userAction != -1 and self.userAction != 0 and self.seek_requested == True:
			if self.userAction == 2:
				print("[player][seekPositionValueUpdate] Seeking player with initially paused state",self.slider_position)
				self.bag_player.seek_playback(self.slider_position,self.currentPlaylist,paused = True)
			else :
				print("[player][seekPositionValueUpdate] Seeking player in running state",self.slider_position)
				self.bag_player.seek_playback(self.slider_position,self.currentPlaylist)

			self.seek_requested = False
		else :
			#print("[player][seekPositionValueUpdate]Neglecting seek position")
			self.seek_requested = False

	def fileOpen(self):
		fileAc = QAction('Open File',self)
		fileAc.setShortcut('Ctrl+O')
		fileAc.setStatusTip('Open File')
		fileAc.triggered.connect(self.addFiles)
		return fileAc
	
	def addFiles(self):
		fileChoosen = QFileDialog.getOpenFileName(self,'Choose a bag file play', os.path.expanduser('~'),"Bag (*.db3)")
		
		if fileChoosen[0] != '':
			print("[player][addFiles]File choosen:",fileChoosen)
			self.currentPlaylist = fileChoosen[0]
			self.duration,epochTime = self.bag_player.get_time_duration(fileChoosen)
			self.seekSlider.setValue(0)
			self.centralWidget().layout().itemAt(0).layout().itemAt(1).widget().setRange(0,int(self.duration))
			self.centralWidget().layout().itemAt(0).layout().itemAt(2).widget().setText('%d.%02d'%(int(self.duration),int((self.duration - int(self.duration))*100)))
			self.userAction = 0

		else:
			print("[player][addFiles]File not choosen")
			
	
	def exitAction(self):
		exitAc = QAction('&Exit',self)
		exitAc.setShortcut('Ctrl+Q')
		exitAc.setStatusTip('Exit App')
		exitAc.triggered.connect(self.closeEvent)
		return exitAc
	
	def closeEvent(self,event):
		reply = QMessageBox.question(self,'Message','Pres Yes to Close.',QMessageBox.Yes|QMessageBox.No,QMessageBox.Yes)
		
		if reply == QMessageBox.Yes :
			qApp.quit()
		else :
			try:
				event.ignore()
			except AttributeError:
				pass
			
def runapp(recorder_obj = None):
	app = QApplication(sys.argv)
	recorder_obj = bagfile_helper()
	print(recorder_obj)
	ex = Qtapplication(recorder_obj)
	sys.exit(app.exec_())
	

if __name__ == '__main__':	
	app = runapp()

# -1 -> bag player is not active
#  0 -> bag file is selected and ready to start playing
#  1 -> bag file is playing
#  2 -> Paused the bag file userAction