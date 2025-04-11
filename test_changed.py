from PyQt5.QtWidgets import *
from PyQt5 import QtGui
from PyQt5.QtCore import *
import os, sys, time, cv2, math, subprocess as sp
import army
import random
import wave, pyaudio
from gtts import gTTS as gtts
from threading import Thread, Lock
from popAssist import *
from Jukebox import *
from snowboy import snowboydecoder, snowboydetect

p = pyaudio.PyAudio()

lock = Lock()
ctrl_right = -1
euc_corrector=0
hello_time_log=0

interrupted = False

FNULL = open(os.devnull, 'w')

GREETING_PATH = "./.greeting/"
GREETING_DICT = {"Hello":"안녕하세요", "Good":"반갑습니다"}

class Window(QWidget, army.Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.showFullScreen()
        self.jukebox.setHidden(True)
        self.objflw.setHidden(True)
        self.gass.setHidden(True)
        self.juke_play.setFlat(True)
        self.juke_prev.setFlat(True)
        self.juke_next.setFlat(True)
        self.juke_play.setText("")
        self.juke_prev.setText("")
        self.juke_next.setText("")
        self.to_juke.setText("")
        self.to_objflw.setText("")
        self.to_gass.setText("")
        self.juke_external.setText("")

        self.to_juke.clicked.connect(self.__to_jukebox)
        self.to_objflw.clicked.connect(self.__to_objflw)
        self.to_gass.clicked.connect(self.__to_gass)
        self.objflw_sw.clicked.connect(self.__obj_move_sw)
        self.juke_play.clicked.connect(self.__play_and_stop)
        self.juke_prev.clicked.connect(self.__playPrev)
        self.juke_next.clicked.connect(self.__playNext)
        self.juke_volume.valueChanged.connect(self.__set_volume)
        self.juke_volume.sliderMoved.connect(self.__set_volume)
        self.juke_external.clicked.connect(self.__juke_ext)
        self.juke_external.pressed.connect(self.__juke_ext_pressed)

        self.progress_timer = QTimer(self)
        self.progress_timer.setInterval(500)
        self.progress_timer.timeout.connect(self.__qt_timer_callback)

        self.objsw=False
        self.loadsw=False
        self.playing=False

        self.gass_text.setAlignment(Qt.AlignCenter)

    def load(self):
        global Pilot, cam, bot, OF, ofth, window
        from pop import Pilot

        pixmap = QtGui.QPixmap("logo2.png")
        jukeimg = QtGui.QPixmap("jukebox.png")
        forward_icon = QtGui.QIcon('forward.png')
        backward_icon = QtGui.QIcon('backward.png')

        self.play_icon = QtGui.QIcon("play.png")
        self.pause_icon = QtGui.QIcon("pause.png")

        self.juke_on_icon = QtGui.QIcon("juke_on.png")
        self.juke_off_icon = QtGui.QIcon("juke_off.png")
        
        self.va_on_icon = QtGui.QIcon("va_on.png")
        self.va_off_icon = QtGui.QIcon("va_off.png")

        self.od_on_icon = QtGui.QIcon("od_on.png")
        self.od_off_icon = QtGui.QIcon("od_off.png")
        
        self.gass_on_img = QtGui.QPixmap("gass_on.png")
        self.gass_off_img = QtGui.QPixmap("gass_off.png")

        self.move_run_icon = QtGui.QIcon("run.png")
        self.move_stop_icon = QtGui.QIcon("stop.png")

        self.ext_ok_icon = QtGui.QIcon("usb_on.png")
        self.ext_fail_icon = QtGui.QIcon("usb_off2.png")
        self.ext_icon = QtGui.QIcon("usb_norm.png")
        self.ext_notfound_icon = QtGui.QIcon("usb_not_found.png")

        cam=Pilot.Camera(width=640, height=480)
        bot=Pilot.SerBot()

        OF=Pilot.Object_Follow(cam)
        OF.load_model()

        self.logo.setPixmap(pixmap)
        self.juke_img.setPixmap(jukeimg)
        self.juke_next.setIcon(forward_icon)
        self.juke_prev.setIcon(backward_icon)
        self.juke_play.setIcon(self.play_icon)
        self.to_juke.setIcon(self.juke_off_icon)
        self.to_objflw.setIcon(self.od_off_icon)
        self.to_gass.setIcon(self.va_off_icon)
        self.objflw_sw.setIcon(self.play_icon)
        self.juke_external.setIcon(self.ext_icon)
        
        self.loadsw=True

        def _OF_Thread():
            log=[]
            while True:
                global hello_time_log
                try:
                    v=OF.detect(index='person')
                    
                    if v is not None: 
                        if len(log)==0:
                            if time.time()-hello_time_log>2:
                                hello_time_log=time.time()
                                #sayHello()
                                log.append({'data':v, 'timestamp':time.time()})
                        else:
                            eucsw=True
                            for i in range(len(log)):
                                l=log[i]
                                euc=1/(math.sqrt((v['x']-l['data']['x'])**2+(v['y']-l['data']['y'])**2+(v['size_rate']-l['data']['size_rate'])**2)+1)
                                if euc>=0.88+euc_corrector and time.time()-l['timestamp']<=5.:
                                    log[i]={'data':v, 'timestamp':time.time()}
                                    eucsw=False
                                    break
                            
                            if eucsw:
                                if time.time()-hello_time_log>2:
                                    hello_time_log=time.time()
                                    #sayHello()
                                    log.append({'data':v, 'timestamp':time.time()})

                    img=OF.value
                    img=cv2.resize(img,(867,650))
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
                    h,w,c = img.shape
                    qImg = QtGui.QImage(img.data, w, h, w*c, QtGui.QImage.Format_RGB888)
                    pixmap = QtGui.QPixmap.fromImage(qImg)
                    window.objflw_img.setPixmap(pixmap)
                    #window.objflw_img.resize(pixmap.width(), pixmap.height())
                    lock.acquire()

                    if ctrl_right == 1:
                        if v is not None:
                            steer=v['x']*4

                            if steer > 1:
                                steer=1
                            elif steer < -1:
                                steer=-1

                            bot.steering=steer

                            if v['size_rate']<0.20:
                                if window.objsw:
                                    bot.forward(90)
                                else:
                                    bot.stop()
                            else:
                                if window.objsw:
                                    if bot.steering<-0.5 :
                                        bot.setSpeed(40)
                                        time.sleep(0.1)
                                        bot.turnLeft()
                                    elif bot.steering>0.5:
                                        bot.setSpeed(40)
                                        time.sleep(0.1)
                                        bot.turnRight()
                                    else:
                                        bot.stop()
                                else:
                                    bot.stop()
                        else:
                            bot.stop()  

                    lock.release()
                except Exception as e:
                    print(e)
                    if lock.locked():
                        lock.release()
                    pass

        ofth=Thread(target=_OF_Thread, daemon=True)
        ofth.start()

        gassth=Gass_worker()
        gassth.start()

        gassth.settext.connect(self.set_gass_text)
        gassth.setimg.connect(self.set_gass_img)

        gassth.setplaypause.connect(self.set_play_pause)
        gassth.setplayprev.connect(self.set_play_prev)
        gassth.setplaynext.connect(self.set_play_next)

    def __to_jukebox(self, checked):
        if not self.loadsw:
            self.jukebox.setHidden(True)
            self.objflw.setHidden(True)
            self.gass.setHidden(True)
            self.menu.setHidden(False)
            self.to_juke.setChecked(False)
            self.to_objflw.setChecked(False)
            self.to_gass.setChecked(False)
        else:
            if checked:
                self.objflw.setHidden(True)
                self.gass.setHidden(True)
                self.menu.setHidden(True)
                self.to_objflw.setChecked(False)
                self.to_gass.setChecked(False)
                self.to_juke.setIcon(self.juke_on_icon)
                self.to_objflw.setIcon(self.od_off_icon)
                self.to_gass.setIcon(self.va_off_icon)
                lock.acquire()
                global ctrl_right
                ctrl_right=0
                lock.release()
                time.sleep(0.1)
                self.jukebox.setHidden(False)
            else:
                self.jukebox.setHidden(True)
                self.to_juke.setIcon(self.juke_off_icon)

                if not self.to_juke.isChecked() and not self.to_objflw.isChecked() and not self.to_gass.isChecked(): 
                    time.sleep(0.1)
                    self.menu.setHidden(False)

    def __to_objflw(self, checked):
        if not self.loadsw:
            self.jukebox.setHidden(True)
            self.objflw.setHidden(True)
            self.gass.setHidden(True)
            self.menu.setHidden(False)
            self.to_juke.setChecked(False)
            self.to_objflw.setChecked(False)
            self.to_gass.setChecked(False)
        else:
            if checked:
                self.jukebox.setHidden(True)
                self.gass.setHidden(True)
                self.menu.setHidden(True)
                self.to_juke.setChecked(False)
                self.to_gass.setChecked(False)
                self.to_juke.setIcon(self.juke_off_icon)
                self.to_objflw.setIcon(self.od_on_icon)
                self.to_gass.setIcon(self.va_off_icon)
                lock.acquire()
                global ctrl_right
                ctrl_right=1
                lock.release()
                time.sleep(0.1)
                self.objflw.setHidden(False)
            else:
                self.objflw.setHidden(True)
                self.to_objflw.setIcon(self.od_off_icon)
                
                if not self.to_juke.isChecked() and not self.to_objflw.isChecked() and not self.to_gass.isChecked(): 
                    time.sleep(0.1)
                    self.menu.setHidden(False)
    
    def __to_gass(self, checked):
        if not self.loadsw:
            self.jukebox.setHidden(True)
            self.objflw.setHidden(True)
            self.gass.setHidden(True)
            self.menu.setHidden(False)
            self.to_juke.setChecked(False)
            self.to_objflw.setChecked(False)
            self.to_gass.setChecked(False)
        else:
            if checked:
                self.jukebox.setHidden(True)
                self.objflw.setHidden(True)
                self.menu.setHidden(True)
                self.to_juke.setChecked(False)
                self.to_objflw.setChecked(False)
                self.to_juke.setIcon(self.juke_off_icon)
                self.to_objflw.setIcon(self.od_off_icon)
                self.to_gass.setIcon(self.va_on_icon)
                lock.acquire()
                global ctrl_right
                ctrl_right=2
                lock.release()
                time.sleep(0.1)
                self.gass.setHidden(False)
            else:
                self.gass.setHidden(True)
                self.to_gass.setIcon(self.va_off_icon)
                
                if not self.to_juke.isChecked() and not self.to_objflw.isChecked() and not self.to_gass.isChecked(): 
                    time.sleep(0.1)
                    self.menu.setHidden(False)

    def __obj_move_sw(self, checked):
        if checked: 
            self.objflw_sw.setIcon(self.pause_icon)
            self.objsw=True
        else:
            self.objflw_sw.setIcon(self.play_icon)
            self.objsw=False

    def __juke_umount(self):
        jukebox.umountUsb()

        if self.playing:
            jukebox.play()
            self.juke_play.setIcon(self.pause_icon)
            self.juke_play.setChecked(True)
        else:
            self.juke_play.setIcon(self.play_icon)
            self.juke_play.setChecked(False)

        self.juke_external.setChecked(False)
        self.juke_external.setIcon(self.ext_icon)
        self.juke_title.setText(jukebox.musicName)

    def __juke_ext(self, checked):
        if checked:
            if sp.call(['ls /dev/sd*'], shell=True, stdout=FNULL):
                self.juke_external.setIcon(self.ext_notfound_icon)
                self.juke_external.setChecked(False)
            else:
                jukebox.mountUsb()
                if not jukebox.isUsbVailed:
                    self.juke_external.setIcon(self.ext_fail_icon)
                    self.juke_external.setChecked(False)
                else:
                    self.juke_external.setIcon(self.ext_ok_icon)
        else:
            self.__juke_umount()

    def __juke_ext_pressed(self):
        self.juke_external.setIcon(self.ext_icon)

    def __set_volume(self, amount):
        jukebox.volume=amount

    def __play_and_stop(self, checked):
        if checked:
            self.juke_play.setIcon(self.pause_icon)
            self.juke_play.setIconSize(QSize(100, 100))

            jukebox.play()
            self.progress_timer.start()
            self.playing=True
        else:
            self.juke_play.setIcon(self.play_icon)
            self.juke_play.setIconSize(QSize(100, 100))

            jukebox.pause()
            self.progress_timer.stop()
            self.playing=False
        
        self.juke_title.setText(jukebox.musicName)

    def __playNext(self):
        jukebox.playNext()
        self.juke_title.setText(jukebox.musicName)
        self.progressBar.reset()

        if not self.playing:
            jukebox.pause()

    def __playPrev(self):
        jukebox.playPrev()
        self.juke_title.setText(jukebox.musicName)
        self.progressBar.reset()

        if not self.playing:
            jukebox.pause()

    def __qt_timer_callback(self):
        self.__set_progress()
        self.__check_usb()

    def __set_progress(self):
        progress=jukebox.progress
        self.progressBar.setValue(progress)
        if jukebox.isCompleted:
            self.__playNext()

    def __check_usb(self):
        if jukebox.isUsbConnected:
            if sp.call(['ls /dev/sd*'], shell=True, stdout=FNULL):
                self.__juke_umount()

    @pyqtSlot(str)
    def set_gass_text(self, text):
        self.gass_text.setText(text)

    @pyqtSlot(bool)
    def set_gass_img(self, sw):
        if sw:
            self.gass_img.setPixmap(self.gass_on_img)
        else:
            self.gass_img.setPixmap(self.gass_off_img)

    @pyqtSlot(bool)
    def set_play_pause(self,sw):
        self.__play_and_stop(sw)
        self.juke_play.toggle()

    @pyqtSlot()
    def set_play_prev(self):
        self.__playPrev()
    
    @pyqtSlot()
    def set_play_next(self):
        self.__playNext()

class Gass_worker(QThread):

    settext = pyqtSignal(str)
    setimg = pyqtSignal(bool)

    setplaypause = pyqtSignal(bool)
    setplayprev = pyqtSignal()
    setplaynext = pyqtSignal()

    detector = snowboydecoder.HotwordDetector("snowboy.pmdl", sensitivity=0.51)

    def run(self):
        def userAction(text): 
            action = False

            self.setimg.emit(False)
            self.settext.emit(text)

            lock.acquire()

            r = max(text.find("노래"), text.find("음악"), text.find("곡"))

            if max(text.find("다음"), text.find("앞")) < r:
                self.setplaynext.emit()
                action = True
            elif max(text.find("이전"), text.find("뒤")) < r:
                self.setplayprev.emit()
                action = True
            if max(text.find("재생", r), text.find("틀어", r)) != -1:
                self.setplaypause.emit(True)
                action = True
            elif max(text.find("멈춰", r), text.find("중지", r)) != -1:
                self.setplaypause.emit(False)
                action = True

            lock.release()

            return action

        def onStart():
            self.setimg.emit(True)
            snowboydecoder.play_audio_file()
            print(">>> Start recording....")

        def callAssist():
            ga.assist(onStart)

        def isInterrupted():
            return interrupted

        stream = create_conversation_stream()
        stream.volume_percentage = 100
        ga = GAssistant(stream, local_device_handler=userAction)

        self.detector.start(detected_callback=callAssist, interrupt_check=isInterrupted)
        self.detector.terminate()

def sayHello():
    if not os.path.isdir(GREETING_PATH):
        os.mkdir(GREETING_PATH)

    filename = ""
    number = random.randint(0, len(GREETING_DICT) - 1)

    for file, word in GREETING_DICT.items():
        fileName = GREETING_PATH + file + ".wav"
        temp_fileName = GREETING_PATH + file + ".mp3"

        if not os.path.isfile(fileName):
            tts = gtts(word, lang='ko')
            tts.save(temp_fileName)

            subprocess.call(["ffmpeg", "-y", "-i", temp_fileName, "-acodec", "pcm_u8", "-ar", "48000", fileName], stdout=FNULL)

        if number == 0:
            filename = fileName
            break

        number -= 1

    def playThread():
        if window.playing: jukebox.pause()

        w = wave.open(filename)
        s = p.open(format=p.get_format_from_width(w.getsampwidth()),
                          channels=w.getnchannels(),
                          rate=w.getframerate(),
                          output=True)

        data = w.readframes(1024)
        while len(data) > 0:
            s.write(data)
            data = w.readframes(1024)

        s.stop_stream()
        s.close()

        w.close()

        p.terminate()

        if window.playing: jukebox.play()

    play = Thread(target=playThread)
    play.start()

try:
    jukebox = Jukebox()

    app = QApplication(sys.argv)
    window = Window()
    window.show()
    t = QTimer()
    t.singleShot(1000,window.load)
    app.exec_()
finally:
    interrupted = True
