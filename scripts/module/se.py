import rospkg
import subprocess

PATH = rospkg.RosPack().get_path('sound_system') + "/SE"


class SE:
    def __init__(self):
        self.WAKEUP = PATH + "/" + "wakeup.wav"
        self.START = PATH + "/" + "start.wav"
        self.STOP = PATH + "/" + "stop.wav"
    
    @staticmethod
    def play(se):
        # type: (str) -> None
        subprocess.call(["aplay", se], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
