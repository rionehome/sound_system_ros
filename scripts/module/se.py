import rospkg
import subprocess

PATH = rospkg.RosPack().get_path('sound_system') + "/SE"


class SE:
    def __init__(self):
        self.se_path = "{}/etc/SE/".format(rospkg.RosPack().get_path('sound_system'))
        self.WAKEUP = self.se_path + "wakeup.wav"
        self.START = self.se_path + "start.wav"
        self.STOP = self.se_path + "stop.wav"
    
    @staticmethod
    def play(se):
        # type: (str) -> None
        subprocess.call(["aplay", se], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
