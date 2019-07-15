import rospkg
import subprocess


class SE:

    def __init__(self):
        path = rospkg.RosPack().get_path('sound_system') + "/SE"
        self.WAKEUP = path + "/" + "wakeup.wav"
        self.START = path + "/" + "start.wav"
        self.STOP = path + "/" + "stop.wav"

    @staticmethod
    def play(se):
        # type: (str) -> None
        subprocess.call(["aplay", se], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
