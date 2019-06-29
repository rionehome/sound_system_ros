import rospkg
import subprocess


class SE:
    def __init__(self):
        self.path = rospkg.RosPack().get_path('sound_system') + "/SE"
        self.wakeup = self.path + "/" + "wakeup.wav"
        self.start = self.path + "/" + "start.wav"
        self.stop = self.path + "/" + "stop.wav"
        self.sound_list = {"wakeup": self.wakeup, "start": self.start, "stop": self.stop}
    
    def play(self, se):
        # type: (str) -> None
        if se in self.sound_list:
            subprocess.call(["aplay", self.sound_list[se]], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
