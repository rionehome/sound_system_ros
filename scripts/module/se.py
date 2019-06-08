import rospkg
import subprocess
import rospy

PATH = rospkg.RosPack().get_path('sound_system') + "/SE"


class SE:
    WAKEUP = PATH + "/" + "wakeup.wav"
    START = PATH + "/" + "start.wav"
    STOP = PATH + "/" + "stop.wav"

    @staticmethod
    def play(se):
        # type: (str) -> None
        subprocess.call(["aplay", se], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
