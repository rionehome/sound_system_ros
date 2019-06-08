# sound_system_ros
## Overview
音声認識、発話を管理するパッケージです。

デフォルトでは音声認識にjuliusを使用しています。

## Setup
発話にSvoxPicoを使用しています
以下のコマンドでインストールが必要です
```
sudo apt-get install -y libttspico-utils
```

## Usage

(音声認識)
```
roslaunch sound_system sound_system.launch
```
or

(音声認識＋発話)
```
roslaunch sound_system sound_system_demo.launch
```

**起動直後は音声認識は停止している**

**`/sound_system/recognition/active`** に true をpublishする

## Node
**`name` sound_system**

### Subscribe Topic

* **`/sound_system/recognition/activate`** 音声認識のresume/pause ( std_msgs/Bool )
    true : resume
    false: pause

* **`/sound_system/speak`** 文の読み上げ ( std_msgs/String )

* **`/sound_system/recognition`** 音声認識の開始要求 ( std_msgs/String )


### Publish Topic

* **`/sound_system/recognition/result`** 音声認識の結果文 ( std_msgs/String )


### Parameter

* **`config`** juliusのjconfファイルを指定 ( default = localhost )

* **`host`**   juliusのホストを指定します ( default = localhost )

* **`port`**   juliusのポートを指定 ( default = 10500 )

* **`debug`**  juliusの標準出力の表示判定 ( default = false )

