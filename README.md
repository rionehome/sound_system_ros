# sound_system_ros
## Overview
音声認識、発話を管理するパッケージです。

音声認識に julius

Hotword検知には Snowboy

発話は Svox

## Setup
発話にSvoxPicoを使用しています
以下のコマンドでインストールが必要です
```
sudo apt-get install -y libttspico-utils
```

Hotword検出に使っているSnowboyはC言語のライブラリに依存している

scripts/module/の中にある_snowboydetect.soがCのライブラリ
ただし各自の環境依存でコンパイルするものなので場合によっては動かない可能性ある

その場合は自分でmakeして出来たものと入れ替えて欲しい
詳しくはQiitaなどでググッて
もしかしたら後日書くかもしれない


## Usage

(音声認識,発話など音声認識の中心)
```
roslaunch sound_system sound_system.launch
```

(Hotwordの検知)
```
roslaunch sound_system hotword.launch 
```

(ナビゲーション用のなんちゃって言語処理)
```
roslaunch sound_system navigation_nlp.launch
```

起動直後はHotword検知待ち

Hotwordは「Hey Ducker」


動かすときは「Hey Ducker」で起動させ、その後目的の命令を話すという形式

## Node
**`name` sound_system**