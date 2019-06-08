#! /bin/sh
DIR=$(cd $(dirname $0); pwd)
JCONF="$DIR/$1"
PORT=$2
cd $DIR
./julius -C $JCONF -C am-gmm.jconf -module $PORT
