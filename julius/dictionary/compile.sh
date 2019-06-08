#! /bin/sh

WS=$(cd $(dirname $0); pwd)
yomi=$1

iconv -f utf-8 -t eucjp ${WS}/$yomi.yomi | perl ${WS}/yomi2voca.pl > ${WS}/$yomi.dic
