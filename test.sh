#!/bin/sh
port=16

gpio -g mode $port in

echo "はじめ"

while true ; do
 btn=`gpio -g read $port`
 if [ $btn -eq 1 ] ; then
  echo "ok"
  sudo navit -c ~/.navit/navit.xml
  sleep 2
 fi

done