#!/bin/bash
if [ ! "$BASH_VERSION" ] ; then
    exec /bin/bash "$0" "$@"
fi

# launch simulation first
./simviz_project &
SIMVIZ_PID=$!

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    kill -2 $SIMVIZ_PID  
}

sleep 5

# launch controller
./controller_project &
CONTROLLER_PID=$!

sleep 1.5

# launch interfaces server (Cradles1.wav or Cradles.wav)
canberra-gtk-play -f ./resources/Cradles.wav &
AUDIO_PID=$!

# wait for simviz to quit
wait $SIMVIZ_PID

# onnce simviz dies, kill controller & interfaces server
kill $CONTROLLER_PID
#for pid in $(ps -ef | grep interface/server.py | awk '{print $2}'); do kill -9 $pid; done
kill $AUDIO_PID
