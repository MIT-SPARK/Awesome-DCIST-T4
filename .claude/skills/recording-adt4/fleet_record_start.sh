#!/bin/bash
# Start run-adt4 -r style recordings in each machine's running adt4_system session (window "recording").
M=${1:-$(dirname "$0")/machines.txt}
while read -r h d tf; do
  case $h in "" | "#"*) continue ;; esac
  ( timeout 40 ssh -n -o BatchMode=yes -o ConnectTimeout=8 "$h" "
    O=\$HOME/adt4_output/$d; T=\$HOME/dcist_ws/src/awesome_dcist_t4/dcist_launch_system/recording/${tf}_topics.txt
    tmux has-session -t adt4_system 2>/dev/null || { echo \"\$(hostname): no adt4_system session\"; exit 1; }
    [ -e \$O/recorded_data ] && { echo \"\$(hostname): \$O/recorded_data exists, not starting\"; exit 1; }
    tmux list-windows -t adt4_system -F '#W' | grep -qx recording && { echo \"\$(hostname): recording window already open\"; exit 1; }
    tmux new-window -d -t adt4_system -n recording \"zsh -ic 'source \\\${ADT4_WS}/install/setup.zsh && ros2 run dcist_launch_system record_topics \$O/recorded_data \$T; exec zsh'\"
    sleep 8; echo \"\$(hostname): started -> \$O/recorded_data (\$(basename \$T)) bag_proc=\$(pgrep -fc 'bin/ros2 bag recor[d]')\"
  " 2>&1 || echo "$h: not started" ) &
done < "$M"
wait
