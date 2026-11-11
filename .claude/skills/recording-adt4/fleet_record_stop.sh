#!/bin/bash
# Stop the recordings cleanly (Ctrl-C so ros2 bag writes metadata.yaml) and report bag size.
M=${1:-$(dirname "$0")/machines.txt}
while read -r h d tf; do
  case $h in "" | "#"*) continue ;; esac
  ( timeout 60 ssh -n -o BatchMode=yes -o ConnectTimeout=8 "$h" "
    O=\$HOME/adt4_output/$d/recorded_data
    tmux send-keys -t adt4_system:recording C-c 2>/dev/null
    for i in \$(seq 1 30); do pgrep -f 'bin/ros2 bag recor[d]' >/dev/null || break; sleep 1; done
    echo \"\$(hostname): bag_proc=\$(pgrep -fc 'bin/ros2 bag recor[d]') metadata=\$( [ -f \$O/metadata.yaml ] && echo ok || echo MISSING) size=\$(du -sh \$O 2>/dev/null | cut -f1)\"
  " 2>&1 || echo "$h: ssh failed" ) &
done < "$M"
wait
