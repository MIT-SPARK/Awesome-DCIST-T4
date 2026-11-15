#!/usr/bin/env bash
# Deploy an Awesome-DCIST-T4 branch to several machines over SSH.
#
#   fleet_deploy.sh start  <branch> <ssh target>...   copy deploy_on_machine.sh and start it in tmux `adt4_deploy`
#   fleet_deploy.sh status <ssh target>...            summary of each machine's deploy log
#   fleet_deploy.sh wait   <ssh target>...            block until every deploy has ended, then print status
#
# start refuses a machine that has a running adt4_system session or ros2 bag recorder:
# stop those first (Ctrl-C a recording so the bag gets its metadata.yaml).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
MODE=${1:?usage: fleet_deploy.sh start <branch> <ssh target>... | status|wait <ssh target>...}; shift
SSH="ssh -o BatchMode=yes -o ConnectTimeout=8"
summary() {
  $SSH "$1" 'f=$(ls -t ~/adt4_deploy_*.log 2>/dev/null | head -1); [ -z "$f" ] && { echo "  no deploy log"; exit; }
    grep -E "^(repo:|moved|MISMATCH|submodules:|SUBMODULES_FAIL|REPO_MOVED|CHECKOUT_FAIL|FETCH_FAIL|=== build|BUILD_EXIT|=== deploy end)|Summary:|Failed" "$f" | tail -12 | sed "s/^/  /"' 2>&1
}
case $MODE in
  start)
    BRANCH=${1:?branch}; shift
    for h in "$@"; do
      echo "== $h"
      busy=$($SSH "$h" 'tmux has-session -t adt4_system 2>/dev/null && echo session; pgrep -f "bin/ros2 bag recor[d]" >/dev/null && echo recording' 2>&1)
      if [ -n "$busy" ]; then echo "  SKIPPED: running $busy (stop it first)"; continue; fi
      home=$($SSH "$h" 'echo $HOME') || { echo "  unreachable"; continue; }
      scp -q -o BatchMode=yes "$HERE/deploy_on_machine.sh" "$h:$home/adt4_deploy_on_machine.sh" &&
      $SSH "$h" "test -s $home/adt4_deploy_on_machine.sh && { tmux kill-session -t adt4_deploy 2>/dev/null; tmux new-session -d -s adt4_deploy 'zsh -i $home/adt4_deploy_on_machine.sh $BRANCH; exec zsh' && echo '  started (tmux adt4_deploy)'; }" ||
        echo "  FAILED to start"
    done ;;
  status)
    for h in "$@"; do echo "== $h"; summary "$h"; done ;;
  wait)
    for h in "$@"; do
      until $SSH "$h" 'f=$(ls -t ~/adt4_deploy_*.log 2>/dev/null | head -1); grep -qE "=== deploy end|_FAIL|REPO_MOVED" "$f"' 2>/dev/null; do sleep 30; done
    done
    for h in "$@"; do echo "== $h"; summary "$h"; done ;;
  *) echo "unknown mode $MODE"; exit 2 ;;
esac
