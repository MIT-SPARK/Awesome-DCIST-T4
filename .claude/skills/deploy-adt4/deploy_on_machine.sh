#!/usr/bin/env zsh
# Deploy an Awesome-DCIST-T4 branch on THIS machine: check out the branch in the colcon
# workspace, update every submodule recursively, and build. Run it on the machine itself
# (fleet_deploy.sh copies it over and starts it in a tmux session).
#
#   deploy_on_machine.sh <branch> [repo_url]
#
# Env: ADT4_WS (default ~/dcist_ws). Log: ~/adt4_deploy_<YYYYMMDD>.log.
BRANCH=${1:?usage: deploy_on_machine.sh <branch> [repo_url]}
REPO=${2:-https://github.com/MIT-SPARK/Awesome-DCIST-T4.git}
WS=${ADT4_WS:-$HOME/dcist_ws}
STAMP=$(date +%Y%m%d)
LOG=$HOME/adt4_deploy_$STAMP.log
exec > >(tee -a $LOG) 2>&1
echo "=== deploy $BRANCH start $(date)"
cd $WS/src/awesome_dcist_t4 || { echo "NO_REPO $WS/src/awesome_dcist_t4"; exit 1; }

# 1. Snapshot what is there now, so the machine can go back.
SNAP=$HOME/adt4_predeploy_$STAMP.txt
{ date; git rev-parse --abbrev-ref HEAD; git log -1 --oneline; git submodule status; git status --short; } > $SNAP 2>&1
git branch -f predeploy-$STAMP HEAD
echo "snapshot: $SNAP, branch predeploy-$STAMP"

# 2. Check out the branch.
git fetch -q "$REPO" "$BRANCH" || { echo "FETCH_FAIL"; exit 1; }
TARGET=$(git rev-parse FETCH_HEAD)
git checkout -q -f -B "$BRANCH" "$TARGET" 2>&1 | grep -v "unable to rmdir"
[ "$(git rev-parse HEAD)" = "$TARGET" ] || { echo "CHECKOUT_FAIL"; exit 1; }
echo "repo: $(git log --oneline -1)"

# 3. A submodule path that is a plain, non-empty folder (code that used to live in the repo
# itself) blocks `git submodule update`; move it aside instead of deleting it.
ASIDE=$HOME/adt4_predeploy_${STAMP}_intree
for p in $(git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}'); do
  if [ -d "$p" ] && [ ! -e "$p/.git" ] && [ -n "$(ls -A "$p" 2>/dev/null)" ]; then
    mkdir -p $ASIDE && mv "$p" $ASIDE/ && echo "moved plain folder $p aside to $ASIDE"
  fi
done

# 4. Submodules, recursively. GitHub serves commits that only exist on a fork through the
# upstream repo, so pins to fork commits fetch like any other.
git submodule sync -q --recursive
git submodule update --init --recursive 2>&1 | grep -v -e "^Submodule path" -e "unable to rmdir" | tail -20
BAD=0
for p in $(git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}'); do
  [ "$(git config --get submodule.$p.update)" = none ] && continue
  want=$(git ls-tree HEAD "$p" | awk '{print $3}'); [ -z "$want" ] && continue
  have=""; [ -e "$p/.git" ] && have=$(git -C "$p" rev-parse HEAD 2>/dev/null)
  [ "$have" = "$want" ] || { echo "MISMATCH $p want ${want:0:8} have ${have:0:8}"; BAD=1; }
done
# never act on the main repo by mistake: it must still be at the branch
[ "$(git rev-parse HEAD)" = "$TARGET" ] || { echo "REPO_MOVED"; exit 1; }
[ $BAD = 0 ] && echo "submodules: all at their pins" || { echo "SUBMODULES_FAIL"; exit 1; }

# 5. Build.
echo "=== build $(date)"
cd $WS && source /opt/ros/jazzy/setup.zsh && colcon build --continue-on-error 2>&1 | tail -30
echo "BUILD_EXIT=${pipestatus[1]}"
echo "=== deploy end $(date)"
