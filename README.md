# 😁 Awesome DCIST T4 😁

<div align="center">
  A curated collection of resources focused on the MIT DCIST 🪖 stack and related technologies.

  [**Browse the CRA**](https://arl.devcom.army.mil/cras/dcist-cra/) | [**Contribute**](CONTRIBUTING.md)
</div>


## Contents

- [Installation](#installation)
- [Documentation](#documentation)
- [Tools & Utilities](#tools-utilities)
- [Sponsors](#sponsors)


## Installation

Install dependencies:
```bash
sudo apt install pipx python3-virtualenv
pipx install -f tmuxp pre-commit
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.zshrc
```

Set up the workspace:
```bash
# Feel free to change the workspace
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws
git clone git@github.com:MIT-SPARK/Awesome-DCIST-T4.git src/awesome_dcist_t4 --recursive
vcs import src < src/awesome_dcist_t4/install/packages.yaml
rosdep install --from-paths src --ignore-src -r -y
echo export DCIST_WS=`pwd` >> ~/.zshrc

# Feel free to change the environment path (currently unused)
python3 -m virtualenv ~/environments/dcist --download
echo export DCIST_ENV=$(realpath ~/environments/dcist) >> ~/.zshrc
```

Build:
```bash
# this next step is CRUCIAL to get colcon to behave properly
pushd $DCIST_WS
# this will error most likely but it's fine...
colcon build --continue-on-error --symlink-install
popd
```

You should be able to load a tmuxp launch file by navigating to
`awesome-dcist-t4/dcist_launch_system/tmux` and running

```bash
tmuxp load dcist_launch.yaml
```

## Documentation
Haha

## Tools & Utilities

## Sponsors

A big thank you to our sponsors for their generous support:

* [ARL Distributed and Collaborative Intelligent Systems and Technology Collaborative Research Alliance (DCIST
CRA) agreement W911NF-17-2-0181](https://arl.devcom.army.mil/cras/dcist-cra/)
