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
sudo apt install pipx python3-virtualenv python3-colcon-clean wget
pipx install -f tmuxp
pipx install -f pre-commit
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.zshrc
```

Set up semantic inference dependencies [here](https://github.com/MIT-SPARK/semantic_inference/blob/ros2/docs/closed_set.md#getting-dependencies)
*(Required to run hydra online but not required for running from a bag with semantic inference data)*

Make your workspace (feel free to choose a different path):
```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws
```

Set up your environment variables (make sure you are in your workspace this):
```bash
echo export ADT4_WS=$(pwd) >> ~/.zshrc
echo export ADT4_DLS_PKG=${ADT4_WS}/src/awesome_dcist_t4/dcist_launch_system >> ~/.zshrc
echo export ADT4_ENV=${HOME}/environments/dcist >> ~/.zshrc

# Source to update changes
source ~/.zshrc
```

You may want to leave these unspecified on development machines:
```bash
echo export ADT4_OUTPUT_DIR=${HOME}/adt4_output/init >> ~/.zshrc
echo export ADT4_ROBOT_NAME=spot >> ~/.zshrc
```

You also will want to set up any secrets you have (**do not commit anywhere**):
```bash
echo export ADT4_BOSDYN_USERNAME=user >> ~/.zshrc
echo export ADT4_BOSDYN_PASSWORD=pass >> ~/.zshrc
echo export ADT4_BOSDYN_IP="192.168.80.3" >> ~/.zshrc
echo export ADT4_OPENAI_API_KEY=key >> ~/.zshrc
echo export ADT4_DEEPGRAM_API_KEY=key >> ~/.zshrc
```

Set up the ROS workspace and dependencies:
```bash
# Set up the ROS packages
cd ${ADT4_WS}
git clone git@github.com:MIT-SPARK/Awesome-DCIST-T4.git src/awesome_dcist_t4 --recursive
vcs import src < src/awesome_dcist_t4/install/packages.yaml
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
echo 'build: {symlink-install: true, cmake-args: [-DCMAKE_BUILD_TYPE=RelWithDebInfo]}' > colcon_defaults.yaml
```

You probably want to add some scripts to your path for convenience:
```bash
echo 'export PATH=$PATH:$ADT4_WS/src/awesome_dcist_t4/dcist_launch_system/bin' >> ~/.zshrc
```

Set up the Python environments and packages:
```bash
bash ${ADT4_WS}/src/awesome_dcist_t4/install/python_setup.bash
sudo ln -s ${ADT4_WS}/src/fast_downward/fast-downward.py /usr/local/bin/fast-downward
```

You probably want to use Zenoh (see bottom of README for details):
```bash
echo export RMW_IMPLEMENTATION=rmw_zenoh_cpp >> ~/.zshrc
source ~/.zshrc
```

Then, build the workspace as follows:
```bash
pushd ${ADT4_WS}  # this is CRUCIAL to get colcon to behave properly
colcon build --continue-on-error
popd
```

If building the workspace is too memory intensive, set the `MAKEFLAGS` environment
variable to restrict the number of parallel jobs and add the argument `--executor sequential`
to the `colcon build` command.

## Python environments

Different nodes can use different python environments; we have two (`roman` and `spark_env`) currently.
You should periodically rerun the python setup script to make sure everything is installed and up to date!

> :bangbang: **Important** </br>
> The `spark_dsg` package needs to build python bindings every time it is updated. This means that you need to manually pip install `spark_dsg`!
> You can do this without running the full python setup script by running
>    ```bash
>    source ${ADT4_ENV}/spark_env/bin/activate
>    pip install ${ADT4_WS}/src/awesome_dcist_t4/spark_dsg
>    ```


### Environment Variable Summary

| Environment Variable Name         | Description                                                       |
|-----------------------------------|-------------------------------------------------------------------|
| ADT4\_WS                          | Colcon workspace root                                             |
| ADT4\_ENV                         | Directory of Python venvs                                         |
| ADT4\_OUTPUT\_DIR                 | Output directory (logs, data, etc)                                |
| ADT4\_BOSDYN\_USERNAME            | Spot username                                                     |
| ADT4\_BOSDYN\_PASSWORD            | Spot password                                                     |
| ADT4\_BOSDYN\_IP                  | Spot IP (e.g, 192.168.80.3 for wifi)                              |
| ADT4\_PRIOR\_MAP                  | Path to ADT4 output logging directory that includes stored map    |
| ADT4\_ROBOT\_NAME                 | Robot name (e.g. "spot")                                          |
| ADT4\_DLS\_PKG                    | Path to dicst\_launch\_system package                             |
| ADT4\_SIM\_TIME                   | Use sim time? true/false                                          |
| ADT4\_OPENAI\_API\_KEY            | OpenAI API Key                                                    |
| ADT4\_DEEPGRAM\_API\_KEY          | Deepgram API Key                                                  |
| MAKEFLAGS="-j 4"                  | # of parallel jobs for colcon build                               |

Note that while some of these environment variables can be set to dummy values
(e.g., API Keys for functionality you aren't using), all of them need to be set
to *something* in order to launch the system properly.

### Silvus Network Computer IPs

| Hostname      | Description           | IP              |
|---------------|-----------------------|-----------------|
| euclid        | Jasper Jockey         | 192.167.70.3    |
| hamilton      | Topaz Jockey          | 192.167.70.2    |
| russell       | Aaron's laptop        | 192.167.70.101  |
| bonsai        | RRG dev computer      | ???.???.??.???  |
| willow        | RRG base station      | 192.167.70.100  |

### MIT Network Computer IPs
| Hostname      | Description           | IP              |
|---------------|-----------------------|-----------------|
| euclid        | Jasper Jockey         | 10.29.219.68    |
| hamilton      | Topaz Jockey          | 10.29.225.80    |
| russell       | Aaron's laptop        | 10.29.129.166   |
| bonsai        | RRG dev computer      | ???.???.??.???  |
| willow        | RRG base station      | 10.29.170.228   |

## Example System Configurations

### Hydra with Bag

If you want to see hydra running on real Spot data, get the bag from Aaron (and
convert by `pipx install rosbags; rosbags-convert --src
spot_hydra_twocam_test.bag --dst spot_hydra_twocam_test`).

Make sure you have an override file for the tf qos, e.g., `~/.tf_overrides.yaml`
looks like

```yaml
/tf_static: {depth: 1, durability: transient_local}
```

Then run
```bash
ADT4_SIM_TIME=true tmuxp load dcist_launch_system/tmux/autogenerated/spot_bag-bag.yaml
```

In a separate window, run the bag

```
ros2 bag play /path/to/spot_hydra_twocam_test --clock --qos-profile-overrides-path ~/.tf_overrides.yaml
```

### Prior Scene Graph and Movable Spot

Alternatively, you can drive spot around (and plan in?) a prior scene graph.

First, a quick discussion on logging.
During each session, a directory is created from the path set in `$ADT4_OUTPUT_DIR`.
Hydra scene graphs and ROMAN maps are stored in this directory (see [below](#saving-a-map))
Before using a prior scene graph, you need to set the `ADT4_PRIOR_MAP` environment variable to
the absolute path of the adt4 output directory used when creating the original scene graph
(if you `ls $ADT4_PRIOR_MAP` you should see `hydra` and `roman` subdirectories).

Then run
```
ADT4_SIM_TIME=false tmuxp load dcist_launch_system/tmux/autogenerated/spot_prior_dsg-spot_prior_dsg.yaml
```

There will be controls for moving spot around in one of the windows. You can
send spot a plan with:

```
ros2 topic pub /hilbert/omniplanner_node/language_planner/language_goal omniplanner_msgs/msg/LanguageGoalMsg "{robot_id: 'hilbert', domain_type: 'goto_points', command: 'R(70) R(23)'}" -1
```

You can also manually specify the list of string symbols, instead of relying on the "language" module to do it for you:
```
ros2 topic pub /hilbert/omniplanner_node/goto_points/goto_points_goal omniplanner_msgs/msg/GotoPointsGoalMsg "{robot_id: 'hilbert', point_names_to_visit: ['R(70)']}" -1
```

You can try out the TSP planner with:
```
ros2 topic pub /hilbert/omniplanner_node/tsp_planner/solve_tsp_goal omniplanner_msgs/msg/GotoPointsGoalMsg "{robot_id: 'hilbert', point_names_to_visit: ['R(69)', 'R(35)', 'R(71)', 'R(83)']}" -1
```

This should result in a planned path appearing in RVIZ, and spot following the
plan.

You can also send PDDL goals such as:
```
ros2 topic pub /hilbert/omniplanner_node/visit_objects_pddl/pddl_goal omniplanner_msgs/msg/PddlGoalMsg "{robot_id: 'hilbert', pddl_goal: '(or (visited-place r116) (and (visited-place r69) (visited-place r83)))'}" -1
```
Currently you can use `visited-place` and `visited-object` predicates and
general conjunctions/disjunctions/negations to specify goals.

This example shows how to send region-grounded goals. It should work, although currently it will take a *long* time to plan (~2+ minutes).

```
ros2 topic pub /hilbert/omniplanner_node/region_rearrange_objects_pddl/pddl_goal omniplanner_msgs/msg/PddlGoalMsg "{robot_id: 'hilbert', pddl_goal: '(and (visited-region r70) (at-place p1042) (object-in-place o94 p2157))'}" -1
```


### Catalog of Supported Launch Configurations

| Tmux Launch Name                                 | Description                                                                                        |
|--------------------------------------------------|----------------------------------------------------------------------------------------------------|
| `base_station-real_spot.yaml`                    | Runs base station planner, language interface, and visualizer                                      |
| `data_collection-real_spot.yaml`                 | Runs real spot interface for collecting sensor/robot data                                          |
| `real_spot_relocalize-real_spot_relocalize.yaml` | Runs online spot interface for relocalizing in a prior scene graph                                 |
| `real_spot-real_spot.yaml`                       | Runs online spot interface for building a scene graph and planning in current scene graph          |
| `spot_bag_relocalize-relocalize.yaml`            | Runs with single spot robot, based on a bag, using sim time and relocalizes in a prior scene graph |
| `spot_bag-bag.yaml`                              | Runs with single spot robot, based on a bag, using sim time                                        |
| `spot_prior_dsg-spot_prior_dsg.yaml`             | Runs with a single spot robot and a prior scene graph                                              |



## Tools & Utilities

### Saving a Scene Graph

Most of the launch configurations launch a scene graph saver node. You can save
a scene graph from the commandline by running

```
ros2 service call /dsg_saver/save_dsg dcist_launch_system_msgs/srv/SaveDsg "{save_path: '', include_mesh: true}"
```
If you leave `save_path`, it will default to saving under `$ADT4_OUTPUT_DIR`

## Configuring Parameters

This section will give some technical explanation about how parameters are
managed in ADT4. The following section builds on this and explains the full
power of the `experiment_manifest.yaml` file that allows composing overrides
and generating tmux launch files.

All ROS nodes in `ADT4` load their config from
`dcist_launch_system/config/<experiment_key>/<node_name>.yaml`. The yaml files
under the `<experiment_key>` directory have a full set of the yaml parameters
that are necessary to run the stack. However, most parameters are the same
between different configurations, usually requiring only a small number of
parameters to be changed from different robot or autonomy configurations.

To make it possible to keep track of which parameters can be shared between
configurations vs. which are system-specific, the system-specific
configurations are automatically generated from files that explicitly factorize
which parameters should be different for a given configuration.

In `dcist_launch_system/config_generation/base_params`, we store a full set of
*all* the parameters. Then, in
`dcist_launch_system/config_generation/experiment_overrides/<experiment_key>`
we have override files that specify specific parameters that should be
overridden for a given configuration. We use `config_utilities` to do this yaml templating.
For example, consider `base_params/spot_executor_node.yaml`:

```yaml
/**:
    ros__parameters:
        odom_frame: "map"
        body_frame: "body"
        use_fake_spot_pose: false
        fake_spot_external_pose: true
        spot_ip: "10.0.0.3"
        bosdyn_client_username: "$(env BOSDYN_CLIENT_USERNAME)"
        bosdyn_client_password: "$(env BOSDYN_CLIENT_PASSWORD)"
        .....
```

`use_fake_spot_pose` and `fake_external_pose` need to be changed depending on
whether we are running from a bag that provides a pose for spot, or a
"simulated" spot where spot controls its own pose. In
`config_generation/experiment_overrides/spot_prior_dsg/spot_executor_node_overlay.yaml`
we specify that for the `spot_prior_dsg` parameter setting, these two flags should be
flipped:

```yaml
---
/**:
    ros__parameters:
        use_fake_spot_pose: true
        fake_spot_external_pose: false
```

The naming of this file is important -- our compositor will update a `<node
name>.yaml` base param file by searching for `<node name>_overlay.yaml` file.
This specifies the values that should be used in the `base_params` file.

To do the generation, run `generate_configs.py`, which can be found in
`config_generation`. `generate_configs.py` takes three arguments -- a root
directory for the parameter specifications live (i.e.,
`dcist_launch_system/config_generation`), the directory where you want to
output config files (normally `dcist_launch_system/config`) and where to output
the generated tmux launch files (normally
`dcist_launch_system/tmux/autogenerated`). There is also a verbose and quiet
flag.

There is a script that has the correct input/output directories filled in, so
you should normally be able to run
`dcist_launch_system/scripts/generate_configs.sh`. But you can also run the
python script directly to control input/output directories, e.g.,:
```bash
cd dcist_launch_system/config_generation
./generate_configs.py . ../config ../tmux/autogenerated
```
You will need to source your workspace to make sure `composite-configs` (the
executable from `config_utilities` that handles compositing the YAML) is
avaiable on the path.

## Config Generation "Experiment Manifest"

The config generation process is controlled by a file called
`experiment_manifest.yaml` found in the root directory that
`generate_configs.py` is invoked with (e.g.,
`dcist_launch_system/config_generation/experiment_manifest.yaml`. There are
three parts of this file -- `launch_configs`, `configs`, and `experiments`.

The `launch_configs` allow you to build up a tmuxp file composed of reusable
windows. Each item under launch configs is a name for a group of windows. A
name with no children is a "window primitive", and a yaml file with the same
name should be present in `config_generation/launch_components`. Note that the
launch specification is composable, in the sense that the children of a launch
config don't need to be primitives.

The `configs` follow a similar compositional logic. A config with no children
should be a directory under `experiment_overrides`. A parameter grouping that
does have children will apply all descendant primitive configs as overrides.

Finally, the `experiments` section generates launch files for the specified
combination of launch config X parameter grouping X {other variables}. The
syntax and semantics of these {other variables} is going to change soon, but
roughly they map to environment variables that are set in the tmux file.

## Zenoh
Middleware is, of course, a personal choice, though in this project, the
default is to use Zenoh.
### Installation
```bash
sudo apt update && sudo apt install ros-jazzy-rmw-zenoh-cpp
```
### Setting Default RMW
```bash
echo export RMW_IMPLEMENTATION=rmw_zenoh_cpp >> ~/.zshrc
```
### Starting Zenoh Router
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```
By default, the tmux launch files will start the zenoh router for you, so you
probably don't need to run this command manually.

### Connecting Multiple Zenoh Routers
To connect multiple Zenoh routers across laptops, copy
[DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5](https://github.com/ros2/rmw_zenoh/blob/rolling/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5)
to a location of your choosing (e.g., your home directory) and modify the
`connect` block to include the endpoint(s) that the other host's `Zenoh router(s)`
is listening on. For example, if another `Zenoh router` is listening on IP address
`192.168.1.1` and port `7447` (the default) on its host, modify the config file to connect to
this router as shown below:

```json5
/// ... preceding parts of the config file.
{
  connect: {
    endpoints: ["tcp/192.168.1.1:7447"],
  },
}
/// ... following parts of the config file.
```
Then, start the `Zenoh router` after setting the `ZENOH_ROUTER_CONFIG_URI` environment variable to the absolute path of the modified config file.

```bash
echo export ZENOH_ROUTER_CONFIG_URI=/path/to/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 >> ~/.zshrc
```
For additional instructions, please refer [here](https://github.com/ros2/rmw_zenoh/).

### Saving a Map

A current session's map will be saved to the `$ADT4_OUTPUT_DIR` directory when you either:

* Press enter in the bottom pane in the `core` tmux window.
This will automatically cause ROMAN and Hydra to log their outputs.
* Or use `Ctrl-c` on the ROMAN mapping node (top right pane in the `roman` tmux window) and call the service: `ros2 service call /${ADT4_ROBOT_NAME}/shutdown std_srvs/Empty` to have Hydra shutdown

After doing one of these two options, the following paths should exist:
* `$ADT4_OUTPUT_DIR/roman/roman_map.pkl`
* `$ADT4_OUTPUT_DIR/hydra`

## Sponsors

A big thank you to our sponsors for their generous support:

* [ARL Distributed and Collaborative Intelligent Systems and Technology Collaborative Research Alliance (DCIST
CRA) agreement W911NF-17-2-0181](https://arl.devcom.army.mil/cras/dcist-cra/)
