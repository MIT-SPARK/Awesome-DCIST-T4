## Setting up bags and running Hydra

Can provide more detail, but basically you need to merge the odom source into all of the bags (I have a tool in ianvs for this now, ros2 also has a tool that will do this, but can't remap topics)

Then there's some infrastructure in hydra (under eval) that will run a set of trials and save the results.

All the (relevant) results from when I ran the different Hydra versions are in the zip directory on google drive

## Running Ouroboros

Setup a virtual environment:

```
python3 -m virtualenv ouroboros --download
source ouroboros/bin/activate
pip install -e ~/colcon_ws/src/ouroboros/ouroboros[learned]
```

You shouldn't need to recreate the descriptors, but see [here](../dcist_launch_system/scripts/make_outdoor_db.sh) and [here](../dcist_launch_system/scripts/make_outdoor_db.sh).

Getting loop closures:
```
python3 -m ouroboros loopclose kimera_multi_hybrid_salad_db.pkl -o hybrid_salad_loopclosures.json
```

You'll also need to run the ouroboros convert script on the output (in the dcist launch system script dir) which just remaps the scene names to robot IDs.


## Runnning fusion

Make sure you have your workspace sourced. Then you can run:

```
offline_fusion ~/hybrid_gt 12_08_acl_jackal2 12_08_sparkal1 12_08_sparkal2 --config-utilities-file ~/colcon_ws/src/awesome_dcist_t4/hydra_multi/hydra_multi/config/offline.yaml
```
or
```
offline_fusion ~/test_fusion 12_08_acl_jackal2 12_08_sparkal1 12_08_sparkal2 --config-utilities-file ~/colcon_ws/src/awesome_dcist_t4/hydra_multi/hydra_multi/config/offline.yaml -v=3 --config-utilities-yaml "{lcd_path: /data/iser_lcd/hybrid_salad_loopclosures_ordered.json}"
```

## Getting metrics

Requires `feature/iser` for the hydra-multi-system repo. Might be worth dumping the eval directory in hydra-multi directly instead, idk. Then
```
python get_metrics.py ~/hybrid_gt ~/test_fusion
```
should run the eval for a pair of scene graphs. Technically we could probably also have the same multi-robot structure as the full ablation eval that's already there, but this seemed easier to start with.
