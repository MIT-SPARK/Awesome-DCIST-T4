#!/bin/bash

DATAPATH="/data/datasets/kimera_multi/hybrid_12_08"
RGB_TOPIC="forward/color/image_raw/compressed"
DEPTH_TOPIC="forward/depth/image_rect_raw"
CMD="python3 -m ouroboros bag"

$CMD $DATAPATH/12_08_acl_jackal2 /acl_jackal2/$RGB_TOPIC -d /acl_jackal2/$DEPTH_TOPIC -o /data/kimera_multi_hybrid_salad_db.pkl
for name in sparkal1 sparkal2
do
    $CMD $DATAPATH/12_08_$name /$name/$RGB_TOPIC -d /$name/$DEPTH_TOPIC -a /data/kimera_multi_hybrid_salad_db.pkl
done
