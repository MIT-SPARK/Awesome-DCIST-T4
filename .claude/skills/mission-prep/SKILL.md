---
name: mission-prep
description: Turn a mission described in words ("delete all cones, add cones at T5840 and T5855, make T2655 and its neighbours 'base camp', block T36 and T7181, the square around T4465/T6797/T5558/T5553 is a parking lot") into a mission-prep edit file (adt4-mission-prep/1 JSON) for the web console's Launch window / map editor, resolved against the map loaded on the base station (willow), with overhead renders (plain + satellite) of places, regions and objects. Use for new mission files, follow-up files ("also add ...", "remove the obstacles"), or re-rendering a file.
---

# Mission-prep edit files

A **mission-prep file** edits the live map for a mission: delete / add objects, remove
places as obstacles (and put them back), add or change named regions. The console
applies a file as ONE map edit (one transaction, one map-version bump, one Undo, plan
repair notified), at launch (after the base station reloads the prior) or later from the
map editor; **Reset** undoes the applied files. Files on the base station in
`~/adt4_output/mission_prep/` (recursively) are listed in both places. While a trial
records, every applied file is copied into the trial's log. Code:
`adt4_console/mission_prep/` (console repo), CLI `python -m adt4_console.mission_prep`.

**Never apply a file to willow's map yourself, and never (re)launch.** Make the file +
renders, copy them to willow's `~/adt4_output/mission_prep/`, and tell Harel. (Exception:
he explicitly asks.)

## 1. Intent steps

Write the mission as `intent.json`, a list of steps. Place symbols are matched ignoring
case (`t5840` = `T5840`).

| Mission wording | Step |
|---|---|
| delete all cones | `{"do": "delete_objects", "class": "cone"}` (re-resolved at load: deletes every cone the live map has then) |
| delete O12, O40 / cones near T5 | `{"do": "delete_objects", "symbols": ["O12", "O40"]}` / `{"do": "delete_objects", "class": "cone", "near": {"place": "T5", "radius": 3}}` |
| add cones at T5840, T5855 | `{"do": "add_objects", "class": "cone", "at": ["T5840", "T5855"]}` (on the place's center, attached to it) |
| remove places / obstacles at T36, T7181 | `{"do": "retire_places", "places": ["T36", "T7181"], "reason": "obstacle (mission 2)"}` |
| put the places back / remove the obstacles | `{"do": "restore_places", "places": ["T36"]}`, or `"places": "all"` for every removed place (generic) |
| a region over T5122 and adjacent places, called "sidewalk end" | `{"do": "add_region", "name": "sidewalk end", "class": "sidewalk", "places": ["T5122"], "expand": "adjacent"}` |
| T2655 and T4289 are "base camp" | `{"do": "add_region", "name": "base camp", "places": ["T2655", "T4289"]}` (add `"expand": "adjacent"` if he says "and around") |
| the square around T4465, T6797, T5558, T5553 is "Parking Lot" | `{"do": "add_region", "name": "...", "class": "parking lot", "polygon_from": ["T4465", "T6797", "T5558", "T5553"]}` (places inside the convex hull of the corners) |
| add T7 (+ neighbours) to "base camp" | `{"do": "add_to_region", "name": "base camp", "places": ["T7"], "expand": "adjacent"}` |
| rename / delete a region | `{"do": "rename_region", "name": "a", "to": "b"}` / `{"do": "delete_region", "name": "a"}` |

Rules the compiler applies (tell Harel when they matter):
- **adjacent** = the place + its 1-hop `MESH_PLACE_CONNECTED` neighbours.
- **A new region takes its places**: a place has one parent region (the planner reads
  one), so members leave their old region. A later region's adjacent ring / polygon
  interior never takes an earlier (same-file) region's places; a place named explicitly
  always moves.
- **Region names are unique ignoring case**: "Parking Lot" collides with an existing
  "parking lot" -> ask for another name (suggest "parking lot 7"; 2026-11-18 Harel chose
  "intersection 2" over reusing "intersection").
- `class` is the region kind (labelspace: parking lot, intersection, sidewalk, road,
  courtyard, ...); a new class (e.g. "base camp") is added. Default class = the name.
- Removed places never join regions; steps run in order.

Ambiguities to ASK about (AskUserQuestion) before compiling: an id that does not resolve
(often a typo: T244 was T2444), one region vs one per listed place, a name collision.

## 2. Compile against willow's live map (read only)

```bash
ssh -N -L 17687:127.0.0.1:7687 rrg@100.106.173.34 &   # willow over tailscale; read-only use
TUN=$!
PY=~/environments/dcist/spark_env/bin/python          # neo4j, matplotlib, contextily, shapely
export PYTHONPATH=<adt4_console checkout>:$PYTHONPATH   # e.g. $ADT4_WS/src/awesome_dcist_t4/adt4_console
OUT=~/adt4_output/mission_prep/<prior>/<name>         # local, NOT in git; e.g. .../penn2026_trav_ganav_regions/mission2
$PY -m adt4_console.mission_prep compile --neo4j bolt://127.0.0.1:17687 \
    --intent intent.json --name mission2 --summary "Mission 2: ..." --out $OUT/mission2.json
```
It prints the report (`<name>_report.md`: what each step resolved to, members, places
taken from which regions) and exits 1 on unresolved steps. Fix or ask, then recompile.
- A **follow-up** file that builds on files not yet applied: `--after mission2.json`.
  Once willow has applied them, compile against the live map (it already holds them).
- A **generic** file (any mission / map): `--any-map`. E.g. `remove_obstacles.json` (on
  willow at the top of `mission_prep/`) is intent `[{"do": "restore_places", "places": "all"}]`.

## 3. Render (plain + satellite, whole map + zoom on the edits)

```bash
scp rrg@100.106.173.34:adt4_output/<prior>/geo_anchor.json /tmp/geo_anchor.json
$PY -m adt4_console.mission_prep render --neo4j bolt://127.0.0.1:17687 --file $OUT/mission2.json \
    --geo-anchor /tmp/geo_anchor.json --out-dir $OUT/png [--after mission1.json]
kill $TUN
```
Four PNGs: `<name>_{plain,satellite}_{full,zoom}.png`. Places (grey dots + edges), regions
(discs around their places; new ones bold, labelled "name (R#, n places)"), objects
(added = stars, deleted = red crosses, cones = triangles), removed places (black boxed
crosses), referenced place ids. No mesh. Show them to Harel (SendUserFile) — at least
`satellite_zoom` and `plain_full`.

## 4. Hand over

```bash
rsync -a $OUT/ rrg@100.106.173.34:adt4_output/mission_prep/<prior>/<name>/
```
Tell Harel: the file is listed in the console under Launch -> *Mission edit files*
(applied after the base reloads the prior) and map editor -> *Mission edit files*
(Check, then Apply on the running map); Reset undoes applied files.

**Mission files and renders never go in git** (any repo): they live on the base station
(`~/adt4_output/mission_prep/`, the console's list) and locally in `$OUT`.

## Checks a file passes

`python -m adt4_console.mission_prep verify --file F --neo4j bolt://127.0.0.1:17687` shows
what loading it now would do (read only). The console's Check button (`POST
/api/prep/check`) runs the same resolution plus a graph_edit dry run.
