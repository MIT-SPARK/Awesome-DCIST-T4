# ADT4 web console

A browser viewer for ADT4 maps: the 3D scene graph (objects, places, rooms, the robot
trajectory), the mesh, object images, keyframes and the places they see, with an optional
satellite underlay. Maps come from a **run directory** (`$ADT4_OUTPUT_DIR` of a mapping run) or
from **Neo4j** (heracles), and a run can be loaded into Neo4j from the Maps page.

The server (`adt4_console/`, FastAPI) runs in `spark_env`; the UI (`frontend/`, React + Vite) is
built into `adt4_console/static/` at install time. The HTTP / WebSocket API is in
[docs/API.md](docs/API.md).

## Install

`install/python_setup.bash` installs the Python dependencies (`install/spark_requirements.txt`)
and builds the UI when Node.js 20.19 or newer is present. Ubuntu's `nodejs` (18.x) is too old for
Vite 7; install Node 22 first, e.g. with nvm:

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
nvm install 22
```

or from [NodeSource](https://github.com/nodesource/distributions). To (re)build only the UI:

```bash
cd $ADT4_WS/src/awesome_dcist_t4/adt4_console/frontend
npm ci && npm run build      # writes ../adt4_console/static
```

Without a built UI the server still runs and serves the API, with a placeholder page that says
how to build it. `colcon build` installs the package (and the UI, if built) as `adt4_console`.

## Run

```bash
source $ADT4_WS/install/setup.zsh     # spark_dsg, heracles, hydra_python
$ADT4_WS/src/awesome_dcist_t4/adt4_console/bin/adt4-console --map run=$ADT4_OUTPUT_DIR
```

and open http://localhost:8090 (or `http://<base station>:8090`). `bin/adt4-console` re-execs
itself in `$ADT4_ENV/spark_env`. Useful options (`--help` for all):

| option | |
|---|---|
| `--map NAME=RUN_DIR` | a run directory (with `hydra/backend/dsg.json`) as source `file:NAME`; repeatable |
| `--neo4j NAME=URI` | a heracles Neo4j database as source `neo4j:NAME`, e.g. `live=bolt://localhost:7687`; repeatable |
| `--live-neo4j NAME` | the `--neo4j` target that the Maps page loads runs into (default: the first) |
| `--output-root DIR` | where the Maps page lists runs (default `~/adt4_output`) |
| `--port`, `--host` | default `0.0.0.0:8090` |
| `--token TOKEN` | require a token for the API (`$ADT4_CONSOLE_TOKEN`); open the UI as `http://host:8090/?token=TOKEN` |
| `--geo-anchor`, `--tiles-dir`, `--tiles-live` | satellite underlay, see below |

Neo4j credentials come from `HERACLES_NEO4J_USERNAME` / `HERACLES_NEO4J_PASSWORD`, else
`ADT4_NEO4J_USERNAME` / `ADT4_NEO4J_PASSWORD`, else `neo4j` / `neo4j_pw`.

The `base_station_heracles` session starts the console on the base station (tmux window
`console`), showing the heracles database (`ADT4_HERACLES_IP` / `ADT4_HERACLES_PORT`) and, when
`ADT4_PRIOR_MAP` is set, the prior map's run directory.

## View a map with images

1. Build a map with the `image_logging` config, e.g. from a bag:
   `run-adt4 spot_bag-image_logging ... --output-dir $RUN`, and save it (see "Saving a Map" in
   the top-level README). Hydra and Khronos write keyframe archives and object track logs.
2. Index the images (optional: the console does it on first open, which takes a while on a big
   run): `hydra image-store index $RUN` (writes `$RUN/image_store`), then
   `hydra image-store check $RUN`.
3. `adt4-console --map run=$RUN`. In the viewer, pick an object to see its images (with masks), a
   trajectory pose for its keyframe, or a traversability place for the keyframes that saw it.

## Neo4j maps: load and activate

With `--neo4j`, the Maps page lists the runs under `--output-root` and can **load** one into the
live Neo4j target: the database is wiped (after a confirmation if other tools added objects or
edits) and filled in stages that the page shows as a progress job: `graph` (the map is shown from
here on), `images` (links the image store; with a heracles that supports it also the image
evidence as Observation / keyframe rows), `features` (place / room features from
`hydra/backend/semantic_features.npz`, if present) and `keyframes` (the place visibility index).
A run already in the database is not reloaded. Planners and other tools reading that Neo4j see the
new map as well.

The console works with the heracles version ADT4 pins. Some features need newer heracles
functions; without them they are off and the server logs which one is missing once:
`read_version` (change polling by version, incremental updates; otherwise changes are noticed by
the node count), `read_mesh_path` / `store_mesh_path` (mesh of Neo4j maps),
`add_agents_from_dsg` (trajectory of loaded maps) and the image ingest functions (Observation rows;
object images then come straight from the linked image store).

## Satellite underlay

A map can be drawn over satellite imagery when it has a GPS anchor (map frame → UTM). Fit one
from the bag's GPS fixes and the Hydra trajectory (needs `mcap`, `rosbags`, `pyproj`):

```bash
cd $ADT4_WS/src/awesome_dcist_t4/adt4_console/scripts/geo
python3 anchor_from_gps.py --bag $BAG --topic /<robot>/fix \
    --trajectory $RUN/hydra/backend/trajectory.csv --out $RUN/geo_anchor.json --min-status 2
```

The console picks up `<run>/geo_anchor.json` (or its parent's) for that run's sources; pass
`--geo-anchor SRC=FILE` (`SRC` = source name or `*` for all) to use another one. Tiles are read
from `--tiles-dir` (default `~/.adt4_console/tiles/<layer>/{z}/{x}/{y}.jpg`). Fetch them ahead of
time for offline use:

```bash
python3 fetch_tiles.py --anchor $RUN/geo_anchor.json \
    --trajectory $RUN/hydra/backend/trajectory.csv --margin-m 80 --zmin 15 --zmax 19
```

or start the console with `--tiles-live` to fetch missing tiles from the internet (and keep
them). The viewer's layer panel turns the underlay on and off.

## Tests

```bash
cd adt4_console
$ADT4_ENV/spark_env/bin/python -m pytest tests
```

The Neo4j tests wipe their database, so they only run against a scratch instance named in
`ADT4_TEST_SCRATCH_NEO4J`:

```bash
docker run -d --name console_test_neo4j -p 127.0.0.1:7721:7687 -e NEO4J_AUTH=neo4j/neo4j_pw neo4j:5.25.1
ADT4_TEST_SCRATCH_NEO4J=bolt://localhost:7721 $ADT4_ENV/spark_env/bin/python -m pytest tests
```
