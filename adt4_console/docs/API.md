# ADT4 Console — API & protocol contract

The console is a FastAPI server (`adt4_console/adt4_console`, runs in `spark_env`) and a
React + Vite SPA (`adt4_console/frontend`, built into `adt4_console/adt4_console/static/` at
install time). This document is the contract between them.

The graph and mesh-tile wire formats are defined in `adt4_console/adt4_console/codec.py` (read
its module docstring); `frontend/src/codec.ts` mirrors it byte for byte.

## 0. Process / environment

* Python: `$ADT4_ENV/spark_env` (fastapi, uvicorn, msgpack, pyproj, numpy, scipy, pillow;
  `install/spark_requirements.txt`). `bin/adt4-console` re-execs into it when `$ADT4_ENV` is set.
* spark_dsg, heracles and `hydra_python.image_store` come from the colcon workspace (sourced
  `install/setup.zsh`). heracles is only needed for Neo4j sources; newer heracles APIs (version
  counter, mesh path, image ingest) are optional and their features turn off with one log line
  when missing (`heracles_compat.py`).

CLI (`adt4-console`):
```
adt4-console [--host 0.0.0.0] [--port 8090] [--token TOKEN | env ADT4_CONSOLE_TOKEN]
             [--neo4j NAME=URI ...]      # Neo4j sources; creds HERACLES_NEO4J_USERNAME/PASSWORD,
                                         #   else ADT4_NEO4J_USERNAME/PASSWORD, else neo4j/neo4j_pw
             [--live-neo4j NAME]         # the --neo4j target maps are loaded into (default: first)
             [--map NAME=RUN_DIR ...]    # file sources (a run dir with hydra/backend/dsg.json)
             [--output-root ~/adt4_output]                    # runs listed on the Maps page
             [--geo-anchor SRC=FILE ...] [--tiles-dir ~/.adt4_console/tiles] [--tiles-live]
             [--log-level info]
```
Caches: `~/.adt4_console/mesh_cache` (mesh tiles), `~/.adt4_console/place_keyframes`,
`~/.cache/adt4_console/map_sizes.json` (run sizes).

## 1. Conventions

* JSON for REST unless noted. Binary graph/mesh payloads are **msgpack** with
  `Content-Type: application/x-msgpack` (HTTP gzip handles compression).
* Auth: if a token is configured every `/api/*` request needs `Authorization: Bearer <token>`
  or `?token=<token>` (used by `<img>`/WebSocket). 401 otherwise (WebSocket: close 4401).
  Static files are public.
* Errors: `{"detail": "..."}` with 4xx/5xx. Unknown source/node → 404; a source that cannot be
  served right now (Neo4j down, spark_dsg missing) → 503.
* Graph source ids (`{src}`) are path segments: `neo4j:<name>`, `file:<name>`.

## 2. WebSocket `/api/ws`

All frames are **binary msgpack** objects `{topic: str, seq: int, data: any}`. The client may
send `{"sub": ["job/", "graph/neo4j:live", ...]}` (topic prefixes; default = everything except
`graph/*/delta`). The server keeps a per-client bounded queue and drops the *oldest* frame of
the same topic when full; latest-value topics (`jobs`, `job/…`, `…/version`) keep only the
newest frame and are replayed to a client when it subscribes.

| topic | data |
|---|---|
| `jobs` | list of job summaries |
| `job/<id>` | full job object (§4) on every change |
| `maps/active` | the live map status (§5) at every load step |
| `graph/<src>/version` | `{version, reason}` → clients refetch snapshot/manifest. With an incremental Neo4j update also `{base, delta}`: `delta` is `codec.GraphDelta.encode()` turning version `base` into `version`; a viewer holding exactly `base` applies it in place, any other refetches |

## 3. Health

* `GET /api/health` → `{ok, version, neo4j: {name: bool}}` (whether each Neo4j source answers).

## 4. Jobs

Map loads run as jobs.

```
job = {id, kind, title, state: "running"|"waiting"|"succeeded"|"failed"|"cancelled",
       created, finished, progress: {key: pct}, log: [{t, level, text}],
       confirm: null | {kind, message, required_text, yes_label, no_label, detail},
       result: any}
```
* `GET /api/jobs` (summaries: no log), `GET /api/jobs/{id}`
* `POST /api/jobs/{id}/confirm` `{answer: bool, text?: str}`: if `required_text` is set the
  answer counts as yes only if `text == required_text`. Unanswered confirms time out after
  300 s (→ `None`). 409 when nothing is pending.
* `POST /api/jobs/{id}/cancel` (best effort; marks cancelled, a pending confirm is answered `None`)
* level ∈ heading|info|dim|ok|warn|error

## 5. Maps

* `GET /api/maps[?local=true]` → `{machines: {"local": [RUN] | {error}}, pending: ["local"]?, base: "local"}`.
  The runs under `--output-root` plus the run dirs of `--map` / registered sources, read from the
  filesystem (`run_scan.py`, read-only: the store index is opened `mode=ro`). `local` is accepted
  for compatibility; only this machine is listed.
  `RUN` = `{name, path, subdirs, is_prior, mtime, ctime, name_date, size, size_bytes, size_pending,
  signature, has_dsg, has_dsg_mesh, has_mesh, dsg_mtime?, has_image_store, image_store, run_info?, error?}`:
  * `mtime`/`ctime` are the run dir's epoch seconds; `name_date` a date parsed from the name
    (`20261018`, `2026-10-18`, `03192026` MMDDYYYY, `2026-07`) or null. `is_prior`: `hydra/` and
    `roman/` were both saved. `run_info` = `{experiment, robot, run_id}` from `run_info.json`.
  * `has_dsg`/`has_dsg_mesh`/`has_mesh` are `hydra/backend/{dsg.json, dsg_with_mesh.json, mesh.ply}`.
  * `image_store` is null without `<run>/image_store/store.sqlite` and without raw image logs,
    else `{indexed, raw?, frames: {keyframe, object}, frames_total, observations, sensors,
    schema_version, keyframes_unanchored, sampled_files, size_bytes, check: {ok, problems: [str]}}`.
    `raw` = `{keyframe_archives, tracks}` (Hydra/Khronos image logs); a run with raw logs and no
    store yet has `indexed: false` (the console indexes it when the map is opened, as
    `hydra image-store index` would). The quick check passes when the index opens, has a
    `schema_version`, no frame lacks its sensor or has an unknown kind, every keyframe is anchored
    and the image files of the first and last 4 frames exist.
  * Sizes come from one `du -b -d1 <run>`, cached per run keyed by `signature` (newest mtime of the
    run dir, its subdirs, `hydra/backend` and the store index). Uncached runs are measured within a
    3 s budget; the rest come back `size_pending: true` with `pending: ["local"]` and are measured
    in the background: the page re-lists while `pending` is non-empty.
* `POST /api/maps/register` `{name, path}` → adds a `file:<name>` source at runtime (400 without a
  scene graph); returns the source info (§7).
* **Live map** (`map_loader.py`). Neo4j maps are loaded into the *live* target (`--live-neo4j
  NAME`, default the first `--neo4j`).
  * `GET /api/maps/active[?target=NAME]` → `{target, uri, run, name, dsg, nodes, has_images,
    has_features, ready, job_id, stages: [{id, label, state, pct, detail}], error?}`; the same
    object is pushed on `maps/active` at every step.
  * `POST /api/maps/activate {path, target?, force?}` (a run dir or dsg file on this machine) →
    `{job_id, ...status}`, or `{already: true, ...status}` when the target already holds that dsg
    (unless `force`); 409 `{detail: {detail, job_id}}` while another load runs; 400 for a path
    without a scene graph; 404 for an unknown target.
  * The job asks before wiping only when the target has objects added by other tools or map edits
    (`detail: {added_objects, map_edits, nodes}`), then loads in stages, each 0-100 on the job's
    `progress` and on `maps/active`: `graph` (wipe; agents, objects, places, mesh places, rooms,
    buildings, edges, labelspaces, mesh path, source; version bump → **`ready: true`**), then with
    an image store only `images` (links the store in `_GraphMetadata.image_store_paths`, and with a
    heracles that can: keyframe images, Observations, SubKeyframes, CameraCalibs, observation →
    frame links), `features` (place / room SigLIP2 features from
    `hydra/backend/semantic_features.npz`), `keyframes` (the place × keyframe index). Stages a run
    lacks are `skipped`. Each stage ends with a version bump.

## 6. Geo: GPS anchor and satellite tiles

Maps are drawn in their own map frame. A **GPS anchor** ties a map to the world: a 2-D rigid
transform `utm = R(yaw) · [x, y] + t` (EPSG UTM zone, no scale), fitted from the robot's GPS fixes
against the Hydra trajectory by `adt4_console/scripts/geo/anchor_from_gps.py`, which writes
`geo_anchor.json` next to the run's `hydra/` (or its parent): `{epsg, yaw, t, z0, rms, n, source,
lonlat_center}`. `z0` is a rough ground height in the map frame.

* Anchor lookup: an explicit `--geo-anchor SRC=FILE` (`SRC` a source id or name; `*` = every
  source) wins; otherwise the source's run dir (file sources) or the run dir of the Neo4j map.
* `GET /api/geo/anchor` → `{anchor}`, the first source that has one.
* `GET /api/geo/anchor/{src}?cx=&cy=` → `{src, anchor, affine}`, or `{anchor: null}`. `affine` is
  a least-squares affine fit around `(cx, cy)` (±300 m): `map_to_lonlat` and `lonlat_to_map` as
  2×3 rows plus `max_err_m`; the browser needs no projection library. 503 without pyproj.
* `GET /api/geo/layers` → `{layers: [{id, attribution, zmin, zmax, bbox_lonlat, cached, live, default}]}`
  from each layer's `tiles.json`. A manifest may name its source `url` (XYZ template); `default:
  true` makes it the layer drawn under the map (otherwise Esri, else the first).
* `GET /api/geo/tiles/{layer}/{z}/{x}/{y}` → the tile from `--tiles-dir`
  (`<dir>/<layer>/{z}/{x}/{y}.jpg`, written by `adt4_console/scripts/geo/fetch_tiles.py`). On a
  miss, with `--tiles-live`, it is fetched from the layer's URL and kept (a failed URL is not
  retried for 60 s); without it a miss is a 404. Offline is the default.

## 7. Graph sources

All served in the `codec.py` formats.

* `GET /api/graph/sources` → `[{id, kind: "neo4j"|"file", label, version, nodes}]`
* `GET /api/graph/{src}/snapshot` → msgpack `encode_graph(...)` + `{"version": int, "src": str}`.
  Pre-gzipped once per version, with an `ETag` (`If-None-Match` → 304) and `X-Uncompressed-Length`.
* `GET /api/graph/{src}/stats` →
  ```
  {layers: [{key: "2/0", name: "objects", kind: "objects", nodes}],   # sorted by layer, partition
   nodes, edges: {"objects-places": n, ...}, edges_total,
   mesh: {vertices, faces, tiles}, labels: [{label, name, count, layer}],   # top 20 object labels
   version, src}
  ```
  Edge kinds are the two layer *kinds* sorted and joined with `-` (`agents-agents` is the trajectory).
* `GET /api/graph/{src}/mesh/manifest` → JSON (empty `tiles` without a mesh):
  ```
  {tile_size, enc: 2,
   lods: [{level: 0, cell: 0, name: "full"}, {level: 1, cell: 0.2, name: "medium"}, {level: 2, cell: 0.4, name: "coarse"}],
   tiles: {key: {hash, nv, nf, origin, lod: [[bytes, etag] | null, ...]}},   # one entry per `lods` item
   bundles: [{level: 2, bytes, etag}]}
  ```
  `lod[i]` gives tile `key` at level `lods[i].level`; `bytes` is the gzipped size on the wire;
  `null` = no faces at that level. Levels 1 and 2 are vertex clustering on a 0.2 / 0.4 m grid over
  **all** tiles at once (`codec.decimate_tiles`): no cracks, and every tile keeps its key.
* `GET /api/graph/{src}/mesh/tile/{key}` → msgpack tile dict (`codec.tile_mesh` entry, full level).
* `GET /api/graph/{src}/mesh/tile/{key}?enc=2&lod=L&h=ETAG` → msgpack **packed tile**
  (`codec.pack_tile`; browser `codec.ts decodePackedTile`), pre-gzipped. With `h` equal to the
  tile's current etag the response is `immutable`, otherwise `no-cache`. `lod` without `enc=2` →
  422; unknown level or empty tile → 404.
* `GET /api/graph/{src}/mesh/bundle/{level}?h=ETAG` → msgpack array of every packed tile of `level`.
* Tiles, levels and their gzipped blobs are computed once per mesh file and cached in memory and
  on disk (`~/.adt4_console/mesh_cache`, keyed by mesh path + size + mtime, newest 6 kept). At
  startup file sources (snapshot + tiles) and Neo4j meshes are warmed in the background.
* `GET /api/graph/{src}/node/{sym}` → `{sym, layer, part, label, label_name, name, pos, bbox,
  attributes, images: [url], n_images, masks: [bool], image_info: [{kind, own, dist_m, yaw_deg,
  frame_id, anchor}]}`. Object images are the observations (kept ones, best score first); an agent
  node's images are its own keyframe, else the nearest keyframes by pose.
* `GET /api/graph/{src}/imaged` → `{src, syms}`: nodes with object images (search thumbnails).
* `GET /api/graph/{src}/image/{sym}/{i}[?w=&mask=1&crop=1]` → JPEG: the i-th image of the node,
  optionally downscaled to width `w`, with the segmentation mask overlaid, or cropped to the
  object (mask / 2-D box). Variants are cached in memory.
* `GET /api/graph/{src}/frame/{frame_id}[?w=]` → JPEG of one of the map's keyframes.
* `GET /api/graph/{src}/near?x&y[&yaw][&n=6]` → `{src, frames: [{image, kind, dist_m, yaw_deg,
  frame_id, anchor}]}`: the keyframes taken closest to map point (x, y) (heading `yaw` preferred);
  `image` = `GET /api/graph/{src}/near/image?…&i=k[&w=]`.
* `GET /api/graph/{src}/trajectory` → `{points: [[t_s, x, y, z], ...], breaks: [i, ...], robots:
  [{key, start, session?, robot?, bag?}, ...]}` from the agents layer, ordered by robot then time.
  `breaks` start a new polyline (another robot, or a step over 4 m). Pre-gzipped JSON with `ETag`.
* `GET /api/graph/{src}/place/{sym}/keyframes?sort=newest|oldest|visible` → every keyframe that
  observed the traversability place `sym` (MESH_PLACES, 3p1; another node → 404, another `sort` → 422):
  ```
  {src, sym, version, sort, n, keyframes_total, table: "ready"|"building", seconds,
   keyframes: [{frame_id, agent, robot_key, t_ns, visible, dist_m, image}],
   sessions: [{key, session, robot, bag, bag_short, n, first_ns, last_ns, date}]}
  ```
  `visible` is the fraction of the place's 9 sample points the keyframe saw (centre + 8 points at
  0.6 of the boundary rays, projected with the anchor agent pose ∘ `body_T_sensor`; seen when
  z > 0.3 m, range < 12 m, inside the image and not occluded in the depth image; without depth only
  within 5 m). `image` = `GET /api/graph/{src}/frame/{frame_id}`. The first query of a graph version
  indexes the keyframes and builds the full place × keyframe table in a background thread
  (`table: "building"` meanwhile: that place alone is tested); the table is kept in
  `~/.adt4_console/place_keyframes/`. A map without image store answers `n: 0`.

Place boundaries: graph records carry the spark_dsg `TravNodeAttributes` ray boundary of
traversability ("mesh") places as three optional keys, present only when some record has one:
`nrad` (uint16[N], rays per node, 0 = none), `radii` (uint16[sum], cm, concatenated in node
order) and `states` (uint8[sum], `TraversabilityState`: 0 unknown, 1 traversable, 2
intraversable, 3 traversed). Ray i of a node points at angle i/N·2π from `pos`.

Sources:
* **File** (`--map`, `/api/maps/register`): `spark_dsg.DynamicSceneGraph.load` of
  `hydra/backend/dsg.json`; mesh from `hydra/backend/mesh.ply` (else the graph mesh); images from
  `<run>/image_store` via `hydra_python.image_store` (a run with only raw image logs is indexed on
  first use). Version bumps when the DSG file's mtime changes.
* **Neo4j** (`--neo4j`): `heracles.graph_interface.db_to_spark_dsg(db)` plus the `Agent` rows; mesh
  from `read_mesh_path`; images from `Observation` rows, else the stores named by
  `_GraphMetadata.image_store_paths`. Polled every 2 s (`_State.version` via `read_version`, and
  the node count); a change pushes `graph/neo4j:<n>/version`. A write that tells the console which
  nodes it touched is folded into the loaded graph (`GraphService.apply_change`) and pushed as a
  delta instead of a full reload.
