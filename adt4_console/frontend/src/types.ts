// Types for the ADT4 console viewer REST/WS contract (adt4_console/docs/API.md).

export interface Health {
  ok: boolean;
  version: string;
  /** per Neo4j source: reachable */
  neo4j: Record<string, boolean>;
}

export type JobState = "running" | "waiting" | "succeeded" | "failed" | "cancelled";
export type LogLevel = "heading" | "info" | "dim" | "ok" | "warn" | "error";

export interface JobConfirm {
  kind: string;
  message: string;
  required_text?: string | null;
  yes_label?: string | null;
  no_label?: string | null;
  detail?: unknown;
}

export interface Job {
  id: string;
  kind: string;
  title: string;
  state: JobState;
  created: number;
  finished?: number | null;
  progress?: Record<string, number>;
  log?: {t: number; level: LogLevel; text: string}[];
  confirm?: JobConfirm | null;
  result?: unknown;
}

export interface ImageStoreInfo {
  /** frame counts by kind: keyframe / subkeyframe / object */
  frames: Partial<Record<"keyframe" | "subkeyframe" | "object", number>> & Record<string, number>;
  frames_total: number;
  observations: number;
  sensors: number;
  schema_version?: string | null;
  keyframes_unanchored?: number;
  /** du of <run>/image_store; null while sizes are pending */
  size_bytes?: number | null;
  check: {ok: boolean; problems: string[]};
}

export interface MapEntry {
  name: string;
  size: string | number;
  subdirs?: string[];
  is_prior?: boolean;
  path?: string;
  abs_path?: string;
  /** epoch seconds */
  mtime?: number | null;
  ctime?: number | null;
  name_date?: number | null;
  dsg_mtime?: number | null;
  size_bytes?: number | null;
  size_pending?: boolean;
  has_dsg?: boolean;
  has_dsg_mesh?: boolean;
  has_mesh?: boolean;
  has_image_store?: boolean;
  image_store?: ImageStoreInfo | null;
  sessions?: number;
  error?: string;
}

/** GET api/maps?local=true: this host's runs under machines[base]. */
export interface MapsResponse {
  machines: Record<string, MapEntry[] | {error: string}>;
  /** hosts whose sizes are still being measured in the background */
  pending?: string[];
  /** this host's key in `machines` */
  base?: string;
}

export interface GraphSource {
  id: string;
  kind: "neo4j" | "file";
  label: string;
  version: number;
  nodes: number;
}

export interface TileManifest {
  tile_size: number;
  /** 2: tiles also served packed (`?enc=2`, codec decodePackedTile) */
  enc?: number;
  /** levels of detail (0 = full; cell = vertex-clustering size in metres) */
  lods?: {level: number; cell: number; name: string}[];
  tiles: Record<
    string,
    {
      hash: string;
      nv: number;
      nf: number;
      origin: [number, number, number];
      /** per `lods` entry: [gzipped bytes, etag], null = no faces at that level */
      lod?: ([number, string] | null)[];
    }
  >;
  /** a whole level in one download (`mesh/bundle/{level}`): the coarse preview */
  bundles?: {level: number; bytes: number; etag: string}[];
}

export interface NodeDetail {
  sym: string;
  layer: number;
  part: number;
  label: number;
  label_name?: string;
  name?: string;
  pos: [number, number, number];
  bbox?: number[] | null;
  attributes?: Record<string, unknown>;
  images?: string[];
}
