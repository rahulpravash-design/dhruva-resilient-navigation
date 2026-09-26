"""Build maps/road_graph.json for the demo area from OpenStreetMap with osmnx.

STATUS: NOT RUN. Needs `pip install osmnx` (named in the master spec) and network access to OSM/Overpass, neither of
which was available while this was written; only the JSON schema it targets (dhruva/mapmatch/graph.py) is tested.

usage: python scripts/build_map.py NORTH SOUTH EAST WEST [out.json]
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def maxspeed_kph(value):
    if isinstance(value, list):
        value = value[0]
    try:
        return float(str(value).split()[0])
    except (ValueError, IndexError):
        return 50.0


def main(argv):
    import osmnx as ox  # imported here so the rest of the repo never depends on it

    north, south, east, west = (float(x) for x in argv[1:5])
    out = Path(argv[5]) if len(argv) > 5 else ROOT / "maps" / "road_graph.json"
    graph = ox.graph_from_bbox((west, south, east, north), network_type="drive", simplify=True)
    nodes = {str(n): [d["y"], d["x"]] for n, d in graph.nodes(data=True)}
    edges = []
    for i, (u, v, d) in enumerate(graph.edges(data=True)):
        geom = [[y, x] for x, y in d["geometry"].coords] if "geometry" in d else None
        edge = {"id": f"e{i}", "u": str(u), "v": str(v), "name": str(d.get("name", "")) if not isinstance(
            d.get("name"), list) else str(d["name"][0]), "tunnel": d.get("tunnel", "no") not in ("no", None, False),
            "maxspeed_kph": maxspeed_kph(d.get("maxspeed", 50)), "oneway": bool(d.get("oneway", False))}
        if geom:
            edge["geometry"] = geom
        edges.append(edge)
    lat0 = sum(p[0] for p in nodes.values()) / len(nodes)
    lon0 = sum(p[1] for p in nodes.values()) / len(nodes)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"origin": [lat0, lon0], "nodes": nodes, "edges": edges}))
    print(f"wrote {out}: {len(nodes)} nodes, {len(edges)} edges")


if __name__ == "__main__":
    main(sys.argv)
