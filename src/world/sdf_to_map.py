#!/usr/bin/env python3
# SDF → 2D occupancy map (PNG + YAML), ROS/Nav2 compatible.
# - Planes = road (free/white)
# - Thin/low boxes can be treated as road via heuristics
# - Any shape with top_z > z_thresh = obstacle (black)
# - Cylinders tolerate missing <length>/<height> (optional default via CLI)
# - Boundary selection: all | road | largest_road_rect | named

import argparse, math, os, sys, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np, cv2, yaml

# ----------------------------
# Math helpers
# ----------------------------
def Rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s,  c]], dtype=float)

def parse_pose(txt: str):
    """SDF <pose>: x y z roll pitch yaw  (roll/pitch ignored for 2D)"""
    if not txt:
        return np.array([0.0, 0.0, 0.0]), (0.0, 0.0, 0.0)
    v = [float(x) for x in txt.strip().split()]
    if len(v) == 6:
        x, y, z, rr, pp, yy = v
    elif len(v) == 3:
        x, y, z = v; rr = pp = yy = 0.0
    else:
        raise ValueError(f"Bad pose format: '{txt}'")
    return np.array([x, y, z], dtype=float), (rr, pp, yy)

def rect_corners(sx: float, sy: float) -> np.ndarray:
    hx, hy = sx/2.0, sy/2.0
    return np.array([[-hx,-hy],[hx,-hy],[hx,hy],[-hx,hy]], dtype=float)

def poly_area(pts: np.ndarray) -> float:
    """Polygon area (abs, m^2)."""
    x, y = pts[:,0], pts[:,1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

# ----------------------------
# Safe XML parsing
# ----------------------------
def safe_text(el) -> str:
    return el.text if (el is not None and el.text is not None) else ""

def safe_float(el, default: float = 0.0) -> float:
    try:
        return float(el.text.strip())
    except Exception:
        return default

def cylinder_r_h(cyl_el, ctx: str = "", assume_default_h: float | None = None):
    """Read <radius> and <length>/<height>; tolerate missing tags."""
    r_el = cyl_el.find("radius")
    r = safe_float(r_el, 0.0)

    h_el = cyl_el.find("length")
    if h_el is None:
        h_el = cyl_el.find("height")

    if h_el is None or safe_text(h_el).strip() == "":
        if assume_default_h is not None:
            h = float(assume_default_h)
            print(f"[WARN] Cylinder has no <length>/<height>; using default {h} m. {ctx}", file=sys.stderr)
        else:
            h = 0.0
            print(f"[WARN] Cylinder has no <length>/<height> (treating as 0). {ctx}", file=sys.stderr)
    else:
        h = float(h_el.text.strip())

    return r, h

# ----------------------------
# Parse SDF → shapes
# ----------------------------
def collect_shapes(root, assume_cyl_h: float | None = None):
    """
    Return list of shape dicts:
      - type: "plane" | "rect" | "circle"
      - points (Nx2) for plane/rect, or center/radius for circle
      - z_bot, z_top, thickness (for rect/circle), name (model/link/collision)
    Yaw-only assumption for 2D projection (no roll/pitch).
    """
    ns = {"s": root.tag.split('}')[0].strip('{')} if '}' in root.tag else {}
    models = list(root.findall(".//model", ns))
    if not models and root.tag.endswith("model"):
        models = [root]

    out = []
    for m in models:
        m_pose_el = m.find("pose", ns)
        m_pose = safe_text(m_pose_el)
        m_t, (_, _, m_yaw) = parse_pose(m_pose)

        for link in m.findall("link", ns):
            l_pose_el = link.find("pose", ns)
            l_pose = safe_text(l_pose_el)
            l_t, (_, _, l_yaw) = parse_pose(l_pose)

            yaw_ml = m_yaw + l_yaw
            t_ml = m_t[:2] + Rz(m_yaw) @ l_t[:2]
            z_ml = m_t[2] + l_t[2]

            for col in link.findall("collision", ns):
                c_pose_el = col.find("pose", ns)
                c_pose = safe_text(c_pose_el)
                c_t, (_, _, c_yaw) = parse_pose(c_pose)

                yaw = yaw_ml + c_yaw
                t_xy = t_ml + Rz(yaw_ml) @ c_t[:2]
                z_center = z_ml + c_t[2]

                g = col.find("geometry", ns)
                if g is None:
                    continue

                name = f"{m.get('name','?')}/{link.get('name','?')}/{col.get('name','?')}"

                # Plane (road)
                pl = g.find("plane", ns)
                if pl is not None:
                    sz_el = pl.find("size", ns)
                    if sz_el is None:
                        continue
                    sx, sy = [float(x) for x in safe_text(sz_el).split()]
                    pts = rect_corners(sx, sy)
                    pts = (Rz(yaw) @ pts.T).T + t_xy
                    out.append({"type":"plane","points":pts,"z_bot":z_center,"z_top":z_center,
                                "thickness":0.0,"name":name})
                    continue

                # Box
                bx = g.find("box", ns)
                if bx is not None:
                    sz_el = bx.find("size", ns)
                    if sz_el is None:
                        continue
                    sx, sy, sz = [float(x) for x in safe_text(sz_el).split()]
                    pts = rect_corners(sx, sy)
                    pts = (Rz(yaw) @ pts.T).T + t_xy
                    out.append({"type":"rect","points":pts,
                                "z_bot":z_center - sz/2.0, "z_top":z_center + sz/2.0,
                                "thickness":sz,"name":name})
                    continue

                # Cylinder
                cy = g.find("cylinder", ns)
                if cy is not None:
                    r, h = cylinder_r_h(cy, f"({name})", assume_default_h=assume_cyl_h)
                    out.append({"type":"circle","center":(float(t_xy[0]), float(t_xy[1])),
                                "radius":r, "z_bot":z_center - h/2.0, "z_top":z_center + h/2.0,
                                "thickness":h,"name":name})
                    continue

                # Mesh → skip (warn)
                if g.find("mesh", ns) is not None:
                    print(f"[WARN] Skipping mesh at {name}", file=sys.stderr)

    return out

# ----------------------------
# Bounds & coordinate mapping
# ----------------------------
def shape_bounds(s):
    """Return (xmin, xmax, ymin, ymax) for a shape dict."""
    if s["type"] in ("plane", "rect"):
        xs = s["points"][:,0]; ys = s["points"][:,1]
        return float(xs.min()), float(xs.max()), float(ys.min()), float(ys.max())
    elif s["type"] == "circle":
        cx, cy = s["center"]; r = s["radius"]
        return cx - r, cx + r, cy - r, cy + r
    return 0.0, 0.0, 0.0, 0.0

def bounds_from_subset(shapes_subset):
    xs, ys = [], []
    for s in shapes_subset:
        x0, x1, y0, y1 = shape_bounds(s)
        xs += [x0, x1]; ys += [y0, y1]
    if not xs:
        # Fallback tiny box to avoid crash
        return 0.0, 1.0, 0.0, 1.0
    return min(xs), max(xs), min(ys), max(ys)

def w2i(pts, res, x0, y0, H):
    u = (pts[:,0] - x0) / res
    v = H - 1 - (pts[:,1] - y0) / res
    return np.stack([u, v], axis=1).astype(np.int32)

# ----------------------------
# Generation with road heuristics & boundary control
# ----------------------------
def generate_map(
    sdf_path: str,
    out_prefix: str = "map",
    res: float = 0.05,
    pad: float = 1.0,
    z_thresh: float = 0.1,
    bg_mode: str = "free",               # "free" or "occupied"
    road_box_max_z: float = 0.1,        # boxes with top_z <= this → road
    road_box_max_thickness: float = 0.1,# thin boxes <= this → road
    road_name_contains: str = "",        # comma list; if given, name contains → road
    min_road_area: float = 1.0,          # ignore tiny patches
    assume_cyl_h: float | None = None,   # default height for cylinders missing length/height
    boundary: str = "all",               # "all" | "road" | "largest_road_rect" | "named"
    boundary_name_contains: str = ""     # tokens for "named"
):
    root = ET.parse(sdf_path).getroot()
    shp  = collect_shapes(root, assume_cyl_h=assume_cyl_h)
    if not shp:
        raise SystemExit("No supported shapes found (plane/box/cylinder).")

    # --- road classifier (heuristics + optional name filter) ---
    road_tokens = [t.strip().lower() for t in road_name_contains.split(",") if t.strip()]
    def looks_like_road(s) -> bool:
        nm = s.get("name","").lower()
        by_name = (not road_tokens) or any(t in nm for t in road_tokens)
        if s["type"] == "plane":
            return True if not road_tokens else by_name
        if s["type"] == "rect":
            is_thin = s.get("thickness", 1e9) <= road_box_max_thickness
            low_top = s["z_top"] <= road_box_max_z
            area_ok = poly_area(s["points"]) >= min_road_area
            return (is_thin and low_top and area_ok) and (True if not road_tokens else by_name)
        return False

    # --- boundary selection ---
    b_tokens = [t.strip().lower() for t in boundary_name_contains.split(",") if t.strip()]
    def is_named(s):
        nm = s.get("name","").lower()
        return any(t in nm for t in b_tokens)

    if boundary == "named" and b_tokens:
        subset = [s for s in shp if is_named(s)]
        if not subset:
            print("[WARN] boundary=named but no shapes matched; falling back to all.", file=sys.stderr)
            subset = shp
    elif boundary == "road":
        subset = [s for s in shp if looks_like_road(s)]
        if not subset:
            print("[WARN] boundary=road found no road-like shapes; falling back to all.", file=sys.stderr)
            subset = shp
    elif boundary == "largest_road_rect":
        road_rects = [s for s in shp if s["type"] == "rect" and looks_like_road(s)]
        if road_rects:
            subset = [max(road_rects, key=lambda s: poly_area(s["points"]))]
        else:
            print("[WARN] boundary=largest_road_rect found no road rectangles; falling back to all.", file=sys.stderr)
            subset = shp
    else:
        subset = shp

    x0, x1, y0, y1 = bounds_from_subset(subset)
    x0 -= pad; y0 -= pad; x1 += pad; y1 += pad

    W = max(1, int(math.ceil((x1 - x0) / res)))
    H = max(1, int(math.ceil((y1 - y0) / res)))

    # Background fill
    img = np.full((H,W), 255, np.uint8) if bg_mode == "free" else np.zeros((H,W), np.uint8)

    # 1) Paint ROADS (white)
    for s in shp:
        if looks_like_road(s):
            if s["type"] in ("plane","rect"):
                poly = w2i(s["points"], res, x0, y0, H).reshape((-1,1,2))
                cv2.fillPoly(img, [poly], 255)

    # 2) Paint OBSTACLES (top_z > z_thresh) as black
    for s in shp:
        if s["z_top"] > z_thresh:
            if s["type"] == "rect":
                poly = w2i(s["points"], res, x0, y0, H).reshape((-1,1,2))
                cv2.fillPoly(img, [poly], 0)
            elif s["type"] == "circle":
                cx, cy = s["center"]
                c = w2i(np.array([[cx,cy]]), res, x0, y0, H)[0]
                rp = max(1, int(round(s["radius"]/res)))
                cv2.circle(img, tuple(c), rp, 0, -1)
            # planes are thin; typically not obstacles

    # Save outputs
    png = f"{out_prefix}.png"
    yaml_p = f"{out_prefix}.yaml"
    cv2.imwrite(png, img)
    with open(yaml_p, "w") as f:
        yaml.dump({
            "image": os.path.basename(png),
            "resolution": float(res),
            "origin": [float(x0), float(y0), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25
        }, f, default_flow_style=False)

    print(f"[OK] {png} {yaml_p}  size={W}x{H}px  res={res} m/px  origin=({x0:.3f},{y0:.3f},0)")
    return png, yaml_p

# ----------------------------
# CLI
# ----------------------------
def default_sdf_path():
    cwd  = Path.cwd() / "road.sdf"
    here = Path(__file__).parent / "road.sdf"
    return str(cwd if cwd.exists() else here)

def main():
    ap = argparse.ArgumentParser(description="Convert SDF (world/model) to 2D occupancy map (PNG+YAML).")
    ap.add_argument("sdf", nargs="?", help="Path to SDF (default: road.sdf in CWD or next to script)")
    ap.add_argument("-o","--out", default="map", help="Output prefix (default: map)")
    ap.add_argument("-r","--resolution", type=float, default=0.05, help="Meters per pixel (default 0.05)")
    ap.add_argument("-p","--padding", type=float, default=1.0, help="Padding around geometry in meters (default 1.0)")
    ap.add_argument("--zthresh", type=float, default=0.1, help="Top height > zthresh becomes obstacle (default 0.1)")
    ap.add_argument("--bg", choices=["free","occupied"], default="free", help="Background fill (default free/white)")
    ap.add_argument("--road-box-max-z", type=float, default=0.12, help="Boxes with top_z <= this count as road")
    ap.add_argument("--road-box-max-thickness", type=float, default=0.06, help="Thin boxes (<=) can be road")
    ap.add_argument("--road-name-contains", type=str, default="",
                    help="Comma list; if provided, only names containing tokens are considered road")
    ap.add_argument("--min-road-area", type=float, default=1.0, help="Ignore tiny plane/rect areas (m^2)")
    ap.add_argument("--assume-cylinder-height", type=float, default=None,
                    help="If a cylinder lacks <length>/<height>, assume this height (m)")
    ap.add_argument("--boundary", choices=["all","road","largest_road_rect","named"], default="all",
                    help="Where to take the image bounds from (default: all)")
    ap.add_argument("--boundary-name-contains", type=str, default="",
                    help="Comma tokens for boundary=named (e.g., 'outer_road,road_box')")
    args = ap.parse_args()

    sdf_path = args.sdf or default_sdf_path()
    generate_map(
        sdf_path,
        out_prefix=args.out,
        res=args.resolution,
        pad=args.padding,
        z_thresh=args.zthresh,
        bg_mode=args.bg,
        road_box_max_z=args.road_box_max_z,
        road_box_max_thickness=args.road_box_max_thickness,
        road_name_contains=args.road_name_contains,
        min_road_area=args.min_road_area,
        assume_cyl_h=args.assume_cylinder_height,
        boundary=args.boundary,
        boundary_name_contains=args.boundary_name_contains
    )

if __name__ == "__main__":
    main()
