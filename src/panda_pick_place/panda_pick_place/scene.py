"""
Random scene generation for the colour pick-and-place demo.

Every call to `generate_scene()` produces a new layout of the three coloured
boxes and the obstacles on the table, and `write_scene()` writes:

  * a Gazebo world (the base table world with the objects inserted), and
  * a JSON description of the same scene, which the pick-and-place node uses
    to add obstacles to the MoveIt planning scene.

`relayout_scene()` + `apply_scene_to_gazebo()` shuffle the layout of a
simulation that is already running ("New random layout" in the web UI).

All coordinates are in the Gazebo/URDF `world` frame.
"""

import json
import math
import os
import random
import subprocess

# Robot base (panda_link0) position in the world, see panda_description/urdf/arm.xacro
ROBOT_BASE = (0.6, 0.0, 0.35)

# Height of the coffee table top surface (table pose z + mesh height)
TABLE_TOP_Z = 0.3657

# Table area covered by the four coffee tables, used for the MoveIt collision object
TABLE_MIN_XY = (-0.66, -0.33)
TABLE_MAX_XY = (1.93, 0.98)

BOX_SIZE = (0.05, 0.05, 0.08)
BOX_MASS = 0.08

COLORS = {
    "R": {"name": "red", "rgba": (0.9, 0.05, 0.05, 1.0)},
    "G": {"name": "green", "rgba": (0.05, 0.8, 0.05, 1.0)},
    "B": {"name": "blue", "rgba": (0.05, 0.05, 0.9, 1.0)},
}

# Region on the table, in front of the robot and under the camera, where
# boxes may be spawned (also limited by the reachable ring below).
PICK_X_RANGE = (0.20, 1.00)
PICK_Y_RANGE = (0.32, 0.78)
PICK_RADIUS_RANGE = (0.38, 0.68)

# Open-top crate the selected box is dropped into (behind/left of the robot).
CRATE = {
    "position": (0.22, -0.14),
    "inner_size": 0.20,
    "wall_thickness": 0.012,
    "height": 0.12,
}

# Obstacle templates: (name, size x/y/z, rgba). Neutral grey colours so the
# colour detector never mistakes them for a target box.
OBSTACLE_TYPES = [
    ("wall", (0.03, 0.22, 0.24), (0.55, 0.55, 0.55, 1.0)),
    ("pillar", (0.06, 0.06, 0.36), (0.25, 0.25, 0.25, 1.0)),
    ("block", (0.14, 0.08, 0.14), (0.85, 0.85, 0.85, 1.0)),
    ("tower", (0.09, 0.09, 0.30), (0.40, 0.40, 0.42, 1.0)),
    ("slab", (0.20, 0.12, 0.06), (0.70, 0.70, 0.72, 1.0)),
]
DEFAULT_NUM_OBSTACLES = 6

# Obstacles may be placed anywhere in this ring around the robot base, except
# near the crate, so they end up between the robot, the boxes and the crate.
OBSTACLE_RADIUS_RANGE = (0.30, 0.66)
OBSTACLE_ANGLE_RANGE = (-0.35, math.pi + 0.35)  # world yaw around the base

BOX_MIN_SEPARATION = 0.16
BOX_OBSTACLE_CLEARANCE = 0.10
OBSTACLE_MIN_SEPARATION = 0.10
CRATE_CLEARANCE = 0.12


def _footprint_radius(size):
    return 0.5 * math.hypot(size[0], size[1])


def _dist_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _sample_box_position(rng):
    while True:
        x = rng.uniform(*PICK_X_RANGE)
        y = rng.uniform(*PICK_Y_RANGE)
        r = _dist_xy((x, y), ROBOT_BASE)
        if PICK_RADIUS_RANGE[0] <= r <= PICK_RADIUS_RANGE[1]:
            return x, y


def _crate_radius():
    return 0.5 * math.sqrt(2) * (CRATE["inner_size"] + 2 * CRATE["wall_thickness"])


def _obstacle_templates(num_obstacles, rng):
    """Name, size and colour of each obstacle (cycling through the types)."""
    kinds = [OBSTACLE_TYPES[i % len(OBSTACLE_TYPES)] for i in range(num_obstacles)]
    rng.shuffle(kinds)
    return [{"name": f"obstacle_{i}_{kind}", "size": list(size), "rgba": list(rgba)}
            for i, (kind, size, rgba) in enumerate(kinds)]


def _sample_layout(rng, templates):
    boxes = {}
    for color_id in COLORS:
        for _ in range(500):
            x, y = _sample_box_position(rng)
            if all(_dist_xy((x, y), b["position"]) >= BOX_MIN_SEPARATION for b in boxes.values()):
                break
        else:
            return None
        boxes[color_id] = {
            "name": f"{COLORS[color_id]['name']}_box",
            "color": COLORS[color_id]["name"],
            "position": [x, y, TABLE_TOP_Z + BOX_SIZE[2] / 2],
            "yaw": rng.uniform(-math.pi / 4, math.pi / 4),
            "size": list(BOX_SIZE),
        }

    obstacles = []
    box_radius = _footprint_radius(BOX_SIZE)
    for index, template in enumerate(templates):
        size = template["size"]
        radius = _footprint_radius(size)
        for attempt in range(500):
            # Half of the obstacles go on the way from the robot to a box, the
            # rest anywhere around the robot
            if index % 2 == 0 and attempt < 300:
                # Prefer spots on the way from the robot to one of the boxes
                target = rng.choice(list(boxes.values()))["position"]
                angle = math.atan2(target[1] - ROBOT_BASE[1], target[0] - ROBOT_BASE[0])
                angle += rng.uniform(-0.25, 0.25)
                dist = rng.uniform(0.35, 0.65) * _dist_xy(target, ROBOT_BASE)
                dist = max(dist, OBSTACLE_RADIUS_RANGE[0])
            else:
                angle = rng.uniform(*OBSTACLE_ANGLE_RANGE)
                dist = rng.uniform(*OBSTACLE_RADIUS_RANGE)
            x = ROBOT_BASE[0] + dist * math.cos(angle)
            y = ROBOT_BASE[1] + dist * math.sin(angle)
            if not (TABLE_MIN_XY[0] + radius < x < TABLE_MAX_XY[0] - radius
                    and TABLE_MIN_XY[1] + radius < y < TABLE_MAX_XY[1] - radius):
                continue
            if any(_dist_xy((x, y), b["position"]) < radius + box_radius + BOX_OBSTACLE_CLEARANCE
                   for b in boxes.values()):
                continue
            if any(_dist_xy((x, y), o["position"]) < radius + _footprint_radius(o["size"]) + OBSTACLE_MIN_SEPARATION
                   for o in obstacles):
                continue
            if _dist_xy((x, y), CRATE["position"]) < radius + _crate_radius() + CRATE_CLEARANCE:
                continue
            break
        else:
            return None
        obstacles.append({
            "name": template["name"],
            "position": [x, y, TABLE_TOP_Z + size[2] / 2],
            "yaw": rng.uniform(-math.pi, math.pi),
            "size": list(size),
            "rgba": list(template["rgba"]),
        })

    return boxes, obstacles


def _crate_parts():
    """Bottom plate and four walls of the drop crate, as (name, center, size)."""
    cx, cy = CRATE["position"]
    inner, t, h = CRATE["inner_size"], CRATE["wall_thickness"], CRATE["height"]
    outer = inner + 2 * t
    z0 = TABLE_TOP_Z
    off = inner / 2 + t / 2
    return [
        ("bottom", (cx, cy, z0 + t / 2), (outer, outer, t)),
        ("wall_px", (cx + off, cy, z0 + h / 2), (t, outer, h)),
        ("wall_nx", (cx - off, cy, z0 + h / 2), (t, outer, h)),
        ("wall_py", (cx, cy + off, z0 + h / 2), (inner, t, h)),
        ("wall_ny", (cx, cy - off, z0 + h / 2), (inner, t, h)),
    ]


def generate_scene(seed=None, num_obstacles=DEFAULT_NUM_OBSTACLES):
    """Return a random, collision-free scene description (dict)."""
    rng = random.Random(seed)
    return _scene_from_templates(rng, seed, _obstacle_templates(num_obstacles, rng))


def relayout_scene(scene, rng=None):
    """New random positions for the boxes and for the same obstacles as `scene`."""
    templates = [{"name": o["name"], "size": o["size"], "rgba": o["rgba"]} for o in scene["obstacles"]]
    return _scene_from_templates(rng or random.Random(), None, templates)


def _scene_from_templates(rng, seed, templates):
    for _ in range(200):
        layout = _sample_layout(rng, templates)
        if layout is not None:
            boxes, obstacles = layout
            break
    else:
        raise RuntimeError(f"Could not place {len(templates)} obstacles; try fewer (num_obstacles)")

    cx, cy = CRATE["position"]
    return {
        "seed": seed,
        "frame_id": "world",
        "table": {
            "top_z": TABLE_TOP_Z,
            "min_xy": list(TABLE_MIN_XY),
            "max_xy": list(TABLE_MAX_XY),
        },
        "boxes": boxes,
        "obstacles": obstacles,
        "crate": {
            "name": "drop_crate",
            "position": [cx, cy, TABLE_TOP_Z],
            "inner_size": CRATE["inner_size"],
            "height": CRATE["height"],
            "floor_thickness": CRATE["wall_thickness"],
            "parts": [
                {"name": f"drop_crate_{name}", "position": list(center), "size": list(size)}
                for name, center, size in _crate_parts()
            ],
        },
    }


# ----------------------------------------------------------------------------
# SDF generation
# ----------------------------------------------------------------------------

def _rgba(rgba):
    return " ".join(f"{c:.3f}" for c in rgba)


def _box_inertia(mass, size):
    x, y, z = size
    return (mass / 12.0 * (y * y + z * z),
            mass / 12.0 * (x * x + z * z),
            mass / 12.0 * (x * x + y * y))


def _box_model_sdf(box, rgba):
    x, y, z = box["position"]
    sx, sy, sz = box["size"]
    ixx, iyy, izz = _box_inertia(BOX_MASS, box["size"])
    return f"""
    <model name='{box["name"]}'>
      <pose>{x:.4f} {y:.4f} {z + 0.002:.4f} 0 0 {box["yaw"]:.4f}</pose>
      <link name='link'>
        <inertial>
          <mass>{BOX_MASS}</mass>
          <inertia><ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy><izz>{izz:.6g}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name='collision'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface>
            <friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction>
            <contact><ode><kp>1e6</kp><kd>100</kd><min_depth>0.001</min_depth><max_vel>0.1</max_vel></ode></contact>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{_rgba(rgba)}</ambient>
            <diffuse>{_rgba(rgba)}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def _static_box_model_sdf(name, position, size, rgba, yaw=0.0):
    x, y, z = position
    sx, sy, sz = size
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{_rgba(rgba)}</ambient>
            <diffuse>{_rgba(rgba)}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


# Drop crate: yellow walls with a black rim and a dark floor, so it cannot be
# mistaken for a box. Yellow (OpenCV hue ~24) lies between the detector's red
# (<= 8) and green (>= 45) ranges.
CRATE_WALL_RGBA = (1.0, 0.78, 0.0, 1.0)
CRATE_RIM_RGBA = (0.08, 0.08, 0.08, 1.0)
CRATE_FLOOR_RGBA = (0.2, 0.2, 0.22, 1.0)
CRATE_RIM_HEIGHT = 0.02


def _material_sdf(rgba):
    return f"""<material>
            <ambient>{_rgba(rgba)}</ambient>
            <diffuse>{_rgba(rgba)}</diffuse>
          </material>"""


def _crate_model_sdf(crate):
    links = []
    for part in crate["parts"]:
        x, y, z = part["position"]
        sx, sy, sz = part["size"]
        if part["name"].endswith("_bottom"):
            visuals = f"""
        <visual name='visual'>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          {_material_sdf(CRATE_FLOOR_RGBA)}
        </visual>"""
        else:
            # Yellow wall with a black stripe along its top edge
            wall_h = sz - CRATE_RIM_HEIGHT
            visuals = f"""
        <visual name='visual'>
          <pose>0 0 {-CRATE_RIM_HEIGHT / 2:.4f} 0 0 0</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {wall_h:.4f}</size></box></geometry>
          {_material_sdf(CRATE_WALL_RGBA)}
        </visual>
        <visual name='rim'>
          <pose>0 0 {wall_h / 2:.4f} 0 0 0</pose>
          <geometry><box><size>{sx + 0.002:.4f} {sy + 0.002:.4f} {CRATE_RIM_HEIGHT:.4f}</size></box></geometry>
          {_material_sdf(CRATE_RIM_RGBA)}
        </visual>"""
        links.append(f"""
      <link name='{part["name"]}'>
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <collision name='collision'>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>{visuals}
      </link>""")
    return f"""
    <model name='{crate["name"]}'>
      <static>true</static>{"".join(links)}
    </model>
"""


def scene_to_sdf_objects(scene):
    parts = []
    for color_id, box in scene["boxes"].items():
        parts.append(_box_model_sdf(box, COLORS[color_id]["rgba"]))
    for obstacle in scene["obstacles"]:
        parts.append(_static_box_model_sdf(
            obstacle["name"], obstacle["position"], obstacle["size"],
            obstacle["rgba"], obstacle["yaw"]))
    parts.append(_crate_model_sdf(scene["crate"]))
    return "".join(parts)


WORLD_PLACEHOLDER = "<!-- PICK_PLACE_OBJECTS"


def write_scene(base_world_path, output_dir, seed=None, num_obstacles=DEFAULT_NUM_OBSTACLES):
    """Generate a random scene and write `<output_dir>/pick_place.world` and
    `<output_dir>/scene.json`. Returns (world_path, scene_path, scene)."""
    scene = generate_scene(seed=seed, num_obstacles=num_obstacles)

    with open(base_world_path) as f:
        world = f.read()
    index = world.find(WORLD_PLACEHOLDER)
    if index < 0:
        raise RuntimeError(f"{base_world_path} has no '{WORLD_PLACEHOLDER}' marker")
    world = world[:index] + scene_to_sdf_objects(scene) + world[index:]

    os.makedirs(output_dir, exist_ok=True)
    world_path = os.path.join(output_dir, "pick_place.world")
    scene_path = os.path.join(output_dir, "scene.json")
    with open(world_path, "w") as f:
        f.write(world)
    with open(scene_path, "w") as f:
        json.dump(scene, f, indent=2)
    return world_path, scene_path, scene


def load_scene(scene_path):
    with open(scene_path) as f:
        return json.load(f)


def save_scene(scene, scene_path):
    with open(scene_path, "w") as f:
        json.dump(scene, f, indent=2)


# ----------------------------------------------------------------------------
# Changing the layout of a running simulation
# ----------------------------------------------------------------------------

def _gz_service(world, service, reqtype, request, timeout_ms=5000):
    """Call a Gazebo transport service with the `gz service` command line tool."""
    result = subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/{service}",
         "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
         "--timeout", str(timeout_ms), "--req", request],
        capture_output=True, text=True, timeout=timeout_ms / 1000 + 5)
    if "data: true" not in result.stdout:
        raise RuntimeError(f"Gazebo service {service} failed: {result.stdout.strip()} {result.stderr.strip()}")


def apply_scene_to_gazebo(old_scene, new_scene, world="pick_place_world"):
    """Replace the boxes and obstacles of `old_scene` in the running Gazebo
    world with those of `new_scene`.

    Models are removed and created again rather than moved: obstacles are
    static models, whose collision shapes the physics engine does not move.
    """
    for model in list(old_scene["boxes"].values()) + old_scene["obstacles"]:
        _gz_service(world, "remove/blocking", "gz.msgs.Entity", f'name: "{model["name"]}" type: MODEL')

    models = [_box_model_sdf(box, COLORS[color_id]["rgba"]) for color_id, box in new_scene["boxes"].items()]
    models += [_static_box_model_sdf(o["name"], o["position"], o["size"], o["rgba"], o["yaw"])
               for o in new_scene["obstacles"]]
    # One EntityFactory per model; the SDF goes into a protobuf text string, so
    # it must be on one line (it only uses single quotes)
    request = " ".join(
        'data { sdf: "<sdf version=\'1.9\'>%s</sdf>" }' % " ".join(sdf.split()) for sdf in models)
    _gz_service(world, "create_multiple/blocking", "gz.msgs.EntityFactory_V", request)
