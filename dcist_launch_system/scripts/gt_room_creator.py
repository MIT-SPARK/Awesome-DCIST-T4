import matplotlib.pyplot as plt
import numpy as np
import trimesh
import yaml

DEFAULT_Z = 2.5
DEFAULT_HEIGHT = 8.0
DEFAULT_ROTATION = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}


bounding_boxes = {}  # group_id -> list of boxes
clicks = []
current_group = 1


def onclick(event):
    if event.xdata is None or event.ydata is None:
        return
    clicks.append((event.xdata, event.ydata))
    print(f"[Group {current_group}] Click: {event.xdata:.2f}, {event.ydata:.2f}")

    if len(clicks) == 2:
        x1, y1 = clicks[0]
        x2, y2 = clicks[1]
        center = [(x1 + x2) / 2, (y1 + y2) / 2, DEFAULT_Z]
        extents = [abs(x2 - x1), abs(y2 - y1), DEFAULT_HEIGHT]
        print("centers: ", center)
        print(type(round(center[0], 2)))
        box = {
            "center": [float(round(x, 2)) for x in center],
            "extents": [float(round(x, 2)) for x in extents],
            "rotation": DEFAULT_ROTATION.copy(),
        }

        if current_group not in bounding_boxes:
            bounding_boxes[current_group] = []
        bounding_boxes[current_group].append(box)
        print(f"  → Added to group {current_group}: {box}")
        print("bounding_boxes: ", bounding_boxes)

        ax.add_patch(
            plt.Rectangle(
                (min(x1, x2), min(y1, y2)),
                abs(x2 - x1),
                abs(y2 - y1),
                fill=True,
                edgecolor="red",
                linewidth=2,
            )
        )
        ax.text(
            center[0],
            center[1],
            str(current_group),
            color="blue",
            ha="center",
            va="center",
        )
        plt.draw()
        clicks.clear()


def onkey(event):
    global current_group

    if event.key == "enter":
        with open("gt_room_bounding_boxes.yaml", "w") as f:
            yaml.dump(bounding_boxes, f, sort_keys=True)
        print("✅ Saved bounding_boxes.yaml")
        plt.close()
    elif event.key.isdigit() and int(event.key) > 0:
        current_group = int(event.key)
        print(f"🔀 Switched to group {current_group}")


# Load the .ply file
mesh_fn = "/home/aaron/adt4_output/tues_morning_spot_bag_construction_area_2_processed/hydra/backend/mesh.ply"
mesh = trimesh.load(mesh_fn)

# Extract vertex positions (Nx3)
vertices = mesh.vertices

# Choose a projection: XY, XZ, or YZ
proj = "xy"  # change to "xz" or "yz" if desired

if proj == "xy":
    x = vertices[:, 0]
    y = vertices[:, 1]
elif proj == "xz":
    x = vertices[:, 0]
    y = vertices[:, 2]
elif proj == "yz":
    x = vertices[:, 1]
    y = vertices[:, 2]
else:
    raise ValueError("Invalid projection: choose from 'xy', 'xz', 'yz'")

# Plot
fig, ax = plt.subplots()
plt.xlabel(proj[0].upper())
plt.ylabel(proj[1].upper())
plt.title(f"{proj.upper()} Projection of PLY Vertices")


ax.set_title("Click 2 corners per box. Press 1–9 to change group. Enter to finish.")
fig.canvas.mpl_connect("button_press_event", onclick)
fig.canvas.mpl_connect("key_press_event", onkey)


# plt.axis("equal")
# plt.grid(True)

rs = mesh.visual.vertex_colors[:, 0]
gs = mesh.visual.vertex_colors[:, 1]
bs = mesh.visual.vertex_colors[:, 2]
colors = np.array([rs, gs, bs])
colors = mesh.visual.vertex_colors[:, :4]
colors[:, 3] = 100
print("color shape: ", colors.shape)

plt.scatter(x, y, s=2, c=colors / 255)

plt.show()
