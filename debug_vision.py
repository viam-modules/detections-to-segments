"""
Debug script: connects to a Viam robot, calls the detections-to-segments vision service,
and saves the image with detection overlays plus point cloud data for inspection.

Usage:
    pip install viam-sdk pillow numpy
    python debug_vision.py
"""

import asyncio
import io
import os
import struct

from PIL import Image, ImageDraw
from viam.robot.client import RobotClient
from viam.rpc.dial import DialOptions
from viam.services.vision import VisionClient

# Load env vars from viam.env if it exists.
env_path = os.path.join(os.path.dirname(__file__), "..", "viam.env")
if os.path.exists(env_path):
    with open(env_path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                os.environ.setdefault(k.strip(), v.strip())

ROBOT_ADDRESS = os.environ.get("VIAM_ROBOT_ADDRESS", "vino2-main.kssbd6djf3.viam.cloud")
VISION_SERVICE_NAME = os.environ.get("VIAM_VISION_SERVICE", "vision-detections-to-segments")

OUTPUT_DIR = "debug_output"


async def main():
    api_key = os.environ.get("VIAM_API_KEY", "")
    api_key_id = os.environ.get("VIAM_API_KEY_ID", "")
    if not api_key or not api_key_id:
        print("Set VIAM_API_KEY and VIAM_API_KEY_ID (or put them in ../viam.env)")
        return

    print(f"Connecting to {ROBOT_ADDRESS}...")
    robot = await RobotClient.at_address(
        ROBOT_ADDRESS,
        RobotClient.Options(
            dial_options=DialOptions.with_api_key(
                api_key=api_key, api_key_id=api_key_id,
            ),
        ),
    )
    print(f"Connected! Resources: {[r.name for r in robot.resource_names]}")

    vision = VisionClient.from_robot(robot, VISION_SERVICE_NAME)
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # --- capture_all_from_camera ---
    print(f"\nCalling capture_all_from_camera on '{VISION_SERVICE_NAME}'...")
    try:
        result = await vision.capture_all_from_camera(
            "",  # use the default camera configured on the module
            return_image=True,
            return_detections=True,
            return_object_point_clouds=True,
            timeout=60,
        )
    except Exception as e:
        print(f"  ERROR: {e}")
        await robot.close()
        return

    # --- Image + detections ---
    if result.image is not None:
        pil = Image.open(io.BytesIO(result.image.data)).convert("RGB")
        draw = ImageDraw.Draw(pil)

        dets = result.detections or []
        print(f"\n  Image size: {pil.size}")
        print(f"  Detections: {len(dets)}")
        for i, det in enumerate(dets):
            print(f"    [{i}] {det.class_name} conf={det.confidence:.2f} "
                  f"bbox=({det.x_min},{det.y_min})-({det.x_max},{det.y_max})")
            draw.rectangle(
                [det.x_min, det.y_min, det.x_max, det.y_max],
                outline="lime", width=3,
            )
            label = f"{det.class_name} {det.confidence:.2f}"
            draw.text((det.x_min, det.y_min - 15), label, fill="lime")

        img_path = os.path.join(OUTPUT_DIR, "capture.png")
        pil.save(img_path)
        print(f"  Saved annotated image to {img_path}")
    else:
        print("  No image returned")

    # --- Object point clouds ---
    objects = result.objects or []
    print(f"\n  Object point clouds: {len(objects)}")
    for i, obj in enumerate(objects):
        pc_bytes = obj.point_cloud
        geom = obj.geometries
        print(f"    [{i}] point_cloud bytes={len(pc_bytes) if pc_bytes else 0}, "
              f"geometries={geom}")

        if pc_bytes:
            # Try to parse PCD data and print summary stats
            try:
                pc_text = pc_bytes.decode("ascii", errors="replace")
                lines = pc_text.split("\n")
                header_lines = []
                data_start = 0
                for j, line in enumerate(lines):
                    header_lines.append(line)
                    if line.startswith("DATA"):
                        data_start = j + 1
                        break

                print(f"    PCD header:")
                for hl in header_lines:
                    print(f"      {hl}")

                # Parse points if ASCII format
                if "DATA ascii" in pc_text:
                    points = []
                    for line in lines[data_start:]:
                        parts = line.strip().split()
                        if len(parts) >= 3:
                            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                            points.append((x, y, z))

                    if points:
                        xs = [p[0] for p in points]
                        ys = [p[1] for p in points]
                        zs = [p[2] for p in points]
                        print(f"    Point count: {len(points)}")
                        print(f"    X range: [{min(xs):.1f}, {max(xs):.1f}]")
                        print(f"    Y range: [{min(ys):.1f}, {max(ys):.1f}]")
                        print(f"    Z range: [{min(zs):.1f}, {max(zs):.1f}]")
                elif "DATA binary" in pc_text:
                    # Find where binary data starts (after header + newline)
                    header_end = pc_bytes.index(b"DATA binary") + len(b"DATA binary\n")
                    binary_data = pc_bytes[header_end:]

                    # Parse number of points from header
                    num_points = 0
                    point_size = 0
                    for hl in header_lines:
                        if hl.startswith("POINTS"):
                            num_points = int(hl.split()[1])
                        if hl.startswith("SIZE"):
                            sizes = [int(s) for s in hl.split()[1:]]
                            point_size = sum(sizes)

                    if num_points > 0 and point_size > 0:
                        points = []
                        for pi in range(min(num_points, 10000)):
                            offset = pi * point_size
                            if offset + 12 > len(binary_data):
                                break
                            x, y, z = struct.unpack_from("<fff", binary_data, offset)
                            points.append((x, y, z))

                        if points:
                            xs = [p[0] for p in points]
                            ys = [p[1] for p in points]
                            zs = [p[2] for p in points]
                            print(f"    Point count: {len(points)} (of {num_points})")
                            print(f"    X range: [{min(xs):.1f}, {max(xs):.1f}]")
                            print(f"    Y range: [{min(ys):.1f}, {max(ys):.1f}]")
                            print(f"    Z range: [{min(zs):.1f}, {max(zs):.1f}]")

                # Save raw PCD file
                pcd_path = os.path.join(OUTPUT_DIR, f"object_{i}.pcd")
                with open(pcd_path, "wb") as f:
                    f.write(pc_bytes)
                print(f"    Saved PCD to {pcd_path}")

            except Exception as e:
                print(f"    Error parsing point cloud: {e}")

    await robot.close()
    print(f"\nDone! Output saved to {OUTPUT_DIR}/")


if __name__ == "__main__":
    asyncio.run(main())
