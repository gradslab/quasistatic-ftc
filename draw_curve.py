import numpy as np

from pydrake.geometry import Cylinder, Rgba
from pydrake.math import RigidTransform, RotationMatrix


def draw_curve(
    meshcat,
    path,
    r=1.0,
    z0=2.0,
    N=200,
    radius=0.03,   # default thickness
    rgba=Rgba(1.0, 0.0, 0.0, 1.0),
    ):
    """
    Draws a thick circular curve using cylinders.
    Compatible with all Drake Python versions.
    """
    # Generate points on the circle
    theta = np.linspace(0.0, 2.0 * np.pi, N)
    points = np.column_stack((
        r * np.cos(theta),
        r * np.sin(theta),
        z0 * np.ones_like(theta),
    ))

    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]

        dp = p1 - p0
        length = np.linalg.norm(dp)
        if length < 1e-9:
            continue

        z_hat = dp / length  # desired cylinder axis (world)

        # Pick a helper vector not parallel to z_hat
        x_ref = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(x_ref, z_hat)) > 0.9:
            x_ref = np.array([0.0, 1.0, 0.0])

        y_hat = np.cross(z_hat, x_ref)
        y_hat /= np.linalg.norm(y_hat)

        x_hat = np.cross(y_hat, z_hat)

        # Rotation matrix: columns are body axes in world
        R = RotationMatrix(np.column_stack((x_hat, y_hat, z_hat)))

        X_WC = RigidTransform(R, 0.5 * (p0 + p1))

        seg_path = f"{path}/{i}"
        meshcat.SetObject(
            seg_path,
            Cylinder(radius=radius, length=length),
            rgba=rgba,
        )
        meshcat.SetTransform(seg_path, X_WC)
