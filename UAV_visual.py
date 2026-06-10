import numpy as np

from pydrake.geometry import Box, Cylinder
from pydrake.math import RigidTransform, RollPitchYaw


def add_quadrotor_visual(
    plant,
    body,
    scale=0.5,
    hub_size=(0.12, 0.12, 0.025),
    yaw_deg=45.0,   # rotation about b3 (body z), in degrees
):
    """
    Quadrotor visual attached to an existing rigid body.

    - scale controls overall size of the drone
    - visuals are rotated about b3 by yaw_deg
    - all rotors are green by default
    - rotor_1 has two stacked visuals: rotor_1_green and rotor_1_black
    - arms stop exactly at propeller centers
    """

    # ---- Colors (4x1 numpy arrays as required by Drake) ----
    hub_color        = np.array([[0.2], [0.4], [1.0], [1.0]])
    arm_color        = np.array([[0.2], [0.2], [0.2], [1.0]])
    rotor_green_col  = np.array([[0.0], [0.8], [0.0], [1.0]])  # green
    rotor_black_col  = np.array([[0.0], [0.0], [0.0], [0.2]])  # black

    # ---- Rotation of the UAV visual about b3 ----
    yaw_rad = np.deg2rad(yaw_deg)
    R_BV = RollPitchYaw(0.0, 0.0, yaw_rad).ToRotationMatrix()
    Rmat = R_BV.matrix()

    # ---- Hub ----
    Lx, Ly, Lz = hub_size
    plant.RegisterVisualGeometry(
        body,
        RigidTransform(R_BV, [0.0, 0.0, 0.0]),
        Box(Lx, Ly, Lz),
        "hub",
        hub_color,
    )

    # ---- Arms (shortened; end at prop centers) ----
    arm_width  = 0.03
    arm_height = 0.02

    d = scale / 2.0          # rotor distance from center
    arm_len_vis = d          # arm length
    arm_center  = d / 2.0    # arm center location

    # X-arms
    for sign in (+1.0, -1.0):
        p_local = np.array([sign * arm_center, 0.0, 0.0])
        p_B = Rmat @ p_local
        plant.RegisterVisualGeometry(
            body,
            RigidTransform(R_BV, p_B),
            Box(arm_len_vis, arm_width, arm_height),
            f"arm_x_{sign:+}",
            arm_color,
        )

    # Y-arms
    for sign in (+1.0, -1.0):
        p_local = np.array([0.0, sign * arm_center, 0.0])
        p_B = Rmat @ p_local
        plant.RegisterVisualGeometry(
            body,
            RigidTransform(R_BV, p_B),
            Box(arm_width, arm_len_vis, arm_height),
            f"arm_y_{sign:+}",
            arm_color,
        )

    # ---- Rotors ----
    rotor_radius    = 0.12
    rotor_thickness = 0.01

    rotor_local_positions = [
        np.array([+d,  0.0, 0.0]),  # rotor 1 (front, say)
        np.array([0.0, +d,  0.0]),  # rotor 2
        np.array([-d,  0.0, 0.0]),  # rotor 3
        np.array([0.0, -d,  0.0]),  # rotor 4
    ]

    for i, p_local in enumerate(rotor_local_positions):
        p_B = Rmat @ p_local

        if i == 0:
            # Rotor 1: two stacked visuals (green + black) at the same pose

            # Green rotor (healthy)
            plant.RegisterVisualGeometry(
                body,
                RigidTransform(R_BV, p_B),
                Cylinder(rotor_radius, rotor_thickness),
                "rotor_1_green",
                rotor_green_col,
            )

            # Black rotor (for failure visualization)
            plant.RegisterVisualGeometry(
                body,
                RigidTransform(R_BV, p_B),
                Cylinder(rotor_radius, rotor_thickness),
                "rotor_1_black",
                rotor_black_col,
            )
        else:
            # Rotors 2, 3, 4: single green visual each
            plant.RegisterVisualGeometry(
                body,
                RigidTransform(R_BV, p_B),
                Cylinder(rotor_radius, rotor_thickness),
                f"rotor_{i+1}",
                rotor_green_col,
            )
