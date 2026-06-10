# main.py
import time
import math
import numpy as np
import matplotlib.pyplot as plt


from pydrake.all import (
    DiagramBuilder,
    Simulator,
    AddMultibodyPlantSceneGraph,
    Meshcat,
    MeshcatVisualizer,
    LogVectorOutput,
    Quaternion,
)

from pydrake.multibody.tree import (
    QuaternionFloatingJoint,
    RotationalInertia,
    SpatialInertia,
)

from pydrake.geometry import Box, Cylinder, Rgba
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.math import SpatialVelocity
from pydrake.visualization import AddFrameTriadIllustration
from scipy.spatial.transform import Rotation as SciRot

from qsf_controller import QSF_combined
from draw_curve import draw_curve
from UAV_visual import add_quadrotor_visual



def main():
    # Simualtion parameters
    t_final = 40
    t_switch = 20
    port = 7004
    g  = 9.81
    
    # Block's geometry and color
    Lx, Ly, Lz = 0.2, 0.2, 0.05
    color_block = np.array([[0.2], [0.4], [1.0], [1.0]])
    
    # Inertial parameters
    m = 3
    J_cm = RotationalInertia(0.03, 0.03, 0.06)

    p_cm = np.array([[0.0], [0.0], [0.0]])

    spatial_inertia = SpatialInertia.MakeFromCentralInertia(m, p_cm, J_cm)
    
    # Path parameters for a circular path
    r1 = 1     
    r2 = 2
    vd = 1.5

    # -------------------------------- #
    #        Initial conditions        #
    # -------------------------------- #
    # Initial position (world frame)
    p0 = [-0.7, 1.75, 0]

    v0 = np.array([[-1], [2], [0.5]])

    # Initial orientation: Y-X-Z (2-1-3) Euler angles [rad]
    tht0 = -0.1   # Y
    phi0 = 0.2    # X
    psi0 = 1    # Z

    rot0 = SciRot.from_euler("YXZ", [tht0, phi0, psi0], degrees=False)
    R0 = RotationMatrix(rot0.as_matrix())

    # Initial pose of the body in the world frame
    pose0 = RigidTransform(R0, p0)

    om0 = np.array([[0.9], [-0.1], [0.5]])
    om0_W = R0.matrix() @ om0
    vom0 = SpatialVelocity(w=om0_W, v=v0)
    
    # ------------------------------- #
    #        Create the system        #
    # ------------------------------- #

    # Create an empty system first
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0) # time_step = 0.0 fro continous time system
    
    # View the empty system on a browser
    meshcat = Meshcat(port=port)

    # Draw the curve
    draw_curve(
        meshcat,
        "/curve",
        r=r1,
        z0=r2,
        N=150,
        radius=0.007,   # ← thickness
        rgba=Rgba(0.9, 1.0, 0.1, 1.0),
    )

    # Viewing angle 
    meshcat.SetCameraPose(
        camera_in_world=[2.5, 2.5, 2.5],
        target_in_world=[0.0, 0.0, 0.0],
    )
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
 
    # Scene visuals: add better looking inertial frame
    AddFrameTriadIllustration(
        scene_graph=scene_graph,
        body=plant.world_body(),   # world frame
        plant=plant,               # recommended / required for multibody args
        name="world",
        length=0.5,
        radius=0.007,               # thickness
        opacity=0.5,
    )



    # Add body 1: block
    block = plant.AddRigidBody("block", spatial_inertia)
     
    # Body 1 visuals
    add_quadrotor_visual(
        plant,
        block,
        scale = 0.6,
        yaw_deg=45.0
    )
    # If needed, show body frame
    # AddFrameTriadIllustration(
    #    scene_graph=scene_graph,
    #    body=block,                # this specific body
    #    plant=plant,
    #    name="block",
    #    length=0.3,
    #    radius=0.005,
    #    opacity=0.9,
    #)

    # Add b1-axis to the block
    axis_length = 0.3
    axis_thickness = 0.01

    color_b1 = np.array([[1.0], [0.0], [0.0], [1.0]])  # rgb color

    # Place the bar so it lies along +b1 (body x)
    b1_ax = RigidTransform([axis_length / 2.0, 0.0, 0.0])
    plant.RegisterVisualGeometry(
        block,
        b1_ax,
        Box(axis_length, axis_thickness, axis_thickness),
        "b1_axis_visual",
        color_b1,
    )
    #plant.RegisterVisualGeometry(
    #    block,
    #    RigidTransform(),
    #    Box(Lx, Ly, Lz),
    #    "block_visual",
    #    color_block,
    #)


    # Add the floating joint, i.e., free joint
    plant.AddJoint(
        QuaternionFloatingJoint(
            "block_floating",
            plant.world_frame(),
            block.body_frame(),
        )
    )
    

    
    # Finished creating the plant (or the world!)
    plant.Finalize()

    

    
    # Correct abstract type for applied spatial forces
    model_value = plant.get_applied_spatial_force_input_port().Allocate()
    # -------------------------------- #
    #        Add the controller        #
    # -------------------------------- #
    controller = builder.AddSystem(
        QSF_combined(  J = J_cm,
            m = m,
            g = g,
            r1 = r1,
            r2 = r2,
            vd = vd,
            body_index=block.index(),
            model_value=model_value,
            t_switch = t_switch,
        )
    )
    # ------------------------------ #
    #        Make block diagram      #
    # ------------------------------ #

    # Wiring: plant state -> controller, controller -> plant force
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), plant.get_applied_spatial_force_input_port())

    
    # Add a scope for the entire state vector
    state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    
    # Add a scope for the force f (world-frame force) from the cotnroller
    force_logger = LogVectorOutput(
          controller.GetOutputPort("force_world"),
          builder,
    )
    # Add a scope for tau (body-frame torque) from controller
    tau_logger = LogVectorOutput(
        controller.GetOutputPort("tau_body"),
        builder,
    )

    # Add a scope for thrust from controller
    thrust_logger = LogVectorOutput(
        controller.GetOutputPort("thrust"),
        builder,
    )
    
    

    # Finished the block diagram!
    diagram = builder.Build()

    # ------------------------------ #
    #        Start simulation        #
    # ------------------------------ #
    simulator = Simulator(diagram)

    integrator = simulator.get_mutable_integrator()
    #integrator.set_target_accuracy(1e-6)     # default is ~1e-3
    #integrator.set_maximum_step_size(1e-3)   # optional hard cap

    context = simulator.get_mutable_context()
    plant_context = plant.GetMyContextFromRoot(context)

    plant.SetFreeBodyPose(plant_context, block, pose0)
    plant.SetFreeBodySpatialVelocity(plant_context, block, vom0)

    # Simulation setup
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(True)


    # ------------------------------ #
    #   Recording + rotor failure    #
    # ------------------------------ #

    simulator.Initialize()

    # Make sure we start from a fresh recording each run
    visualizer.DeleteRecording()

    # Start recording poses (via MeshcatVisualizer) and any
    # SetProperty calls with time_in_recording.
    visualizer.StartRecording()

    # Meshcat paths for the two stacked rotor-1 geometries.
    # These match what you saw in the tree:
    #   /drake/visualizer/block/rotor_1_green
    #   /drake/visualizer/block/rotor_1_black
    rotor_green_path = "/drake/visualizer/block/rotor_1_green"
    rotor_black_path = "/drake/visualizer/block/rotor_1_black"

    # At t = 0:
    #   - show the green rotor_1
    #   - hide the black rotor_1
    # This updates the live view and stores a keyframe at time 0.
    meshcat.SetProperty(
        rotor_green_path,
        "visible",
        True,
        time_in_recording=0.0,
    )
    meshcat.SetProperty(
        rotor_black_path,
        "visible",
        False,
        time_in_recording=0.0,
    )

    # 1) Run the simulation until the switch time
    simulator.AdvanceTo(t_switch)

    # 2) At t_switch:
    #      - hide green geometry
    #      - show black geometry
    #    This affects both live view and recording.
    meshcat.SetProperty(
        rotor_green_path,
        "visible",
        False,
        time_in_recording=t_switch,
    )
    meshcat.SetProperty(
        rotor_black_path,
        "visible",
        True,
        time_in_recording=t_switch,
    )

    # 3) Continue simulation until the final time
    simulator.AdvanceTo(t_final)

    # Stop and publish the recording
    visualizer.StopRecording()
    visualizer.PublishRecording()


    
    #visualizer.StartRecording()
    #simulator.AdvanceTo(t_final)
    #visualizer.StopRecording()
    #visualizer.PublishRecording()

    # ----- Extract logged state -----
    log = state_logger.FindLog(context)
    t = log.sample_times()
    q = log.data()[0:plant.num_positions(), :]
    vel = log.data()[plant.num_positions():, :]
    # --------------------------- #
    #        Labeling data        #
    # --------------------------- #
    
    # postions
    p1 = q[4, :]
    p2 = q[5, :]
    p3 = q[6, :]

    # quaternions (Drake ordering: w, x, y, z)
    qw = q[0, :].copy()
    qx = q[1, :].copy()
    qy = q[2, :].copy()
    qz = q[3, :].copy()

    # Normalize quaternions (safety)
    norm_q = np.sqrt(qw ** 2 + qx ** 2 + qy ** 2 + qz ** 2)
    qw /= norm_q
    qx /= norm_q
    qy /= norm_q
    qz /= norm_q


    # Convert quaterions to Y-X-Z Euler angles
    # First, preallocate 2-1-3 (Y-X-Z) Euler angles
    tht = np.zeros_like(t)  # rotation about Y (axis 2)
    phi = np.zeros_like(t)  # rotation about X (axis 1)
    psi = np.zeros_like(t)  # rotation about Z (axis 3)
    
    # Second, convert
    for k in range(t.size):
        # SciPy expects quaternion as [x, y, z, w]
        rot_k = SciRot.from_quat([qx[k], qy[k], qz[k], qw[k]])
        # Equivalent to MATLAB rotm2eul(R,'YXZ')
        eul = rot_k.as_euler("YXZ", degrees=False)

        tht[k] = eul[0]  # Y
        phi[k] = eul[1]  # X
        psi[k] = eul[2]  # Z

    # velocities
    omega_W = vel[0:3, :]   # angular velocity in world
    v     = vel[3:6, :]   # translational velocity in world frame
    
    # Convert omega_W to omega in body frame
    omega = np.zeros_like(omega_W)
    for k in range(t.size):
        # SciPy expects quaternion as [x, y, z, w]
        rot_k = SciRot.from_quat([qx[k], qy[k], qz[k], qw[k]])
        R_WB = rot_k.as_matrix()   # maps body -> world: v_W = R_WB @ v_B

        # Convert world-frame angular velocity to body-frame:
        # omega_B = R_WB^T * omega_W
        omega[:, k] = R_WB.T @ omega_W[:, k]

    omega1 = omega[0]
    omega2 = omega[1]
    omega3 = omega[2]
    
    vx = v[0]
    vy = v[1]
    vz = v[2]

    # force f
    force_log = force_logger.FindLog(context)
    force_data = force_log.data()   # shape (3, N)
    fx = force_data[0, :]
    fy = force_data[1, :]
    fz = force_data[2, :]



    # torque tau
    tau_log = tau_logger.FindLog(context)
    tau_data = tau_log.data()  # shape (3, N)
    tau1 = tau_data[0, :]
    tau2 = tau_data[1, :]
    tau3 = tau_data[2, :]


    # thrust u__t
    u__t_log = thrust_logger.FindLog(context)
    u__t_data = u__t_log.data()       # shape (1, N)
    u__t = u__t_data[0, :]            # shape (N,)

    # ---------------------- #
    #        Plotting        #
    # ---------------------- #
    plt.figure()
    plt.plot(t, p1, label="p1")
    plt.plot(t, p2, label="p2")
    plt.plot(t, p3, label="p3")

    plt.xlabel("time (s)")
    plt.ylabel("position (m)")
    plt.title("Rigid body CoM position vs time")
    plt.legend()
    plt.grid(True)

    plt.show()
    
    

    # ------------------------- #
    #        Save to CSV        #
    # ------------------------- #
    np.savetxt(
        "Drake_data.csv",
        np.column_stack((t, p1, p2, p3, qw, qx, qy, qz, tht, phi, psi, vx, vy, vz , omega1, omega2, omega3, fx, fy, fz, u__t, tau1, tau2, tau3)),
        delimiter=",",
        header="t, p1, p2, p3, qw, qx, qy, qz, tht, phi, psi, vx, vy, vz , omega1, omega2, omega3, fx, fy, fz, u__t, tau1, tau2, tau3",
        comments="",
    )
    

    # ---------------------------------- #
    #       Print useful messages        #
    # ---------------------------------- #
    
    print(f"Meshcat: http://localhost:{port}")
    print("Keep-alive running; press Ctrl+C to quit.")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
