# -*- coding: utf8 -*-

import argparse
import os
import random
import time

import cv2
import imageio
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp

import pydrake
from pydrake.solvers import ik
import pydrake.math as drakemath
from pydrake.all import (
    AbstractValue,
    AddFlatTerrainToWorld,
    AddModelInstancesFromSdfString,
    AddModelInstanceFromUrdfFile,
    BasicVector,
    Box,
    CompliantMaterial,
    DiagramBuilder,
    Expression,
    FloatingBaseType,
    Image,
    LeafSystem,
    PixelType,
    PortDataType,
    RgbdCamera,
    RigidBody,
    RigidBodyFrame,
    RigidBodyPlant,
    RigidBodyTree,
    RollPitchYawFloatingJoint,
    RungeKutta2Integrator,
    Shape,
    SignalLogger,
    Simulator,
    Variable,
    VisualElement
)

from underactuated.meshcat_rigid_body_visualizer import (
    MeshcatRigidBodyVisualizer)

import meshcat
import meshcat.transformations as tf
import meshcat.geometry as g


def save_image_uint16(name, im):
    array_as_uint16 = im.astype(np.uint16)
    imageio.imwrite(name, array_as_uint16)


def save_image_uint8(name, im):
    array_as_uint8 = im.astype(np.uint8)
    imageio.imwrite(name, array_as_uint8)


def save_image_colormap(name, im):
    plt.imsave(name, im, cmap=plt.cm.inferno)


def save_depth_colormap(name, im, near, far):
    cmapped = plt.cm.jet((far - im)/(far - near))
    zero_range_mask = im < near
    cmapped[:, :, 0][zero_range_mask] = 0
    cmapped[:, :, 1][zero_range_mask] = 0
    cmapped[:, :, 2][zero_range_mask] = 0
    imageio.imwrite(name, (cmapped*255.).astype(np.uint8))


def setup_tabletop(rbt):
    table_sdf_path = os.path.join(
        pydrake.getDrakePath(),
        "examples", "kuka_iiwa_arm", "models", "table",
        "extra_heavy_duty_table_surface_only_collision.sdf")

    object_urdf_paths = [
        os.path.join(
            pydrake.getDrakePath(),
            "examples", "kuka_iiwa_arm", "models", "objects",
            "block_for_pick_and_place.urdf"),
        os.path.join(
            pydrake.getDrakePath(),
            "examples", "kuka_iiwa_arm", "models", "objects",
            "big_robot_toy.urdf"),
        os.path.join(
            pydrake.getDrakePath(),
            "examples", "kuka_iiwa_arm", "models", "objects",
            "simple_cylinder.urdf")
        ]

    AddFlatTerrainToWorld(rbt)
    table_frame_robot = RigidBodyFrame(
        "table_frame_robot", rbt.world(),
        [0.0, 0, 0], [0, 0, 0])
    AddModelInstancesFromSdfString(
        open(table_sdf_path).read(), FloatingBaseType.kFixed,
        table_frame_robot, rbt)

    table_top_z_in_world = 0.736 + 0.057 / 2

    num_objects = np.random.randint(10)+1
    for i in range(num_objects):
        object_ind = np.random.randint(len(object_urdf_paths))
        hp = (np.random.random(2)-0.5)*0.25
        object_init_frame = RigidBodyFrame(
            "object_init_frame", rbt.world(),
            [hp[0], hp[1], table_top_z_in_world+0.05],
            np.random.random(3))
        AddModelInstanceFromUrdfFile(
            object_urdf_paths[object_ind % len(object_urdf_paths)],
            FloatingBaseType.kRollPitchYaw,
            object_init_frame, rbt)

    # Add camera geometry!
    camera_link = RigidBody()
    camera_link.set_name("camera_link")
    camera_visual_element = VisualElement(
        Box([0.1, 0.1, 0.1]),
        np.eye(4), [1., 0., 1., 0.5])
    camera_link.AddVisualElement(camera_visual_element)
    # necessary so this last link isn't pruned by the rbt.compile() call
    camera_link.set_spatial_inertia(np.eye(6))
    camera_link.add_joint(
        rbt.world(),
        RollPitchYawFloatingJoint(
            "camera_floating_base",
            np.eye(4)))
    rbt.add_rigid_body(camera_link)

    # - Add frame for camera fixture.
    camera_frame = RigidBodyFrame(
        name="rgbd_camera_frame", body=camera_link,
        xyz=[0.105, 0., 0.], rpy=[0., 0., 0.])
    rbt.addFrame(camera_frame)

    rbt.compile()

    # Project arrangement to nonpenetration with IK
    constraints = []

    constraints.append(ik.MinDistanceConstraint(
        model=rbt, min_distance=0.01, active_bodies_idx=list(),
        active_group_names=set()))

    for body_i in range(2, rbt.get_num_bodies()):
        constraints.append(ik.WorldPositionConstraint(
            model=rbt, body=body_i, pts=np.array([0., 0., 0.]),
            lb=np.array([-0.5, -0.5, table_top_z_in_world]),
            ub=np.array([0.5, 0.5, table_top_z_in_world+0.5])))

    q0 = np.zeros(rbt.get_num_positions())
    options = ik.IKoptions(rbt)
    options.setDebug(True)
    options.setMajorIterationsLimit(10000)
    options.setIterationsLimit(100000)
    results = ik.InverseKin(
        rbt, q0, q0, constraints, options)

    qf = results.q_sol[0]
    info = results.info[0]
    print "Projected with info %d" % info
    return qf


class CameraPoseInjectionBlock(LeafSystem):
    def __init__(self, rbt, t_var, trajectory):
        LeafSystem.__init__(self)
        self.set_name('camera pose injection')
        self.trajectory = trajectory
        self.t_var = t

        self.nq = rbt.get_num_positions()
        self.nu = self.nq + rbt.get_num_velocities()

        self.state_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   self.nu)
        self.state_output_port = \
            self._DeclareVectorOutputPort(
                    BasicVector(self.nu),
                    self._DoCalcVectorOutput)

    def _DoCalcVectorOutput(self, context, y_data):
        y = y_data.get_mutable_value()
        x = self.EvalVectorInput(
            context, self.state_input_port.get_index()).get_value()
        y[:] = x[:]
        y[(self.nq-6):self.nq] = [tr.Evaluate(
            {self.t_var: context.get_time()}) for tr in self.trajectory]


class DepthImageCorruptionBlock(LeafSystem):
    def __init__(self, camera, save_dir):
        LeafSystem.__init__(self)
        self.set_name('depth image corruption superclass')
        self.camera = camera

        self.save_dir = save_dir
        if len(self.save_dir) > 0:
            os.system("rm -r %s" % self.save_dir)
            os.system("mkdir -p %s" % self.save_dir)

        self.depth_image_input_port = \
            self._DeclareInputPort(PortDataType.kAbstractValued,
                                   camera.depth_image_output_port().size())

        self.color_image_input_port = \
            self._DeclareInputPort(PortDataType.kAbstractValued,
                                   camera.color_image_output_port().size())

        self.depth_image_output_port = \
            self._DeclareAbstractOutputPort(
                self._DoAllocDepthCameraImage,
                self._DoCalcAbstractOutput)

    def _DoAllocDepthCameraImage(self):
        test = AbstractValue.Make(Image[PixelType.kDepth32F](
            self.camera.depth_camera_info().width(),
            self.camera.depth_camera_info().height()))
        return test

    def _DoCalcAbstractOutput(self, context, y_data):
        print "OVERRIDE ME"
        sys.exit(-1)


class DepthImageCNNCorruptionBlock(DepthImageCorruptionBlock):
    def __init__(self, camera, save_dir, single_scene_mode=False):
        DepthImageCorruptionBlock.__init__(self, camera, save_dir)
        self.set_name('depth image corruption, cnn')

        if single_scene_mode:
            self.model = network.load_trained_model(
                weights_path="DepthSim/python/models/"
                "2017-06-16-30_depth_batch4.hdf5")
        else:
            self.model = network.load_trained_model(
                weights_path="DepthSim/python/models/net_depth_seg_v1.hdf5")

        self.near_distance = 0.2
        self.far_distance = 3.5
        self.dropout_threshold = 0.5
        self.iter = 0

    def _DoCalcAbstractOutput(self, context, y_data):
        start_time = time.time()

        u_data = self.EvalAbstractInput(context, 1).get_value()
        h, w, _ = u_data.data.shape
        rgb_image = np.empty((h, w), dtype=np.float64)
        rgb_image[:, :] = u_data.data[:, :, 0]

        if len(self.save_dir) > 0:
            save_image_uint8(
                "%s/%05d_rgb.png" % (self.save_dir, self.iter), rgb_image)

        u_data = self.EvalAbstractInput(context, 0).get_value()
        h, w, _ = u_data.data.shape
        depth_image = np.empty((h, w), dtype=np.float64)
        depth_image[:, :] = u_data.data[:, :, 0]
        good_mask = np.isfinite(depth_image)
        depth_image = np.clip(depth_image, self.near_distance,
                              self.far_distance)

        depth_image_normalized = depth_image / self.far_distance
        depth_image_resized = cv2.resize(
            depth_image_normalized, (640, 480),
            interpolation=cv2.INTER_NEAREST)

        stack = np.empty((1, 480, 640, 1))
        stack[0, :, :, 0] = depth_image_resized[:, :]
        predicted_prob_map = self.model.predict_on_batch(stack)

        if len(self.save_dir) > 0:
            save_depth_colormap(
                "%s/%05d_input_depth.png" % (self.save_dir, self.iter),
                depth_image_resized, self.near_distance/self.far_distance, 1.0)
            save_image_colormap(
                "%s/%05d_mask.png" % (self.save_dir, self.iter),
                predicted_prob_map[0, :, :, 0])

        network.apply_mask(
            predicted_prob_map, depth_image_resized, self.dropout_threshold)
        depth_image = self.far_distance * depth_image_resized

        if len(self.save_dir) > 0:
            save_depth_colormap(
                "%s/%05d_masked_depth.png" % (self.save_dir, self.iter),
                depth_image, self.near_distance, self.far_distance)

        # Where it's infinite, set to 0
        depth_image = np.where(
            good_mask, depth_image,
            np.zeros(depth_image.shape))
        y_data.get_mutable_value().mutable_data[:, :, 0] = \
            depth_image[:, :]
        print "Elapsed in render (cnn): %f seconds" % \
            (time.time() - start_time)
        self.iter += 1


class RgbdCameraMeshcatVisualizer(LeafSystem):
    def __init__(self,
                 camera,
                 rbt,
                 draw_timestep=0.033333,
                 prefix="RBCameraViz",
                 zmq_url="tcp://127.0.0.1:6000"):
        LeafSystem.__init__(self)
        self.set_name('camera meshcat visualization')
        self.timestep = draw_timestep
        self._DeclarePeriodicPublish(draw_timestep, 0.0)
        self.camera = camera
        self.rbt = rbt
        self.prefix = prefix

        self.camera_input_port = \
            self._DeclareInputPort(PortDataType.kAbstractValued,
                                   camera.depth_image_output_port().size())
        self.state_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   rbt.get_num_positions() +
                                   rbt.get_num_velocities())

        self.ax = None

        # Set up meshcat
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        self.vis[prefix].delete()

    def _DoPublish(self, context, event):
        u_data = self.EvalAbstractInput(context, 0).get_value()
        x = self.EvalVectorInput(context, 1).get_value()
        w, h, _ = u_data.data.shape
        depth_image = u_data.data[:, :, 0]

        if self.ax is None:
            self.ax = plt.imshow(depth_image)
        else:
            self.ax.set_data(depth_image)
        plt.pause(1E-12)

        # Convert depth image to point cloud, with +z being
        # camera "forward"
        Kinv = np.linalg.inv(
            self.camera.depth_camera_info().intrinsic_matrix())
        U, V = np.meshgrid(np.arange(h), np.arange(w))
        points_in_camera_frame = np.vstack([
            U.flatten(),
            V.flatten(),
            np.ones(w*h)])
        points_in_camera_frame = Kinv.dot(points_in_camera_frame) * \
            depth_image.flatten()

        # The depth camera has some offset from the camera's root frame,
        # so take than into account.
        pose_mat = self.camera.depth_camera_optical_pose().matrix()
        points_in_camera_frame = pose_mat[0:3, 0:3].dot(points_in_camera_frame)
        points_in_camera_frame += np.tile(pose_mat[0:3, 3], [w*h, 1]).T

        kinsol = self.rbt.doKinematics(x)
        points_in_world_frame = self.rbt.transformPoints(
            kinsol,
            points_in_camera_frame,
            self.camera.frame().get_frame_index(),
            0)

        # Color points according to their normalized height
        min_height = 0.7
        max_height = 1.0
        colors = cm.jet(
            (points_in_world_frame[2, :]-min_height)/(max_height-min_height)
            ).T[0:3, :]

        self.vis[self.prefix]["points"].set_object(
            g.PointCloud(position=points_in_world_frame,
                         color=colors,
                         size=0.005))


if __name__ == "__main__":
    np.set_printoptions(precision=5, suppress=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-T", "--duration",
                        type=float,
                        help="Duration to run sim.",
                        default=2.*np.pi)
    parser.add_argument("--test",
                        action="store_true",
                        help="Help out CI by launching a meshcat server for "
                             "the duration of the test.")
    parser.add_argument("--corruption_method",
                        type=str,
                        default="cnn",
                        help="[cnn, cnn_single_scene, none]")
    parser.add_argument("--seed",
                        type=int,
                        default=42,
                        help="Random seed for rng, "
                             "including scene generation.")
    parser.add_argument("--save_dir",
                        type=str,
                        default="",
                        help="Directory to save depth diagnostic images."
                             " If not specified, does not save images.")
    parser.add_argument("--orbit",
                        action="store_true",
                        help="Orbit camera at 1 radian per second.")
    args = parser.parse_args()

    np.random.seed(args.seed)
    random.seed(args.seed)

    meshcat_server_p = None
    if args.test:
        print "Spawning"
        import subprocess
        meshcat_server_p = subprocess.Popen(["meshcat-server"])
    else:
        print "Warning: if you have not yet run meshcat-server in another " \
              "terminal, this will hang."

    # Construct the robot and its environment
    rbt = RigidBodyTree()
    q0 = setup_tabletop(rbt)

    # Set up a visualizer for the robot
    pbrv = MeshcatRigidBodyVisualizer(rbt, draw_timestep=0.01)
    # (wait while the visualizer warms up and loads in the models)
    time.sleep(2.0)

    # Make our RBT into a plant for simulation
    rbplant = RigidBodyPlant(rbt)
    rbplant.set_name("Rigid Body Plant")
    allmaterials = CompliantMaterial()
    allmaterials.set_youngs_modulus(1E8)  # default 1E9
    allmaterials.set_dissipation(1.0)     # default 0.32
    allmaterials.set_friction(0.9)        # default 0.9.
    rbplant.set_default_compliant_material(allmaterials)

    # Build up our simulation by spawning controllers and loggers
    # and connecting them to our plant.
    builder = DiagramBuilder()
    # The diagram takes ownership of all systems
    # placed into it.
    rbplant_sys = builder.AddSystem(rbplant)

    # Hack a camera gantry together by corrupting the robot
    # state being used to generate the camera image.
    # TODO(anon) Replace this with a less silly way of
    # writing a trajectory...
    t = Variable("t")
    if args.orbit is True:
        traj = [drakemath.cos(t),
                drakemath.sin(t),
                Expression(1.3),
                Expression(0.0),
                Expression(0.5),
                -np.pi + t]
    else:
        traj = [Expression(1.),
                Expression(0.),
                Expression(1.3),
                Expression(0.0),
                Expression(0.5),
                Expression(-np.pi)]

    pose_injector = builder.AddSystem(
        CameraPoseInjectionBlock(rbt, t, traj))
    builder.Connect(rbplant_sys.state_output_port(),
                    pose_injector.state_input_port)

    # Hook up the visualizer we created earlier.
    visualizer = builder.AddSystem(pbrv)
    builder.Connect(pose_injector.state_output_port,
                    visualizer.get_input_port(0))

    # Add a camera, too, though no controller or estimator
    # will consume the output of it.
    camera = builder.AddSystem(
        RgbdCamera(name="camera", tree=rbt,
                   frame=rbt.findFrame("rgbd_camera_frame"),
                   z_near=0.2, z_far=3.5, fov_y=np.pi / 4,
                   width=640, height=480,
                   show_window=False))
    builder.Connect(pose_injector.state_output_port,
                    camera.get_input_port(0))

    if (args.corruption_method == "cnn" or
       args.corruption_method == "cnn_single_scene"):
        camera.set_color_camera_optical_pose(
            camera.depth_camera_optical_pose())

        from RGBDCNN import network
        depth_corruptor = builder.AddSystem(
            DepthImageCNNCorruptionBlock(
                camera, args.save_dir,
                single_scene_mode=(
                    args.corruption_method == "cnn_single_scene")))
        builder.Connect(camera.color_image_output_port(),
                        depth_corruptor.color_image_input_port)
        builder.Connect(camera.depth_image_output_port(),
                        depth_corruptor.depth_image_input_port)
        final_depth_output_port = depth_corruptor.depth_image_output_port
    elif args.corruption_method == "none":
        camera.set_color_camera_optical_pose(
            camera.depth_camera_optical_pose())
        final_depth_output_port = camera.depth_image_output_port()
    else:
        print "Got invalid corruption method %s." % args.corruption_method
        sys.exit(-1)

    camera_meshcat_visualizer = builder.AddSystem(
        RgbdCameraMeshcatVisualizer(camera, rbt))
    builder.Connect(final_depth_output_port,
                    camera_meshcat_visualizer.camera_input_port)
    builder.Connect(pose_injector.state_output_port,
                    camera_meshcat_visualizer.state_input_port)

    # Done!
    diagram = builder.Build()

    # Create a simulator for it.
    simulator = Simulator(diagram)

    # The simulator simulates forward from a given Context,
    # so we adjust the simulator's initial Context to set up
    # the initial state.
    state = simulator.get_mutable_context().\
        get_mutable_continuous_state_vector()
    x0 = np.zeros(rbplant_sys.get_num_states())
    x0[0:q0.shape[0]] = q0
    state.SetFromVector(x0)

    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    # Simulator time steps will be very small, so don't
    # force the rest of the system to update every single time.
    simulator.set_publish_every_time_step(False)

    # From iiwa_wsg_simulation.cc:
    # When using the default RK3 integrator, the simulation stops
    # advancing once the gripper grasps the box.  Grasping makes the
    # problem computationally stiff, which brings the default RK3
    # integrator to its knees.
    timestep = 0.0001
    simulator.reset_integrator(
        RungeKutta2Integrator(diagram, timestep,
                              simulator.get_mutable_context()))

    # This kicks off simulation. Most of the run time will be spent
    # in this call.
    simulator.StepTo(args.duration)
    print("Final state: ", state.CopyToVector())

    if meshcat_server_p is not None:
        meshcat_server_p.kill()
        meshcat_server_p.wait()
