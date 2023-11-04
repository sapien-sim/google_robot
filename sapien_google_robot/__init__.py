import sapien
import numpy as np
import os


class EverydayRobot(sapien.Widget):
    def __init__(self):
        pass

    def load(self, scene: sapien.Scene):
        loader = scene.create_urdf_loader()
        loader.set_material(0.3, 0.3, 0.0)

        loader.set_link_material("link_finger_left", 1.0, 1.0, 0.0)
        loader.set_link_material("link_finger_tip_left", 1.0, 1.0, 0.0)
        loader.set_link_material("link_finger_right", 1.0, 1.0, 0.0)
        loader.set_link_material("link_finger_tip_right", 1.0, 1.0, 0.0)
        loader.set_link_patch_radius("link_finger_left", 0.05)
        loader.set_link_patch_radius("link_finger_tip_left", 0.05)
        loader.set_link_patch_radius("link_finger_right", 0.05)
        loader.set_link_patch_radius("link_finger_tip_right", 0.05)
        loader.set_link_min_patch_radius("link_finger_left", 0.05)
        loader.set_link_min_patch_radius("link_finger_tip_left", 0.05)
        loader.set_link_min_patch_radius("link_finger_right", 0.05)
        loader.set_link_min_patch_radius("link_finger_tip_right", 0.05)

        loader.set_link_material("link_wheel_left", 0.0, 0.0, 0.0)
        loader.set_link_material("link_wheel_right", 0.0, 0.0, 0.0)
        loader.set_link_material("link_rear_wheel_left", 0.0, 0.0, 0.0)
        loader.set_link_material("link_rear_wheel_right", 0.0, 0.0, 0.0)

        loader.load_multiple_collisions_from_file = True

        loader.fix_root_link = False

        path = os.path.join(
            os.path.dirname(__file__),
            "./googlerobot_description/meta_sim_fix_wheel.urdf",
        )
        self.robot = loader.load(path)
        pose = self.robot.pose
        for link in self.robot.links:
            if link.name != "link_base":
                link.disable_gravity = True

        self.robot.pose = sapien.Pose([0, 0, 0.0640623])
        # self.robot.pose = sapien.Pose([0, 0, 0.1])

        self.gripper_joints = [
            self.robot.find_joint_by_name(j)
            for j in ["joint_finger_left", "joint_finger_right"]
        ]

        self.arm_joints = [
            self.robot.find_joint_by_name(j)
            for j in [
                "joint_torso",
                "joint_shoulder",
                "joint_bicep",
                "joint_elbow",
                "joint_forearm",
                "joint_wrist",
                "joint_gripper",
            ]
        ]

        self.head_joints = [
            self.robot.find_joint_by_name(j)
            for j in ["joint_head_pan", "joint_head_tilt"]
        ]

        self._setup_passive_drives()

        self.set_arm_pd([2000] * 7, [200] * 7, [50] * 7)
        self.set_gripper_pd(1e4, 1e2, 0.3)

        c = sapien.physx.PhysxRigidDynamicComponent()
        c.set_kinematic(True)
        self.drive_dummy = sapien.Entity().add_component(c).add_to_scene(scene)

        self.base_drive = scene.create_drive(
            self.drive_dummy,
            sapien.Pose(q=[0.7071068, 0, -0.7071068, 0]),
            self.robot.root,
            sapien.Pose(q=[0.7071068, 0, -0.7071068, 0]),
        )
        self.set_base_drive_damping(1e4, 1e4)

    def _setup_passive_drives(self):
        for n in ["joint_finger_tip_left", "joint_finger_tip_right"]:
            j = self.robot.find_joint_by_name(n)
            j.set_drive_property(1e3, 1e2, mode="acceleration")
            j.set_drive_target(0)

    def set_arm_pd(self, ps, ds, limits):
        for j, p, d, l in zip(self.arm_joints, ps, ds, limits):
            j.set_drive_property(p, d, l, "acceleration")

    def set_head_pd(self, ps, ds, limits):
        for j, p, d, l in zip(self.head_joints, ps, ds, limits):
            j.set_drive_property(p, d, l, "acceleration")

    def set_gripper_pd(self, p, d, limit):
        for j in self.gripper_joints:
            j.set_drive_property(p, d, limit, "acceleration")

    def set_arm_target(self, target):
        for t, j in zip(target, self.arm_joints):
            j.set_drive_target(t)

    def set_arm_target_velocity(self, target):
        for t, j in zip(target, self.arm_joints):
            j.set_drive_velocity_target(t)

    def set_head_target(self, target):
        for t, j in zip(target, self.head_joints):
            j.set_drive_target(t)

    def set_gripper_target(self, target):
        for j in self.gripper_joints:
            j.set_drive_target(target)

    def set_base_drive_damping(self, linear, angular):
        self.base_drive.set_drive_property_z(0, linear, mode="acceleration")
        self.base_drive.set_drive_property_twist(0, angular, mode="acceleration")

    def step(self):
        self.drive_dummy.set_pose(self.robot.root.pose)

    def set_base_drive(self, linear, angular):
        self.base_drive.set_drive_velocity_target([0, 0, -linear], [-angular, 0, 0])

    def unload(self, scene: sapien.Scene):
        scene.remove_articulation(self.robot)


if __name__ == "__main__":
    scene = sapien.Scene()
    scene.load_widget_from_package("demo_arena", "DemoArena")
    robot = EverydayRobot()
    scene.load_widget(robot)

    # robot.set_base_drive(1, 0.5)

    scene.set_timestep(1 / 120)

    viewer = scene.create_viewer()
    while not viewer.closed:
        robot.step()
        scene.step()
        scene.update_render()
        viewer.render()
