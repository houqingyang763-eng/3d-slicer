"""Build the shared MoveIt request for a needle-tip pose in UR base."""
import math

from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetMotionPlan
from shape_msgs.msg import SolidPrimitive


def ros_pose(position, quaternion):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quaternion)
    return pose


def needle_plan_request(parameter, joint_state, position_m, orientation_xyzw):
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.group_name = parameter("planning_group")
    motion.pipeline_id = parameter("pipeline_id")
    motion.planner_id = parameter("planner_id")
    motion.num_planning_attempts = 1
    motion.allowed_planning_time = float(parameter("planning_time_s"))
    motion.max_velocity_scaling_factor = float(parameter("velocity_scaling"))
    motion.max_acceleration_scaling_factor = float(parameter("acceleration_scaling"))
    motion.start_state.is_diff = True
    motion.start_state.joint_state = joint_state
    motion.workspace_parameters.header.frame_id = "base"
    motion.workspace_parameters.min_corner.x = motion.workspace_parameters.min_corner.y = motion.workspace_parameters.min_corner.z = -2.0
    motion.workspace_parameters.max_corner.x = motion.workspace_parameters.max_corner.y = motion.workspace_parameters.max_corner.z = 2.0

    # Constrain the point on tool0 so orientation tolerance cannot shift the tip
    # outside the position tolerance, as it could with a fixed flange target.
    position = PositionConstraint()
    position.header.frame_id = "base"
    position.link_name = "tool0"
    position.target_point_offset.x, position.target_point_offset.y, position.target_point_offset.z = map(float, parameter("tip_offset_m"))
    position.weight = 1.0
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(parameter("position_tolerance_m"))])
    position.constraint_region.primitives = [sphere]
    position.constraint_region.primitive_poses = [ros_pose(position_m, [0.0, 0.0, 0.0, 1.0])]
    orientation = OrientationConstraint()
    orientation.header.frame_id = "base"
    orientation.link_name = "tool0"
    orientation.orientation = ros_pose([0.0, 0.0, 0.0], orientation_xyzw).orientation
    # Bound the rotation-vector norm by distributing the overall budget over three axes.
    axis_tolerance = float(parameter("orientation_tolerance_rad")) / math.sqrt(3.0)
    orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = orientation.absolute_z_axis_tolerance = axis_tolerance
    orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
    orientation.weight = 1.0
    constraints = Constraints(name="needle_tip_target_v1", position_constraints=[position],
                              orientation_constraints=[orientation])
    motion.goal_constraints = [constraints]
    return request
