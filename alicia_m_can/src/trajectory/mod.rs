pub mod convergence;
pub mod governor;
pub mod limits;
pub mod planner;
pub mod sync_shape;
pub mod walker;

pub use convergence::needs_trajectory;
pub use limits::{
    build_trajectory_limits_from_vlim, TrajectoryLimits, DEFAULT_MAX_JOINT_ACCEL_RAD_S2,
    DEFAULT_MAX_JOINT_JERK_RAD_S3,
};
pub use planner::JointTrajectoryPlanner;
pub use walker::{note_stream_goal, step_joint_des};
