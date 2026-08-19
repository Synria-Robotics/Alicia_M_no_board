pub mod gripper;
pub mod gripper_button;
pub mod speed_mapping;

pub use gripper::{motor_to_user, user_to_motor, validate_gripper_value, GRIPPER_MAX, GRIPPER_MIN};
pub use gripper_button::{
    button_event_name, button_event_should_toggle, decode_button_event_from_state,
    pos_to_p_int, ButtonEvent, GripperButtonConfig, DEFAULT_BUTTON_TOGGLE_EVENTS,
};
pub use speed_mapping::{speed_to_gripper_vlim_rad_s, speed_to_vlim_rad_s, ALICIA_SPEED_MAX};
