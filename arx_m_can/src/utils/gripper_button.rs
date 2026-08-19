use motor_vendor_damiao::MotorFeedbackState;

pub use crate::config::GripperButtonConfig;

pub const DEFAULT_GRIPPER_BUTTON_PMAX: f64 = 12.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ButtonEvent {
    None = 0x00,
    Single = 0x01,
    Double = 0x02,
    Long = 0x03,
    UltraLong = 0x04,
}

impl ButtonEvent {
    pub fn from_raw(value: u8) -> Self {
        match value {
            0x00 => ButtonEvent::None,
            0x01 => ButtonEvent::Single,
            0x02 => ButtonEvent::Double,
            0x03 => ButtonEvent::Long,
            0x04 => ButtonEvent::UltraLong,
            _ => ButtonEvent::None,
        }
    }

    pub fn as_u8(self) -> u8 {
        self as u8
    }
}

pub const DEFAULT_BUTTON_TOGGLE_EVENTS: &[ButtonEvent] = &[ButtonEvent::Single];

pub fn pos_to_p_int(pos: f32, pmax: f64) -> i32 {
    let span = 2.0 * pmax;
    let max_val = (1u32 << 16) - 1;
    let scaled = (f64::from(pos) + pmax) * f64::from(max_val) / span;
    scaled.round() as i32
}

pub fn p_int_to_button_event(p_int: i32) -> ButtonEvent {
    if (0..=4).contains(&p_int) {
        ButtonEvent::from_raw(p_int as u8)
    } else {
        ButtonEvent::None
    }
}

pub fn decode_button_event_from_state(
    state: Option<&MotorFeedbackState>,
    motor_id: u16,
    feedback_id: u16,
    pmax: f64,
) -> Option<u8> {
    let state = state?;
    if u32::from(state.can_id) != u32::from(motor_id & 0x0F) {
        return None;
    }
    if state.arbitration_id != u32::from(feedback_id) {
        return None;
    }
    Some(p_int_to_button_event(pos_to_p_int(state.pos, pmax)).as_u8())
}

pub fn button_event_name(event: u8) -> &'static str {
    match ButtonEvent::from_raw(event) {
        ButtonEvent::None => "none",
        ButtonEvent::Single => "single",
        ButtonEvent::Double => "double",
        ButtonEvent::Long => "long",
        ButtonEvent::UltraLong => "ultra_long",
    }
}

pub fn button_event_should_toggle(
    event: Option<u8>,
    last_seen: u8,
    toggle_events: &[ButtonEvent],
) -> (u8, bool) {
    let Some(event) = event else {
        return (last_seen, false);
    };
    if event == ButtonEvent::None.as_u8() {
        return (ButtonEvent::None.as_u8(), false);
    }
    let toggle = toggle_events
        .iter()
        .any(|candidate| candidate.as_u8() == event);
    if event != last_seen && toggle {
        return (event, true);
    }
    if event != last_seen {
        return (event, false);
    }
    (last_seen, false)
}
