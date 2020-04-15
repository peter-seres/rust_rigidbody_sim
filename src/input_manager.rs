use gilrs::{Gilrs, Axis, Event, EventType};

pub struct ManualControl {
    pub throttle: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32
}

pub struct ControllerInput {
    gilrs: Gilrs,
    pub manual_control: ManualControl,
}

impl ControllerInput {
    pub fn new() -> ControllerInput {

        let gilrs = Gilrs::new().unwrap();

        for (_id, gamepad) in gilrs.gamepads() {
            println!(" >>> Found available gamepad input device: {}", gamepad.name());
        }

        let mc = ManualControl{
            throttle: 0.,
            roll: 0.,
            pitch: 0.,
            yaw: 0.,
        }; 

        let ci = ControllerInput{
                    gilrs: gilrs,
                    manual_control: mc,
                };

        return ci;
    }

    pub fn listen_to_inputs(&mut self) -> () {

        while let Some(Event { id: _, event, time: _ }) = self.gilrs.next_event() {
            match event {
                EventType::AxisChanged(Axis::LeftStickX, val, _) => self.manual_control.yaw = val,
                EventType::AxisChanged(Axis::LeftStickY, val, _) => self.manual_control.throttle = val,
                EventType::AxisChanged(Axis::RightStickX, val, _) => self.manual_control.roll = val,
                EventType::AxisChanged(Axis::RightStickY, val, _) => self.manual_control.pitch = -val,
                EventType::AxisChanged(Axis::DPadX, _, _) => (),
                EventType::AxisChanged(Axis::DPadY, _, _) => (),
                EventType::AxisChanged(Axis::LeftZ, _, _) => (),
                EventType::AxisChanged(Axis::RightZ, _, _) => (),
                EventType::AxisChanged(Axis::Unknown, _, _) => (),
                EventType::ButtonPressed(_, _) => (),
                EventType::ButtonChanged(_, _, _) => (),
                EventType::ButtonRepeated(_, _) => (),
                EventType::ButtonReleased(_, _) => (),
                EventType::Connected => (),
                EventType::Disconnected => (),
                EventType::Dropped => (),
            }
        }
    }
}