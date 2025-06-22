use fuzzy_controller::fuzzy::controller as FC;
use crate::drivers::actuators::servo::Servo;

pub struct FuzzyController {
    fuzzy: FC
}


impl FuzzyController {
    pub fn new() -> Self {
        Self{
            fuzzy: FC::new_fuzzy_controller()
        }
    }

    pub(crate) fn update(&self, p0: f32, p1: f32, p2: f32) -> _ {
        todo!()
    }
}

