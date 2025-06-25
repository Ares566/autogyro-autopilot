use fuzzy_controller::fuzzy::controller as FC;


pub struct FuzzyController {
    fuzzy: FC::FuzzyController
}


impl FuzzyController {
    pub fn new() -> Self {
        Self{
            fuzzy: FC::new_fuzzy_controller()
        }
    }

    pub fn update(&self, target: f32, current: f32, dt: f32) -> f32 {
        self.fuzzy.get_fuzzy_conclusion(current-target, dt)
    }
}

