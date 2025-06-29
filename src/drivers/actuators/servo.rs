
#[derive(Debug)]
pub enum ServoError {
    /// Ошибка конфигурации
    ConfigError,
}

pub struct Servo{

}

impl Servo{
    pub fn new() -> Result<Self, ServoError>{
        Ok(Self{})
    }
    
    pub fn set_position(&self, p0: f32){
        return;
    }
}