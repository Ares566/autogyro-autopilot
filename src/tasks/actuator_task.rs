use embassy_rp::peripherals::PIO0;

use crate::data::{CHANNELS, ControlCommand};
use crate::drivers::actuators::{servo::Servo, dshot::{DshotPio, DshotPioTrait}};


#[embassy_executor::task]
pub async fn task(
    mut motors: DshotPio<'static, 2, PIO0>,
    mut servo_pitch: Servo,
    mut servo_roll: Servo,
) {
    // Получаем приемник команд управления
    let mut control_receiver = CHANNELS.control_channel.receiver();

    loop {
        // Ждем команду управления (блокирующий вариант)
        let cmd = control_receiver.receive().await;

        // Применяем команды к исполнительным механизмам
        motors.throttle_clamp([cmd.throttle_left as u16,cmd.throttle_right as u16]);
        
        servo_pitch.set_position(cmd.cyclic_pitch);
        servo_roll.set_position(cmd.cyclic_roll);
    }
}
