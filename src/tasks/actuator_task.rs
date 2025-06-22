use embassy_rp::peripherals::PIO0;
use embassy_time::{Duration, Instant};
use crate::data::{CHANNELS, ControlCommand, SYSTEM_STATE, FlightMode};
use crate::drivers::actuators::{servo::Servo, dshot::{DshotPio, DshotPioTrait}};

/// Частота обновления исполнительных механизмов (Гц)
const ACTUATOR_UPDATE_RATE_HZ: u32 = 50;

/// Состояние исполнительных механизмов
struct ActuatorState {
    /// Последнее время обновления
    last_update: Instant,
    /// Счетчик отказов
    failure_count: u32,
    /// Флаг аварийной остановки
    emergency_stop: bool,
    /// Последняя валидная команда
    last_valid_command: ControlCommand,
}

#[embassy_executor::task]
pub async fn task(
    mut motors: DshotPio<'static, 2, PIO0>,
    mut servo_pitch: Servo,
    mut servo_roll: Servo,
) {

    defmt::info!("Запуск задачи управления исполнительными механизмами");

    // Инициализация состояния
    let mut state = ActuatorState {
        last_update: Instant::now(),
        failure_count: 0,
        emergency_stop: false,
        last_valid_command: ControlCommand {
            throttle_left: 0.0,
            throttle_right: 0.0,
            cyclic_pitch: 0.0,
            cyclic_roll: 0.0,
        },
    };
    
    // Получаем приемник команд управления
    let mut control_receiver = CHANNELS.control_channel.receiver();

    // Основной цикл
    let mut ticker = embassy_time::Ticker::every(Duration::from_hz(ACTUATOR_UPDATE_RATE_HZ as u64));

    loop {
        ticker.next().await;

        // Проверка аварийного режима
        if let Ok(flight_mode) = SYSTEM_STATE.flight_mode.try_lock() {
            if *flight_mode == FlightMode::Emergency {
                state.emergency_stop = true;
            }
        }

        // Ждем команду управления (блокирующий вариант)
        let cmd = control_receiver.receive().await;

        // Применяем команды к исполнительным механизмам
        motors.throttle_clamp([cmd.throttle_left as u16,cmd.throttle_right as u16]);
        
        servo_pitch.set_position(cmd.cyclic_pitch);
        servo_roll.set_position(cmd.cyclic_roll);
    }
}
