// src/tasks/control_task.rs
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use crate::data::{CHANNELS, SYSTEM_STATE, ImuData, ControlCommand};

// TODO control && Fuzzy Logic
use crate::control::attitude::AttitudeController;
// use crate::control::fuzzy::FuzzyController;

/// Частота цикла управления (Гц)
const CONTROL_RATE_HZ: u32 = 50;

#[embassy_executor::task]
pub async fn task() {
    // Инициализация контроллеров
    let mut attitude_controller = AttitudeController::new();

    // Получаем приемник IMU данных и отправитель команд
    let mut imu_receiver = CHANNELS.imu_channel.receiver();
    let control_sender = CHANNELS.control_channel.sender();

    let mut ticker = embassy_time::Ticker::every(Duration::from_hz(CONTROL_RATE_HZ as u64));

    loop {
        ticker.next().await;

        // Проверяем режим полета
        let flight_mode = *SYSTEM_STATE.flight_mode.lock().await;
        
        // Если система не armed, отправляем нулевые команды
        if !SYSTEM_STATE.armed.load(core::sync::atomic::Ordering::Relaxed) {
            let _ = control_sender.try_send(ControlCommand {
                throttle_left: 0,
                throttle_right: 0,
                cyclic_pitch: 0.0,
                cyclic_roll: 0.0,
            });
            
            // TODO перевод в состояние armed=true
            //  после нужен очевидный для оператора self_test системы (новый SYSTEM_STATE.flight_mode): 
            //  на низких оборотах покрутить обоими моторами 
            //  + тест сервоприводов - круговое движением ротором в обе стороны
            continue;
        }

        // Получаем последние данные IMU (неблокирующий вариант)
        if let Ok(imu_data) = imu_receiver.try_receive() {
            // Рассчитываем управляющие воздействия в зависимости от режима
            let control_cmd = match flight_mode {
                crate::data::FlightMode::Stabilize => {
                    // Режим стабилизации - удержание горизонта
                    attitude_controller.calculate_stabilize(&imu_data)
                }
                crate::data::FlightMode::TakeOff => {
                    // Режим взлета
                    attitude_controller.calculate_takeoff(&imu_data)
                }
                crate::data::FlightMode::Landing => {
                    // Режим посадки
                    attitude_controller.calculate_landing(&imu_data)
                }
                _ => {
                    // Безопасные значения по умолчанию
                    ControlCommand {
                        throttle_left: 0,
                        throttle_right: 0,
                        cyclic_pitch: 0.0,
                        cyclic_roll: 0.0,
                    }
                }
            };

            // Отправляем команду управления
            if let Err(_) = control_sender.try_send(control_cmd) {
                defmt::warn!("Буфер команд управления переполнен");
            }
        }
    }
}
