// src/tasks/control_task.rs
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use crate::data::{ControlCommand, FlightMode, ImuData, CHANNELS, SYSTEM_STATE};

// TODO control && Fuzzy Logic
use crate::control::attitude::AttitudeController;
// use crate::control::fuzzy::FuzzyController;

/// Частота цикла управления (Гц)
pub(crate) const CONTROL_RATE_HZ: u32 = 50;

#[embassy_executor::task]
pub async fn task() {
    // Инициализация контроллеров
    let mut attitude_controller = AttitudeController::new();

    // Получаем приемник IMU данных и отправитель команд
    let mut imu_receiver = CHANNELS.imu_channel.receiver();
    let control_sender = CHANNELS.control_channel.sender();

    let mut ticker = embassy_time::Ticker::every(Duration::from_hz(CONTROL_RATE_HZ as u64));
    let mut armed_count = 0;
    loop {
        ticker.next().await;

        // Если система не armed, отправляем нулевые команды
        if !SYSTEM_STATE
            .armed
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            armed_count += 1;
            let _ = control_sender.try_send(ControlCommand {
                throttle_left: 0,
                throttle_right: 0,
                cyclic_pitch: 0.0,
                cyclic_roll: 0.0,
            });

            // через секунду моторы должны быть в состоянии armed
            if armed_count > 3 * CONTROL_RATE_HZ {
                SYSTEM_STATE
                    .armed
                    .store(true, core::sync::atomic::Ordering::Relaxed);
            }

            continue;
        }

        // Проверяем режим полета
        let flight_mode = *SYSTEM_STATE.flight_mode.lock().await;

        // считываем показатели высоты
        //if let Ok(imu_data) = altitude_receiver.try_receive() {}

        // Получаем последние данные IMU (неблокирующий вариант)
        if let Ok(imu_data) = imu_receiver.try_receive() {
            // Логирование для отладки
            #[cfg(feature = "debug-sensors")]
            defmt::debug!(
                "Sensors: P={} R={} Y={}",
                imu_data.pitch,
                imu_data.roll,
                imu_data.yaw,
            );

            // Рассчитываем управляющие воздействия в зависимости от режима
            let control_cmd = match flight_mode {
                crate::data::FlightMode::PreflightChecks => {
                    armed_count += 1;
                    let mut demo_throtle = 150;

                    if armed_count > 9 * CONTROL_RATE_HZ {
                        demo_throtle = 0;
                        *SYSTEM_STATE.flight_mode.lock().await = FlightMode::Disarmed;
                    }
                    ControlCommand {
                        throttle_left: demo_throtle,
                        throttle_right: demo_throtle,
                        cyclic_pitch: 0.0,
                        cyclic_roll: 0.0,
                    }
                }
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
