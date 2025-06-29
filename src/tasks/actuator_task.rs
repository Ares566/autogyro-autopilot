use core::sync::atomic::Ordering;
use embassy_rp::peripherals::PIO0;
use embassy_time::{Duration, Instant, Timer};
use nalgebra::min;
use num_traits::AsPrimitive;
use num_traits::real::Real;
use crate::data::{CHANNELS, ControlCommand, SYSTEM_STATE, FlightMode};
use crate::drivers::actuators::{servo::Servo, dshot::{DshotPio, DshotPioTrait}};
use crate::utils::math::{constrain, constrain_i32, constrain_u16};

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
            throttle_left: 0,
            throttle_right: 0,
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
        
        // Получение команды управления
        let command = match control_receiver.try_receive() {
            Ok(cmd) => {
                state.failure_count = 0;
                state.last_valid_command = cmd;
                cmd
            }
            Err(_) => {
                // Используем последнюю валидную команду или безопасные значения
                state.failure_count += 1;

                if state.failure_count > ACTUATOR_UPDATE_RATE_HZ { // 1 секунда без команд
                    defmt::warn!("Потеря связи с контроллером управления!");
                    // Переходим в безопасный режим
                    ControlCommand {
                        throttle_left: 0,
                        throttle_right: 0,
                        cyclic_pitch: 0.0,
                        cyclic_roll: 0.0,
                    }
                } else {
                    state.last_valid_command
                }
            }
        };
        //defmt::debug!("Получены команды throttle_left{}, throttle_right {}", command.throttle_left, command.throttle_right);
        // Применение команд с учетом безопасности
        if state.emergency_stop  { //|| !SYSTEM_STATE.armed.load(Ordering::Relaxed)
            // Аварийная остановка // или система не взведена
            apply_safe_shutdown(
                &mut motors,
                &mut servo_pitch,
                &mut servo_roll,
                &mut state,
            ).await;
        } else {
            // Нормальное управление
            apply_control_command(
                command,
                &mut motors,
                &mut servo_pitch,
                &mut servo_roll,
                &mut state,
            ).await;
        }

        // Проверка состояния исполнительных механизмов
        check_actuator_health(&mut state).await;
    }
}

/// Применение команд управления к исполнительным механизмам
async fn apply_control_command(
    cmd: ControlCommand,
    motors: &mut DshotPio<'static, 2, PIO0>,
    servo_pitch: &mut Servo,
    servo_roll: &mut Servo,
    state: &mut ActuatorState,
) {
    // Проверка и ограничение значений команд
    let throttle_left = constrain_u16(cmd.throttle_left, 48, 2048);
    let throttle_right = constrain_u16(cmd.throttle_right, 48, 2048);
    let cyclic_pitch = constrain(cmd.cyclic_pitch, -1.0, 1.0);
    let cyclic_roll = constrain(cmd.cyclic_roll, -1.0, 1.0);
    
    // Применение сглаживания для плавности управления
    let dt: f32 = state.last_update.elapsed().as_secs().as_();
    state.last_update = Instant::now();

    // Ограничение скорости изменения газа (защита от резких рывков)
    const MAX_THROTTLE_RATE: f32 = 2.0; // максимум 200% в секунду
    let max_throttle_change = MAX_THROTTLE_RATE * dt;

    let smoothed_throttle_left = smooth_value(
        state.last_valid_command.throttle_left,
        throttle_left,
        max_throttle_change,
    );

    let smoothed_throttle_right = smooth_value(
        state.last_valid_command.throttle_right,
        throttle_right,
        max_throttle_change,
    );
    //defmt::debug!("smoothed_throttle_left{}, smoothed_throttle_right {}",smoothed_throttle_left, smoothed_throttle_right);
    // Установка газа моторов
    motors.throttle_clamp([smoothed_throttle_left,smoothed_throttle_right ]);

    // Установка позиций сервоприводов
    servo_pitch.set_position(cyclic_pitch);
    servo_roll.set_position(cyclic_roll);

    // Логирование для отладки
    #[cfg(feature = "debug-actuators")]
    defmt::debug!(
        "Actuators: L={} R={} P={} R={}",
        smoothed_throttle_left,
        smoothed_throttle_right,
        cyclic_pitch,
        cyclic_roll
    );
}

/// Безопасное отключение всех исполнительных механизмов
async fn apply_safe_shutdown(
    motors: &mut DshotPio<'static, 2, PIO0>,
    servo_pitch: &mut Servo,
    servo_roll: &mut Servo,
    state: &mut ActuatorState,
) {
    defmt::warn!("Применение безопасного отключения");

    // Плавное снижение оборотов моторов
    let last_throttle = min(state.last_valid_command.throttle_left, state.last_valid_command.throttle_right);
    let mut throttle = last_throttle;
    for i in (1..=10).rev() {

        throttle  = throttle - last_throttle * i / 100;
        motors.throttle_clamp([throttle, throttle]);
        Timer::after(Duration::from_millis(100)).await;
    }

    // Полная остановка моторов
    motors.throttle_minimum();


    // Центрирование сервоприводов
    servo_pitch.set_position(0.0);
    servo_roll.set_position(0.0);
}

/// Проверка состояния исполнительных механизмов
async fn check_actuator_health(state: &mut ActuatorState) {
    // Проверка количества отказов
    if state.failure_count > 10 {
        defmt::error!("Критическое количество отказов исполнительных механизмов!");

        // Переводим систему в аварийный режим
        if let Ok(mut mode) = SYSTEM_STATE.flight_mode.try_lock() {
            *mode = FlightMode::Emergency;
        }

        state.emergency_stop = true;
    }

    // Периодический сброс счетчика отказов
    if state.failure_count > 0 && state.last_update.elapsed().as_secs() > 5 {
        state.failure_count = state.failure_count.saturating_sub(1);
    }
}

/// Плавное изменение значения с ограничением скорости
fn smooth_value(current: u16, target: u16, max_change: f32) -> u16 {
    let diff= (target - current) as f32;
    if diff.abs() <= max_change {
        target
    } else {
        (current as f32 + max_change * diff.signum()) as u16
    }
}
