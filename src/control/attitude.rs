//! Контроллер стабилизации положения автожира

use crate::config::flight::{autogyro, pid, stabilization};
use crate::control::fuzzy::FuzzyController;
use crate::data::{ControlCommand, ImuData};
use crate::utils::math::{apply_expo, constrain};
use core::f32::consts::PI;

/// Контроллер стабилизации положения (attitude)
pub struct AttitudeController {
    // TODO для нечеткого контроллера эти переменные нужны вместе
    ///  контроллер угла крена
    roll_angle_pid: FuzzyController,
    ///  контроллер угловой скорости крена
    roll_rate_pid: FuzzyController,
    // /// PID контроллер угла тангажа
    // pitch_angle_pid: PidController,
    // /// PID контроллер угловой скорости тангажа
    // pitch_rate_pid: PidController,
    // /// PID контроллер угловой скорости рыскания
    // yaw_rate_pid: PidController,
    /// Целевые углы (радианы)
    target_roll: f32,
    target_pitch: f32,
    /// Целевая угловая скорость рыскания (рад/с)
    target_yaw_rate: f32,

    /// Базовый уровень газа
    base_throttle: f32,

    /// Время последнего обновления (для расчета dt)
    last_update_us: u64,
}

impl AttitudeController {
    /// Создание нового контроллера стабилизации
    pub fn new() -> Self {
        // Инициализация PID контроллеров с параметрами из конфигурации
        let roll_angle_pid = FuzzyController::new(
            // pid::roll_angle::KP,
            // pid::roll_angle::KI,
            // pid::roll_angle::KD,
            // pid::roll_angle::I_LIMIT,
            // pid::roll_angle::OUTPUT_LIMIT,
        );

        let roll_rate_pid = FuzzyController::new(
            // pid::roll_rate::KP,
            // pid::roll_rate::KI,
            // pid::roll_rate::KD,
            // pid::roll_rate::I_LIMIT,
            // pid::roll_rate::OUTPUT_LIMIT,
        );
        //
        // let pitch_angle_pid = PidController::new(
        //     pid::pitch_angle::KP,
        //     pid::pitch_angle::KI,
        //     pid::pitch_angle::KD,
        //     pid::pitch_angle::I_LIMIT,
        //     pid::pitch_angle::OUTPUT_LIMIT,
        // );
        //
        // let pitch_rate_pid = PidController::new(
        //     pid::pitch_rate::KP,
        //     pid::pitch_rate::KI,
        //     pid::pitch_rate::KD,
        //     pid::pitch_rate::I_LIMIT,
        //     pid::pitch_rate::OUTPUT_LIMIT,
        // );
        //
        // let yaw_rate_pid = PidController::new(
        //     pid::yaw_rate::KP,
        //     pid::yaw_rate::KI,
        //     pid::yaw_rate::KD,
        //     pid::yaw_rate::I_LIMIT,
        //     pid::yaw_rate::OUTPUT_LIMIT,
        // );

        Self {
            roll_angle_pid,
            roll_rate_pid,
            // pitch_angle_pid,
            // pitch_rate_pid,
            // yaw_rate_pid,
            target_roll: 0.0,
            target_pitch: 0.0,
            target_yaw_rate: 0.0,
            base_throttle: 0.0,
            last_update_us: 0,
        }
    }

    /// Установка целевых углов и газа
    pub fn set_targets(
        &mut self,
        roll_deg: f32,
        pitch_deg: f32,
        yaw_rate_deg_s: f32,
        throttle: f32,
    ) {
        // Преобразуем градусы в радианы и применяем ограничения
        self.target_roll = constrain(
            roll_deg * PI / 180.0,
            -stabilization::MAX_ROLL_ANGLE_DEG * PI / 180.0,
            stabilization::MAX_ROLL_ANGLE_DEG * PI / 180.0,
        );

        self.target_pitch = constrain(
            pitch_deg * PI / 180.0,
            -stabilization::MAX_PITCH_ANGLE_DEG * PI / 180.0,
            stabilization::MAX_PITCH_ANGLE_DEG * PI / 180.0,
        );

        self.target_yaw_rate = constrain(
            yaw_rate_deg_s * PI / 180.0,
            -stabilization::MAX_ANGULAR_RATE_RAD_S,
            stabilization::MAX_ANGULAR_RATE_RAD_S,
        );

        self.base_throttle = constrain(throttle, 0.0, 1.0);
    }

    /// Расчет управляющих команд для режима стабилизации
    pub fn calculate_stabilize(&mut self, imu_data: &ImuData) -> ControlCommand {
        // Расчет времени с последнего обновления
        let dt = if self.last_update_us > 0 {
            (imu_data.timestamp_us - self.last_update_us) as f32 / 1_000_000.0
        } else {
            0.02 // Предполагаем 50Hz если это первый вызов
        };
        self.last_update_us = imu_data.timestamp_us;

        // Проверка на валидность dt
        let dt = constrain(dt, 0.001, 0.1);

        // === Контроль крена (каскадный PID) ===
        // Сначала PID по углу для получения целевой угловой скорости
        let target_roll_rate = self
            .roll_angle_pid
            .update(self.target_roll, imu_data.roll, dt);

        // Затем PID по угловой скорости для получения управляющего сигнала
        let roll_output = self
            .roll_rate_pid
            .update(target_roll_rate, imu_data.roll_rate, dt);
        //
        // // === Контроль тангажа (каскадный PID) ===
        // let target_pitch_rate = self.pitch_angle_pid.update(
        //     self.target_pitch,
        //     imu_data.pitch,
        //     dt,
        // );
        //
        // let pitch_output = self.pitch_rate_pid.update(
        //     target_pitch_rate,
        //     imu_data.pitch_rate,
        //     dt,
        // );
        //
        // // === Контроль рыскания (только по угловой скорости) ===
        // let yaw_output = self.yaw_rate_pid.update(
        //     self.target_yaw_rate,
        //     imu_data.yaw_rate,
        //     dt,
        // );

        // === Формирование команд управления ===
        self.generate_control_command(roll_output, 0.0, 0.0)
    }

    /// Расчет управляющих команд для режима взлета
    pub fn calculate_takeoff(&mut self, imu_data: &ImuData) -> ControlCommand {
        // При взлете удерживаем горизонтальное положение
        self.set_targets(0.0, 5.0, 0.0, self.base_throttle); // Небольшой наклон вперед

        // Используем стандартную стабилизацию с повышенной жесткостью
        let mut cmd = self.calculate_stabilize(imu_data);

        // Ограничиваем углы при взлете для безопасности
        cmd.cyclic_pitch = constrain(cmd.cyclic_pitch, -0.3, 0.3);
        cmd.cyclic_roll = constrain(cmd.cyclic_roll, -0.2, 0.2);

        cmd
    }

    /// Расчет управляющих команд для режима посадки
    pub fn calculate_landing(&mut self, imu_data: &ImuData) -> ControlCommand {
        // При посадке удерживаем строго горизонтальное положение
        self.set_targets(0.0, 0.0, 0.0, self.base_throttle);

        // Используем стандартную стабилизацию
        let mut cmd = self.calculate_stabilize(imu_data);

        // Еще больше ограничиваем углы при посадке
        cmd.cyclic_pitch = constrain(cmd.cyclic_pitch, -0.15, 0.15);
        cmd.cyclic_roll = constrain(cmd.cyclic_roll, -0.15, 0.15);

        cmd
    }

    /// Генерация команд управления из выходов PID контроллеров
    fn generate_control_command(
        &self,
        roll_out: f32,
        pitch_out: f32,
        yaw_out: f32,
    ) -> ControlCommand {
        // Применяем экспоненту для более плавного управления
        let roll_out = apply_expo(roll_out, stabilization::CONTROL_EXPO);
        let pitch_out = apply_expo(pitch_out, stabilization::CONTROL_EXPO);

        // Для автожира управление курсом через дифференциал тяги
        let yaw_differential = yaw_out * autogyro::YAW_THRUST_DIFFERENTIAL;

        // Расчет тяги моторов с учетом управления курсом
        let throttle_left = constrain(self.base_throttle - yaw_differential, 48.0, 2047.0);

        let throttle_right = constrain(self.base_throttle + yaw_differential, 48.0, 2047.0);

        // Компенсация тангажа в зависимости от тяги (особенность автожира)
        let pitch_compensation = self.base_throttle * autogyro::THRUST_TO_PITCH_RATIO;
        let cyclic_pitch = constrain(pitch_out + pitch_compensation, -1.0, 1.0);

        ControlCommand {
            throttle_left,
            throttle_right,
            cyclic_pitch,
            cyclic_roll: roll_out,
        }
    }

    /// Сброс интегральных составляющих PID контроллеров
    pub fn reset_integrators(&mut self) {
        // self.roll_angle_pid.reset();
        // self.roll_rate_pid.reset();
        // self.pitch_angle_pid.reset();
        // self.pitch_rate_pid.reset();
        // self.yaw_rate_pid.reset();
    }

    /// Установка базового уровня газа
    pub fn set_throttle(&mut self, throttle: f32) {
        self.base_throttle = constrain(throttle, 0.0, 1.0);
    }

    /// Получение текущих целевых углов (для телеметрии)
    pub fn get_targets(&self) -> (f32, f32, f32) {
        (
            self.target_roll * 180.0 / PI,
            self.target_pitch * 180.0 / PI,
            self.target_yaw_rate * 180.0 / PI,
        )
    }

    /// Проверка на критические углы
    pub fn check_attitude_limits(&self, imu_data: &ImuData) -> bool {
        let roll_deg = imu_data.roll * 180.0 / PI;
        let pitch_deg = imu_data.pitch * 180.0 / PI;

        // Проверяем превышение безопасных углов
        if roll_deg.abs() > crate::config::hardware::safety::MAX_ROLL_ANGLE_DEG {
            defmt::warn!("Превышен максимальный угол крена: {}", roll_deg);
            return false;
        }

        if pitch_deg.abs() > crate::config::hardware::safety::MAX_PITCH_ANGLE_DEG {
            defmt::warn!("Превышен максимальный угол тангажа: {}", pitch_deg);
            return false;
        }

        true
    }
}

// Тесты для отладки на хосте
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_attitude_controller_creation() {
        let controller = AttitudeController::new();
        let (roll, pitch, yaw_rate) = controller.get_targets();
        assert_eq!(roll, 0.0);
        assert_eq!(pitch, 0.0);
        assert_eq!(yaw_rate, 0.0);
    }

    #[test]
    fn test_target_limits() {
        let mut controller = AttitudeController::new();
        controller.set_targets(45.0, 30.0, 200.0, 0.5);

        let (roll, pitch, _) = controller.get_targets();
        assert!(roll <= stabilization::MAX_ROLL_ANGLE_DEG);
        assert!(pitch <= stabilization::MAX_PITCH_ANGLE_DEG);
    }
}
