//! Драйвер для управления сервоприводами через PWM

use crate::config::hardware::servo::{
    CENTER_PULSE_US, MAX_ANGLE_DEG, MAX_PULSE_US, MIN_PULSE_US, PWM_PERIOD_US,
};
use crate::utils::math::{constrain, map_range};
use embassy_rp::pwm::{Config as PwmConfig, Pwm, SetDutyCycle};
use embassy_time::{Duration, Timer};
use num_traits::{AsPrimitive, ToPrimitive};

/// Ошибки работы с сервоприводом
#[derive(Debug)]
pub enum ServoError {
    /// Неверный угол
    InvalidAngle,
    /// Неверная длительность импульса
    InvalidPulse,
    /// Ошибка конфигурации PWM
    PwmError,
}

impl defmt::Format for ServoError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ServoError::InvalidAngle => defmt::write!(fmt, "Servo: Invalid angle"),
            ServoError::InvalidPulse => defmt::write!(fmt, "Servo: Invalid pulse width"),
            ServoError::PwmError => defmt::write!(fmt, "Servo: PWM configuration error"),
        }
    }
}

/// Драйвер сервопривода
pub struct Servo {
    /// PWM канал
    pwm: Pwm<'static>,
    /// Минимальная длительность импульса (мкс)
    min_pulse: u16,
    /// Максимальная длительность импульса (мкс)
    max_pulse: u16,
    /// Центральная длительность импульса (мкс)
    center_pulse: u16,
    /// Текущая позиция (-1.0 до 1.0)
    current_position: f32,
    /// Инвертировать направление
    inverted: bool,
    /// Ограничение скорости движения (позиций в секунду)
    rate_limit: Option<f32>,
    /// Время последнего обновления для ограничения скорости
    last_update_time: Option<embassy_time::Instant>,
}

impl Servo {
    /// Создание нового сервопривода с настройками по умолчанию
    pub fn new(pwm: Pwm<'static>) -> Self {
        Self {
            pwm,
            min_pulse: MIN_PULSE_US,
            max_pulse: MAX_PULSE_US,
            center_pulse: CENTER_PULSE_US,
            current_position: 0.0,
            inverted: false,
            rate_limit: None,
            last_update_time: None,
        }
    }

    /// Создание сервопривода с пользовательскими настройками
    pub fn with_config(
        pwm: Pwm<'static>,
        min_pulse: u16,
        max_pulse: u16,
        center_pulse: u16,
    ) -> Self {
        Self {
            pwm,
            min_pulse,
            max_pulse,
            center_pulse,
            current_position: 0.0,
            inverted: false,
            rate_limit: None,
            last_update_time: None,
        }
    }

    /// Инициализация сервопривода
    pub async fn init(&mut self) -> Result<(), ServoError> {
        // Настройка PWM для сервопривода
        // Частота = 50Hz (период 20ms)
        // Для RP2040 с частотой 125MHz:
        // divider = 125, top = 20000 -> 125MHz / 125 / 20000 = 50Hz
        let mut config = PwmConfig::default();
        config.divider = 125.into();
        config.top = PWM_PERIOD_US as u16; // 20000 для 20ms периода

        self.pwm.set_config(&config);

        // Устанавливаем центральное положение
        self.set_pulse_width(self.center_pulse)?;

        // Даем время сервоприводу переместиться в центр
        Timer::after(Duration::from_millis(500)).await;

        Ok(())
    }

    /// Установка позиции сервопривода (-1.0 до 1.0)
    /// -1.0 = крайнее левое/нижнее положение
    ///  0.0 = центральное положение  
    ///  1.0 = крайнее правое/верхнее положение
    pub async fn set_position(&mut self, position: f32) -> Result<(), ServoError> {
        // Ограничиваем входное значение
        let position = constrain(position, -1.0, 1.0);

        // Применяем инверсию если необходимо
        let position = if self.inverted { -position } else { position };

        // Применяем ограничение скорости если установлено
        let target_position = if let Some(rate) = self.rate_limit {
            self.apply_rate_limit(position, rate).await
        } else {
            position
        };

        // Преобразуем позицию в длительность импульса
        let pulse_width = if target_position >= 0.0 {
            // Положительная позиция: от центра до максимума
            map_range(
                target_position,
                0.0,
                1.0,
                self.center_pulse as f32,
                self.max_pulse as f32,
            ) as u16
        } else {
            // Отрицательная позиция: от минимума до центра
            map_range(
                target_position,
                -1.0,
                0.0,
                self.min_pulse as f32,
                self.center_pulse as f32,
            ) as u16
        };

        // Устанавливаем длительность импульса
        self.set_pulse_width(pulse_width)?;
        self.current_position = target_position;

        Ok(())
    }

    /// Установка угла сервопривода в градусах
    pub async fn set_angle(&mut self, angle_deg: f32) -> Result<(), ServoError> {
        // Преобразуем угол в позицию (-1.0 до 1.0)
        let position = angle_deg / MAX_ANGLE_DEG;
        self.set_position(position).await
    }

    /// Установка длительности импульса напрямую (в микросекундах)
    pub fn set_pulse_width(&mut self, pulse_us: u16) -> Result<(), ServoError> {
        // Проверяем допустимость значения
        if pulse_us < self.min_pulse || pulse_us > self.max_pulse {
            return Err(ServoError::InvalidPulse);
        }

        // Устанавливаем коэффициент заполнения PWM
        // duty = (pulse_us / period_us) * top
        let duty = ((pulse_us as u32 * self.pwm.max_duty_cycle() as u32) / PWM_PERIOD_US) as u16;
        self.pwm.set_duty_cycle(duty);
        // TODO Unused Result<(), PwmError> that must be used
        defmt::trace!("Серво: импульс {} мкс, duty {}", pulse_us, duty);

        Ok(())
    }

    /// Плавное перемещение в заданную позицию
    pub async fn move_to(
        &mut self,
        target_position: f32,
        duration_ms: u32,
    ) -> Result<(), ServoError> {
        let start_position = self.current_position;
        let steps = (duration_ms / 20).max(1); // Обновление каждые 20ms

        for i in 0..=steps {
            let progress = i as f32 / steps as f32;
            let position = start_position + (target_position - start_position) * progress;

            self.set_position(position).await?;
            Timer::after(Duration::from_millis(20)).await;
        }

        Ok(())
    }

    /// Установка инверсии направления
    pub fn set_inverted(&mut self, inverted: bool) {
        self.inverted = inverted;
    }

    /// Установка ограничения скорости движения
    pub fn set_rate_limit(&mut self, rate_limit: Option<f32>) {
        self.rate_limit = rate_limit;
        if rate_limit.is_none() {
            self.last_update_time = None;
        }
    }

    /// Калибровка сервопривода
    pub async fn calibrate(&mut self) -> Result<(), ServoError> {
        defmt::info!("Начало калибровки сервопривода");

        // Движение в крайние положения для проверки диапазона
        defmt::info!("Движение в левое крайнее положение");
        self.set_position(-1.0).await?;
        Timer::after(Duration::from_millis(3000)).await;

        defmt::info!("Движение в центр");
        self.set_position(0.0).await?;
        Timer::after(Duration::from_millis(3000)).await;

        defmt::info!("Движение в правое крайнее положение");
        self.set_position(1.0).await?;
        Timer::after(Duration::from_millis(3000)).await;

        defmt::info!("Возврат в центр");
        self.set_position(0.0).await?;

        defmt::info!("Калибровка завершена");
        Ok(())
    }

    /// Получение текущей позиции
    pub fn get_position(&self) -> f32 {
        self.current_position
    }

    /// Получение текущего угла в градусах
    pub fn get_angle(&self) -> f32 {
        self.current_position * MAX_ANGLE_DEG
    }

    /// Отключение сервопривода (снятие управляющего сигнала)
    pub fn disable(&mut self) {
        self.pwm.set_duty_cycle(0);
    }

    /// Применение ограничения скорости
    async fn apply_rate_limit(&mut self, target: f32, rate: f32) -> f32 {
        let now = embassy_time::Instant::now();

        if let Some(last_time) = self.last_update_time {
            let dt = (now - last_time).as_secs() as f32;
            let max_change = rate * dt;

            let delta = target - self.current_position;
            let limited_delta = constrain(delta, -max_change, max_change);

            self.last_update_time = Some(now);
            self.current_position + limited_delta
        } else {
            self.last_update_time = Some(now);
            target
        }
    }
}

/// Группа сервоприводов для синхронного управления
pub struct ServoGroup<const N: usize> {
    servos: heapless::Vec<Servo, N>,
}

impl<const N: usize> ServoGroup<N> {
    /// Создание новой группы сервоприводов
    pub fn new() -> Self {
        Self {
            servos: heapless::Vec::new(),
        }
    }

    /// Добавление сервопривода в группу
    pub fn add_servo(&mut self, servo: Servo) -> Result<(), Servo> {
        self.servos.push(servo)
    }

    /// Установка позиций для всех сервоприводов
    pub async fn set_positions(&mut self, positions: &[f32]) -> Result<(), ServoError> {
        if positions.len() != self.servos.len() {
            return Err(ServoError::InvalidAngle);
        }

        for (servo, &position) in self.servos.iter_mut().zip(positions.iter()) {
            servo.set_position(position).await?;
        }

        Ok(())
    }

    /// Синхронное движение всех сервоприводов
    pub async fn move_all_to(
        &mut self,
        targets: &[f32],
        duration_ms: u32,
    ) -> Result<(), ServoError> {
        if targets.len() != self.servos.len() {
            return Err(ServoError::InvalidAngle);
        }

        let steps = (duration_ms / 20).max(1);
        let start_positions: heapless::Vec<f32, N> =
            self.servos.iter().map(|s| s.get_position()).collect();

        for i in 0..=steps {
            let progress = i as f32 / steps as f32;

            for (idx, servo) in self.servos.iter_mut().enumerate() {
                let position =
                    start_positions[idx] + (targets[idx] - start_positions[idx]) * progress;
                servo.set_position(position).await?;
            }

            Timer::after(Duration::from_millis(20)).await;
        }

        Ok(())
    }
}
