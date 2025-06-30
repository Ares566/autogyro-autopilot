//! Обработка данных высоты и вертикальной скорости

use crate::utils::filters::LowPassFilter;

/// Обработчик данных высоты
pub struct AltitudeProcessor {
    /// Фильтр для барометрической высоты
    altitude_filter: LowPassFilter,
    /// Фильтр для вертикальной скорости
    vspeed_filter: LowPassFilter,
    /// Последняя высота для расчета скорости
    last_altitude: f32,
    /// Последнее время измерения
    last_time_us: u64,
    /// Базовое давление на уровне земли
    ground_pressure: f32,
    /// Калибровочное смещение высоты
    altitude_offset: f32,
}

impl AltitudeProcessor {
    pub fn new() -> Self {
        Self {
            altitude_filter: LowPassFilter::new(1.0, 25.0), // 1Hz cutoff @ 25Hz
            vspeed_filter: LowPassFilter::new(2.0, 25.0),   // 2Hz cutoff @ 25Hz
            last_altitude: 0.0,
            last_time_us: 0,
            ground_pressure: 101325.0,
            altitude_offset: 0.0,
        }
    }

    /// Обработка новых данных барометра
    pub fn update(&mut self, pressure: f32, temperature: f32, time_us: u64) -> (f32, f32) {
        // Расчет высоты по барометрической формуле
        let altitude_raw = 44330.0 * (1.0 - libm::powf(pressure / self.ground_pressure, 0.1903));
        let altitude = self.altitude_filter.filter(altitude_raw) - self.altitude_offset;

        // Расчет вертикальной скорости
        let vspeed = if self.last_time_us > 0 {
            let dt = (time_us - self.last_time_us) as f32 / 1_000_000.0;
            if dt > 0.0 && dt < 1.0 { // Защита от неправильных значений времени
                let vspeed_raw = (altitude - self.last_altitude) / dt;
                self.vspeed_filter.filter(vspeed_raw)
            } else {
                0.0
            }
        } else {
            0.0
        };

        self.last_altitude = altitude;
        self.last_time_us = time_us;

        (altitude, vspeed)
    }

    /// Калибровка нулевой высоты
    pub fn calibrate_ground_level(&mut self, current_pressure: f32) {
        self.ground_pressure = current_pressure;
        self.altitude_offset = 0.0;
        self.altitude_filter.reset();
        self.vspeed_filter.reset();
        defmt::info!("Высота откалибрована, давление на земле: {} Па", current_pressure);
    }

    /// Установка известной высоты (например, из GPS)
    pub fn set_altitude(&mut self, known_altitude: f32, current_pressure: f32) {
        let altitude_raw = 44330.0 * (1.0 - libm::powf(current_pressure / 101325.0, 0.1903));
        self.altitude_offset = altitude_raw - known_altitude;
    }
}
