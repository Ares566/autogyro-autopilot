//! Алгоритмы объединения данных с датчиков (sensor fusion)

use core::f32::consts::PI;
use crate::drivers::imu::mpu6500::Mpu6500Data;
use crate::utils::math::{normalize_angle, safe_sqrt};

/// Комплементарный фильтр для объединения данных IMU и магнитометра
pub struct ComplementaryFilter {
    /// Текущие углы Эйлера (радианы)
    roll: f32,
    pitch: f32,
    yaw: f32,

    /// Коэффициент фильтра (0.0-1.0, больше = больше доверия гироскопу)
    alpha: f32,

    /// Флаг первой итерации
    initialized: bool,

    /// Последнее время обновления для интегрирования
    last_update_time: u64,
}

/// Результат работы фильтра
#[derive(Debug, Clone, Copy)]
pub struct FusedData {
    /// Угол крена (радианы)
    pub roll: f32,
    /// Угол тангажа (радианы)
    pub pitch: f32,
    /// Угол рыскания (радианы)
    pub yaw: f32,
    /// Скорость изменения крена (рад/с)
    pub roll_rate: f32,
    /// Скорость изменения тангажа (рад/с)
    pub pitch_rate: f32,
    /// Скорость изменения рыскания (рад/с)
    pub yaw_rate: f32,
}

impl ComplementaryFilter {
    /// Создание нового фильтра
    pub fn new() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            alpha: 0.98, // 98% гироскоп, 2% акселерометр
            initialized: false,
            last_update_time: 0,
        }
    }

    /// Создание фильтра с заданным коэффициентом
    pub fn with_alpha(alpha: f32) -> Self {
        let alpha = alpha.clamp(0.0, 1.0);
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            alpha,
            initialized: false,
            last_update_time: 0,
        }
    }

    /// Обновление фильтра новыми данными
    pub fn update(
        &mut self,
        imu_data: &Mpu6500Data,
        mag_heading: Option<f32>,
        dt: f32,
    ) -> FusedData {
        // При первом вызове инициализируем углы из акселерометра
        if !self.initialized {
            self.roll = imu_data.roll;
            self.pitch = imu_data.pitch;
            self.yaw = mag_heading.unwrap_or(0.0);
            self.initialized = true;

            return FusedData {
                roll: self.roll,
                pitch: self.pitch,
                yaw: self.yaw,
                roll_rate: imu_data.gyro_x,
                pitch_rate: imu_data.gyro_y,
                yaw_rate: imu_data.gyro_z,
            };
        }

        // Интегрирование угловых скоростей гироскопа
        let gyro_roll = self.roll + imu_data.gyro_x * dt;
        let gyro_pitch = self.pitch + imu_data.gyro_y * dt;
        let gyro_yaw = self.yaw + imu_data.gyro_z * dt;

        // Получение углов из акселерометра
        let accel_roll = imu_data.roll;
        let accel_pitch = imu_data.pitch;

        // Комплементарная фильтрация для крена и тангажа
        self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll;
        self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch;

        // Для рыскания используем магнитометр если доступен
        if let Some(mag_yaw) = mag_heading {
            // Применяем фильтр с учетом кратчайшего пути
            let yaw_diff = self.shortest_angular_distance(self.yaw, mag_yaw);
            self.yaw = normalize_angle(self.yaw + (1.0 - self.alpha) * yaw_diff);
        } else {
            // Если магнитометр недоступен, используем только гироскоп
            self.yaw = normalize_angle(gyro_yaw);
        }

        // Ограничиваем углы
        self.roll = self.roll.clamp(-PI, PI);
        self.pitch = self.pitch.clamp(-PI/2.0, PI/2.0);

        FusedData {
            roll: self.roll,
            pitch: self.pitch,
            yaw: self.yaw,
            roll_rate: imu_data.gyro_x,
            pitch_rate: imu_data.gyro_y,
            yaw_rate: imu_data.gyro_z,
        }
    }

    /// Расчет кратчайшего углового расстояния
    fn shortest_angular_distance(&self, from: f32, to: f32) -> f32 {
        let diff = to - from;
        if diff > PI {
            diff - 2.0 * PI
        } else if diff < -PI {
            diff + 2.0 * PI
        } else {
            diff
        }
    }

    /// Установка коэффициента фильтра
    pub fn set_alpha(&mut self, alpha: f32) {
        self.alpha = alpha.clamp(0.0, 1.0);
    }

    /// Сброс фильтра
    pub fn reset(&mut self) {
        self.roll = 0.0;
        self.pitch = 0.0;
        self.yaw = 0.0;
        self.initialized = false;
    }

    /// Получение текущих углов
    pub fn get_angles(&self) -> (f32, f32, f32) {
        (self.roll, self.pitch, self.yaw)
    }
}

/// Расширенный фильтр Калмана для более точного объединения данных
pub struct ExtendedKalmanFilter {
    /// Вектор состояния [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
    state: [f32; 6],
    /// Ковариационная матрица (6x6, хранится как одномерный массив)
    covariance: [f32; 36],
    /// Шум процесса
    process_noise: [f32; 6],
    /// Шум измерений
    measurement_noise: [f32; 3],
}

impl ExtendedKalmanFilter {
    /// Создание нового EKF
    pub fn new() -> Self {
        let mut filter = Self {
            state: [0.0; 6],
            covariance: [0.0; 36],
            process_noise: [0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001],
            measurement_noise: [0.1, 0.1, 0.2],
        };

        // Инициализация диагональной ковариационной матрицы
        for i in 0..6 {
            filter.covariance[i * 6 + i] = 1.0;
        }

        filter
    }

    /// Шаг предсказания (prediction step)
    pub fn predict(&mut self, gyro_x: f32, gyro_y: f32, gyro_z: f32, dt: f32) {
        // Обновление состояния на основе гироскопа
        let roll_rate = gyro_x - self.state[3];  // С учетом смещения
        let pitch_rate = gyro_y - self.state[4];
        let yaw_rate = gyro_z - self.state[5];

        self.state[0] += roll_rate * dt;
        self.state[1] += pitch_rate * dt;
        self.state[2] += yaw_rate * dt;

        // Нормализация углов
        self.state[0] = normalize_angle(self.state[0]);
        self.state[1] = self.state[1].clamp(-PI/2.0, PI/2.0);
        self.state[2] = normalize_angle(self.state[2]);

        // Обновление ковариационной матрицы
        // P = F * P * F' + Q (упрощенная версия)
        for i in 0..6 {
            self.covariance[i * 6 + i] += self.process_noise[i] * dt;
        }
    }

    /// Шаг коррекции для акселерометра
    pub fn update_accel(&mut self, accel_roll: f32, accel_pitch: f32) {
        // Инновация (разница между измерением и предсказанием)
        let innovation_roll = accel_roll - self.state[0];
        let innovation_pitch = accel_pitch - self.state[1];

        // Упрощенное вычисление усиления Калмана
        let k_roll = self.covariance[0] / (self.covariance[0] + self.measurement_noise[0]);
        let k_pitch = self.covariance[7] / (self.covariance[7] + self.measurement_noise[1]);

        // Обновление состояния
        self.state[0] += k_roll * innovation_roll;
        self.state[1] += k_pitch * innovation_pitch;

        // Обновление ковариации
        self.covariance[0] *= 1.0 - k_roll;
        self.covariance[7] *= 1.0 - k_pitch;
    }

    /// Шаг коррекции для магнитометра
    pub fn update_mag(&mut self, mag_yaw: f32) {
        // Учитываем кратчайший путь
        let innovation_yaw = self.shortest_angular_distance(self.state[2], mag_yaw);

        let k_yaw = self.covariance[14] / (self.covariance[14] + self.measurement_noise[2]);

        self.state[2] += k_yaw * innovation_yaw;
        self.state[2] = normalize_angle(self.state[2]);

        self.covariance[14] *= 1.0 - k_yaw;
    }

    /// Получение отфильтрованных углов
    pub fn get_angles(&self) -> (f32, f32, f32) {
        (self.state[0], self.state[1], self.state[2])
    }

    /// Получение оценки смещения гироскопа
    pub fn get_gyro_bias(&self) -> (f32, f32, f32) {
        (self.state[3], self.state[4], self.state[5])
    }

    fn shortest_angular_distance(&self, from: f32, to: f32) -> f32 {
        let diff = to - from;
        if diff > PI {
            diff - 2.0 * PI
        } else if diff < -PI {
            diff + 2.0 * PI
        } else {
            diff
        }
    }
}

/// Простой фильтр нижних частот для сглаживания данных
pub struct LowPassFilter {
    cutoff_freq: f32,
    alpha: f32,
    output: f32,
    initialized: bool,
}

impl LowPassFilter {
    /// Создание фильтра с заданной частотой среза
    pub fn new(cutoff_freq_hz: f32, sample_rate_hz: f32) -> Self {
        let rc = 1.0 / (2.0 * PI * cutoff_freq_hz);
        let dt = 1.0 / sample_rate_hz;
        let alpha = dt / (rc + dt);

        Self {
            cutoff_freq: cutoff_freq_hz,
            alpha,
            output: 0.0,
            initialized: false,
        }
    }

    /// Фильтрация значения
    pub fn filter(&mut self, input: f32) -> f32 {
        if !self.initialized {
            self.output = input;
            self.initialized = true;
        } else {
            self.output = self.alpha * input + (1.0 - self.alpha) * self.output;
        }

        self.output
    }

    /// Сброс фильтра
    pub fn reset(&mut self) {
        self.output = 0.0;
        self.initialized = false;
    }
}

/// Детектор неподвижности для калибровки
pub struct MotionDetector {
    window_size: usize,
    threshold: f32,
    buffer_x: heapless::Vec<f32, 32>,
    buffer_y: heapless::Vec<f32, 32>,
    buffer_z: heapless::Vec<f32, 32>,
}

impl MotionDetector {
    /// Создание детектора с заданным порогом
    pub fn new(threshold: f32) -> Self {
        Self {
            window_size: 20,
            threshold,
            buffer_x: heapless::Vec::new(),
            buffer_y: heapless::Vec::new(),
            buffer_z: heapless::Vec::new(),
        }
    }

    /// Обновление детектора новыми данными гироскопа
    pub fn update(&mut self, gyro_x: f32, gyro_y: f32, gyro_z: f32) -> bool {
        // Добавляем в буферы
        if self.buffer_x.len() >= self.window_size {
            self.buffer_x.remove(0);
            self.buffer_y.remove(0);
            self.buffer_z.remove(0);
        }

        let _ = self.buffer_x.push(gyro_x);
        let _ = self.buffer_y.push(gyro_y);
        let _ = self.buffer_z.push(gyro_z);

        // Проверяем, достаточно ли данных
        if self.buffer_x.len() < self.window_size {
            return false;
        }

        // Вычисляем стандартное отклонение
        let std_x = self.std_deviation(&self.buffer_x);
        let std_y = self.std_deviation(&self.buffer_y);
        let std_z = self.std_deviation(&self.buffer_z);

        // Проверяем, неподвижен ли датчик
        std_x < self.threshold && std_y < self.threshold && std_z < self.threshold
    }

    /// Вычисление стандартного отклонения
    fn std_deviation(&self, data: &heapless::Vec<f32, 32>) -> f32 {
        if data.len() < 2 {
            return 0.0;
        }

        let mean: f32 = data.iter().sum::<f32>() / data.len() as f32;
        let variance: f32 = data.iter()
            .map(|&x| ((x - mean)*(x - mean)) )
            .sum::<f32>() / (data.len() - 1) as f32;

        safe_sqrt(variance)
    }
}
