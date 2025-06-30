//! Цифровые фильтры для обработки сигналов

use core::f32::consts::PI;
use heapless::Vec;
use crate::utils::math::safe_sqrt;

/// Фильтр нижних частот первого порядка (RC-фильтр)
#[derive(Debug, Clone)]
pub struct LowPassFilter {
    /// Частота среза (Гц)
    cutoff_freq: f32,
    /// Частота дискретизации (Гц)
    sample_rate: f32,
    /// Коэффициент фильтра (0.0-1.0)
    alpha: f32,
    /// Текущее выходное значение
    output: f32,
    /// Флаг инициализации
    initialized: bool,
}

impl LowPassFilter {
    /// Создание нового фильтра
    ///
    /// # Параметры
    /// * `cutoff_freq_hz` - частота среза в Гц
    /// * `sample_rate_hz` - частота дискретизации в Гц
    pub fn new(cutoff_freq_hz: f32, sample_rate_hz: f32) -> Self {
        // Вычисляем коэффициент фильтра
        // alpha = dt / (RC + dt), где RC = 1 / (2 * pi * fc)
        let dt = 1.0 / sample_rate_hz;
        let rc = 1.0 / (2.0 * PI * cutoff_freq_hz);
        let alpha = dt / (rc + dt);

        Self {
            cutoff_freq: cutoff_freq_hz,
            sample_rate: sample_rate_hz,
            alpha: alpha.clamp(0.0, 1.0),
            output: 0.0,
            initialized: false,
        }
    }

    /// Фильтрация одного значения
    pub fn filter(&mut self, input: f32) -> f32 {
        if !self.initialized {
            // При первом вызове просто запоминаем значение
            self.output = input;
            self.initialized = true;
        } else {
            // Применяем фильтр: y[n] = α * x[n] + (1 - α) * y[n-1]
            self.output = self.alpha * input + (1.0 - self.alpha) * self.output;
        }

        self.output
    }

    /// Получение текущего выходного значения без обновления
    pub fn get_output(&self) -> f32 {
        self.output
    }

    /// Сброс фильтра
    pub fn reset(&mut self) {
        self.output = 0.0;
        self.initialized = false;
    }

    /// Установка нового значения без фильтрации
    pub fn set_output(&mut self, value: f32) {
        self.output = value;
        self.initialized = true;
    }

    /// Изменение частоты среза
    pub fn set_cutoff_freq(&mut self, cutoff_freq_hz: f32) {
        self.cutoff_freq = cutoff_freq_hz;
        let dt = 1.0 / self.sample_rate;
        let rc = 1.0 / (2.0 * PI * cutoff_freq_hz);
        self.alpha = (dt / (rc + dt)).clamp(0.0, 1.0);
    }

    /// Получение текущей частоты среза
    pub fn get_cutoff_freq(&self) -> f32 {
        self.cutoff_freq
    }
}

/// Фильтр нижних частот второго порядка (Butterworth)
#[derive(Debug, Clone)]
pub struct ButterworthFilter {
    /// Частота среза (Гц)
    cutoff_freq: f32,
    /// Частота дискретизации (Гц)
    sample_rate: f32,
    /// Коэффициенты фильтра
    a0: f32,
    a1: f32,
    a2: f32,
    b1: f32,
    b2: f32,
    /// Предыдущие входные значения
    x1: f32,
    x2: f32,
    /// Предыдущие выходные значения
    y1: f32,
    y2: f32,
}

impl ButterworthFilter {
    /// Создание нового фильтра Баттерворта 2-го порядка
    pub fn new(cutoff_freq_hz: f32, sample_rate_hz: f32) -> Self {
        let mut filter = Self {
            cutoff_freq: cutoff_freq_hz,
            sample_rate: sample_rate_hz,
            a0: 0.0,
            a1: 0.0,
            a2: 0.0,
            b1: 0.0,
            b2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        };

        filter.calculate_coefficients();
        filter
    }

    /// Вычисление коэффициентов фильтра
    fn calculate_coefficients(&mut self) {
        let omega = 2.0 * PI * self.cutoff_freq / self.sample_rate;
        let cos_omega = libm::cosf(omega);
        let sin_omega = libm::sinf(omega);
        let alpha = sin_omega / core::f32::consts::SQRT_2; // Q = 0.707 для Баттерворта

        let a0_inv = 1.0 / (1.0 + alpha);

        self.a0 = (1.0 - cos_omega) / 2.0 * a0_inv;
        self.a1 = (1.0 - cos_omega) * a0_inv;
        self.a2 = self.a0;
        self.b1 = -2.0 * cos_omega * a0_inv;
        self.b2 = (1.0 - alpha) * a0_inv;
    }

    /// Фильтрация одного значения
    pub fn filter(&mut self, input: f32) -> f32 {
        // Разностное уравнение:
        // y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
        let output = self.a0 * input + self.a1 * self.x1 + self.a2 * self.x2
            - self.b1 * self.y1 - self.b2 * self.y2;

        // Сдвиг истории
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = output;

        output
    }

    /// Сброс фильтра
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
}

/// Фильтр верхних частот первого порядка
#[derive(Debug, Clone)]
pub struct HighPassFilter {
    /// Частота среза (Гц)
    cutoff_freq: f32,
    /// Коэффициент фильтра
    alpha: f32,
    /// Предыдущее входное значение
    last_input: f32,
    /// Текущее выходное значение
    output: f32,
    /// Флаг инициализации
    initialized: bool,
}

impl HighPassFilter {
    /// Создание нового фильтра верхних частот
    pub fn new(cutoff_freq_hz: f32, sample_rate_hz: f32) -> Self {
        let dt = 1.0 / sample_rate_hz;
        let rc = 1.0 / (2.0 * PI * cutoff_freq_hz);
        let alpha = rc / (rc + dt);

        Self {
            cutoff_freq: cutoff_freq_hz,
            alpha: alpha.clamp(0.0, 1.0),
            last_input: 0.0,
            output: 0.0,
            initialized: false,
        }
    }

    /// Фильтрация одного значения
    pub fn filter(&mut self, input: f32) -> f32 {
        if !self.initialized {
            self.last_input = input;
            self.output = 0.0;
            self.initialized = true;
        } else {
            // HPF: y[n] = α * (y[n-1] + x[n] - x[n-1])
            self.output = self.alpha * (self.output + input - self.last_input);
            self.last_input = input;
        }

        self.output
    }

    /// Сброс фильтра
    pub fn reset(&mut self) {
        self.last_input = 0.0;
        self.output = 0.0;
        self.initialized = false;
    }
}

/// Полосовой фильтр (комбинация LPF и HPF)
#[derive(Debug, Clone)]
pub struct BandPassFilter {
    low_pass: ButterworthFilter,
    high_pass: HighPassFilter,
}

impl BandPassFilter {
    /// Создание полосового фильтра
    pub fn new(low_freq_hz: f32, high_freq_hz: f32, sample_rate_hz: f32) -> Self {
        Self {
            low_pass: ButterworthFilter::new(high_freq_hz, sample_rate_hz),
            high_pass: HighPassFilter::new(low_freq_hz, sample_rate_hz),
        }
    }

    /// Фильтрация значения
    pub fn filter(&mut self, input: f32) -> f32 {
        // Сначала применяем LPF, затем HPF
        let low_passed = self.low_pass.filter(input);
        self.high_pass.filter(low_passed)
    }

    /// Сброс фильтра
    pub fn reset(&mut self) {
        self.low_pass.reset();
        self.high_pass.reset();
    }
}

/// Медианный фильтр для удаления выбросов
#[derive(Debug, Clone)]
pub struct MedianFilter<const N: usize> {
    buffer: Vec<f32, N>,
    sorted: Vec<f32, N>,
}

impl<const N: usize> MedianFilter<N> {
    /// Создание нового медианного фильтра
    pub fn new() -> Self {
        Self {
            buffer: Vec::new(),
            sorted: Vec::new(),
        }
    }

    /// Добавление значения и получение медианы
    pub fn filter(&mut self, input: f32) -> f32 {
        // Добавляем новое значение
        if self.buffer.len() >= N {
            self.buffer.remove(0);
        }
        let _ = self.buffer.push(input);

        // Если недостаточно данных, возвращаем последнее значение
        if self.buffer.is_empty() {
            return input;
        }

        // Копируем и сортируем
        self.sorted.clear();
        for &val in self.buffer.iter() {
            let _ = self.sorted.push(val);
        }

        // Сортировка пузырьком (для малых массивов эффективно)
        let len = self.sorted.len();
        for i in 0..len {
            for j in 0..len - i - 1 {
                if self.sorted[j] > self.sorted[j + 1] {
                    self.sorted.swap(j, j + 1);
                }
            }
        }

        // Возвращаем медиану
        if len % 2 == 0 {
            (self.sorted[len / 2 - 1] + self.sorted[len / 2]) / 2.0
        } else {
            self.sorted[len / 2]
        }
    }

    /// Сброс фильтра
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.sorted.clear();
    }
}

/// Скользящее среднее
#[derive(Debug, Clone)]
pub struct MovingAverage<const N: usize> {
    buffer: Vec<f32, N>,
    sum: f32,
}

impl<const N: usize> MovingAverage<N> {
    /// Создание нового фильтра скользящего среднего
    pub fn new() -> Self {
        Self {
            buffer: Vec::new(),
            sum: 0.0,
        }
    }

    /// Добавление значения и получение среднего
    pub fn filter(&mut self, input: f32) -> f32 {
        // Если буфер полный, удаляем старое значение
        if self.buffer.len() >= N {
            self.sum -= self.buffer[0];
            self.buffer.remove(0);
        }

        // Добавляем новое значение
        let _ = self.buffer.push(input);
        self.sum += input;

        // Возвращаем среднее
        if self.buffer.is_empty() {
            0.0
        } else {
            self.sum / self.buffer.len() as f32
        }
    }

    /// Получение текущего среднего без добавления нового значения
    pub fn get_average(&self) -> f32 {
        if self.buffer.is_empty() {
            0.0
        } else {
            self.sum / self.buffer.len() as f32
        }
    }

    /// Сброс фильтра
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.sum = 0.0;
    }
}

/// Фильтр для обнаружения пиков
#[derive(Debug, Clone)]
pub struct PeakDetector {
    threshold: f32,
    min_distance: usize,
    last_peak_index: usize,
    current_index: usize,
}

impl PeakDetector {
    /// Создание детектора пиков
    pub fn new(threshold: f32, min_distance: usize) -> Self {
        Self {
            threshold,
            min_distance,
            last_peak_index: 0,
            current_index: 0,
        }
    }

    /// Проверка, является ли значение пиком
    pub fn detect(&mut self, value: f32, previous: f32, next: f32) -> bool {
        self.current_index += 1;

        // Проверяем, что это локальный максимум
        let is_peak = value > previous && value > next && value > self.threshold;

        // Проверяем минимальное расстояние между пиками
        if is_peak && (self.current_index - self.last_peak_index) >= self.min_distance {
            self.last_peak_index = self.current_index;
            true
        } else {
            false
        }
    }

    /// Сброс детектора
    pub fn reset(&mut self) {
        self.last_peak_index = 0;
        self.current_index = 0;
    }
}

/// Адаптивный фильтр с изменяемой частотой среза
#[derive(Debug, Clone)]
pub struct AdaptiveFilter {
    /// Базовый фильтр
    filter: LowPassFilter,
    /// Минимальная частота среза
    min_cutoff: f32,
    /// Максимальная частота среза
    max_cutoff: f32,
    /// Детектор движения для адаптации
    motion_threshold: f32,
}

impl AdaptiveFilter {
    /// Создание адаптивного фильтра
    pub fn new(min_cutoff_hz: f32, max_cutoff_hz: f32, sample_rate_hz: f32) -> Self {
        Self {
            filter: LowPassFilter::new(min_cutoff_hz, sample_rate_hz),
            min_cutoff: min_cutoff_hz,
            max_cutoff: max_cutoff_hz,
            motion_threshold: 0.1,
        }
    }

    /// Фильтрация с адаптацией к скорости изменения сигнала
    pub fn filter(&mut self, input: f32, rate_of_change: f32) -> f32 {
        // Адаптируем частоту среза в зависимости от скорости изменения
        let normalized_rate = (rate_of_change.abs() / self.motion_threshold).clamp(0.0, 1.0);
        let cutoff = self.min_cutoff + (self.max_cutoff - self.min_cutoff) * normalized_rate;

        self.filter.set_cutoff_freq(cutoff);
        self.filter.filter(input)
    }

    /// Сброс фильтра
    pub fn reset(&mut self) {
        self.filter.reset();
    }
}

