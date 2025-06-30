//! Драйвер для магнитометра HMC5983
use cortex_m::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use embassy_rp::i2c::{I2c, Error as I2cError, Blocking};
use embassy_time::{Duration, Timer};
use core::f32::consts::PI;

/// Адрес HMC5983 на шине I2C
const HMC5983_ADDR: u8 = 0x1E;

/// Регистры HMC5983
#[allow(dead_code)]
mod regs {
    pub const CONFIG_A: u8 = 0x00;      // Конфигурация A
    pub const CONFIG_B: u8 = 0x01;      // Конфигурация B (усиление)
    pub const MODE: u8 = 0x02;          // Режим работы
    pub const DATA_X_MSB: u8 = 0x03;    // Данные X (старший байт)
    pub const DATA_X_LSB: u8 = 0x04;    // Данные X (младший байт)
    pub const DATA_Z_MSB: u8 = 0x05;    // Данные Z (старший байт)
    pub const DATA_Z_LSB: u8 = 0x06;    // Данные Z (младший байт)
    pub const DATA_Y_MSB: u8 = 0x07;    // Данные Y (старший байт)
    pub const DATA_Y_LSB: u8 = 0x08;    // Данные Y (младший байт)
    pub const STATUS: u8 = 0x09;        // Регистр статуса
    pub const ID_A: u8 = 0x0A;          // Идентификация A (должен быть 'H')
    pub const ID_B: u8 = 0x0B;          // Идентификация B (должен быть '4')
    pub const ID_C: u8 = 0x0C;          // Идентификация C (должен быть '3')
    pub const TEMP_MSB: u8 = 0x31;      // Температура (старший байт)
    pub const TEMP_LSB: u8 = 0x32;      // Температура (младший байт)
}

/// Частота измерений
#[derive(Debug, Clone, Copy)]
pub enum DataRate {
    /// 0.75 Hz
    Hz0_75 = 0x00,
    /// 1.5 Hz
    Hz1_5 = 0x04,
    /// 3 Hz
    Hz3 = 0x08,
    /// 7.5 Hz
    Hz7_5 = 0x0C,
    /// 15 Hz (по умолчанию)
    Hz15 = 0x10,
    /// 30 Hz
    Hz30 = 0x14,
    /// 75 Hz
    Hz75 = 0x18,
    /// 220 Hz (только в режиме single measurement)
    Hz220 = 0x1C,
}

/// Количество усреднений
#[derive(Debug, Clone, Copy)]
pub enum SampleAverage {
    /// 1 измерение (по умолчанию)
    Avg1 = 0x00,
    /// 2 измерения
    Avg2 = 0x20,
    /// 4 измерения
    Avg4 = 0x40,
    /// 8 измерений
    Avg8 = 0x60,
}

/// Диапазон измерений (усиление)
#[derive(Debug, Clone, Copy)]
pub enum GainRange {
    /// ±0.88 Га
    Ga0_88 = 0x00,
    /// ±1.3 Га (по умолчанию)
    Ga1_3 = 0x20,
    /// ±1.9 Га
    Ga1_9 = 0x40,
    /// ±2.5 Га
    Ga2_5 = 0x60,
    /// ±4.0 Га
    Ga4_0 = 0x80,
    /// ±4.7 Га
    Ga4_7 = 0xA0,
    /// ±5.6 Га
    Ga5_6 = 0xC0,
    /// ±8.1 Га
    Ga8_1 = 0xE0,
}

/// Режим работы
#[derive(Debug, Clone, Copy)]
pub enum OperatingMode {
    /// Непрерывное измерение
    Continuous = 0x00,
    /// Одиночное измерение
    Single = 0x01,
    /// Режим ожидания
    Idle = 0x02,
}

/// Ошибки HMC5983
#[derive(Debug)]
pub enum Hmc5983Error {
    /// Ошибка I2C
    I2c(I2cError),
    /// Неверный идентификатор устройства
    InvalidDevice,
    /// Переполнение данных
    DataOverflow,
    /// Ошибка калибровки
    CalibrationError,
}

impl defmt::Format for Hmc5983Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Hmc5983Error::I2c(_) => defmt::write!(fmt, "HMC5983: I2C error"),
            Hmc5983Error::InvalidDevice => defmt::write!(fmt, "HMC5983: Invalid device ID"),
            Hmc5983Error::DataOverflow => defmt::write!(fmt, "HMC5983: Data overflow"),
            Hmc5983Error::CalibrationError => defmt::write!(fmt, "HMC5983: Calibration error"),
        }
    }
}

impl From<I2cError> for Hmc5983Error {
    fn from(error: I2cError) -> Self {
        Hmc5983Error::I2c(error)
    }
}

/// Данные магнитометра
#[derive(Debug, Clone, Copy)]
pub struct MagData {
    /// Магнитное поле по X (мкТл)
    pub x: f32,
    /// Магнитное поле по Y (мкТл)
    pub y: f32,
    /// Магнитное поле по Z (мкТл)
    pub z: f32,
    /// Температура (°C)
    pub temperature: f32,
    /// Магнитный курс (радианы, 0 = север)
    pub heading: f32,
}

/// Драйвер HMC5983
pub struct Hmc5983 {
    addr: u8,
    gain_lsb_per_gauss: f32,
    /// Калибровка: смещение (hard iron)
    offset: (f32, f32, f32),
    /// Калибровка: масштаб (soft iron)
    scale: (f32, f32, f32),
    /// Магнитное склонение (радианы)
    declination: f32,
}

impl Hmc5983 {
    /// Создание нового экземпляра драйвера
    pub async fn new<I>(i2c: &mut I2c<'_, I, Blocking>) -> Result<Self, Hmc5983Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut hmc = Self {
            addr: HMC5983_ADDR,
            gain_lsb_per_gauss: 1090.0, // Для ±1.3 Га
            offset: (0.0, 0.0, 0.0),
            scale: (1.0, 1.0, 1.0),
            declination: 0.0, // Нужно установить для вашего региона
        };

        // Инициализация
        hmc.init(i2c).await?;

        Ok(hmc)
    }

    /// Инициализация HMC5983
    async fn init<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(), Hmc5983Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Проверка идентификатора
        let id_a = self.read_register(i2c, regs::ID_A).await?;
        let id_b = self.read_register(i2c, regs::ID_B).await?;
        let id_c = self.read_register(i2c, regs::ID_C).await?;

        if id_a != b'H' || id_b != b'4' || id_c != b'3' {
            defmt::error!("Неверный ID HMC5983: {} {} {}", id_a as char, id_b as char, id_c as char);
            return Err(Hmc5983Error::InvalidDevice);
        }

        defmt::info!("Обнаружен HMC5983");

        // Конфигурация A: 8 усреднений, 15Hz, нормальный режим измерения
        let config_a = SampleAverage::Avg8 as u8 | DataRate::Hz15 as u8 | 0x00;
        self.write_register(i2c, regs::CONFIG_A, config_a).await?;

        // Конфигурация B: усиление ±1.3 Га
        self.set_gain(i2c, GainRange::Ga1_3).await?;

        // Режим: непрерывное измерение
        self.write_register(i2c, regs::MODE, OperatingMode::Continuous as u8).await?;

        // Ждем первое измерение
        Timer::after(Duration::from_millis(70)).await;

        defmt::info!("HMC5983 инициализирован успешно");
        Ok(())
    }

    /// Установка усиления
    pub async fn set_gain<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, gain: GainRange) -> Result<(), Hmc5983Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        self.write_register(i2c, regs::CONFIG_B, gain as u8).await?;

        // Обновляем коэффициент преобразования LSB/Gauss
        self.gain_lsb_per_gauss = match gain {
            GainRange::Ga0_88 => 1370.0,
            GainRange::Ga1_3 => 1090.0,
            GainRange::Ga1_9 => 820.0,
            GainRange::Ga2_5 => 660.0,
            GainRange::Ga4_0 => 440.0,
            GainRange::Ga4_7 => 390.0,
            GainRange::Ga5_6 => 330.0,
            GainRange::Ga8_1 => 230.0,
        };

        Ok(())
    }

    /// Чтение данных магнитометра
    pub async fn read<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<MagData, Hmc5983Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Проверка готовности данных
        let status = self.read_register(i2c, regs::STATUS).await?;
        if (status & 0x01) == 0 {
            // Данные не готовы
            Timer::after(Duration::from_millis(10)).await;
        }

        // Чтение 6 байт данных магнитометра
        let mut buf = [0u8; 6];
        self.read_registers(i2c, regs::DATA_X_MSB, &mut buf).await?;

        // Преобразование (порядок: X, Z, Y в регистрах!)
        let x_raw = i16::from_be_bytes([buf[0], buf[1]]);
        let z_raw = i16::from_be_bytes([buf[2], buf[3]]);
        let y_raw = i16::from_be_bytes([buf[4], buf[5]]);

        // Проверка на переполнение (-4096 означает overflow)
        if x_raw == -4096 || y_raw == -4096 || z_raw == -4096 {
            return Err(Hmc5983Error::DataOverflow);
        }

        // Чтение температуры
        let temp_raw = self.read_temperature(i2c).await?;
        let temperature = temp_raw as f32 / 128.0 + 25.0; // Формула из datasheet

        // Преобразование в микротеслы (1 Гаусс = 100 мкТл)
        let mut x = (x_raw as f32 / self.gain_lsb_per_gauss) * 100.0;
        let mut y = (y_raw as f32 / self.gain_lsb_per_gauss) * 100.0;
        let mut z = (z_raw as f32 / self.gain_lsb_per_gauss) * 100.0;

        // Применение калибровки
        x = (x - self.offset.0) * self.scale.0;
        y = (y - self.offset.1) * self.scale.1;
        z = (z - self.offset.2) * self.scale.2;

        // Расчет магнитного курса (в горизонтальной плоскости)
        let mut heading = libm::atan2f(-y, x); // Минус Y из-за ориентации осей

        // Добавляем магнитное склонение
        heading += self.declination;

        // Нормализация в диапазон [0, 2π]
        if heading < 0.0 {
            heading += 2.0 * PI;
        } else if heading > 2.0 * PI {
            heading -= 2.0 * PI;
        }

        Ok(MagData {
            x,
            y,
            z,
            temperature,
            heading,
        })
    }

    /// Чтение температуры
    async fn read_temperature<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<i16, Hmc5983Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut buf = [0u8; 2];
        self.read_registers(i2c, regs::TEMP_MSB, &mut buf).await?;
        Ok(i16::from_be_bytes([buf[0], buf[1]]))
    }

    /// Калибровка магнитометра (hard/soft iron)
    pub async fn calibrate<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, duration_s: u32) -> Result<(), Hmc5983Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        defmt::info!("Начало калибровки магнитометра");
        defmt::info!("Вращайте устройство по всем осям в течение {} секунд", duration_s);

        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;
        let mut min_z = f32::MAX;
        let mut max_z = f32::MIN;

        let start = embassy_time::Instant::now();
        let mut samples = 0u32;

        // Временно отключаем текущую калибровку
        let old_offset = self.offset;
        let old_scale = self.scale;
        self.offset = (0.0, 0.0, 0.0);
        self.scale = (1.0, 1.0, 1.0);

        while start.elapsed().as_secs() < duration_s as u64 {
            if let Ok(data) = self.read(i2c).await {
                min_x = min_x.min(data.x);
                max_x = max_x.max(data.x);
                min_y = min_y.min(data.y);
                max_y = max_y.max(data.y);
                min_z = min_z.min(data.z);
                max_z = max_z.max(data.z);
                samples += 1;
            }

            Timer::after(Duration::from_millis(50)).await;
        }

        if samples < 50 {
            defmt::error!("Недостаточно данных для калибровки");
            self.offset = old_offset;
            self.scale = old_scale;
            return Err(Hmc5983Error::CalibrationError);
        }

        // Расчет смещений (hard iron)
        self.offset = (
            (max_x + min_x) / 2.0,
            (max_y + min_y) / 2.0,
            (max_z + min_z) / 2.0,
        );

        // Расчет масштабов (soft iron, упрощенный)
        let avg_delta_x = max_x - min_x;
        let avg_delta_y = max_y - min_y;
        let avg_delta_z = max_z - min_z;
        let avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0;

        if avg_delta_x > 0.0 && avg_delta_y > 0.0 && avg_delta_z > 0.0 {
            self.scale = (
                avg_delta / avg_delta_x,
                avg_delta / avg_delta_y,
                avg_delta / avg_delta_z,
            );
        }

        defmt::info!("Калибровка завершена:");
        defmt::info!("  Смещения: X={}, Y={}, Z={}", self.offset.0, self.offset.1, self.offset.2);
        defmt::info!("  Масштабы: X={}, Y={}, Z={}", self.scale.0, self.scale.1, self.scale.2);
        defmt::info!("  Собрано {} образцов", samples);

        Ok(())
    }

    /// Установка магнитного склонения для вашего региона
    pub fn set_declination(&mut self, declination_deg: f32) {
        self.declination = declination_deg * PI / 180.0;
        defmt::info!("Установлено магнитное склонение: {}°", declination_deg);
    }

    /// Самотестирование
    pub async fn self_test<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>) -> Result<bool, Hmc5983Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        defmt::info!("Запуск самотестирования HMC5983");

        // Включаем режим положительного самотестирования
        let config_a = SampleAverage::Avg8 as u8 | DataRate::Hz15 as u8 | 0x01;
        self.write_register(i2c, regs::CONFIG_A, config_a).await?;

        Timer::after(Duration::from_millis(100)).await;

        // Читаем данные с включенным самотестированием
        let data_pos = self.read(i2c).await?;

        // Включаем режим отрицательного самотестирования
        let config_a = SampleAverage::Avg8 as u8 | DataRate::Hz15 as u8 | 0x02;
        self.write_register(i2c, regs::CONFIG_A, config_a).await?;

        Timer::after(Duration::from_millis(100)).await;

        // Читаем данные
        let data_neg = self.read(i2c).await?;

        // Возвращаем нормальный режим
        let config_a = SampleAverage::Avg8 as u8 | DataRate::Hz15 as u8 | 0x00;
        self.write_register(i2c, regs::CONFIG_A, config_a).await?;

        // Проверяем результаты (ожидаемые диапазоны из datasheet)
        let diff_x = (data_pos.x - data_neg.x).abs();
        let diff_y = (data_pos.y - data_neg.y).abs();
        let diff_z = (data_pos.z - data_neg.z).abs();

        defmt::info!("Результаты самотестирования:");
        defmt::info!("  Разница X: {} мкТл", diff_x);
        defmt::info!("  Разница Y: {} мкТл", diff_y);
        defmt::info!("  Разница Z: {} мкТл", diff_z);

        // Проверяем, что разница в ожидаемых пределах
        let test_passed = diff_x > 100.0 && diff_x < 500.0 &&
            diff_y > 100.0 && diff_y < 500.0 &&
            diff_z > 100.0 && diff_z < 500.0;

        if test_passed {
            defmt::info!("Самотестирование ПРОЙДЕНО");
        } else {
            defmt::warn!("Самотестирование НЕ ПРОЙДЕНО");
        }

        Ok(test_passed)
    }

    // Вспомогательные методы I2C
    async fn read_register<I>(&self, i2c: &mut I2c<'_, I, Blocking>, reg: u8) -> Result<u8, I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut buf = [0u8; 1];
        i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    async fn write_register<I>(&self, i2c: &mut I2c<'_, I, Blocking>, reg: u8, value: u8) -> Result<(), I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        i2c.write(self.addr, &[reg, value])
    }

    async fn read_registers<I>(&self, i2c: &mut I2c<'_, I, Blocking>, start_reg: u8, buf: &mut [u8]) -> Result<(), I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        i2c.write_read(self.addr, &[start_reg], buf)
    }
}
