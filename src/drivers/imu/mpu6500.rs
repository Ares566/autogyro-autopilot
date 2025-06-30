//! Драйвер для MPU6500 (6-осевой IMU без магнитометра)
use cortex_m::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use embassy_rp::i2c::{I2c, Error as I2cError, Blocking};
use embassy_time::{Duration, Timer, Instant};
use core::f32::consts::PI;

/// Адрес MPU6500 по умолчанию
const MPU6500_ADDR: u8 = 0x68;

/// Регистры MPU6500
#[allow(dead_code)]
mod regs {
    // Управление и конфигурация
    pub const PWR_MGMT_1: u8 = 0x6B;      // Управление питанием 1
    pub const PWR_MGMT_2: u8 = 0x6C;      // Управление питанием 2
    pub const CONFIG: u8 = 0x1A;           // Конфигурация
    pub const GYRO_CONFIG: u8 = 0x1B;     // Конфигурация гироскопа
    pub const ACCEL_CONFIG: u8 = 0x1C;    // Конфигурация акселерометра
    pub const ACCEL_CONFIG_2: u8 = 0x1D;  // Конфигурация акселерометра 2
    pub const SMPLRT_DIV: u8 = 0x19;      // Делитель частоты выборки
    pub const INT_PIN_CFG: u8 = 0x37;     // Конфигурация INT пина
    pub const INT_ENABLE: u8 = 0x38;      // Включение прерываний
    pub const INT_STATUS: u8 = 0x3A;      // Статус прерываний
    pub const WHO_AM_I: u8 = 0x75;        // Идентификатор устройства
    pub const SIGNAL_PATH_RESET: u8 = 0x68; // Сброс сигнального пути
    pub const USER_CTRL: u8 = 0x6A;       // Пользовательский контроль
    pub const FIFO_EN: u8 = 0x23;         // Включение FIFO
    
    // Регистры данных
    pub const ACCEL_XOUT_H: u8 = 0x3B;    // Старший байт X акселерометра
    pub const ACCEL_XOUT_L: u8 = 0x3C;    // Младший байт X акселерометра
    pub const ACCEL_YOUT_H: u8 = 0x3D;    // Старший байт Y акселерометра
    pub const ACCEL_YOUT_L: u8 = 0x3E;    // Младший байт Y акселерометра
    pub const ACCEL_ZOUT_H: u8 = 0x3F;    // Старший байт Z акселерометра
    pub const ACCEL_ZOUT_L: u8 = 0x40;    // Младший байт Z акселерометра
    pub const TEMP_OUT_H: u8 = 0x41;      // Старший байт температуры
    pub const TEMP_OUT_L: u8 = 0x42;      // Младший байт температуры
    pub const GYRO_XOUT_H: u8 = 0x43;     // Старший байт X гироскопа
    pub const GYRO_XOUT_L: u8 = 0x44;     // Младший байт X гироскопа
    pub const GYRO_YOUT_H: u8 = 0x45;     // Старший байт Y гироскопа
    pub const GYRO_YOUT_L: u8 = 0x46;     // Младший байт Y гироскопа
    pub const GYRO_ZOUT_H: u8 = 0x47;     // Старший байт Z гироскопа
    pub const GYRO_ZOUT_L: u8 = 0x48;     // Младший байт Z гироскопа
}

/// Диапазон измерения акселерометра
#[derive(Debug, Clone, Copy)]
pub enum AccelRange {
    /// ±2g
    G2 = 0x00,
    /// ±4g
    G4 = 0x08,
    /// ±8g
    G8 = 0x10,
    /// ±16g
    G16 = 0x18,
}

/// Диапазон измерения гироскопа
#[derive(Debug, Clone, Copy)]
pub enum GyroRange {
    /// ±250°/s
    Deg250 = 0x00,
    /// ±500°/s
    Deg500 = 0x08,
    /// ±1000°/s
    Deg1000 = 0x10,
    /// ±2000°/s
    Deg2000 = 0x18,
}

/// Полоса пропускания DLPF
#[derive(Debug, Clone, Copy)]
pub enum DlpfBandwidth {
    /// 260Hz акселерометр, 256Hz гироскоп
    Bw260Hz = 0,
    /// 184Hz акселерометр, 188Hz гироскоп
    Bw184Hz = 1,
    /// 94Hz акселерометр, 98Hz гироскоп
    Bw94Hz = 2,
    /// 44Hz акселерометр, 42Hz гироскоп
    Bw44Hz = 3,
    /// 21Hz акселерометр, 20Hz гироскоп
    Bw21Hz = 4,
    /// 10Hz акселерометр, 10Hz гироскоп
    Bw10Hz = 5,
    /// 5Hz акселерометр, 5Hz гироскоп
    Bw5Hz = 6,
}

/// Ошибки MPU6500
#[derive(Debug)]
pub enum Mpu6500Error {
    /// Ошибка I2C
    I2c(I2cError),
    /// Неверный идентификатор устройства
    InvalidDevice,
    /// Ошибка конфигурации
    ConfigError,
    /// Данные не готовы
    DataNotReady,
}

impl defmt::Format for Mpu6500Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Mpu6500Error::I2c(_) => defmt::write!(fmt, "MPU6500: I2C error"),
            Mpu6500Error::InvalidDevice => defmt::write!(fmt, "MPU6500: Invalid device ID"),
            Mpu6500Error::ConfigError => defmt::write!(fmt, "MPU6500: Configuration error"),
            Mpu6500Error::DataNotReady => defmt::write!(fmt, "MPU6500: Data not ready"),
        }
    }
}

impl From<I2cError> for Mpu6500Error {
    fn from(error: I2cError) -> Self {
        Mpu6500Error::I2c(error)
    }
}

/// Сырые данные с MPU6500
#[derive(Debug, Clone, Copy)]
pub struct RawData {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
    pub temp: i16,
}

/// Обработанные данные с MPU6500
#[derive(Debug, Clone, Copy)]
pub struct Mpu6500Data {
    /// Ускорение по X в м/с²
    pub accel_x: f32,
    /// Ускорение по Y в м/с²
    pub accel_y: f32,
    /// Ускорение по Z в м/с²
    pub accel_z: f32,
    /// Угловая скорость по X в рад/с
    pub gyro_x: f32,
    /// Угловая скорость по Y в рад/с
    pub gyro_y: f32,
    /// Угловая скорость по Z в рад/с
    pub gyro_z: f32,
    /// Температура в °C
    pub temperature: f32,
    /// Расчетные углы (из акселерометра)
    pub roll: f32,
    pub pitch: f32,
}

/// Драйвер MPU6500
pub struct Mpu6500 {
    /// Адрес устройства
    addr: u8,
    /// Диапазон акселерометра
    accel_range: AccelRange,
    /// Диапазон гироскопа
    gyro_range: GyroRange,
    /// Масштабный коэффициент для акселерометра
    accel_scale: f32,
    /// Масштабный коэффициент для гироскопа
    gyro_scale: f32,
    /// Смещения гироскопа для калибровки
    gyro_offset: (f32, f32, f32),
    /// Смещения акселерометра для калибровки
    accel_offset: (f32, f32, f32),
}

impl Mpu6500 {
    /// Создание нового экземпляра драйвера
    pub async fn new<I>(i2c: &mut I2c<'_, I, Blocking>) -> Result<Self, Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut mpu = Self {
            addr: MPU6500_ADDR,
            accel_range: AccelRange::G2,
            gyro_range: GyroRange::Deg250,
            accel_scale: 16384.0, // Для ±2g
            gyro_scale: 131.0,    // Для ±250°/s
            gyro_offset: (0.0, 0.0, 0.0),
            accel_offset: (0.0, 0.0, 0.0),
        };
        
        // Инициализация устройства
        mpu.init(i2c).await?;
        
        Ok(mpu)
    }
    
    /// Инициализация MPU6500
    async fn init<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(), Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Проверка идентификатора устройства
        let who_am_i = self.read_register(i2c, regs::WHO_AM_I).await?;
        // MPU6500 может возвращать 0x70, 0x71, 0x73 или другие значения
        if who_am_i != 0x70 && who_am_i != 0x71 && who_am_i != 0x73 && who_am_i != 0x68 {
            defmt::warn!("Неожиданный ID MPU6500: 0x{:02x}, продолжаем...", who_am_i);
        }
        defmt::info!("MPU6500 WHO_AM_I: 0x{:02x}", who_am_i);
        
        // Сброс устройства
        self.write_register(i2c, regs::PWR_MGMT_1, 0x80).await?;
        Timer::after(Duration::from_millis(100)).await;
        
        // Сброс сигнальных путей
        self.write_register(i2c, regs::SIGNAL_PATH_RESET, 0x07).await?;
        Timer::after(Duration::from_millis(100)).await;
        
        // Выбор источника тактирования (PLL с X гироскопом)
        self.write_register(i2c, regs::PWR_MGMT_1, 0x01).await?;
        Timer::after(Duration::from_millis(10)).await;
        
        // Включение всех осей акселерометра и гироскопа
        self.write_register(i2c, regs::PWR_MGMT_2, 0x00).await?;
        
        // Конфигурация DLPF (Digital Low Pass Filter)
        // DLPF_CFG = 3: 44Hz акселерометр, 42Hz гироскоп
        self.write_register(i2c, regs::CONFIG, DlpfBandwidth::Bw44Hz as u8).await?;
        
        // Частота выборки = 1kHz / (1 + SMPLRT_DIV)
        // Для 100Hz: 1000Hz / (1 + 9) = 100Hz
        self.write_register(i2c, regs::SMPLRT_DIV, 9).await?;
        
        // Конфигурация гироскопа (по умолчанию ±250°/s)
        self.set_gyro_range(i2c, GyroRange::Deg250).await?;
        
        // Конфигурация акселерометра (по умолчанию ±2g)
        self.set_accel_range(i2c, AccelRange::G2).await?;
        
        // Конфигурация фильтра акселерометра
        // A_DLPF_CFG = 3: 44.8Hz
        self.write_register(i2c, regs::ACCEL_CONFIG_2, 0x03).await?;
        
        // Отключение FIFO
        self.write_register(i2c, regs::FIFO_EN, 0x00).await?;
        
        // Конфигурация прерываний (включаем data ready)
        self.write_register(i2c, regs::INT_PIN_CFG, 0x10).await?; // LATCH_INT_EN
        self.write_register(i2c, regs::INT_ENABLE, 0x01).await?;  // DATA_RDY_EN
        
        // Небольшая задержка для стабилизации
        Timer::after(Duration::from_millis(50)).await;
        
        defmt::info!("MPU6500 инициализирован успешно");
        Ok(())
    }
    
    /// Установка диапазона измерения акселерометра
    pub async fn set_accel_range<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, range: AccelRange) -> Result<(), Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        self.write_register(i2c, regs::ACCEL_CONFIG, range as u8).await?;
        self.accel_range = range;
        
        // Обновляем масштабный коэффициент
        self.accel_scale = match range {
            AccelRange::G2 => 16384.0,
            AccelRange::G4 => 8192.0,
            AccelRange::G8 => 4096.0,
            AccelRange::G16 => 2048.0,
        };
        
        defmt::info!("Установлен диапазон акселерометра: {:?}", range as u8);
        Ok(())
    }
    
    /// Установка диапазона измерения гироскопа
    pub async fn set_gyro_range<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, range: GyroRange) -> Result<(), Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        self.write_register(i2c, regs::GYRO_CONFIG, range as u8).await?;
        self.gyro_range = range;
        
        // Обновляем масштабный коэффициент
        self.gyro_scale = match range {
            GyroRange::Deg250 => 131.0,
            GyroRange::Deg500 => 65.5,
            GyroRange::Deg1000 => 32.8,
            GyroRange::Deg2000 => 16.4,
        };
        
        defmt::info!("Установлен диапазон гироскопа: {:?}", range as u8);
        Ok(())
    }
    
    /// Чтение сырых данных
    pub async fn read_raw<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<RawData, Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Читаем все 14 байт данных одной операцией
        let mut buf = [0u8; 14];
        self.read_registers(i2c, regs::ACCEL_XOUT_H, &mut buf).await?;
        
        // Преобразуем байты в 16-битные значения (big-endian)
        Ok(RawData {
            accel_x: i16::from_be_bytes([buf[0], buf[1]]),
            accel_y: i16::from_be_bytes([buf[2], buf[3]]),
            accel_z: i16::from_be_bytes([buf[4], buf[5]]),
            temp: i16::from_be_bytes([buf[6], buf[7]]),
            gyro_x: i16::from_be_bytes([buf[8], buf[9]]),
            gyro_y: i16::from_be_bytes([buf[10], buf[11]]),
            gyro_z: i16::from_be_bytes([buf[12], buf[13]]),
        })
    }
    
    /// Чтение и обработка всех данных
    pub async fn read_all<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<Mpu6500Data, Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let raw = self.read_raw(i2c).await?;
        
        // Преобразование в физические единицы
        // Акселерометр: преобразуем в м/с²
        let accel_x = ((raw.accel_x as f32 / self.accel_scale) - self.accel_offset.0) * 9.81;
        let accel_y = ((raw.accel_y as f32 / self.accel_scale) - self.accel_offset.1) * 9.81;
        let accel_z = ((raw.accel_z as f32 / self.accel_scale) - self.accel_offset.2) * 9.81;
        
        // Гироскоп: преобразуем из °/с в рад/с
        let gyro_x = ((raw.gyro_x as f32 / self.gyro_scale) * PI / 180.0) - self.gyro_offset.0;
        let gyro_y = ((raw.gyro_y as f32 / self.gyro_scale) * PI / 180.0) - self.gyro_offset.1;
        let gyro_z = ((raw.gyro_z as f32 / self.gyro_scale) * PI / 180.0) - self.gyro_offset.2;
        
        // Температура: Temperature = (TEMP_OUT / 333.87) + 21.0
        let temperature = (raw.temp as f32 / 333.87) + 21.0;
        
        // Расчет углов из акселерометра
        let roll = libm::atan2f(accel_y, accel_z);
        let pitch = libm::atan2f(-accel_x, libm::sqrtf(accel_y * accel_y + accel_z * accel_z));
        
        Ok(Mpu6500Data {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
            temperature,
            roll,
            pitch,
        })
    }
    
    /// Калибровка гироскопа (определение смещения нуля)
    pub async fn calibrate_gyro<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, samples: u16) -> Result<(f32, f32, f32), Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        defmt::info!("Начало калибровки гироскопа, {} измерений", samples);
        defmt::info!("НЕ ДВИГАЙТЕ УСТРОЙСТВО!");
        
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_z = 0.0;
        
        // Собираем данные
        for i in 0..samples {
            let raw = self.read_raw(i2c).await?;
            
            sum_x += (raw.gyro_x as f32 / self.gyro_scale) * PI / 180.0;
            sum_y += (raw.gyro_y as f32 / self.gyro_scale) * PI / 180.0;
            sum_z += (raw.gyro_z as f32 / self.gyro_scale) * PI / 180.0;
            
            if i % 10 == 0 {
                defmt::info!("Калибровка: {}/{}", i, samples);
            }
            
            Timer::after(Duration::from_millis(10)).await;
        }
        
        // Вычисляем средние значения смещения
        self.gyro_offset = (
            sum_x / samples as f32,
            sum_y / samples as f32,
            sum_z / samples as f32,
        );
        
        defmt::info!("Калибровка завершена. Смещения гироскопа:");
        defmt::info!("  X: {} рад/с", self.gyro_offset.0);
        defmt::info!("  Y: {} рад/с", self.gyro_offset.1);
        defmt::info!("  Z: {} рад/с", self.gyro_offset.2);
        
        Ok(self.gyro_offset)
    }
    
    /// Калибровка акселерометра
    pub async fn calibrate_accel<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, samples: u16) -> Result<(f32, f32, f32), Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        defmt::info!("Начало калибровки акселерометра");
        defmt::info!("Устройство должно лежать горизонтально!");
        
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_z = 0.0;
        
        for _ in 0..samples {
            let raw = self.read_raw(i2c).await?;
            
            sum_x += raw.accel_x as f32 / self.accel_scale;
            sum_y += raw.accel_y as f32 / self.accel_scale;
            sum_z += raw.accel_z as f32 / self.accel_scale;
            
            Timer::after(Duration::from_millis(10)).await;
        }
        
        // Средние значения
        let avg_x = sum_x / samples as f32;
        let avg_y = sum_y / samples as f32;
        let avg_z = sum_z / samples as f32;
        
        // Смещения (Z должен показывать 1g)
        self.accel_offset = (avg_x, avg_y, avg_z - 1.0);
        
        defmt::info!("Калибровка акселерометра завершена");
        
        Ok(self.accel_offset)
    }
    
    /// Проверка готовности новых данных
    pub async fn data_ready<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<bool, Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let status = self.read_register(i2c, regs::INT_STATUS).await?;
        Ok((status & 0x01) != 0)
    }
    
    /// Самотестирование
    pub async fn self_test<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>) -> Result<bool, Mpu6500Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        defmt::info!("Запуск самотестирования MPU6500");
        
        // Сохраняем текущую конфигурацию
        let orig_gyro_config = self.read_register(i2c, regs::GYRO_CONFIG).await?;
        let orig_accel_config = self.read_register(i2c, regs::ACCEL_CONFIG).await?;
        
        // Включаем самотестирование
        self.write_register(i2c, regs::GYRO_CONFIG, 0xE0).await?;  // Self-test enabled, ±250°/s
        self.write_register(i2c, regs::ACCEL_CONFIG, 0xE0).await?; // Self-test enabled, ±2g
        
        Timer::after(Duration::from_millis(100)).await;
        
        // Читаем данные с включенным самотестированием
        let raw_st = self.read_raw(i2c).await?;
        
        // Восстанавливаем конфигурацию
        self.write_register(i2c, regs::GYRO_CONFIG, orig_gyro_config).await?;
        self.write_register(i2c, regs::ACCEL_CONFIG, orig_accel_config).await?;
        
        Timer::after(Duration::from_millis(100)).await;
        
        // Читаем данные без самотестирования
        let raw_normal = self.read_raw(i2c).await?;
        
        // Проверяем разницу (должна быть существенной)
        let accel_diff_x = (raw_st.accel_x - raw_normal.accel_x).abs();
        let accel_diff_y = (raw_st.accel_y - raw_normal.accel_y).abs();
        let accel_diff_z = (raw_st.accel_z - raw_normal.accel_z).abs();
        
        defmt::info!("Разница самотестирования:");
        defmt::info!("  Accel X: {}", accel_diff_x);
        defmt::info!("  Accel Y: {}", accel_diff_y);
        defmt::info!("  Accel Z: {}", accel_diff_z);
        
        // Проверяем, что разница достаточная (порог зависит от устройства)
        let test_passed = accel_diff_x > 500 && accel_diff_y > 500 && accel_diff_z > 500;
        
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

// Тесты для отладки
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_scale_factors() {
        assert_eq!(AccelRange::G2 as u8, 0x00);
        assert_eq!(GyroRange::Deg250 as u8, 0x00);
    }
}
