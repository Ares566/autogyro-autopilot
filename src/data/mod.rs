// src/data/mod.rs
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;

/// Размеры буферов каналов
const SENSOR_CHANNEL_SIZE: usize = 10;
const CONTROL_CHANNEL_SIZE: usize = 5;

/// Данные с IMU
#[derive(Clone, Copy, Debug)]
pub struct ImuData {
    pub roll: f32,       // Крен в радианах
    pub pitch: f32,      // Тангаж в радианах
    pub yaw: f32,        // Рыскание в радианах
    pub roll_rate: f32,  // Угловая скорость по крену (рад/с)
    pub pitch_rate: f32, // Угловая скорость по тангажу (рад/с)
    pub yaw_rate: f32,   // Угловая скорость по рысканию (рад/с)
    pub timestamp_us: u64,
}

/// Данные высоты
#[derive(Clone, Copy, Debug)]
pub struct AltitudeData {
    pub altitude_m: f32,         // Высота в метрах
    pub vertical_speed_mps: f32, // Вертикальная скорость м/с
    pub pressure_pa: f32,        // Давление в Паскалях
    pub temperature_c: f32,      // Температура в Цельсиях
    pub timestamp_us: u64,
}

/// Данные GPS
#[derive(Clone, Copy, Debug)]
pub struct GpsData {
    pub latitude: f64,         // Широта
    pub longitude: f64,        // Долгота
    pub altitude_msl_m: f32,   // Высота над уровнем моря
    pub ground_speed_mps: f32, // Путевая скорость м/с
    pub course_deg: f32,       // Курс в градусах
    pub satellites: u8,        // Количество спутников
    pub hdop: f32,             // Горизонтальная точность
    pub timestamp_us: u64,
}

/// Команды управления
#[derive(Clone, Copy, Debug)]
pub struct ControlCommand {
    pub throttle_left: u16,  // Газ левого мотора (48 - 2047)
    pub throttle_right: u16, // Газ правого мотора (48 - 2047)
    pub cyclic_pitch: f32,   // Циклический шаг по тангажу (-1.0 - 1.0)
    pub cyclic_roll: f32,    // Циклический шаг по крену (-1.0 - 1.0)
}

/// Общее состояние системы (для быстрого доступа без блокировок)
pub struct SystemState {
    pub armed: AtomicBool,
    pub flight_mode: Mutex<CriticalSectionRawMutex, FlightMode>,
    pub last_imu: Mutex<CriticalSectionRawMutex, Option<ImuData>>,
    pub last_altitude: Mutex<CriticalSectionRawMutex, Option<AltitudeData>>,
    pub last_gps: Mutex<CriticalSectionRawMutex, Option<GpsData>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FlightMode {
    Disarmed,
    Stabilize,
    TakeOff,
    Landing,
    Emergency,
}

/// Каналы для передачи данных между задачами
pub struct DataChannels {
    // Каналы от датчиков к обработке
    pub imu_channel: Channel<CriticalSectionRawMutex, ImuData, SENSOR_CHANNEL_SIZE>,
    pub altitude_channel: Channel<CriticalSectionRawMutex, AltitudeData, SENSOR_CHANNEL_SIZE>,
    pub gps_channel: Channel<CriticalSectionRawMutex, GpsData, SENSOR_CHANNEL_SIZE>,

    // Канал команд управления
    pub control_channel: Channel<CriticalSectionRawMutex, ControlCommand, CONTROL_CHANNEL_SIZE>,
}

impl DataChannels {
    pub const fn new() -> Self {
        Self {
            imu_channel: Channel::new(),
            altitude_channel: Channel::new(),
            gps_channel: Channel::new(),
            control_channel: Channel::new(),
        }
    }
}

impl SystemState {
    pub const fn new() -> Self {
        Self {
            armed: AtomicBool::new(false),
            flight_mode: Mutex::new(FlightMode::Disarmed),
            last_imu: Mutex::new(None),
            last_altitude: Mutex::new(None),
            last_gps: Mutex::new(None),
        }
    }

    /// Проверка готовности системы к полету
    pub async fn is_ready_for_flight(&self) -> bool {
        let imu_ok = self.last_imu.lock().await.is_some();
        let alt_ok = self.last_altitude.lock().await.is_some();
        let gps_ok = true; //self.last_gps.lock().await.is_some();

        imu_ok && alt_ok && gps_ok
    }
}

// Статические экземпляры для глобального доступа
pub static CHANNELS: DataChannels = DataChannels::new();
pub static SYSTEM_STATE: SystemState = SystemState::new();
