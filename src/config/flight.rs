//! Конфигурация параметров полета и PID контроллеров

use core::f32::consts::PI;

/// Параметры PID контроллеров
pub mod pid {
    /// PID коэффициенты для контроля угла крена (roll)
    pub mod roll_angle {
        pub const KP: f32 = 4.5; // Пропорциональный коэффициент
        pub const KI: f32 = 0.02; // Интегральный коэффициент
        pub const KD: f32 = 0.15; // Дифференциальный коэффициент
        pub const I_LIMIT: f32 = 10.0; // Ограничение интегральной составляющей
        pub const OUTPUT_LIMIT: f32 = 1.0; // Максимальный выход (-1.0 до 1.0)
    }

    /// PID коэффициенты для контроля угловой скорости крена
    pub mod roll_rate {
        pub const KP: f32 = 0.15;
        pub const KI: f32 = 0.05;
        pub const KD: f32 = 0.004;
        pub const I_LIMIT: f32 = 20.0;
        pub const OUTPUT_LIMIT: f32 = 1.0;
    }

    /// PID коэффициенты для контроля угла тангажа (pitch)
    pub mod pitch_angle {
        pub const KP: f32 = 4.5;
        pub const KI: f32 = 0.02;
        pub const KD: f32 = 0.15;
        pub const I_LIMIT: f32 = 10.0;
        pub const OUTPUT_LIMIT: f32 = 1.0;
    }

    /// PID коэффициенты для контроля угловой скорости тангажа
    pub mod pitch_rate {
        pub const KP: f32 = 0.15;
        pub const KI: f32 = 0.05;
        pub const KD: f32 = 0.004;
        pub const I_LIMIT: f32 = 20.0;
        pub const OUTPUT_LIMIT: f32 = 1.0;
    }

    /// PID коэффициенты для контроля курса (yaw)
    pub mod yaw_rate {
        pub const KP: f32 = 2.5;
        pub const KI: f32 = 0.01;
        pub const KD: f32 = 0.0;
        pub const I_LIMIT: f32 = 10.0;
        pub const OUTPUT_LIMIT: f32 = 0.3; // Ограничено для автожира
    }

    /// PID коэффициенты для контроля высоты
    pub mod altitude {
        pub const KP: f32 = 1.5;
        pub const KI: f32 = 0.05;
        pub const KD: f32 = 0.8;
        pub const I_LIMIT: f32 = 5.0;
        pub const OUTPUT_LIMIT: f32 = 0.5; // Ограничение изменения газа
    }

    /// PID коэффициенты для контроля вертикальной скорости
    pub mod vertical_speed {
        pub const KP: f32 = 3.0;
        pub const KI: f32 = 0.1;
        pub const KD: f32 = 0.05;
        pub const I_LIMIT: f32 = 10.0;
        pub const OUTPUT_LIMIT: f32 = 0.8;
    }

    /// PID коэффициенты для контроля позиции (GPS)
    pub mod position {
        pub const KP: f32 = 0.8;
        pub const KI: f32 = 0.01;
        pub const KD: f32 = 0.2;
        pub const I_LIMIT: f32 = 5.0;
        pub const OUTPUT_LIMIT: f32 = 15.0; // Максимальный угол наклона в градусах
    }
}

/// Параметры режима взлета
pub mod takeoff {
    /// Целевая высота взлета (метры)
    pub const TARGET_ALTITUDE_M: f32 = 10.0;

    /// Скорость набора высоты (м/с)
    pub const CLIMB_RATE_MS: f32 = 1.5;

    /// Минимальная тяга для взлета (0.0 - 1.0)
    pub const MIN_THROTTLE: u16 = 600;

    /// Начальная тяга при взлете (0.0 - 1.0)
    pub const INITIAL_THROTTLE: u16 = 1200;

    /// Максимальная тяга при взлете (0.0 - 1.0)
    pub const MAX_THROTTLE: u16 = 2000;

    /// Время раскрутки моторов перед взлетом (мс)
    pub const MOTOR_SPINUP_TIME_MS: u64 = 2000;

    /// Минимальные обороты ротора для взлета (об/мин)
    pub const MIN_ROTOR_RPM: f32 = 200.0;

    /// Максимальный угол тангажа при взлете (градусы)
    pub const MAX_PITCH_ANGLE_DEG: f32 = 10.0;

    /// Допустимая ошибка по высоте для завершения взлета (метры)
    pub const ALTITUDE_TOLERANCE_M: f32 = 0.5;
}

/// Параметры режима посадки
pub mod landing {
    /// Скорость снижения при посадке (м/с)
    pub const DESCENT_RATE_MS: f32 = 0.8;

    /// Скорость финального снижения (м/с)
    pub const FINAL_DESCENT_RATE_MS: f32 = 0.3;

    /// Высота начала финального снижения (метры)
    pub const FINAL_APPROACH_ALTITUDE_M: f32 = 2.0;

    /// Минимальная тяга при посадке (0.0 - 1.0)
    pub const MIN_THROTTLE: f32 = 0.15;

    /// Тяга при финальном снижении (0.0 - 1.0)
    pub const FINAL_THROTTLE: f32 = 0.1;

    /// Высота отключения моторов (метры)
    pub const MOTOR_CUTOFF_ALTITUDE_M: f32 = 0.3;

    /// Максимальный угол наклона при посадке (градусы)
    pub const MAX_TILT_ANGLE_DEG: f32 = 5.0;

    /// Время ожидания после касания земли (мс)
    pub const GROUND_SETTLE_TIME_MS: u64 = 1000;
}

/// Параметры стабилизации
pub mod stabilization {
    /// Максимальный угол крена в режиме стабилизации (градусы)
    pub const MAX_ROLL_ANGLE_DEG: f32 = 25.0;

    /// Максимальный угол тангажа в режиме стабилизации (градусы)
    pub const MAX_PITCH_ANGLE_DEG: f32 = 20.0;

    /// Максимальная угловая скорость (рад/с)
    pub const MAX_ANGULAR_RATE_RAD_S: f32 = 3.14; // ~180°/s

    /// Зона нечувствительности для углов (градусы)
    pub const ANGLE_DEADBAND_DEG: f32 = 1.0;

    /// Коэффициент экспоненты для сглаживания управления
    pub const CONTROL_EXPO: f32 = 0.3;
}

/// Параметры навигации
pub mod navigation {
    /// Радиус достижения путевой точки (метры)
    pub const WAYPOINT_RADIUS_M: f32 = 3.0;

    /// Максимальная скорость полета (м/с)
    pub const MAX_SPEED_MS: f32 = 15.0;

    /// Крейсерская скорость (м/с)
    pub const CRUISE_SPEED_MS: f32 = 10.0;

    /// Минимальная скорость для автожира (м/с)
    pub const MIN_SPEED_MS: f32 = 5.0;

    /// Максимальный угол крена при повороте (градусы)
    pub const MAX_BANK_ANGLE_DEG: f32 = 30.0;

    /// Упреждение поворота (метры)
    pub const TURN_LEAD_DISTANCE_M: f32 = 5.0;
}

/// Параметры автожира
pub mod autogyro {
    /// Минимальная скорость для авторотации (м/с)
    pub const MIN_AUTOROTATION_SPEED_MS: f32 = 8.0;

    /// Оптимальные обороты ротора (об/мин)
    pub const OPTIMAL_ROTOR_RPM: f32 = 350.0;

    /// Минимальные обороты ротора для полета (об/мин)
    pub const MIN_ROTOR_RPM: f32 = 250.0;

    /// Максимальные обороты ротора (об/мин)
    pub const MAX_ROTOR_RPM: f32 = 450.0;

    /// Коэффициент связи тяги и угла тангажа
    pub const THRUST_TO_PITCH_RATIO: f32 = 0.15;

    /// Задержка реакции ротора (мс)
    pub const ROTOR_RESPONSE_DELAY_MS: u64 = 500;

    /// Дифференциал тяги для управления курсом
    pub const YAW_THRUST_DIFFERENTIAL: f32 = 0.1;
}

/// Параметры безопасности полета
pub mod safety {
    /// Минимальная высота для автоматических маневров (метры)
    pub const MIN_SAFE_ALTITUDE_M: f32 = 3.0;

    /// Высота аварийной посадки (метры)
    pub const EMERGENCY_LANDING_ALTITUDE_M: f32 = 30.0;

    /// Максимальный угол атаки (градусы)
    pub const MAX_ANGLE_OF_ATTACK_DEG: f32 = 15.0;

    /// Время до автоматической посадки при потере связи (секунды)
    pub const FAILSAFE_TIMEOUT_S: u32 = 30;

    /// Максимальная дистанция от точки взлета (метры)
    pub const MAX_DISTANCE_FROM_HOME_M: f32 = 500.0;

    /// Минимальное количество спутников GPS
    pub const MIN_GPS_SATELLITES: u8 = 6;

    /// Максимальный HDOP для навигации
    pub const MAX_GPS_HDOP: f32 = 2.5;
}

/// Преобразование единиц измерения
pub mod conversions {
    use super::*;

    /// Преобразование градусов в радианы
    pub const fn deg_to_rad(deg: f32) -> f32 {
        deg * (PI / 180.0)
    }

    /// Преобразование радианов в градусы
    pub const fn rad_to_deg(rad: f32) -> f32 {
        rad * (180.0 / PI)
    }

    /// Преобразование м/с в км/ч
    pub const fn ms_to_kmh(ms: f32) -> f32 {
        ms * 3.6
    }

    /// Преобразование км/ч в м/с
    pub const fn kmh_to_ms(kmh: f32) -> f32 {
        kmh / 3.6
    }
}

/// Настройки логирования и телеметрии
pub mod telemetry {
    /// Приоритеты сообщений
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum MessagePriority {
        Critical = 0,
        High = 1,
        Normal = 2,
        Low = 3,
    }

    /// Маска логирования (битовые флаги)
    pub mod log_mask {
        pub const ATTITUDE: u32 = 1 << 0;
        pub const ALTITUDE: u32 = 1 << 1;
        pub const GPS: u32 = 1 << 2;
        pub const MOTORS: u32 = 1 << 3;
        pub const PID: u32 = 1 << 4;
        pub const SYSTEM: u32 = 1 << 5;
        pub const ALL: u32 = 0xFFFFFFFF;
    }
}
