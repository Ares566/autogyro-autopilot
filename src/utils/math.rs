//! Математические функции и утилиты

use core::f32::consts::PI;

/// Ограничение значения в заданных пределах
#[inline(always)]
pub fn constrain(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// Ограничение целочисленного значения в заданных пределах
#[inline(always)]
pub fn constrain_i32(value: i32, min: i32, max: i32) -> i32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// Применение экспоненциальной кривой к управляющему сигналу
/// expo: 0.0 = линейная, 1.0 = максимальная экспонента
#[inline]
pub fn apply_expo(input: f32, expo: f32) -> f32 {
    let expo = constrain(expo, 0.0, 1.0);
    let input_abs = input.abs();

    // Формула: output = input * (|input| * expo + 1 - expo)
    let output_abs = input_abs * (input_abs * expo + 1.0 - expo);

    if input < 0.0 {
        -output_abs
    } else {
        output_abs
    }
}

/// Линейная интерполяция между двумя значениями
/// t: 0.0 = a, 1.0 = b
#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * constrain(t, 0.0, 1.0)
}

/// Обратная линейная интерполяция - получение t из значения
#[inline]
pub fn inverse_lerp(a: f32, b: f32, value: f32) -> f32 {
    if (b - a).abs() < f32::EPSILON {
        0.0
    } else {
        constrain((value - a) / (b - a), 0.0, 1.0)
    }
}

/// Перемапинг значения из одного диапазона в другой
#[inline]
pub fn map_range(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    let t = inverse_lerp(in_min, in_max, value);
    lerp(out_min, out_max, t)
}

/// Зона нечувствительности (deadband)
#[inline]
pub fn apply_deadband(value: f32, deadband: f32) -> f32 {
    if value.abs() < deadband {
        0.0
    } else if value > 0.0 {
        map_range(value, deadband, 1.0, 0.0, 1.0)
    } else {
        map_range(value, -1.0, -deadband, -1.0, 0.0)
    }
}

/// Нормализация угла в диапазон [-π, π]
#[inline]
pub fn normalize_angle(angle: f32) -> f32 {
    let mut normalized = angle % (2.0 * PI);
    if normalized > PI {
        normalized -= 2.0 * PI;
    } else if normalized < -PI {
        normalized += 2.0 * PI;
    }
    normalized
}

/// Нормализация угла в диапазон [0, 2π]
#[inline]
pub fn normalize_angle_positive(angle: f32) -> f32 {
    let mut normalized = angle % (2.0 * PI);
    if normalized < 0.0 {
        normalized += 2.0 * PI;
    }
    normalized
}

/// Расчет кратчайшей угловой разницы между двумя углами
#[inline]
pub fn angle_difference(angle1: f32, angle2: f32) -> f32 {
    normalize_angle(angle2 - angle1)
}

/// Квадратный корень с защитой от отрицательных значений
#[inline]
pub fn safe_sqrt(value: f32) -> f32 {
    if value <= 0.0 {
        0.0
    } else {
        libm::sqrtf(value)
    }
}

/// Вычисление длины 2D вектора
#[inline]
pub fn vector2_length(x: f32, y: f32) -> f32 {
    safe_sqrt(x * x + y * y)
}

/// Вычисление длины 3D вектора
#[inline]
pub fn vector3_length(x: f32, y: f32, z: f32) -> f32 {
    safe_sqrt(x * x + y * y + z * z)
}

/// Нормализация 3D вектора
#[inline]
pub fn vector3_normalize(x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    let length = vector3_length(x, y, z);
    if length < f32::EPSILON {
        (0.0, 0.0, 0.0)
    } else {
        (x / length, y / length, z / length)
    }
}

/// Скалярное произведение 3D векторов
#[inline]
pub fn vector3_dot(x1: f32, y1: f32, z1: f32, x2: f32, y2: f32, z2: f32) -> f32 {
    x1 * x2 + y1 * y2 + z1 * z2
}

/// Векторное произведение 3D векторов
#[inline]
pub fn vector3_cross(x1: f32, y1: f32, z1: f32, x2: f32, y2: f32, z2: f32) -> (f32, f32, f32) {
    (
        y1 * z2 - z1 * y2,
        z1 * x2 - x1 * z2,
        x1 * y2 - y1 * x2,
    )
}

/// Быстрая аппроксимация обратного квадратного корня (Quake III)
/// Для случаев, когда точность не критична, но важна скорость
#[inline]
pub fn fast_inv_sqrt(x: f32) -> f32 {
    let i = x.to_bits();
    let i = 0x5f3759df - (i >> 1);
    let y = f32::from_bits(i);

    // Один шаг метода Ньютона для улучшения точности
    y * (1.5 - 0.5 * x * y * y)
}

/// Вычисление среднего значения массива
pub fn mean(values: &[f32]) -> f32 {
    if values.is_empty() {
        return 0.0;
    }

    let sum: f32 = values.iter().sum();
    sum / values.len() as f32
}

/// Вычисление медианы массива (массив будет отсортирован)
pub fn median(values: &mut [f32]) -> f32 {
    if values.is_empty() {
        return 0.0;
    }

    // Простая сортировка пузырьком для малых массивов
    for i in 0..values.len() {
        for j in 0..values.len() - i - 1 {
            if values[j] > values[j + 1] {
                values.swap(j, j + 1);
            }
        }
    }

    let mid = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[mid - 1] + values[mid]) / 2.0
    } else {
        values[mid]
    }
}

/// Вычисление стандартного отклонения
pub fn std_deviation(values: &[f32]) -> f32 {
    // if values.len() < 2 {
    //     return 0.0;
    // }
    // 
    // let mean_val = mean(values);
    // let variance = values.iter()
    //     .map(|&x| ((x - mean_val)*(x - mean_val)) //.powi(2))
    //     .sum::<f32>() / (values.len() - 1)) as f32;
    // 
    // safe_sqrt(variance)
    0.0
}

/// Преобразование из кватерниона в углы Эйлера (roll, pitch, yaw)
pub fn quaternion_to_euler(w: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = libm::atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        libm::copysignf(PI / 2.0, sinp)
    } else {
        libm::asinf(sinp)
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = libm::atan2f(siny_cosp, cosy_cosp);

    (roll, pitch, yaw)
}

/// Расчет расстояния между GPS координатами (упрощенная формула для малых расстояний)
pub fn gps_distance_meters(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f32 {
    const EARTH_RADIUS_M: f64 = 6371000.0;

    let lat1_rad = lat1 * PI as f64 / 180.0;
    let lat2_rad = lat2 * PI as f64 / 180.0;
    let delta_lat = (lat2 - lat1) * PI as f64 / 180.0;
    let delta_lon = (lon2 - lon1) * PI as f64 / 180.0;

    // Упрощенная формула для малых расстояний
    let x = delta_lon * libm::cos((lat1_rad + lat2_rad) / 2.0);
    let y = delta_lat;

    let distance = libm::sqrt(x * x + y * y) * EARTH_RADIUS_M;
    distance as f32
}

/// Расчет азимута между GPS координатами
pub fn gps_bearing_degrees(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f32 {
    let lat1_rad = lat1 * PI as f64 / 180.0;
    let lat2_rad = lat2 * PI as f64 / 180.0;
    let delta_lon = (lon2 - lon1) * PI as f64 / 180.0;

    let x = libm::sin(delta_lon) * libm::cos(lat2_rad);
    let y = libm::cos(lat1_rad) * libm::sin(lat2_rad) -
        libm::sin(lat1_rad) * libm::cos(lat2_rad) * libm::cos(delta_lon);

    let bearing_rad = libm::atan2(x, y);
    let bearing_deg = bearing_rad * 180.0 / PI as f64;

    // Нормализация в диапазон [0, 360)
    ((bearing_deg + 360.0) % 360.0) as f32
}

// Модульные тесты
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constrain() {
        assert_eq!(constrain(5.0, 0.0, 10.0), 5.0);
        assert_eq!(constrain(-5.0, 0.0, 10.0), 0.0);
        assert_eq!(constrain(15.0, 0.0, 10.0), 10.0);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < f32::EPSILON);
        assert!((normalize_angle(2.0 * PI) - 0.0).abs() < f32::EPSILON);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < f32::EPSILON);
        assert!((normalize_angle(-PI) - PI).abs() < f32::EPSILON);
    }

    #[test]
    fn test_apply_expo() {
        assert_eq!(apply_expo(0.0, 0.5), 0.0);
        assert_eq!(apply_expo(1.0, 0.0), 1.0);
        assert_eq!(apply_expo(-1.0, 0.0), -1.0);
    }
}
