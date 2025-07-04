# Автопилот автожира на Rust

[![Rust](https://img.shields.io/badge/rust-1.75%2B-orange.svg)](https://www.rust-lang.org)
[![Embassy](https://img.shields.io/badge/embassy-0.4.0-blue.svg)](https://embassy.dev)
[![Platform](https://img.shields.io/badge/platform-RP2040-green.svg)](https://www.raspberrypi.com/products/rp2040/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Автопилот для автожира, разработанный на языке Rust для микроконтроллера Raspberry Pi Pico (RP2040) с использованием
асинхронного embedded фреймворка Embassy.

## 🚁 Особенности

- **Полностью автономный полет** - автоматический взлет и посадка
- **Асинхронная архитектура** - эффективное использование ресурсов с Embassy
- **Каскадное управление** - двухуровневые PID контроллеры для точной стабилизации
- **Безопасность** - множественные проверки и ограничения для предотвращения аварий
- **Модульная структура** - легко расширяемый и поддерживаемый код
- **Специфика автожира** - учет особенностей авторотации и управления

## 📋 Требования

### Аппаратное обеспечение

- **Микроконтроллер**: Raspberry Pi Pico (RP2040)
- **IMU**: MPU6050 (акселерометр + гироскоп)
- **Барометр**: BMP280
- **GPS**: NEO-M8N или совместимый (NMEA протокол)
- **ESC**: 2x с поддержкой DShot600
- **Сервоприводы**: 2x для управления циклическим шагом ротора
- **Питание**: 3S-4S LiPo батарея с BEC 5V

### Программное обеспечение

- Rust 1.75 или новее
- probe-rs для прошивки и отладки
- cargo-embassy для работы с Embassy

## 🛠️ Установка

1. **Установите Rust и необходимые инструменты:**

```bash
# Установка Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Установка целевой платформы
rustup target add thumbv6m-none-eabi

# Установка инструментов для прошивки
cargo install probe-rs --features cli
cargo install cargo-embassy
```

- **Клонируйте репозиторий:**

```bash
git clone https://github.com/your-username/autogyro-autopilot.git
cd autogyro-autopilot
```

- **Настройте конфигурацию:**

```bash
# Отредактируйте параметры под ваше железо
vim src/config/hardware.rs
vim src/config/flight.rs
```

**🚀 Использование**

**Сборка проекта**

```bash
cargo build --release
```

**Прошивка микроконтроллера**

```bash
# С использованием probe-rs
cargo embed --release

# Или через cargo-embassy
cargo embassy flash --release
```

**Отладка**

```bash
# Запуск с выводом отладочной информации
cargo embed --release --log

# Мониторинг RTT логов
probe-rs rtt --chip RP2040
```

**📁 Структура проекта**

```
autogyro-autopilot/
├── src/
│   ├── main.rs              # Точка входа
│   ├── config/              # Конфигурация
│   │   ├── hardware.rs      # Пины, частоты, адреса
│   │   └── flight.rs        # PID коэффициенты, параметры полета
│   ├── drivers/             # Драйверы устройств
│   │   ├── imu/            # MPU6050
│   │   ├── baro/           # BMP280
│   │   ├── gps/            # NMEA парсер
│   │   └── actuators/      # DShot, PWM
│   ├── control/             # Алгоритмы управления
│   │   ├── attitude.rs      # Стабилизация положения
│   │   ├── altitude.rs      # Контроль высоты
│   │   └── fuzzy.rs         # Fuzzy контроллеры
│   ├── flight/              # Режимы полета
│   │   ├── takeoff.rs       # Автоматический взлет
│   │   └── landing.rs       # Автоматическая посадка
│   └── tasks/               # Асинхронные задачи
│       ├── sensor_task.rs   # Опрос датчиков
│       ├── control_task.rs  # Управление
│       └── actuator_task.rs # Моторы и сервоприводы
├── Cargo.toml
├── memory.x
└── README.md
```

**⚙️ Конфигурация**

**Настройка пинов (src/config/hardware.rs)**

```rust
pub mod pins {
    pub mod i2c {
        pub const SDA_PIN: u8 = 4;  // GPIO4
        pub const SCL_PIN: u8 = 5;  // GPIO5
    }
    pub mod servos {
        pub const PITCH_PIN: u8 = 10; // GPIO10
        pub const ROLL_PIN: u8 = 11;  // GPIO11
    }
    // ...
}
```

**🔧 Калибровка**

- **Калибровка IMU:**
    - Поместите автожир на ровную поверхность
    - Запустите режим калибровки через телеметрию
    - Дождитесь завершения (около 10 секунд)

- **Проверка безопасности:**
    - Проверьте все ограничения углов
    - Убедитесь в правильности направления коррекции
    - Протестируйте аварийные режимы

**📊 Телеметрия**

Автопилот отправляет следующие данные через UART (115200 бод):

- Углы ориентации (крен, тангаж, рыскание)
- Высота и вертикальная скорость
- GPS координаты и путевая скорость
- Состояние системы и режим полета
- Уровень батареи (если подключен датчик)

**🛡️ Безопасность**

- **Ограничения углов** - максимум 30° крен, 25° тангаж
- **Проверка датчиков** - автоматический переход в аварийный режим при отказе
- **Минимальная высота** - автоматические маневры только выше 5м
- **Failsafe** - автоматическая посадка при потере связи
- **Геозона** - ограничение максимальной дистанции от точки взлета

**🤝 Вклад в проект**

Приветствуются любые улучшения! Пожалуйста:

- Форкните репозиторий
- Создайте ветку для ваших изменений
- Напишите тесты если возможно
- Отправьте pull request

**📝 Лицензия**

Этот проект распространяется под лицензией MIT. См. файл LICENSE для деталей.

**⚠️ Отказ от ответственности**

Этот автопилот является экспериментальным проектом. Использование в реальных полетах осуществляется на ваш страх и риск.
Всегда соблюдайте местные законы и правила использования БПЛА.

**🙏 Благодарности**

- https://embassy.dev - за отличный async embedded фреймворк
- https://probe.rs - за инструменты отладки
- Сообщество Rust Embedded - за помощь и поддержку

**📞 Контакты**

По вопросам и предложениям обращайтесь через Issues на GitHub.

**Безопасных полетов!** 🚁✨



