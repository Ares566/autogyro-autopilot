//! Информация о системе и тактировании

use embassy_rp::clocks;
use embassy_rp::peripherals;

/// Структура с информацией о частотах системы
#[derive(Debug, Clone, Copy)]
pub struct SystemClocks {
    pub sys_freq: u32,
    pub peri_freq: u32,
    pub usb_freq: u32,
    pub adc_freq: u32,
    pub rtc_freq: u16,
    pub ref_freq: u32,
}

/// Получить текущие частоты системы
pub fn get_system_clocks() -> SystemClocks {
    SystemClocks {
        sys_freq: clocks::clk_sys_freq(),
        peri_freq: clocks::clk_peri_freq(),
        usb_freq: clocks::clk_usb_freq(),
        adc_freq: clocks::clk_adc_freq(),
        rtc_freq: clocks::clk_rtc_freq(),
        ref_freq: clocks::clk_ref_freq(),
    }
}

/// Вывести информацию о частотах в лог
pub fn print_clock_info() {
    let clocks = get_system_clocks();

    defmt::info!("=== Конфигурация тактирования ===");
    defmt::info!("Системная частота: {} МГц", clocks.sys_freq / 1_000_000);
    defmt::info!("Периферийная частота: {} МГц", clocks.peri_freq / 1_000_000);
    defmt::info!("USB частота: {} МГц", clocks.usb_freq / 1_000_000);
    defmt::info!("ADC частота: {} МГц", clocks.adc_freq / 1_000_000);
    defmt::info!("RTC частота: {} кГц", clocks.rtc_freq / 1_000);
    defmt::info!("Опорная частота: {} МГц", clocks.ref_freq / 1_000_000);
}

/// Проверить корректность частот для нашего применения
pub fn validate_clocks() -> Result<(), &'static str> {
    let clocks = get_system_clocks();

    // Проверяем системную частоту (должна быть достаточной для DShot)
    if clocks.sys_freq < 100_000_000 {
        return Err("Системная частота слишком низкая для DShot600");
    }

    // Проверяем периферийную частоту для I2C и UART
    if clocks.peri_freq < 48_000_000 {
        return Err("Периферийная частота слишком низкая");
    }

    Ok(())
}
