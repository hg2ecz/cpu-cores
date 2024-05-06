use std::env;
mod cpu_core;
mod readhexfile;

fn main() {
    let Some(filename) = env::args().nth(1) else {
        eprintln!("Argument: pic16f84a HEX file. Exit.");
        std::process::exit(-1);
    };

    let (code, configbits, eeprom) = readhexfile::readhexfile(&filename); // Read iHEX file
    println!("{code:04x?}\n {eeprom:02x?}");
    let mut cpu = cpu_core::Cpu::new(&code, configbits, &eeprom); // load code into simulated microcontroller

    // Demo - save eeprom & load eeprom
    let eeprom = cpu.eeprom_save_laststate(); // [0u8; cpu_core::EEPROMSIZE]
    cpu.eeprom_load_laststate(eeprom); // load from saved data ... here is only a demo

    // main
    #[cfg(feature = "cpu_debug")]
    cpu.set_debug(true); // set or unset (false)
    loop {
        cpu.nextclk();

        // examples - GPIO IN / OUT
        let pa_in = 0x00;
        let pb_in = 0x00;
        cpu.gpio_in(pa_in, pb_in);
        let (_pa, _pb) = cpu.gpio_out();
        let (_pa_dir, _pb_dir) = cpu.gpio_getdirection(); // for debug & review

        // an example break
        if _pb == 0x55 {
            break;
        }
    }
    // at end, the contents of eeprom is readable & you can save to file.
    let _eeprom_out = cpu.eeprom_save_laststate(); // [0u8; cpu_core::EEPROMSIZE];
}
