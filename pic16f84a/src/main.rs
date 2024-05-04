use std::env;
mod cpu_core;
mod readhexfile;

fn main() {
    let Some(filename) = env::args().nth(1) else {
        eprintln!("Argument: pic16f84a HEX file. Exit.");
        std::process::exit(-1);
    };
    let (code, configbits, eeprom) = readhexfile::readhexfile(&filename);
    println!("{code:04x?}\n {eeprom:02x?}");
    let mut cpu = cpu_core::Cpu::new(&code, configbits, &eeprom); // load code
    cpu.set_debug(true);
    // Test
    cpu.gpio_in(0x00, 0x00);
    let (_pa, _pb) = cpu.gpio_out();
    loop {
        cpu.nextclk();
        // cpu.gpio_in(pa_in, pb_in);
        // let (pa, pb) = cpu.gpio_out();
        // let (pa_dir, pb_dir) = cpu.gpio_getdirection();
    }
}
