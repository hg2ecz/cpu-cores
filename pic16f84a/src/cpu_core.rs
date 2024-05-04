/* CPU core (pic16f84a):
    - ROM
    - instruction decoder
    - Wreg
    - RAM + special addresses
    - stack

   Special:
    - I/O
    - wdt + prescaler
    - timer
    - eeprom
*/

pub const ROMSIZE: usize = 0x400;
pub const EEPROMSIZE: usize = 64;

const REG_INDF: usize = 0x00;
const REG_TMR0: usize = 0x01; // OPTION
const REG_PCL: usize = 0x02;
const REG_STATUS: usize = 0x03;
const REG_FSR: usize = 0x04;
const REG_PORTA: usize = 0x05; // TRISA
const REG_PORTB: usize = 0x06; // TRISB
                               // 0x07;
const REG_EEDATA: usize = 0x08; // EECON1
const REG_EEADDR: usize = 0x09; // EECON2
const REG_PCLATH: usize = 0x0a;
const REG_INTCON: usize = 0x0b;

pub struct Cpu {
    // PC: from memreg
    configbits: u16,
    skipnext: bool,
    wreg: u8,          // 8 bit Wreg
    ram: [u8; 0x50],   // 8 bit
    rom: [u16; 0x400], // 14 bit
    stack: [usize; 8], // 13 bit
    stackptr: usize,   // 8 level stack

    psc_ct: u8,       // timer prescaler counter
    tmrdiv2: bool,    // timer: div2
    sleep: bool,      // sleep instruction, cpu halt
    callflag: bool,   // 2 cycle
    returnflag: bool, // 2 cycle

    // special register from rambank1
    option_reg: u8,
    trisa: u8,
    trisb: u8,
    eecon1: u8,
    eecon2: u8,         // not a physical register
    eeprom: [u8; 0x40], // pic16f84a has 64 bytes EEPROM

    debugmode: bool,
}

impl Cpu {
    pub fn new(prog: &[u16], configbits: u16, eeprom: &[u8]) -> Self {
        let mut cpu = Cpu {
            configbits,
            skipnext: false,
            wreg: 0,
            ram: [0; 0x50],
            rom: [0x00; ROMSIZE],
            stack: [0; 8],
            stackptr: 0,
            psc_ct: 0,
            tmrdiv2: false,
            sleep: false,
            callflag: false,
            returnflag: false,

            trisa: 0,
            trisb: 0,
            option_reg: 0,
            eecon1: 0,
            eecon2: 0, // not a physical register
            eeprom: [0x00; EEPROMSIZE],
            debugmode: false,
        };
        cpu.rom[..prog.len()].copy_from_slice(prog);
        cpu.eeprom[..eeprom.len()].copy_from_slice(eeprom);
        cpu.reset();
        cpu
    }

    // Reset CPU
    pub fn reset(&mut self) {
        self.ram[REG_PCL] = 0x00;
        self.ram[REG_STATUS] = 0b0001_1000 | (self.ram[REG_STATUS] & 0x07);
        self.ram[REG_PCLATH] = 0x00;
        self.ram[REG_INTCON] &= 1;
        self.option_reg = 0xff;
        self.trisa = 0x1f;
        self.trisb = 0xff;
        self.eecon1 &= 0x08;
    }

    // Ov every CLK - it is the main function
    pub fn nextclk(&mut self) {
        self.cpu_core();
        self.timer_wdt(true); // true: from nextclk
        self.eeprom();
    }

    // Set or unset debug print
    pub fn set_debug(&mut self, debugmode: bool) {
        self.debugmode = debugmode;
    }

    // Physical GPIO input
    pub fn gpio_in(&mut self, porta: u8, portb: u8) {
        let paold = self.ram[REG_PORTA];
        let pa = (paold & !self.trisa) | (porta & self.trisa);
        self.ram[REG_PORTA] = pa;

        let pbold = self.ram[REG_PORTB];
        let pb = (pbold & !self.trisb) | (portb & self.trisb);
        self.ram[REG_PORTB] = pb;

        // set INTF, RBIF and activate IRQ
        self.portb_change_irq(pbold, pb);
        // TIMER from RA4 Low-High or High-Low edge
        if (self.option_reg & 0x20 == 0 && paold & 0x10 != pa & 0x10)
            && ((self.option_reg & 0x10 == 0 && pa & 0x10 == 0x10)
                || (self.option_reg & 0x10 == 0x10 && pa & 0x10 == 0x00))
        {
            self.timer_wdt(false)
        } // false: from gpio
    }

    // Physical GPIO output
    pub fn gpio_out(&self) -> (u8, u8) {
        (self.ram[REG_PORTA], self.ram[REG_PORTB])
    }

    // This shows, which port is out and which is input. Mostly for debug only.
    pub fn gpio_getdirection(&self) -> (u8, u8) {
        (self.trisa, self.trisb)
    }

    // ------------------
    // Internal functions
    // ------------------

    fn get_rambank(&self) -> u8 {
        (self.ram[REG_STATUS] & 0x20 != 0) as u8
    }

    fn ramrd(&self, addr_in: u8) -> u8 {
        let addr = addr_in as usize;
        // special rambank1 cases
        if self.get_rambank() == 1 {
            match addr {
                REG_TMR0 => return self.option_reg,
                REG_PORTA => return self.trisa,
                REG_PORTB => return self.trisb,
                REG_EEDATA => return self.eecon1,
                REG_EEADDR => return 0, // self.eecon2 - 0x55 0xAA before EEPROM write
                _ => (),
            }
        }
        // Remark: all of other registers emulated from RAM
        match addr {
            REG_INDF => self.ram[self.ram[REG_FSR] as usize], // INDF, not real mem
            _ => self.ram[addr],
        }
    }

    fn ramwr(&mut self, addr_in: u8, data: u8, dst_wreg: bool) {
        // DST: wreg
        if dst_wreg {
            self.wreg = data;
            return;
        }
        let addr = addr_in as usize;
        // DST: RAM or special registers - first: special rambank1 cases
        if self.get_rambank() == 1 {
            match addr {
                REG_TMR0 => {
                    self.option_reg = data;
                    return;
                }
                REG_PORTA => {
                    self.trisa = data & 0x1f;
                    return;
                }
                REG_PORTB => {
                    self.trisb = data;
                    return;
                }
                REG_EEDATA => {
                    self.eecon1 = data;
                    return;
                }
                REG_EEADDR => {
                    self.eecon2 = data; // self.eecon2 - 0x55 0xAA before EEPROM write
                    return;
                }
                _ => (),
            }
        }
        match addr {
            REG_INDF => self.ram[self.ram[REG_INDF] as usize] = data, // INDF, not real mem
            REG_PORTA => {
                self.ram[REG_PORTA] =
                    (self.ram[REG_PORTA] & self.trisa) | (data & 0x1F & !self.trisa)
            }
            REG_PORTB => {
                self.ram[REG_PORTB] = (self.ram[REG_PORTB] & self.trisb) | (data & !self.trisb)
            }
            _ => self.ram[addr] = data,
        }
    }

    fn get_carry(&self) -> u8 {
        self.ram[REG_STATUS] & 1
    }

    fn set_carry(&mut self, carry: bool) {
        let d = self.ram[REG_STATUS];
        self.ram[REG_STATUS] = if carry { d | 1 } else { d & !1 };
    }

    fn set_dc(&mut self, dc: bool) {
        let d = self.ram[REG_STATUS];
        self.ram[REG_STATUS] = if dc { d | 2 } else { d & !2 };
    }

    fn set_zero(&mut self, val: u8) {
        let d = self.ram[REG_STATUS];
        self.ram[REG_STATUS] = if val == 0 { d | 4 } else { d & !4 };
    }

    fn debug1(&self, s: &str) {
        if self.debugmode {
            let pc = ((self.ram[REG_PCLATH] as usize) << 8) + self.ram[REG_PCL] as usize;
            let (porta, portb) = self.gpio_out();
            let (trisa, trisb) = self.gpio_getdirection();
            println!(
                "{pc:04x}:  {s}\t\tZdC: {:03b}  W:{:02x}\t\t PA:{porta:05b} PB:{portb:08b}  (tra:{trisa:05b} trb:{trisb:08b})",
                self.ram[REG_STATUS] & 0x07,
                self.wreg
            );
        }
    }

    fn debug2(&self, s: &str, num: u8, num_address: bool) {
        if self.debugmode {
            let pc = ((self.ram[REG_PCLATH] as usize) << 8) + self.ram[REG_PCL] as usize;
            let (porta, portb) = self.gpio_out();
            let (trisa, trisb) = self.gpio_getdirection();
            if num_address {
                println!(
                    "{pc:04x}:  {s}\t{num}\tZdC: {:03b}  W:{:02x}\tMEMx:{:02x}\t PA:{porta:05b} PB:{portb:08b}  (tra:{trisa:05b} trb:{trisb:08b})",
                    self.ram[REG_STATUS] & 0x07,
                    self.wreg,
                    self.ramrd(num)
                );
            } else {
                println!(
                    "{pc:04x}:  {s}\t{num}\tZdC: {:03b}  W:{:02x}",
                    self.ram[REG_STATUS] & 0x07,
                    self.wreg,
                );
            }
        }
    }

    fn debug3(&self, s: &str, addr: u8, dst: bool) {
        if self.debugmode {
            let pc = ((self.ram[REG_PCLATH] as usize) << 8) + self.ram[REG_PCL] as usize;
            let (porta, portb) = self.gpio_out();
            let (trisa, trisb) = self.gpio_getdirection();
            println!(
                "{pc:04x}:  {s}\t{addr},{}\tZdC: {:03b}  W:{:02x}\tMEMx:{:02x}\t PA:{porta:05b} PB:{portb:08b}  (tra:{trisa:05b} trb:{trisb:08b})",
                if dst { 'w' } else { 'f' },
                self.ram[REG_STATUS] & 0x07,
                self.wreg,
                self.ramrd(addr)
            );
        }
    }

    fn debug3num(&self, s: &str, addr: u8, num: u8) {
        if self.debugmode {
            let pc = ((self.ram[REG_PCLATH] as usize) << 8) + self.ram[REG_PCL] as usize;
            let (porta, portb) = self.gpio_out();
            let (trisa, trisb) = self.gpio_getdirection();
            println!(
                "{pc:04x}:  {s}\t{addr},{num}\t\t\tMEMx:{:02x}\t PA:{porta:05b} PB:{portb:08b}  (tra:{trisa:05b} trb:{trisb:08b})",
                self.ramrd(addr)
            );
        }
    }

    fn debugjmp(&self, s: &str, jmp: usize) {
        if self.debugmode {
            let pc = ((self.ram[REG_PCLATH] as usize) << 8) + self.ram[REG_PCL] as usize;
            println!("{pc:04x}:  {s}\t{:#02x}", jmp as u16);
        }
    }

    // ------------------
    // CPU core function
    // ------------------
    fn cpu_core(&mut self) {
        if self.sleep {
            return;
        }
        let mut pc = ((self.ram[REG_PCLATH] as usize) << 8) + self.ram[REG_PCL] as usize;
        let mut skip_increment_pc = false;
        let memptr = self.rom[pc] as u8 & 0x7f;
        let dst_wreg = self.rom[pc] & 0x80 == 0;
        let bitsetcnt = (self.rom[pc] >> 7) as u8 & 7;

        if self.returnflag {
            pc = self.stack[self.stackptr];
            self.stackptr = self.stackptr.wrapping_sub(1) & 0x07;
            self.returnflag = false;

            self.skipnext = true;
            skip_increment_pc = true;
        }
        if self.callflag {
            self.callflag = false;
            return;
        }

        if !self.skipnext {
            match self.rom[pc] >> 8 {
                0b00_0111 => {
                    let (res, owf) = self.ramrd(memptr).overflowing_add(self.wreg);
                    let dc = (self.ramrd(memptr) & 0x0f) + (self.wreg & 0x0f);
                    self.ramwr(memptr, res, dst_wreg);
                    self.set_carry(owf);
                    self.set_dc(dc >> 4 != 0);
                    self.set_zero(res);
                    self.debug3("ADDWF", memptr, dst_wreg);
                }
                0b00_0101 => {
                    let res = self.ramrd(memptr) & self.wreg;
                    self.ramwr(memptr, res, dst_wreg);
                    self.set_zero(res);
                    self.debug3("ANDWF", memptr, dst_wreg);
                }
                0b00_0001 => {
                    self.ramwr(memptr, 0, dst_wreg);
                    self.set_zero(0);
                    if dst_wreg {
                        self.debug1("CLRW")
                    } else {
                        self.debug2("CLRF", memptr, true)
                    }
                }
                0b00_1001 => {
                    let res = self.ramrd(memptr) ^ 0xff;
                    self.ramwr(memptr, res, dst_wreg);
                    self.set_zero(res);
                    self.debug3("COMF", memptr, dst_wreg);
                } // COMF (negation)
                0b00_0011 => {
                    let res = self.ramrd(memptr).wrapping_sub(1);
                    self.ramwr(memptr, res, dst_wreg);
                    self.set_zero(res);
                    self.debug3("DECF", memptr, dst_wreg);
                }
                0b00_1011 => {
                    let res = self.ramrd(memptr).wrapping_sub(1);
                    self.ramwr(memptr, res, dst_wreg);
                    if res == 0 {
                        self.skipnext = true
                    }
                    self.debug3("DECFSZ", memptr, dst_wreg);
                }
                0b00_1010 => {
                    let res = self.ramrd(memptr).wrapping_add(1);
                    self.ramwr(memptr, res, dst_wreg);
                    self.set_zero(res);
                    self.debug3("INCF", memptr, dst_wreg);
                }
                0b00_1111 => {
                    let res = self.ramrd(memptr).wrapping_add(1);
                    self.ramwr(memptr, res, dst_wreg);
                    if res == 0 {
                        self.skipnext = true;
                    }
                    self.debug3("INCFSZ", memptr, dst_wreg);
                }
                0b00_0100 => {
                    let res = self.ramrd(memptr) | self.wreg;
                    self.ramwr(memptr, res, dst_wreg);
                    self.set_zero(res);
                    self.debug3("IORWF", memptr, dst_wreg);
                }
                0b00_1000 => {
                    let res = self.ramrd(memptr);
                    self.ramwr(memptr, res, dst_wreg);
                    self.set_zero(res);
                    self.debug3("MOVF", memptr, dst_wreg);
                }
                0b00_0000 => {
                    match self.rom[pc] as u8 {
                        0x64 => {
                            self.psc_ct = 0;
                            self.ram[REG_STATUS] |= 0x18; // TO:1 PD:1
                            self.debug1("CLRWDT");
                        }
                        0x01 => {
                            self.ram[REG_INTCON] |= 0x80; // GIE set
                            self.returnflag = true;
                            self.debug1("RETFIE");
                        } // RETFIE. 2 cycle
                        0x04 => {
                            self.returnflag = true;
                            self.debug1("RETURN");
                        } // RETURN, 2 cycle
                        0x63 => {
                            self.sleep = true;
                            self.psc_ct = 0;
                            self.ram[REG_STATUS] &= !0x08;
                            self.ram[REG_STATUS] |= 0x10; // TO:1 PD:0
                            self.debug1("SLEEP");
                        }
                        _ => {
                            self.ramwr(memptr, self.wreg, dst_wreg);
                            if dst_wreg {
                                self.debug1("NOP");
                            } else {
                                self.debug3("MOVWF", memptr, dst_wreg);
                            }
                        }
                    }
                }
                0b00_1101 => {
                    let data = self.ramrd(memptr);
                    self.ramwr(memptr, data << 1 | self.get_carry(), dst_wreg);
                    self.set_carry(data & 0x80 != 0);
                    self.debug2("RLF", memptr, true);
                }
                0b00_1100 => {
                    let data = self.ramrd(memptr);
                    self.ramwr(memptr, data >> 1 | (self.get_carry() << 7), dst_wreg);
                    self.set_carry(data & 0x01 != 0);
                    self.debug2("RRF", memptr, true);
                }
                0b00_0010 => {
                    let (res, owf) = self.ramrd(memptr).overflowing_sub(self.wreg);
                    let dc = (self.ramrd(memptr) & 0x0f) - (self.wreg & 0x0f);
                    self.ramwr(memptr, res, dst_wreg);
                    self.set_carry(owf);
                    self.set_dc(dc >> 4 != 0);
                    self.set_zero(res);
                    self.debug3("SUBWF", memptr, dst_wreg);
                }
                0b00_1110 => {
                    let r = self.ramrd(memptr);
                    self.ramwr(memptr, (r << 4) | (r >> 4), dst_wreg);
                    self.debug3("SWAPF", memptr, dst_wreg);
                }

                0b00_0110 => {
                    let res = self.ramrd(memptr) ^ self.wreg;
                    self.ramwr(memptr, res, dst_wreg);
                    self.set_zero(res);
                    self.debug3("XORWF", memptr, dst_wreg);
                }

                0b01_0000..=0b01_0011 => {
                    self.ramwr(memptr, self.ramrd(memptr) & !(1 << bitsetcnt), false);
                    self.debug3num("BCF", memptr, bitsetcnt);
                }
                0b01_0100..=0b01_0111 => {
                    self.ramwr(memptr, self.ramrd(memptr) | (1 << bitsetcnt), false);
                    self.debug3num("BSF", memptr, bitsetcnt);
                }
                0b01_1000..=0b01_1011 => {
                    if self.ramrd(memptr) & (1 << bitsetcnt) == 0 {
                        self.skipnext = true
                    }
                    self.debug3num("BTFSC", memptr, bitsetcnt);
                }
                0b01_1100..=0b01_1111 => {
                    if self.ramrd(memptr) & (1 << bitsetcnt) == 1 {
                        self.skipnext = true
                    }
                    self.debug3num("BTFSS", memptr, bitsetcnt);
                }

                0b11_1110 | 0b11_1111 => {
                    let (res, owf) = (self.rom[pc] as u8).overflowing_add(self.wreg);
                    let dc = (self.rom[pc] & 0x0f) as u8 - (self.wreg & 0x0f);
                    self.wreg = res;
                    self.set_carry(owf);
                    self.set_dc(dc >> 4 != 0);
                    self.set_zero(res);
                    self.debug2("ADDLW", self.rom[pc] as u8, false);
                }
                0b11_1001 => {
                    let res = self.rom[pc] as u8 & self.wreg;
                    self.wreg = res;
                    self.set_zero(res);
                    self.debug2("ANDLW", self.rom[pc] as u8, false);
                }
                0b10_0000..=0b10_0111 => {
                    self.stackptr = self.stackptr.wrapping_add(1) & 0x07;
                    self.stack[self.stackptr] = pc;
                    pc = (pc & !0x7ff) | (self.rom[pc] as usize & 0x7ff);
                    skip_increment_pc = true;
                    self.debugjmp("CALL", pc);
                }
                // CLRWDT: see in MOVWF section
                0b10_1000..=0b10_1111 => {
                    pc = (pc & !0x7ff) | (self.rom[pc] as usize & 0x7ff);
                    skip_increment_pc = true;
                    self.debugjmp("GOTO", pc);
                }
                0b11_1000 => {
                    let res = self.rom[pc] as u8 | self.wreg;
                    self.wreg = res;
                    self.set_zero(res);
                    self.debug2("IORLW", self.rom[pc] as u8, false);
                }
                0b11_0000..=0b11_0011 => {
                    self.wreg = self.rom[pc] as u8;
                    self.debug2("MOVLW", self.rom[pc] as u8, false);
                } // MOWLW
                // RETFIE: see in MOVWF section
                0b11_0100..=0b11_0111 => {
                    self.wreg = self.rom[pc] as u8;
                    self.returnflag = true;
                    self.debug2("RETLW", self.rom[pc] as u8, false);
                } // RETLW, 2 cycle
                // RETURN: see in MOVWF section
                // SLEEP: see in MOVWF section
                0b11_1100 | 0b11_1101 => {
                    let (res, owf) = (self.rom[pc] as u8).overflowing_sub(self.wreg);
                    let dc = (self.rom[pc] & 0x0f) as u8 - (self.wreg & 0x0f);
                    self.wreg = res;
                    self.set_carry(owf);
                    self.set_dc(dc >> 4 != 0);
                    self.set_zero(res);
                    self.debug2("SUBLW", self.rom[pc] as u8, false);
                }
                0b11_1010 => {
                    let res = self.rom[pc] as u8 ^ self.wreg;
                    self.wreg = res;
                    self.set_zero(res);
                    self.debug2("XORLW", self.rom[pc] as u8, false);
                }
                _ => println!("Unknown opcode: {:04x}", self.rom[pc]),
            }
        } else {
            self.skipnext = false;
        }
        if !skip_increment_pc {
            pc += 1;
            pc &= ROMSIZE - 1;
        }
        self.ram[REG_PCLATH] = (pc >> 8) as u8;
        self.ram[REG_PCL] = pc as u8;
    }

    // ---------------------------
    // Special hardware components
    // ---------------------------

    fn interrupt_activate(&mut self) {
        let pc = 4; // IRQ entry point
        self.ram[REG_PCLATH] = (pc >> 8) as u8;
        self.ram[REG_PCL] = pc as u8;
        self.ram[REG_INTCON] &= !0x80; // GIE 0
    }

    fn portb_change_irq(&mut self, pbold: u8, pb: u8) {
        // GIE + PORT_CHANGE_ENABLE - PB7..PB4
        if pbold & 0xf0 != pb & 0xf0 {
            if self.ram[REG_INTCON] & 0x89 == 0x88 {
                self.interrupt_activate();
            }
            self.ram[REG_INTCON] |= 1; // Port change int flag
        }
        // GIE + RB0/INT source
        if (self.option_reg & 0x40 == 0 && pbold & 1 == 1 && pb & 1 == 0)
            || (self.option_reg & 0x40 != 0 && pbold & 1 == 0 && pb & 1 == 1)
        {
            if self.ram[REG_INTCON] & 0x92 == 0x90 {
                self.interrupt_activate();
            }
            self.ram[REG_INTCON] |= 2; // RB0 int flag
        }
    }

    fn timer_wdt(&mut self, from_nextclk: bool) {
        // called from_nextclk and from gpio_in
        if self.option_reg & 0x20 == 0x20 && from_nextclk {
            return;
        };
        let mut tmr_event = false;
        self.psc_ct = self.psc_ct.wrapping_add(1);
        if self.psc_ct & (1 << (self.option_reg & 7)) != 0 {
            self.psc_ct = 0;
            // TMR0 or WDT
            if self.option_reg & 0x08 == 0 {
                tmr_event = true;
            } else if self.configbits & 4 != 0 {
                self.reset()
            }
        }
        // TMR0 without PSA  or  PSA event
        if self.option_reg & 0x08 != 0 || tmr_event {
            self.tmrdiv2 = !self.tmrdiv2;
            if !self.tmrdiv2 {
                let (ct, owf) = self.ram[REG_TMR0].overflowing_add(1);
                self.ram[REG_TMR0] = ct;
                if owf {
                    if self.ram[REG_INTCON] & 0xa4 == 0xa0 {
                        self.interrupt_activate();
                    }
                    self.ram[REG_INTCON] |= 4; // RB0 int flag
                }
            }
        }
    }

    fn eeprom(&mut self) {
        // Checking of eecon2 0x55 0xaa steps are missing here.
        if self.eecon1 & 6 == 6 {
            self.eeprom[self.ram[REG_EEADDR] as usize] = self.ram[REG_EEDATA];
            if self.ram[REG_INTCON] & 0xc0 == 0xc0 && self.eecon1 & 0x10 == 0 {
                self.interrupt_activate();
            }
            self.eecon1 &= !2;
            self.eecon1 |= 0x10;
        }
        if self.eecon1 & 1 == 1 {
            self.ram[REG_EEDATA] = self.eeprom[self.ram[REG_EEADDR] as usize];
            self.eecon1 &= !1;
        }
    }
}
