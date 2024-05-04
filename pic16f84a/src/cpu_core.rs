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

const REG_INDF: u8 = 0x00;
const REG_TMR0: u8 = 0x01; // OPTION
const REG_PCL: u8 = 0x02;
const REG_STATUS: u8 = 0x03;
const REG_FSR: u8 = 0x04;
const REG_PORTA: u8 = 0x05; // TRISA
const REG_PORTB: u8 = 0x06; // TRISB
                            //const REG_UNIMPLEMENTED: u8 = 0x07;
const REG_EEDATA: u8 = 0x08; // EECON1
const REG_EEADDR: u8 = 0x09; // EECON2
const REG_PCLATH: u8 = 0x0a;
const REG_INTCON: u8 = 0x0b;

pub struct Cpu {
    //pc: usize,           // 13 bit
    skipnext: bool,
    wreg: u8,           // 8 bit
    ram: [u8; 0x50],    // 8 bit
    rom: [u16; 0x400],  // 14 bit
    eeprom: [u8; 0x40], // pic16f84a has 64 bytes EEPROM
    stack: [usize; 8],  // 13 bit
    stackptr: u8,       // 8 level stack

    wdt: u8,           // watchdog timer
    wdt_prescaler: u8, // watchdog timer prescaler
    sleep: bool,       // sleep instruction, cpu halt
    callflag: bool,    // 2 cycle
    returnflag: bool,  // 2 cycle

    // special register from rambank1
    option_reg: u8,
    trisa: u8,
    trisb: u8,
    eecon1: u8,
    eecon2: u8, // not a physical register

    debugmode: bool,
}

impl Cpu {
    pub fn new(prog: &[u16], _configbits: u16, eeprom: &[u8]) -> Self {
        let mut cpu = Cpu {
            //pc: 0,
            skipnext: false,
            wreg: 0,
            ram: [0; 0x50],
            rom: [0x00; ROMSIZE],
            eeprom: [0x00; EEPROMSIZE],
            stack: [0; 8],
            stackptr: 0,
            wdt: 0,
            wdt_prescaler: 0,
            sleep: false,
            callflag: false,
            returnflag: false,

            trisa: 0,
            trisb: 0,
            option_reg: 0,
            eecon1: 0,
            eecon2: 0, // not a physical register
            debugmode: false,
        };
        cpu.rom[..prog.len()].copy_from_slice(prog);
        cpu.eeprom[..eeprom.len()].copy_from_slice(eeprom);
        cpu.reset();
        cpu
    }

    pub fn reset(&mut self) {
        self.ram[REG_PCL as usize] = 0x00;
        self.ram[REG_STATUS as usize] = 0b0001_1000 | (self.ram[REG_STATUS as usize] & 0x07);
        self.ram[REG_PCLATH as usize] = 0x00;
        self.ram[REG_INTCON as usize] &= 1;
        self.option_reg = 0xff;
        self.trisa = 0xff;
        self.trisb = 0xff;
        self.eecon1 &= 0x08;
    }

    pub fn nextclk(&mut self) {
        self.cpu_core();
        self.timer();
        self.watchdog();
        self.eeprom();
    }

    // Physical GPIO input
    pub fn gpio_in(&mut self, porta: u8, portb: u8) {
        self.ram[REG_PORTA as usize] =
            (self.ram[REG_PORTA as usize] & !self.trisa) | (porta & self.trisa);
        self.ram[REG_PORTB as usize] =
            (self.ram[REG_PORTB as usize] & !self.trisb) | (portb & self.trisb);
    }

    // Physical GPIO output
    pub fn gpio_out(&self) -> (u8, u8) {
        (self.ram[REG_PORTA as usize], self.ram[REG_PORTB as usize])
    }

    pub fn gpio_getdir(&self) -> (u8, u8) {
        (self.trisa, self.trisb)
    }

    // ------------------
    // Internal functions
    // ------------------

    fn get_rambank(&self) -> u8 {
        (self.ram[REG_STATUS as usize] & 0x20 != 0) as u8
    }

    fn ramrd(&self, addr: u8) -> u8 {
        // special rambank1 cases
        if self.get_rambank() == 1 {
            match addr {
                REG_TMR0 => return self.option_reg,
                REG_PORTA => return self.trisa,
                REG_PORTB => return self.trisb,
                REG_EEDATA => return self.eecon1,
                REG_EEADDR => return self.eecon2,
                _ => (),
            }
        }
        // Remark: all of other registers emulated from RAM
        match addr {
            REG_INDF => self.ram[self.ram[REG_FSR as usize] as usize], // INDF, not real mem
            _ => self.ram[addr as usize],
        }
    }

    fn ramwr(&mut self, addr: u8, data: u8, dst_wreg: bool) {
        // DST: wreg
        if dst_wreg {
            self.wreg = data;
            return;
        }
        // DST: RAM or special registers - first: special rambank1 cases
        if self.get_rambank() == 1 {
            match addr {
                REG_TMR0 => {
                    self.option_reg = data;
                    return;
                }
                REG_PORTA => {
                    self.trisa = data;
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
                    self.eecon2 = data;
                    return;
                }
                _ => (),
            }
        }
        match addr {
            REG_INDF => self.ram[self.ram[REG_INDF as usize] as usize] = data, // INDF, not real mem
            REG_PORTA => {
                self.ram[REG_PORTA as usize] =
                    (self.ram[REG_PORTA as usize] & self.trisa) | (data & !self.trisa)
            }
            REG_PORTB => {
                self.ram[REG_PORTB as usize] =
                    (self.ram[REG_PORTB as usize] & self.trisb) | (data & !self.trisb)
            }
            _ => self.ram[addr as usize] = data,
        }
    }

    fn get_carry(&self) -> u8 {
        self.ramrd(REG_STATUS) & 1
    }

    fn set_carry(&mut self, carry: bool) {
        self.ramwr(
            REG_STATUS,
            if carry {
                self.ramrd(REG_STATUS) | 1
            } else {
                self.ramrd(REG_STATUS) & !1
            },
            false,
        );
    }

    fn set_dc(&mut self, dc: bool) {
        self.ramwr(
            REG_STATUS,
            if dc {
                self.ramrd(REG_STATUS) | 2
            } else {
                self.ramrd(REG_STATUS) & !2
            },
            false,
        );
    }

    fn set_zero(&mut self, val: u8) {
        self.ramwr(
            REG_STATUS,
            if val == 0 {
                self.ramrd(REG_STATUS) | 4
            } else {
                self.ramrd(REG_STATUS) & !4
            },
            false,
        );
    }

    fn debug1(&self, s: &str) {
        if self.debugmode {
            let pc = ((self.ramrd(REG_PCLATH) as usize) << 8) + self.ramrd(REG_PCL) as usize;
            println!(
                "{pc:04x}:  {s}\t\tZdC: {:03b}  W:{:02x}",
                self.ramrd(REG_STATUS) & 0x07,
                self.wreg
            );
        }
    }

    fn debug2(&self, s: &str, num: u8, num_address: bool) {
        if self.debugmode {
            let pc = ((self.ramrd(REG_PCLATH) as usize) << 8) + self.ramrd(REG_PCL) as usize;
            if num_address {
                println!(
                    "{pc:04x}:  {s}\t{num}\tZdC: {:03b}  W:{:02x}\tMEMx:{:02x}",
                    self.ramrd(REG_STATUS) & 0x07,
                    self.wreg,
                    self.ramrd(num)
                );
            } else {
                println!(
                    "{pc:04x}:  {s}\t{num}\tZdC: {:03b}  W:{:02x}",
                    self.ramrd(REG_STATUS) & 0x07,
                    self.wreg,
                );
            }
        }
    }

    fn debug3(&self, s: &str, addr: u8, dst: bool) {
        if self.debugmode {
            let pc = ((self.ramrd(REG_PCLATH) as usize) << 8) + self.ramrd(REG_PCL) as usize;
            println!(
                "{pc:04x}:  {s}\t{addr},{}\tZdC: {:03b}  W:{:02x}\tMEMx:{:02x}",
                if dst { 'w' } else { 'f' },
                self.ramrd(REG_STATUS) & 0x07,
                self.wreg,
                self.ramrd(addr)
            );
        }
    }

    fn debug3num(&self, s: &str, addr: u8, num: u8) {
        if self.debugmode {
            let pc = ((self.ramrd(REG_PCLATH) as usize) << 8) + self.ramrd(REG_PCL) as usize;
            println!(
                "{pc:04x}:  {s}\t{addr},{num}\t\t\tMEMx:{:02x}",
                self.ramrd(addr)
            );
        }
    }

    fn debugjmp(&self, s: &str, jmp: usize) {
        if self.debugmode {
            let pc = ((self.ramrd(0x0a) as usize) << 8) + self.ramrd(0x02) as usize; // PCLATH and PCL
            println!("{pc:04x}:  {s}\t{:#02x}", jmp as u16);
        }
    }

    pub fn set_debug(&mut self, debugmode: bool) {
        self.debugmode = debugmode;
    }

    // ------------------
    // CPU core function
    // ------------------
    fn cpu_core(&mut self) {
        if self.sleep {
            return;
        }
        let mut pc = ((self.ramrd(REG_PCLATH) as usize) << 8) + self.ramrd(REG_PCL) as usize;
        let mut skip_increment_pc = false;
        let memptr = self.rom[pc] as u8 & 0x7f;
        let dst_wreg = self.rom[pc] & 0x80 == 0;
        let bitsetcnt = (self.rom[pc] >> 7) as u8 & 7;

        if self.returnflag {
            pc = self.stack[self.stackptr as usize];
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
                            self.wdt = 0;
                            self.wdt_prescaler = 0;
                            self.ramwr(REG_STATUS, self.ramrd(REG_STATUS) | 0x18, false); // TO:1 PD:1
                            self.debug1("CLRWDT");
                        }
                        0x01 => {
                            self.ramwr(0x0b, self.ramrd(0x0b) | 0x80, false); // GIE set
                            self.returnflag = true;
                            self.debug1("RETFIE");
                        } // RETFIE. 2 cycle
                        0x04 => {
                            self.returnflag = true;
                            self.debug1("RETURN");
                        } // RETURN, 2 cycle
                        0x63 => {
                            self.sleep = true;
                            self.wdt = 0;
                            self.wdt_prescaler = 0;
                            self.ramwr(REG_STATUS, (self.ramrd(REG_STATUS) & !0x08) | 0x10, false); // TO:1 PD:0
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
                    self.ramwr(0, res, true);
                    self.set_carry(owf);
                    self.set_dc(dc >> 4 != 0);
                    self.set_zero(res);
                    self.debug2("ADDLW", self.rom[pc] as u8, false);
                }
                0b11_1001 => {
                    let res = self.rom[pc] as u8 & self.wreg;
                    self.ramwr(0, res, true);
                    self.set_zero(res);
                    self.debug2("ANDLW", self.rom[pc] as u8, false);
                }
                0b10_0000..=0b10_0111 => {
                    self.stackptr = self.stackptr.wrapping_add(1) & 0x07;
                    self.stack[self.stackptr as usize] = pc;
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
                    self.ramwr(0, res, true);
                    self.set_zero(res);
                    self.debug2("IORLW", self.rom[pc] as u8, false);
                }
                0b11_0000..=0b11_0011 => {
                    self.ramwr(0, self.rom[pc] as u8, true);
                    self.debug2("MOVLW", self.rom[pc] as u8, false);
                } // MOWLW
                // RETFIE: see in MOVWF section
                0b11_0100..=0b11_0111 => {
                    self.ramwr(0, self.rom[pc] as u8, true);
                    self.returnflag = true;
                    self.debug2("RETLW", self.rom[pc] as u8, false);
                } // RETLW, 2 cycle
                // RETURN: see in MOVWF section
                // SLEEP: see in MOVWF section
                0b11_1100 | 0b11_1101 => {
                    let (res, owf) = (self.rom[pc] as u8).overflowing_sub(self.wreg);
                    let dc = (self.rom[pc] & 0x0f) as u8 - (self.wreg & 0x0f);
                    self.ramwr(0, res, true);
                    self.set_carry(owf);
                    self.set_dc(dc >> 4 != 0);
                    self.set_zero(res);
                    self.debug2("SUBLW", self.rom[pc] as u8, false);
                }
                0b11_1010 => {
                    let res = self.rom[pc] as u8 ^ self.wreg;
                    self.ramwr(0, res, true);
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
        self.ramwr(REG_PCLATH, (pc >> 8) as u8, false);
        self.ramwr(REG_PCL, pc as u8, false);
    }

    // ---------------------------
    // Special hardware components
    // ---------------------------

    fn timer(&mut self) {}

    fn watchdog(&mut self) {}

    fn eeprom(&mut self) {}
}
