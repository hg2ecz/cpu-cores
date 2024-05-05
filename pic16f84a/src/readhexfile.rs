use crate::cpu_core;
use std::fs::read_to_string;

fn asc2num(cv: &[char], reverz: bool) -> (u16, u8) {
    let mut val = 0;
    for &c in cv {
        val *= 0x10;
        val += match c {
            '0'..='9' => c as u16 - 0x30,
            'A'..='F' => c as u16 - 0x41 + 10,
            _ => panic!("Not a hexa digit!"),
        }
    }
    if reverz {
        val = val >> 8 | val << 8;
    }
    let chk = (val >> 8) + (val & 0xff);
    (val, chk as u8)
}

// Intel HEX format:
//   :     - line start
//   xx    - length (0..255 data byte)
//   xx xx - addr (16 bit)
//   xx    - type (00: data, 01: end)
//   xx .. - data bytes (length is above)
//   xx    - add values from previous bytes + this number, the result must be ZERO
pub fn readhexfile(filename: &str) -> (Vec<u16>, u16, Vec<u8>) {
    let mut code = vec![];
    let mut configbits = 0;
    let mut eeprom = vec![];
    let mut fileend = false;
    for line in read_to_string(filename).unwrap().lines() {
        let mut chksum = 0_u8;
        let cv: Vec<_> = line.chars().collect();
        if cv[0] != ':' {
            break;
        }
        let (lenbytes, chk) = asc2num(&cv[1..=2], false);
        chksum = chksum.wrapping_add(chk);
        let (addr, chk) = asc2num(&cv[3..=6], false);
        chksum = chksum.wrapping_add(chk);
        let (dtype, chk) = asc2num(&cv[7..=8], false);
        chksum = chksum.wrapping_add(chk);
        for i in 0..lenbytes as usize / 2 {
            let (val, chk) = asc2num(&cv[9 + 4 * i..=12 + 4 * i], true);
            chksum = chksum.wrapping_add(chk);
            match addr {
                0..=0x3fff => {
                    if code.len() < cpu_core::ROMSIZE {
                        code.push(val);
                    }
                }
                0x4200..=0x4400 => {
                    if eeprom.len() < cpu_core::EEPROMSIZE {
                        eeprom.push(val as u8);
                    }
                }
                0x400E => configbits = val,
                _ => (),
            }
        }
        let (_val, chk) = asc2num(
            &cv[9 + 2 * lenbytes as usize..=10 + 2 * lenbytes as usize],
            false,
        );
        chksum = chksum.wrapping_add(chk);
        if chksum != 0 {
            panic!("Panic: cheksum error.")
        }
        if dtype == 1 {
            fileend = true;
            break;
        }
    }
    if !fileend {
        panic!("Panic: end not found in file.")
    }
    (code, configbits, eeprom)
}
