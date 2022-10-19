// Copyright Claudio Mattera 2022.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

pub const FIRST_REGISTER: u8 = 0x88;
pub const FIRST_LENGTH: usize = 26;
pub const SECOND_REGISTER: u8 = 0xe1;
pub const SECOND_LENGTH: usize = 7;
pub const TOTAL_LENGTH: usize = FIRST_LENGTH + SECOND_LENGTH;

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct CalibrationData {
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,

    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,

    pub dig_h1: u8,
    pub dig_h2: i16,
    pub dig_h3: u8,
    pub dig_h4: i16,
    pub dig_h5: i16,
    pub dig_h6: i8,
}

impl From<&[u8; TOTAL_LENGTH]> for CalibrationData {
    fn from(data: &[u8; TOTAL_LENGTH]) -> Self {
        Self {
            dig_t1: u16::from_le_bytes([data[0], data[1]]),
            dig_t2: i16::from_le_bytes([data[2], data[3]]),
            dig_t3: i16::from_le_bytes([data[4], data[5]]),

            dig_p1: u16::from_le_bytes([data[6], data[7]]),
            dig_p2: i16::from_le_bytes([data[8], data[9]]),
            dig_p3: i16::from_le_bytes([data[10], data[11]]),
            dig_p4: i16::from_le_bytes([data[12], data[13]]),
            dig_p5: i16::from_le_bytes([data[14], data[15]]),
            dig_p6: i16::from_le_bytes([data[16], data[17]]),
            dig_p7: i16::from_le_bytes([data[18], data[19]]),
            dig_p8: i16::from_le_bytes([data[20], data[21]]),
            dig_p9: i16::from_le_bytes([data[22], data[23]]),

            dig_h1: data[25],
            dig_h2: i16::from_le_bytes([data[26], data[27]]),
            dig_h3: data[28],
            dig_h4: (data[29] as i16) << 4 | (data[30] as i16) & 0xF,
            dig_h5: (((data[30] as i16) & 0xF0) >> 4) | ((data[31] as i16) << 4),
            dig_h6: data[32] as i8,
        }
    }
}

impl CalibrationData {
    pub fn compensate_temperature(&self, adc_t: u32) -> u32 {
        let var1 =
            ((((adc_t as i32) >> 3) - ((self.dig_t1 as i32) << 1)) * (self.dig_t2 as i32)) >> 11;
        let var2 = ((((((adc_t as i32) >> 4) - (self.dig_t1 as i32))
            * (((adc_t as i32) >> 4) - (self.dig_t1 as i32)))
            >> 12)
            * (self.dig_t3 as i32))
            >> 14;

        let t_fine = var1 + var2;

        t_fine as u32
    }

    pub fn compensate_pressure(&self, adc_p: u32, t_fine: u32) -> u32 {
        let var1 = (t_fine as i64) - 128_000;
        let var2 = var1 * var1 * self.dig_p6 as i64;
        let var2 = var2 + ((var1 * (self.dig_p5 as i64)) << 17);
        let var2 = var2 + ((self.dig_p4 as i64) << 35);
        let var1 =
            ((var1 * var1 * (self.dig_p3 as i64)) >> 8) + ((var1 * (self.dig_p2 as i64)) << 12);
        let var1 = ((((1_i64) << 47) + var1) * (self.dig_p1 as i64)) >> 33;

        if var1 == 0 {
            // division by zero
            0
        } else {
            let var4 = 1_048_576 - adc_p as i64;
            let var4 = ((((var4 as i64) << 31) - var2) * 3125) / var1;
            let var1 = ((self.dig_p9 as i64) * ((var4 as i64) >> 13) * ((var4 as i64) >> 13)) >> 25;
            let var2 = ((self.dig_p8 as i64) * var4) >> 19;
            let var5 = ((var4 + var1 + var2) >> 8) + ((self.dig_p7 as i64) << 4);

            let p = var5;

            p as u32
        }
    }

    pub fn compensate_humidity(&self, adc_h: u16, t_fine: u32) -> u32 {
        let adc_h = adc_h as i32;

        let v_x1_u32r: i32 = t_fine as i32 - 76_800_i32;
        let v_x1_u32r: i32 = (((((adc_h as i32) << 14)
            - ((self.dig_h4 as i32) << 20)
            - ((self.dig_h5 as i32) * v_x1_u32r))
            + (16_384_i32))
            >> 15)
            * (((((((v_x1_u32r * (self.dig_h6 as i32)) >> 10)
                * (((v_x1_u32r * (self.dig_h3 as i32)) >> 11) + (32_768_i32)))
                >> 10)
                + (2_097_152_i32))
                * (self.dig_h2 as i32)
                + 8192_i32)
                >> 14);
        let v_x1_u32r: i32 = v_x1_u32r
            - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (self.dig_h1 as i32)) >> 4);
        let v_x1_u32r = if v_x1_u32r < 0 { 0 } else { v_x1_u32r };
        let v_x1_u32r = if v_x1_u32r > 419_430_400 {
            419_430_400
        } else {
            v_x1_u32r
        };
        (v_x1_u32r >> 12) as u32
    }
}

#[cfg(test)]
pub use tests::TEST_CALIBRATION_DATA;

#[cfg(test)]
mod tests {
    use super::*;

    use assert2::check;

    pub const TEST_CALIBRATION_DATA: CalibrationData = CalibrationData {
        dig_t1: 0x6fb1_u16,
        dig_t2: 0x6a2e_i16,
        dig_t3: 0x32_i16,

        dig_p1: 0x8fe2_u16,
        dig_p2: -10486_i16,
        dig_p3: 0x0bd0_i16,
        dig_p4: 0x1f75_i16,
        dig_p5: -59_i16,
        dig_p6: -7_i16,
        dig_p7: 0x26ac_i16,
        dig_p8: -10230_i16,
        dig_p9: 0x10bd_i16,

        dig_h1: 0x4b_u8,
        dig_h2: 0x0177_i16,
        dig_h3: 0x00_u8,
        dig_h4: 0x0120_i16,
        dig_h5: 0x0032_i16,
        dig_h6: 0x001e_i8,
    };

    #[test]
    fn test_parse_calibration_data() {
        let raw_register_data: [u8; TOTAL_LENGTH] = [
            0xb1, 0x6f, 0x2e, 0x6a, 0x32, 0x00, 0xe2, 0x8f, 0x0a, 0xd7, 0xd0, 0x0b, 0x75, 0x1f,
            0xc5, 0xff, 0xf9, 0xff, 0xac, 0x26, 0x0a, 0xd8, 0xbd, 0x10, 0x00, 0x4b, 0x77, 0x01,
            0x00, 0x12, 0x20, 0x03, 0x1e,
        ];

        let coefficients = CalibrationData::from(&raw_register_data);

        check!(coefficients == TEST_CALIBRATION_DATA);
    }

    #[test]
    fn test_compensate_temperature() {
        let adc_t = 0x84510;

        let actual = TEST_CALIBRATION_DATA.compensate_temperature(adc_t);
        let expected = 0x22391;

        check!(actual == expected, "0x{:08X} == 0x{:08X}", actual, expected);
    }

    #[test]
    fn test_compensate_pressure() {
        let adc_p = 0x4f570;
        let t_fine = 0x22391;

        let actual = TEST_CALIBRATION_DATA.compensate_pressure(adc_p, t_fine);
        let expected = 0x018b_663f;

        check!(actual == expected, "0x{:08X} == 0x{:08X}", actual, expected);
    }

    #[test]
    fn test_compensate_humidity() {
        let adc_h = 0x5eb4;
        let t_fine = 0x22391;

        let actual = TEST_CALIBRATION_DATA.compensate_humidity(adc_h, t_fine);
        let expected = 0x8399;

        check!(actual == expected, "0x{:08X} == 0x{:08X}", actual, expected);
    }
}
