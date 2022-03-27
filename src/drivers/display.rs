// SPDX-License-Identifier: GPL-3.0-or-later

use embassy_stm32::gpio::low_level::{AFType, Pin};
use embassy_stm32::gpio::{Output, Level, Speed};

use embassy_stm32::{rcc::low_level::RccPeripheral, pac::fsmc::vals};

use embassy_stm32::peripherals as p;

use crate::consts::display::*;

pub struct Display {
    pub reset: Output<'static, p::PG2>,
    pub backlight: Output<'static, p::PG8>,
}

impl Display {
    // We use Bank0 (0x60000000) to address the display.
    // The A16 wire is used to select the DATA or CMD register. Its address is
    // 0x00020000 = 1 << (16 + 1) (The +1 is because of the 16 bit addressing
    // mode as opposed to 8 bit).
    const TFT_CMD:  *mut u16 = 0x6000_0000u32 as *mut u16;
    const TFT_DATA: *mut u16 = 0x6002_0000u32 as *mut u16;

    /*
    pub const FULL_SCREEN: Rectangle = Rectangle::new(
        Point::new(0,0),
        Size::new(Self::WIDTH as u32, Self::HEIGHT as u32)
    );
    */

    #[inline(never)]
    pub fn new(
        reset: p::PG2,
        backlight: p::PG8,

        output_enable: p::PD4,
        write_enable: p::PD5,
        cs: p::PD7,
        a16: p::PD11,

        d0: p::PD14,
        d1: p::PD15,
        d2: p::PD0,
        d3: p::PD1,
        d4: p::PE7,
        d5: p::PE8,
        d6: p::PE9,
        d7: p::PE10,
        d8: p::PE11,
        d9: p::PE12,
        d10: p::PE13,
        d11: p::PE14,
        d12: p::PE15,
        d13: p::PD8,
        d14: p::PD9,
        d15: p::PD10,

        fsmc: p::FSMC,
    ) -> Self {
        let fsmc = embassy_stm32::pac::FSMC;
        p::FSMC::enable();

        let reset = Output::new(reset, Level::Low, Speed::Medium);
        let backlight = Output::new(backlight, Level::Low, Speed::Medium);

        unsafe {
            // PD4: EXMC_NOE: Output Enable
            output_enable.set_as_af(12, AFType::OutputPushPull);
            // PD5: EXMC_NWE: Write enable
            write_enable.set_as_af(12, AFType::OutputPushPull);
            // PD7: EXMC_NE0: Chip select
            cs.set_as_af(12, AFType::OutputPushPull);
            // A16: Selects the Command or Data register
            a16.set_as_af(12, AFType::OutputPushPull);

            d0.set_as_af(12, AFType::OutputPushPull);
            d1.set_as_af(12, AFType::OutputPushPull);
            d2.set_as_af(12, AFType::OutputPushPull);
            d3.set_as_af(12, AFType::OutputPushPull);
            d4.set_as_af(12, AFType::OutputPushPull);
            d5.set_as_af(12, AFType::OutputPushPull);
            d6.set_as_af(12, AFType::OutputPushPull);
            d7.set_as_af(12, AFType::OutputPushPull);
            d8.set_as_af(12, AFType::OutputPushPull);
            d9.set_as_af(12, AFType::OutputPushPull);
            d10.set_as_af(12, AFType::OutputPushPull);
            d11.set_as_af(12, AFType::OutputPushPull);
            d12.set_as_af(12, AFType::OutputPushPull);
            d13.set_as_af(12, AFType::OutputPushPull);
            d14.set_as_af(12, AFType::OutputPushPull);
            d15.set_as_af(12, AFType::OutputPushPull);
        }

        unsafe {
            fsmc.bcr1().write(|w| {
                // Enable NOR Bank 0
                w.set_mbken(vals::BcrMbken::ENABLED);
                // data width: 16 bits
                w.set_mwid(vals::BcrMwid::BITS16);
                // write: enable
                w.set_wren(vals::BcrWren::ENABLED);
            });

            fsmc.btr1().write(|w| {
                // Access Mode A
                w.set_accmod(vals::BtrAccmod::B);

                // Setup according to mono-x stock firmware
                w.set_addset(6);
                w.set_addhld(6);
                w.set_datast(20);
                w.set_datlat(0);
            });
        }

        Self { reset, backlight }
    }
 
    pub fn write_cmd(&mut self, v: u16) {
        unsafe { Self::TFT_CMD.write_volatile(v); }
    }

    pub fn write_data(&mut self, v: u16) {
        unsafe { Self::TFT_DATA.write_volatile(v); }
    }

    pub fn read_data(&mut self)->u16 {
        unsafe { return Self::TFT_DATA.read_volatile(); }
    }

    pub fn init(&mut self) {
        // This sequence is mostly taken from the original firmware
        delay_ms(10);
        self.reset.set_high();
        delay_ms(10);
        self.reset.set_low();
        delay_ms(80);
        self.reset.set_high();
        delay_ms(50);

        // Setup according to stock fw
        // Mono-x ILI9488 480x340
        self.reg_write (0xE0, &[0x00, 0x13, 0x18, 0x04, 0x0f, 0x06, 0x3a, 0x56, 0x4d, 0x03, 0x0a, 0x06, 0x30, 0x3e, 0x0f]);
        self.reg_write (0xE1, &[0x00, 0x13, 0x18, 0x01, 0x11, 0x06, 0x38, 0x34, 0x4d, 0x06, 0x0d, 0x0b, 0x31, 0x37, 0x0f]);
        self.reg_write (0xc0, &[0x18, 0x17]);
        self.reg_write (0xc1, &[0x41]);
        self.reg_write (0xc5, &[0x00, 0x40, 0x00, 0x40]);
        self.reg_write (0x36, &[0x2a]);
        self.reg_write (0x3a, &[0x55]);
        self.reg_write (0xb0, &[0x00]);
        self.reg_write (0xb4, &[0x01]);
        self.reg_write (0xb6, &[0x02, 0x02]);
        self.reg_write (0xe9, &[0x00]);
        self.reg_write (0xf7, &[0xa9, 0x51, 0x2c, 0x82]);
        self.reg_write (0x20, &[]);
        self.reg_write (0x11, &[]);
        self.reg_write (0x29, &[]);
        self.reg_write (0x2c, &[]);
        self.reg_write (0x2a, &[0x00, 0x00, 0x01, 0x01df]);
        self.reg_write (0x2b, &[0x00, 0x00, 0x01, 0x013f]);
        
        // Read display ID4
        let mut id = [0_u16; 4];
        self.reg_read (0xD3, &mut id);
        debug! ("read:{:?}", id);

        delay_ms(110);
    }

    pub fn write_data_as_two_u8(&mut self, v: u16) {
        self.write_data(v >> 8);
        self.write_data(v & 0xFF);
    }

    pub fn reg_write(&mut self, cmd: u16, args: &[u16]) {
        self.write_cmd(cmd);
        for a in args {
            self.write_data(*a);
        }
    }

    pub fn reg_read(&mut self, cmd: u16, dst: &mut [u16]) {
        self.write_cmd(cmd);
        for n in 0..dst.len() {
            dst[n] = self.read_data();
        }
    }

    pub fn start_drawing(&mut self, top_left: (u16, u16), bottom_right: (u16, u16)) {
        let (left, top) = top_left;
        let (right, bottom) =  bottom_right;

        self.write_cmd(0x2A);
        self.write_data_as_two_u8(left);
        self.write_data_as_two_u8(right - 1);
        self.write_cmd(0x2B);
        self.write_data_as_two_u8(top);
        self.write_data_as_two_u8(bottom - 1);
        self.write_cmd(0x2C);
    }

    pub fn start_drawing_full_screen(&mut self) {
        self.start_drawing((0,0), (WIDTH, HEIGHT));
    }

    pub fn fill_screen(&mut self, color: u16) {
        self.start_drawing_full_screen();
        for _ in 0..WIDTH {
            for _ in 0..HEIGHT {
                self.write_data(color);
            }
        }
    }

    /*
    pub fn draw_background_image(&mut self, ext_flash: &mut ExtFlash, img_index: u8, area: &Rectangle) {
        let area = area.intersection(&self.bounding_box());
        if area.is_zero_sized() {
            return;
        }

        let image_addr = 0x30000 * (img_index as u32);

        let width = area.size.width as u16;
        let left_col = area.top_left.x as u16;
        let right_col = left_col + width;

        const BYTES_PER_PIXEL: u32 = 2;

        let mut buf_ = [0u8; (BYTES_PER_PIXEL as usize)*Self::WIDTH as usize];

        for row in area.rows() {
            let buf = &mut buf_[0..(BYTES_PER_PIXEL as usize)*(width as usize)];
            let start_pixel_index = (row as u32) * (Self::WIDTH as u32) + left_col as u32;
            ext_flash.0.read(image_addr + BYTES_PER_PIXEL*start_pixel_index, buf).unwrap();

            let row = row as u16;
            self.start_drawing((left_col,  row),
                               (right_col, row+1));

            for i in 0..width {
                let i = i as usize;
                self.write_data(((buf[2*i+1] as u16) << 8) | buf[2*i] as u16);
            }
        }
    }
    */
}


// Embedded Graphics integration

use core::convert::TryInto;
use embedded_graphics::{
    prelude::*,
    pixelcolor::{Rgb565, raw::RawU16},
    primitives::Rectangle,
};

use super::delay_ms;

impl DrawTarget for Display {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            const W: i32 = WIDTH as i32;
            const H: i32 = HEIGHT as i32;
            if let Ok((x @ 0..=W, y @ 0..=H)) = coord.try_into() {
                let x = x as u16;
                let y = y as u16;
                self.start_drawing((x,y), (x+1,y+1));
                self.write_data(RawU16::from(color).into_inner());
            }
        }

        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        // Clamp area to drawable part of the display target
        let drawable_area = area.intersection(&self.bounding_box());

        // Check that there are visible pixels to be drawn
        if drawable_area.size != Size::zero() {
            let start = drawable_area.top_left;
            let end = drawable_area.bottom_right().unwrap();
            self.start_drawing((start.x as u16, start.y as u16),
                               ((end.x+1) as u16, (end.y+1) as u16));

            area.points()
                .zip(colors)
                .filter(|(pos, _color)| drawable_area.contains(*pos))
                .for_each(|(_, color)| self.write_data(RawU16::from(color).into_inner()));
        }
        Ok(())
    }
}

impl OriginDimensions for Display {
    fn size(&self) -> Size {
        Size::new(WIDTH.into(), HEIGHT.into())
    }
}
