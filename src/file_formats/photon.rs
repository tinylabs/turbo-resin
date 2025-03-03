// SPDX-License-Identifier: GPL-3.0-or-later

// Based on https://github.com/sn4k3/UVtools/blob/master/UVtools.Core/FileFormats/PhotonWorkshopFile.cs

use core::mem::MaybeUninit;
use crate::drivers::lcd::Color;
use crate::util::io::{Seek, BufReader, ReadPartial};

use crate::consts::io::*;

use embassy::blocking_mutex::raw::NoopRawMutex;
use embassy::channel::mpsc::{self, Channel, Receiver, Sender};

use alloc::vec::Vec;

impl Layer {
    pub async fn for_each_pixel<'a, R: ReadPartial + Seek>(
        &'a self, reader: &'a mut R, mut f: impl FnMut(Color, u16),
    ) -> Result<(), R::Error> {
        reader.seek_from_start(self.data_address);
        let mut buf_reader = BufReader::new(reader, self.data_length as usize);

        let mut buffer: [MaybeUninit::<u8>; FILE_READER_BUFFER_SIZE] = MaybeUninit::uninit_array();
        let mut color_repeat: Option<(u8, u8)> = None;

        while let Some(data) = buf_reader.next(&mut buffer).await? {
            for b in data {
                if let Some((color, repeat)) = color_repeat.take() {
                    let repeat = ((repeat as u16) << 8) | *b as u16;
                    f(color, repeat);
                } else {
                    let color = b >> 4;
                    let repeat = b & 0x0F;
                    if color == 0 || color == 0xF {
                        color_repeat = Some((color, repeat));
                        continue;
                    } else {
                        f(color, repeat as u16);
                    }
               }
            }
        }

        Ok(())
    }
}


#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub struct Header {
    pub magic: [u8; 12], // 'ANYCUBIC'
    pub version: u32,
    pub area_num: u32,
    pub config1_offset: u32,
    pub unknown: u32,
    pub preview_offset: u32,
    pub preview_end_offset: u32,
    pub layer_definition_offset: u32,
    pub config2_offset: u32,
    pub machine_offset: u32,
    pub layer_image_offset: u32,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub struct Config1 {
    pub magic: [u8; 12], // 'HEADER'
    pub length: u32,
    pub pixel_size_um: f32,
    pub layer_height: f32,
    pub exposure_time: f32,
    pub wait_time_before_cure: f32,
    pub bottom_exposure_time: f32,
    pub bottom_layers_count: f32,
    pub lift_height: f32,
    pub lift_speed: f32,
    pub retract_speed: f32,
    pub volume_ml: f32,
    pub anti_aliasing: u32,
    pub resolution_x: u32,
    pub resolution_y: u32,
    pub weight_g: f32,
    pub price: f32,
    pub price_currency: u32,
    pub per_layer_override: u32,
    pub print_time: u32,
    pub transition_layer_count: u32,
    pub padding: u32,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub struct Preview {
    pub magic: [u8; 12], // 'PREVIEW'
    pub length: u32,
    pub resolution_x: u32,
    pub dpi: u32,
    pub resolution_y: u32,
    // Body follows immediately with `resolution_x * resolution_y * 2` bytes
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub struct LayerDefinition {
    pub magic: [u8; 12], // 'LAYERDEF'
    pub length: u32,
    pub layer_count: u32,
    // Immediately followed by layer_count * Layer
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub struct Layer {
    pub data_address: u32,
    pub data_length: u32,
    pub lift_height: f32,
    pub lift_speed: f32,
    pub exposure_time: f32,
    pub layer_height: f32,
    pub non_zero_pixel_count: u32,
    pub padding: u32,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub struct Config2 {
    pub magic: [u8; 12], // 'EXTRA'
    pub unknown1: u32,
    pub unknown2: u32,
    pub bottom_lift_height1: f32,
    pub bottom_lift_speed1: f32,
    pub bottom_retract_speed1: f32,
    pub bottom_lift_height2: f32,
    pub bottom_lift_speed2: f32,
    pub bottom_retract_speed2: f32,
    pub unknown3: u32,
    pub lift_height1: f32,
    pub lift_speed1: f32,
    pub retract_speed1: f32,
    pub lift_height2: f32,
    pub lift_speed2: f32,
    pub retract_speed2: f32,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub struct Machine {
    pub magic: [u8; 12], // 'MACHINE'
    pub lenght: u32,
    pub name: [u8; 96],
    pub layer_image_format: [u8; 24],
    pub display_width: f32,
    pub display_height: f32,
    pub z_length_mm: f32,
    pub version1: u32,
    pub version2: u32,
}
