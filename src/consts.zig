pub const cpu_clock_hz: f64 = 33_868_800.0;

pub const gpu_cycles_hblank_start_ntsc: u32 = 2560;
pub const gpu_cycles_hblank_end_ntsc: u32 = 3413;
pub const gpu_scans_vblank_start_ntsc: u32 = 240;
pub const gpu_scans_vblank_end_ntsc: u32 = 263;

pub const gpu_cycles_hblank_start_pal: u32 = 2560;
pub const gpu_cycles_hblank_end_pal: u32 = 3406;
pub const gpu_scans_vblank_start_pal: u32 = 288;
pub const gpu_scans_vblank_end_pal: u32 = 314;

pub const gpu_cycles_per_cpu_cycle = (11.0 / 7.0);

pub fn gpuFrameTime(cycles_per_scanline: u32, scanlines_per_frame: u32) f64 {
    return (@as(f64, @floatFromInt(cycles_per_scanline)) *
        @as(f64, @floatFromInt(scanlines_per_frame)) /
        gpu_cycles_per_cpu_cycle) / cpu_clock_hz;
}

pub const gpu_target_frame_time_ntsc: f64 = gpuFrameTime(gpu_cycles_hblank_end_ntsc, gpu_scans_vblank_end_ntsc);
pub const gpu_target_frame_time_pal: f64 = gpuFrameTime(gpu_cycles_hblank_end_pal, gpu_scans_vblank_end_pal);
