pub const cpu_clock_hz: f64 = 33_868_800.0;

pub const gpu_cycles_hblank_start_ntsc: u32 = 2560;
pub const gpu_cycles_hblank_end_ntsc: u32 = 3413;
pub const gpu_scans_vblank_start_ntsc: u32 = 240;
pub const gpu_scans_vblank_end_ntsc: u32 = 263;
pub const gpu_cycles_per_cpu_cycle = (11.0 / 7.0);

pub const gpu_target_frame_time_ntsc: f64 =
    (@as(f64, @floatFromInt(gpu_cycles_hblank_end_ntsc)) *
        @as(f64, @floatFromInt(gpu_scans_vblank_end_ntsc)) /
        gpu_cycles_per_cpu_cycle) / cpu_clock_hz;
