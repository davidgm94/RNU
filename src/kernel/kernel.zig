const std = @import("std");
const builtin = @import("builtin");

const kernel = @This();

pub const arch = @import("arch.zig");
pub const log = std.log;
pub const build_mode = builtin.mode;
pub const Physical = @import("physical.zig");
pub const Virtual = @import("virtual.zig");
pub usingnamespace @import("assertion.zig");
pub usingnamespace @import("data_manipulation.zig");
const panic_file = @import("panic.zig");
pub const panic = panic_file.panic;
pub const TODO = panic_file.TODO;
pub const SourceLocation = panic_file.SourceLocation;
pub const bounds = arch.Bounds;
pub const Spinlock = arch.Spinlock;
pub const AVL = @import("avl.zig");
pub const Heap = @import("heap.zig");
pub const CoreHeap = @import("core_heap.zig");
pub const PSF1 = @import("psf1.zig");
pub const graphics = @import("graphics.zig");
pub const scheduler = @import("scheduler.zig");
pub const Filesystem = @import("filesystem.zig");
pub const Disk = @import("disk.zig");
// TODO: move this to drivers
pub const RNUFS = @import("rnu_fs.zig");
pub const driver = @import("driver.zig");
pub const Driver = driver.Driver;
pub const ELF = @import("elf.zig");
pub const Syscall = @import("syscall.zig");
pub const DMA = @import("../drivers/dma.zig");
comptime {
    kernel.reference_all_declarations(Syscall);
}

pub var address_space = Virtual.AddressSpace.from_context(undefined);
pub var memory_region = Virtual.Memory.Region.new(Virtual.Address.new(0xFFFF900000000000), 0xFFFFF00000000000 - 0xFFFF900000000000);
pub const core_memory_region = Virtual.Memory.Region.new(Virtual.Address.new(0xFFFF800100000000), 0xFFFF800200000000 - 0xFFFF800100000000);

pub var heap: Heap = undefined;
pub var core_heap: CoreHeap = undefined;
pub var font: PSF1.Font = undefined;
pub const Writer = std.io.Writer;
pub var higher_half_direct_map: Virtual.Address = undefined;
pub var file: File = undefined;
pub var sections_in_memory: []Virtual.Memory.RegionWithPermissions = undefined;

pub const File = struct {
    address: Virtual.Address,
    size: u64,
};

pub var cpus: []arch.CPU = undefined;

pub const PrivilegeLevel = enum(u1) {
    kernel = 0,
    user = 1,
};
