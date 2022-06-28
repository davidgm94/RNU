const Drivers = @import("../drivers.zig");
const common = @import("../common.zig");
const DMA = Drivers.DMA;
const Driver = @This();

pub const Type = enum(u32) {
    nvme = 0,
    virtio = 1,
    virtual = 2,
};

sector_size: u64,
access: fn (driver: *Driver, buffer: *DMA.Buffer, disk_work: Work) u64,
get_dma_buffer: fn (driver: *Driver, sector_count: u64) DMA.Buffer,
type: Type,

pub const Work = struct {
    sector_offset: u64,
    sector_count: u64,
    operation: Operation,
};

pub const Operation = enum {
    read,
    write,
};

pub var drivers: common.ArrayList(*Driver) = undefined;
