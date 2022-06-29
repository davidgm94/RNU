const kernel = @import("root");
const common = @import("common.zig");
pub const DMA = @import("drivers/dma.zig");

pub const Disk = @import("drivers/disk.zig");
pub const Filesystem = @import("drivers/filesystem.zig");
pub const RNUFS = @import("drivers/rnu_fs.zig");
pub const graphics = @import("drivers/graphics.zig");
pub const NVMe = @import("drivers/nvme.zig");
pub const PCI = @import("drivers/pci.zig");

pub fn Driver(comptime Generic: type, comptime Specific: type) type {
    // TODO: improve safety
    const child_fields = common.fields(Specific);
    common.comptime_assert(child_fields.len > 0);
    const first_field = child_fields[0];
    common.comptime_assert(first_field.field_type == Generic);

    return struct {
        const log = common.log.scoped(.DriverInitialization);
        const Initialization = Specific.Initialization;

        pub fn init(allocator: common.Allocator, context: Initialization.Context) Initialization.Error!void {
            const driver = Initialization.callback(allocator, context) catch |err| {
                log.debug("An error ocurred initializating driver {}: {}", .{ Specific, err });
                return err;
            };

            Generic.drivers.append(allocator, @ptrCast(*Generic, driver)) catch return Initialization.Error.allocation_failure;
        }
    };
}

pub const AllocationCallback = fn (size: u64) ?u64;

pub fn init() !void {
    const log = common.log.scoped(.drivers);
    const allocator = kernel.core_heap.allocator;
    try kernel.arch.init_block_drivers(allocator);
    log.debug("Initialized block drivers", .{});

    try kernel.arch.init_graphics_drivers(allocator);
    log.debug("Initialized graphics drivers", .{});
}
