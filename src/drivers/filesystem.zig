const kernel = @import("root");
const Drivers = @import("../drivers.zig");
const common = @import("../common.zig");
const TODO = kernel.TODO;
const log = common.log.scoped(.FS);

const Driver = @This();

const Type = enum(u32) {
    RNU = 0,
    ext2 = 1,
};

type: Type,
allocator: common.Allocator,
disk: *Drivers.Disk,
/// At the moment, the memory returned by the filesystem driver is constant
read_file_callback: fn read(driver: *Driver, name: []const u8) []const u8,

pub const InitializationError = error{
    allocation_failure,
};

//pub fn init(comptime SpecificDriver: type, comptime InitializationContext: type, comptime init_callback: fn (driver: *SpecificDriver, context: InitializationContext) InitializationError!void, context: InitializationContext) InitializationError!void {
//const driver_allocation = kernel.heap.allocate(@sizeOf(SpecificDriver), true, true) orelse return InitializationError.allocation_failure;
//const driver = @intToPtr(*SpecificDriver, driver_allocation.virtual);
//init_callback(driver, context) catch |err| return err;

//// Register the driver
////if (drivers.len == 0) {
////drivers.ptr = &_drivers_array;
////}
////const index = drivers.len;
////drivers.len += 1;
////drivers[index] = @ptrCast(*Driver, driver);
//TODO(@src());
//}

pub var drivers: common.ArrayList(*Driver) = undefined;
