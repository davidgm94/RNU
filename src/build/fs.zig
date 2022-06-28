const std = @import("std");
const drivers = @import("../drivers.zig");
const common = @import("../common.zig");
const FS = common.FS;
const assert = common.assert;
const Disk = drivers.Disk;
const FSBuilder = @This();

const log = std.log_scoped(.build_fs);

disk: Disk,
buffer: []u8,

pub const Initialization = struct {
    pub const Context = struct {
        size: u64,
        alignment: u64,
        sector_size: u64,
    };
    pub const Error = error{
        allocation_failure,
    };

    pub fn callback(allocator: common.Allocator, initialization: Initialization.Context) Error!*FSBuilder {
        var builder = allocator.create(FSBuilder) catch return Error.allocation_failure;
        if (true) @panic("truly initialize this");
        builder.* = FSBuilder{
            .disk = Disk{
                .sector_size = initialization.sector_size,
                .access = undefined, //access: fn (driver: *Driver, buffer: *DMA.Buffer, disk_work: Work) u64,
                .get_dma_buffer = undefined, //get_dma_buffer: fn (driver: *Driver, sector_count: u64) DMA.Buffer,
                .type = .virtual,
            },
            .buffer = allocator.allocBytes(@intCast(u29, initialization.alignment), initialization.size, 0, 0) catch return Error.allocation_failure,
        };
        std.mem.set(u8, builder.buffer, 0);
        return builder;
    }
};

//pub const MemoryDisk = struct {
//bytes: []u8,
//};

//fn clone_and_null_terminate(comptime FixedByteArray: type, str: []const u8) FixedByteArray {
//var result = std.mem.zeroes(FixedByteArray);
//assert(str.len < result.len);
//std.mem.copy(u8, result[0..str.len], str);
//// Maybe this could be avoided?
//result[str.len] = 0;
//return result;
//}

//pub fn add_file(disk: MemoryDisk, name: []const u8, content: []const u8) void {
//var it = disk.bytes;
//_ = name;
//_ = content;
//if (true) @panic("revisit this");
//while (it[0] != 0) : (it = it[FS.sector_size..]) {
//unreachable;
//}

//var node = @ptrCast(*FS.Node, @alignCast(@alignOf(FS.Node), it.ptr));
//node.* = FS.Node{
//.size = content.len,
//.type = .file,
//.name = clone_and_null_terminate(@TypeOf(node.name), name),
//.parent = std.mem.zeroes([100]u8),
//.last_modification = 0,
//};

//const left = it[FS.sector_size..];
//assert(left.len > content.len);
//std.mem.copy(u8, left, content);
//}

//pub fn read_debug(disk: MemoryDisk) void {
//var node = @ptrCast(*FS.Node, @alignCast(@alignOf(FS.Node), disk.bytes.ptr));
//log.debug("Node size: {}. Node name: {s}", .{ node.size, node.name });
//log.debug("First bytes:", .{});
//for (disk.bytes[FS.sector_size .. FS.sector_size + 0x20]) |byte, i| {
//log.debug("[{}]: 0x{x}", .{ i, byte });
//}
//}
