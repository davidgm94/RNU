const root = @import("root");
const drivers = @import("../drivers.zig");
const common = @import("../common.zig");
const std = @import("std");
const Filesystem = drivers.Filesystem;
const RNUFS = @import("../common/fs.zig");
const GenericDriver = drivers.Driver;
const Disk = drivers.Disk;
const DMA = drivers.DMA;
const log = common.log.scoped(.RNUFS);

const Driver = @This();

fs: Filesystem,

pub const Initialization = struct {
    pub const Context = *Disk;
    pub const Error = error{
        allocation_failure,
    };

    pub fn callback(allocator: std.mem.Allocator, initialization_context: Context) Filesystem.InitializationError!*Driver {
        const driver = allocator.create(Driver) catch return Error.allocation_failure;
        driver.* = Driver{
            .fs = Filesystem{
                .type = .RNU,
                .disk = initialization_context,
                .read_file_callback = read_file,
                .allocator = allocator,
            },
        };

        return driver;
    }
};

pub fn seek_file(fs_driver: *Filesystem, name: []const u8) ?SeekResult {
    log.debug("Seeking file {s}", .{name});
    const sectors_to_read_at_time = 1;
    var sector: u64 = 0;
    log.debug("Initializing search buffer", .{});
    var search_buffer = fs_driver.disk.get_dma_buffer(fs_driver.disk, sectors_to_read_at_time);
    log.debug("Done initializing search buffer", .{});
    const sector_size = fs_driver.disk.sector_size;

    while (true) {
        log.debug("FS driver asking read", .{});
        const sectors_read = fs_driver.disk.access(fs_driver.disk, &search_buffer, Disk.Work{
            .sector_offset = sector,
            .sector_count = sectors_to_read_at_time,
            .operation = .read,
        });
        log.debug("FS driver ending read", .{});
        if (sectors_read != sectors_to_read_at_time) common.panic(@src(), "Driver internal error: cannot seek file", .{});
        var node = @intToPtr(*RNUFS.Node, search_buffer.virtual_address);
        const node_name_cstr = @ptrCast([*:0]const u8, &node.name);
        const node_name = node_name_cstr[0..common.cstr_len(node_name_cstr)];
        if (node_name.len == 0) @panic("file not found: no files");

        log.debug("Node name: {s}", .{node_name});

        if (common.string_eq(node_name, name)) {
            return SeekResult{
                .sector = sector,
                .node = node.*,
            };
        }

        log.debug("Node size: {}", .{node.size});
        const sectors_to_add = 1 + common.bytes_to_sector(node.size, sector_size, .can_be_not_exact);
        log.debug("Sectors to add: {}", .{sectors_to_add});
        sector += sectors_to_add;
    }
}

pub fn read_file(fs_driver: *Filesystem, name: []const u8) []const u8 {
    log.debug("About to read a file...", .{});
    if (seek_file(fs_driver, name)) |seek_result| {
        const node_size = seek_result.node.size;
        const sector_size = fs_driver.disk.sector_size;
        const bytes_to_read = common.align_forward(node_size, sector_size);
        // TODO: maybe this should not be exact
        const sectors_to_read = common.bytes_to_sector(bytes_to_read, sector_size, .must_be_exact);
        // TODO:  @MaybeBug maybe allocate in the heap?
        var buffer = fs_driver.disk.get_dma_buffer(fs_driver.disk, sectors_to_read);
        // Add one to skip the metadata
        const sectors_read = fs_driver.disk.access(fs_driver.disk, &buffer, Disk.Work{
            .sector_offset = seek_result.sector + 1,
            .sector_count = sectors_to_read,
            .operation = .read,
        });

        if (sectors_read != sectors_to_read) common.panic(@src(), "Driver internal error: cannot read file", .{});

        return @intToPtr([*]const u8, buffer.virtual_address)[0..node_size];
    } else {
        @panic("unable to find file");
    }
}

pub const SeekResult = struct {
    sector: u64,
    node: RNUFS.Node,
};
