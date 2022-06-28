const kernel = @import("root");
const common = @import("common");
const virtio = @import("virtio.zig");
const SplitQueue = virtio.SplitQueue;
const MMIO = virtio.MMIO;
const Descriptor = virtio.Descriptor;
const sector_size = kernel.arch.sector_size;
const Disk = kernel.Disk;
const GenericDriver = kernel.driver;

const Driver = @This();

disk: Disk,
queue: *volatile SplitQueue,
mmio: *volatile MMIO,
batch_read_byte_count: u64,

const log = common.log.scoped(.VirtioBlock);
pub const Initialization = struct {
    pub const Context = u64;
    pub const Error = error{
        allocation_failure,
    };

    pub fn callback(allocate: GenericDriver.AllocationCallback, mmio_address: u64) Error!*Driver {
        const driver_allocation = allocate(@sizeOf(Driver)) orelse return Error.allocation_failure;
        const driver = @intToPtr(*Driver, driver_allocation);
        kernel.arch.Virtual.map(mmio_address, 1);
        driver.mmio = @intToPtr(*volatile MMIO, mmio_address);
        driver.mmio.init(BlockFeature);
        driver.queue = driver.mmio.add_queue_to_device(0);
        driver.disk.read_callback = read_callback;

        // TODO: stop hardcoding interrupt number
        const interrupt = kernel.arch.Interrupts.Interrupt{
            .handler = handler,
            .pending_operations_handler = foo,
        };
        interrupt.register(8);
        driver.mmio.set_driver_initialized();

        log.debug("Block driver initialized", .{});

        return driver;
    }
};

fn foo() void {
    @panic("reached here");
}

const BlockFeature = enum(u6) {
    size_max = 1,
    seg_max = 2,
    geometry = 4,
    read_only = 5,
    blk_size = 6,
    flush = 9,
    topology = 10,
    config_wce = 11,
    discard = 13,
    write_zeroes = 14,
};

const BlockType = enum(u32) {
    in = 0,
    out = 1,
    flush = 4,
    discard = 11,
    write_zeroes = 13,
};

const Request = struct {
    const Header = struct {
        block_type: BlockType,
        reserved: u32,
        sector: u64,
    };
};

const Operation = enum {
    read,
    write,
};

/// The sector buffer address needs to be physical and have at least 512 (sector_size) bytes available
pub fn operate(driver: *Driver, comptime operation: Operation, sector_index: u64, sector_buffer_physical_address: u64) void {
    const status_size = 1;
    const header_size = @sizeOf(Request.Header);

    const status_buffer = kernel.heap.allocate(status_size, true, true) orelse @panic("status buffer unable to be allocated");
    const header_buffer = kernel.heap.allocate(header_size, true, true) orelse @panic("header buffer unable to be allocated");
    // TODO: Here we should distinguish between virtual and physical addresses
    const header = @intToPtr(*Request.Header, header_buffer.virtual);
    header.block_type = switch (operation) {
        .read => BlockType.in,
        .write => BlockType.out,
    };
    header.sector = sector_index;

    var descriptor1: u16 = 0;
    var descriptor2: u16 = 0;
    var descriptor3: u16 = 0;

    driver.queue.push_descriptor(&descriptor3).* = Descriptor{
        .address = status_buffer.physical,
        .flags = @enumToInt(Descriptor.Flag.write_only),
        .length = 1,
        .next = 0,
    };

    driver.queue.push_descriptor(&descriptor2).* = Descriptor{
        .address = sector_buffer_physical_address,
        .flags = @enumToInt(Descriptor.Flag.next) | if (operation == Operation.read) @enumToInt(Descriptor.Flag.write_only) else 0,
        .length = sector_size,
        .next = descriptor3,
    };

    driver.queue.push_descriptor(&descriptor1).* = Descriptor{
        .address = header_buffer.physical,
        .flags = @enumToInt(Descriptor.Flag.next),
        .length = @sizeOf(Request.Header),
        .next = descriptor2,
    };

    driver.queue.push_available(descriptor1);
    driver.mmio.notify_queue();
}

pub fn handler() u64 {
    kernel.assert(@src(), kernel.Disk.drivers.len > 0);
    // TODO: can use more than one driver:
    const driver = @ptrCast(*Driver, kernel.Disk.drivers[0]);
    const descriptor = driver.queue.pop_used() orelse @panic("virtio block descriptor corrupted");
    // TODO Get virtual of this physical address @Virtual @Physical
    const header = @intToPtr(*volatile Request.Header, kernel.arch.Virtual.AddressSpace.physical_to_virtual(descriptor.address));
    const operation: Operation = switch (header.block_type) {
        .in => .read,
        .out => .write,
        else => unreachable,
    };
    _ = operation;
    const sector_descriptor = driver.queue.get_descriptor(descriptor.next) orelse @panic("unable to get descriptor");

    const status_descriptor = driver.queue.get_descriptor(sector_descriptor.next) orelse @panic("unable to get descriptor");
    const status = @intToPtr([*]u8, kernel.arch.Virtual.AddressSpace.physical_to_virtual(status_descriptor.address))[0];
    //log.debug("Disk operation status: {}", .{status});
    if (status != 0) kernel.crash("Disk operation failed: {}", .{status});

    driver.batch_read_byte_count += sector_size;

    return 0;
}

pub fn read_callback(disk_driver: *Disk, buffer: []u8, start_sector: u64, sector_count: u64) u64 {
    const driver = @ptrCast(*Driver, disk_driver);
    log.debug("Asked {} sectors from sector {}", .{ sector_count, start_sector });
    const total_size = sector_count * sector_size;
    kernel.assert(@src(), buffer.len >= total_size);
    var bytes_asked: u64 = 0;
    var sector_i: u64 = start_sector;
    while (sector_i < sector_count + start_sector) : ({
        sector_i += 1;
    }) {
        const sector_physical = kernel.arch.Virtual.AddressSpace.virtual_to_physical(@ptrToInt(&buffer[bytes_asked]));
        log.debug("Sending request for sector {}", .{sector_i});
        driver.operate(.read, sector_i, sector_physical);
        bytes_asked += sector_size;
        while (driver.batch_read_byte_count != bytes_asked) {
            //kernel.spinloop_hint();
        }
    }

    kernel.assert(@src(), bytes_asked == driver.batch_read_byte_count);

    const read_bytes = driver.batch_read_byte_count;
    driver.batch_read_byte_count = 0;
    log.debug("Block device read {} bytes. Asked sector count: {}", .{ read_bytes, sector_count });
    kernel.assert(@src(), sector_count * sector_size == read_bytes);

    return sector_count;
}
