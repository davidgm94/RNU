const kernel = @import("root");
const common = kernel.common;
const virtio = @import("virtio.zig");
const MMIO = virtio.MMIO;
const SplitQueue = virtio.SplitQueue;
const Descriptor = virtio.Descriptor;

const Graphics = kernel.graphics;
const log = common.log.scoped(.VirtioGPU);
const TODO = kernel.TODO;

const GenericDriver = kernel.driver;

const Driver = @This();

graphics: Graphics,
// TODO: organize this mess
mmio: *volatile MMIO,
control_queue: *volatile SplitQueue,
cursor_queue: *volatile SplitQueue,
pmode: ResponseDisplayInfo.Display,
request_counters: [11]u64,
framebuffer_id: u32,
pending_display_info_request: bool,

pub const Initialization = struct {
    pub const Context = u64;
    pub const Error = error{
        allocation_failure,
    };

    pub fn callback(allocate: GenericDriver.AllocationCallback, mmio_address: Context) Error!*Driver {
        kernel.arch.Virtual.map(mmio_address, 1);
        const driver_allocation = allocate(@sizeOf(Driver)) orelse return Error.allocation_failure;
        const driver = @intToPtr(*Driver, driver_allocation);
        driver.graphics.type = .virtio;
        // Have to manually set the initialization driver here to get it from the interrupt
        initialization_driver = driver;
        driver.mmio = @intToPtr(*volatile MMIO, mmio_address);
        driver.mmio.init(GPUFeature);

        driver.control_queue = driver.mmio.add_queue_to_device(0);
        driver.cursor_queue = driver.mmio.add_queue_to_device(1);

        // TODO: stop hardcoding interrupt number
        const interrupt = kernel.arch.Interrupts.Interrupt{
            .handler = handler,
            .pending_operations_handler = pending_operations_handler,
        };
        interrupt.register(7);
        driver.mmio.set_driver_initialized();

        driver.pending_display_info_request = true;
        driver.request_display_info();

        driver.resize_display();

        //const framebuffer = @intToPtr([*]u32, framebuffer_allocation.virtual)[0..framebuffer_pixel_count];
        //for (framebuffer) |*pixel| {
        //pixel.* = 0xffffffff;
        //}

        log.debug("GPU driver initialized", .{});

        return driver;
    }
};

fn resize_display(driver: *Driver) void {
    driver.create_resource_2d();
    driver.attach_backing();
    driver.set_scanout();

    for (driver.graphics.framebuffer.buffer[0 .. driver.graphics.framebuffer.width * driver.graphics.framebuffer.height]) |*p| {
        p.* = 0xffffffff;
    }
    driver.send_and_flush_framebuffer();
}

const AttachBackingDescriptor = struct {
    attach: ResourceAttachBacking,
    entry: MemoryEntry,
};

fn attach_backing(driver: *Driver) void {
    const framebuffer_pixel_count = driver.pmode.rect.width * driver.pmode.rect.height;
    const framebuffer_size = @sizeOf(u32) * framebuffer_pixel_count;
    const framebuffer_allocation = kernel.core_heap.allocate(framebuffer_size, true, true) orelse @panic("unable to allocate framebuffer");

    driver.graphics.framebuffer = Graphics.Framebuffer{
        .buffer = @intToPtr([*]u32, framebuffer_allocation.virtual),
        .width = driver.pmode.rect.width,
        .height = driver.pmode.rect.height,
        .cursor = Graphics.Point{ .x = 0, .y = 0 },
    };

    log.debug("New framebuffer address: 0x{x}", .{@ptrToInt(driver.graphics.framebuffer.buffer)});

    const backing_allocation = kernel.core_heap.allocate(@sizeOf(AttachBackingDescriptor), true, true) orelse @panic("unable to allocate backing");
    const backing_descriptor = @intToPtr(*volatile AttachBackingDescriptor, backing_allocation.virtual);
    backing_descriptor.* = AttachBackingDescriptor{ .attach = ResourceAttachBacking{
        .header = ControlHeader{
            .type = ControlType.cmd_resource_attach_backing,
            .flags = 0,
            .fence_id = 0,
            .context_id = 0,
            .padding = 0,
        },
        .resource_id = driver.framebuffer_id,
        .entry_count = 1,
    }, .entry = MemoryEntry{
        .address = framebuffer_allocation.physical,
        .length = framebuffer_size,
        .padding = 0,
    } };

    // TODO: fix double allocation
    driver.send_request_and_wait(backing_descriptor, null);
}

fn set_scanout(driver: *Driver) void {
    var set_scanout_descriptor = common.zeroes(SetScanout);
    set_scanout_descriptor.header.type = ControlType.cmd_set_scanout;
    set_scanout_descriptor.rect = driver.pmode.rect;
    set_scanout_descriptor.resource_id = driver.framebuffer_id;

    driver.send_request_and_wait(set_scanout_descriptor, null);
}

fn create_resource_2d(driver: *Driver) void {
    var create = common.zeroes(ResourceCreate2D);
    create.header.type = ControlType.cmd_resource_create_2d;
    create.format = Format.R8G8B8A8_UNORM;
    driver.framebuffer_id +%= 1;
    create.resource_id = driver.framebuffer_id;
    log.debug("Resource id: {}", .{create.resource_id});
    create.width = driver.pmode.rect.width;
    create.height = driver.pmode.rect.height;

    driver.send_request_and_wait(create, null);
}

fn pending_operations_handler() void {
    const driver = if (Graphics.drivers.len > 0) @ptrCast(*Driver, Graphics.drivers[0]) else initialization_driver;
    var device_status = driver.mmio.device_status;
    //log.debug("Device status: {}", .{device_status});
    if (device_status.contains(.failed) or device_status.contains(.device_needs_reset)) {
        kernel.crash("Unrecoverable device status: {}", .{device_status});
    }
    //const interrupt_status = driver.mmio.interrupt_status;
    //log.debug("Interrupt status: {}", .{interrupt_status});
    const old = driver.pmode;
    common.runtime_assert(@src(), old.rect.width == driver.graphics.framebuffer.width);
    common.runtime_assert(@src(), old.rect.height == driver.graphics.framebuffer.height);
    driver.request_display_info();
    const new = driver.pmode;
    log.debug("Old: {}, {}. New: {}, {}", .{ old.rect.width, old.rect.height, new.rect.width, new.rect.height });

    if (old.rect.width != new.rect.width or old.rect.height != new.rect.height) {
        driver.resize_display();
    }
}

fn request_display_info(driver: *Driver) void {
    common.runtime_assert(@src(), driver.pending_display_info_request);
    var header = common.zeroes(ControlHeader);
    header.type = ControlType.cmd_get_display_info;

    driver.send_request_and_wait(header, ResponseDisplayInfo);

    driver.pending_display_info_request = false;
}

fn transfer_to_host(driver: *Driver) void {
    var transfer_to_host_descriptor = common.zeroes(TransferControlToHost2D);
    transfer_to_host_descriptor.header.type = ControlType.cmd_transfer_to_host_2d;
    transfer_to_host_descriptor.rect = driver.pmode.rect;
    transfer_to_host_descriptor.resource_id = driver.framebuffer_id;

    driver.send_request_and_wait(transfer_to_host_descriptor, null);
}

fn flush(driver: *Driver) void {
    var flush_operation = common.zeroes(ResourceFlush);
    flush_operation.header.type = ControlType.cmd_resource_flush;
    flush_operation.rect = driver.pmode.rect;
    flush_operation.resource_id = driver.framebuffer_id;

    driver.send_request_and_wait(flush_operation, null);

    log.debug("Flush processed successfully", .{});
}

const Configuration = struct {
    events_read: Event,
    events_clear: Event,
    scanout_count: u32,
    reserved: u32,
};

const Event = common.Bitflag(true, enum(u32) {
    display = 0,
});

const GPUFeature = enum(u6) {
    virgl_3d_mode = 0,
    edid = 1,
};

const ControlType = enum(u32) {
    // 2D
    cmd_get_display_info = 0x0100,
    cmd_resource_create_2d,
    cmd_resource_unref,
    cmd_set_scanout,
    cmd_resource_flush,
    cmd_transfer_to_host_2d,
    cmd_resource_attach_backing,
    cmd_resource_detach_backing,
    cmd_get_capset_info,
    cmd_get_capset,
    cmd_get_edid,

    // cursor
    cmd_update_cursor = 0x0300,
    cmd_move_cursor,

    // success responses
    resp_ok_nodata = 0x1100,
    resp_ok_display_info,
    resp_ok_capset_info,
    resp_ok_capset,
    resp_ok_edid,

    // error responses
    resp_err_unspec = 0x1200,
    resp_err_out_of_memory,
    resp_err_invalid_scanout_id,
    resp_err_invalid_resource_id,
    resp_err_invalid_context_id,
    resp_err_invalid_parameter,

    fn get_request_counter_index(control_type: ControlType) u64 {
        common.runtime_assert(@src(), @enumToInt(control_type) < @enumToInt(ControlType.cmd_update_cursor));
        return @enumToInt(control_type) - @enumToInt(ControlType.cmd_get_display_info);
    }
};

const Flag = enum(u32) {
    fence = 1 << 0,
};

const ControlHeader = struct {
    type: ControlType,
    flags: u32,
    fence_id: u64,
    context_id: u32,
    padding: u32,
};

const max_scanouts = 16;

const Rect = struct {
    x: u32,
    y: u32,
    width: u32,
    height: u32,
};

const ResponseDisplayInfo = struct {
    header: ControlHeader,
    pmodes: [max_scanouts]Display,

    const Display = struct {
        rect: Rect,
        enabled: u32,
        flags: u32,
    };
};

const Format = enum(u32) {
    B8G8R8A8_UNORM = 1,
    B8G8R8X8_UNORM = 2,
    A8R8G8B8_UNORM = 3,
    X8R8G8B8_UNORM = 4,

    R8G8B8A8_UNORM = 67,
    X8B8G8R8_UNORM = 68,

    A8B8G8R8_UNORM = 121,
    R8G8B8X8_UNORM = 134,
};

const ResourceCreate2D = struct {
    header: ControlHeader,
    resource_id: u32,
    format: Format,
    width: u32,
    height: u32,
};

const MemoryEntry = struct {
    address: u64,
    length: u32,
    padding: u32,
};

const ResourceAttachBacking = struct {
    header: ControlHeader,
    resource_id: u32,
    entry_count: u32,
    // TODO: variable-length array afterwards

    inline fn set_entry(self: *volatile @This(), index: u64, entry: MemoryEntry) void {
        const entry_ptr = @intToPtr(*MemoryEntry, @ptrToInt(self) + @sizeOf(@This()) + (@sizeOf(MemoryEntry) * index));
        entry_ptr.* = entry;
    }
};

const SetScanout = struct {
    header: ControlHeader,
    rect: Rect,
    scanout_id: u32,
    resource_id: u32,
};

const TransferControlToHost2D = struct {
    header: ControlHeader,
    rect: Rect,
    offset: u64,
    resource_id: u32,
    padding: u32,
};

const ResourceFlush = struct {
    header: ControlHeader,
    rect: Rect,
    resource_id: u32,
    padding: u32,
};

pub fn send_and_flush_framebuffer(driver: *Driver) void {
    driver.transfer_to_host();
    driver.flush();
}

pub fn operate(driver: *Driver, request_bytes: []const u8, response_size: u32) void {
    const request = kernel.core_heap.allocate(request_bytes.len, true, true) orelse @panic("unable to allocate memory for gpu request");
    common.copy(u8, @intToPtr([*]u8, request.virtual)[0..request_bytes.len], request_bytes);

    var descriptor1: u16 = 0;
    var descriptor2: u16 = 0;

    driver.control_queue.push_descriptor(&descriptor2).* = Descriptor{
        .address = (kernel.core_heap.allocate(response_size, true, true) orelse @panic("unable to get memory for gpu response")).physical,
        .flags = @enumToInt(Descriptor.Flag.write_only),
        .length = response_size,
        .next = 0,
    };

    driver.control_queue.push_descriptor(&descriptor1).* = Descriptor{
        .address = request.physical,
        .flags = @enumToInt(Descriptor.Flag.next),
        .length = @intCast(u32, request_bytes.len),
        .next = descriptor2,
    };

    driver.control_queue.push_available(descriptor1);
    driver.mmio.notify_queue();
}

fn handler() u64 {
    // TODO: use more than one driver
    const driver = if (Graphics.drivers.len > 0) @ptrCast(*Driver, Graphics.drivers[0]) else initialization_driver;
    var device_status = driver.mmio.device_status;
    log.debug("Device status: {}", .{device_status});
    if (device_status.contains(.failed) or device_status.contains(.device_needs_reset)) {
        kernel.crash("Unrecoverable device status: {}", .{device_status});
    }
    const interrupt_status = driver.mmio.interrupt_status;
    log.debug("Interrupt status: {}", .{interrupt_status});

    var operations_pending: u64 = 0;

    if (interrupt_status.contains(.configuration_change)) {
        const configuration = @intToPtr(*volatile Configuration, @ptrToInt(driver.mmio) + MMIO.configuration_offset);
        const events_read = configuration.events_read;
        if (events_read.contains(.display)) {
            // TODO: check if all events are handled to write the proper bitmask
            configuration.events_clear = events_read;

            operations_pending += 1;
            driver.pending_display_info_request = true;
        } else {
            @panic("corrupted notification");
        }
    } else {
        const descriptor = driver.control_queue.pop_used() orelse {
            if (device_status.contains(.failed) or device_status.contains(.device_needs_reset)) {
                kernel.crash("Unrecoverable device status: {}", .{device_status});
            }
            kernel.crash("virtio GPU descriptor corrupted", .{});
        };
        const header = @intToPtr(*volatile ControlHeader, kernel.arch.Virtual.AddressSpace.physical_to_virtual(descriptor.address));
        const request_descriptor = driver.control_queue.get_descriptor(descriptor.next) orelse @panic("unable to request descriptor");

        handle_ex(driver, header, request_descriptor);
    }

    // TODO: check if all events are handled to write the proper bitmask
    driver.mmio.interrupt_ack = interrupt_status.bits;

    return operations_pending;
}

fn handle_ex(driver: *Driver, header: *volatile ControlHeader, request_descriptor: *volatile Descriptor) void {
    const control_header = @intToPtr(*ControlHeader, kernel.arch.Virtual.AddressSpace.physical_to_virtual(request_descriptor.address));

    switch (header.type) {
        .cmd_get_display_info => {
            if (control_header.type != ControlType.resp_ok_display_info) {
                kernel.crash("Unable to process {s} request successfully: {s}", .{ @tagName(header.type), @tagName(control_header.type) });
            }
            const display_info = @ptrCast(*ResponseDisplayInfo, control_header).*;
            log.debug("Display info changed", .{});
            for (display_info.pmodes) |pmode_it, i| {
                if (pmode_it.enabled != 0) log.debug("[{}] pmode: {}", .{ i, pmode_it });
            }
            driver.pmode = display_info.pmodes[0];
        },
        else => {
            if (control_header.type != ControlType.resp_ok_nodata) {
                kernel.crash("Unable to process {s} request successfully: {s}", .{ @tagName(header.type), @tagName(control_header.type) });
            }
        },
    }

    const request_counter_index = header.type.get_request_counter_index();
    driver.request_counters[request_counter_index] +%= 1;
}

fn send_request_and_wait(driver: *Driver, request_descriptor: anytype, comptime ResponseType: ?type) void {
    var request_bytes: []const u8 = undefined;
    var control_header_type: ControlType = undefined;

    switch (@typeInfo(@TypeOf(request_descriptor))) {
        .Pointer => {
            control_header_type = @ptrCast(*const ControlHeader, request_descriptor).type;
            request_bytes = kernel.as_bytes(request_descriptor);
        },
        else => {
            control_header_type = @ptrCast(*const ControlHeader, &request_descriptor).type;
            request_bytes = kernel.as_bytes(&request_descriptor);
        },
    }

    const response_size = if (ResponseType) |RT| @sizeOf(RT) else @sizeOf(ControlHeader);
    log.debug("Sending {s}, Request size: {}. Response size: {}", .{ @tagName(control_header_type), request_bytes.len, response_size });
    const request_counter_index = control_header_type.get_request_counter_index();
    const request_counter = driver.request_counters[request_counter_index] +% 1;
    driver.operate(request_bytes, response_size);

    while (driver.request_counters[request_counter_index] != request_counter) {
        //kernel.spinloop_hint();
    }

    log.debug("{s} #{} processed successfully", .{ @tagName(control_header_type), request_counter });
}

var initialization_driver: *Driver = undefined;
