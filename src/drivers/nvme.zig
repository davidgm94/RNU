// This has been implemented with NVMe Specification 2.0b
const kernel = @import("root");
const common = @import("../common.zig");
const log = common.log.scoped(.NVMe);
const TODO = kernel.TODO;
const Disk = kernel.drivers.Disk;
const PCI = kernel.drivers.PCI;
const DMA = kernel.drivers.DMA;

const x86_64 = @import("../kernel/arch/x86_64.zig");

const Physical = kernel.Physical;
const Virtual = kernel.Virtual;
const PhysicalAddress = common.PhysicalAddress;
const VirtualAddress = common.VirtualAddress;

const NVMe = @This();
const Driver = NVMe;

pub var driver: *Driver = undefined;

device: *PCI.Device,
capabilities: CAP,
version: Version,
doorbell_stride: u64,
ready_transition_timeout: u64,
maximum_data_transfer_bytes: u64,
rtd3_entry_latency_us: u32,
maximum_data_outstanding_commands: u16,
model: [40]u8,

admin_submission_queue: [*]u8,
admin_completion_queue: [*]u8,
admin_completion_queue_head: u32,
admin_submission_queue_tail: u32,
admin_completion_queue_phase: bool,
admin_completion_queue_last_result: u32,
admin_completion_queue_last_status: u16,

io_submission_queue: ?[*]u8,
io_completion_queue: ?[*]u8,
io_completion_queue_head: u32,
io_submission_queue_tail: u32,
io_submission_queue_head: u32,
io_completion_queue_phase: bool,

prp_list_pages: [io_queue_entry_count]PhysicalAddress,
prp_list_virtual: VirtualAddress,

drives: []Drive,

const general_timeout = 5000;
const admin_queue_entry_count = 2;
const io_queue_entry_count = 256;
const submission_queue_entry_bytes = 64;
const completion_queue_entry_bytes = 16;
const Command = [16]u32;

const Drive = struct {
    disk: Disk,
    nsid: u32,

    pub const Initialization = struct {
        pub const Context = *NVMe.Drive;
        pub const Error = error{
            allocation_failure,
        };

        pub fn callback(allocator: common.Allocator, drive: *NVMe.Drive) Error!*Drive {
            _ = allocator;
            return drive;
        }
    };

    pub fn new(sector_size: u64, nsid: u32) Drive {
        return Drive{
            .disk = Disk{
                .type = .nvme,
                .sector_size = sector_size,
                .access = access, // access: fn (driver: *Driver, buffer: *DMA.Buffer, disk_work: Work) u64,
                .get_dma_buffer = get_dma_buffer,
            },
            .nsid = nsid,
        };
    }

    pub fn access(disk: *Disk, buffer: *DMA.Buffer, disk_work: Disk.Work) u64 {
        common.assert(@src(), buffer.completed_size == 0);
        common.assert(@src(), buffer.address.is_page_aligned());
        const drive = @fieldParentPtr(Drive, "disk", disk);
        const nvme = driver;
        log.debug("NVMe access", .{});
        log.debug("NVMe drive: {}", .{drive});
        log.debug("Buffer: (0x{x}, {})", .{ buffer.address.value, buffer.total_size });
        log.debug("Work: {}", .{disk_work});
        // TODO: @Lock
        const new_tail = (nvme.io_submission_queue_tail + 1) % io_queue_entry_count;
        const submission_queue_full = new_tail == nvme.io_submission_queue_head;

        // TODO: @Lock
        while (submission_queue_full) : ({
            // Acquire lock
        }) {
            // TODO: @Hack
            TODO(@src());
            // Release lock
        }

        const request_byte_size = disk_work.sector_count * disk.sector_size;
        const offset = buffer.address.value % kernel.arch.page_size;
        kernel.assert(@src(), offset == 0);
        log.debug("Offset: 0x{x}", .{offset});

        if (request_byte_size <= 2 * kernel.arch.page_size) {
            log.debug("Buffer address: 0x{x}", .{buffer.address.value});
            const prp1 = buffer.address.translate(&kernel.address_space) orelse TODO(@src());
            const first_prp_length = kernel.arch.page_size - offset;
            log.debug("TODO: is this correct? PRP1 length: {}", .{first_prp_length});
            var prp2 = Physical.Address.temporary_invalid();

            if (request_byte_size > first_prp_length) {
                prp2 = buffer.address.offset(first_prp_length).translate(&kernel.address_space) orelse TODO(@src());
            }

            const prps = [2]Physical.Address{ prp1, prp2 };

            var command = @ptrCast(*Command, @alignCast(@alignOf(Command), &nvme.io_submission_queue.?[nvme.io_submission_queue_tail * submission_queue_entry_bytes]));
            command[0] = (nvme.io_submission_queue_tail << 16) | @as(u32, if (disk_work.operation == .write) 0x01 else 0x02);
            // TODO:
            command[1] = drive.nsid;
            command[2] = 0;
            command[3] = 0;
            command[4] = 0;
            command[5] = 0;
            command[6] = @truncate(u32, prps[0].value);
            command[7] = @truncate(u32, prps[0].value >> 32);
            command[8] = @truncate(u32, prps[1].value);
            command[9] = @truncate(u32, prps[1].value >> 32);
            command[10] = @truncate(u32, disk_work.sector_offset);
            command[11] = @truncate(u32, disk_work.sector_offset >> 32);
            // TODO: what size is really this?
            command[12] = @intCast(u16, disk_work.sector_count);
            command[13] = 0;
            command[14] = 0;
            command[15] = 0;

            nvme.io_submission_queue_tail = new_tail;
            log.debug("Sending the command", .{});
            @fence(.SeqCst);
            nvme.write_sqtdbl(1, new_tail);
            asm volatile ("hlt");

            return disk_work.sector_count;
        } else {
            TODO(@src());
        }
    }

    pub fn get_dma_buffer(disk: *Disk, allocator: common.Allocator, sector_count: u64) common.Allocator.Error!DMA.Buffer {
        const byte_size = kernel.align_forward(sector_count * disk.sector_size, kernel.arch.page_size);
        return DMA.Buffer{
            .virtual_address = try allocator.allocBytes(@intCast(u29, kernel.arch.page_size), byte_size, 0, 0),
            .total_size = byte_size,
            .complete_size = 0,
        };
    }
    //get_dma_buffer: fn (driver: *Driver, sector_count: u64) DMA.Buffer,
};

pub const Initialization = struct {
    pub const Context = *PCI;
    pub const Error = error{
        allocation_failure,
        not_found,
    };

    pub fn callback(allocator: common.Allocator, pci: *PCI) Error!*Driver {
        const nvme_device = find(pci) orelse return Error.not_found;
        log.debug("Found controller", .{});
        driver = allocator.create(Driver) catch return Error.allocation_failure;
        driver.* = NVMe.new(nvme_device);
        const result = driver.device.enable_features(PCI.Device.Features.from_flags(&.{ .interrupts, .busmastering_dma, .memory_space_access, .bar0 }));
        kernel.assert(@src(), result);
        log.debug("Device features enabled", .{});
        driver.init(allocator);

        log.debug("Driver initialized", .{});

        return driver;
    }
};

pub fn new(device: *PCI.Device) NVMe {
    return NVMe{
        .device = device,
        .capabilities = undefined,
        .version = undefined,
        .doorbell_stride = 0,
        .ready_transition_timeout = 0,
        .maximum_data_transfer_bytes = 0,
        .rtd3_entry_latency_us = 0,
        .maximum_data_outstanding_commands = 0,
        .model = undefined,
        .admin_submission_queue = undefined,
        .admin_completion_queue = undefined,
        .admin_submission_queue_tail = 0,
        .admin_completion_queue_head = 0,
        .admin_completion_queue_phase = false,
        .admin_completion_queue_last_result = 0,
        .admin_completion_queue_last_status = 0,
        .io_submission_queue = null,
        .io_completion_queue = null,
        .io_submission_queue_tail = 0,
        .io_completion_queue_head = 0,
        .io_submission_queue_head = 0,
        .io_completion_queue_phase = false,
        .prp_list_pages = undefined,
        .prp_list_virtual = undefined,
        .drives = &.{},
    };
}

pub fn find(pci: *PCI) ?*PCI.Device {
    return pci.find_device(0x1, 0x8);
}

inline fn read(nvme: *NVMe, comptime register: Property) register.type {
    log.debug("Reading {} bytes from BAR register #{} at offset 0x{x})", .{ @sizeOf(register.type), 0, register.offset });
    return nvme.device.read_bar(register.type, 0, register.offset);
}

inline fn write(nvme: *NVMe, comptime register: Property, value: register.type) void {
    log.debug("Writing {} bytes (0x{x}) to BAR register #{} at offset 0x{x})", .{ @sizeOf(register.type), value, 0, register.offset });
    nvme.device.write_bar(register.type, 0, register.offset, value);
}

inline fn read_sqtdbl(nvme: *NVMe, index: u32) u32 {
    return nvme.device.read_bar(u32, 0, 0x1000 + nvme.doorbell_stride * (2 * index + 0));
}

inline fn read_cqhdbl(nvme: *NVMe, index: u32) u32 {
    return nvme.device.read_bar(u32, 0, 0x1000 + nvme.doorbell_stride * (2 * index + 1));
}

inline fn write_sqtdbl(nvme: *NVMe, index: u32, value: u32) void {
    nvme.device.write_bar(u32, 0, 0x1000 + nvme.doorbell_stride * (2 * index + 0), value);
}

inline fn write_cqhdbl(nvme: *NVMe, index: u32, value: u32) void {
    nvme.device.write_bar(u32, 0, 0x1000 + nvme.doorbell_stride * (2 * index + 1), value);
}

pub fn issue_admin_command(nvme: *NVMe, command: *Command, result: ?*u32) bool {
    _ = result;
    @ptrCast(*Command, @alignCast(@alignOf(Command), &nvme.admin_submission_queue[nvme.admin_submission_queue_tail * @sizeOf(Command)])).* = command.*;
    nvme.admin_submission_queue_tail = (nvme.admin_submission_queue_tail + 1) % admin_queue_entry_count;

    // TODO: reset event
    @fence(.SeqCst); // best memory barrier?
    kernel.assert(@src(), kernel.arch.are_interrupts_enabled());
    log.debug("Entering in a wait state", .{});
    nvme.write_sqtdbl(0, nvme.admin_submission_queue_tail);
    asm volatile ("hlt");
    // TODO: wait for event
    //

    if (nvme.admin_completion_queue_last_status != 0) {
        const do_not_retry = nvme.admin_completion_queue_last_status & 0x8000 != 0;
        const more = nvme.admin_completion_queue_last_status & 0x4000 != 0;
        const command_retry_delay = @truncate(u8, nvme.admin_completion_queue_last_status >> 12) & 0x03;
        const status_code_type = @truncate(u8, nvme.admin_completion_queue_last_status >> 9) & 0x07;
        const status_code = @truncate(u8, nvme.admin_completion_queue_last_status >> 1);
        _ = do_not_retry;
        _ = more;
        _ = command_retry_delay;
        _ = status_code_type;
        _ = status_code;
        log.debug("Admin command failed", .{});

        return false;
    }

    if (result) |p_result| p_result.* = nvme.admin_completion_queue_last_status;
    return true;
}

const PRPs = [2]Physical.Address;

pub fn init(nvme: *NVMe, allocator: kernel.Allocator) void {
    nvme.capabilities = nvme.read(cap);
    nvme.version = nvme.read(vs);
    log.debug("Capabilities = {}. Version = {}", .{ nvme.capabilities, nvme.version });

    kernel.assert(@src(), nvme.version.major == 1 and nvme.version.minor == 4);
    if (nvme.version.major > 1) @panic("version too new");
    if (nvme.version.major < 1) @panic("f1");
    if (nvme.version.major == 1 and nvme.version.minor < 1) @panic("f2");
    if (nvme.capabilities.mqes == 0) @panic("f3");
    kernel.assert_unsafe(@bitOffsetOf(CAP, "nssrs") == 36);
    if (!nvme.capabilities.css.nvm_command_set) @panic("f4");
    if (nvme.capabilities.mpsmin < kernel.arch.page_shifter - 12) @panic("f5");
    if (nvme.capabilities.mpsmax < kernel.arch.page_shifter - 12) @panic("f6");

    nvme.doorbell_stride = @as(u64, 4) << nvme.capabilities.doorbell_stride;
    log.debug("NVMe doorbell stride: 0x{x}", .{nvme.doorbell_stride});

    nvme.ready_transition_timeout = nvme.capabilities.timeout * @as(u64, 500);
    log.debug("NVMe ready transition timeout: 0x{x}", .{nvme.ready_transition_timeout});

    const previous_configuration = nvme.read(cc);
    log.debug("Previous configuration: 0x{x}", .{previous_configuration});

    log.debug("we are here", .{});
    if (previous_configuration.enable) {
        log.debug("the controller was enabled", .{});
        log.debug("branch", .{});
        // TODO. HACK we should use a timeout here
        // TODO: PRobably buggy
        while (!nvme.read(csts).ready) {
            log.debug("busy waiting", .{});
        }
        var config = nvme.read(cc);
        config.enable = false;
        nvme.write(cc, config);
    }

    {
        // TODO. HACK we should use a timeout here
        while (nvme.read(csts).ready) {}
        log.debug("past the timeout", .{});
    }

    nvme.write(cc, blk: {
        var cc_value = nvme.read(cc);
        cc_value.css = .nvm_command_set;
        cc_value.mps = kernel.arch.page_shifter - 12;
        cc_value.ams = .round_robin;
        cc_value.shn = .no_notification;
        cc_value.iosqes = 6;
        cc_value.iocqes = 4;
        break :blk cc_value;
    });
    nvme.write(aqa, blk: {
        var aqa_value = nvme.read(aqa);
        aqa_value.asqs = admin_queue_entry_count - 1;
        aqa_value.acqs = admin_queue_entry_count - 1;
        break :blk aqa_value;
    });

    const admin_submission_queue_size = admin_queue_entry_count * submission_queue_entry_bytes;
    const admin_completion_queue_size = admin_queue_entry_count * completion_queue_entry_bytes;
    const admin_queue_page_count = kernel.align_forward(admin_submission_queue_size, kernel.arch.page_size) + kernel.align_forward(admin_completion_queue_size, kernel.arch.page_size);
    const admin_queue_physical_address = kernel.Physical.Memory.allocate_pages(admin_queue_page_count) orelse @panic("admin queue");
    const admin_submission_queue_physical_address = admin_queue_physical_address;
    const admin_completion_queue_physical_address = admin_queue_physical_address.offset(kernel.align_forward(admin_submission_queue_size, kernel.arch.page_size));

    nvme.write(asq, ASQ{
        .reserved = 0,
        .asqb = @truncate(u52, admin_submission_queue_physical_address.value >> 12),
    });
    nvme.write(acq, ACQ{
        .reserved = 0,
        .acqb = @truncate(u52, admin_completion_queue_physical_address.value >> 12),
    });

    const admin_submission_queue_virtual_address = admin_submission_queue_physical_address.to_higher_half_virtual_address();
    const admin_completion_queue_virtual_address = admin_completion_queue_physical_address.to_higher_half_virtual_address();
    kernel.address_space.map(admin_submission_queue_physical_address, admin_submission_queue_virtual_address, kernel.Virtual.AddressSpace.Flags.from_flags(&.{.read_write}));
    kernel.address_space.map(admin_completion_queue_physical_address, admin_completion_queue_virtual_address, kernel.Virtual.AddressSpace.Flags.from_flags(&.{.read_write}));

    nvme.admin_submission_queue = admin_submission_queue_virtual_address.access([*]u8);
    nvme.admin_completion_queue = admin_completion_queue_virtual_address.access([*]u8);

    nvme.write(cc, blk: {
        var new_cc = nvme.read(cc);
        new_cc.enable = true;
        break :blk new_cc;
    });

    {
        // TODO: HACK use a timeout
        while (true) {
            const status = nvme.read(csts);
            if (status.cfs) {
                @panic("cfs");
            } else if (status.ready) {
                break;
            }
        }
    }

    if (!nvme.device.enable_single_interrupt(x86_64.interrupts.HandlerInfo.new(nvme, handle_irq))) {
        @panic("f hanlder");
    }

    nvme.write(intmc, 1 << 0);

    // TODO: @Hack remove that 3 for a proper value
    const identify_data_physical_address = kernel.Physical.Memory.allocate_pages(3) orelse @panic("identify");
    const identify_data_virtual_address = identify_data_physical_address.to_higher_half_virtual_address();
    kernel.address_space.map(identify_data_physical_address, identify_data_virtual_address, kernel.Virtual.AddressSpace.Flags.from_flag(.read_write));
    const identify_data = identify_data_virtual_address.access([*]u8);

    {
        var command = kernel.zeroes(Command);
        command[0] = 0x06;
        command[6] = @truncate(u32, identify_data_physical_address.value);
        command[7] = @truncate(u32, identify_data_physical_address.value >> 32);
        command[10] = 0x01;

        if (!nvme.issue_admin_command(&command, null)) @panic("issue identify");

        nvme.maximum_data_transfer_bytes = blk: {
            if (identify_data[77] != 0) {
                // TODO: mpsmin? shouldnt this be mpsmax?
                break :blk @as(u64, 1) << (12 + @intCast(u6, identify_data[77]) + nvme.capabilities.mpsmin);
            } else {
                break :blk 0;
            }
        };

        nvme.rtd3_entry_latency_us = @ptrCast(*u32, @alignCast(@alignOf(u32), &identify_data[88])).*;
        nvme.maximum_data_outstanding_commands = @ptrCast(*u16, @alignCast(@alignOf(u16), &identify_data[514])).*;
        kernel.copy(u8, &nvme.model, identify_data[24 .. 24 + @sizeOf(@TypeOf(nvme.model))]);
        log.debug("NVMe model: {s}", .{nvme.model});

        if (nvme.rtd3_entry_latency_us > 250 * 1000) {
            nvme.rtd3_entry_latency_us = 250 * 1000;
        }

        if (identify_data[111] > 0x01) @panic("unsupported");

        if (nvme.maximum_data_transfer_bytes == 0 or nvme.maximum_data_transfer_bytes == 2097152) {
            nvme.maximum_data_transfer_bytes = 2097152;
        }
    }

    {
        var command = kernel.zeroes(Command);
        command[0] = 0x09;
        command[10] = 0x80;
        command[11] = 0;

        _ = nvme.issue_admin_command(&command, null);
    }

    {
        const size = kernel.align_forward(io_queue_entry_count * completion_queue_entry_bytes, kernel.arch.page_size);
        const page_count = kernel.bytes_to_pages(size, .must_be_exact);
        const queue_physical_address = kernel.Physical.Memory.allocate_pages(page_count) orelse @panic("ph comp");

        const physical_region = kernel.Physical.Memory.Region.new(queue_physical_address, size);
        physical_region.map(&kernel.address_space, queue_physical_address.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.from_flag(.read_write));
        nvme.io_completion_queue = queue_physical_address.to_higher_half_virtual_address().access([*]u8);

        var command = kernel.zeroes(Command);
        command[0] = 0x05;
        command[6] = @truncate(u32, queue_physical_address.value);
        command[7] = @truncate(u32, queue_physical_address.value >> 32);
        command[10] = 1 | ((io_queue_entry_count - 1) << 16);
        command[11] = (1 << 0) | (1 << 1);

        if (!nvme.issue_admin_command(&command, null)) @panic("create queue");
    }

    {
        const size = kernel.align_forward(io_queue_entry_count * submission_queue_entry_bytes, kernel.arch.page_size);
        const page_count = kernel.bytes_to_pages(size, .must_be_exact);
        const queue_physical_address = kernel.Physical.Memory.allocate_pages(page_count) orelse @panic("ph comp");

        const physical_region = kernel.Physical.Memory.Region.new(queue_physical_address, size);
        physical_region.map(&kernel.address_space, queue_physical_address.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.from_flag(.read_write));
        nvme.io_submission_queue = queue_physical_address.to_higher_half_virtual_address().access([*]u8);

        var command = kernel.zeroes(Command);
        command[0] = 0x01;
        command[6] = @truncate(u32, queue_physical_address.value);
        command[7] = @truncate(u32, queue_physical_address.value >> 32);
        command[10] = 1 | ((io_queue_entry_count - 1) << 16);
        command[11] = (1 << 0) | (1 << 16);

        if (!nvme.issue_admin_command(&command, null)) @panic("create queue");
    }

    {
        for (nvme.prp_list_pages) |*prp_list_page| {
            prp_list_page.* = kernel.Physical.Memory.allocate_pages(1) orelse @panic("prp physical");
        }

        kernel.address_space.map(nvme.prp_list_pages[0], nvme.prp_list_pages[0].to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.from_flag(.read_write));
        nvme.prp_list_virtual = nvme.prp_list_pages[0].to_higher_half_virtual_address();
    }

    var nsid: u32 = 0;
    var drive_count: u64 = 0;
    var drives: [64]Drive = undefined;
    namespace: while (true) {
        {
            var command = kernel.zeroes(Command);
            command[0] = 0x06;
            command[1] = nsid;
            command[6] = @truncate(u32, identify_data_physical_address.value);
            command[7] = @truncate(u32, identify_data_physical_address.value >> 32);
            command[10] = 0x02;

            if (!nvme.issue_admin_command(&command, null)) @panic("identify");
        }

        var i: u64 = 0;
        while (i < 1024) : (i += 1) {
            nsid = @ptrCast(*align(1) u32, &identify_data[i]).*;
            if (nsid == 0) break :namespace;
            log.debug("nsid", .{});

            {
                var command = kernel.zeroes(Command);
                command[0] = 0x06;
                command[1] = nsid;
                command[6] = @truncate(u32, identify_data_physical_address.value + 0x1000);
                command[7] = @truncate(u32, (identify_data_physical_address.value + 0x1000) >> 32);
                command[10] = 0x00;

                if (!nvme.issue_admin_command(&command, null)) @panic("identify");
            }

            const formatted_lba_size = identify_data[0x1000 + 26];
            const lba_format = @ptrCast(*u32, @alignCast(@alignOf(u32), &identify_data[@as(u16, 0x1000) + 128 + 4 * @truncate(u4, formatted_lba_size)])).*;
            if (@truncate(u16, lba_format) != 0) continue;
            log.debug("lba_format", .{});

            const sector_bytes_exponent = @truncate(u5, lba_format >> 16);
            if (sector_bytes_exponent < 9 or sector_bytes_exponent > 16) continue;
            const sector_size = @as(u64, 1) << sector_bytes_exponent;
            const drive = &drives[drive_count];
            drive_count += 1;
            kernel.assert(@src(), drive_count < drives.len);
            drive.* = Drive.new(sector_size, nsid);
            log.debug("New drive registered: {}", .{drive});
        }
    }

    nvme.drives = allocator.alloc(Drive, drive_count) catch kernel.crash("unable to allocate for NVMe drives", .{});
    kernel.copy(Drive, nvme.drives, drives[0..drive_count]);
    for (nvme.drives) |*drive| {
        kernel.drivers.Driver(Disk, Drive).init(allocator, drive) catch kernel.crash("Failed to initialized device", .{});
    }

    kernel.assert(@src(), drive_count == 1);
}

pub const Callback = fn (nvme: *NVMe, line: u64) bool;

pub fn handle_irq(nvme: *NVMe, line: u64) bool {
    _ = line;
    var from_admin = false;
    var from_io = false;

    if ((nvme.admin_completion_queue[nvme.admin_completion_queue_head * completion_queue_entry_bytes + 14] & (1 << 0) != 0) != nvme.admin_completion_queue_phase) {
        from_admin = true;
        nvme.admin_completion_queue_last_result = @ptrCast(*u32, @alignCast(@alignOf(u32), &nvme.admin_completion_queue[nvme.admin_completion_queue_head * completion_queue_entry_bytes + 0])).*;
        nvme.admin_completion_queue_last_status = (@ptrCast(*u16, @alignCast(@alignOf(u16), &nvme.admin_completion_queue[nvme.admin_completion_queue_head * completion_queue_entry_bytes + 14])).*) & 0xfffe;
        nvme.admin_completion_queue_head += 1;

        if (nvme.admin_completion_queue_head == admin_queue_entry_count) {
            nvme.admin_completion_queue_phase = !nvme.admin_completion_queue_phase;
            nvme.admin_completion_queue_head = 0;
        }

        nvme.write_cqhdbl(0, nvme.admin_completion_queue_head);

        // TODO: set event
    }

    while (nvme.io_completion_queue != null and ((nvme.io_completion_queue.?[nvme.io_completion_queue_head * completion_queue_entry_bytes + 14] & (1 << 0) != 0) != nvme.io_completion_queue_phase)) {
        log.debug("NVMe IO queue sucess", .{});
        from_io = true;

        const index = @ptrCast(*u16, @alignCast(@alignOf(u16), &nvme.io_completion_queue.?[nvme.io_completion_queue_head * completion_queue_entry_bytes + 12])).*;
        const status = (@ptrCast(*u16, @alignCast(@alignOf(u16), &nvme.io_completion_queue.?[nvme.io_completion_queue_head * completion_queue_entry_bytes + 14])).*) & 0xfffe;

        if (index < io_queue_entry_count) {
            if (status != 0) {
                @panic("failed");
            }
            // TODO: abstraction stuff
        } else @panic("wtf");

        @fence(.SeqCst);

        nvme.io_submission_queue_head = @ptrCast(*u16, @alignCast(@alignOf(u16), &nvme.io_completion_queue.?[nvme.io_completion_queue_head * completion_queue_entry_bytes + 8])).*;
        // TODO: event set
        nvme.io_completion_queue_head += 1;

        if (nvme.io_completion_queue_head == io_queue_entry_count) {
            nvme.io_completion_queue_phase = !nvme.io_completion_queue_phase;
            nvme.io_completion_queue_head = 0;
        }

        nvme.write_cqhdbl(1, nvme.io_completion_queue_head);
    }

    return from_admin or from_io;
}

const DataTransfer = enum(u2) {
    no_data_transfer = 0,
    host_to_controller = 1,
    controller_to_host = 2,
    bidirectional = 3,
};

fn SubmissionQueueEntry(comptime Opcode: type) type {
    return packed struct {
        command_dword0: CommandDword0(Opcode),
    };
}

fn CommandDword0(comptime Opcode: type) type {
    return packed struct {
        opcode: Opcode,
        fuse: FUSE,
        reserved: u4,
        psdt: PSDT,
        cid: u16,

        const FUSE = enum(u2) {
            normal = 0,
            fuse_first_command = 1,
            fuse_second_command = 2,
            reserved = 3,
        };

        const PSDT = enum(u2) {
            prps_used = 0,
            sgls_buffer = 1,
            sgls_segment = 2,
            reserved = 3,
        };

        comptime {
            kernel.assert_unsafe(@sizeOf(CommandDword0) == @sizeOf(u32));
        }
    };
}

const QueueType = enum(u2) {
    admin = 0,
    fabrics = 1,
    io = 2,

    const count = kernel.enum_values(QueueType).len;
};
const AdminOpcode = enum(u8) {
    delete_io_submission_queue = 0x00,
    create_io_submission_queue = 0x01,
    get_log_page = 0x02,
    delete_io_completion_queue = 0x04,
    create_io_completion_queue = 0x05,
    identify = 0x06,
    abort = 0x08,
    set_features = 0x09,
    get_features = 0x0a,
    asynchronous_event_request = 0x0c,
    namespace_management = 0x0d,
    firmware_commit = 0x10,
    firmware_image_download = 0x11,
    device_self_test = 0x14,
    namespace_attachment = 0x15,
    keep_alive = 0x18,
    directive_send = 0x19,
    directive_receive = 0x1a,
    virtualization_management = 0x1c,
    nvme_mi_send = 0x1d,
    nvme_mi_receive = 0x1e,
    capacity_management = 0x20,
    lockdown = 0x24,
    doorbell_buffer_config = 0x7c,
    fabrics_commands = 0x7f,
    format_nvm = 0x80,
    security_send = 0x81,
    security_receive = 0x82,
    sanitize = 0x84,
    get_lba_status = 0x86,

    pub inline fn is_generic_command(opcode: @This()) bool {
        return opcode & 0b10000000 != 0;
    }

    pub inline fn get_data_transfer(opcode: @This()) DataTransfer {
        return @intToEnum(DataTransfer, @truncate(u2, @enumToInt(opcode)));
    }

    pub inline fn get_function(opcode: @This()) u5 {
        return @truncate(u5, (@enumToInt(opcode) & 0b01111100) >> 2);
    }
};
const FabricsOpcode = enum(u8) {
    property_set = 0x00,
    connect = 0x01,
    property_get = 0x04,
    authentication_send = 0x05,
    authentication_receive = 0x06,
    disconnect = 0x08,
};

const IOOpcode = enum(u8) {
    flush = 0x00,
    reservation_register = 0x0d,
    reservation_report = 0x0e,
    reservation_acquire = 0x11,
    reservation_release = 0x15,

    _,
};

const opcodes = [QueueType.count]type{
    AdminOpcode,
    FabricsOpcode,
    IOOpcode,
};

const AdminCommonCommandFormat = packed struct {
    command_dword0: CommandDword0(AdminOpcode),
    nsid: u4,
    reserved: u8,
    // == Same as Common ==
    mptr: u8,
    dptr: u16,
    // == Same as Common ==
    ndt: u4,
    ndm: u4,
    command_dword12: u4,
    command_dword13: u4,
    command_dword14: u4,
    command_dword15: u4,

    comptime {
        kernel.assert_unsafe(@sizeOf(AdminCommonCommandFormat) == @sizeOf(u64));
    }
};

const CommonCompletionQueueEntry = packed struct {
    dw0: u32,
    dw1: u32,
    dw2: DW2,

    const DW2 = packed struct {
        sqhp: u16,
        sqid: u16,

        comptime {
            kernel.assert_unsafe(@sizeOf(DW2) == @sizeOf(u32));
        }
    };

    const DW3 = packed struct {
        cid: u16,
        phase_tag: bool,
        status_field: StatusField,

        comptime {
            kernel.assert_unsafe(@sizeOf(DW3) == @sizeOf(u32));
        }
    };

    const StatusField = packed struct {
        sc: u8,
        sct: u3,
        crd: u2,
        more: bool,
        dnr: bool,
    };

    const GenericCommandStatus = enum(u8) {
        successful_completion = 0x00,
        invalid_command_opcode = 0x01,
        invalid_field_in_command = 0x02,
        command_id_conflict = 0x03,
        data_transfer_error = 0x04,
        commmands_aborted_due_to_power_loss_notification = 0x05,
        internal_error = 0x06,
        command_abort_requested = 0x07,
        command_aborted_due_to_sq_deletion = 0x08,
        command_aborted_due_to_failed_fused_command = 0x09,
        command_aborted_due_to_missing_fused_command = 0x0a,
        invalid_namespace_or_format = 0x0b,
        command_sequence_error = 0x0c,
        invalid_sgl_segment_descriptor = 0x0d,
        invalid_number_of_sgl_descriptors = 0x0e,
        data_sgl_length_invalid = 0x0f,
        metadata_sgl_length_invalid = 0x10,
        sgl_descriptor_type_invalid = 0x11,
        invalid_use_of_controller_memory_buffer = 0x12,
        prp_offset_invalid = 0x13,
        atomic_write_unit_exceeded = 0x14,
        operation_denied = 0x15,
        sgl_offset_invalid = 0x16,
        reserved = 0x17,
        host_identifier_inconsistent_format = 0x18,
        keep_alive_timer_expired = 0x19,
        keep_alive_timeout_invalid = 0x1a,
        command_aborted_due_to_preempt_and_abort = 0x1b,
        sanitize_failed = 0x1c,
        sanitize_in_progress = 0x1d,
        sgl_data_block_glanularity_invalid = 0x1e,
        command_not_supported_for_queue_in_cmb = 0x1f,
        namespace_is_write_protected = 0x20,
        command_interrupted = 0x21,
        transient_transport_error = 0x22,
        command_prohibited_by_command_and_feature_lockdown = 0x23,
        admin_command_media_not_ready = 0x24,
        lba_out_of_range = 0x80,
        capacity_exceeded = 0x81,
        namespace_not_ready = 0x82,
        reservation_conflict = 0x83,
        format_in_progress = 0x84,
        invalid_value_size = 0x85,
        invalid_key_size = 0x86,
        kv_key_does_not_exist = 0x87,
        unrecovered_error = 0x88,
        key_exists = 0x89,
        _,
    };

    const CommandSpecificStatus = enum(u8) {
        completion_queue_invalid = 0x00,
        invalid_queue_identifier = 0x01,
        invalid_queue_size = 0x02,
        abort_command_limit_exceeded = 0x03,
        reserved = 0x04,
        asynchronous_event_request_limi_exceeded = 0x05,
        invalid_firmware_slot = 0x06,
        invalid_firmware_image = 0x07,
        invalid_interrupt_vector = 0x08,
        invalid_log_page = 0x09,
        invalid_format = 0x0a,
        firmware_activation_requires_conventional_reset = 0x0b,
        invalid_queue_deletion = 0x0c,
        feature_identifier_not_saveable = 0x0d,
        feature_not_changeable = 0x0e,
        feature_not_namespace_specific = 0x0f,
        firmware_activation_requires_nvm_subsystem_reset = 0x10,
        firmware_activation_requires_controller_level_reset = 0x11,
        firmware_activation_requires_maximum_time_violation = 0x12,
        firmware_activation_prohibited = 0x13,
        overlapping_range = 0x14,
        namespace_insufficient_capacity = 0x15,
        namespace_identifier_unavailable = 0x16,
        reserved = 0x17,
        namespace_already_attached = 0x18,
        namespace_is_private = 0x19,
        namespace_not_attached = 0x1a,
        thin_provisioning_not_supported = 0x1b,
        controller_list_invalid = 0x1c,
        device_self_test_in_progress = 0x1d,
        boot_partition_write_prohibited = 0x1e,
        invalid_controller_identifier = 0x1f,
        invalid_secondary_controller_state = 0x20,
        invalid_number_of_controller_resources = 0x21,
        invalid_resource_identifier = 0x22,
        sanitize_prohibited_while_persistent_memory_region_is_enabled = 0x23,
        ana_group_identifier_invalid = 0x24,
        ana_attach_failed = 0x25,
        insufficient_capacity = 0x26,
        namespace_attachment_limit_exceeded = 0x27,
        prohibition_of_command_execution_not_supported = 0x28,
        io_command_set_not_supported = 0x29,
        io_command_set_not_enabled = 0x2a,
        io_command_set_combination_rejected = 0x2b,
        invalid_io_command_set = 0x2c,
        identifier_unavailable = 0x2d,
        _,
    };

    const IOCommandSpecificStatus = enum(u8) {
        conflicting_attributes = 0x80,
        invalid_protection_information = 0x81,
        attempted_write_to_read_only_range = 0x82,
        command_size_limit_exceeded = 0x83,
        _,
        zoned_boundary_error = 0xb8,
        zone_is_full = 0xb9,
        zone_is_read_only = 0xba,
        zone_is_offline = 0xbb,
        zone_invalid_write = 0xbc,
        too_many_active_zones = 0xbd,
        too_many_open_zones = 0xbe,
        invalid_zone_state_transition = 0xbf,
    };

    const FabricsCommandSpecificStatus = enum(u8) {
        incompatible_format = 0x80,
        controller_busy = 0x81,
        connect_invalid_parameters = 0x82,
        connected_restar_discovery = 0x83,
        connect_invalid_host = 0x84,
        invalid_queue_type = 0x85,
        discover_restart = 0x90,
        authentication_required = 0x91,
    };

    const MediaAndDataIntegrityError = enum(u8) {
        write_fault = 0x80,
        unrecovered_read_error = 0x81,
        end_to_end_guard_check_error = 0x82,
        end_to_end_application_tag_check_error = 0x83,
        end_to_end_reference_tag_check_error = 0x84,
        compare_failure = 0x85,
        access_denied = 0x86,
        deallocated_or_unwritten_logical_block = 0x87,
        end_to_end_storage_tag_check_error = 0x88,
    };

    const PathRelatedStatus = enum(u8) {
        internal_path_error = 0x00,
        asymmetric_access_persistent_loss = 0x01,
        asymmetric_access_inaccessible = 0x02,
        asymmetric_access_transition = 0x03,

        controller_pathing_error = 0x60,

        host_pathing_error = 0x70,
        command_aborted_by_host = 0x71,
    };
};

const Property = struct {
    offset: u64,
    type: type,
};

const cap = Property{ .offset = 0, .type = CAP };
const vs = Property{ .offset = 0x08, .type = Version };
const intms = Property{ .offset = 0xc, .type = u32 };
const intmc = Property{ .offset = 0x10, .type = u32 };
const cc = Property{ .offset = 0x14, .type = CC };
const csts = Property{ .offset = 0x1c, .type = CSTS };
const nssr = Property{ .offset = 0x20, .type = u32 };
const aqa = Property{ .offset = 0x24, .type = AQA };
const asq = Property{ .offset = 0x28, .type = ASQ };
const acq = Property{ .offset = 0x30, .type = ACQ };
const cmbloc = Property{ .offset = 0x38, .type = CMBLOC };
const cmbsz = Property{ .offset = 0x3c, .type = CMBSZ };
const bpinfo = Property{ .offset = 0x40, .type = BPINFO };
const bprsel = Property{ .offset = 0x44, .type = BPRSEL };
const bpmbl = Property{ .offset = 0x48, .type = BPMBL };
const cmbmsc = Property{ .offset = 0x50, .type = CMBMSC };
const cmbsts = Property{ .offset = 0x58, .type = CMBSTS };
const cmbebs = Property{ .offset = 0x5c, .type = CMBEBS };
const cmbswtp = Property{ .offset = 0x60, .type = CMBSWTP };
const nssd = Property{ .offset = 0x64, .type = u32 };
const crto = Property{ .offset = 0x68, .type = CRTO };
const pmrcap = Property{ .offset = 0xe00, .type = PMRCAP };
const pmrctl = Property{ .offset = 0xe04, .type = PMRCTL };
const pmrsts = Property{ .offset = 0xe08, .type = PMRSTS };
const pmrebs = Property{ .offset = 0xe0c, .type = PMREBS };
const pmrswtp = Property{ .offset = 0xe10, .type = PMRSWTP };
const pmrmscl = Property{ .offset = 0xe14, .type = PMRMSCL };
const pmrmscu = Property{ .offset = 0xe18, .type = u32 };

const CAP = packed struct {
    mqes: u16,
    cqr: bool,
    ams: u2,
    reserved: u5,
    timeout: u8,
    doorbell_stride: u4,
    nssrs: bool,
    css: CSS,
    bps: bool,
    cps: u2,
    mpsmin: u4,
    mpsmax: u4,
    pmrs: bool,
    cmbs: bool,
    nsss: bool,
    crwms: bool,
    crims: bool,
    reserved2: u3,

    const CSS = packed struct {
        nvm_command_set: bool,
        reserved: u5,
        io_command_sets: bool,
        no_io_command_set: bool,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(CAP) == @sizeOf(u64));
    }
};

const Version = packed struct {
    tertiary: u8,
    minor: u8,
    major: u16,
};

const CC = packed struct {
    enable: bool,
    reserved: u3,
    css: CSS,
    mps: u4,
    ams: AMS,
    shn: SHN,
    iosqes: u4,
    iocqes: u4,
    crime: bool,
    reserved2: u7,

    const CSS = enum(u3) {
        nvm_command_set = 0b000,
        all_supported_io_command_sets = 0b110,
        admin_command_set_only = 0b111,
        _,
    };

    const AMS = enum(u3) {
        round_robin = 0b000,
        weighted_round_robin_with_urgent_priority_class = 0b001,
        vendor_specific = 0b111,
        _,
    };

    const SHN = enum(u2) {
        no_notification = 0b00,
        normal_shutdown_notification = 0b01,
        abrupt_shutdown_notification = 0b10,
        reserved = 0b11,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(CC) == @sizeOf(u32));
    }
};

const CSTS = packed struct {
    ready: bool,
    cfs: bool,
    shst: SHST,
    nssro: bool,
    pp: bool,
    st: bool,
    reserved: u25,

    const SHST = enum(u2) {
        norma_operation = 0,
        shutdown_processing_occurring = 1,
        shutdown_processing_complete = 2,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(CSTS) == @sizeOf(u32));
    }
};

const AQA = packed struct {
    asqs: u12,
    reserved: u4,
    acqs: u12,
    reserved2: u4,

    comptime {
        kernel.assert_unsafe(@sizeOf(AQA) == @sizeOf(u32));
    }
};

const ASQ = packed struct {
    reserved: u12,
    asqb: u52,

    comptime {
        kernel.assert_unsafe(@sizeOf(ASQ) == @sizeOf(u64));
    }
};

const ACQ = packed struct {
    reserved: u12,
    acqb: u52,

    comptime {
        kernel.assert_unsafe(@sizeOf(ACQ) == @sizeOf(u64));
    }
};

const CMBLOC = packed struct {
    bir: u3,
    cqmms: bool,
    cqpds: bool,
    cdpmls: bool,
    cdpcils: bool,
    cdmmms: bool,
    cqda: bool,
    reserved: u3,
    offset: u20,

    comptime {
        kernel.assert_unsafe(@sizeOf(CMBLOC) == @sizeOf(u32));
    }
};

const CMBSZ = packed struct {
    sqs: bool,
    cqs: bool,
    lists: bool,
    rds: bool,
    wds: bool,
    reserved: u3,
    szu: SZU,
    size: u20,

    const SZU = enum(u4) {
        kib_4 = 0,
        kib_64 = 1,
        mib_1 = 2,
        mib_16 = 3,
        mib_256 = 4,
        gib_4 = 5,
        gib_64 = 6,
        _,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(CMBSZ) == @sizeOf(u32));
    }
};

const BPINFO = packed struct {
    bpsz: u15,
    reserved: u9,
    brs: BRS,
    reserved: u5,
    abpid: bool,

    const BRS = enum(u2) {
        no_bpr = 0,
        bpr_in_progress = 1,
        bpr_completed = 2,
        bpr_error = 3,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(BPINFO) == @sizeOf(u32));
    }
};

const BPRSEL = packed struct {
    bprsz: u10,
    bprof: u20,
    reserved: bool,
    bpid: bool,

    comptime {
        kernel.assert_unsafe(@sizeOf(BPRSEL) == @sizeOf(u32));
    }
};

const BPMBL = packed struct {
    reserved: u12,
    bmbba: u52,

    comptime {
        kernel.assert_unsafe(@sizeOf(BPMBL) == @sizeOf(u64));
    }
};

const CMBMSC = packed struct {
    cre: bool,
    cmse: bool,
    reserved: u10,
    cba: u52,

    comptime {
        kernel.assert_unsafe(@sizeOf(CMBMSC) == @sizeOf(u64));
    }
};

const CMBSTS = packed struct {
    cbai: bool,
    reserved: u31,

    comptime {
        kernel.assert_unsafe(@sizeOf(CMBSTS) == @sizeOf(u32));
    }
};

const CMBEBS = packed struct {
    cmbszu: CMBSZU,
    read_bypass_behavior: bool,
    reserved: u3,
    cmbwbz: u24,

    const CMBSZU = enum(u4) {
        bytes = 0,
        kib = 1,
        mib = 2,
        gib = 3,
        _,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(CMBEBS) == @sizeOf(u32));
    }
};

const CMBSWTP = packed struct {
    cmbswtu: CMBSWTU,
    reserved: u4,
    cmbswtv: u24,

    const CMBSWTU = enum(u4) {
        bytes_s = 0,
        kibs_s = 1,
        mibs_s = 2,
        gibs_s = 3,
        _,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(CMBSWTP) == @sizeOf(u32));
    }
};

const CRTO = packed struct {
    crwmt: u16,
    crimt: u16,

    comptime {
        kernel.assert_unsafe(@sizeOf(CRTO) == @sizeOf(u32));
    }
};

const PMRCAP = packed struct {
    reserved: u3,
    rds: bool,
    wds: bool,
    bir: u3,
    pmrtu: PMRTU,
    pmrwbm: u4,
    reserved: u2,
    pmrto: u8,
    cmss: bool,
    reserved: u7,

    const PMRTU = enum(u2) {
        ms_500 = 0,
        minutes = 1,
        _,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(PMRCAP) == @sizeOf(u32));
    }
};

const PMRCTL = packed struct {
    enable: bool,
    reserved: u31,

    comptime {
        kernel.assert_unsafe(@sizeOf(PMRCTL) == @sizeOf(u32));
    }
};

const PMRSTS = packed struct {
    err: u8,
    nrdy: bool,
    hsts: HSTS,
    cbai: bool,
    reserved: u20,

    const HSTS = enum(u2) {
        normal = 0,
        restore_error = 1,
        read_only = 2,
        unreliable = 3,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(PMRSTS) == @sizeOf(u32));
    }
};

const PMREBS = packed struct {
    pmrszu: PMRSZU,
    read_bypass_behavior: bool,
    reserved: u3,
    pmrwbz: u24,

    const PMRSZU = enum(u4) {
        bytes = 0,
        kib = 1,
        mib = 2,
        gib = 3,
        _,
    };
    comptime {
        kernel.assert_unsafe(@sizeOf(PMREBS) == @sizeOf(u32));
    }
};

const PMRSWTP = packed struct {
    pmrswtu: PMRSWTU,
    reserved: u4,
    pmrswtv: u24,

    const PMRSWTU = enum(u4) {
        bytes_s = 0,
        kib_s = 1,
        mib_s = 2,
        gib_s,
        _,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(PMRSWTP) == @sizeOf(u32));
    }
};

const PMRMSCL = packed struct {
    reserved: bool,
    cmse: bool,
    reserved2: u10,
    cba: u20,

    comptime {
        kernel.assert_unsafe(@sizeOf(PMRMSCL) == @sizeOf(u32));
    }
};
