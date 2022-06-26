// This has been implemented with NVMe Specification 1.4c
const kernel = @import("../kernel/kernel.zig");
const log = kernel.log.scoped(.NVMe);
const TODO = kernel.TODO;
const PCI = @import("pci.zig");
const DMA = @import("dma.zig");

const x86_64 = @import("../kernel/arch/x86_64.zig");
const Physical = kernel.Physical;
const Virtual = kernel.Virtual;

const NVMe = @This();
pub var controller: NVMe = undefined;

const submission_queue_entry_bytes = 64;
const completion_queue_entry_bytes = 16;

const Admin = struct {
    submission_queue: AdminSubmissionQueue,
    completion_queue: AdminCompletionQueue,

    const queue_entry_count = 2;
};

const IO = struct {
    submission_queue: IOSubmissionQueue,
    completion_queue: IOCompletionQueue,
    lock: kernel.Spinlock,

    const queue_entry_count = 256;
};

const AdminSubmissionQueue = struct {
    ptr: ?[*]u8,
    tail: u32,
};

const IOSubmissionQueue = struct {
    ptr: ?[*]u8,
    tail: u32,
    head: u32,
};

const AdminCompletionQueue = struct {
    ptr: ?[*]u8,
    head: u32,
    last_result: u32,
    last_status: u16,
    phase: bool,
};

const IOCompletionQueue = struct {
    ptr: ?[*]u8,
    head: u32,
    phase: bool,
};

device: *PCI.Device,
capabilities: CAP,
version: Version,
doorbell_stride: u64,
ready_transition_timeout: u64,
maximum_data_transfer_bytes: u64,
rtd3_entry_latency_us: u32,
maximum_data_outstanding_commands: u16,
model: [40]u8,

admin: Admin,
io: IO,

prp_list_pages: [IO.queue_entry_count]Physical.Address,
prp_list_virtual: Virtual.Address,
drives: []Drive,

const general_timeout = 5000;
const Command = [16]u32;

pub fn new(device: *PCI.Device) NVMe {
    return NVMe{
        .device = device,
        .capabilities = kernel.zeroes(CAP),
        .version = kernel.zeroes(Version),
        .doorbell_stride = 0,
        .ready_transition_timeout = 0,
        .maximum_data_transfer_bytes = 0,
        .rtd3_entry_latency_us = 0,
        .maximum_data_outstanding_commands = 0,
        .model = kernel.zeroes([40]u8),
        .admin = Admin{
            .submission_queue = AdminSubmissionQueue{
                .ptr = null,
                .tail = 0,
            },
            .completion_queue = AdminCompletionQueue{
                .ptr = null,
                .head = 0,
                .last_result = 0,
                .last_status = 0,
                .phase = false,
            },
        },
        .io = IO{
            .submission_queue = IOSubmissionQueue{
                .ptr = null,
                .tail = 0,
                .head = 0,
            },
            .completion_queue = IOCompletionQueue{
                .ptr = null,
                .head = 0,
                .phase = false,
            },
            .lock = kernel.Spinlock.new(),
        },
        .drives = &.{},
        .prp_list_pages = [1]Physical.Address{Physical.Address.temporary_invalid()} ** IO.queue_entry_count,
        .prp_list_virtual = Virtual.Address.temporary_invalid(),
    };
}

pub fn find(pci: *PCI) ?*PCI.Device {
    return pci.find_device(0x1, 0x8);
}

const Error = error{
    not_found,
};

pub fn find_and_init(pci: *PCI) Error!void {
    const nvme_device = find(pci) orelse return Error.not_found;
    log.debug("Found NVMe drive", .{});
    controller = NVMe.new(nvme_device);
    const result = controller.device.enable_features(PCI.Device.Features.from_flags(&.{ .interrupts, .busmastering_dma, .memory_space_access, .bar0 }));
    kernel.assert(@src(), result);
    log.debug("Device features enabled", .{});

    controller.init();
}

inline fn read(nvme: *NVMe, comptime register: Property) register.type {
    return nvme.device.read_bar(register.type, 0, register.offset);
}

inline fn write(nvme: *NVMe, comptime register: Property, value: register.type) void {
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
    @ptrCast(*Command, @alignCast(@alignOf(Command), &nvme.admin.submission_queue.ptr.?[nvme.admin.submission_queue.tail * @sizeOf(Command)])).* = command.*;
    nvme.admin.submission_queue.tail = (nvme.admin.submission_queue.tail + 1) % Admin.queue_entry_count;

    // TODO: reset event
    @fence(.SeqCst); // best memory barrier?
    kernel.assert(@src(), kernel.arch.are_interrupts_enabled());
    log.debug("Entering in a wait state", .{});
    nvme.write_sqtdbl(0, nvme.admin.submission_queue.tail);
    asm volatile ("hlt");
    // TODO: wait for event
    //

    if (nvme.admin.completion_queue.last_status != 0) {
        const do_not_retry = nvme.admin.completion_queue.last_status & 0x8000 != 0;
        const more = nvme.admin.completion_queue.last_status & 0x4000 != 0;
        const command_retry_delay = @truncate(u8, nvme.admin.completion_queue.last_status >> 12) & 0x03;
        const status_code_type = @truncate(u8, nvme.admin.completion_queue.last_status >> 9) & 0x07;
        const status_code = @truncate(u8, nvme.admin.completion_queue.last_status >> 1);
        _ = do_not_retry;
        _ = more;
        _ = command_retry_delay;
        _ = status_code_type;
        _ = status_code;
        log.err("Admin command failed", .{});

        return false;
    }

    if (result) |p_result| p_result.* = nvme.admin.completion_queue.last_status;
    return true;
}

const sector_size = 0x200;

pub const Operation = enum {
    read,
    write,
};

pub const DiskWork = struct {
    size: u64,
    offset: u64,
    operation: Operation,
};

const Drive = struct {
    nsid: u32,
};

pub fn access(nvme: *NVMe, drive: NVMe.Drive, buffer: *DMA.Buffer, disk_work: DiskWork) bool {
    log.debug("Accesing NVMe drive for {s}. Buffer: (0x{x}, 0x{x}). To {s} {} bytes from offset {}", .{ @tagName(disk_work.operation), buffer.address.value, buffer.total_size, @tagName(disk_work.operation), disk_work.size, disk_work.offset });
    kernel.assert(@src(), buffer.completed_size == 0);
    const first_segment = buffer.next_segment(.can_write);
    log.debug("First segment: (0x{x}, 0x{x})", .{ first_segment.address.value, first_segment.size });
    const prp1_physical_address = first_segment.address;
    const is_last = first_segment.size == buffer.total_size;
    var prp2_physical_address = Physical.Address.temporary_invalid();
    if (!is_last) {
        // not necessary to cache it, but just in case
        const second_completed_size = buffer.completed_size;
        const second_segment = buffer.next_segment(.cannot_write);
        const second_segment_last = second_completed_size + second_segment.size == buffer.total_size;
        if (second_segment_last) {
            prp2_physical_address = second_segment.address;
        }
    }

    nvme.io.lock.acquire();
    // TODO: @Multithread
    const new_tail = (nvme.io.submission_queue.tail + 1) % IO.queue_entry_count;
    const submission_queue_full = new_tail == nvme.io.submission_queue.head;

    if (!submission_queue_full) {
        // TODO: check if these are aligned to the sector size
        const sector_offset = disk_work.offset / sector_size;
        const sector_count = disk_work.size / sector_size;

        log.debug("about to check for validity", .{});
        if (!prp2_physical_address.is_valid()) {
            log.debug("IO submission queue tail: {}", .{nvme.io.submission_queue.tail});
            prp2_physical_address = nvme.prp_list_pages[nvme.io.submission_queue.tail];
            log.debug("Mapping PRP2 physical address 0x{x} to PRP list virtual base ptr 0x{x}", .{ prp2_physical_address.value, nvme.prp_list_virtual.value });
            kernel.assert(@src(), !kernel.arch.are_interrupts_enabled());
            //kernel.address_space.map(prp2_physical_address, nvme.prp_list_virtual, Virtual.AddressSpace.Flags.from_flags(&.{.read_write}));

            var index: u64 = 0;
            while (buffer.completed_size < buffer.total_size) : (index += 1) {
                if (index == kernel.arch.page_size / @sizeOf(u64)) {
                    @panic("out of bounds");
                }

                // TODO: this is so hacky
                const address = buffer.next_segment(.can_write).address;
                log.debug("Address: 0x{x}", .{address.value});
                nvme.prp_list_virtual.access([*]Physical.Address)[index] = address;
            }
        }
        log.debug("ended to check for validity", .{});

        var command = @ptrCast(*Command, @alignCast(@alignOf(Command), &nvme.io.submission_queue.ptr.?[nvme.io.submission_queue.tail * submission_queue_entry_bytes]));
        command[0] = (nvme.io.submission_queue.tail << 16) | @enumToInt(if (disk_work.operation == .write) NVMCommandOpcode.write else NVMCommandOpcode.read);
        // TODO:
        command[1] = drive.nsid;
        command[2] = 0;
        command[3] = 0;
        command[4] = 0;
        command[5] = 0;
        command[6] = @truncate(u32, prp1_physical_address.value);
        command[7] = @truncate(u32, prp1_physical_address.value >> 32);
        command[8] = @truncate(u32, prp2_physical_address.value);
        command[9] = @truncate(u32, prp2_physical_address.value >> 32);
        command[10] = @truncate(u32, sector_offset);
        command[11] = @truncate(u32, sector_offset >> 32);
        command[12] = @truncate(u16, sector_count);
        command[13] = 0;
        command[14] = 0;
        command[15] = 0;

        nvme.io.submission_queue.tail = new_tail;
        log.debug("Sending the command", .{});
        @fence(.SeqCst);
        nvme.write_sqtdbl(1, new_tail);
        asm volatile ("hlt");
        //for (prp1_virtual_address.access([*]u8)[0..0x1000]) |byte, i| {
        //if (byte != 0) {
        //log.debug("[{}]: 0x{x}", .{ i, byte });
        //}
        //}
        //for (prp2_physical_address.to_higher_half_virtual_address().access([*]u8)[0..0x1000]) |byte, i| {
        //if (byte != 0) {
        //log.debug("[{}]: 0x{x}", .{ i, byte });
        //}
        //}
        TODO(@src());
    } else @panic("queue full");

    if (submission_queue_full) {
        @panic("queue full wait");
    }

    return true;
}

pub fn init(nvme: *NVMe) void {
    log.debug("NVMe initialization", .{});
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
        // TODO. HACK we should use a timeout here
        // TODO: PRobably buggy
        while (!nvme.read(csts).ready) {
            log.debug("busy waiting", .{});
        }
        var config = nvme.read(cc);
        config.enable = false;
        log.debug("NVMe writing configuration", .{});
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
        aqa_value.asqs = Admin.queue_entry_count - 1;
        aqa_value.acqs = Admin.queue_entry_count - 1;
        break :blk aqa_value;
    });

    const admin_submission_queue_size = Admin.queue_entry_count * submission_queue_entry_bytes;
    const admin_completion_queue_size = Admin.queue_entry_count * completion_queue_entry_bytes;
    const admin_queue_page_count = kernel.align_forward(admin_submission_queue_size, kernel.arch.page_size) + kernel.align_forward(admin_completion_queue_size, kernel.arch.page_size);
    const admin_queue_physical_address = Physical.Memory.allocate_pages(admin_queue_page_count) orelse @panic("admin queue");
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
    kernel.address_space.map(admin_submission_queue_physical_address, admin_submission_queue_virtual_address, Virtual.AddressSpace.Flags.from_flags(&.{.read_write}));
    kernel.address_space.map(admin_completion_queue_physical_address, admin_completion_queue_virtual_address, Virtual.AddressSpace.Flags.from_flags(&.{.read_write}));

    nvme.admin.submission_queue.ptr = admin_submission_queue_virtual_address.access([*]u8);
    nvme.admin.completion_queue.ptr = admin_completion_queue_virtual_address.access([*]u8);

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
    const identify_data_physical_address = Physical.Memory.allocate_pages(3) orelse @panic("identify");
    const identify_data_virtual_address = identify_data_physical_address.to_higher_half_virtual_address();
    kernel.address_space.map(identify_data_physical_address, identify_data_virtual_address, Virtual.AddressSpace.Flags.from_flag(.read_write));
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
        const size = kernel.align_forward(IO.queue_entry_count * completion_queue_entry_bytes, kernel.arch.page_size);
        const page_count = kernel.bytes_to_pages(size, true);
        const queue_physical_address = Physical.Memory.allocate_pages(page_count) orelse @panic("ph comp");

        const physical_region = Physical.Memory.Region.new(queue_physical_address, size);
        physical_region.map(&kernel.address_space, queue_physical_address.to_higher_half_virtual_address(), Virtual.AddressSpace.Flags.from_flag(.read_write));
        nvme.io.completion_queue.ptr = queue_physical_address.to_higher_half_virtual_address().access([*]u8);

        var command = kernel.zeroes(Command);
        command[0] = 0x05;
        command[6] = @truncate(u32, queue_physical_address.value);
        command[7] = @truncate(u32, queue_physical_address.value >> 32);
        command[10] = 1 | ((IO.queue_entry_count - 1) << 16);
        command[11] = (1 << 0) | (1 << 1);

        if (!nvme.issue_admin_command(&command, null)) @panic("create queue");
    }

    {
        const size = kernel.align_forward(IO.queue_entry_count * submission_queue_entry_bytes, kernel.arch.page_size);
        const page_count = kernel.bytes_to_pages(size, true);
        const queue_physical_address = Physical.Memory.allocate_pages(page_count) orelse @panic("ph comp");

        const physical_region = Physical.Memory.Region.new(queue_physical_address, size);
        physical_region.map(&kernel.address_space, queue_physical_address.to_higher_half_virtual_address(), Virtual.AddressSpace.Flags.from_flag(.read_write));
        nvme.io.submission_queue.ptr = queue_physical_address.to_higher_half_virtual_address().access([*]u8);

        var command = kernel.zeroes(Command);
        command[0] = 0x01;
        command[6] = @truncate(u32, queue_physical_address.value);
        command[7] = @truncate(u32, queue_physical_address.value >> 32);
        command[10] = 1 | ((IO.queue_entry_count - 1) << 16);
        command[11] = (1 << 0) | (1 << 16);

        if (!nvme.issue_admin_command(&command, null)) @panic("create queue");
    }

    {
        for (nvme.prp_list_pages) |*prp_list_page| {
            prp_list_page.* = Physical.Memory.allocate_pages(1) orelse @panic("prp physical");
        }

        kernel.address_space.map(nvme.prp_list_pages[0], nvme.prp_list_pages[0].to_higher_half_virtual_address(), Virtual.AddressSpace.Flags.from_flag(.read_write));
        nvme.prp_list_virtual = nvme.prp_list_pages[0].to_higher_half_virtual_address();
    }

    var nsid: u32 = 0;
    const expected_drive_count = 1;
    nvme.drives = kernel.core_heap.allocate_many(Drive, expected_drive_count) orelse @panic("unable to allocate space for drives");
    var drive_count: u64 = 0;
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
            if (drive_count > expected_drive_count - 1) {
                @panic("Found more devices than expected");
            }
            const sector_bytes = @as(u64, 1) << sector_bytes_exponent;
            log.debug("sector bytes: {}", .{sector_bytes});

            var drive = &nvme.drives[drive_count];
            drive.* = Drive{
                .nsid = nsid,
            };
            drive_count += 1;
            log.debug("Device NSID: {}", .{nsid});
        }
    }

    kernel.assert(@src(), drive_count <= expected_drive_count);
}

pub const Callback = fn (nvme: *NVMe, line: u64) bool;

pub fn handle_irq(nvme: *NVMe, line: u64) bool {
    _ = line;
    var from_admin = false;
    var from_io = false;

    if (nvme.admin.completion_queue.ptr != null and ((nvme.admin.completion_queue.ptr.?[nvme.admin.completion_queue.head * completion_queue_entry_bytes + 14] & (1 << 0) != 0) != nvme.admin.completion_queue.phase)) {
        from_admin = true;
        nvme.admin.completion_queue.last_result = @ptrCast(*u32, @alignCast(@alignOf(u32), &nvme.admin.completion_queue.ptr.?[nvme.admin.completion_queue.head * completion_queue_entry_bytes + 0])).*;
        nvme.admin.completion_queue.last_status = (@ptrCast(*u16, @alignCast(@alignOf(u16), &nvme.admin.completion_queue.ptr.?[nvme.admin.completion_queue.head * completion_queue_entry_bytes + 14])).*) & 0xfffe;
        nvme.admin.completion_queue.head += 1;

        if (nvme.admin.completion_queue.head == Admin.queue_entry_count) {
            nvme.admin.completion_queue.phase = !nvme.admin.completion_queue.phase;
            nvme.admin.completion_queue.head = 0;
        }

        nvme.write_cqhdbl(0, nvme.admin.completion_queue.head);

        // TODO: set event
    } else {
        kernel.panic("Admin completion queue: {*}", .{nvme.admin.completion_queue.ptr});
    }

    while (nvme.io.completion_queue.ptr != null and ((nvme.io.completion_queue.ptr.?[nvme.io.completion_queue.head * completion_queue_entry_bytes + 14] & (1 << 0) != 0) != nvme.io.completion_queue.phase)) {
        from_io = true;

        const index = @ptrCast(*u16, @alignCast(@alignOf(u16), &nvme.io.completion_queue.ptr.?[nvme.io.completion_queue.head * completion_queue_entry_bytes + 12])).*;
        const status = (@ptrCast(*u16, @alignCast(@alignOf(u16), &nvme.io.completion_queue.ptr.?[nvme.io.completion_queue.head * completion_queue_entry_bytes + 14])).*) & 0xfffe;

        if (index < IO.queue_entry_count) {
            log.debug("Success: {}", .{status == 0});
            if (status != 0) {
                @panic("failed");
            }
            // TODO: abstraction stuff
        } else @panic("wtf");

        @fence(.SeqCst);

        nvme.io.submission_queue.head = @ptrCast(*u16, @alignCast(@alignOf(u16), &nvme.io.completion_queue.ptr.?[nvme.io.completion_queue.head * completion_queue_entry_bytes + 8])).*;
        // TODO: event set
        nvme.io.completion_queue.head += 1;

        if (nvme.io.completion_queue.head == IO.queue_entry_count) {
            nvme.io.completion_queue.phase = !nvme.io.completion_queue.phase;
            nvme.io.completion_queue.head = 0;
        }

        nvme.write_cqhdbl(1, nvme.io.completion_queue.head);
    } else {
        log.err("IO completion queue: {*}", .{nvme.io.completion_queue.ptr});
    }

    return from_admin or from_io;
}

const DataTransfer = enum(u2) {
    no_data_transfer = 0,
    host_to_controller = 1,
    controller_to_host = 2,
    bidirectional = 3,
};

const AdminCommandOpcode = enum(u8) {
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
};

const NVMCommandOpcode = enum(u8) {
    flush = 0x00,
    write = 0x01,
    read = 0x02,
    write_uncorrectable = 0x04,
    compare = 0x05,
    write_zeroes = 0x08,
    dataset_management = 0x09,
    verify = 0x0c,
    reservation_register = 0x0d,
    reservation_report = 0x0e,
    reservation_acquire = 0x11,
    reservation_release = 0x15,
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
const nssd = Property{ .offset = 0x64, .type = u32 };
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
    reserved2: u8,

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
    reserved: u26,

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
    reserved: u19,

    const HSTS = enum(u3) {
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

const CommandDword0 = packed struct {
    opcode: u8,
    fuse: FUSE,
    reserved: u4,
    psdt: PSDT,
    cid: u16,

    const FUSE = enum(u2) {
        normal = 0,
        fuse_first = 1,
        fuse_second = 2,
        reserved = 3,
    };

    const PSDT = enum(u2) {
        prps = 0,
        sgl_buffer = 1,
        sgl_segment = 2,
        reserved = 3,
    };

    comptime {
        kernel.assert_unsafe(@sizeOf(@This()) == @sizeOf(u32));
    }
};

//struct CommonCommand {
//uint8_t opcode;
//uint8_t flags;
//uint16_t commandId;
//uint32_t namespaceId;
//uint32_t cdw2[2];
//uint64_t metadata;
//DataPointer dataPtr;
//uint32_t cdw10;
//uint32_t cdw11;
//uint32_t cdw12;
//uint32_t cdw13;
//uint32_t cdw14;
//uint32_t cdw15;
//};

const CommonCommand = packed struct {
    cdw0: CommandDword0,
    nsid: u32,
    reserved: u64,
    mptr: u64,

    comptime {
        kernel.assert_unsafe(@sizeOf(@This()) == 64);
    }
};

const Command0 = packed union {
    common: CommonCommand,
};
