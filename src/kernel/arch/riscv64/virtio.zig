const kernel = @import("root");
const common = kernel.common;
const TODO = common.TODO;

// TODO: make possible to instantiate more than one same-class virtio driver

const page_size = kernel.arch.page_size;

// **** VIRTIO DRIVER CONFORMANCE ****
// A driver MUST conform to three conformance clauses:
// - Clause 7.2 [ ]
// - One of clauses:
//     - 7.2.1 [ ]
//     - 7.2.2 [ ]
//     - 7.2.3 [ ]
//
// - One of clauses:
//     - 7.2.4  [ ]
//     - 7.2.5  [ ]
//     - 7.2.6  [ ]
//     - 7.2.7  [ ]
//     - 7.2.8  [ ]
//     - 7.2.9  [ ]
//     - 7.2.12 [ ]
//
// CLAUSE 7.2
// A driver must conform to the following normative statements:
//     - 2.1.1      [x] We panic on failure
//     - 2.2.1      [x] We go with the features the virtio driver produces
//     - 2.4.1      [ ] ???
//     - 2.6.1      [x]
//     - 2.6.4.2    [ ] ???
//     - 2.6.5.2    [ ] ???
//     - 2.6.5.3.1  [ ]
//     - 2.6.7.1    [ ]
//     - 2.6.6.1    [ ]
//     - 2.6.8.3    [ ]
//     - 2.6.10.1   [ ]
//     - 2.6.13.3.1 [ ]
//     - 2.6.13.4.1 [ ]
//     - 3.1.1      [ ]
//     - 3.3.1      [ ]
//     - 6.1        [ ]

const ring_size = 128;

pub const Block = @import("virtio_block.zig");
pub const GPU = @import("virtio_gpu.zig");

pub const MMIO = struct {
    magic_value: u32,
    version: u32,
    device_id: u32,
    vendor_id: u32,

    device_features: u32,
    device_feature_selector: u32,
    reserved1: [8]u8,

    driver_features: u32,
    driver_feature_selector: u32,
    reserved2: [8]u8,

    queue_selector: u32,
    queue_num_max: u32,
    queue_num: u32,
    reserved3: [4]u8,

    reserved4: [4]u8,
    queue_ready: u32,
    reserved5: [8]u8,

    queue_notify: u32,
    reserved6: [12]u8,

    interrupt_status: InterruptStatus,
    interrupt_ack: u32,
    reserved7: [8]u8,

    device_status: DeviceStatus,
    reserved8: [12]u8,

    queue_descriptor_low: u32,
    queue_descriptor_high: u32,
    reserved9: [8]u8,

    queue_available_low: u32,
    queue_available_high: u32,
    reserved10: [8]u8,

    queue_used_low: u32,
    queue_used_high: u32,
    reserved11: [8]u8,

    reserved12: [0x4c]u8,
    config_gen: u32,

    pub const magic = 0x74726976;
    pub const version = 2;

    pub const configuration_offset = 0x100;

    pub fn init(mmio: *volatile @This(), comptime FeaturesEnumT: type) void {
        if (mmio.magic_value != magic) @panic("virtio magic corrupted");
        if (mmio.version != version) @panic("outdated virtio spec");
        if (mmio.device_id == 0) @panic("invalid device");

        // 1. Reset
        mmio.reset_device();
        if (mmio.device_status.bits != 0) @panic("Device status should be reset and cleared");

        // 2. Ack the device
        mmio.device_status.or_flag(.acknowledge);

        // 3. The driver knows how to use the device
        mmio.device_status.or_flag(.driver);

        // 4. Read device feature bits and write (a subset of) them
        var features = mmio.device_features;
        mmio.debug_device_features(FeaturesEnumT);
        log.debug("Features: {b}", .{features});
        // Disable VIRTIO F RING EVENT INDEX
        features &= ~@as(u32, 1 << 29);
        mmio.driver_features = features;

        // 5. Set features ok status bit
        mmio.device_status.or_flag(.features_ok);

        if (!mmio.device_status.contains(.features_ok)) @panic("unsupported features");
    }

    pub inline fn reset_device(mmio: *volatile @This()) void {
        mmio.device_status = DeviceStatus.empty();
    }

    pub inline fn set_driver_initialized(mmio: *volatile @This()) void {
        mmio.device_status.or_flag(.driver_ok);
    }

    pub inline fn notify_queue(mmio: *volatile @This()) void {
        mmio.queue_notify = 0;
    }

    pub fn add_queue_to_device(mmio: *volatile @This(), selected_queue: u32) *volatile SplitQueue {
        if (mmio.queue_num_max < ring_size) {
            @panic("foooo");
        }

        mmio.queue_selector = selected_queue;
        mmio.queue_num = ring_size;

        if (mmio.queue_ready != 0) @panic("queue ready");

        const total_size = @sizeOf(SplitQueue) + (@sizeOf(Descriptor) * ring_size) + @sizeOf(Available) + @sizeOf(Used);
        const page_count = total_size / page_size + @boolToInt(total_size % page_size != 0);
        // All physical address space is identity-mapped so mapping is not needed here
        const queue_physical = kernel.arch.Physical.allocate1(page_count) orelse @panic("unable to allocate memory for virtio block device queue");
        const queue = @intToPtr(*volatile SplitQueue, kernel.arch.Virtual.AddressSpace.physical_to_virtual(queue_physical));
        // TODO: distinguist between physical and virtual
        queue.num = 0;
        queue.last_seen_used = 0;
        var ptr: u64 = common.align_forward(queue_physical + @sizeOf(SplitQueue), SplitQueue.descriptor_table_alignment);
        queue.descriptors = @intToPtr(@TypeOf(queue.descriptors), ptr);
        // identity-mapped
        const physical = @ptrToInt(queue);
        const descriptor = physical + @sizeOf(SplitQueue);
        ptr += ring_size * @sizeOf(Descriptor);
        ptr = common.align_forward(ptr, SplitQueue.available_ring_alignment);
        const available = ptr;
        queue.available = @intToPtr(@TypeOf(queue.available), available);
        queue.available.flags = 0;
        queue.available.index = 0;
        ptr += @sizeOf(Available);
        ptr = common.align_forward(ptr, SplitQueue.used_ring_alignment);
        const used = ptr;
        queue.used = @intToPtr(@TypeOf(queue.used), used);
        queue.used.flags = 0;
        queue.used.index = 0;

        // notify device of queue
        mmio.queue_num = ring_size;

        // specify queue structs
        mmio.queue_descriptor_low = @truncate(u32, descriptor);
        mmio.queue_descriptor_high = @truncate(u32, descriptor >> 32);
        mmio.queue_available_low = @truncate(u32, available);
        mmio.queue_available_high = @truncate(u32, available >> 32);
        mmio.queue_used_low = @truncate(u32, used);
        mmio.queue_used_high = @truncate(u32, used >> 32);

        mmio.queue_ready = 1;

        return queue;
    }

    pub const DeviceStatus = common.Bitflag(true, enum(u32) {
        acknowledge = 0,
        driver = 1,
        driver_ok = 2,
        features_ok = 3,
        device_needs_reset = 6,
        failed = 7,
    });

    const Features = enum(u6) {
        ring_indirect_descriptors = 28,
        ring_event_index = 29,
        version_1 = 32,
        access_platform = 33,
        ring_packed = 34,
        in_order = 35,
        order_platform = 36,
        single_root_io_virtualization = 37,
        notification_data = 38,
    };

    const log = common.log.scoped(.MMIO);

    pub fn debug_device_status(mmio: *volatile MMIO) void {
        log.debug("Reading device status...", .{});
        const device_status = mmio.device_status;
        for (common.enum_values(DeviceStatus)) |flag| {
            if (device_status & @enumToInt(flag) != 0) {
                log.debug("Flag set: {}", .{flag});
            }
        }
    }

    pub fn debug_device_features(mmio: *volatile MMIO, comptime FeaturesEnum: type) void {
        mmio.device_feature_selector = 0;
        const low = mmio.device_features;
        mmio.device_feature_selector = 1;
        const high = mmio.device_features;
        mmio.device_feature_selector = 0;

        const features: u64 = (@intCast(u64, high) << 32) | low;
        for (common.enum_values(Features)) |feature| {
            if (features & (@intCast(u64, 1) << @enumToInt(feature)) != 0) {
                log.debug("Device has feature: {s}, bit {}", .{ @tagName(feature), @enumToInt(feature) });
            }
        }

        for (common.enum_values(FeaturesEnum)) |feature| {
            if (features & (@intCast(u64, 1) << @enumToInt(feature)) != 0) {
                log.debug("Device has {}: {s}, bit {}", .{ FeaturesEnum, @tagName(feature), @enumToInt(feature) });
            }
        }
    }
};

pub const Descriptor = struct {
    address: u64,
    length: u32,
    flags: u16,
    next: u16,

    pub const Flag = enum(u16) {
        next = 1 << 0,
        write_only = 1 << 1,
        indirect = 1 << 2,
    };
};

const Available = struct {
    flags: u16,
    index: u16,
    ring: [ring_size]u16,
    event: u16,
    padding: [2]u8,
};

const Used = struct {
    flags: u16,
    index: u16,
    ring: [ring_size]Entry,
    event: u16,

    const Entry = struct {
        id: u32,
        length: u32,
    };
};

pub const SplitQueue = struct {
    num: u32,
    last_seen_used: u32,
    descriptors: [*]align(descriptor_table_alignment) volatile Descriptor,
    available: *align(available_ring_alignment) volatile Available,
    used: *align(used_ring_alignment) volatile Used,

    const descriptor_table_alignment = 16;
    const available_ring_alignment = 2;
    const used_ring_alignment = 4;

    pub fn push_descriptor(queue: *volatile SplitQueue, p_descriptor_index: *u16) *volatile Descriptor {
        // TODO Cause for a bug
        p_descriptor_index.* = @intCast(u16, queue.num);
        const descriptor = &queue.descriptors[queue.num];
        queue.num += 1;

        while (queue.num >= ring_size) : (queue.num -= ring_size) {}

        return descriptor;
    }

    pub fn push_available(queue: *volatile SplitQueue, descriptor: u16) void {
        queue.available.ring[queue.available.index % ring_size] = descriptor;
        queue.available.index += 1;
    }

    pub fn pop_used(queue: *volatile SplitQueue) ?*volatile Descriptor {
        if (queue.last_seen_used == queue.used.index) return null;

        const id = queue.used.ring[queue.last_seen_used % ring_size].id;
        queue.last_seen_used += 1;
        const used = &queue.descriptors[id];

        return used;
    }

    pub fn get_descriptor(queue: *volatile SplitQueue, descriptor_id: u16) ?*volatile Descriptor {
        if (descriptor_id < ring_size) return &queue.descriptors[descriptor_id] else return null;
    }
};

const InterruptStatus = common.Bitflag(true, enum(u32) {
    used_buffer = 0,
    configuration_change = 1,
});
