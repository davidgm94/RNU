// TODO: batch PCI register access
const kernel = @import("../kernel/kernel.zig");
const log = kernel.log.scoped(.PCI);
const TODO = kernel.TODO;
const Controller = @This();
const x86_64 = @import("../kernel/arch/x86_64.zig");
const Physical = kernel.Physical;
const Virtual = kernel.Virtual;

devices: []Device,
bus_scan_states: [256]BusScanState,

pub var controller: Controller = undefined;

pub fn init() void {
    controller.enumerate();
}

const BusScanState = enum(u8) {
    do_not_scan = 0,
    scan_next = 1,
    scanned = 2,
};

pub const Bus = enum(u8) {
    _,
    inline fn new(value: u8) @This() {
        return @intToEnum(@This(), value);
    }

    inline fn inc(bus: *Bus) void {
        const current = @enumToInt(bus.*);
        if (current == 0xff) @panic("bus");
        bus.* = @intToEnum(Bus, current + 1);
    }
};

pub const Function = enum(u8) {
    _,
    inline fn new(value: u8) @This() {
        return @intToEnum(@This(), value);
    }

    inline fn inc(function: *Function) void {
        const current = @enumToInt(function.*);
        if (current == 0xff) @panic("function");
        function.* = @intToEnum(Function, current + 1);
    }
};

pub const Slot = enum(u8) {
    _,
    inline fn new(value: u8) Slot {
        return @intToEnum(Slot, value);
    }

    inline fn inc(slot: *Slot) void {
        const current = @enumToInt(slot.*);
        if (current == 0xff) @panic("slot");
        slot.* = @intToEnum(Slot, current + 1);
    }
};

const pci_read_config = kernel.arch.pci_read_config;
const pci_write_config = kernel.arch.pci_read_config;

fn Header(comptime HeaderT: type) type {
    return struct {
        fn get_type_from_field_name(comptime field_name: []const u8) type {
            comptime var header: HeaderT = undefined;
            return @TypeOf(@field(header, field_name));
        }

        pub fn read(comptime field_name: []const u8, bus: Bus, slot: Slot, function: Function) get_type_from_field_name(field_name) {
            const FieldType = get_type_from_field_name(field_name);
            return read_extended(FieldType, bus, slot, function, @offsetOf(HeaderT, field_name));
        }

        pub fn read_extended(comptime FieldType: type, bus: Bus, slot: Slot, function: Function, offset: u8) FieldType {
            return pci_read_config(FieldType, bus, slot, function, offset);
        }

        pub fn read_from_function(comptime field_name: []const u8, function: Function) get_type_from_field_name(field_name) {
            return read(field_name, Bus.new(0), Slot.new(0), function);
        }

        pub fn read_base(comptime field_name: []const u8) get_type_from_field_name(field_name) {
            return read(field_name, Bus.new(0), Slot.new(0), Function.new(0));
        }

        pub fn read_from_offset(comptime field_name: []const u8, bus: Bus, slot: Slot, function: Function, offset: u8) get_type_from_field_name(field_name) {
            const FieldType = get_type_from_field_name(field_name);
            return read_extended(FieldType, bus, slot, function, offset + @offsetOf(HeaderT, field_name));
        }

        pub fn get_offset(comptime field_name: []const u8) u64 {
            return @offsetOf(HeaderT, field_name);
        }
    };
}

pub const CommonHeader = Header(packed struct {
    vendor_id: u16,
    device_id: u16,
    command: u16,
    status: u16,
    prog_if: u8,
    revision_id: u8,
    subclass_code: u8,
    class_code: u8,
    cache_line_size: u8,
    latency_timer: u8,
    header_type: u8,
    bist: u8,

    comptime {
        kernel.assert_unsafe(@sizeOf(@This()) == 0x10);
    }
});

pub const HeaderType0x00 = Header(packed struct {
    vendor_id: u16,
    device_id: u16,
    command: u16,
    status: u16,
    prog_if: u8,
    revision_id: u8,
    subclass_code: u8,
    class_code: u8,
    cache_line_size: u8,
    latency_timer: u8,
    header_type: u8,
    bist: u8,
    // Start of specific header
    bar0: u32,
    bar1: u32,
    bar2: u32,
    bar3: u32,
    bar4: u32,
    bar5: u32,
    cardbus_cis_pointer: u32,
    subsystem_vendor_id: u16,
    subsystem_id: u16,
    expansion_rom_base_address: u32,
    capabilities_pointer: u8,
    reserved: u24,
    reserved2: u32,
    interrupt_line: u8,
    interrupt_pin: u8,
    min_grant: u8,
    max_latency: u8,

    comptime {
        kernel.assert_unsafe(@sizeOf(@This()) == 0x40);
    }
});

fn check_vendor(bus: Bus, slot: Slot, function: Function) bool {
    const vendor_id = CommonHeader.read("vendor_id", bus, slot, function);
    return vendor_id != kernel.max_int(u16);
}

const HeaderType = enum(u8) {
    x0 = 0,
    x1 = 1,
    x2 = 2,
};

fn enumerate(pci: *Controller) void {
    var base_function = Function.new(0);
    const base_header_type = CommonHeader.read_from_function("header_type", base_function);
    log.debug("Base header type: 0x{x}", .{base_header_type});
    kernel.assert(@src(), base_header_type == 0x0);
    const base_function_count: u8 = if (base_header_type & 0x80 != 0) 8 else 1;
    var buses_to_scan: u8 = 0;
    while (@enumToInt(base_function) < base_function_count) : (base_function.inc()) {
        if (!check_vendor(Bus.new(0), Slot.new(0), base_function)) continue;
        pci.bus_scan_states[@enumToInt(base_function)] = .scan_next;
        buses_to_scan += 1;
    }

    const original_bus_to_scan_count = buses_to_scan;

    if (buses_to_scan == 0) kernel.panic("unable to find any PCI bus", .{});

    base_function = Function.new(0);
    kernel.assert(@src(), @enumToInt(base_function) == 0);
    var device_count: u64 = 0;
    // First scan the buses to find out how many PCI devices the computer has
    while (buses_to_scan > 0) {
        var bus_i: u9 = 0;
        while (bus_i < 256) : (bus_i += 1) {
            const bus = Bus.new(@intCast(u8, bus_i));
            if (pci.bus_scan_states[@enumToInt(bus)] != .scan_next) continue;
            log.debug("Scanning bus {}...", .{@enumToInt(bus)});
            pci.bus_scan_states[@enumToInt(bus)] = .scanned;
            buses_to_scan -= 1;
            var device = Slot.new(0);

            while (@enumToInt(device) < 32) : (device.inc()) {
                if (!check_vendor(bus, device, Function.new(0))) continue;

                const header_type = CommonHeader.read("header_type", bus, device, base_function);
                const real_header_type = @intToEnum(HeaderType, header_type & 0x3f);
                log.debug("Header type: 0x{x}", .{@enumToInt(real_header_type)});
                const function_count: u8 = if (header_type & 0x80 != 0) 8 else 1;
                var function = Function.new(0);
                log.debug("Function count: {}", .{function_count});

                while (@enumToInt(function) < function_count) : (function.inc()) {
                    if (!check_vendor(bus, device, function)) continue;

                    device_count += 1;
                    const class_code = CommonHeader.read("class_code", bus, device, function);
                    const subclass_code = CommonHeader.read("subclass_code", bus, device, function);

                    if (class_code == 0x06 and subclass_code == 0x04) {
                        const secondary_bus = HeaderType0x00.read("bar1", bus, device, function);
                        buses_to_scan += 1;
                        pci.bus_scan_states[secondary_bus] = .scan_next;
                    }
                }
            }
        }
    }

    base_function = Function.new(0);
    while (@enumToInt(base_function) < base_function_count) : (base_function.inc()) {
        if (!check_vendor(Bus.new(0), Slot.new(0), base_function)) continue;
        pci.bus_scan_states[@enumToInt(base_function)] = .scan_next;
    }

    kernel.assert(@src(), device_count > 0);
    if (device_count > 0) {
        log.debug("Device count: {}", .{device_count});

        buses_to_scan = original_bus_to_scan_count;
        pci.devices = kernel.core_heap.allocate_many(Device, device_count) orelse @panic("unable to allocate pci devices");

        var registered_device_count: u64 = 0;

        log.debug("Buses to scan: {}", .{buses_to_scan});

        base_function = Function.new(0);
        kernel.assert(@src(), @enumToInt(base_function) == 0);

        while (buses_to_scan > 0) {
            var bus_i: u9 = 0;
            while (bus_i < 256) : (bus_i += 1) {
                const bus = Bus.new(@intCast(u8, bus_i));
                if (pci.bus_scan_states[@enumToInt(bus)] != .scan_next) continue;

                log.debug("Scanning bus {}...", .{bus});
                pci.bus_scan_states[@enumToInt(bus)] = .scanned;
                buses_to_scan -= 1;

                var device = Slot.new(0);

                while (@enumToInt(device) < 32) : (device.inc()) {
                    if (!check_vendor(bus, device, base_function)) continue;

                    const header_type = CommonHeader.read("header_type", bus, device, base_function);
                    const real_header_type = @intToEnum(HeaderType, header_type & 0x3f);
                    log.debug("Header type: 0x{x}", .{@enumToInt(real_header_type)});
                    const function_count: u8 = if (header_type & 0x80 != 0) 8 else 1;
                    var function = Function.new(0);
                    log.debug("Function count: {}", .{function_count});

                    while (@enumToInt(function) < function_count) : (function.inc()) {
                        if (!check_vendor(bus, device, function)) continue;

                        //const device_class = pci_read_config(u32, Bus.new(bus), Slot.new(device), Function.new(function), 0x08);
                        //log.debug("Device class: 0x{x}", .{device_class});
                        //const interrupt_information = pci_read_config(u32, Bus.new(bus), Slot.new(device), Function.new(function), 0x3c);
                        //log.debug("Interrupt information: 0x{x}", .{interrupt_information});

                        const pci_device = &pci.devices[registered_device_count];
                        registered_device_count += 1;

                        pci_device.class_code = CommonHeader.read("class_code", bus, device, function);
                        pci_device.subclass_code = CommonHeader.read("subclass_code", bus, device, function);
                        pci_device.prog_if = CommonHeader.read("prog_if", bus, device, function);

                        pci_device.bus = bus;
                        pci_device.slot = device;
                        pci_device.function = function;

                        pci_device.interrupt_pin = HeaderType0x00.read("interrupt_pin", bus, device, function);
                        pci_device.interrupt_line = HeaderType0x00.read("interrupt_line", bus, device, function);

                        pci_device.device_id = CommonHeader.read("device_id", bus, device, function);
                        pci_device.vendor_id = CommonHeader.read("vendor_id", bus, device, function);
                        log.debug("Device ID: 0x{x}. Vendor ID: 0x{x}", .{ pci_device.device_id, pci_device.vendor_id });
                        pci_device.subsystem_id = HeaderType0x00.read("subsystem_id", bus, device, function);
                        pci_device.subsystem_vendor_id = HeaderType0x00.read("subsystem_vendor_id", bus, device, function);

                        pci_device.base_addresses[0] = HeaderType0x00.read("bar0", bus, device, function);
                        pci_device.base_addresses[1] = HeaderType0x00.read("bar1", bus, device, function);
                        pci_device.base_addresses[2] = HeaderType0x00.read("bar2", bus, device, function);
                        pci_device.base_addresses[3] = HeaderType0x00.read("bar3", bus, device, function);
                        pci_device.base_addresses[4] = HeaderType0x00.read("bar4", bus, device, function);
                        pci_device.base_addresses[5] = HeaderType0x00.read("bar5", bus, device, function);

                        const class_code_name = if (pci_device.class_code < class_code_names.len) class_code_names[pci_device.class_code] else "Unknown";
                        const subclass_code_name = switch (pci_device.class_code) {
                            1 => if (pci_device.subclass_code < subclass1_code_names.len) subclass1_code_names[pci_device.subclass_code] else "",
                            12 => if (pci_device.subclass_code < subclass12_code_names.len) subclass12_code_names[pci_device.subclass_code] else "",
                            else => "",
                        };
                        const prog_if_name = if (pci_device.class_code == 12 and pci_device.subclass_code == 3 and pci_device.prog_if / 0x10 < prog_if_12_3_names.len) prog_if_12_3_names[pci_device.prog_if / 0x10] else "";
                        log.debug("PCI device. Class 0x{x} ({s}). Subclass: 0x{x} ({s}). Prog IF: {s}", .{ pci_device.class_code, class_code_name, pci_device.subclass_code, subclass_code_name, prog_if_name });
                    }
                }
            }
        }
    }
}

pub fn find_device(pci: *Controller, class_code: u8, subclass_code: u8) ?*Device {
    for (pci.devices) |*device| {
        if (device.class_code == class_code and device.subclass_code == subclass_code) {
            return device;
        }
    }

    return null;
}

// TODO: harden the search
pub fn find_virtio_device(pci: *Controller) ?*Device {
    for (pci.devices) |*device| {
        // TODO: better matching
        if (device.vendor_id == 0x1af4) {
            return device;
        }
    }

    return null;
}

// TODO: Report this Zig bug
//pub fn find_device_by_fields(pci: *Controller, comptime field_names: []const []const u8, comptime field_values: anytype) ?*Device {
//const field_values_unrolled = kernel.fields(@TypeOf(field_values));
//next_device: for (pci.devices) |*device| {
//inline for (field_names) |field_name, field_index| {
//const actual_field_value = @field(device, field_name);
//const field_value_struct_field = field_values_unrolled[field_index];
//_ = field_value_struct_field.default_value;

//const asked_field_value = @ptrCast(*align(1) const field_value_struct_field.field_type, field_value_struct_field.default_value.?).*;
//if (asked_field_value != actual_field_value) {
//continue :next_device;
//}
//}
//}

//return null;
//}

const class_code_names = [_][]const u8{
    "Unknown",
    "Mass storage controller",
    "Network controller",
    "Display controller",
    "Multimedia controller",
    "Memory controller",
    "Bridge controller",
    "Simple communication controller",
    "Base system peripheral",
    "Input device controller",
    "Docking station",
    "Processor",
    "Serial bus controller",
    "Wireless controller",
    "Intelligent controller",
    "Satellite communication controller",
    "Encryption controller",
    "Signal processing controller",
};

const subclass1_code_names = [_][]const u8{
    "SCSI bus controller",
    "IDE controller",
    "Floppy disk controller",
    "IPI bus controller",
    "RAID controller",
    "ATA controller",
    "Serial ATA",
    "Serial attached SCSI",
    "Non-volatile memory controller (NVMe)",
};

const subclass12_code_names = [_][]const u8{
    "FireWire (IEEE 1394) controller",
    "ACCESS bus",
    "SSA",
    "USB controller",
    "Fibre channel",
    "SMBus",
    "InfiniBand",
    "IPMI interface",
    "SERCOS interface (IEC 61491)",
    "CANbus",
};

const prog_if_12_3_names = [_][]const u8{
    "UHCI",
    "OHCI",
    "EHCI",
    "XHCI",
};

pub const Device = struct {
    device_id: u16,
    vendor_id: u16,
    subsystem_id: u16,
    subsystem_vendor_id: u16,
    domain: u32,
    class_code: u8,
    subclass_code: u8,
    prog_if: u8,
    bus: Bus,
    slot: Slot,
    function: Function,
    interrupt_pin: u8,
    interrupt_line: u8,

    base_virtual_addresses: [6]kernel.Virtual.Address,
    base_physical_addresses: [6]kernel.Physical.Address,
    base_addresses_size: [6]u64,
    base_addresses: [6]u32,

    //uint8_t  *baseAddressesVirtual[6];
    //uintptr_t baseAddressesPhysical[6];
    //size_t    baseAddressesSizes[6];

    //uint32_t baseAddresses[6];

    pub inline fn read_config(device: *Device, comptime T: type, offset: u8) T {
        return kernel.arch.pci_read_config(T, device.bus, device.slot, device.function, offset);
    }

    pub inline fn write_config(device: *Device, comptime T: type, value: T, offset: u8) void {
        return kernel.arch.pci_write_config(T, value, device.bus, device.slot, device.function, offset);
    }

    pub inline fn read_bar(device: *Device, comptime T: type, index: u64, offset: u64) T {
        const base_address = device.base_addresses[index];
        if (T != u64) {
            if (base_address & 1 != 0) {
                const port = @intCast(u16, (base_address & ~@as(u32, 3)) + offset);
                return kernel.arch.io_read(T, port);
            } else {
                return device.base_virtual_addresses[index].offset(offset).access(*volatile T).*;
            }
        } else {
            if (base_address & 1 != 0) {
                return device.read_bar(u32, index, offset) | (@intCast(u64, device.read_bar(u64, index, offset + @sizeOf(u32))) << 32);
            } else {
                return device.base_virtual_addresses[index].offset(offset).access(*volatile T).*;
            }
        }
    }

    pub inline fn write_bar(device: *Device, comptime T: type, index: u64, offset: u64, value: T) void {
        const base_address = device.base_addresses[index];
        if (T != u64) {
            if (base_address & 1 != 0) {
                const port = @intCast(u16, (base_address & ~@as(@TypeOf(base_address), 3)) + offset);
                kernel.arch.io_write(T, port, value);
            } else {
                const virtual_address = device.base_virtual_addresses[index].offset(offset);
                virtual_address.access(*volatile T).* = value;
            }
        } else {
            if (base_address & 1 != 0) {
                device.write_bar(u32, index, offset, @truncate(u32, value));
                device.write_bar(u32, index, offset + @sizeOf(u32), @truncate(u32, value >> 32));
            } else {
                device.base_virtual_addresses[index].offset(offset).access(*volatile T).* = value;
            }
        }
    }

    pub const Features = kernel.Bitflag(false, enum(u64) {
        bar0 = 0,
        bar1 = 1,
        bar2 = 2,
        bar3 = 3,
        bar4 = 4,
        bar5 = 5,
        interrupts = 8,
        busmastering_dma = 9,
        memory_space_access = 10,
        io_port_access = 11,
    });

    pub fn enable_features(device: *Device, features: Features) bool {
        log.debug("Enabling features for device {}", .{device});
        var config = device.read_config(u32, 4);
        if (features.contains(.interrupts)) config &= ~@as(u32, 1 << 10);
        if (features.contains(.busmastering_dma)) config |= 1 << 2;
        if (features.contains(.memory_space_access)) config |= 1 << 1;
        if (features.contains(.io_port_access)) config |= 1 << 0;
        log.debug("Writing config: 0x{x}", .{config});
        device.write_config(u32, config, 4);

        if (device.read_config(u32, 4) != config) {
            return false;
        }

        for (device.base_addresses) |*base_address_ptr, i| {
            if (~features.bits & (@as(u64, 1) << @intCast(u3, i)) != 0) continue;
            const base_address = base_address_ptr.*;
            if (base_address & 1 != 0) continue; // BAR is an IO port
            log.debug("Actually setting up base address #{}", .{i});

            if (base_address & 0b1000 == 0) {
                // TODO: not prefetchable
            }

            const is_size_64 = base_address & 0b100 != 0;
            log.debug("is size 64: {}", .{is_size_64});

            var address: u64 = 0;
            var size: u64 = 0;

            if (is_size_64) {
                device.write_config(u32, kernel.max_int(u32), 0x10 + 4 * @intCast(u8, i));
                device.write_config(u32, kernel.max_int(u32), 0x10 + 4 * @intCast(u8, i + 1));
                size = device.read_config(u32, 0x10 + 4 * @intCast(u8, i));
                size |= @intCast(u64, device.read_config(u32, 0x10 + 4 * @intCast(u8, i + 1))) << 32;
                device.write_config(u32, base_address, 0x10 + 4 * @intCast(u8, i));
                device.write_config(u32, device.base_addresses[i + 1], 0x10 + 4 * @intCast(u8, i + 1));
                address = base_address;
                address |= @intCast(u64, device.base_addresses[i + 1]) << 32;
            } else {
                device.write_config(u32, kernel.max_int(u32), 0x10 + 4 * @intCast(u8, i));
                size = device.read_config(u32, 0x10 + 4 * @intCast(u8, i));
                size |= @as(u64, kernel.max_int(u32)) << 32;
                device.write_config(u32, base_address, 0x10 + 4 * @intCast(u8, i));
                address = base_address;
            }

            if (size == 0 or address == 0) return false;
            log.debug("Address: 0x{x}. Size: {}", .{ address, size });
            size &= ~@as(u64, 0xf);
            size = ~size + 1;
            address &= ~@as(u64, 0xf);
            log.debug("Address: 0x{x}. Size: {}", .{ address, size });

            device.base_physical_addresses[i] = kernel.Physical.Address.new(address);
            device.base_virtual_addresses[i] = device.base_physical_addresses[i].to_higher_half_virtual_address();
            const physical_region = kernel.Physical.Memory.Region.new(device.base_physical_addresses[i], size);
            physical_region.map(&kernel.address_space, device.base_virtual_addresses[i], kernel.Virtual.AddressSpace.Flags.from_flags(&.{ .cache_disable, .read_write }));

            log.debug("Virtual 0x{x}. Physical 0x{x}", .{ device.base_virtual_addresses[i].value, device.base_physical_addresses[i].value });
            device.base_addresses_size[i] = size;
        }

        return true;
    }

    pub fn enable_single_interrupt(device: *Device, handler: x86_64.interrupts.HandlerInfo) bool {
        if (device.enable_MSI(handler)) return true;
        if (device.interrupt_pin == 0) return false;
        if (device.interrupt_pin > 4) return false;

        const result = device.enable_features(Features.from_flag(.interrupts));
        kernel.assert(@src(), result);

        // TODO: consider some stuff Essence does?
        const interrupt_line: ?u64 = null;

        if (handler.register_IRQ(interrupt_line, device)) {
            return true;
        }

        TODO(@src());
    }

    pub fn enable_MSI(device: *Device, handler: x86_64.interrupts.HandlerInfo) bool {
        _ = handler;
        const status = device.read_config(u32, 0x04) >> 16;

        if (~status & (1 << 4) != 0) return false;

        var pointer = device.read_config(u8, 0x34);
        var index: u64 = 0;

        while (true) {
            if (pointer == 0) break;
            if (index >= 0xff) break;
            index += 1;

            const dw = device.read_config(u32, pointer);
            const next_pointer = @truncate(u8, dw >> 8);
            const id = @truncate(u8, dw);

            if (id != 5) {
                pointer = next_pointer;
                continue;
            }

            // TODO: maybe this is a bug.NVMe should support MSI
            TODO(@src());
            //const msi =
        }

        return false;
    }

    pub fn read_capabilities_pointer(device: *Device) u8 {
        return HeaderType0x00.read("capabilities_pointer", device.bus, device.slot, device.function);
    }

    //pub fn read_bar(device: *Device, bar_index: u8) u32 {
    //}

    //pub fn bar_info(device: *Device, bar_index: u8) Physical.Memory.Region {

    //}
};
