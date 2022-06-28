const kernel = @import("root");
const common = kernel.common;
const log = common.log.scoped(.Virtio);
const TODO = kernel.TODO;
const PCI = @import("pci.zig");

fn next(caps: *u8, device: *PCI.Device) ?u8 {
    const current_caps = caps.*;
    if (current_caps == 0) return null;
    caps.* = device.read_config(u8, current_caps + 1);
    return current_caps;
}

const CapabilityOffset = enum(u8) {
    len = 2,
    cfg_type = 3,
    bar = 4,
    offset = 8,
    length = 12,
    notify_cap_mult = 16,
};

const ConfigurationType = enum(u8) {
    common = 1,
    notify = 2,
    isr = 3,
    device = 4,
    pci = 5,
};

const vendor_specific_capability = 0x09;

fn read_capability_data(comptime T: type, device: *PCI.Device, capability_offset: CapabilityOffset, caps: u8) T {
    return device.read_config(T, caps + @enumToInt(capability_offset));
}

pub fn init_from_pci(device: *PCI.Device) void {
    const driver = PCIDriver.detect_bar(device);
    _ = driver;
    TODO(@src());
}

const PCIDriver = struct {
    fn detect_bar(device: *PCI.Device) PCIDriver {
        var caps = device.read_capabilities_pointer() & 0xfc;

        while (next(&caps, device)) |cap| {
            const vendor_id = PCI.CommonHeader.read_from_offset("vendor_id", device.bus, device.slot, device.function, cap);
            log.debug("Vendor id: 0x{x}", .{vendor_id});
            log.debug("Caps: 0x{x}", .{cap});

            if (vendor_id == vendor_specific_capability) {
                const configuration_type = @intToEnum(ConfigurationType, read_capability_data(u8, device, .cfg_type, cap));
                const bar = read_capability_data(u8, device, .bar, cap);
                const offset = read_capability_data(u32, device, .offset, cap);
                const length = read_capability_data(u32, device, .length, cap);

                log.debug("Configuration type: {s}. BAR: 0x{x}. Offset: 0x{x}. Length: 0x{x}", .{ @tagName(configuration_type), bar, offset, length });
            }
        }

        TODO(@src());
    }
};
