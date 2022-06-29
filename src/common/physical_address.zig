const common = @import("../common.zig");
const PhysicalAddress = @This();
const VirtualAddress = common.VirtualAddress;

value: u64,

pub var max: u64 = 0;
pub var max_bit: u6 = 0;

pub inline fn new(value: u64) PhysicalAddress {
    const physical_address = PhysicalAddress{
        .value = value,
    };

    if (!physical_address.is_valid()) {
        common.panic(@src(), "physical address 0x{x} is invalid", .{physical_address.value});
    }

    return physical_address;
}

pub inline fn temporary_invalid() PhysicalAddress {
    return maybe_invalid(0);
}

pub inline fn maybe_invalid(value: u64) PhysicalAddress {
    return PhysicalAddress{
        .value = value,
    };
}

pub inline fn is_valid(physical_address: PhysicalAddress) bool {
    common.runtime_assert(@src(), physical_address.value != 0);
    common.runtime_assert(@src(), max_bit != 0);
    common.runtime_assert(@src(), max > 1000);
    return physical_address.value <= max;
}

pub inline fn page_up(physical_address: *PhysicalAddress, comptime page_size: u64) void {
    common.runtime_assert(@src(), physical_address.is_page_aligned(page_size));
    physical_address.value += page_size;
}

pub inline fn page_down(physical_address: *PhysicalAddress, comptime page_size: u64) void {
    common.runtime_assert(@src(), physical_address.is_page_aligned(page_size));
    physical_address.value -= page_size;
}

pub inline fn page_align_forward(physical_address: *PhysicalAddress, comptime page_size: u64) void {
    physical_address.value = common.align_forward(physical_address.value, page_size);
}

pub inline fn page_align_backward(physical_address: *PhysicalAddress, comptime page_size: u64) void {
    physical_address.value = common.align_backward(physical_address.value, page_size);
}

pub inline fn is_page_aligned(physical_address: PhysicalAddress, comptime page_size: u64) bool {
    return common.is_aligned(physical_address.value, page_size);
}

pub inline fn belongs_to_region(physical_address: PhysicalAddress, region: common.PhysicalMemoryRegion) bool {
    return physical_address.value >= region.address.value and physical_address.value < region.address.value + region.size;
}

pub inline fn offset(physical_address: PhysicalAddress, asked_offset: u64) PhysicalAddress {
    return PhysicalAddress.new(physical_address.value + asked_offset);
}

pub inline fn identity_virtual_address(physical_address: PhysicalAddress) VirtualAddress {
    return physical_address.identity_virtual_address_extended(false);
}

pub inline fn identity_virtual_address_extended(physical_address: PhysicalAddress, comptime override: bool) VirtualAddress {
    const root = @import("root");
    if (@hasDecl(root, "Virtual")) {
        if (!override and root.Virtual.initialized) common.TODO(@src());
    }
    return VirtualAddress.new(physical_address.value);
}

pub inline fn access_identity(physical_address: PhysicalAddress, comptime Ptr: type) Ptr {
    const root = @import("root");
    if (@hasDecl(root, "Virtual")) {
        common.runtime_assert(@src(), !root.Virtual.initialized);
    }

    return @intToPtr(Ptr, identity_virtual_address(physical_address).value);
}

pub inline fn access(physical_address: PhysicalAddress, comptime Ptr: type) Ptr {
    const root = @import("root");
    const initialized_virtual = @hasDecl(root, "Virtual") and root.Virtual.initialized;
    return if (initialized_virtual) physical_address.access_higher_half(Ptr) else physical_address.access_identity(Ptr);
}

pub inline fn to_higher_half_virtual_address(physical_address: PhysicalAddress) VirtualAddress {
    const root = @import("root");
    var higher_half: u64 = 0;
    if (@hasDecl(root, "higher_half_direct_map")) {
        higher_half = root.higher_half_direct_map.value;
    }
    return VirtualAddress.new(physical_address.value + higher_half);
}

pub inline fn access_higher_half(physical_address: PhysicalAddress, comptime Ptr: type) Ptr {
    return @intToPtr(Ptr, physical_address.to_higher_half_virtual_address().value);
}
