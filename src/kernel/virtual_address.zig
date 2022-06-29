const VirtualAddress = @This();
const PhysicalAddress = @import("physical_address.zig");

value: u64,

pub inline fn new(value: u64) VirtualAddress {
    return VirtualAddress{
        .value = value,
    };
}

pub inline fn access(virtual_address: VirtualAddress, comptime Ptr: type) Ptr {
    return @intToPtr(Ptr, virtual_address.value);
}

pub inline fn identity_physical_address(virtual_address: VirtualAddress) PhysicalAddress {
    return PhysicalAddress.new(virtual_address.value);
}

pub inline fn page_up(virtual_address: *VirtualAddress, comptime page_size: u64) void {
    virtual_address.value += page_size;
}

pub inline fn page_align_backward(virtual_address: *VirtualAddress, comptime page_size: u64) void {
    virtual_address.value = common.align_backward(virtual_address.value, page_size);
}

pub inline fn page_align_forward(virtual_address: *VirtualAddress, comptime page_size: u64) void {
    virtual_address.value = common.align_forward(virtual_address.value, page_size);
}

pub inline fn is_page_aligned(virtual_address: VirtualAddress, comptime page_size: u64) bool {
    return common.is_aligned(virtual_address.value, page_size);
}

pub inline fn belongs_to_region(virtual_address: VirtualAddress, region: Virtual.Memory.Region) bool {
    return virtual_address.value >= region.address.value and virtual_address.value < region.address.value + region.size;
}

pub inline fn offset(virtual_address: VirtualAddress, asked_offset: u64) VirtualAddress {
    return VirtualAddress.new(virtual_address.value + asked_offset);
}
