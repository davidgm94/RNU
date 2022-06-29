const Region = @This();
const common = @import("../common.zig");
const PhysicalAddress = common.PhysicalAddress;
address: PhysicalAddress,
size: u64,

pub fn new(address: PhysicalAddress, size: u64) Region {
    return Region{
        .address = address,
        .size = size,
    };
}
