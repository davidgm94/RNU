const kernel = @import("root");
const common = kernel.common;
const VirtualAddress = common.VirtualAddress;
const Virtual = kernel.Virtual;

const Physical = kernel.Physical;

pub const Region = struct {
    address: VirtualAddress,
    size: u64,

    item_address: kernel.AVL.Tree(Region).Item,
    item_size: kernel.AVL.Tree(Region).Item,
    used: bool,

    pub fn new(address: VirtualAddress, size: u64) Region {
        return Region{
            .address = address,
            .size = size,
            .item_address = kernel.AVL.Tree(Region).Item{},
            .item_size = kernel.AVL.Tree(Region).Item{},
            .used = false,
        };
    }

    pub fn map(region: Region, address_space: *Virtual.AddressSpace, base_physical_address: Physical.Address, flags: kernel.Virtual.AddressSpace.Flags) void {
        var physical_address = base_physical_address;
        var virtual_address = region.address;
        var size_it: u64 = 0;
        while (size_it < region.size) : (size_it += kernel.arch.page_size) {
            address_space.arch.map(physical_address, virtual_address, flags);
            physical_address.page_up();
            virtual_address.page_up();
        }
    }
};

pub const RegionWithPermissions = struct {
    descriptor: Region,
    read: bool,
    write: bool,
    execute: bool,
};
