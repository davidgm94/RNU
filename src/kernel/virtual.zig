const kernel = @import("root");
const common = kernel.common;
const Virtual = @This();
const Physical = kernel.Physical;
const log = common.log.scoped(.Virtual);
pub const Memory = @import("virtual_memory.zig");

pub var initialized = false;

pub const AddressSpace = struct {
    arch: kernel.arch.AddressSpace,
    free_regions_by_address: kernel.AVL.Tree(Virtual.Memory.Region) = .{},
    free_regions_by_size: kernel.AVL.Tree(Virtual.Memory.Region) = .{},
    used_regions: kernel.AVL.Tree(Virtual.Memory.Region) = .{},

    pub inline fn new() ?AddressSpace {
        return from_context(kernel.arch.AddressSpace.new() orelse return null);
    }

    pub inline fn from_current() ?AddressSpace {
        return from_context(kernel.arch.AddressSpace.from_current());
    }

    pub inline fn from_context(context: anytype) AddressSpace {
        return AddressSpace{
            .arch = context,
        };
    }

    // TODO: handle free in error case
    pub inline fn new_for_user() ?*AddressSpace {
        const address_space = kernel.core_heap.allocator.create(AddressSpace) catch return null;
        address_space.* = new() orelse return null;
        address_space.arch.map_kernel_address_space_higher_half();
        return address_space;
    }

    // TODO: manage virtual memory
    pub inline fn allocate(address_space: *AddressSpace, size: u64) ?Virtual.Address {
        _ = address_space;
        const page_count = kernel.bytes_to_pages(size, .must_be_exact);
        const physical_address = kernel.Physical.Memory.allocate_pages(page_count) orelse return null;
        return physical_address.to_higher_half_virtual_address();
    }

    pub inline fn allocate_and_map(address_space: *AddressSpace, size: u64, flags: Flags) ?Virtual.Address {
        _ = address_space;
        const page_count = kernel.bytes_to_pages(size, .must_be_exact);
        const physical_address = kernel.Physical.Memory.allocate_pages(page_count) orelse return null;
        const virtual_address = physical_address.to_higher_half_virtual_address();

        const physical_region = kernel.Physical.Memory.Region.new(physical_address, size);
        physical_region.map(address_space, virtual_address, flags);

        return virtual_address;
    }

    pub inline fn translate_address(address_space: *AddressSpace, virtual_address: Virtual.Address) ?Physical.Address {
        return address_space.arch.translate_address(virtual_address);
    }

    pub inline fn map(address_space: *AddressSpace, physical_address: Physical.Address, virtual_address: Virtual.Address, flags: Flags) void {
        address_space.arch.map(physical_address, virtual_address, flags);
        const checked_physical_address = address_space.translate_address(virtual_address) orelse @panic("mapping failed");
        kernel.assert(@src(), checked_physical_address.value == physical_address.value);
    }

    pub const Flags = kernel.Bitflag(true, enum(u64) {
        read_write = 1,
        user = 2,
        write_through = 3,
        cache_disable = 4,
        accessed = 5,
        dirty = 6,
        pat = 7, // must be 0
        global = 8,
        execute_disable = 63,
    });

    pub inline fn make_current(address_space: *AddressSpace) void {
        address_space.arch.make_current();
    }

    const IntegrationError = error{
        region_allocation_failed,
    };

    pub fn integrate_mapped_physical_entry(address_space: *AddressSpace, entry: kernel.Physical.Memory.Map.Entry, base_virtual_address: Virtual.Address) !void {
        kernel.assert(@src(), entry.descriptor.size != 0);
        //log.debug("Integrating (0x{x}, {}) into 0x{x}", .{ entry.descriptor.address.value, entry.descriptor.size, base_virtual_address.value });

        if (entry.descriptor.size != entry.allocated_size) {
            const free_region_offset = base_virtual_address.value + entry.allocated_size;
            const free_region_size = entry.descriptor.size - entry.allocated_size;
            //log.debug("Allocating free region", .{});
            const free_region = kernel.core_heap.allocator.create(Virtual.Memory.Region) catch return IntegrationError.region_allocation_failed;
            //log.debug("Allocated free region", .{});
            free_region.* = Virtual.Memory.Region.new(Virtual.Address.new(free_region_offset), free_region_size);
            //log.debug("Free region: (0x{x}, {})", .{ free_region.address.value, free_region.size });

            var result = address_space.free_regions_by_address.insert(&free_region.item_address, free_region, free_region.address.value, .panic);
            kernel.assert(@src(), result);
            result = address_space.free_regions_by_size.insert(&free_region.item_size, free_region, free_region.size, .allow);
            kernel.assert(@src(), result);
        }

        if (entry.allocated_size > 0) {
            //log.debug("allocating region", .{});
            const used_region = kernel.core_heap.allocator.create(Virtual.Memory.Region) catch return IntegrationError.region_allocation_failed;
            //log.debug("allocated region", .{});
            used_region.* = Virtual.Memory.Region.new(base_virtual_address, entry.allocated_size);
            used_region.used = true;
            //log.debug("Used region: (0x{x}, {})", .{ used_region.address.value, used_region.size });

            kernel.assert(@src(), used_region.address.value != 0);
            const result = address_space.used_regions.insert(&used_region.item_address, used_region, used_region.address.value, .panic);
            kernel.assert(@src(), result);
        }
    }

    pub fn integrate_mapped_physical_region(address_space: *AddressSpace, region: kernel.Physical.Memory.Region, base_virtual_address: Virtual.Address) !void {
        kernel.assert(@src(), region.size != 0);
        log.debug("region size: {}", .{region.size});

        const used_region = kernel.core_heap.allocator.create(Virtual.Memory.Region) catch return IntegrationError.region_allocation_failed;
        used_region.* = Virtual.Memory.Region.new(base_virtual_address, region.size);
        used_region.used = true;
        log.debug("Used region: (0x{x}, {})", .{ used_region.address.value, used_region.size });

        kernel.assert(@src(), used_region.address.value != 0);
        const result = address_space.used_regions.insert(&used_region.item_address, used_region, used_region.address.value, .panic);
        kernel.assert(@src(), result);
    }
};
