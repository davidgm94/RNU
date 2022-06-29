const kernel = @import("root");
const common = kernel.common;
const Physical = kernel.Physical;
const Virtual = kernel.Virtual;
const log = common.log.scoped(.PhysicalMemory);
const TODO = kernel.TODO;
const VirtualAddress = common.VirtualAddress;
const PhysicalAddress = common.PhysicalAddress;
pub var memory_map: Map = undefined;
pub const Region = common.PhysicalMemoryRegion;

/// This contains physical memory regions
pub const Map = struct {
    usable: []Entry,
    reclaimable: []Entry,
    framebuffer: []Region,
    kernel_and_modules: []Region,
    reserved: []Region,

    pub const RegionType = enum(u3) {
        usable = 0,
        reclaimable = 1,
        framebuffer = 2,
        kernel_and_modules = 3,
        reserved = 4,
    };

    pub fn get_used_memory(mmap: *Map) u64 {
        var allocated_memory: u64 = 0;
        for (mmap.usable) |region| {
            allocated_memory += region.allocated_size;
        }

        return allocated_memory;
    }

    pub fn find_address(mmap: *Map, physical_address: PhysicalAddress) ?RegionType {
        for (mmap.usable) |region| {
            if (physical_address.belongs_to_region(region.descriptor)) {
                return .usable;
            }
        }
        for (mmap.reclaimable) |region| {
            if (physical_address.belongs_to_region(region.descriptor)) {
                return .reclaimable;
            }
        }
        for (mmap.framebuffer) |region| {
            if (physical_address.belongs_to_region(region)) {
                return .framebuffer;
            }
        }
        for (mmap.kernel_and_modules) |region| {
            if (physical_address.belongs_to_region(region)) {
                return .kernel_and_modules;
            }
        }
        for (mmap.reserved) |region| {
            if (physical_address.belongs_to_region(region)) {
                return .reserved;
            }
        }

        return null;
    }

    pub const Entry = struct {
        descriptor: Region,
        allocated_size: u64,
        type: Type,

        pub const BitsetBaseType = u64;

        pub const Type = enum(u64) {
            usable = 0,
            reclaimable = 1,
            framebuffer = 2,
            kernel_and_modules = 3,
            reserved = 4,
        };

        pub fn get_bitset(entry: *Entry) []BitsetBaseType {
            return get_bitset_from_address_and_size(entry.descriptor.address, entry.descriptor.size);
        }

        pub fn get_bitset_from_address_and_size(address: PhysicalAddress, size: u64) []BitsetBaseType {
            const page_count = kernel.bytes_to_pages(size, .must_be_exact);
            const bitset_len = common.remainder_division_maybe_exact(page_count, @bitSizeOf(BitsetBaseType), .can_be_not_exact);
            return if (kernel.Virtual.initialized) address.access_higher_half([*]BitsetBaseType)[0..bitset_len] else address.access_identity([*]BitsetBaseType)[0..bitset_len];
        }

        pub fn setup_bitset(entry: *Entry) void {
            log.debug("Setting up bitset", .{});
            const page_count = kernel.bytes_to_pages(entry.allocated_size, .must_be_exact);
            log.debug("Set up bitset", .{});
            const bitsize = @bitSizeOf(Map.Entry.BitsetBaseType);
            const quotient = page_count / bitsize;
            const remainder_bitsize_max: u64 = bitsize - 1;
            const popcount = @popCount(@TypeOf(remainder_bitsize_max), remainder_bitsize_max);
            const remainder = @intCast(common.IntType(.unsigned, popcount), page_count % bitsize);

            const bitset = entry.get_bitset();

            for (bitset[0..quotient]) |*bitset_elem| {
                bitset_elem.* = common.max_int(Map.Entry.BitsetBaseType);
            }

            var remainder_i: @TypeOf(remainder) = 0;
            while (remainder_i < remainder) : (remainder_i += 1) {
                bitset[quotient] |= @as(u64, 1) << remainder_i;
            }
        }

        fn debug(entry: *Entry) void {
            log.debug("(0x{x},\t{})", .{ entry.descriptor.address, entry.descriptor.size });
        }
    };

    pub fn debug(mmap: *Map) void {
        log.debug("Usable", .{});
        for (mmap.usable) |region| {
            log.debug("(0x{x},\t0x{x},\t{})", .{ region.address, region.address + region.size, region.size });
        }
        log.debug("Reclaimable", .{});
        for (mmap.reclaimable) |region| {
            log.debug("(0x{x},\t0x{x},\t{})", .{ region.address, region.address + region.size, region.size });
        }
        log.debug("Framebuffer", .{});
        for (mmap.framebuffer) |region| {
            log.debug("(0x{x},\t0x{x},\t{})", .{ region.address, region.address + region.size, region.size });
        }
        log.debug("Kernel and modules", .{});
        for (mmap.kernel_and_modules) |region| {
            log.debug("(0x{x},\t0x{x},\t{})", .{ region.address, region.address + region.size, region.size });
        }
        log.debug("Reserved", .{});
        for (mmap.reserved) |region| {
            log.debug("(0x{x},\t0x{x},\t{})", .{ region.address, region.address + region.size, region.size });
        }
    }
};

pub fn allocate_pages(page_count: u64) ?PhysicalAddress {
    const take_hint = true;
    const size = page_count * kernel.arch.page_size;
    // TODO: don't allocate if they are different regions (this can cause issues?)
    for (memory_map.usable) |*region| {
        if (region.descriptor.size - region.allocated_size >= size) {
            const region_page_count = region.descriptor.size / kernel.arch.page_size;
            const supposed_bitset_size = region_page_count / @bitSizeOf(Map.Entry.BitsetBaseType);
            const bitset = region.get_bitset();
            common.runtime_assert(@src(), bitset.len >= supposed_bitset_size);
            var region_allocated_page_count: u64 = 0;
            const allocated_page_count = region.allocated_size / kernel.arch.page_size;

            const start_index = if (take_hint) allocated_page_count / @bitSizeOf(u64) else 0;
            var first_address: u64 = 0;

            bitset_loop: for (bitset[start_index..]) |*bitset_elem| {
                comptime var bit: u64 = 0;

                inline while (bit < @bitSizeOf(u64)) : (bit += 1) {
                    const bit_set = bitset_elem.* & (1 << bit) != 0;
                    if (region_allocated_page_count == page_count) {
                        break :bitset_loop;
                    } else {
                        if (!bit_set) {
                            if (first_address == 0) {
                                const offset = (bit + (start_index * @bitSizeOf(u64))) * kernel.arch.page_size;
                                first_address = region.descriptor.address.value + offset;
                            }

                            bitset_elem.* = bitset_elem.* | (1 << bit);
                            region_allocated_page_count += 1;
                        }
                    }
                }
            }

            if (region_allocated_page_count == page_count) {
                const result = first_address;
                region.allocated_size += region_allocated_page_count * kernel.arch.page_size;
                common.runtime_assert(@src(), result != 0);
                return PhysicalAddress.new(result);
            }

            common.runtime_assert(@src(), region.allocated_size + size > region.descriptor.size);
            common.runtime_assert(@src(), first_address != 0);
            const original_allocated_size = region.allocated_size - (region_allocated_page_count * kernel.arch.page_size);
            const original_allocated_page_count = original_allocated_size / kernel.arch.page_size;
            var byte = original_allocated_page_count / @bitSizeOf(u64);
            var bit = original_allocated_page_count % @bitSizeOf(u64);

            common.runtime_assert(@src(), region_allocated_page_count > 0);

            if (bit > 0) {
                while (bit < @bitSizeOf(u64)) : (bit += 1) {
                    bitset[byte] &= (~(@as(u64, 1) << @intCast(u6, bit)));
                    region_allocated_page_count -= 1;
                }
            }

            if (region_allocated_page_count >= 64) {
                TODO(@src());
            }

            if (region_allocated_page_count > 0) {
                TODO(@src());
            }

            region.allocated_size = original_allocated_size;
        }
    }

    @panic("allocation failed, no memory");
}

pub fn map(region: Region, address_space: *Virtual.AddressSpace, base_virtual_address: VirtualAddress, flags: kernel.Virtual.AddressSpace.Flags) void {
    return map_extended(region, address_space, base_virtual_address, true, flags);
}

pub fn map_extended(region: Region, address_space: *Virtual.AddressSpace, base_virtual_address: VirtualAddress, comptime is_page_aligned: bool, flags: kernel.Virtual.AddressSpace.Flags) void {
    var physical_address = region.address;
    var virtual_address = base_virtual_address;
    var region_size = region.size;
    if (!is_page_aligned) {
        physical_address.page_align_backward();
        virtual_address.page_align_backward();
        region_size = common.align_forward(region_size, kernel.arch.page_size);
    }
    log.debug("Mapping (0x{x}, 0x{x}) to (0x{x}, 0x{x})", .{ physical_address.value, physical_address.value + region_size, virtual_address.value, virtual_address.value + region_size });
    var size_it: u64 = 0;
    while (size_it < region_size) : (size_it += kernel.arch.page_size) {
        address_space.arch.map(physical_address, virtual_address, flags);
        physical_address.page_up(kernel.arch.page_size);
        virtual_address.page_up(kernel.arch.page_size);
    }
}
