const kernel = @import("root");
const common = kernel.common;
const TODO = kernel.TODO;
const log = common.log.scoped(.Physical);

const MemoryMap = @import("memory_map.zig");
var bitset_memory: [0x400_000 + (10 * kernel.arch.page_size)]u8 align(kernel.arch.page_size) = undefined;
var bitset_byte_count: u64 = 0;

pub var available_regions: []Region = undefined;
pub var reserved_regions: []Region.Descriptor = undefined;
var _reserved: [64]Region.Descriptor = undefined;
var _available: [64]Region = undefined;

pub var kernel_region: *Region.Descriptor = undefined;
pub var device_tree_region: *Region.Descriptor = undefined;

pub const Region = struct {
    descriptor: Region.Descriptor,
    allocated_page_count: u64,
    bitset: []u64,

    pub const Descriptor = struct {
        address: u64,
        page_count: u64,
    };
};

pub fn init() void {
    const memory_map = MemoryMap.get();
    reserved_regions.ptr = &_reserved;
    reserved_regions.len = memory_map.reserved.len;
    kernel.copy(Region.Descriptor, reserved_regions, memory_map.reserved);

    const kernel_start = kernel.bounds.get_start();
    const kernel_end = kernel.bounds.get_end();
    const kernel_size = kernel_end - kernel_start;
    reserved_regions.len += 1;
    kernel_region = &reserved_regions[reserved_regions.len - 1];
    kernel_region.address = kernel_start;
    kernel_region.page_count = kernel_size / kernel.arch.page_size;
    kernel.assert(@src(), kernel_size & (kernel.arch.page_size - 1) == 0);

    reserved_regions.len += 1;
    device_tree_region = &reserved_regions[reserved_regions.len - 1];
    device_tree_region.address = kernel.arch.device_tree.base_address;
    device_tree_region.page_count = kernel.align_forward(kernel.arch.device_tree.header.size, kernel.arch.page_size) / kernel.arch.page_size;

    log.debug("Reserved regions", .{});
    for (reserved_regions) |reserved, i| {
        log.debug("[{}] (0x{x}, {})", .{ i, reserved.address, reserved.page_count });
    }

    available_regions.ptr = &_available;
    available_regions.len = memory_map.available.len;
    for (memory_map.available) |available, i| {
        available_regions[i].descriptor = available;
    }

    for (reserved_regions) |reserved| {
        const reserved_size = reserved.page_count * kernel.arch.page_size;
        for (available_regions) |*available, available_i| {
            const available_size = available.descriptor.page_count * kernel.arch.page_size;
            if (reserved.address >= available.descriptor.address and reserved.address < available.descriptor.address + available_size) {
                const start_matches = reserved.address == available.descriptor.address;
                const end_matches = reserved.address + reserved_size == available.descriptor.address + available_size;

                if (!start_matches and !end_matches) {
                    kernel.assert(@src(), available_i == available_regions.len - 1);
                    const first = available;
                    available_regions.len += 1;
                    const second = &available_regions[available_i + 1];
                    const original_size = available_size;
                    second.descriptor.address = reserved.address + reserved_size;
                    const original_end_address = first.descriptor.address + original_size;
                    second.descriptor.page_count = (original_end_address - second.descriptor.address) / kernel.arch.page_size;
                    first.descriptor.page_count -= second.descriptor.page_count + reserved.page_count;
                } else if (start_matches and end_matches) {
                    kernel.assert(@src(), available_i == available_regions.len - 1);
                    available_regions.len -= 1;
                } else if (start_matches) {
                    available.descriptor.address = reserved.address + reserved_size;
                    available.descriptor.page_count -= reserved.page_count;
                } else if (end_matches) {
                    TODO(@src());
                } else {
                    @panic("unreachableeEEEEE");
                }

                break;
            }
            // TODO: contemplate the case in which the reserved region is bigger than the available region
        }
    }

    log.debug("Available regions:", .{});
    for (available_regions) |region, i| {
        log.debug("[{}] (0x{x}, {} -- 0x{x})", .{ i, region.descriptor.address, region.descriptor.page_count, region.descriptor.page_count * kernel.arch.page_size });
    }

    for (available_regions) |*region| {
        // Align to u64
        const bitset_len = (region.descriptor.page_count / @bitSizeOf(u64)) + @boolToInt(region.descriptor.page_count % @bitSizeOf(u64) != 0);
        const bytes_to_allocate = bitset_len * @sizeOf(u64);
        bitset_byte_count = kernel.align_forward(bitset_byte_count, kernel.arch.page_size);
        region.bitset.ptr = @ptrCast([*]u64, @alignCast(kernel.arch.page_size, &bitset_memory[bitset_byte_count]));
        region.bitset.len = bytes_to_allocate / @sizeOf(u64);
        bitset_byte_count += bytes_to_allocate;
    }
}

pub fn allocate1(page_count: u64) ?u64 {
    const take_hint = true;
    // TODO: don't allocate if they are different regions (this can cause issues?)
    for (available_regions) |*region| {
        if (region.descriptor.page_count - region.allocated_page_count >= page_count) {
            const supposed_bitset_size = region.descriptor.page_count / @bitSizeOf(u64);
            kernel.assert(@src(), region.bitset.len >= supposed_bitset_size);
            var region_allocated_page_count: u64 = 0;

            const start_index = if (take_hint) region.allocated_page_count / @bitSizeOf(u64) else 0;
            var first_address: u64 = 0;

            bitset_loop: for (region.bitset[start_index..]) |*bitset_elem| {
                comptime var bit: u64 = 0;

                inline while (bit < @bitSizeOf(u64)) : (bit += 1) {
                    const bit_set = bitset_elem.* & (1 << bit) != 0;
                    if (region_allocated_page_count == page_count) {
                        break :bitset_loop;
                    } else {
                        if (!bit_set) {
                            if (first_address == 0) {
                                const offset = (bit + (start_index * @bitSizeOf(u64))) * kernel.arch.page_size;
                                first_address = region.descriptor.address + offset;
                            }

                            bitset_elem.* = bitset_elem.* | (1 << bit);
                            region_allocated_page_count += 1;
                        }
                    }
                }
            }

            if (region_allocated_page_count == page_count) {
                const result = first_address;
                region.allocated_page_count += region_allocated_page_count;
                kernel.assert(@src(), result != 0);
                return result;
            }

            log.debug("Asked page count: {}. Pages to deallocate: {}. Region page count: {}. Region already allocated: {}", .{ page_count, region_allocated_page_count, region.descriptor.page_count, region.allocated_page_count });

            kernel.assert(@src(), region.allocated_page_count + page_count > region.descriptor.page_count);
            kernel.assert(@src(), first_address != 0);
            const original_allocated_page_count = region.allocated_page_count - region_allocated_page_count;
            var byte = original_allocated_page_count / @bitSizeOf(u64);
            var bit = original_allocated_page_count % @bitSizeOf(u64);

            kernel.assert(@src(), region_allocated_page_count > 0);

            if (bit > 0) {
                while (bit < @bitSizeOf(u64)) : (bit += 1) {
                    region.bitset[byte] &= (~(@as(u64, 1) << @intCast(u6, bit)));
                    region_allocated_page_count -= 1;
                }
            }

            if (region_allocated_page_count >= 64) {
                TODO(@src());
            }

            if (region_allocated_page_count > 0) {
                TODO(@src());
            }

            region.allocated_page_count = original_allocated_page_count;
        }
    }

    @panic("allocation failed, no memory");
}
