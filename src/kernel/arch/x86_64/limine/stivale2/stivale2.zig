const kernel = @import("root");
const common = kernel.common;
const stivale = @import("header.zig");
const log = common.log.scoped(.stivale);
const assert = common.runtime_assert;
const x86_64 = @import("../../../x86_64.zig");

const VirtualAddress = common.VirtualAddress;
const PhysicalAddress = common.PhysicalAddress;

pub const Struct = stivale.Struct;

pub const Error = error{
    memory_map,
    higher_half_direct_map,
    kernel_file,
    pmrs,
    rsdp,
    smp,
};

pub fn process_bootloader_information(stivale2_struct: *Struct) Error!void {
    kernel.sections_in_memory = try process_pmrs(stivale2_struct);
    log.debug("Process sections in memory", .{});
    kernel.file = try process_kernel_file(stivale2_struct);
    log.debug("Process kernel file in memory", .{});
    kernel.cpus = try process_smp(stivale2_struct);
    log.debug("Process SMP info", .{});
}

pub fn find(comptime StructT: type, stivale2_struct: *Struct) ?*align(1) StructT {
    var tag_opt = get_tag_from_physical(PhysicalAddress.new(stivale2_struct.tags));

    while (tag_opt) |tag| {
        if (tag.identifier == StructT.id) {
            return @ptrCast(*align(1) StructT, tag);
        }

        tag_opt = get_tag_from_physical(PhysicalAddress.new(tag.next));
    }

    return null;
}

fn get_tag_from_physical(physical_address: PhysicalAddress) ?*align(1) stivale.Tag {
    return if (kernel.Virtual.initialized) physical_address.access_higher_half(?*align(1) stivale.Tag) else physical_address.access_identity(?*align(1) stivale.Tag);
}

pub fn process_memory_map(stivale2_struct: *Struct) Error!kernel.Physical.Memory.Map {
    const memory_map_struct = find(Struct.MemoryMap, stivale2_struct) orelse return Error.memory_map;
    const memory_map_entries = memory_map_struct.memmap()[0..memory_map_struct.entry_count];
    var result = kernel.Physical.Memory.Map{
        .usable = &[_]kernel.Physical.Memory.Map.Entry{},
        .reclaimable = &[_]kernel.Physical.Memory.Map.Entry{},
        .framebuffer = &[_]kernel.Physical.Memory.Region{},
        .kernel_and_modules = &[_]kernel.Physical.Memory.Region{},
        .reserved = &[_]kernel.Physical.Memory.Region{},
    };

    // First, it is required to find a spot in memory big enough to host all the memory map entries in a architecture-independent and bootloader-independent way. This is the host entry
    const host_entry = blk: {
        for (memory_map_entries) |*entry| {
            if (entry.type == .usable) {
                const bitset = kernel.Physical.Memory.Map.Entry.get_bitset_from_address_and_size(PhysicalAddress.new(entry.address), entry.size);
                const bitset_size = bitset.len * @sizeOf(kernel.Physical.Memory.Map.Entry.BitsetBaseType);
                // INFO: this is separated since the bitset needs to be in a different page than the memory map
                const bitset_page_count = kernel.bytes_to_pages(bitset_size, .can_be_not_exact);
                // Allocate a bit more memory than needed just in case
                const memory_map_allocation_size = memory_map_struct.entry_count * @sizeOf(kernel.Physical.Memory.Map.Entry);
                const memory_map_page_count = kernel.bytes_to_pages(memory_map_allocation_size, .can_be_not_exact);
                const total_allocated_page_count = bitset_page_count + memory_map_page_count;
                const total_allocation_size = kernel.arch.page_size * total_allocated_page_count;
                common.runtime_assert(@src(), entry.size > total_allocation_size);
                result.usable = @intToPtr([*]kernel.Physical.Memory.Map.Entry, entry.address + common.align_forward(bitset_size, kernel.arch.page_size))[0..1];
                var block = &result.usable[0];
                block.* = kernel.Physical.Memory.Map.Entry{
                    .descriptor = kernel.Physical.Memory.Region{
                        .address = PhysicalAddress.new(entry.address),
                        .size = entry.size,
                    },
                    .allocated_size = total_allocation_size,
                    .type = .usable,
                };

                block.setup_bitset();

                break :blk block;
            }
        }

        @panic("There is no memory map entry big enough to store the memory map entries");
    };

    // The counter starts with one because we have already filled the memory map with the host entry
    for (memory_map_entries) |*entry| {
        if (entry.type == .usable) {
            if (entry.address == host_entry.descriptor.address.value) continue;

            const index = result.usable.len;
            result.usable.len += 1;
            var result_entry = &result.usable[index];
            result_entry.* = kernel.Physical.Memory.Map.Entry{
                .descriptor = kernel.Physical.Memory.Region{
                    .address = PhysicalAddress.new(entry.address),
                    .size = entry.size,
                },
                .allocated_size = 0,
                .type = .usable,
            };

            const bitset = result_entry.get_bitset();
            const bitset_size = bitset.len * @sizeOf(kernel.Physical.Memory.Map.Entry.BitsetBaseType);
            result_entry.allocated_size = common.align_forward(bitset_size, kernel.arch.page_size);
            result_entry.setup_bitset();
        }
    }

    result.reclaimable.ptr = @intToPtr(@TypeOf(result.reclaimable.ptr), @ptrToInt(result.usable.ptr) + (@sizeOf(kernel.Physical.Memory.Map.Entry) * result.usable.len));

    for (memory_map_entries) |*entry| {
        if (entry.type == .bootloader_reclaimable) {
            const index = result.reclaimable.len;
            result.reclaimable.len += 1;
            var result_entry = &result.reclaimable[index];
            result_entry.* = kernel.Physical.Memory.Map.Entry{
                .descriptor = kernel.Physical.Memory.Region{
                    .address = PhysicalAddress.new(entry.address),
                    .size = entry.size,
                },
                .allocated_size = 0,
                .type = .reclaimable,
            };

            // Don't use the bitset here because it would imply using memory that may not be usable at the moment of writing the bitset to this region
        }
    }

    result.framebuffer.ptr = @intToPtr(@TypeOf(result.framebuffer.ptr), @ptrToInt(result.reclaimable.ptr) + (@sizeOf(kernel.Physical.Memory.Map.Entry) * result.reclaimable.len));

    for (memory_map_entries) |*entry| {
        if (entry.type == .framebuffer) {
            const index = result.framebuffer.len;
            result.framebuffer.len += 1;
            var result_entry = &result.framebuffer[index];
            result_entry.* = kernel.Physical.Memory.Region{
                .address = PhysicalAddress.new(entry.address),
                .size = entry.size,
            };

            // Don't use the bitset here because it would imply using memory that may not be usable at the moment of writing the bitset to this region
        }
    }

    result.kernel_and_modules.ptr = @intToPtr(@TypeOf(result.kernel_and_modules.ptr), @ptrToInt(result.framebuffer.ptr) + (@sizeOf(kernel.Physical.Memory.Region) * result.framebuffer.len));

    for (memory_map_entries) |*entry| {
        if (entry.type == .kernel_and_modules) {
            const index = result.kernel_and_modules.len;
            result.kernel_and_modules.len += 1;
            var result_entry = &result.kernel_and_modules[index];
            result_entry.* = kernel.Physical.Memory.Region{
                .address = PhysicalAddress.new(entry.address),
                .size = entry.size,
            };

            // Don't use the bitset here because it would imply using memory that may not be usable at the moment of writing the bitset to this region
        }
    }

    common.runtime_assert(@src(), result.kernel_and_modules.len == 1);

    result.reserved.ptr = @intToPtr(@TypeOf(result.reserved.ptr), @ptrToInt(result.kernel_and_modules.ptr) + (@sizeOf(kernel.Physical.Memory.Region) * result.kernel_and_modules.len));

    for (memory_map_entries) |*entry| {
        if (entry.type == .reserved) {
            const index = result.reserved.len;
            result.reserved.len += 1;
            var result_entry = &result.reserved[index];
            result_entry.* = kernel.Physical.Memory.Region{
                .address = PhysicalAddress.new(entry.address),
                .size = entry.size,
            };

            // Don't use the bitset here because it would imply using memory that may not be usable at the moment of writing the bitset to this region
        }
    }

    log.debug("Memory map initialized", .{});

    return result;
}

pub fn process_higher_half_direct_map(stivale2_struct: *Struct) Error!VirtualAddress {
    const hhdm_struct = find(Struct.HHDM, stivale2_struct) orelse return Error.higher_half_direct_map;
    log.debug("HHDM: 0x{x}", .{hhdm_struct.addr});
    return VirtualAddress.new(hhdm_struct.addr);
}

pub fn process_pmrs(stivale2_struct: *Struct) Error![]kernel.Virtual.Memory.RegionWithPermissions {
    log.debug("Here", .{});
    const pmrs_struct = find(stivale.Struct.PMRs, stivale2_struct) orelse return Error.pmrs;
    log.debug("PMRS struct: 0x{x}", .{@ptrToInt(pmrs_struct)});
    const pmrs = pmrs_struct.pmrs()[0..pmrs_struct.entry_count];
    if (pmrs.len == 0) return Error.pmrs;
    log.debug("past this", .{});

    common.runtime_assert(@src(), kernel.Virtual.initialized);
    const kernel_section_allocation_size = common.align_forward(@sizeOf(kernel.Virtual.Memory.RegionWithPermissions) * pmrs.len, kernel.arch.page_size);
    const kernel_section_allocation = kernel.address_space.allocate(kernel_section_allocation_size) orelse return Error.pmrs;
    const kernel_sections = kernel_section_allocation.access([*]kernel.Virtual.Memory.RegionWithPermissions)[0..pmrs.len];

    for (pmrs) |pmr, i| {
        const kernel_section = &kernel_sections[i];
        kernel_section.descriptor.address = VirtualAddress.new(pmr.address);
        kernel_section.descriptor.size = pmr.size;
        const permissions = pmr.permissions;
        kernel_section.read = permissions & (1 << stivale.Struct.PMRs.PMR.readable) != 0;
        kernel_section.write = permissions & (1 << stivale.Struct.PMRs.PMR.writable) != 0;
        kernel_section.execute = permissions & (1 << stivale.Struct.PMRs.PMR.executable) != 0;
    }

    return kernel_sections;
}
pub fn get_pmrs(stivale2_struct: *Struct) []Struct.PMRs.PMR {
    const pmrs_struct = find(stivale.Struct.PMRs, stivale2_struct) orelse unreachable;
    const pmrs = pmrs_struct.pmrs()[0..pmrs_struct.entry_count];
    return pmrs;
}

/// This procedure copies the kernel file in a region which is usable and whose allocationcan be registered in the physical allocator bitset
pub fn process_kernel_file(stivale2_struct: *Struct) Error!kernel.File {
    common.runtime_assert(@src(), kernel.Virtual.initialized);
    const kernel_file = find(stivale.Struct.KernelFileV2, stivale2_struct) orelse return Error.kernel_file;
    const file_address = PhysicalAddress.new(kernel_file.kernel_file);
    const file_size = kernel_file.kernel_size;
    const allocation = kernel.address_space.allocate(common.align_forward(file_size, kernel.arch.page_size)) orelse return Error.kernel_file;
    const dst = allocation.access([*]u8)[0..file_size];
    const src = file_address.access_higher_half([*]u8)[0..file_size];
    log.debug("Copying kernel file to (0x{x}, 0x{x})", .{ @ptrToInt(dst.ptr), @ptrToInt(dst.ptr) + dst.len });
    common.copy(u8, dst, src);
    return kernel.File{
        .address = allocation,
        .size = file_size,
    };
}

pub fn process_rsdp(stivale2_struct: *Struct) Error!PhysicalAddress {
    const rsdp_struct = find(stivale.Struct.RSDP, stivale2_struct) orelse return Error.rsdp;
    const rsdp = rsdp_struct.rsdp;
    log.debug("RSDP struct: 0x{x}", .{rsdp});
    const rsdp_address = PhysicalAddress.new(rsdp);
    return rsdp_address;
}

pub fn process_smp(stivale2_struct: *Struct) Error![]kernel.arch.CPU {
    common.runtime_assert(@src(), kernel.Virtual.initialized);
    const smp_struct = find(stivale.Struct.SMP, stivale2_struct) orelse return Error.smp;
    log.debug("SMP struct: {}", .{smp_struct});

    const page_count = kernel.bytes_to_pages(smp_struct.cpu_count * @sizeOf(kernel.arch.CPU), .can_be_not_exact);
    const allocation = kernel.Physical.Memory.allocate_pages(page_count) orelse return Error.smp;
    const cpus = allocation.access_higher_half([*]kernel.arch.CPU)[0..smp_struct.cpu_count];
    const smps = smp_struct.smp_info()[0..smp_struct.cpu_count];
    common.runtime_assert(@src(), smps[0].lapic_id == smp_struct.bsp_lapic_id);
    cpus[0].is_bootstrap = true;

    for (smps) |smp, cpu_index| {
        const cpu = &cpus[cpu_index];
        cpu.lapic_id = smp.lapic_id;
    }

    return cpus;
}
