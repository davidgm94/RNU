// Paging behavior is controlled by the following control bits:
// • The WP and PG flags in control register CR0 (bit 16 and bit 31, respectively).
// • The PSE, PAE, PGE, LA57, PCIDE, SMEP, SMAP, PKE, CET, and PKS flags in control register CR4 (bit 4, bit 5, bit 7, bit 12, bit 17, bit 20, bit 21, bit 22, bit 23, and bit 24, respectively).
// • The LME and NXE flags in the IA32_EFER MSR (bit 8 and bit 11, respectively).
// • The AC flag in the EFLAGS register (bit 18).
// • The “enable HLAT” VM-execution control (tertiary processor-based VM-execution control bit 1; see Section 24.6.2, “Processor-Based VM-Execution Controls,” in the Intel® 64 and IA-32 Architectures Software Developer’s Manual, Volume 3C).

const kernel = @import("root");
const common = kernel.common;
const x86_64 = kernel.arch.x86_64;

const Physical = kernel.Physical;
const Virtual = kernel.Virtual;
const TODO = kernel.TODO;
const log = common.log.scoped(.Paging_x86_64);

const PML4Table = [512]PML4E;
const PDPTable = [512]PDPTE;
const PDTable = [512]PDE;
const PTable = [512]PTE;

pub var should_log = false;

pub fn init(stivale_pmrs: []x86_64.Stivale2.Struct.PMRs.PMR) void {
    log.debug("About to dereference memory regions", .{});
    var bootloader_address_space = kernel.address_space;
    kernel.address_space = kernel.Virtual.AddressSpace.new() orelse unreachable;
    kernel.zero(kernel.address_space.arch.get_pml4().access([*]u8)[0..@sizeOf(PML4Table)]);

    // Map the kernel and do some tests
    {
        for (stivale_pmrs) |pmr| {
            const section_virtual_address = Virtual.Address.new(pmr.address);
            const kernel_section_virtual_region = Virtual.Memory.Region.new(section_virtual_address, pmr.size);
            const section_physical_address = bootloader_address_space.translate_address(section_virtual_address) orelse @panic("address not translated");
            kernel_section_virtual_region.map(&kernel.address_space, section_physical_address, blk: {
                var flags = kernel.Virtual.AddressSpace.Flags.empty();
                const permissions = pmr.permissions;
                const executable = permissions & x86_64.Stivale2.Struct.PMRs.PMR.executable != 0;
                const readable = permissions & x86_64.Stivale2.Struct.PMRs.PMR.readable != 0;
                //const writable = permissions & x86_64.Stivale2.Struct.PMRs.PMR.writable != 0;

                if (!executable) {
                    flags.or_flag(.execute_disable);
                }
                kernel.assert(@src(), readable);
                //if (writable) {
                flags.or_flag(.read_write);
                //}

                break :blk flags;
            });
        }
    }

    for (kernel.Physical.Memory.map.usable) |region| {
        region.descriptor.map(&kernel.address_space, region.descriptor.address.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.from_flags(&.{ .read_write, .user }));
    }
    log.debug("Mapped usable", .{});

    for (kernel.Physical.Memory.map.reclaimable) |region| {
        region.descriptor.map(&kernel.address_space, region.descriptor.address.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.from_flags(&.{ .read_write, .user }));
    }
    log.debug("Mapped reclaimable", .{});

    for (kernel.Physical.Memory.map.framebuffer) |region| {
        region.map(&kernel.address_space, region.address.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.from_flags(&.{ .read_write, .user }));
    }
    log.debug("Mapped framebuffer", .{});

    //for (kernel.Physical.Memory.map.reserved) |region| {
    //region.map(&kernel.address_space, region.address.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.empty());
    //}

    kernel.address_space.make_current();
    // Update physical pointers to virtual ones
    kernel.Physical.Memory.map.usable.ptr = @intToPtr(@TypeOf(kernel.Physical.Memory.map.usable.ptr), @ptrToInt(kernel.Physical.Memory.map.usable.ptr) + kernel.higher_half_direct_map.value);
    kernel.Physical.Memory.map.reclaimable.ptr = @intToPtr(@TypeOf(kernel.Physical.Memory.map.reclaimable.ptr), @ptrToInt(kernel.Physical.Memory.map.reclaimable.ptr) + kernel.higher_half_direct_map.value);
    kernel.Physical.Memory.map.framebuffer.ptr = @intToPtr(@TypeOf(kernel.Physical.Memory.map.framebuffer.ptr), @ptrToInt(kernel.Physical.Memory.map.framebuffer.ptr) + kernel.higher_half_direct_map.value);
    kernel.Physical.Memory.map.reserved.ptr = @intToPtr(@TypeOf(kernel.Physical.Memory.map.reserved.ptr), @ptrToInt(kernel.Physical.Memory.map.reserved.ptr) + kernel.higher_half_direct_map.value);
    kernel.Physical.Memory.map.kernel_and_modules.ptr = @intToPtr(@TypeOf(kernel.Physical.Memory.map.kernel_and_modules.ptr), @ptrToInt(kernel.Physical.Memory.map.kernel_and_modules.ptr) + kernel.higher_half_direct_map.value);
    kernel.Virtual.initialized = true;
    log.debug("Memory mapping initialized!", .{});

    for (kernel.Physical.Memory.map.reclaimable) |*region| {
        const bitset = region.get_bitset();
        const bitset_size = bitset.len * @sizeOf(kernel.Physical.Memory.Map.Entry.BitsetBaseType);
        region.allocated_size = kernel.align_forward(bitset_size, kernel.arch.page_size);
        region.setup_bitset();
    }

    const old_reclaimable = kernel.Physical.Memory.map.reclaimable.len;
    kernel.Physical.Memory.map.usable.len += old_reclaimable;
    kernel.Physical.Memory.map.reclaimable.len = 0;

    log.debug("Reclaimed reclaimable physical memory. Counting with {} more regions", .{old_reclaimable});

    var insertion_result = false;
    insertion_result = kernel.address_space.free_regions_by_address.insert(&kernel.memory_region.item_address, &kernel.memory_region, kernel.memory_region.address.value, .panic);
    kernel.assert(@src(), insertion_result);
    insertion_result = kernel.address_space.free_regions_by_size.insert(&kernel.memory_region.item_size, &kernel.memory_region, kernel.memory_region.size, .allow);
    kernel.assert(@src(), insertion_result);
    log.debug("Set root for Virtual Memory Manager tree", .{});
    log.debug("Tree address (free/addr): 0x{x}", .{@ptrToInt(&kernel.address_space.free_regions_by_address)});
    log.debug("Tree address (free/size): 0x{x}", .{@ptrToInt(&kernel.address_space.free_regions_by_size)});
    log.debug("Tree address (used): 0x{x}", .{@ptrToInt(&kernel.address_space.used_regions)});

    for (Physical.Memory.map.usable) |physical_entry| {
        kernel.address_space.integrate_mapped_physical_entry(physical_entry, physical_entry.descriptor.address.to_higher_half_virtual_address()) catch @panic("unable to integrate physical region into vmm");
    }
    log.debug("Finished the integration of usable regions into the kernel address space successfully!", .{});
    for (Physical.Memory.map.framebuffer) |physical_region| {
        kernel.address_space.integrate_mapped_physical_region(physical_region, physical_region.address.to_higher_half_virtual_address()) catch @panic("unable to integrate physical region into vmm");
    }
    log.debug("Finished the integration of framebuffer regions into the kernel address space successfully!", .{});
    for (Physical.Memory.map.usable) |physical_entry| {
        log.debug("(0x{x},\t0x{x}) - 0x{x}", .{ physical_entry.descriptor.address.value, physical_entry.descriptor.address.value + physical_entry.descriptor.size, physical_entry.descriptor.address.value + physical_entry.allocated_size });
    }
    log.debug("Paging initialized", .{});
}

pub const AddressSpace = struct {
    cr3: u64 = 0,

    const Indices = [kernel.enum_values(PageIndex).len]u16;

    pub inline fn new() ?AddressSpace {
        const page_count = kernel.bytes_to_pages(@sizeOf(PML4Table), .must_be_exact);
        const cr3_physical_address = kernel.Physical.Memory.allocate_pages(page_count) orelse return null;
        return AddressSpace{
            .cr3 = cr3_physical_address.value,
        };
    }

    pub inline fn from_current() AddressSpace {
        return AddressSpace{
            .cr3 = x86_64.cr3.read_raw(),
        };
    }

    pub inline fn from_context(context: anytype) AddressSpace {
        // This is taking a u64 instead of a physical address to easily put here the value of the CR3 register
        comptime kernel.assert_unsafe(@TypeOf(context) == u64);
        const cr3 = context;
        return AddressSpace{
            .cr3 = cr3,
        };
    }

    pub fn get_pml4(address_space: AddressSpace) Physical.Address {
        return Physical.Address.new(address_space.cr3);
    }

    pub fn map_kernel_address_space_higher_half(address_space: AddressSpace) void {
        const used_memory_before = kernel.Physical.Memory.map.get_used_memory();
        const cr3 = address_space.cr3;
        // TODO: proper address
        const cr3_physical_address = kernel.Physical.Address.new(cr3);
        const cr3_virtual_address = cr3_physical_address.to_higher_half_virtual_address();
        kernel.address_space.map(cr3_physical_address, cr3_virtual_address, kernel.Virtual.AddressSpace.Flags.from_flags(&.{ .read_write, .user }));
        const pml4 = cr3_virtual_address.access(*PML4Table);
        kernel.zero_slice(pml4[0..0x100]);
        kernel.copy(PML4E, pml4[0x100..], kernel.Physical.Address.new(kernel.address_space.arch.cr3).access_higher_half(*PML4Table)[0x100..]);
        const used_memory_after = kernel.Physical.Memory.map.get_used_memory();
        const memory_overhead = used_memory_after - used_memory_before;
        log.debug("USER CR3: 0x{x}", .{cr3_physical_address.value});
        log.debug("Kernel used memory: {}", .{used_memory_after});
        log.debug("Kernel mapping memory overhead: {}", .{memory_overhead});
    }

    pub fn map(arch_address_space: *AddressSpace, physical_address: Physical.Address, virtual_address: Virtual.Address, flags: kernel.Virtual.AddressSpace.Flags) void {
        if (should_log) log.debug("Init mapping", .{});
        kernel.assert(@src(), virtual_address.is_page_aligned());
        kernel.assert(@src(), physical_address.is_page_aligned());

        const indices = compute_indices(virtual_address);

        var pdp: *volatile PDPTable = undefined;
        {
            var pml4 = arch_address_space.get_pml4().access(*PML4Table);
            var pml4_entry = &pml4[indices[@enumToInt(PageIndex.PML4)]];
            var pml4_entry_value = pml4_entry.value;

            if (pml4_entry_value.contains(.present)) {
                pdp = get_address_from_entry_bits(pml4_entry_value.bits).access(@TypeOf(pdp));
            } else {
                const pdp_allocation = kernel.Physical.Memory.allocate_pages(kernel.bytes_to_pages(@sizeOf(PDPTable), .must_be_exact)) orelse @panic("unable to alloc pdp");
                pdp = pdp_allocation.access(@TypeOf(pdp));
                pdp.* = kernel.zeroes(PDPTable);
                pml4_entry_value.or_flag(.present);
                pml4_entry_value.or_flag(.read_write);
                pml4_entry_value.or_flag(.user);
                pml4_entry_value.bits = set_entry_in_address_bits(pml4_entry_value.bits, pdp_allocation);
                pml4_entry.value = pml4_entry_value;
            }
        }
        if (should_log) log.debug("PDP", .{});

        var pd: *volatile PDTable = undefined;
        {
            var pdp_entry = &pdp[indices[@enumToInt(PageIndex.PDP)]];
            var pdp_entry_value = pdp_entry.value;

            if (pdp_entry_value.contains(.present)) {
                pd = get_address_from_entry_bits(pdp_entry_value.bits).access(@TypeOf(pd));
            } else {
                const pd_allocation = kernel.Physical.Memory.allocate_pages(kernel.bytes_to_pages(@sizeOf(PDTable), .must_be_exact)) orelse @panic("unable to alloc pd");
                pd = pd_allocation.access(@TypeOf(pd));
                pd.* = kernel.zeroes(PDTable);
                pdp_entry_value.or_flag(.present);
                pdp_entry_value.or_flag(.read_write);
                pdp_entry_value.or_flag(.user);
                pdp_entry_value.bits = set_entry_in_address_bits(pdp_entry_value.bits, pd_allocation);
                pdp_entry.value = pdp_entry_value;
            }
        }

        var pt: *volatile PTable = undefined;
        {
            var pd_entry = &pd[indices[@enumToInt(PageIndex.PD)]];
            var pd_entry_value = pd_entry.value;

            if (pd_entry_value.contains(.present)) {
                pt = get_address_from_entry_bits(pd_entry_value.bits).access(@TypeOf(pt));
            } else {
                const pt_allocation = kernel.Physical.Memory.allocate_pages(kernel.bytes_to_pages(@sizeOf(PTable), .must_be_exact)) orelse @panic("unable to alloc pt");
                pt = pt_allocation.access(@TypeOf(pt));
                pt.* = kernel.zeroes(PTable);
                pd_entry_value.or_flag(.present);
                pd_entry_value.or_flag(.read_write);
                pd_entry_value.or_flag(.user);
                pd_entry_value.bits = set_entry_in_address_bits(pd_entry_value.bits, pt_allocation);
                pd_entry.value = pd_entry_value;
            }
        }

        pt[indices[@enumToInt(PageIndex.PT)]] = blk: {
            var pte = PTE{
                .value = PTE.Flags.empty(),
            };

            if (flags.contains(.read_write)) {
                pte.value.or_flag(.read_write);
            }
            if (flags.contains(.user)) {
                pte.value.or_flag(.user);
            }
            pte.value.or_flag(.present);
            pte.value.bits = set_entry_in_address_bits(pte.value.bits, physical_address);

            break :blk pte;
        };

        if (should_log) log.debug("Ended mapping", .{});
    }

    pub fn translate_address(address_space: *AddressSpace, asked_virtual_address: Virtual.Address) ?Physical.Address {
        const virtual_address = if (asked_virtual_address.is_page_aligned()) asked_virtual_address else Virtual.Address.new(kernel.align_backward(asked_virtual_address.value, kernel.arch.page_size));

        const indices = compute_indices(virtual_address);

        var pdp: *volatile PDPTable = undefined;
        {
            //log.debug("CR3: 0x{x}", .{address_space.cr3});
            var pml4 = address_space.get_pml4().access(*PML4Table);
            const pml4_entry = &pml4[indices[@enumToInt(PageIndex.PML4)]];
            var pml4_entry_value = pml4_entry.value;

            if (!pml4_entry_value.contains(.present)) return null;
            //log.debug("PML4 present", .{});

            pdp = get_address_from_entry_bits(pml4_entry_value.bits).access(@TypeOf(pdp));
        }
        if (should_log) log.debug("PDP", .{});

        var pd: *volatile PDTable = undefined;
        {
            var pdp_entry = &pdp[indices[@enumToInt(PageIndex.PDP)]];
            var pdp_entry_value = pdp_entry.value;

            if (!pdp_entry_value.contains(.present)) return null;
            //log.debug("PDP present", .{});

            pd = get_address_from_entry_bits(pdp_entry_value.bits).access(@TypeOf(pd));
        }
        if (should_log) log.debug("PD", .{});

        var pt: *volatile PTable = undefined;
        {
            var pd_entry = &pd[indices[@enumToInt(PageIndex.PD)]];
            var pd_entry_value = pd_entry.value;

            if (!pd_entry_value.contains(.present)) return null;
            //log.debug("PD present", .{});

            pt = get_address_from_entry_bits(pd_entry_value.bits).access(@TypeOf(pt));
        }
        if (should_log) log.debug("PT", .{});

        const pte = pt[indices[@enumToInt(PageIndex.PT)]];
        if (!pte.value.contains(.present)) return null;

        var physical_address = get_address_from_entry_bits(pte.value.bits);
        if (asked_virtual_address.is_page_aligned()) {
            physical_address.page_align_backward();
        }

        return physical_address;
    }

    fn compute_indices(virtual_address: Virtual.Address) Indices {
        var indices: Indices = undefined;
        var va = virtual_address.value;
        va = va >> 12;
        indices[3] = @truncate(u9, va);
        va = va >> 9;
        indices[2] = @truncate(u9, va);
        va = va >> 9;
        indices[1] = @truncate(u9, va);
        va = va >> 9;
        indices[0] = @truncate(u9, va);

        return indices;
    }

    pub fn make_current(address_space: *AddressSpace) void {
        log.debug("Applying address space: 0x{x}", .{address_space.cr3});
        x86_64.cr3.write_raw(address_space.cr3);
        log.debug("Applied address space: 0x{x}", .{address_space.cr3});
    }
};

const address_mask: u64 = 0x000000fffffff000;
fn set_entry_in_address_bits(old_entry_value: u64, new_address: Physical.Address) u64 {
    kernel.assert(@src(), kernel.Physical.Address.max_bit == 40);
    kernel.assert(@src(), new_address.is_page_aligned());
    const address_masked = new_address.value & address_mask;
    const old_entry_value_masked = old_entry_value & ~address_masked;
    const result = address_masked | old_entry_value_masked;
    return result;
}

fn get_address_from_entry_bits(entry_bits: u64) Physical.Address {
    kernel.assert(@src(), kernel.Physical.Address.max_bit == 40);
    const address = entry_bits & address_mask;
    kernel.assert(@src(), kernel.is_aligned(address, kernel.arch.page_size));

    return Physical.Address.new(address);
}

const PageIndex = enum(u3) {
    PML4 = 0,
    PDP = 1,
    PD = 2,
    PT = 3,
};

const PML4E = struct {
    value: Flags,

    const Flags = kernel.Bitflag(true, enum(u64) {
        present = 0,
        read_write = 1,
        user = 2,
        page_level_write_through = 3,
        page_level_cache_disable = 4,
        accessed = 5,
        hlat_restart = 11,
        execute_disable = 63, // IA32_EFER.NXE must be 1
    });
};

const PDPTE = struct {
    value: Flags,

    const Flags = kernel.Bitflag(true, enum(u64) {
        present = 0,
        read_write = 1,
        user = 2,
        page_level_write_through = 3,
        page_level_cache_disable = 4,
        accessed = 5,
        page_size = 7, // must be 0
        hlat_restart = 11,
        execute_disable = 63, // IA32_EFER.NXE must be 1
    });
};

const PDE = struct {
    value: Flags,

    const Flags = kernel.Bitflag(true, enum(u64) {
        present = 0,
        read_write = 1,
        user = 2,
        page_level_write_through = 3,
        page_level_cache_disable = 4,
        accessed = 5,
        page_size = 7, // must be 0
        hlat_restart = 11,
        execute_disable = 63, // IA32_EFER.NXE must be 1
    });
};

const PTE = struct {
    value: Flags,

    const Flags = kernel.Bitflag(true, enum(u64) {
        present = 0,
        read_write = 1,
        user = 2,
        page_level_write_through = 3,
        page_level_cache_disable = 4,
        accessed = 5,
        dirty = 6,
        pat = 7, // must be 0
        global = 8,
        hlat_restart = 11,
        // TODO: protection key
        execute_disable = 63, // IA32_EFER.NXE must be 1
    });
};

pub const HandlePageFaultFlags = kernel.Bitflag(false, enum(u32) {
    write = 0,
    supervisor = 1,
});

pub const HandlePageFaultError = error{};
pub fn handle_page_fault(virtual_address: Virtual.Address, flags: HandlePageFaultFlags) !void {
    log.debug("Handling page fault", .{});
    if (flags.contains(.supervisor)) {
        if (virtual_address.belongs_to_region(kernel.memory_region)) {} else {
            @panic("can't map to unknown region");
        }
    } else {
        @panic("can't handle page fault for user mode");
    }
    log.debug("why are we here");
    unreachable;
}
