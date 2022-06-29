const kernel = @import("root");
const common = kernel.common;
const x86_64 = kernel.arch.x86_64;
const log = common.log.scoped(.ACPI);
const TODO = kernel.TODO;
const Virtual = kernel.Virtual;
const Physical = kernel.Physical;
const PhysicalAddress = common.PhysicalAddress;
const VirtualAddress = common.VirtualAddress;

const Signature = enum(u32) {
    APIC = @ptrCast(*const u32, "APIC").*,
    FACP = @ptrCast(*const u32, "FACP").*,
    HPET = @ptrCast(*const u32, "HPET").*,
    MCFG = @ptrCast(*const u32, "MCFG").*,
    WAET = @ptrCast(*const u32, "WAET").*,
};

/// ACPI initialization. We should have a page mapper ready before executing this function
pub fn init(rsdp_physical_address: PhysicalAddress) void {
    var rsdp_physical_page = rsdp_physical_address;
    log.debug("RSDP: 0x{x}", .{rsdp_physical_address.value});
    rsdp_physical_page.page_align_backward(kernel.arch.page_size);
    kernel.address_space.map(rsdp_physical_page, rsdp_physical_page.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.empty());
    const rsdp1 = rsdp_physical_address.access_higher_half(*align(1) RSDP1);
    if (rsdp1.revision == 0) {
        log.debug("First version", .{});
        log.debug("RSDT: 0x{x}", .{rsdp1.RSDT_address});
        const rsdt_physical_address = PhysicalAddress.new(rsdp1.RSDT_address);
        var rsdt_physical_page = rsdt_physical_address;
        rsdt_physical_page.page_align_backward(kernel.arch.page_size);
        kernel.address_space.map(rsdt_physical_page, rsdt_physical_page.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.empty());
        log.debug("Mapped RSDT: 0x{x}", .{rsdt_physical_page.to_higher_half_virtual_address().value});
        const rsdt = rsdt_physical_address.access_higher_half(*align(1) Header);
        log.debug("RSDT length: {}", .{rsdt.length});
        const rsdt_table_count = (rsdt.length - @sizeOf(Header)) / @sizeOf(u32);
        log.debug("RSDT table count: {}", .{rsdt_table_count});
        const tables = @intToPtr([*]align(1) u32, @ptrToInt(rsdt) + @sizeOf(Header))[0..rsdt_table_count];
        for (tables) |table_address| {
            log.debug("Table address: 0x{x}", .{table_address});
            const table_physical_address = PhysicalAddress.new(table_address);
            var table_physical_page = table_physical_address;
            table_physical_page.page_align_backward(kernel.arch.page_size);
            kernel.address_space.map(table_physical_page, table_physical_page.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.empty());
            const header = table_physical_address.access_higher_half(*align(1) Header);

            switch (header.signature) {
                .APIC => {
                    const madt = @ptrCast(*align(1) MADT, header);
                    log.debug("MADT: {}", .{madt});
                    log.debug("LAPIC address: 0x{x}", .{madt.LAPIC_address});

                    const madt_top = @ptrToInt(madt) + madt.header.length;
                    var offset = @ptrToInt(madt) + @sizeOf(MADT);

                    var processor_count: u64 = 0;
                    var iso_count: u64 = 0;
                    var entry_length: u64 = 0;

                    while (offset != madt_top) : (offset += entry_length) {
                        const entry_type = @intToPtr(*MADT.Type, offset).*;
                        entry_length = @intToPtr(*u8, offset + 1).*;
                        processor_count += @boolToInt(entry_type == .LAPIC);
                        iso_count += @boolToInt(entry_type == .ISO);
                    }

                    x86_64.iso = kernel.core_heap.allocator.alloc(x86_64.ISO, iso_count) catch @panic("iso");
                    var iso_i: u64 = 0;

                    common.runtime_assert(@src(), processor_count == kernel.cpus.len);

                    offset = @ptrToInt(madt) + @sizeOf(MADT);

                    while (offset != madt_top) : (offset += entry_length) {
                        const entry_type = @intToPtr(*MADT.Type, offset).*;
                        entry_length = @intToPtr(*u8, offset + 1).*;

                        switch (entry_type) {
                            .LAPIC => {
                                const lapic = @intToPtr(*align(1) MADT.LAPIC, offset);
                                log.debug("LAPIC: {}", .{lapic});
                                common.runtime_assert(@src(), @sizeOf(MADT.LAPIC) == entry_length);
                            },
                            .IO_APIC => {
                                const ioapic = @intToPtr(*align(1) MADT.IO_APIC, offset);
                                log.debug("IO_APIC: {}", .{ioapic});
                                common.runtime_assert(@src(), @sizeOf(MADT.IO_APIC) == entry_length);
                                x86_64.ioapic.gsi = ioapic.global_system_interrupt_base;
                                x86_64.ioapic.address = PhysicalAddress.new(ioapic.IO_APIC_address);
                                kernel.address_space.map(x86_64.ioapic.address, x86_64.ioapic.address.to_higher_half_virtual_address(), kernel.Virtual.AddressSpace.Flags.from_flags(&.{ .read_write, .cache_disable }));
                                x86_64.ioapic.id = ioapic.IO_APIC_ID;
                            },
                            .ISO => {
                                const iso = @intToPtr(*align(1) MADT.InterruptSourceOverride, offset);
                                log.debug("ISO: {}", .{iso});
                                common.runtime_assert(@src(), @sizeOf(MADT.InterruptSourceOverride) == entry_length);
                                const iso_ptr = &x86_64.iso[iso_i];
                                iso_i += 1;
                                iso_ptr.gsi = iso.global_system_interrupt;
                                iso_ptr.source_IRQ = iso.source;
                                iso_ptr.active_low = iso.flags & 2 != 0;
                                iso_ptr.level_triggered = iso.flags & 8 != 0;
                            },
                            .LAPIC_NMI => {
                                const lapic_nmi = @intToPtr(*align(1) MADT.LAPIC_NMI, offset);
                                log.debug("LAPIC_NMI: {}", .{lapic_nmi});
                                common.runtime_assert(@src(), @sizeOf(MADT.LAPIC_NMI) == entry_length);
                            },
                            else => kernel.crash("ni: {}", .{entry_type}),
                        }
                    }
                },
                else => {
                    log.debug("Ignored table: {s}", .{@tagName(header.signature)});
                },
            }
        }
    } else {
        common.runtime_assert(@src(), rsdp1.revision == 2);
        //const rsdp2 = @ptrCast(*RSDP2, rsdp1);
        log.debug("Second version", .{});
        TODO(@src());
    }
}

const rsdt_signature = [4]u8{ 'R', 'S', 'D', 'T' };
pub fn check_valid_sdt(rsdt: *align(1) Header) void {
    log.debug("Header size: {}", .{@sizeOf(Header)});
    common.runtime_assert(@src(), @sizeOf(Header) == 36);
    if (rsdt.revision != 1) {
        @panic("bad revision");
    }
    if (!kernel.string_eq(&rsdt.signature, &rsdt_signature)) {
        @panic("bad signature");
    }
    if (rsdt.length >= 16384) {
        @panic("bad length");
    }
    if (checksum(@ptrCast([*]u8, rsdt)[0..rsdt.length]) != 0) {
        @panic("bad checksum");
    }
}

fn checksum(slice: []const u8) u8 {
    if (slice.len == 0) return 0;

    var total: u64 = 0;
    for (slice) |byte| {
        total += byte;
    }

    return @truncate(u8, total);
}

const RSDP1 = extern struct {
    signature: [8]u8,
    checksum: u8,
    OEM_ID: [6]u8,
    revision: u8,
    RSDT_address: u32,

    comptime {
        common.comptime_assert(@sizeOf(RSDP1) == 20);
    }
};

const RSDP2 = packed struct {
    rsdp1: RSDP1,
    length: u32,
    XSDT_address: u64,
    extended_checksum: u8,
    reserved: [3]u8,
};

const Header = extern struct {
    signature: Signature,
    length: u32,
    revision: u8,
    checksum: u8,
    OEM_ID: [6]u8,
    OEM_table_ID: [8]u8,
    OEM_revision: u32,
    creator_ID: u32,
    creator_revision: u32,
    comptime {
        common.comptime_assert(@sizeOf(Header) == 36);
    }
};

const MADT = extern struct {
    header: Header,
    LAPIC_address: u32,
    flags: u32,

    const Type = enum(u8) {
        LAPIC = 0,
        IO_APIC = 1,
        ISO = 2,
        NMI_source = 3,
        LAPIC_NMI = 4,
        LAPIC_address_override = 5,
        IO_SAPIC = 6,
        LSAPIC = 7,
        platform_interrupt_sources = 8,
        Lx2APIC = 9,
        Lx2APIC_NMI = 0xa,
        GIC_CPU_interface = 0xb,
        GIC_distributor = 0xc,
        GIC_MSI_frame = 0xd,
        GIC_redistributor = 0xe,
        GIC_interrupt_translation_service = 0xf,
    };

    const LAPIC = struct {
        type: Type,
        length: u8,
        ACPI_processor_UID: u8,
        APIC_ID: u8,
        flags: u32,
    };

    const IO_APIC = struct {
        type: Type,
        length: u8,
        IO_APIC_ID: u8,
        reserved: u8,
        IO_APIC_address: u32,
        global_system_interrupt_base: u32,
    };

    const InterruptSourceOverride = packed struct {
        type: Type,
        length: u8,
        bus: u8,
        source: u8,
        global_system_interrupt: u32,
        flags: u16,
    };

    const LAPIC_NMI = packed struct {
        type: Type,
        length: u8,
        ACPI_processor_UID: u8,
        flags: u16,
        LAPIC_lint: u8,
    };
};

const MCFG = packed struct {
    header: Header,
    reserved: u64,

    fn get_configurations(mcfg: *align(1) MCFG) []Configuration {
        const entry_count = (mcfg.header.length - @sizeOf(MCFG)) / @sizeOf(Configuration);
        const configuration_base = @ptrToInt(mcfg) + @sizeOf(MCFG);
        return @intToPtr([*]Configuration, configuration_base)[0..entry_count];
    }

    comptime {
        common.comptime_assert(@sizeOf(MCFG) == @sizeOf(Header) + @sizeOf(u64));
        common.comptime_assert(@sizeOf(Configuration) == 0x10);
    }

    const Configuration = packed struct {
        base_address: u64,
        segment_group_number: u16,
        start_bus: u8,
        end_bus: u8,
        reserved: u32,
    };
};
