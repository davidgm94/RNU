const kernel = @import("root");
const common = kernel.common;
const x86_64 = kernel.arch.x86_64;
const DescriptorTable = x86_64.DescriptorTable;
const TSS = @import("tss.zig");
const log = common.log.scoped(.GDT);

pub const Table = packed struct {
    null_entry: Entry = 0, // 0x00
    code_16: Entry = 0x00009a000000ffff, // 0x08
    data_16: Entry = 0x000093000000ffff, // 0x10
    code_32: Entry = 0x00cf9a000000ffff, // 0x18
    data_32: Entry = 0x00cf93000000ffff, // 0x20
    code_64: Entry = 0x00af9b000000ffff, // 0x28
    data_64: Entry = 0x00af93000000ffff, // 0x30
    user_code_64: Entry = 0x00affb000000ffff, // 0x38
    user_data_64: Entry = 0x00aff3000000ffff, // 0x40
    tss: TSS.Descriptor, // 0x48

    comptime {
        common.comptime_assert(@sizeOf(Table) == 9 * @sizeOf(Entry) + @sizeOf(TSS.Descriptor));
        common.comptime_assert(@offsetOf(Table, "code_64") == 0x28);
        common.comptime_assert(@offsetOf(Table, "data_64") == 0x30);
        common.comptime_assert(@offsetOf(Table, "user_code_64") == 0x38);
        common.comptime_assert(@offsetOf(Table, "user_data_64") == 0x40);
        common.comptime_assert(@offsetOf(Table, "tss") == 9 * @sizeOf(Entry));
    }

    pub fn initial_setup(gdt: *Table) void {
        log.debug("Loading GDT...", .{});
        gdt.* = Table{
            .tss = bootstrap_tss.get_descriptor(),
        };
        log.debug("GDT loaded", .{});
        gdt.load();
        log.debug("GDT loaded", .{});
        x86_64.flush_segments_kernel();
        x86_64.set_current_cpu(&kernel.cpus[0]);
        _ = x86_64.get_current_cpu();
    }

    pub inline fn load(gdt: *Table) void {
        const register = DescriptorTable.Register{
            .limit = @sizeOf(Table) - 1,
            .address = @ptrToInt(gdt),
        };

        asm volatile (
            \\  lgdt %[gdt_register]
            :
            : [gdt_register] "*p" (&register),
        );
    }

    pub inline fn update_tss(gdt: *Table, tss: *TSS.Struct) void {
        gdt.tss = tss.get_descriptor();
        const tss_selector: u16 = @offsetOf(Table, "tss");
        asm volatile (
            \\ltr %[tss_selector]
            :
            : [tss_selector] "r" (tss_selector),
        );
        log.debug("Updated TSS", .{});
    }
};

const bootstrap_tss = TSS.Struct{};

const Entry = u64;

pub fn save() DescriptorTable.Register {
    var register: DescriptorTable.Register = undefined;
    asm volatile ("sgdt %[gdt_register]"
        :
        : [gdt_register] "*p" (&register),
    );

    return register;
}
