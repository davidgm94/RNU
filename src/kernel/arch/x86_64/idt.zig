const kernel = @import("root");
const Interrupt = @import("interrupts.zig");
const DescriptorTable = @import("descriptor_table.zig");
const IDT = @This();

const log = common.log.scoped(.IDT);

entries: [256]Descriptor,

pub const Descriptor = packed struct {
    offset_low: u16,
    segment_selector: u16,
    interrupt_stack_table: u3,
    reserved: u5 = 0,
    type: GateType,
    reserved1: u1 = 0,
    descriptor_privilege_level: u2,
    present: u1,
    offset_mid: u16,
    offset_high: u32,
    reserved2: u32 = 0,
};

const GateType = enum(u4) {
    task = 0x5,
    interrupt = 0xe,
    trap = 0xf,
};

pub inline fn load(table: *IDT) void {
    const idtr = DescriptorTable.Register{
        .limit = @sizeOf(IDT) - 1,
        .address = @ptrToInt(table),
    };

    asm volatile (
        \\lidt (%[idt_address])
        :
        : [idt_address] "r" (&idtr),
    );
}

pub var interrupt_i: u64 = 0;
pub fn add_interrupt_handler(table: *IDT, handler: Descriptor) void {
    table.entries[interrupt_i] = handler;
    interrupt_i += 1;
}
