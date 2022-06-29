const kernel = @import("root");
const common = kernel.common;
const TODO = common.TODO;

var plic_base: u64 = 0;
var plic_size: u64 = 0;
const max_interrupt = 32;

pub const Interrupt = struct {
    handler: fn () u64,
    pending_operations_handler: fn () void,

    pub fn register(interrupt: Interrupt, index: u64) void {
        interrupt_handlers[index] = interrupt;
    }
};

var interrupt_handlers: [max_interrupt]Interrupt = undefined;

inline fn get_priority() [*]volatile u32 {
    return @intToPtr([*]volatile u32, plic_base + 0);
}

inline fn get_pending() [*]volatile u32 {
    return @intToPtr([*]volatile u32, plic_base + 0x1000);
}

inline fn get_senable(hart_id: u64) [*]volatile u32 {
    return @intToPtr([*]volatile u32, plic_base + 0x2080 + hart_id * 0x1000);
}

inline fn get_spriority(hart_id: u64) [*]volatile u32 {
    return @intToPtr([*]volatile u32, plic_base + 0x201000 + hart_id * 0x1000);
}

inline fn get_sclaim(hart_id: u64) *volatile u32 {
    return @intToPtr(*volatile u32, plic_base + 0x201004 + hart_id * 0x1000);
}

pub fn init(hart_id: u64) void {
    log.debug("About to initialized interrupts", .{});
    const plic_dt = kernel.arch.device_tree.find_property("soc", "reg", .exact, &[_][]const u8{"plic"}, &[_]kernel.arch.DeviceTree.SearchType{.start}) orelse @panic("unable to find PLIC in the device tree");
    plic_base = kernel.arch.dt_read_int(u64, plic_dt.value);
    plic_size = kernel.arch.dt_read_int(u64, plic_dt.value[@sizeOf(u64)..]);
    common.runtime_assert(@src(), plic_size & (kernel.arch.page_size - 1) == 0);
    kernel.arch.Virtual.directMap(
        kernel.arch.Virtual.kernel_init_pagetable,
        plic_base,
        plic_size / kernel.arch.page_size,
        kernel.arch.PTE_READ | kernel.arch.PTE_WRITE,
        false,
    );

    var interrupt_i: u64 = 1;
    while (interrupt_i <= max_interrupt) : (interrupt_i += 1) {
        get_priority()[interrupt_i] = 0xffff_ffff;
    }

    get_senable(hart_id)[0] = 0b11111111111;
    get_spriority(hart_id)[0] = 0;
    log.debug("PLIC initialized", .{});
}

const log = common.log.scoped(.Interrupts_init);
const ilog = common.log.scoped(.Interrupts_PLIC);

// TODO: should this be locked?
pub fn handle_external_interrupt(hart_id: u64) void {
    const claimed_interrupt_number = get_sclaim(hart_id).*;
    //ilog.debug("PLIC interrupt number: {}", .{claimed_interrupt_number});
    if (claimed_interrupt_number == 0) @panic("PLIC handler is told an external interrupt has been received, but claim indicates otherwise");
    const operations_to_be_handled = interrupt_handlers[claimed_interrupt_number].handler();
    get_sclaim(hart_id).* = claimed_interrupt_number;

    if (operations_to_be_handled > 0) {
        interrupt_handlers[claimed_interrupt_number].pending_operations_handler();
    }
}
