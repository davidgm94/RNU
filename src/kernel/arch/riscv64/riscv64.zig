const kernel = @import("root");
const common = kernel.common;

pub const Spinlock = @import("spinlock.zig");
pub const sync = @import("sync.zig");
pub const DeviceTree = @import("device_tree.zig");
pub const Timer = @import("timer.zig");
pub const Paging = @import("paging.zig");
pub const Physical = @import("physical.zig");
pub const Virtual = @import("virtual.zig");
pub const Interrupts = @import("interrupts.zig");
pub const SBI = @import("opensbi.zig");
pub const virtio = @import("virtio.zig");
const UART = @import("uart.zig").UART;

pub const page_size = 0x1000;
pub const sector_size = 0x200;
pub const max_cpu = 64;
pub const dt_read_int = kernel.read_int_big;
pub var cpu_count: u64 = 0;
pub var current_cpu: u64 = 0;

const log = common.log.scoped(.RISCV64);

const TODO = kernel.TODO;

const Context = struct {
    integer: [32]u64,
    pc: u64,
    interrupt_stack: u64,
    float: [32]u64,
    hart_id: u64,
    pid: u64,
};

pub fn get_indexed_stack(index: u64) u64 {
    return @ptrToInt(&stack_top) - (hart_stack_size * index);
}

extern var stack_top: u64;
const hart_stack_size = 0x8000;

pub const LocalStorage = struct {
    context: Context,
    padding: [page_size - @sizeOf(Context)]u8,

    comptime {
        kernel.assert_unsafe(@sizeOf(LocalStorage) == page_size);
    }

    pub fn init(self: *@This(), hart_id: u64, boot_hart: bool) void {
        self.context.hart_id = hart_id;
        self.context.interrupt_stack = @ptrToInt(&stack_top) - hart_stack_size * hart_id;
        log.debug("Interrupt stack: 0x{x}", .{self.context.interrupt_stack});
        self.context.pid = 0;

        sscratch.write(@ptrToInt(&self.context));

        var sstatus_value = sstatus.read();
        sstatus_value |= 1 << 18 | 1 << 1;
        if (!boot_hart) sstatus_value |= 1 << 8 | 1 << 5;
        sstatus.write(sstatus_value);

        sie.write(0x220);
        // TODO: correct names
        sync.set_hart_id(@ptrToInt(self));

        if (!boot_hart) {
            TODO(@src());
        }
        //SBI.set_timer(0);
    }

    pub fn get() *LocalStorage {
        return @intToPtr(*LocalStorage, tp.read());
    }
};

pub var local_storage: [max_cpu]LocalStorage = undefined;

pub const OldContext = struct {
    reg: [32]usize,
    sstatus: usize,
    sepc: usize,
};

const Scause = enum(u64) {
    instruction_address_misaligned = 0,
    instruction_access_fault = 1,
    illegal_instruction = 2,
    breakpoint = 3,
    load_address_misaligned = 4,
    load_access_fault = 5,
    store_address_misaligned = 6,
    store_access_fault = 7,
    environment_call_from_user_mode = 8,
    environment_call_from_supervisor_mode = 9,
    instruction_page_fault = 12,
    load_page_fault = 13,
    store_page_fault = 15,

    supervisor_software_interrupt = 0x8000_0000_0000_0001,
    supervisor_timer_interrupt = 0x8000_0000_0000_0005,
    supervisor_external_interrupt = 0x8000_0000_0000_0009,
};

const ilog = common.log.scoped(.Interrupt);

export fn kernel_interrupt_handler(context: *OldContext, scause: Scause, stval: usize) void {
    kernel.arch.writer.should_lock = false;
    defer kernel.arch.writer.should_lock = true;

    _ = context;
    _ = stval;
    //ilog.debug("Interrupt. SCAUSE: {}. STVAL: 0x{x}", .{ scause, stval });
    const hart_id = local_storage[current_cpu].context.hart_id;
    switch (scause) {
        .supervisor_external_interrupt => Interrupts.handle_external_interrupt(hart_id),
        else => spinloop(),
    }
}

pub fn spinloop() noreturn {
    while (true) {}
}

pub const UART0 = 0x1000_0000;
pub var uart = UART(UART0){
    .lock = Spinlock{
        ._lock = 0,
        .hart = null,
    },
};

pub var device_tree: DeviceTree = undefined;

fn CSR(comptime reg_name: []const u8, comptime BitT: type) type {
    return struct {
        pub const Bit = BitT;
        pub inline fn write(value: u64) void {
            asm volatile ("csrw " ++ reg_name ++ ", %[arg1]"
                :
                : [arg1] "r" (value),
            );
        }

        pub inline fn read() u64 {
            return asm volatile ("csrr %[ret], " ++ reg_name
                : [ret] "=r" (-> usize),
            );
        }

        pub inline fn set(comptime bit: Bit) void {
            const value: u64 = 1 << @enumToInt(bit);
            asm volatile ("csrs " ++ reg_name ++ ", %[arg1]"
                :
                : [arg1] "r" (value),
            );
        }

        pub inline fn clear(comptime bit: Bit) void {
            const value: u64 = 1 << @enumToInt(bit);
            asm volatile ("csrc " ++ reg_name ++ ", %[arg1]"
                :
                : [arg1] "r" (value),
            );
        }
    };
}

const sstatus = CSR("sstatus", enum(u32) {
    SIE = 1,
    SPIE = 5,
    UBE = 6,
    SPP = 8,
    SUM = 18,
    MXR = 19,
});
const sie = CSR("sie", enum(u32) {
    SSIE = 1,
    STIE = 5,
    SEIE = 9,
});
const cycle = CSR("cycle", enum(u32) { foo = 0 });
const stvec = CSR("stvec", enum(u32) { foo = 0 });
pub const SATP = CSR("satp", enum(u32) { foo = 0 });
const sscratch = CSR("sscratch", enum(u32) { foo = 0 });

const SATP_SV39: usize = (8 << 60);
pub inline fn MAKE_SATP(pagetable: usize) usize {
    return (SATP_SV39 | (pagetable >> 12));
}

pub fn enable_interrupts() void {
    sstatus.set(.SIE);
}

// disable interrupt
pub fn disable_interrupts() void {
    sstatus.clear(.SIE);
}

pub fn write_function(bytes: []const u8) u64 {
    const bytes_written = uart.write_bytes(bytes);
    return bytes_written;
}

//pub inline fn print(comptime format: []const u8, args: anytype) void {
//writer.print(format, args) catch unreachable;
//}

//pub inline fn write(bytes: []const u8) void {
//_ = writer.write(bytes) catch unreachable;
//}

pub const Bounds = struct {
    extern const kernel_start: u8;
    extern const kernel_end: u8;

    pub inline fn get_start() u64 {
        return @ptrToInt(&kernel_start);
    }

    pub inline fn get_end() u64 {
        return @ptrToInt(&kernel_end);
    }
};

pub const PTE_VALID: usize = (1 << 0);
pub const PTE_READ: usize = (1 << 1);
pub const PTE_WRITE: usize = (1 << 2);
pub const PTE_EXEC: usize = (1 << 3);
pub const PTE_USER: usize = (1 << 4);
pub const PTE_GLOBAL: usize = (1 << 5);
pub const PTE_ACCESSED: usize = (1 << 6);
pub const PTE_DIRTY: usize = (1 << 7);

pub const memory_layout = struct {
    pub const UART0: usize = 0x1000_0000;
};
pub inline fn flush_tlb() void {
    asm volatile ("sfence.vma zero, zero");
}
pub const PAGE_INDEX_MASK: usize = 0x1FF; // 9 bits
pub const PTE_FLAG_MASK: usize = 0x3FF; // 10 bits

pub inline fn PTE_FLAGS(pte: usize) usize {
    return pte & PTE_FLAG_MASK;
}

pub inline fn PAGE_INDEX(level: usize, virtual_address: usize) usize {
    const page_index = (virtual_address >> PAGE_INDEX_SHIFT(level)) & PAGE_INDEX_MASK;
    return page_index;
}

pub inline fn PTE_TO_PA(pte: usize) usize {
    const pa = (pte >> 10) << 12;
    return pa;
}

pub inline fn PA_TO_PTE(pa: usize) usize {
    const pte = (pa >> 12) << 10;
    return pte;
}
pub inline fn PAGE_INDEX_SHIFT(level: usize) u6 {
    return @intCast(u6, PAGE_SHIFT + 9 * level);
}
pub const PAGE_SHIFT: usize = 12;

pub fn get_context(hart_id: u64, machine: u64) u64 {
    return 2 * hart_id + machine;
}

pub const AddressSpace = struct {};

pub const tp = CommonRegister("tp");

pub fn CommonRegister(comptime reg_name: []const u8) type {
    return struct {
        pub inline fn write(value: u64) void {
            asm volatile ("mv " ++ reg_name ++ ", %[arg1]"
                :
                : [arg1] "r" (value),
            );
        }

        pub inline fn read() u64 {
            return asm volatile ("mv %[ret], " ++ reg_name
                : [ret] "=r" (-> u64),
            );
        }
    };
}

pub extern fn trap() callconv(.Naked) void;
pub fn register_trap_handler(trap_handler: u64) void {
    stvec.write(trap_handler);
}

pub extern fn switch_context(old: *kernel.scheduler.Context, new: *kernel.scheduler.Context) callconv(.C) void;

comptime {
    asm (
        \\.section .text
        \\.align 16
        \\.global switch_context
        \\switch_context:
        \\        sd ra, 0(a0)
        \\sd sp, 8(a0)
        \\sd s0, 16(a0)
        \\sd s1, 24(a0)
        \\sd s2, 32(a0)
        \\sd s3, 40(a0)
        \\sd s4, 48(a0)
        \\sd s5, 56(a0)
        \\sd s6, 64(a0)
        \\sd s7, 72(a0)
        \\sd s8, 80(a0)
        \\sd s9, 88(a0)
        \\sd s10, 96(a0)
        \\sd s11, 104(a0)
        \\ld ra, 0(a1)
        \\ld sp, 8(a1)
        \\ld s0, 16(a1)
        \\ld s1, 24(a1)
        \\ld s2, 32(a1)
        \\ld s3, 40(a1)
        \\ld s4, 48(a1)
        \\ld s5, 56(a1)
        \\ld s6, 64(a1)
        \\ld s7, 72(a1)
        \\ld s8, 80(a1)
        \\ld s9, 88(a1)
        \\ld s10, 96(a1)
        \\ld s11, 104(a1)
        \\
        \\ret
    );
}

fn init_logger() void {
    uart.init(false);
}

fn init_cpu_count() void {
    log.debug("CPU count initialized with 1. Is it correct?", .{});
    // TODO: take from the device tree
    cpu_count = 1;
}

fn init_persistent_storage() void {
    log.debug("Initializing persistent storage...", .{});
    // TODO: use the device tree
    kernel.Driver(kernel.Disk, virtio.Block).init(0x10008000) catch @panic("Failed to initialize block driver");
    kernel.Driver(kernel.Filesystem, kernel.RNUFS).init(kernel.Disk.drivers[kernel.Disk.drivers.len - 1]) catch @panic("Failed to initialize filesystem driver");
}

fn init_graphics() void {
    log.debug("Initializing graphics...", .{});
    const driver = kernel.Filesystem.drivers[0];
    const file = driver.read_file_callback(driver, "font.psf");
    log.debug("Font read from disk", .{});
    kernel.font = kernel.PSF1.Font.parse(file);
    log.debug("Font parsed", .{});
    // TODO: use the device tree
    kernel.Driver(kernel.graphics, virtio.GPU).init(0x10007000) catch @panic("error initializating graphics driver");
}

export fn riscv_start(boot_hart_id: u64, fdt_address: u64) callconv(.C) noreturn {
    current_cpu = boot_hart_id;
    register_trap_handler(@ptrToInt(trap));
    init_logger();
    log.debug("Hello RNU. Arch: {s}. Build mode: {s}. Boot HART id: {}. Device tree address: 0x{x}", .{ @tagName(kernel.current_arch), @tagName(kernel.build_mode), boot_hart_id, fdt_address });
    device_tree.base_address = fdt_address;
    device_tree.parse();
    init_cpu_count();
    Timer.init();
    const time_start = Timer.get_timestamp();
    Paging.init();
    Interrupts.init(boot_hart_id);
    local_storage[boot_hart_id].init(boot_hart_id, true);
    const time = Timer.get_time_from_timestamp(Timer.get_timestamp() - time_start);
    init_persistent_storage();
    init_graphics();

    kernel.graphics.drivers[0].draw_horizontal_line(kernel.graphics.Line{ .start = kernel.graphics.Point{ .x = 10, .y = 10 }, .end = kernel.graphics.Point{ .x = 100, .y = 10 } }, kernel.graphics.Color{ .red = 0, .green = 0, .blue = 0, .alpha = 0 });
    kernel.graphics.drivers[0].test_draw_rect();
    kernel.graphics.drivers[0].draw_rect(kernel.graphics.Rect{ .x = 10, .y = 10, .width = 10, .height = 10 }, kernel.graphics.Color{ .red = 0, .green = 0, .blue = 0, .alpha = 0 });
    var i: u64 = 0;
    while (i < 100) : (i += 1) {
        kernel.graphics.drivers[0].draw_string(kernel.graphics.Color{ .red = 0, .green = 0, .blue = 0, .alpha = 0 }, "Hello Mariana");
    }
    @ptrCast(*virtio.GPU, kernel.graphics.drivers[0]).send_and_flush_framebuffer();

    log.debug("Initialized in {} s {} us", .{ time.s, time.us });
    spinloop();
    //kernel.scheduler.schedule();
}
