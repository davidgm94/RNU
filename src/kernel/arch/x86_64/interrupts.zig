const kernel = @import("../../kernel.zig");
const PIC = @import("pic.zig");
const IDT = @import("idt.zig");
const GDT = @import("gdt.zig");
const x86_64 = @import("../x86_64.zig");
const PCI = @import("../../../drivers/pci.zig");

const interrupts = @This();
const Context = x86_64.Context;

const TODO = kernel.TODO;
const Thread = kernel.scheduler.Thread;
const Virtual = kernel.Virtual;
const log = kernel.log.scoped(.interrupts);
const Handler = fn () callconv(.Naked) void;

export var idt: IDT = undefined;

pub fn install_interrupt_handlers() void {
    idt.add_interrupt_handler(get_handler_descriptor(0, false));
    idt.add_interrupt_handler(get_handler_descriptor(1, false));
    idt.add_interrupt_handler(get_handler_descriptor(2, false));
    idt.add_interrupt_handler(get_handler_descriptor(3, false));
    idt.add_interrupt_handler(get_handler_descriptor(4, false));
    idt.add_interrupt_handler(get_handler_descriptor(5, false));
    idt.add_interrupt_handler(get_handler_descriptor(6, false));
    idt.add_interrupt_handler(get_handler_descriptor(7, false));
    idt.add_interrupt_handler(get_handler_descriptor(8, true));
    idt.add_interrupt_handler(get_handler_descriptor(9, false));
    idt.add_interrupt_handler(get_handler_descriptor(10, true));
    idt.add_interrupt_handler(get_handler_descriptor(11, true));
    idt.add_interrupt_handler(get_handler_descriptor(12, true));
    idt.add_interrupt_handler(get_handler_descriptor(13, true));
    idt.add_interrupt_handler(get_handler_descriptor(14, true));
    idt.add_interrupt_handler(get_handler_descriptor(15, false));
    idt.add_interrupt_handler(get_handler_descriptor(16, false));
    idt.add_interrupt_handler(get_handler_descriptor(17, true));
    idt.add_interrupt_handler(get_handler_descriptor(18, false));
    idt.add_interrupt_handler(get_handler_descriptor(19, false));
    idt.add_interrupt_handler(get_handler_descriptor(20, false));
    idt.add_interrupt_handler(get_handler_descriptor(21, false));
    idt.add_interrupt_handler(get_handler_descriptor(22, false));
    idt.add_interrupt_handler(get_handler_descriptor(23, false));
    idt.add_interrupt_handler(get_handler_descriptor(24, false));
    idt.add_interrupt_handler(get_handler_descriptor(25, false));
    idt.add_interrupt_handler(get_handler_descriptor(26, false));
    idt.add_interrupt_handler(get_handler_descriptor(27, false));
    idt.add_interrupt_handler(get_handler_descriptor(28, false));
    idt.add_interrupt_handler(get_handler_descriptor(29, false));
    idt.add_interrupt_handler(get_handler_descriptor(30, false));
    idt.add_interrupt_handler(get_handler_descriptor(31, false));
    idt.add_interrupt_handler(get_handler_descriptor(32, false));
    idt.add_interrupt_handler(get_handler_descriptor(33, false));
    idt.add_interrupt_handler(get_handler_descriptor(34, false));
    idt.add_interrupt_handler(get_handler_descriptor(35, false));
    idt.add_interrupt_handler(get_handler_descriptor(36, false));
    idt.add_interrupt_handler(get_handler_descriptor(37, false));
    idt.add_interrupt_handler(get_handler_descriptor(38, false));
    idt.add_interrupt_handler(get_handler_descriptor(39, false));
    idt.add_interrupt_handler(get_handler_descriptor(40, false));
    idt.add_interrupt_handler(get_handler_descriptor(41, false));
    idt.add_interrupt_handler(get_handler_descriptor(42, false));
    idt.add_interrupt_handler(get_handler_descriptor(43, false));
    idt.add_interrupt_handler(get_handler_descriptor(44, false));
    idt.add_interrupt_handler(get_handler_descriptor(45, false));
    idt.add_interrupt_handler(get_handler_descriptor(46, false));
    idt.add_interrupt_handler(get_handler_descriptor(47, false));
    idt.add_interrupt_handler(get_handler_descriptor(48, false));
    idt.add_interrupt_handler(get_handler_descriptor(49, false));
    idt.add_interrupt_handler(get_handler_descriptor(50, false));
    idt.add_interrupt_handler(get_handler_descriptor(51, false));
    idt.add_interrupt_handler(get_handler_descriptor(52, false));
    idt.add_interrupt_handler(get_handler_descriptor(53, false));
    idt.add_interrupt_handler(get_handler_descriptor(54, false));
    idt.add_interrupt_handler(get_handler_descriptor(55, false));
    idt.add_interrupt_handler(get_handler_descriptor(56, false));
    idt.add_interrupt_handler(get_handler_descriptor(57, false));
    idt.add_interrupt_handler(get_handler_descriptor(58, false));
    idt.add_interrupt_handler(get_handler_descriptor(59, false));
    idt.add_interrupt_handler(get_handler_descriptor(60, false));
    idt.add_interrupt_handler(get_handler_descriptor(61, false));
    idt.add_interrupt_handler(get_handler_descriptor(62, false));
    idt.add_interrupt_handler(get_handler_descriptor(63, false));
    idt.add_interrupt_handler(get_handler_descriptor(64, false));
    idt.add_interrupt_handler(get_handler_descriptor(65, false));
    idt.add_interrupt_handler(get_handler_descriptor(66, false));
    idt.add_interrupt_handler(get_handler_descriptor(67, false));
    idt.add_interrupt_handler(get_handler_descriptor(68, false));
    idt.add_interrupt_handler(get_handler_descriptor(69, false));
    idt.add_interrupt_handler(get_handler_descriptor(70, false));
    idt.add_interrupt_handler(get_handler_descriptor(71, false));
    idt.add_interrupt_handler(get_handler_descriptor(72, false));
    idt.add_interrupt_handler(get_handler_descriptor(73, false));
    idt.add_interrupt_handler(get_handler_descriptor(74, false));
    idt.add_interrupt_handler(get_handler_descriptor(75, false));
    idt.add_interrupt_handler(get_handler_descriptor(76, false));
    idt.add_interrupt_handler(get_handler_descriptor(77, false));
    idt.add_interrupt_handler(get_handler_descriptor(78, false));
    idt.add_interrupt_handler(get_handler_descriptor(79, false));
    idt.add_interrupt_handler(get_handler_descriptor(80, false));
    idt.add_interrupt_handler(get_handler_descriptor(81, false));
    idt.add_interrupt_handler(get_handler_descriptor(82, false));
    idt.add_interrupt_handler(get_handler_descriptor(83, false));
    idt.add_interrupt_handler(get_handler_descriptor(84, false));
    idt.add_interrupt_handler(get_handler_descriptor(85, false));
    idt.add_interrupt_handler(get_handler_descriptor(86, false));
    idt.add_interrupt_handler(get_handler_descriptor(87, false));
    idt.add_interrupt_handler(get_handler_descriptor(88, false));
    idt.add_interrupt_handler(get_handler_descriptor(89, false));
    idt.add_interrupt_handler(get_handler_descriptor(90, false));
    idt.add_interrupt_handler(get_handler_descriptor(91, false));
    idt.add_interrupt_handler(get_handler_descriptor(92, false));
    idt.add_interrupt_handler(get_handler_descriptor(93, false));
    idt.add_interrupt_handler(get_handler_descriptor(94, false));
    idt.add_interrupt_handler(get_handler_descriptor(95, false));
    idt.add_interrupt_handler(get_handler_descriptor(96, false));
    idt.add_interrupt_handler(get_handler_descriptor(97, false));
    idt.add_interrupt_handler(get_handler_descriptor(98, false));
    idt.add_interrupt_handler(get_handler_descriptor(99, false));

    idt.add_interrupt_handler(get_handler_descriptor(100, false));
    idt.add_interrupt_handler(get_handler_descriptor(101, false));
    idt.add_interrupt_handler(get_handler_descriptor(102, false));
    idt.add_interrupt_handler(get_handler_descriptor(103, false));
    idt.add_interrupt_handler(get_handler_descriptor(104, false));
    idt.add_interrupt_handler(get_handler_descriptor(105, false));
    idt.add_interrupt_handler(get_handler_descriptor(106, false));
    idt.add_interrupt_handler(get_handler_descriptor(107, false));
    idt.add_interrupt_handler(get_handler_descriptor(108, false));
    idt.add_interrupt_handler(get_handler_descriptor(109, false));
    idt.add_interrupt_handler(get_handler_descriptor(110, false));
    idt.add_interrupt_handler(get_handler_descriptor(111, false));
    idt.add_interrupt_handler(get_handler_descriptor(112, false));
    idt.add_interrupt_handler(get_handler_descriptor(113, false));
    idt.add_interrupt_handler(get_handler_descriptor(114, false));
    idt.add_interrupt_handler(get_handler_descriptor(115, false));
    idt.add_interrupt_handler(get_handler_descriptor(116, false));
    idt.add_interrupt_handler(get_handler_descriptor(117, false));
    idt.add_interrupt_handler(get_handler_descriptor(118, false));
    idt.add_interrupt_handler(get_handler_descriptor(119, false));
    idt.add_interrupt_handler(get_handler_descriptor(120, false));
    idt.add_interrupt_handler(get_handler_descriptor(121, false));
    idt.add_interrupt_handler(get_handler_descriptor(122, false));
    idt.add_interrupt_handler(get_handler_descriptor(123, false));
    idt.add_interrupt_handler(get_handler_descriptor(124, false));
    idt.add_interrupt_handler(get_handler_descriptor(125, false));
    idt.add_interrupt_handler(get_handler_descriptor(126, false));
    idt.add_interrupt_handler(get_handler_descriptor(127, false));
    idt.add_interrupt_handler(get_handler_descriptor(128, false));
    idt.add_interrupt_handler(get_handler_descriptor(129, false));
    idt.add_interrupt_handler(get_handler_descriptor(130, false));
    idt.add_interrupt_handler(get_handler_descriptor(131, false));
    idt.add_interrupt_handler(get_handler_descriptor(132, false));
    idt.add_interrupt_handler(get_handler_descriptor(133, false));
    idt.add_interrupt_handler(get_handler_descriptor(134, false));
    idt.add_interrupt_handler(get_handler_descriptor(135, false));
    idt.add_interrupt_handler(get_handler_descriptor(136, false));
    idt.add_interrupt_handler(get_handler_descriptor(137, false));
    idt.add_interrupt_handler(get_handler_descriptor(138, false));
    idt.add_interrupt_handler(get_handler_descriptor(139, false));
    idt.add_interrupt_handler(get_handler_descriptor(140, false));
    idt.add_interrupt_handler(get_handler_descriptor(141, false));
    idt.add_interrupt_handler(get_handler_descriptor(142, false));
    idt.add_interrupt_handler(get_handler_descriptor(143, false));
    idt.add_interrupt_handler(get_handler_descriptor(144, false));
    idt.add_interrupt_handler(get_handler_descriptor(145, false));
    idt.add_interrupt_handler(get_handler_descriptor(146, false));
    idt.add_interrupt_handler(get_handler_descriptor(147, false));
    idt.add_interrupt_handler(get_handler_descriptor(148, false));
    idt.add_interrupt_handler(get_handler_descriptor(149, false));
    idt.add_interrupt_handler(get_handler_descriptor(150, false));
    idt.add_interrupt_handler(get_handler_descriptor(151, false));
    idt.add_interrupt_handler(get_handler_descriptor(152, false));
    idt.add_interrupt_handler(get_handler_descriptor(153, false));
    idt.add_interrupt_handler(get_handler_descriptor(154, false));
    idt.add_interrupt_handler(get_handler_descriptor(155, false));
    idt.add_interrupt_handler(get_handler_descriptor(156, false));
    idt.add_interrupt_handler(get_handler_descriptor(157, false));
    idt.add_interrupt_handler(get_handler_descriptor(158, false));
    idt.add_interrupt_handler(get_handler_descriptor(159, false));
    idt.add_interrupt_handler(get_handler_descriptor(160, false));
    idt.add_interrupt_handler(get_handler_descriptor(161, false));
    idt.add_interrupt_handler(get_handler_descriptor(162, false));
    idt.add_interrupt_handler(get_handler_descriptor(163, false));
    idt.add_interrupt_handler(get_handler_descriptor(164, false));
    idt.add_interrupt_handler(get_handler_descriptor(165, false));
    idt.add_interrupt_handler(get_handler_descriptor(166, false));
    idt.add_interrupt_handler(get_handler_descriptor(167, false));
    idt.add_interrupt_handler(get_handler_descriptor(168, false));
    idt.add_interrupt_handler(get_handler_descriptor(169, false));
    idt.add_interrupt_handler(get_handler_descriptor(170, false));
    idt.add_interrupt_handler(get_handler_descriptor(171, false));
    idt.add_interrupt_handler(get_handler_descriptor(172, false));
    idt.add_interrupt_handler(get_handler_descriptor(173, false));
    idt.add_interrupt_handler(get_handler_descriptor(174, false));
    idt.add_interrupt_handler(get_handler_descriptor(175, false));
    idt.add_interrupt_handler(get_handler_descriptor(176, false));
    idt.add_interrupt_handler(get_handler_descriptor(177, false));
    idt.add_interrupt_handler(get_handler_descriptor(178, false));
    idt.add_interrupt_handler(get_handler_descriptor(179, false));
    idt.add_interrupt_handler(get_handler_descriptor(180, false));
    idt.add_interrupt_handler(get_handler_descriptor(181, false));
    idt.add_interrupt_handler(get_handler_descriptor(182, false));
    idt.add_interrupt_handler(get_handler_descriptor(183, false));
    idt.add_interrupt_handler(get_handler_descriptor(184, false));
    idt.add_interrupt_handler(get_handler_descriptor(185, false));
    idt.add_interrupt_handler(get_handler_descriptor(186, false));
    idt.add_interrupt_handler(get_handler_descriptor(187, false));
    idt.add_interrupt_handler(get_handler_descriptor(188, false));
    idt.add_interrupt_handler(get_handler_descriptor(189, false));
    idt.add_interrupt_handler(get_handler_descriptor(190, false));
    idt.add_interrupt_handler(get_handler_descriptor(191, false));
    idt.add_interrupt_handler(get_handler_descriptor(192, false));
    idt.add_interrupt_handler(get_handler_descriptor(193, false));
    idt.add_interrupt_handler(get_handler_descriptor(194, false));
    idt.add_interrupt_handler(get_handler_descriptor(195, false));
    idt.add_interrupt_handler(get_handler_descriptor(196, false));
    idt.add_interrupt_handler(get_handler_descriptor(197, false));
    idt.add_interrupt_handler(get_handler_descriptor(198, false));
    idt.add_interrupt_handler(get_handler_descriptor(199, false));
    idt.add_interrupt_handler(get_handler_descriptor(200, false));

    idt.add_interrupt_handler(get_handler_descriptor(201, false));
    idt.add_interrupt_handler(get_handler_descriptor(202, false));
    idt.add_interrupt_handler(get_handler_descriptor(203, false));
    idt.add_interrupt_handler(get_handler_descriptor(204, false));
    idt.add_interrupt_handler(get_handler_descriptor(205, false));
    idt.add_interrupt_handler(get_handler_descriptor(206, false));
    idt.add_interrupt_handler(get_handler_descriptor(207, false));
    idt.add_interrupt_handler(get_handler_descriptor(208, false));
    idt.add_interrupt_handler(get_handler_descriptor(209, false));
    idt.add_interrupt_handler(get_handler_descriptor(210, false));
    idt.add_interrupt_handler(get_handler_descriptor(211, false));
    idt.add_interrupt_handler(get_handler_descriptor(212, false));
    idt.add_interrupt_handler(get_handler_descriptor(213, false));
    idt.add_interrupt_handler(get_handler_descriptor(214, false));
    idt.add_interrupt_handler(get_handler_descriptor(215, false));
    idt.add_interrupt_handler(get_handler_descriptor(216, false));
    idt.add_interrupt_handler(get_handler_descriptor(217, false));
    idt.add_interrupt_handler(get_handler_descriptor(218, false));
    idt.add_interrupt_handler(get_handler_descriptor(219, false));
    idt.add_interrupt_handler(get_handler_descriptor(220, false));
    idt.add_interrupt_handler(get_handler_descriptor(221, false));
    idt.add_interrupt_handler(get_handler_descriptor(222, false));
    idt.add_interrupt_handler(get_handler_descriptor(223, false));
    idt.add_interrupt_handler(get_handler_descriptor(224, false));
    idt.add_interrupt_handler(get_handler_descriptor(225, false));
    idt.add_interrupt_handler(get_handler_descriptor(226, false));
    idt.add_interrupt_handler(get_handler_descriptor(227, false));
    idt.add_interrupt_handler(get_handler_descriptor(228, false));
    idt.add_interrupt_handler(get_handler_descriptor(229, false));
    idt.add_interrupt_handler(get_handler_descriptor(230, false));
    idt.add_interrupt_handler(get_handler_descriptor(231, false));
    idt.add_interrupt_handler(get_handler_descriptor(232, false));
    idt.add_interrupt_handler(get_handler_descriptor(233, false));
    idt.add_interrupt_handler(get_handler_descriptor(234, false));
    idt.add_interrupt_handler(get_handler_descriptor(235, false));
    idt.add_interrupt_handler(get_handler_descriptor(236, false));
    idt.add_interrupt_handler(get_handler_descriptor(237, false));
    idt.add_interrupt_handler(get_handler_descriptor(238, false));
    idt.add_interrupt_handler(get_handler_descriptor(239, false));
    idt.add_interrupt_handler(get_handler_descriptor(240, false));
    idt.add_interrupt_handler(get_handler_descriptor(241, false));
    idt.add_interrupt_handler(get_handler_descriptor(242, false));
    idt.add_interrupt_handler(get_handler_descriptor(243, false));
    idt.add_interrupt_handler(get_handler_descriptor(244, false));
    idt.add_interrupt_handler(get_handler_descriptor(245, false));
    idt.add_interrupt_handler(get_handler_descriptor(246, false));
    idt.add_interrupt_handler(get_handler_descriptor(247, false));
    idt.add_interrupt_handler(get_handler_descriptor(248, false));
    idt.add_interrupt_handler(get_handler_descriptor(249, false));
    idt.add_interrupt_handler(get_handler_descriptor(250, false));
    idt.add_interrupt_handler(get_handler_descriptor(251, false));
    idt.add_interrupt_handler(get_handler_descriptor(252, false));
    idt.add_interrupt_handler(get_handler_descriptor(253, false));
    idt.add_interrupt_handler(get_handler_descriptor(254, false));
    idt.add_interrupt_handler(get_handler_descriptor(255, false));
}

pub fn init() void {
    // Initialize interrupts
    log.debug("Initializing interrupts", .{});
    PIC.disable();
    install_interrupt_handlers();
    log.debug("Installed interrupt handlers", .{});
    idt.load();
    log.debug("Loaded IDT", .{});
    x86_64.enable_interrupts();
    log.debug("Enabled interrupts", .{});
}

const Exception = enum(u5) {
    divide_by_zero = 0x00,
    debug = 0x01,
    non_maskable_interrupt = 0x2,
    breakpoint = 0x03,
    overflow = 0x04,
    bound_range_exceeded = 0x05,
    invalid_opcode = 0x06,
    device_not_available = 0x07,
    double_fault = 0x08,
    coprocessor_segment_overrun = 0x09,
    invalid_tss = 0x0a,
    segment_not_present = 0x0b,
    stack_segment_fault = 0x0c,
    general_protection_fault = 0x0d,
    page_fault = 0x0e,
    x87_floating_point_exception = 0x10,
    alignment_check = 0x11,
    machine_check = 0x12,
    simd_floating_point_exception = 0x13,
    virtualization_exception = 0x14,
    control_protection_exception = 0x15,
    hypervisor_injection_exception = 0x1c,
    vmm_communication_exception = 0x1d,
    security_exception = 0x1e,
};

const PageFaultErrorCode = kernel.Bitflag(false, enum(u64) {
    present = 0,
    write = 1,
    user = 2,
    reserved_write = 3,
    instruction_fetch = 4,
    protection_key = 5,
    shadow_stack = 6,
    software_guard_extensions = 15,
});

export fn interrupt_handler(context: *Context) align(0x10) callconv(.C) void {
    if (x86_64.are_interrupts_enabled()) {
        @panic("interrupts are enabled");
    }

    log.debug("===================== INT 0x{x} =====================", .{context.interrupt_number});
    const should_swap_gs = x86_64.cs.read() != 0x28;
    log.debug("Should swap GS: {}", .{should_swap_gs});
    if (should_swap_gs) {
        asm volatile ("swapgs");
    }
    defer {
        if (should_swap_gs) asm volatile ("swapgs");
    }

    if (x86_64.get_current_cpu()) |current_cpu| {
        if (current_cpu.spinlock_count != 0 and context.cr8 != 0xe) {
            @panic("spinlock count bug");
        }
    }

    switch (context.interrupt_number) {
        0x0...0x19 => {
            context.debug();
            const exception = @intToEnum(Exception, context.interrupt_number);
            const usermode = context.cs & 3 != 0;
            if (usermode) {
                @panic("usermode not implemented yet");
            } else {
                if (context.cs != @offsetOf(GDT.Table, "code_64")) @panic("invalid cs");
                switch (exception) {
                    .page_fault => {
                        const error_code = PageFaultErrorCode.from_bits(@intCast(u16, context.error_code));
                        const page_fault_address = x86_64.cr2.read();
                        log.debug("Page fault address: 0x{x}. Error code: {}", .{ page_fault_address, error_code });
                        if (error_code.contains(.reserved_write)) {
                            @panic("reserved write");
                        }

                        log.debug("why are we here", .{});
                        if (true) unreachable;

                        x86_64.disable_interrupts();
                    },
                    else => @panic("ni"),
                }
                log.debug("Exception: {s}", .{@tagName(exception)});
            }
        },
        0x40 => {
            kernel.scheduler.yield(context);
        },
        irq_base...irq_base + 0x20 => {
            // TODO: @Lock
            // TODO: check lines
            const line = context.interrupt_number - irq_base;
            // TODO: dont hard code
            const handler = irq_handlers[0];
            const result = handler.callback(handler.context, line);
            if (!result) log.err("Interrupt couldn't be handled", .{});
            x86_64.get_current_cpu().?.lapic.end_of_interrupt();
        },
        0x80 => {
            log.debug("We are getting a syscall", .{});
            context.debug();
            unreachable;
        },
        else => unreachable,
    }

    context.check(@src());

    if (x86_64.are_interrupts_enabled()) {
        @panic("interrupts should not be enabled");
    }
}

const std = @import("std");

inline fn prologue() void {
    asm volatile (
        \\cld
        \\push %%rax
        \\push %%rbx
        \\push %%rcx
        \\push %%rdx
        \\push %%rdi
        \\push %%rsi
        \\push %%rbp
        \\push %%r8
        \\push %%r9
        \\push %%r10
        \\push %%r11
        \\push %%r12
        \\push %%r13
        \\push %%r14
        \\push %%r15
        \\xor %%rax, %%rax
        \\mov %%ds, %%rax
        \\push %% rax
        \\mov %%cr8, %%rax
        \\push %%rax
        \\mov %%rsp, %%rdi
    );
}

pub fn get_handler(comptime interrupt_number: u64, comptime has_error_code: bool) fn handler() align(0x10) callconv(.Naked) void {
    return struct {
        pub fn handler() align(0x10) callconv(.Naked) void {
            if (comptime !has_error_code) asm volatile ("push $0");
            asm volatile ("push %[interrupt_number]"
                :
                : [interrupt_number] "i" (interrupt_number),
            );

            prologue();

            asm volatile ("call interrupt_handler");

            epilogue();

            @panic("Interrupt epilogue didn't iret properly");
        }
    }.handler;
}

pub fn get_handler_descriptor(comptime interrupt_number: u64, comptime has_error_code: bool) IDT.Descriptor {
    kernel.assert(@src(), interrupt_number == IDT.interrupt_i);
    const handler_function = get_handler(interrupt_number, has_error_code);

    const handler_address = @ptrToInt(handler_function);
    return IDT.Descriptor{
        .offset_low = @truncate(u16, handler_address),
        .offset_mid = @truncate(u16, handler_address >> 16),
        .offset_high = @truncate(u32, handler_address >> 32),
        .segment_selector = @offsetOf(GDT.Table, "code_64"), // @TODO: this should change as the GDT selector changes
        .interrupt_stack_table = 0,
        .type = .interrupt,
        .descriptor_privilege_level = 0,
        .present = 1,
    };
}

pub inline fn epilogue() void {
    asm volatile (
        \\cli
        \\pop %%rax
        \\mov %%rax, %%cr8
        \\pop %%rax
        \\mov %%rax, %%ds
        \\mov %%rax, %%es
        \\mov %%rax, %%fs
        //\\mov %%rax, %%gs
        \\pop %%r15
        \\pop %%r14
        \\pop %%r13
        \\pop %%r12
        \\pop %%r11
        \\pop %%r10
        \\pop %%r9
        \\pop %%r8
        \\pop %%rbp
        \\pop %%rsi
        \\pop %%rdi
        \\pop %%rdx
        \\pop %%rcx
        \\pop %%rbx
        \\pop %%rax
        \\add $0x10, %%rsp
        \\iretq
    );
}

pub var msi_handlers: [x86_64.interrupt_vector_msi_count]HandlerInfo = undefined;

pub const HandlerInfo = struct {
    const Callback = fn (u64, u64) bool;
    callback: Callback,
    context: u64,

    pub fn new(context: anytype, callback: anytype) HandlerInfo {
        return HandlerInfo{
            .callback = @ptrCast(Callback, callback),
            .context = @ptrToInt(context),
        };
    }

    pub fn register_MSI(handler: HandlerInfo) void {
        const msi_end = x86_64.interrupt_vector_msi_start + x86_64.interrupt_vector_msi_count;
        var msi = x86_64.interrupt_vector_msi_start;
        while (msi < msi_end) : (msi += 1) {
            if (msi_handlers[msi].address != 0) continue;
            msi_handlers[msi] = handler;
        }
    }

    pub fn register_IRQ(handler: HandlerInfo, maybe_line: ?u64, pci_device: *PCI.Device) bool {
        // TODO: @Lock
        if (maybe_line) |line| {
            if (line > 0x20) @panic("unexpected irq");
        }

        var found = false;

        // TODO: @Lock
        for (irq_handlers) |*irq_handler| {
            if (irq_handler.context == 0) {
                found = true;
                irq_handler.* = .{
                    .callback = handler.callback,
                    .context = handler.context,
                    .line = pci_device.interrupt_line,
                    .pci_device = pci_device,
                };
                break;
            }
        }

        if (!found) return false;

        if (maybe_line) |line| {
            return setup_interrupt_redirection_entry(line);
        } else {
            return setup_interrupt_redirection_entry(9) and
                setup_interrupt_redirection_entry(10) and
                setup_interrupt_redirection_entry(11);
        }
    }
};

var already_setup: u32 = 0;
const irq_base = 0x50;

fn setup_interrupt_redirection_entry(asked_line: u64) bool {
    // TODO: @Lock
    if (already_setup & (@as(u32, 1) << @intCast(u5, asked_line)) != 0) return true;
    const processor_irq = irq_base + @intCast(u32, asked_line);
    _ = processor_irq;

    var active_low = false;
    var level_triggered = false;
    var line = asked_line;

    for (x86_64.iso) |iso| {
        if (iso.source_IRQ == line) {
            line = iso.gsi;
            active_low = iso.active_low;
            level_triggered = iso.level_triggered;
            break;
        }
    }

    if (line >= x86_64.ioapic.gsi and line < (x86_64.ioapic.gsi + @truncate(u8, x86_64.ioapic.read(1) >> 16))) {
        line -= x86_64.ioapic.gsi;
        const redirection_table_index: u32 = @intCast(u32, line) * 2 + 0x10;
        var redirection_entry = processor_irq;
        if (active_low) redirection_entry |= (1 << 13);
        if (level_triggered) redirection_entry |= (1 << 15);

        x86_64.ioapic.write(redirection_table_index, 1 << 16);
        x86_64.ioapic.write(redirection_table_index + 1, kernel.cpus[0].lapic_id << 24);
        x86_64.ioapic.write(redirection_table_index, redirection_entry);

        already_setup |= @as(u32, 1) << @intCast(u5, asked_line);
        return true;
    } else {
        @panic("ioapic");
    }
}

var irq_handlers: [0x40]IRQHandler = undefined;

const IRQHandler = struct {
    callback: HandlerInfo.Callback,
    context: u64,
    line: u64,
    pci_device: *PCI.Device,
};
