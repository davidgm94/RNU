const Spinlock = @This();

const kernel = @import("../../kernel.zig");
const builtin = @import("builtin");
const log = kernel.log.scoped(.Spinlock_x86_64);
const AtomicRmwOp = builtin.AtomicRmwOp;
status: bool,
were_interrupts_enabled: bool,

pub fn new() Spinlock {
    return Spinlock{
        .status = false,
        .were_interrupts_enabled = false,
    };
}

pub fn acquire(spinlock: *volatile Spinlock) void {
    const are_interrupts_enabled = kernel.arch.are_interrupts_enabled();
    kernel.arch.disable_interrupts();
    const expected = false;
    //spinlock.assert_lock_status(expected);
    if (kernel.arch.get_current_cpu()) |current_cpu| {
        current_cpu.spinlock_count += 1;
    }
    while (@cmpxchgStrong(@TypeOf(spinlock.status), @ptrCast(*bool, &spinlock.status), expected, !expected, .Acquire, .Monotonic) != null) {}
    @fence(.Acquire);

    spinlock.were_interrupts_enabled = are_interrupts_enabled;
}

pub fn release(spinlock: *volatile Spinlock) void {
    const expected = true;
    if (kernel.arch.get_current_cpu()) |current_cpu| {
        current_cpu.spinlock_count -= 1;
    }
    spinlock.assert_lock_status(expected);
    const were_interrupts_enabled = spinlock.were_interrupts_enabled;
    @fence(.Release);
    //const result = @cmpxchgStrong(@TypeOf(spinlock.status), @ptrCast(*bool, &spinlock.status), expected, !expected, .Release, .Monotonic);
    spinlock.status = false;
    //kernel.assert(@src(), result == null);
    if (were_interrupts_enabled) {
        kernel.arch.enable_interrupts();
    }
}

inline fn assert_lock_status(spinlock: *volatile Spinlock, expected_status: bool) void {
    if (expected_status != spinlock.status or kernel.arch.are_interrupts_enabled()) {
        kernel.panic("Spinlock not in a desired state", .{});
    }
}
