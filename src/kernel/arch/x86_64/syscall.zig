const kernel = @import("root");
const common = kernel.common;
const x86_64 = kernel.arch.x86_64;

const log = common.log.scoped(.Syscall_x86_64);

pub fn enable() void {
    x86_64.IA32_LSTAR.write(@ptrToInt(syscall_entry_point));
    // TODO: figure out what this does
    x86_64.IA32_FMASK.write(@truncate(u22, ~@as(u64, 1 << 1)));
    // TODO: figure out what this does
    x86_64.IA32_STAR.write(@offsetOf(x86_64.GDT.Table, "code_64") << 32);
    // TODO: figure out what this does
    var efer = x86_64.IA32_EFER.read();
    efer.or_flag(.SCE);
    x86_64.IA32_EFER.write(efer);
    log.debug("Enabled syscalls", .{});
}

//export fn get_kernel_stack() callconv(.C) u64 {
//log.debug("Getting kernel stack...", .{});
//const current_cpu = get_current_cpu() orelse @panic("foo");
//return current_cpu.current_thread.?.kernel_stack.value;
//}
//pub inline fn get_current_cpu() ?*CPU {
////return @intToPtr(?*kernel.arch.CPU, IA32_GS_BASE.read());
//return asm volatile (
//: [result] "=r" (-> ?*kernel.arch.CPU),
//);
//}

export fn syscall_entry_point() callconv(.Naked) void {
    comptime {
        kernel.assert_unsafe(@offsetOf(kernel.arch.CPU, "current_thread") == 0x08);
        kernel.assert_unsafe(@offsetOf(kernel.scheduler.Thread, "kernel_stack") == 0);
    }
    asm volatile (
        \\mov %%gs:[0], %%r15
        \\add %[offset], %%r15
        \\mov (%%r15), %%r15
        \\mov (%%r15), %%r15
        \\mov %%r15, %%rbp
        \\push %%rbp
        \\mov %%rbp, %%rsp
        \\sub $0x10, %%rsp
        :
        : [offset] "i" (@intCast(u8, @offsetOf(kernel.arch.CPU, "current_thread"))),
    );

    const syscall_number = x86_64.rax.read();
    _ = kernel.Syscall.syscall_handlers[syscall_number](0, 0, 0, 0);
    asm volatile ("sysret");
}
