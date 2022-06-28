const kernel = @import("root");
const common = kernel.common;
const log = common.log.scoped(.Syscall);
const TODO = kernel.TODO;
const x86_64 = @import("arch/x86_64.zig");

pub const ID = enum(u64) {
    thread_exit = 0,
};

const Handler = fn (argument0: u64, argument1: u64, argument2: u64, argument3: u64) callconv(.C) u64;
pub export const syscall_handlers = [1]Handler{
    thread_exit,
};

pub fn thread_exit(argument0: u64, argument1: u64, argument2: u64, argument3: u64) callconv(.C) u64 {
    var a: u64 = 0;
    log.debug("a:{}", .{a});
    _ = argument0;
    _ = argument1;
    _ = argument2;
    _ = argument3;
    TODO(@src());
}
