const kernel = @This();
pub const common = @import("common");

pub const drivers = @import("drivers.zig");
pub const arch = @import("kernel/arch.zig");
pub const Physical = @import("kernel/physical.zig");
pub const Virtual = @import("kernel/virtual.zig");
pub const bounds = arch.Bounds;
pub const Spinlock = arch.Spinlock;
pub const AVL = @import("kernel/avl.zig");
pub const CoreHeap = @import("kernel/core_heap.zig");
pub const PSF1 = @import("kernel/psf1.zig");
pub const scheduler = @import("kernel/scheduler.zig");
pub const ELF = @import("kernel/elf.zig");
pub const Syscall = @import("kernel/syscall.zig");
comptime {
    common.reference_all_declarations(Syscall);
}

pub var main_storage: *drivers.Filesystem = undefined;
pub var address_space = Virtual.AddressSpace.from_context(undefined);
pub var memory_region = Virtual.Memory.Region.new(VirtualAddress.new(0xFFFF900000000000), 0xFFFFF00000000000 - 0xFFFF900000000000);

pub var core_heap: CoreHeap = undefined;
pub var font: PSF1.Font = undefined;
pub var higher_half_direct_map: VirtualAddress = undefined;
pub var file: File = undefined;
pub var sections_in_memory: []Virtual.Memory.RegionWithPermissions = undefined;

pub const File = struct {
    address: VirtualAddress,
    size: u64,
};

pub var cpus: []arch.CPU = undefined;

pub const PrivilegeLevel = enum(u1) {
    kernel = 0,
    user = 1,
};

/// Define root.log_level to override the default
pub const log_level: common.log.Level = switch (common.build_mode) {
    .Debug => .debug,
    .ReleaseSafe => .debug,
    .ReleaseFast, .ReleaseSmall => .info,
};

pub fn log(comptime level: common.log.Level, comptime scope: @TypeOf(.EnumLiteral), comptime format: []const u8, args: anytype) void {
    const scope_prefix = if (scope == .default) ": " else "(" ++ @tagName(scope) ++ "): ";

    //var time: [20]u8 = undefined; // 20 should be enough for 64 bit system
    //const buffer = time[0..];
    //const time_str = kernel.fmt.bufPrint(buffer, "{d:>6}", .{@intToFloat(f64, kernel.arch.Clock.TICK) / @intToFloat(f64, kernel.arch.HZ)}) catch @panic("Unexpected format error in root.log");
    const prefix = "[" ++ @tagName(level) ++ "] " ++ scope_prefix;

    //kernel.arch.writer.writeAll("[") catch unreachable;
    //kernel.arch.writer.writeAll(time_str) catch unreachable;
    //kernel.arch.writer.writeAll("] ") catch unreachable;
    kernel.arch.Writer.lock.acquire();
    kernel.arch.writer.print(prefix ++ format ++ "\n", args) catch unreachable;
    kernel.arch.Writer.lock.release();
}

//var panicking: usize = 0;
pub fn panic(message: []const u8, _: ?*common.StackTrace) noreturn {
    kernel.crash("{s}", .{message});
}

pub fn crash(comptime format: []const u8, args: anytype) noreturn {
    const crash_log = common.log.scoped(.PANIC);
    @setCold(true);
    kernel.arch.disable_interrupts();
    crash_log.err(format, args);
    while (true) {}
}

pub fn TODO(src: common.SourceLocation) noreturn {
    crash("TODO: {s}:{}:{} {s}()", .{ src.file, src.line, src.column, src.fn_name });
}

const VirtualAddress = common.VirtualAddress;

pub inline fn bytes_to_pages(bytes: u64, comptime must_be_exact: common.MustBeExact) u64 {
    return common.bytes_to_pages_extended(bytes, kernel.arch.page_size, must_be_exact);
}
