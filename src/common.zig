const std = @import("std");
const builtin = @import("builtin");
const root = @import("root");

pub const FS = @import("common/fs.zig");
pub const VirtualAddress = @import("common/virtual_address.zig");
pub const PhysicalAddress = @import("common/physical_address.zig");

// ARCH
pub const Cpu = std.Target.Cpu;
pub const cpu = builtin.cpu;

// BUILD
pub const build_mode = builtin.mode;

// META PROGRAMMING
pub const reference_all_declarations = std.testing.refAllDecls;
pub const Type = std.builtin.Type;
pub const fields = std.meta.fields;
pub const enum_values = std.enums.values;
pub const IntType = std.meta.Int;
pub const max_int = std.math.maxInt;

// MEMORY ALLOCATION
pub const Allocator = std.mem.Allocator;

// DATA STRUCTURES
pub const ArrayList = std.ArrayListUnmanaged;
pub const ArrayListAligned = std.ArrayListAlignedUnmanaged;
pub const MultiArrayList = std.MultiArrayList;

// DEBUG
pub const StackTrace = std.builtin.StackTrace;
pub const SourceLocation = std.builtin.SourceLocation;

// ATOMIC
pub const AtomicRmwOp = std.builtin.AtomicRmwOp;
pub const AtomicOrder = std.builtin.AtomicOrder;
//pub const spinloop_hint = std.atomic.spinLoopHint;

// LOG
pub const log = std.log;

// STDIO
pub const Writer = std.io.Writer;

// MEMORY MANIPULATION
pub const equal = std.mem.eql;
pub const starts_with = std.mem.startsWith;
pub const ends_with = std.mem.endsWith;
pub const copy = std.mem.copy;
pub const as_bytes = std.mem.asBytes;
const internal_read_int_big = std.mem.readIntBig;

// INTERNAL
const internal_format = std.fmt.format;
const InternalFormatOptions = std.fmt.FormatOptions;

// SIZES
pub const kb = 1024;
pub const mb = kb * 1024;
pub const gb = mb * 1024;
pub const tb = gb * 1024;

fn get_page_size(arch: Cpu.Arch) comptime_int {
    return switch (arch) {
        .x86_64 => 0x1000,
        .riscv64 => 0x1000,
    };
}

pub inline fn string_eq(a: []const u8, b: []const u8) bool {
    return equal(u8, a, b);
}

pub inline fn string_starts_with(str: []const u8, slice: []const u8) bool {
    return starts_with(u8, str, slice);
}

pub inline fn string_ends_with(str: []const u8, slice: []const u8) bool {
    return ends_with(u8, str, slice);
}

pub inline fn align_forward(n: u64, alignment: u64) u64 {
    const mask: u64 = alignment - 1;
    const result = (n + mask) & ~mask;
    return result;
}

pub inline fn align_backward(n: u64, alignment: u64) u64 {
    return n & ~(alignment - 1);
}

pub inline fn is_aligned(n: u64, alignment: u64) bool {
    return n & (alignment - 1) == 0;
}

pub inline fn read_int_big(comptime T: type, slice: []const u8) T {
    return internal_read_int_big(T, slice[0..@sizeOf(T)]);
}

pub inline fn zero_typed_address(address: u64, comptime T: type) *T {
    const result = @intToPtr(*T, address);
    result.* = zeroes(T);
    return result;
}

pub inline fn zero_range(address: u64, size: u64) void {
    zero(@intToPtr([*]u8, address)[0..size]);
}

pub inline fn zero(bytes: []u8) void {
    for (bytes) |*byte| byte.* = 0;
}

pub inline fn zero_slice(slice: anytype) void {
    const bytes = as_bytes(slice);
    zero(bytes);
}

pub inline fn zeroes(comptime T: type) T {
    var result: T = undefined;
    zero(@ptrCast([*]u8, &result)[0..@sizeOf(T)]);
    return result;
}

pub inline fn static_assert(condition: bool) void {
    if (!is_comptime()) unreachable;
    if (!condition) unreachable;
}

pub inline fn assert(source_location: SourceLocation, condition: bool) void {
    if (is_comptime()) unreachable;
    if (!condition) {
        panic(source_location, "Assert failed", .{});
    }
}

pub const MustBeExact = enum {
    must_be_exact,
    can_be_not_exact,
};

pub inline fn bytes_to_pages_extended(bytes: u64, page_size: u64, comptime must_be_exact: MustBeExact) u64 {
    return remainder_division_maybe_exact(bytes, page_size, must_be_exact);
}

pub inline fn bytes_to_sector(bytes: u64, sector_size: u64, comptime must_be_exact: MustBeExact) u64 {
    return remainder_division_maybe_exact(bytes, sector_size, must_be_exact);
}

pub inline fn remainder_division_maybe_exact(dividend: u64, divisor: u64, comptime must_be_exact: MustBeExact) u64 {
    if (divisor == 0) unreachable;
    const quotient = dividend / divisor;
    const remainder = dividend % divisor;
    const remainder_not_zero = remainder != 0;
    if (must_be_exact == .must_be_exact and remainder_not_zero) panic(@src(), "remainder not exact when asked to be exact: {} / {}", .{ dividend, divisor });

    return quotient + @boolToInt(remainder_not_zero);
}

pub fn cstr_len(cstr: [*:0]const u8) u64 {
    var length: u64 = 0;
    while (cstr[length] != 0) : (length += 1) {}
    return length;
}

pub fn Bitflag(comptime is_volatile: bool, comptime EnumT: type) type {
    return struct {
        pub const Enum = EnumT;
        const BitFlagIntType = IntType(.unsigned, @bitSizeOf(EnumT));
        const Ptr = if (is_volatile) *volatile @This() else *@This();

        bits: BitFlagIntType,

        pub inline fn from_flags(comptime flags: []const EnumT) @This() {
            const result = comptime blk: {
                if (flags.len > @bitSizeOf(EnumT)) @compileError("More flags than bits\n");

                comptime var bits: BitFlagIntType = 0;

                inline for (flags) |field| {
                    bits |= 1 << @enumToInt(field);
                }

                break :blk bits;
            };
            return @This(){ .bits = result };
        }

        pub fn from_bits(bits: BitFlagIntType) @This() {
            return @This(){ .bits = bits };
        }

        pub inline fn from_flag(comptime flag: EnumT) @This() {
            const bits = 1 << @enumToInt(flag);
            return @This(){ .bits = bits };
        }

        pub inline fn empty() @This() {
            return @This(){
                .bits = 0,
            };
        }

        pub inline fn all() @This() {
            var result = comptime blk: {
                var bits: BitFlagIntType = 0;
                inline for (@typeInfo(EnumT).Enum.fields) |field| {
                    bits |= 1 << field.value;
                }
                break :blk @This(){
                    .bits = bits,
                };
            };
            return result;
        }

        pub inline fn is_empty(self: @This()) bool {
            return self.bits == 0;
        }

        /// This assumes invalid values in the flags can't be set.
        pub inline fn is_all(self: @This()) bool {
            return all().bits == self.bits;
        }

        pub inline fn contains(self: @This(), comptime flag: EnumT) bool {
            return ((self.bits & (1 << @enumToInt(flag))) >> @enumToInt(flag)) != 0;
        }

        // TODO: create a mutable version of this
        pub inline fn or_flag(self: Ptr, comptime flag: EnumT) void {
            self.bits |= 1 << @enumToInt(flag);
        }

        pub fn format(bitflag: @This(), comptime _: []const u8, _: InternalFormatOptions, writer: anytype) @TypeOf(writer).Error!void {
            try writer.writeAll("{\n");
            for (enum_values(Enum)) |enum_value| {
                const bit_value = @boolToInt(bitflag.bits & (@as(BitFlagIntType, 1) << @intCast(u6, @enumToInt(enum_value))) != 0);
                try internal_format(writer, "\t{s}: {}\n", .{ @tagName(enum_value), bit_value });
            }
            try writer.writeAll("}");
        }
    };
}

const is_root_package_build = @hasDecl(root, "Builder");

pub fn TODO(src: SourceLocation) noreturn {
    _ = src;
    const decls = @typeInfo(root).Struct.decls;
    for (decls) |decl| {
        @compileLog(decl.name);
    }
    if (is_root_package_build) {
        unreachable; //std.debug.panic("TODO at {s}:{}:{} {s}()", .{ src.file, src.line, src.column, src.fn_name });
    } else {
        unreachable; //root.crash("TODO at {s}:{}:{} {s}()", .{ src.file, src.line, src.column, src.fn_name });
    }
}
pub fn panic(src: SourceLocation, comptime message: []const u8, args: anytype) noreturn {
    _ = src;
    _ = message;
    _ = args;
    if (is_root_package_build) {
        log.err("PANIC at {s}:{}:{} {s}()", .{ src.file, src.line, src.column, src.fn_name });
        unreachable;
    } else {
        //root.crash(message, args);
        unreachable;
    }
}

/// @Hack This currently works to determine if the code is being executed at compile time or at run time.
pub fn is_comptime() bool {
    var a: bool = false;
    return @TypeOf(@boolToInt(a)) == comptime_int;
}
