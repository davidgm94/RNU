const common = @import("common");
pub const Register = packed struct {
    limit: u16,
    address: u64,

    comptime {
        common.comptime_assert(@sizeOf(Register) == 10);
    }
};
