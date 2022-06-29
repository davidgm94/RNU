const kernel = @import("root");
fn PackedOptional(IntegerType: type) type {
    const backed_type = @typeInfo(IntegerType);
    comptime common.runtime_assert(backed_type == .Int);
    comptime common.runtime_assert(backed_type.Int.bits > 1);
    return packed struct {
        value: common.IntType(backed_type.Int.signedness, backed_type.Int.bits - 1),
        optional: u1,
    };
}
