const kernel = @import("root");
fn PackedOptional(IntegerType: type) type {
    const backed_type = @typeInfo(IntegerType);
    comptime kernel.assert_unsafe(backed_type == .Int);
    comptime kernel.assert_unsafe(backed_type.Int.bits > 1);
    return packed struct {
        value: kernel.IntType(backed_type.Int.signedness, backed_type.Int.bits - 1),
        optional: u1,
    };
}
