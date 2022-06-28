//! This is not an official USTAR implementation, but my own custom version just to get started with
pub const Node = struct {
    type: NodeType,
    size: u64,
    last_modification: u64,
    name: [100]u8,
    parent: [100]u8,
};

pub const NodeType = enum(u64) {
    empty = 0,
    file = 1,
    directory = 2,
};

pub const sector_size = 0x200;
