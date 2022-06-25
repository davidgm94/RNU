const kernel = @import("../kernel/kernel.zig");
const TODO = kernel.TODO;
const log = kernel.log.scoped(.DMA);

pub const Buffer = struct {
    address: kernel.Virtual.Address,
    total_size: u64,
    completed_size: u64,

    pub const WriteOption = enum {
        can_write,
        cannot_write,
    };

    pub fn next_segment(buffer: *Buffer, comptime can_write: WriteOption) Slice {
        if (buffer.completed_size >= buffer.total_size) {
            @panic("completed size bigger than natural");
        }

        if (buffer.address.is_null()) {
            @panic("DMA buffer virtual address is invalid");
        }

        const virtual_address = buffer.address.offset(buffer.completed_size);
        const physical_address = virtual_address.translate(&kernel.address_space);
        const offset_into_page = virtual_address.offset_into_page();

        if (physical_address.value == 0) {
            @panic("invalid physical address");
        }

        var transfer_byte_count = kernel.arch.page_size;
        if (offset_into_page != 0) {
            transfer_byte_count -= offset_into_page;
            physical_address.value += offset_into_page;
        }

        if (transfer_byte_count > buffer.total_size - buffer.completed_size) {
            transfer_byte_count = buffer.total_size - buffer.completed_size;
        }

        if (can_write) {
            buffer.completed_size += transfer_byte_count;
        }

        return Slice{
            .address = physical_address,
            .size = transfer_byte_count,
            .parent = buffer,
        };
    }
};

pub const Slice = struct {
    address: kernel.Physical.Address,
    size: u64,
};
