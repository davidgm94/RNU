const drivers = @import("../drivers.zig");

pub const Buffer = struct {
    virtual_address: u64, // TODO: This should be a wrapped VirtualAddress, but since the code is shared we can't do that
    total_size: u64,
    completed_size: u64,
};

pub const WriteOption = enum {
    can_write,
    cannot_write,
};

pub const Initialization = struct {
    size: u64,
    alignment: u64,
};

pub fn new(allocator: drivers.Allocator, initialization: Initialization) !Buffer {
    const allocation_slice = try allocator.allocBytes(@intCast(u29, initialization.alignment), initialization.size, 0, 0);
    drivers.assert(allocation_slice[0] == 0xaa);
    _ = allocation_slice;
    unreachable;
    //return Buffer{
    //.address = Virtual.Address.new(@ptrToInt(allocation_slice.ptr)),
    //.total_size = initialization.size,
    //.completed_size = 0,
    //};
}

//pub fn next_segment(buffer: *Buffer, comptime write_option: WriteOption) Slice {
//if (buffer.completed_size >= buffer.total_size) {
//@panic("completed size bigger than natural");
//}

//if (!buffer.address.is_valid()) {
//@panic("DMA buffer virtual address is invalid");
//}

//const virtual_address = buffer.address.offset(buffer.completed_size);
//var physical_address = virtual_address.translate(&kernel.address_space) orelse @panic("couldnt be translated");
//const offset_into_page = virtual_address.offset_into_page();

//if (!physical_address.is_valid()) {
//@panic("invalid physical address");
//}

//var transfer_byte_count = kernel.arch.page_size;
//if (offset_into_page != 0) {
//transfer_byte_count -= offset_into_page;
//physical_address.value += offset_into_page;
//@panic("wtf");
//}

//if (transfer_byte_count > buffer.total_size - buffer.completed_size) {
//transfer_byte_count = buffer.total_size - buffer.completed_size;
//@panic("wtf");
//}

//if (write_option == .can_write) {
//buffer.completed_size += transfer_byte_count;
//}

//return Slice{
//.address = physical_address,
//.size = transfer_byte_count,
//};
//}
//pub const Slice = struct {
//address: kernel.Physical.Address,
//size: u64,
//};
