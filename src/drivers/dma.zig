const common = @import("../common.zig");

const TODO = common.TODO;
const log = common.log.scoped(.DMA);
const VirtualAddress = common.VirtualAddress;

const Allocator = common.Allocator;

pub const Buffer = struct {
    address: VirtualAddress,
    total_size: u64,
    completed_size: u64,

    pub const WriteOption = enum {
        can_write,
        cannot_write,
    };

    pub const Initialization = struct {
        size: u64,
        alignment: u64,
    };

    pub fn new(allocator: Allocator, initialization: Initialization) !Buffer {
        const allocation_slice = try allocator.allocBytes(@intCast(u29, initialization.alignment), initialization.size, 0, 0);
        return Buffer{
            .address = VirtualAddress.new(@ptrToInt(allocation_slice.ptr)),
            .total_size = initialization.size,
            .completed_size = 0,
        };
    }

    // TODO: implement when the time comes
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
};

//pub const Slice = struct {
//address: PhysicalAddress,
//size: u64,
//};
