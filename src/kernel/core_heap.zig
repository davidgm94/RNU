const kernel = @import("root");
const common = kernel.common;
const log = common.log.scoped(.CoreHeap);
const TODO = common.TODO;
const Virtual = kernel.Virtual;

const Heap = @This();

pub const Region = struct {
    virtual: common.VirtualAddress,
    size: u64,
    allocated: u64,
};

allocator: common.Allocator,
// TODO: use another synchronization primitive
lock: kernel.Spinlock,
regions: [region_count]Region,
address_space: *Virtual.AddressSpace,

const region_size = 2 * common.mb;
pub const region_count = 0x100000000 / region_size;

pub fn init(heap: *Heap, address_space: *Virtual.AddressSpace) void {
    heap.allocator.ptr = heap;
    heap.allocator.vtable = &allocator_interface.vtable;
    heap.address_space = address_space;
}

var allocator_interface = struct {
    vtable: common.Allocator.VTable = .{
        .alloc = @ptrCast(fn alloc(heap: *anyopaque, len: usize, ptr_align: u29, len_align: u29, return_address: usize) common.Allocator.Error![]u8, alloc),
        .resize = @ptrCast(fn resize(heap: *anyopaque, old_mem: []u8, old_align: u29, new_size: usize, len_align: u29, return_address: usize) ?usize, resize),
        .free = @ptrCast(fn free(heap: *anyopaque, old_mem: []u8, old_align: u29, return_address: usize) void, free),
    },

    fn alloc(heap: *Heap, size: usize, ptr_align: u29, len_align: u29, return_address: usize) common.Allocator.Error![]u8 {
        heap.lock.acquire();
        defer heap.lock.release();
        common.runtime_assert(@src(), size < region_size);

        log.debug("Asked allocation: Size: {}. Pointer alignment: {}. Length alignment: {}. Return address: 0x{x}", .{ size, ptr_align, len_align, return_address });
        var alignment: u64 = len_align;
        if (ptr_align > alignment) alignment = ptr_align;

        const region = blk: {
            for (heap.regions) |*region| {
                if (region.size > 0) {
                    region.allocated = common.align_forward(region.allocated, alignment);
                    common.runtime_assert(@src(), (region.size - region.allocated) >= size);
                    break :blk region;
                } else {
                    log.debug("have to allocate region", .{});
                    const virtual_address = heap.address_space.allocate(region_size) orelse return common.Allocator.Error.OutOfMemory;

                    region.* = Region{
                        .virtual = virtual_address,
                        .size = region_size,
                        .allocated = 0,
                    };

                    break :blk region;
                }
            }

            @panic("unreachableeee");
        };

        const result_address = region.virtual.value + region.allocated;
        region.allocated += size;
        return @intToPtr([*]u8, result_address)[0..size];
    }

    fn resize(heap: *Heap, old_mem: []u8, old_align: u29, new_size: usize, len_align: u29, return_address: usize) ?usize {
        _ = heap;
        _ = old_mem;
        _ = old_align;
        _ = new_size;
        _ = len_align;
        _ = return_address;
        TODO(@src());
    }

    fn free(heap: *Heap, old_mem: []u8, old_align: u29, return_address: usize) void {
        _ = heap;
        _ = old_mem;
        _ = old_align;
        _ = return_address;
        TODO(@src());
    }
}{};
