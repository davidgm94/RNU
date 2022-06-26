const kernel = @import("kernel");
const log = kernel.log.scoped(.CoreHeap);
const TODO = kernel.TODO;
const Physical = kernel.Physical;
const Virtual = kernel.Virtual;

const Heap = @This();

pub const Region = struct {
    virtual: Virtual.Address,
    size: u64,
    allocated: u64,
};

allocator: kernel.Allocator,
// TODO: use another synchronization primitive
lock: kernel.Spinlock,
regions: [region_count]Region,
address_space: *Virtual.AddressSpace,

const region_size = 2 * kernel.mb;
pub const region_count = kernel.core_memory_region.size / region_size;

pub fn init(heap: *Heap, address_space: *Virtual.AddressSpace) void {
    heap.allocator.ptr = heap;
    heap.allocator.vtable = &allocator_interface.vtable;
    heap.address_space = address_space;
}

var allocator_interface = struct {
    vtable: kernel.Allocator.VTable = .{
        .alloc = @ptrCast(fn alloc(heap: *anyopaque, len: usize, ptr_align: u29, len_align: u29, return_address: usize) kernel.Allocator.Error![]u8, alloc),
        .resize = @ptrCast(fn resize(heap: *anyopaque, old_mem: []u8, old_align: u29, new_size: usize, len_align: u29, return_address: usize) ?usize, resize),
        .free = @ptrCast(fn free(heap: *anyopaque, old_mem: []u8, old_align: u29, return_address: usize) void, free),
    },

    fn alloc(heap: *Heap, size: usize, ptr_align: u29, len_align: u29, return_address: usize) kernel.Allocator.Error![]u8 {
        heap.lock.acquire();
        defer heap.lock.release();
        kernel.assert(@src(), size < region_size);

        log.debug("Asked allocation: Size: {}. Pointer alignment: {}. Length alignment: {}. Return address: 0x{x}", .{ size, ptr_align, len_align, return_address });
        var alignment: u64 = len_align;
        if (ptr_align > alignment) alignment = ptr_align;

        const region = blk: {
            for (heap.regions) |*region| {
                if (region.size > 0) {
                    region.allocated = kernel.align_forward(region.allocated, alignment);
                    kernel.assert(@src(), (region.size - region.allocated) >= size);
                    break :blk region;
                } else {
                    log.debug("have to allocate region", .{});
                    const virtual_address = heap.address_space.allocate(region_size) orelse return kernel.Allocator.Error.OutOfMemory;

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
