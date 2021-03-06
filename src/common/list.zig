const common = @import("../common.zig");
const Allocator = common.Allocator;
const TODO = common.TODO;

/// This list works when you are having multiple lists of the same type
pub fn ListItem(comptime T: type) type {
    return struct {
        previous: ?*@This() = null,
        next: ?*@This() = null,
        list: ?*List(T) = null,
        data: T, // = common.zeroes(T), This trips a bug in stage 1

        pub fn new(data: T) @This() {
            return @This(){
                .data = data,
            };
        }
    };
}

pub fn List(comptime T: type) type {
    return struct {
        first: ?*ListItem(T) = null,
        last: ?*ListItem(T) = null,
        count: u64 = 0,

        pub fn append(list: *@This(), list_item: *ListItem(T), item: T) !void {
            common.runtime_assert(@src(), list_item.previous == null);
            common.runtime_assert(@src(), list_item.next == null);
            common.runtime_assert(@src(), list_item.list == null);
            list_item.data = item;

            if (list.last) |last| {
                common.runtime_assert(@src(), list.first != null);
                common.runtime_assert(@src(), list.count > 0);
                last.next = list_item;
                list_item.previous = last;
                list.last = list_item;
            } else {
                common.runtime_assert(@src(), list.first == null);
                common.runtime_assert(@src(), list.count == 0);
                list.first = list_item;
                list.last = list_item;
            }

            list.count += 1;
            list_item.list = list;
        }

        pub fn remove(list: *@This(), list_item: *ListItem(T)) void {
            common.runtime_assert(@src(), list_item.list == list);
            if (list_item.previous) |previous| {
                previous.next = list_item.next;
            } else {
                list.first = list_item.next;
            }

            if (list_item.next) |next| {
                next.previous = list_item.previous;
            } else {
                // Last element of the list.
                list.last = list_item.previous;
            }

            list.count -= 1;
            list_item.list = null;
            common.runtime_assert(@src(), list.count == 0 or (list.first != null and list.last != null));
        }
    };
}

pub fn StableBuffer(comptime T: type, comptime bucket_size: comptime_int) type {
    common.comptime_assert(bucket_size % 64 == 0);
    return struct {
        first: ?*Bucket = null,
        last: ?*Bucket = null,
        bucket_count: u64 = 0,
        element_count: u64 = 0,

        pub const Bucket = struct {
            bitset: [bitset_size]u64 = [1]u64{0} ** bitset_size,
            count: u64 = 0,
            previous: ?*@This() = null,
            next: ?*@This() = null,
            data: [bucket_size]T,

            pub const bitset_size = bucket_size / 64;
            pub const size = bucket_size;

            pub const FindIndexResult = struct {
                bitset_index: u32,
                bit_index: u32,
            };
            pub fn allocate_index(bucket: *Bucket) u64 {
                common.runtime_assert(@src(), bucket.count + 1 <= bucket_size);

                for (bucket.bitset) |*bitset_elem, bitset_i| {
                    // @ZigBug using a comptime var here ends with an infinite loop
                    var bit_i: u8 = 0;
                    while (bit_i < @bitSizeOf(u64)) : (bit_i += 1) {
                        if (bitset_elem.* & (@as(@TypeOf(bitset_elem.*), 1) << @intCast(u6, bit_i)) == 0) {
                            bitset_elem.* |= @as(@TypeOf(bitset_elem.*), 1) << @intCast(u6, bit_i);
                            bucket.count += 1;
                            return bitset_i * @bitSizeOf(u64) + bit_i;
                        }
                    }
                }

                @panic("wtf");
            }
        };

        pub fn add_one(stable_buffer: *@This(), allocator: Allocator) common.Allocator.Error!*T {
            if (stable_buffer.first == null) {
                const first_bucket = try allocator.create(Bucket);
                first_bucket.* = Bucket{ .data = common.zeroes([bucket_size]T) };
                stable_buffer.first = first_bucket;
                first_bucket.bitset[0] = 1;
                stable_buffer.bucket_count += 1;
                stable_buffer.element_count += 1;

                return &first_bucket.data[0];
            } else {
                var iterator = stable_buffer.first;
                var last: ?*Bucket = null;
                while (iterator) |next| {
                    if (next.count < bucket_size) {
                        const index = next.allocate_index();
                        stable_buffer.element_count += 1;
                        const result = &next.data[index];
                        return result;
                    }

                    last = next;
                    iterator = next.next;
                }

                @panic("buffer end");
            }
        }
    };
}
