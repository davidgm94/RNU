const kernel = @import("root");
const common = kernel.common;
const log = common.log.scoped(.AVL);

pub fn Tree(comptime T: type) type {
    return struct {
        root: ?*Item = null,

        /// This is always refering to the tree
        const Self = @This();
        // TODO: add more keys?
        const Key = u64;

        pub const SearchMode = enum {
            exact,
            smallest_above_or_equal,
            largest_below_or_equal,
        };

        pub const DuplicateKeyPolicy = enum {
            panic,
            allow,
            fail,
        };

        pub fn insert(self: *@This(), item: *Item, item_value: ?*T, key: Key, duplicate_key_policy: DuplicateKeyPolicy) bool {
            //log.debug("Validating before insertion of {*} with key 0x{x}", .{ item, key });
            self.validate();
            //log.debug("Validated before insertion of {*} with key 0x{x}", .{ item, key });

            if (item.tree != null) {
                kernel.crash("item with key 0x{x} already in tree {*}", .{ key, item.tree });
            }

            item.tree = self;

            item.key = key;
            item.children[0] = null;
            item.children[1] = null;
            item.value = item_value;
            item.height = 1;

            var link = &self.root;
            var parent: ?*Item = null;

            while (true) {
                if (link.*) |node| {
                    if (item.compare(node) == 0) {
                        if (duplicate_key_policy == .panic) @panic("avl duplicate panic") else if (duplicate_key_policy == .fail) return false;
                    }

                    const child_index = @boolToInt(item.compare(node) > 0);
                    link = &node.children[child_index];
                    parent = node;
                } else {
                    link.* = item;
                    item.parent = parent;
                    break;
                }
            }

            var fake_root = kernel.zeroes(Item);
            self.root.?.parent = &fake_root;
            fake_root.tree = self;
            fake_root.children[0] = self.root;

            var item_it = item.parent.?;

            while (item_it != &fake_root) {
                const left_height = if (item_it.children[0]) |left| left.height else 0;
                const right_height = if (item_it.children[1]) |right| right.height else 0;
                const balance = left_height - right_height;
                item_it.height = 1 + if (balance > 0) left_height else right_height;
                var new_root: ?*Item = null;
                var old_parent = item_it.parent.?;

                if (balance > 1 and Item.compare_keys(key, item_it.children[0].?.key) <= 0) {
                    const right_rotation = item_it.rotate_right();
                    new_root = right_rotation;
                    const old_parent_child_index = @boolToInt(old_parent.children[1] == item_it);
                    old_parent.children[old_parent_child_index] = right_rotation;
                } else if (balance > 1 and Item.compare_keys(key, item_it.children[0].?.key) > 0 and item_it.children[0].?.children[1] != null) {
                    item_it.children[0] = item_it.children[0].?.rotate_left();
                    item_it.children[0].?.parent = item_it;
                    const right_rotation = item_it.rotate_right();
                    new_root = right_rotation;
                    const old_parent_child_index = @boolToInt(old_parent.children[1] == item_it);
                    old_parent.children[old_parent_child_index] = right_rotation;
                } else if (balance < -1 and Item.compare_keys(key, item_it.children[1].?.key) > 0) {
                    const left_rotation = item_it.rotate_left();
                    new_root = left_rotation;
                    const old_parent_child_index = @boolToInt(old_parent.children[1] == item_it);
                    old_parent.children[old_parent_child_index] = left_rotation;
                } else if (balance < -1 and Item.compare_keys(key, item_it.children[1].?.key) <= 0 and item_it.children[1].?.children[0] != null) {
                    item_it.children[1] = item_it.children[1].?.rotate_right();
                    item_it.children[1].?.parent = item_it;
                    const left_rotation = item_it.rotate_left();
                    new_root = left_rotation;
                    const old_parent_child_index = @boolToInt(old_parent.children[1] == item_it);
                    old_parent.children[old_parent_child_index] = left_rotation;
                }

                if (new_root) |new_root_unwrapped| new_root_unwrapped.parent = old_parent;
                item_it = old_parent;
            }

            self.root = fake_root.children[0];
            self.root.?.parent = null;

            self.validate();
            return true;
        }

        pub fn find(self: *@This(), key: Key, search_mode: SearchMode) ?*Item {
            if (self.modcheck) @panic("concurrent access");
            self.validate();
            return self.find_recursive(self.root, key, search_mode);
        }

        pub fn find_recursive(self: *@This(), maybe_root: ?*Item, key: Key, search_mode: SearchMode) ?*Item {
            if (maybe_root) |root| {
                if (Item.compare_keys(root.key, key) == 0) return root;

                switch (search_mode) {
                    .exact => return self.find_recursive(root.children[0], key, search_mode),
                    .smallest_above_or_equal => {
                        if (Item.compare_keys(root.key, key) > 0) {
                            if (self.find_recursive(root.children[0], key, search_mode)) |item| return item else return root;
                        } else return self.find_recursive(root.children[1], key, search_mode);
                    },
                    .largest_below_or_equal => {
                        if (Item.compare_keys(root.key, key) < 0) {
                            if (self.find_recursive(root.children[1], key, search_mode)) |item| return item else return root;
                        } else return self.find_recursive(root.children[0], key, search_mode);
                    },
                }
            } else {
                return null;
            }
        }

        pub fn remove(self: *@This(), item: *Item) void {
            if (self.modcheck) @panic("concurrent modification");
            self.modcheck = true;
            defer self.modcheck = false;

            self.validate();
            if (item.tree != self) @panic("item not in tree");

            var fake_root = kernel.zeroes(Item);
            self.root.?.parent = &fake_root;
            fake_root.tree = self;
            fake_root.children[0] = self.root;

            if (item.children[0] != null and item.children[1] != null) {
                const smallest = 0;
                const a = self.find_recursive(item.children[1], smallest, .smallest_above_or_equal).?;
                const b = item;
                a.swap(b);
            }

            var link = &item.parent.?.children[@boolToInt(item.parent.?.children[1] == item)];
            link.* = if (item.children[0]) |left| left else item.children[1];

            item.tree = null;
            var item_it = blk: {
                if (link.*) |link_u| {
                    link_u.parent = item.parent;
                    break :blk link.*.?;
                } else break :blk item.parent.?;
            };

            while (item_it != &fake_root) {
                const left_height = if (item_it.children[0]) |left| left.height else 0;
                const right_height = if (item_it.children[1]) |right| right.height else 0;
                const balance = left_height - right_height;
                item_it.height = 1 + if (balance > 0) left_height else right_height;

                var new_root: ?*Item = null;
                var old_parent = item_it.parent.?;

                if (balance > 1) {
                    const left_balance = if (item_it.children[0]) |left| left.get_balance() else 0;
                    if (left_balance >= 0) {
                        const right_rotation = item_it.rotate_right();
                        new_root = right_rotation;
                        const old_parent_child_index = @boolToInt(old_parent.children[1] == item_it);
                        old_parent.children[old_parent_child_index] = right_rotation;
                    } else {
                        item_it.children[0] = item_it.children[0].?.rotate_left();
                        item_it.children[0].?.parent = item_it;
                        const right_rotation = item_it.rotate_right();
                        new_root = right_rotation;
                        const old_parent_child_index = @boolToInt(old_parent.children[1] == item_it);
                        old_parent.children[old_parent_child_index] = right_rotation;
                    }
                } else if (balance < -1) {
                    const right_balance = if (item_it.children[1]) |left| left.get_balance() else 0;
                    if (right_balance <= 0) {
                        const left_rotation = item_it.rotate_left();
                        new_root = left_rotation;
                        const old_parent_child_index = @boolToInt(old_parent.children[1] == item_it);
                        old_parent.children[old_parent_child_index] = left_rotation;
                    } else {
                        item_it.children[1] = item_it.children[1].?.rotate_right();
                        item_it.children[1].?.parent = item_it;
                        const left_rotation = item_it.rotate_left();
                        new_root = left_rotation;
                        const old_parent_child_index = @boolToInt(old_parent.children[1] == item_it);
                        old_parent.children[old_parent_child_index] = left_rotation;
                    }
                }

                if (new_root) |new_root_unwrapped| new_root_unwrapped.parent = old_parent;
                item_it = old_parent;
            }

            self.root = fake_root.children[0];
            if (self.root) |root| {
                if (root.parent != &fake_root) @panic("incorrect root parent");
                root.parent = null;
            }

            self.validate();
        }

        fn validate(self: *@This()) void {
            //log.debug("Validating tree: {*}", .{self});
            if (self.root) |root| {
                //log.debug("Root: {*}", .{root});
                _ = root.validate(self, null);
            }
            //log.debug("Validated tree: {*}", .{self});
        }

        pub const Item = struct {
            value: ?*T = null,
            children: [2]?*Item = [_]?*Item{ null, null },
            parent: ?*Item = null,
            tree: ?*Self = null,
            key: Key = 0,
            height: i32 = 0,

            fn rotate_left(self: *@This()) *Item {
                const x = self;
                const y = x.children[1].?;
                const maybe_t = y.children[0];
                y.children[0] = x;
                x.children[1] = maybe_t;
                x.parent = y;
                if (maybe_t) |t| t.parent = x;

                {
                    const left_height = if (x.children[0]) |left| left.height else 0;
                    const right_height = if (x.children[1]) |right| right.height else 0;
                    const balance = left_height - right_height;
                    x.height = 1 + if (balance > 0) left_height else right_height;
                }

                {
                    const left_height = if (y.children[0]) |left| left.height else 0;
                    const right_height = if (y.children[1]) |right| right.height else 0;
                    const balance = left_height - right_height;
                    y.height = 1 + if (balance > 0) left_height else right_height;
                }

                return y;
            }

            fn rotate_right(self: *@This()) *Item {
                const y = self;
                const x = y.children[0].?;
                const maybe_t = x.children[1];
                x.children[1] = y;
                y.children[0] = maybe_t;
                y.parent = x;
                if (maybe_t) |t| t.parent = y;

                {
                    const left_height = if (y.children[0]) |left| left.height else 0;
                    const right_height = if (y.children[1]) |right| right.height else 0;
                    const balance = left_height - right_height;
                    y.height = 1 + if (balance > 0) left_height else right_height;
                }

                {
                    const left_height = if (x.children[0]) |left| left.height else 0;
                    const right_height = if (x.children[1]) |right| right.height else 0;
                    const balance = left_height - right_height;
                    x.height = 1 + if (balance > 0) left_height else right_height;
                }

                return x;
            }

            fn swap(self: *@This(), other: *@This()) void {
                self.parent.?.children[@boolToInt(self.parent.?.children[1] == self)] = other;
                other.parent.?.children[@boolToInt(other.parent.?.children[1] == other)] = self;

                var temporal_self = self.*;
                var temporal_other = other.*;
                self.parent = temporal_other.parent;
                other.parent = temporal_self.parent;
                self.height = temporal_other.height;
                other.height = temporal_self.height;
                self.children[0] = temporal_other.children[0];
                self.children[1] = temporal_other.children[1];
                other.children[0] = temporal_self.children[0];
                other.children[1] = temporal_self.children[1];

                if (self.children[0]) |a_left| a_left.parent = self;
                if (self.children[1]) |a_right| a_right.parent = self;
                if (other.children[0]) |b_left| b_left.parent = other;
                if (other.children[1]) |b_right| b_right.parent = other;
            }

            fn get_balance(self: *@This()) i32 {
                const left_height = if (self.children[0]) |left| left.height else 0;
                const right_height = if (self.children[1]) |right| right.height else 0;
                return left_height - right_height;
            }

            fn validate(self: *@This(), tree: *Self, parent: ?*@This()) i32 {
                //log.debug("Validating node with key 0x{x}", .{self.key});
                if (self.parent != parent) kernel.crash("Expected parent: {*}, got parent: {*}", .{ parent, self.parent });
                if (self.tree != tree) kernel.crash("Expected tree: {*}, got tree: {*}", .{ tree, self.tree });

                const left_height = blk: {
                    if (self.children[0]) |left| {
                        if (left.compare(self) > 0) @panic("invalid tree");
                        break :blk left.validate(tree, self);
                    } else {
                        break :blk @as(i32, 0);
                    }
                };

                const right_height = blk: {
                    if (self.children[1]) |right| {
                        if (right.compare(self) < 0) @panic("invalid tree");
                        break :blk right.validate(tree, self);
                    } else {
                        break :blk @as(i32, 0);
                    }
                };

                const height = 1 + if (left_height > right_height) left_height else right_height;
                if (height != self.height) @panic("invalid tree");

                //log.debug("Validated node {*}", .{self});

                return height;
            }

            fn compare(self: *@This(), other: *@This()) i32 {
                return compare_keys(self.key, other.key);
            }

            fn compare_keys(key1: u64, key2: u64) i32 {
                if (key1 < key2) return -1;
                if (key1 > key2) return 1;
                return 0;
            }
        };
    };
}
