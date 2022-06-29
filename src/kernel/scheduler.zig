const kernel = @import("root");
const common = kernel.common;
const TODO = common.TODO;
const log = common.log.scoped(.Scheduler);
const VirtualAddress = common.VirtualAddress;

const Virtual = kernel.Virtual;
const Physical = kernel.Physical;
const PrivilegeLevel = kernel.PrivilegeLevel;

pub const Context = kernel.arch.Context;

pub var lock: kernel.arch.Spinlock = undefined;
var thread_pool: [8192]Thread = undefined;
var thread_id: u64 = 0;

pub fn yield(context: *Context) noreturn {
    const current_cpu = kernel.arch.get_current_cpu().?;
    if (current_cpu.spinlock_count > 0) {
        @panic("spins active when yielding");
    }
    kernel.arch.disable_interrupts();
    lock.acquire();
    var old_address_space: *kernel.Virtual.AddressSpace = undefined;
    if (lock.were_interrupts_enabled) @panic("ffff");
    if (current_cpu.current_thread) |current_thread| {
        current_thread.context = context;
        old_address_space = current_thread.address_space;
    } else {
        old_address_space = &kernel.address_space;
    }
    const new_thread = pick_thread();
    new_thread.time_slices += 1;
    // TODO: idle

    //log.debug("RSP: 0x{x}", .{context.rsp});
    //log.debug("Stack top: 0x{x}", .{new_thread.kernel_stack_base.value + new_thread.kernel_stack_size});
    //common.runtime_assert(@src(), context.rsp < new_thread.kernel_stack_base.value + new_thread.kernel_stack_size);

    //kernel.arch.next_timer(1);
    kernel.arch.switch_context(new_thread.context, new_thread.address_space, new_thread.kernel_stack.value, new_thread, old_address_space);
}

pub const Thread = struct {
    kernel_stack: VirtualAddress,
    privilege_level: PrivilegeLevel,
    type: Type,
    kernel_stack_base: VirtualAddress,
    kernel_stack_size: u64,
    user_stack_base: VirtualAddress,
    user_stack_reserve: u64,
    user_stack_commit: u64,
    id: u64,
    context: *kernel.arch.Context,
    time_slices: u64,
    last_known_execution_address: u64,
    address_space: *Virtual.AddressSpace,

    // TODO: idle thread
    const Type = enum(u1) {
        normal = 0,
        idle = 1,
    };

    pub const EntryPoint = struct {
        start_address: u64,
        argument: u64,
    };

    pub fn spawn(privilege_level: PrivilegeLevel, entry_point: EntryPoint) *Thread {
        // TODO: lock
        const new_thread_id = thread_id;
        const thread_index = new_thread_id % thread_pool.len;
        var thread = &thread_pool[thread_index];
        thread_id += 1;

        log.debug("here", .{});
        // TODO: should we always use the same address space for kernel tasks?
        thread.address_space = switch (privilege_level) {
            .kernel => &kernel.address_space,
            .user => Virtual.AddressSpace.new_for_user() orelse unreachable,
        };

        var kernel_stack_size: u64 = 0x5000;
        const user_stack_reserve: u64 = switch (privilege_level) {
            .kernel => kernel_stack_size,
            .user => 0x400000,
        };
        const user_stack_commit: u64 = switch (privilege_level) {
            .kernel => 0,
            .user => 0x10000,
        };
        var user_stack: VirtualAddress = undefined;
        // TODO: implemented idle thread

        const kernel_stack = kernel.address_space.allocate(kernel_stack_size) orelse @panic("unable to allocate the kernel stack");
        log.debug("Kernel stack: 0x{x}", .{kernel_stack.value});
        user_stack = switch (privilege_level) {
            .kernel => kernel_stack,
            .user => blk: {
                // TODO: lock
                const user_stack_physical_address = kernel.Physical.Memory.allocate_pages(kernel.bytes_to_pages(user_stack_reserve, .must_be_exact)) orelse unreachable;
                const user_stack_physical_region = kernel.Physical.Memory.Region.new(user_stack_physical_address, user_stack_reserve);
                const user_stack_base_virtual_address = VirtualAddress.new(0x5000_0000_0000);
                Physical.Memory.map(user_stack_physical_region, thread.address_space, user_stack_base_virtual_address, kernel.Virtual.AddressSpace.Flags.from_flags(&.{ .read_write, .user }));

                break :blk user_stack_base_virtual_address;
            },
        };
        thread.privilege_level = privilege_level;
        log.debug("Thread privilege: {}", .{thread.privilege_level});
        thread.kernel_stack_base = kernel_stack;
        thread.kernel_stack_size = kernel_stack_size;
        thread.user_stack_base = switch (privilege_level) {
            .kernel => VirtualAddress.new(0),
            .user => user_stack,
        };
        log.debug("USB: 0x{x}", .{thread.user_stack_base.value});
        thread.user_stack_reserve = user_stack_reserve;
        thread.user_stack_commit = user_stack_commit;
        thread.id = new_thread_id;
        thread.type = .normal;
        common.runtime_assert(@src(), thread.type == .normal);

        if (thread.type != .idle) {
            log.debug("Creating arch-specific thread initialization", .{});
            // TODO: hack
            thread.context = switch (privilege_level) {
                .kernel => kernel.arch.Context.new(thread, entry_point),
                .user => blk: {
                    var kernel_entry_point_virtual_address_page = VirtualAddress.new(entry_point.start_address);
                    kernel_entry_point_virtual_address_page.page_align_backward(kernel.arch.page_size);
                    const offset = entry_point.start_address - kernel_entry_point_virtual_address_page.value;
                    log.debug("Offset: 0x{x}", .{offset});
                    const entry_point_physical_address_page = kernel.address_space.translate_address(kernel_entry_point_virtual_address_page) orelse @panic("unable to retrieve pa");
                    const user_entry_point_virtual_address_page = VirtualAddress.new(0x6000_0000_0000);
                    thread.address_space.map(entry_point_physical_address_page, user_entry_point_virtual_address_page, kernel.Virtual.AddressSpace.Flags.from_flags(&.{ .user, .read_write }));
                    const user_entry_point_virtual_address = VirtualAddress.new(user_entry_point_virtual_address_page.value + offset);
                    break :blk kernel.arch.Context.new(thread, EntryPoint{
                        .start_address = user_entry_point_virtual_address.value,
                        .argument = entry_point.argument,
                    });
                },
            };
        }

        return thread;
    }

    pub fn terminate(thread: *Thread) void {
        _ = thread;
        TODO(@src());
    }
};

fn pick_thread() *Thread {
    const current_cpu = kernel.arch.get_current_cpu().?;
    const current_thread_id = if (current_cpu.current_thread) |current_thread| current_thread.id else 0;
    common.runtime_assert(@src(), current_thread_id < thread_id);
    //const next_thread_index = kernel.arch.read_timestamp() % thread_id;
    const next_thread_index = 0;
    const new_thread = &thread_pool[next_thread_index];
    return new_thread;
}

//pub fn syscall5(number: SYS, arg1: usize, arg2: usize, arg3: usize, arg4: usize, arg5: usize) usize {
//return asm volatile ("syscall"
//: [ret] "={rax}" (-> usize),
//: [number] "{rax}" (@enumToInt(number)),
//[arg1] "{rdi}" (arg1),
//[arg2] "{rsi}" (arg2),
//[arg3] "{rdx}" (arg3),
//[arg4] "{r10}" (arg4),
//[arg5] "{r8}" (arg5),
//: "rcx", "r11", "memory"
//);
//}

fn user_space() callconv(.Naked) noreturn {
    _ = asm volatile (
        \\mov %%rsp, %%rbp
    );
    var a = [_]u8{ 'u', 's', 'e', 'r', '\n' };
    _ = x86_64.writer_function(&a);
    _ = asm volatile (
        \\call user_space_foo
        \\syscall
        : [ret] "={rax}" (-> usize),
        : [number] "{rax}" (@as(u64, 0)),
          //[arg1] "{rdi}" (arg1),
          //[arg2] "{rsi}" (arg2),
          //[arg3] "{rdx}" (arg3),
          //[arg4] "{r10}" (arg4),
          //[arg5] "{r8}" (arg5),
          //: "rcx", "r11", "memory"
    );
    unreachable;
}

const x86_64 = @import("./arch/x86_64.zig");

export fn user_space_foo() callconv(.C) void {}

fn test_thread(arg: u64) void {
    while (true) {
        log.debug("THREAD {}", .{arg});
    }
}

pub fn test_threads(thread_count: u64) void {
    var thread_i: u64 = 0;
    while (thread_i < thread_count) : (thread_i += 1) {
        _ = Thread.spawn(.kernel, Thread.EntryPoint{
            .start_address = @ptrToInt(test_thread),
            .argument = thread_i,
        });
    }
}

pub fn test_userspace() void {
    _ = Thread.spawn(.user, Thread.EntryPoint{
        .start_address = @ptrToInt(user_space),
        .argument = 2,
    });
}

pub fn init() void {
    //test_threads(1);
    test_userspace();
}
