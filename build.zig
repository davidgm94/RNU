const std = @import("std");
const builtin = @import("builtin");
const log = std.log;
const print = std.debug.print;
const assert = std.debug.assert;
const unix = @cImport(@cInclude("unistd.h"));

const common = @import("src/common.zig");
const drivers = @import("src/drivers.zig");
const BuildDisk = @import("src/build/disk.zig");

pub const sector_size = 0x200;
const Builder = std.build.Builder;
const LibExeObjStep = std.build.LibExeObjStep;
const Step = std.build.Step;
const RunStep = std.build.RunStep;
const FileSource = std.build.FileSource;
const Package = std.build.Pkg;
const OptionsStep = std.build.OptionsStep;
const WriteFileStep = std.build.WriteFileStep;

const building_os = builtin.target.os.tag;
const building_arch = builtin.target.cpu.arch;
const Arch = std.Target.Cpu.Arch;

const cache_dir = "zig-cache";
const kernel_name = "kernel.elf";
const kernel_path = cache_dir ++ "/" ++ kernel_name;

pub fn build(b: *Builder) void {
    common.test_comptime_hack();
    const kernel = b.allocator.create(Kernel) catch unreachable;
    // zig fmt: off
    kernel.* = Kernel {
        .builder = b,
        .options = .{
            .arch = Kernel.Options.x86_64.new(.{ .bootloader = .limine, .protocol = .stivale2 }),
            .run = .{
                .disks = &.{
                    .{ .interface = .nvme, .filesystem = .RNU, .userspace_programs = &.{ "minimal" }, .resource_files = &.{ "zap-light16.psf" } }
                },
                .memory = .{ .amount = 4, .unit = .G, },
                .emulator = .{
                    .qemu = .{
                        .vga = .std,
                        .smp = 8,
                        .log = .{ .file = "zig-cache/logfile", .guest_errors = true, .cpu = false, .assembly = true, .interrupts = true, },
                        .run_for_debug = true,
                    },
                },
            },
        }
    };
    // zig fmt: on
    kernel.create();
}

const Kernel = struct {
    builder: *Builder,
    executable: *LibExeObjStep = undefined,
    userspace_programs: []*LibExeObjStep = &.{},
    options: Options,
    boot_image_step: Step = undefined,
    disk_count: u64 = 0,
    disk_step: Step = undefined,
    debug_step: Step = undefined,
    run_argument_list: std.ArrayList([]const u8) = undefined,
    debug_argument_list: std.ArrayList([]const u8) = undefined,
    gdb_script: *WriteFileStep = undefined,

    fn create(kernel: *Kernel) void {
        kernel.create_executable();
        kernel.create_disassembly_step();
        kernel.create_userspace_programs();
        kernel.create_boot_image();
        kernel.create_disk();
        kernel.create_run_and_debug_steps();
    }

    fn create_executable(kernel: *Kernel) void {
        kernel.executable = kernel.builder.addExecutable(kernel_name, "src/kernel.zig");
        var target = get_target_base(kernel.options.arch);

        switch (kernel.options.arch) {
            .riscv64 => {
                target.cpu_features_sub.addFeature(@enumToInt(std.Target.riscv.Feature.d));
                kernel.executable.code_model = .medium;

                const assembly_files = [_][]const u8{
                    "start.S",
                    "interrupt.S",
                };

                const arch_source_dir = kernel.builder.fmt("src/kernel/arch/{s}/", .{@tagName(kernel.options.arch)});

                for (assembly_files) |asm_file| {
                    const asm_file_path = std.mem.concat(kernel.builder.allocator, u8, &.{ arch_source_dir, asm_file }) catch unreachable;
                    kernel.executable.addAssemblyFile(asm_file_path);
                }

                const linker_path = std.mem.concat(kernel.builder.allocator, u8, &.{ arch_source_dir, "linker.ld" }) catch unreachable;
                kernel.executable.setLinkerScriptPath(FileSource.relative(linker_path));
            },
            .x86_64 => {
                kernel.executable.code_model = .kernel;
                //kernel.executable.pie = true;
                kernel.executable.force_pic = true;
                kernel.executable.disable_stack_probing = true;
                kernel.executable.strip = false;
                kernel.executable.code_model = .kernel;
                kernel.executable.red_zone = false;
                kernel.executable.omit_frame_pointer = false;
                const linker_script_path = std.mem.concat(kernel.builder.allocator, u8, &.{ BootImage.x86_64.Limine.base_path, @tagName(kernel.options.arch.x86_64.bootloader.limine.protocol), ".ld" }) catch unreachable;
                kernel.executable.setLinkerScriptPath(FileSource.relative(linker_script_path));
            },
            else => unreachable,
        }

        add_common_packages(kernel.executable);

        kernel.executable.setTarget(target);
        kernel.executable.setBuildMode(kernel.builder.standardReleaseOptions());
        kernel.executable.setOutputDir(cache_dir);
        kernel.executable.emit_llvm_ir = .{ .emit_to = "zig-cache/kernel_llvm.ir" };

        kernel.builder.default_step.dependOn(&kernel.executable.step);
    }

    fn create_disassembly_step(kernel: *Kernel) void {
        var arg_list = std.ArrayList([]const u8).init(kernel.builder.allocator);
        const main_args = &.{ "llvm-objdump", kernel_path };
        const common_flags = &.{ "-d", "-S" };
        arg_list.appendSlice(main_args) catch unreachable;
        arg_list.appendSlice(common_flags) catch unreachable;
        switch (kernel.options.arch) {
            .x86_64 => arg_list.append("-Mintel") catch unreachable,
            else => {},
        }
        const disassembly_kernel = kernel.builder.addSystemCommand(arg_list.items);
        disassembly_kernel.step.dependOn(&kernel.executable.step);
        const disassembly_kernel_step = kernel.builder.step("disasm", "Disassembly the kernel ELF");
        disassembly_kernel_step.dependOn(&disassembly_kernel.step);
    }

    fn create_userspace_programs(kernel: *Kernel) void {
        const linker_script_path = kernel.builder.fmt("src/common/arch/{s}/user/linker.ld", .{@tagName(kernel.options.arch)});
        common.runtime_assert(@src(), kernel.options.run.disks.len == 1);

        var unique_programs = std.ArrayList([]const u8).init(kernel.builder.allocator);
        {
            for (kernel.options.run.disks) |disk| {
                next_program: for (disk.userspace_programs) |program| {
                    for (unique_programs.items) |unique_program| {
                        if (common.string_eq(unique_program, program)) continue :next_program;
                    }

                    unique_programs.append(program) catch unreachable;
                }
            }
        }
        var userspace_programs = std.ArrayList(*LibExeObjStep).initCapacity(kernel.builder.allocator, unique_programs.items.len) catch unreachable;

        for (unique_programs.items) |userspace_program_name| {
            const out_filename = kernel.builder.fmt("{s}.elf", .{userspace_program_name});
            const main_source_file = kernel.builder.fmt("src/user/{s}/main.zig", .{userspace_program_name});
            const program = kernel.builder.addExecutable(out_filename, main_source_file);
            program.setTarget(get_target_base(kernel.options.arch));
            program.setOutputDir(cache_dir);
            program.setBuildMode(kernel.builder.standardReleaseOptions());
            //program.setBuildMode(.ReleaseSafe);
            program.setLinkerScriptPath(FileSource.relative(linker_script_path));
            program.entry_symbol_name = "_start";

            add_common_packages(program);

            kernel.builder.default_step.dependOn(&program.step);

            userspace_programs.appendAssumeCapacity(program);
        }

        kernel.userspace_programs = userspace_programs.items;
    }

    fn create_boot_image(kernel: *Kernel) void {
        kernel.boot_image_step = switch (kernel.options.arch) {
            .x86_64 => switch (kernel.options.arch.x86_64.bootloader) {
                .limine => BootImage.x86_64.Limine.new(kernel),
            },
            else => unreachable,
        };
    }

    fn create_disk(kernel: *Kernel) void {
        Disk.create(kernel);
    }

    fn create_run_and_debug_steps(kernel: *Kernel) void {
        kernel.run_argument_list = std.ArrayList([]const u8).init(kernel.builder.allocator);
        switch (kernel.options.run.emulator) {
            .qemu => {
                const qemu_name = std.mem.concat(kernel.builder.allocator, u8, &.{ "qemu-system-", @tagName(kernel.options.arch) }) catch unreachable;
                kernel.run_argument_list.append(qemu_name) catch unreachable;

                switch (kernel.options.arch) {
                    .x86_64 => {
                        const image_flag = "-cdrom";
                        const image_path = switch (kernel.options.arch.x86_64.bootloader) {
                            .limine => Kernel.BootImage.x86_64.Limine.image_path,
                        };
                        kernel.run_argument_list.append(image_flag) catch unreachable;
                        kernel.run_argument_list.append(image_path) catch unreachable;
                    },
                    .riscv64 => {
                        kernel.run_argument_list.append("-bios") catch unreachable;
                        kernel.run_argument_list.append("default") catch unreachable;
                        kernel.run_argument_list.append("-kernel") catch unreachable;
                        kernel.run_argument_list.append(kernel_path) catch unreachable;
                    },
                    else => unreachable,
                }

                {
                    kernel.run_argument_list.append("-no-reboot") catch unreachable;
                    kernel.run_argument_list.append("-no-shutdown") catch unreachable;
                }

                {
                    const memory_arg = kernel.builder.fmt("{}{s}", .{ kernel.options.run.memory.amount, @tagName(kernel.options.run.memory.unit) });
                    kernel.run_argument_list.append("-m") catch unreachable;
                    kernel.run_argument_list.append(memory_arg) catch unreachable;
                }

                if (kernel.options.run.emulator.qemu.smp) |smp_count| {
                    kernel.run_argument_list.append("-smp") catch unreachable;
                    kernel.run_argument_list.append(kernel.builder.fmt("{}", .{smp_count})) catch unreachable;
                }

                if (kernel.options.run.emulator.qemu.vga) |vga_option| {
                    kernel.run_argument_list.append("-vga") catch unreachable;
                    kernel.run_argument_list.append(@tagName(vga_option)) catch unreachable;
                } else {
                    kernel.run_argument_list.append("-nographic") catch unreachable;
                }

                if (kernel.options.arch == .x86_64) {
                    kernel.run_argument_list.append("-debugcon") catch unreachable;
                    kernel.run_argument_list.append("stdio") catch unreachable;
                }

                kernel.run_argument_list.append("-global") catch unreachable;
                kernel.run_argument_list.append("virtio-mmio.force-legacy=false") catch unreachable;

                for (kernel.options.run.disks) |disk, disk_i| {
                    kernel.run_argument_list.append("-drive") catch unreachable;
                    const disk_id = kernel.builder.fmt("disk{}", .{disk_i});
                    const disk_path = kernel.builder.fmt("zig-cache/{s}.bin", .{disk_id});
                    const drive_options = kernel.builder.fmt("file={s},if=none,id={s},format=raw", .{ disk_path, disk_id });
                    kernel.run_argument_list.append(drive_options) catch unreachable;

                    switch (disk.interface) {
                        .nvme => {
                            kernel.run_argument_list.append("-device") catch unreachable;
                            const device_options = kernel.builder.fmt("nvme,drive={s},serial=1234", .{disk_id});
                            kernel.run_argument_list.append(device_options) catch unreachable;
                        },
                        .virtio => {
                            kernel.run_argument_list.append("-device") catch unreachable;
                            const device_type = switch (kernel.options.arch) {
                                .x86_64 => "pci",
                                .riscv64 => "device",
                                else => unreachable,
                            };
                            const device_options = kernel.builder.fmt("virtio-blk-{s},drive={s}", .{ device_type, disk_id });
                            kernel.run_argument_list.append(device_options) catch unreachable;
                        },
                        else => unreachable,
                    }
                }

                // Here the arch-specific stuff start and that's why the lists are split. For debug builds virtualization is pointless since it gives you no debug information
                kernel.run_argument_list.append("-machine") catch unreachable;
                kernel.debug_argument_list = kernel.run_argument_list.clone() catch unreachable;
                const machine = switch (kernel.options.arch) {
                    .x86_64 => "q35",
                    .riscv64 => "virt",
                    else => unreachable,
                };
                kernel.debug_argument_list.append(machine) catch unreachable;
                if (kernel.options.arch == building_arch and !kernel.options.run.emulator.qemu.run_for_debug) {
                    kernel.run_argument_list.append(kernel.builder.fmt("{s},accel=kvm:whpx:tcg", .{machine})) catch unreachable;
                } else {
                    kernel.run_argument_list.append(machine) catch unreachable;

                    if (kernel.options.run.emulator.qemu.log) |log_options| {
                        var log_what = std.ArrayList(u8).init(kernel.builder.allocator);
                        if (log_options.guest_errors) log_what.appendSlice("guest_errors,") catch unreachable;
                        if (log_options.cpu) log_what.appendSlice("cpu,") catch unreachable;
                        if (log_options.interrupts) log_what.appendSlice("int,") catch unreachable;
                        if (log_options.assembly) log_what.appendSlice("in_asm,") catch unreachable;

                        if (log_what.items.len > 0) {
                            // Delete the last comma
                            _ = log_what.pop();

                            const log_flag = "-d";
                            kernel.run_argument_list.append(log_flag) catch unreachable;
                            kernel.debug_argument_list.append(log_flag) catch unreachable;
                            kernel.run_argument_list.append(log_what.items) catch unreachable;
                            kernel.debug_argument_list.append(log_what.items) catch unreachable;
                        }

                        if (log_options.file) |log_file| {
                            const log_file_flag = "-D";
                            kernel.run_argument_list.append(log_file_flag) catch unreachable;
                            kernel.debug_argument_list.append(log_file_flag) catch unreachable;
                            kernel.run_argument_list.append(log_file) catch unreachable;
                            kernel.debug_argument_list.append(log_file) catch unreachable;
                        }
                    }
                }

                common.QEMU.add_isa_debug_exit(kernel.builder.allocator, &kernel.run_argument_list) catch unreachable;
                kernel.run_argument_list.append("-trace") catch unreachable;
                kernel.run_argument_list.append("-nvme*") catch unreachable;
                kernel.debug_argument_list.append("-trace") catch unreachable;
                kernel.debug_argument_list.append("-nvme*") catch unreachable;
                kernel.run_argument_list.append("-trace") catch unreachable;
                kernel.run_argument_list.append("-pci*") catch unreachable;
                kernel.debug_argument_list.append("-trace") catch unreachable;
                kernel.debug_argument_list.append("-pci*") catch unreachable;

                kernel.debug_argument_list.append("-S") catch unreachable;
                kernel.debug_argument_list.append("-s") catch unreachable;
            },
        }

        print("Emulator command:\n", .{});
        for (kernel.run_argument_list.items) |arg| {
            print("{s} ", .{arg});
        }
        print("\n\n", .{});

        const run_command = kernel.builder.addSystemCommand(kernel.run_argument_list.items);
        run_command.step.dependOn(kernel.builder.default_step);
        run_command.step.dependOn(&kernel.disk_step);
        const run_step = kernel.builder.step("run", "run step");
        run_step.dependOn(&run_command.step);

        switch (kernel.options.arch) {
            .x86_64 => run_command.step.dependOn(&kernel.boot_image_step),
            else => unreachable,
        }

        var gdb_script_buffer = std.ArrayList(u8).init(kernel.builder.allocator);
        switch (kernel.options.arch) {
            .x86_64 => gdb_script_buffer.appendSlice("set disassembly-flavor intel\n") catch unreachable,
            else => {},
        }
        gdb_script_buffer.appendSlice(
            \\symbol-file zig-cache/kernel.elf
            \\target remote localhost:1234
            \\b start
            \\c
        ) catch unreachable;

        kernel.gdb_script = kernel.builder.addWriteFile("gdb_script", gdb_script_buffer.items);
        kernel.builder.default_step.dependOn(&kernel.gdb_script.step);

        // We need a member variable because we need consistent memory around it to do @fieldParentPtr
        kernel.debug_step = Step.init(.custom, "_debug_", kernel.builder.allocator, do_debug_step);
        kernel.debug_step.dependOn(&kernel.boot_image_step);
        kernel.debug_step.dependOn(&kernel.gdb_script.step);
        kernel.debug_step.dependOn(&kernel.disk_step);

        const debug_step = kernel.builder.step("debug", "Debug the program with QEMU and GDB");
        debug_step.dependOn(&kernel.debug_step);
    }

    const BootImage = struct {
        const x86_64 = struct {
            const Limine = struct {
                const build_installer_path = "src/build/arch/x86_64/limine/";
                const installer = @import("src/build/arch/x86_64/limine/installer.zig");
                const base_path = "src/common/arch/x86_64/limine/";
                const to_install_path = build_installer_path ++ "to_install/";
                const image_path = "zig-cache/universal.iso";

                fn new(kernel: *Kernel) Step {
                    var step = Step.init(.custom, "_limine_image_", kernel.builder.allocator, Limine.build);
                    step.dependOn(&kernel.executable.step);
                    return step;
                }

                fn build(step: *Step) !void {
                    const kernel = @fieldParentPtr(Kernel, "boot_image_step", step);
                    assert(kernel.options.arch == .x86_64);
                    const img_dir_path = kernel.builder.fmt("{s}/img_dir", .{kernel.builder.cache_root});
                    const cwd = std.fs.cwd();
                    cwd.deleteFile(image_path) catch {};
                    const img_dir = try cwd.makeOpenPath(img_dir_path, .{});
                    const img_efi_dir = try img_dir.makeOpenPath("EFI/BOOT", .{});
                    const limine_dir = try cwd.openDir(to_install_path, .{});

                    const limine_efi_bin_file = "limine-cd-efi.bin";
                    const files_to_copy_from_limine_dir = [_][]const u8{
                        "limine.cfg",
                        "limine.sys",
                        "limine-cd.bin",
                        limine_efi_bin_file,
                    };

                    for (files_to_copy_from_limine_dir) |filename| {
                        log.debug("Trying to copy {s}", .{filename});
                        try std.fs.Dir.copyFile(limine_dir, filename, img_dir, filename, .{});
                    }
                    try std.fs.Dir.copyFile(limine_dir, "BOOTX64.EFI", img_efi_dir, "BOOTX64.EFI", .{});
                    try std.fs.Dir.copyFile(cwd, kernel_path, img_dir, std.fs.path.basename(kernel_path), .{});

                    var xorriso_process = std.ChildProcess.init(&.{ "xorriso", "-as", "mkisofs", "-quiet", "-b", "limine-cd.bin", "-no-emul-boot", "-boot-load-size", "4", "-boot-info-table", "--efi-boot", limine_efi_bin_file, "-efi-boot-part", "--efi-boot-image", "--protective-msdos-label", img_dir_path, "-o", image_path }, kernel.builder.allocator);
                    // Ignore stderr and stdout
                    xorriso_process.stdin_behavior = std.ChildProcess.StdIo.Ignore;
                    xorriso_process.stdout_behavior = std.ChildProcess.StdIo.Ignore;
                    xorriso_process.stderr_behavior = std.ChildProcess.StdIo.Ignore;
                    _ = try xorriso_process.spawnAndWait();

                    try Limine.installer.install(image_path, false, null);
                }
            };
        };
    };

    const Disk = struct {
        fn create(kernel: *Kernel) void {
            kernel.disk_step = Step.init(.custom, "disk_create", kernel.builder.allocator, make);

            const named_step = kernel.builder.step("disk", "Create a disk blob to use with QEMU");
            named_step.dependOn(&kernel.disk_step);

            for (kernel.userspace_programs) |program| {
                kernel.disk_step.dependOn(&program.step);
            }
        }

        fn make(step: *Step) !void {
            const kernel = @fieldParentPtr(Kernel, "disk_step", step);
            const max_file_length = std.math.maxInt(usize);

            for (kernel.options.run.disks) |disk, disk_i| {
                const disk_memory = std.os.mmap(
                    null,
                    1 * common.gb,
                    std.os.PROT.READ | std.os.PROT.WRITE,
                    std.os.MAP.PRIVATE | std.os.MAP.ANONYMOUS,
                    -1,
                    0,
                ) catch unreachable;
                defer std.os.munmap(disk_memory);
                var build_disk_buffer = common.ArrayListAligned(u8, 0x1000){
                    .items = disk_memory,
                    .capacity = disk_memory.len,
                };
                build_disk_buffer.items.len = 0;
                var build_disk = BuildDisk.new(build_disk_buffer);
                var build_fs = drivers.RNUFS.Initialization.callback(kernel.builder.allocator, &build_disk.disk) catch @panic("wtf");

                for (disk.resource_files) |resource_file| {
                    const file = try std.fs.cwd().readFileAlloc(kernel.builder.allocator, kernel.builder.fmt("resources/{s}", .{resource_file}), max_file_length);
                    build_fs.fs.write_new_file(&build_fs.fs, 0, resource_file, file);
                }

                for (disk.userspace_programs) |userspace_program_name| {
                    const userspace_program = find_userspace_program(kernel, userspace_program_name) orelse @panic("wtf");
                    const exe_name = userspace_program.out_filename;
                    const exe_path = userspace_program.output_path_source.getPath();
                    const exe_file_content = try std.fs.cwd().readFileAlloc(kernel.builder.allocator, exe_path, std.math.maxInt(usize));
                    build_fs.fs.write_new_file(&build_fs.fs, 0, exe_name, exe_file_content);
                }

                const disk_size = build_disk.buffer.items.len;
                const disk_sector_count = common.bytes_to_sector(disk_size, build_disk.disk.sector_size, .must_be_exact);
                log.debug("Disk size: {}. Disk sector count: {}", .{ disk_size, disk_sector_count });

                try std.fs.cwd().writeFile(kernel.builder.fmt("zig-cache/disk{}.bin", .{disk_i}), build_disk.buffer.items);
            }
        }

        fn find_userspace_program(kernel: *Kernel, userspace_program_name: []const u8) ?*LibExeObjStep {
            for (kernel.userspace_programs) |userspace_program| {
                const ending = ".elf";
                common.runtime_assert(@src(), std.ascii.endsWithIgnoreCase(userspace_program.out_filename, ending));
                const name = userspace_program.out_filename[0 .. userspace_program.out_filename.len - ending.len];
                if (common.string_eq(name, userspace_program_name)) {
                    return userspace_program;
                }
            }

            return null;
        }
    };

    const Options = struct {
        arch: Options.ArchSpecific,
        run: RunOptions,

        const x86_64 = struct {
            bootloader: union(Bootloader) {
                limine: Limine,
            },

            const Bootloader = enum {
                limine,
            };

            fn new(context: anytype) Options.ArchSpecific {
                return switch (context.bootloader) {
                    .limine => .{
                        .x86_64 = .{
                            .bootloader = .{
                                .limine = .{
                                    .protocol = context.protocol,
                                },
                            },
                        },
                    },
                    else => unreachable,
                };
            }

            const Limine = struct {
                protocol: Protocol,

                const Protocol = enum(u32) {
                    stivale2,
                    limine,
                };
            };
        };

        const ArchSpecific = union(Arch) {
            arm,
            armeb,
            aarch64,
            aarch64_be,
            aarch64_32,
            arc,
            avr,
            bpfel,
            bpfeb,
            csky,
            hexagon,
            m68k,
            mips,
            mipsel,
            mips64,
            mips64el,
            msp430,
            powerpc,
            powerpcle,
            powerpc64,
            powerpc64le,
            r600,
            amdgcn,
            riscv32,
            riscv64: void,
            sparc,
            sparc64,
            sparcel,
            s390x,
            tce,
            tcele,
            thumb,
            thumbeb,
            i386,
            x86_64: x86_64,
            xcore,
            nvptx,
            nvptx64,
            le32,
            le64,
            amdil,
            amdil64,
            hsail,
            hsail64,
            spir,
            spir64,
            kalimba,
            shave,
            lanai,
            wasm32,
            wasm64,
            renderscript32,
            renderscript64,
            ve,
            // Stage1 currently assumes that architectures above this comment
            // map one-to-one with the ZigLLVM_ArchType enum.
            spu_2,
            spirv32,
            spirv64,
        };

        const RunOptions = struct {
            disks: []const DiskOptions,
            memory: Memory,
            emulator: union(enum) {
                qemu: QEMU,
            },

            const Memory = struct {
                amount: u64,
                unit: Unit,

                const Unit = enum(u3) {
                    K = 1,
                    M = 2,
                    G = 3,
                    T = 4,
                };
            };

            const QEMU = struct {
                vga: ?VGA,
                log: ?LogOptions,
                smp: ?u64,
                run_for_debug: bool,
                const VGA = enum {
                    std,
                    virtio,
                };
            };

            const DiskOptions = struct {
                interface: common.DiskDriverType,
                filesystem: common.FilesystemDriverType,
                userspace_programs: []const []const u8,
                resource_files: []const []const u8,
            };

            const LogOptions = struct {
                file: ?[]const u8,
                guest_errors: bool,
                cpu: bool,
                interrupts: bool,
                assembly: bool,
            };
        };
    };
};

const CPUFeatures = struct {
    enabled: std.Target.Cpu.Feature.Set,
    disabled: std.Target.Cpu.Feature.Set,
};

fn get_riscv_base_features() CPUFeatures {
    var features = CPUFeatures{
        .enabled = std.Target.Cpu.Feature.Set.empty,
        .disabled = std.Target.Cpu.Feature.Set.empty,
    };
    const Feature = std.Target.riscv.Feature;
    features.enabled.addFeature(@enumToInt(Feature.a));

    return features;
}

fn get_x86_base_features() CPUFeatures {
    var features = CPUFeatures{
        .enabled = std.Target.Cpu.Feature.Set.empty,
        .disabled = std.Target.Cpu.Feature.Set.empty,
    };

    const Feature = std.Target.x86.Feature;
    features.disabled.addFeature(@enumToInt(Feature.x87));
    features.disabled.addFeature(@enumToInt(Feature.mmx));
    features.disabled.addFeature(@enumToInt(Feature.sse));
    features.disabled.addFeature(@enumToInt(Feature.sse2));
    features.disabled.addFeature(@enumToInt(Feature.avx));
    features.disabled.addFeature(@enumToInt(Feature.avx2));

    features.enabled.addFeature(@enumToInt(Feature.soft_float));

    return features;
}

fn get_target_base(arch: Arch) std.zig.CrossTarget {
    const cpu_features = switch (arch) {
        .riscv64 => get_riscv_base_features(),
        .x86_64 => get_x86_base_features(),
        else => unreachable,
    };

    const target = std.zig.CrossTarget{
        .cpu_arch = arch,
        .os_tag = .freestanding,
        .abi = .none,
        .cpu_features_add = cpu_features.enabled,
        .cpu_features_sub = cpu_features.disabled,
    };

    return target;
}

fn do_debug_step(step: *Step) !void {
    const kernel = @fieldParentPtr(Kernel, "debug_step", step);
    switch (common.os) {
        .linux => {
            const gdb_script_path = kernel.gdb_script.getFileSource(kernel.gdb_script.files.first.?.data.basename).?.getPath(kernel.builder);
            const first_pid = try std.os.fork();
            if (first_pid == 0) {
                var debugger_process = std.ChildProcess.init(&[_][]const u8{ "gf2", "-x", gdb_script_path }, kernel.builder.allocator);
                _ = try debugger_process.spawnAndWait();
            } else {
                var qemu_process = std.ChildProcess.init(kernel.debug_argument_list.items, kernel.builder.allocator);
                try qemu_process.spawn();

                _ = std.os.waitpid(first_pid, 0);
                _ = try qemu_process.kill();
            }
        },
        else => unreachable,
    }
}

fn add_common_packages(executable: *LibExeObjStep) void {
    var context_package = Package{
        .name = "context",
        .source = .{ .path = "src/context.zig" },
        .dependencies = undefined,
    };
    var common_package = Package{
        .name = "common",
        .source = .{ .path = "src/common.zig" },
        .dependencies = undefined,
    };
    context_package.dependencies = &.{common_package};
    common_package.dependencies = &.{context_package};

    executable.addPackage(common_package);
    executable.addPackage(context_package);
}
