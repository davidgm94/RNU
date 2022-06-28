const kernel = @import("root");
const common = kernel.common;
const log = common.log.scoped(.ELF);

const FileHeader = extern struct {
    // e_ident
    magic: u8 = magic,
    elf_id: [3]u8 = elf_signature.*,
    bit_count: u8 = @enumToInt(Bits.b64),
    endianness: u8 = @enumToInt(Endianness.little),
    header_version: u8 = 1,
    os_abi: u8 = @enumToInt(ABI.SystemV),
    abi_version: u8 = 0,
    padding: [7]u8 = [_]u8{0} ** 7,
    object_type: u16 = @enumToInt(ObjectFileType.executable), // e_type
    machine: u16 = @enumToInt(Machine.AMD64),
    version: u32 = 1,
    entry: u64,
    program_header_offset: u64 = 0x40,
    section_header_offset: u64,
    flags: u32 = 0,
    header_size: u16 = 0x40,
    program_header_size: u16 = @sizeOf(ProgramHeader),
    program_header_entry_count: u16 = 1,
    section_header_size: u16 = @sizeOf(SectionHeader),
    section_header_entry_count: u16,
    name_section_header_index: u16,

    const magic = 0x7f;
    const elf_signature = "ELF";
    const Bits = enum(u8) {
        b32 = 1,
        b64 = 2,
    };

    const Endianness = enum(u8) {
        little = 1,
        big = 2,
    };

    const ABI = enum(u8) {
        SystemV = 0,
    };

    const ObjectFileType = enum(u16) {
        none = 0,
        relocatable = 1,
        executable = 2,
        dynamic = 3,
        core = 4,
        lo_os = 0xfe00,
        hi_os = 0xfeff,
        lo_proc = 0xff00,
        hi_proc = 0xffff,
    };

    const Machine = enum(u16) {
        AMD64 = 0x3e,
    };
};

const ProgramHeader = extern struct {
    type: u32 = @enumToInt(ProgramHeaderType.load),
    flags: u32 = @enumToInt(Flags.readable) | @enumToInt(Flags.executable),
    offset: u64,
    virtual_address: u64,
    physical_address: u64,
    size_in_file: u64,
    size_in_memory: u64,
    alignment: u64 = 0,

    const ProgramHeaderType = enum(u32) {
        @"null" = 0,
        load = 1,
        dynamic = 2,
        interpreter = 3,
        note = 4,
        shlib = 5, // reserved
        program_header = 6,
        tls = 7,
        lo_os = 0x60000000,
        hi_os = 0x6fffffff,
        lo_proc = 0x70000000,
        hi_proc = 0x7fffffff,
    };

    const Flags = enum(u8) {
        executable = 1,
        writable = 2,
        readable = 4,
    };
};

const SectionHeader = extern struct {
    name_offset: u32,
    type: u32,
    flags: u64,
    address: u64,
    offset: u64,
    size: u64,
    // section index
    link: u32,
    info: u32,
    alignment: u64,
    entry_size: u64,

    // type
    const ID = enum(u32) {
        @"null" = 0,
        program_data = 1,
        symbol_table = 2,
        string_table = 3,
        relocation_entries_addends = 4,
        symbol_hash_table = 5,
        dynamic_linking_info = 6,
        notes = 7,
        program_space_no_data = 8,
        relocation_entries = 9,
        reserved = 10,
        dynamic_linker_symbol_table = 11,
        array_of_constructors = 14,
        array_of_destructors = 15,
        array_of_pre_constructors = 16,
        section_group = 17,
        extended_section_indices = 18,
        number_of_defined_types = 19,
        start_os_specific = 0x60000000,
    };

    const Flag = enum(u64) {
        writable = 0x01,
        alloc = 0x02,
        executable = 0x04,
        mergeable = 0x10,
        contains_null_terminated_strings = 0x20,
        info_link = 0x40,
        link_order = 0x80,
        os_non_conforming = 0x100,
        section_group = 0x200,
        tls = 0x400,
        mask_os = 0x0ff00000,
        mask_processor = 0xf0000000,
        ordered = 0x4000000,
        exclude = 0x8000000,
    };
};
pub fn parse(file: []const u8) void {
    const file_header = @ptrCast(*align(1) const FileHeader, file.ptr);
    if (file_header.magic != FileHeader.magic) @panic("magic");
    if (!kernel.string_eq(&file_header.elf_id, FileHeader.elf_signature)) @panic("signature");
    log.debug("Parsed so far the kernel ELF file\n{}", .{file_header});
}
