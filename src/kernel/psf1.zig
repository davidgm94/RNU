const kernel = @import("root");
const common = kernel.common;
const log = common.log.scoped(.PSF1);
const TODO = common.TODO;

pub const Header = struct {
    magic: [2]u8,
    mode: u8,
    char_size: u8,

    const magic = [_]u8{ 0x36, 0x04 };
};

pub const Font = struct {
    header: Header,
    glyph_buffer: []const u8,

    pub fn parse(file: []const u8) Font {
        const header = @ptrCast(*align(1) const Header, file.ptr).*;
        log.debug("Header: {}", .{header});

        if (header.magic[0] != Header.magic[0] or header.magic[1] != Header.magic[1]) @panic("magic PSF1 font corrupted");

        const glyph_count: u64 = if (header.mode == 1) 512 else 256;
        const glyph_buffer_size = header.char_size * glyph_count;

        const glyph_buffer = file[@sizeOf(Header) .. @sizeOf(Header) + glyph_buffer_size];

        return Font{
            .header = header,
            .glyph_buffer = glyph_buffer,
        };
    }
};
