const kernel = @import("root");
const common = @import("common");
const Font = kernel.PSF1.Font;

const log = common.log.scoped(.graphics);
const Driver = @This();

const Type = enum(u64) {
    virtio = 0,
};

type: Type,
framebuffer: Framebuffer,

pub fn draw_char(driver: *Driver, color: Color, point: Point, character: u8) void {
    const framebuffer = driver.framebuffer.buffer[0 .. driver.framebuffer.width * driver.framebuffer.height];
    const font_buffer_char_offset = @intCast(u64, character) * kernel.font.header.char_size;
    const font_buffer = kernel.font.glyph_buffer[font_buffer_char_offset .. font_buffer_char_offset + kernel.font.header.char_size];

    for (font_buffer[0..16]) |font_byte, offset_from_y| {
        const y = point.y + offset_from_y;
        var x = point.x;
        const x_max = point.x + 8;

        while (x < x_max) : (x += 1) {
            if (font_byte & (@as(u8, 0b1000_0000) >> @intCast(u3, x - point.x)) != 0) {
                // TODO: correct the color
                framebuffer[x + (y * driver.framebuffer.width)] = @bitCast(u32, color);
            }
        }
    }
}

pub fn draw_string(driver: *Driver, color: Color, string: []const u8) void {
    const framebuffer = &driver.framebuffer;
    for (string) |char| {
        driver.draw_char(color, framebuffer.cursor, char);
        framebuffer.cursor.x += 8;

        if (driver.framebuffer.cursor.x + 8 > driver.framebuffer.width) {
            driver.framebuffer.cursor.x = 0;
            driver.framebuffer.cursor.y += 16;
        }
    }
}
pub fn draw_horizontal_line(driver: *Driver, line: Line, color: Color) void {
    kernel.assert(@src(), line.start.y == line.end.y);
    kernel.assert(@src(), line.start.x < line.end.x);
    const length = line.end.x - line.start.x;
    const start_i = line.start.x + (line.start.y * driver.framebuffer.width);
    for (driver.framebuffer.buffer[start_i .. start_i + length]) |*pixel| {
        pixel.* = @bitCast(u32, color);
    }
}

/// This assumes they are all the same height (start.y and end.y are the same for all of them)
pub fn draw_parallel_vertical_lines(driver: *Driver, x_coordinates: []u32, height_start: u32, height_end: u32, color: Color) void {
    var y_offset = height_start * driver.framebuffer.width;
    const y_max_offset = height_end * driver.framebuffer.width;

    while (y_offset < y_max_offset) : (y_offset += driver.framebuffer.width) {
        for (x_coordinates) |x| {
            const index = x + y_offset;
            driver.framebuffer.buffer[index] = @bitCast(u32, color);
        }
    }
}

pub fn draw_rect(driver: *Driver, rect: Rect, color: Color) void {
    driver.draw_horizontal_line(Line.straight_horizontal(rect.x, rect.y, rect.width), color);
    driver.draw_parallel_vertical_lines(&[_]u32{ rect.x, rect.x + rect.width }, rect.y, rect.y + rect.height, color);
    driver.draw_horizontal_line(Line.straight_horizontal(rect.x, rect.y + rect.height, rect.width), color);
}

pub fn test_draw_rect(driver: *Driver) void {
    draw_rect(driver, Rect{
        .x = 600,
        .y = 600,
        .width = 30,
        .height = 60,
    }, Color{ .red = 0, .green = 0, .blue = 0, .alpha = 0 });
}

pub const Framebuffer = struct {
    buffer: [*]u32,
    width: u32,
    height: u32,
    cursor: Point,
};

pub const Point = struct {
    x: u32,
    y: u32,
};

pub const Color = struct {
    red: u8,
    green: u8,
    blue: u8,
    alpha: u8,
};

pub const Rect = struct {
    x: u32,
    y: u32,
    width: u32,
    height: u32,
};

pub const Line = struct {
    start: Point,
    end: Point,

    fn straight_horizontal(x_start: u32, y: u32, width: u32) Line {
        return Line{
            .start = Point{ .x = x_start, .y = y },
            .end = Point{ .x = x_start + width, .y = y },
        };
    }

    fn straight_vertical(x: u32, y_start: u32, height: u32) Line {
        return Line{
            .start = Point{ .x = x, .y = y_start },
            .end = Point{ .x = x, .y = y_start + height },
        };
    }
};

pub var drivers: []*Driver = undefined;
pub var _drivers_array: [64]*Driver = undefined;
