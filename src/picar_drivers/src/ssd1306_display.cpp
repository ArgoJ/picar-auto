#include "picar_drivers/ssd1306_display.hpp"

extern "C" {
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <zlib.h>
}

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace picar_drivers {

SSD1306Display::SSD1306Display(int bus_number, int address, const std::string &font_path)
{
    const std::string dev = "/dev/i2c-" + std::to_string(bus_number);
    file_ = open(dev.c_str(), O_RDWR);
    if (file_ < 0) {
        throw std::runtime_error("SSD1306: Failed to open " + dev + ": " + std::strerror(errno));
    }
    if (ioctl(file_, I2C_SLAVE, address) < 0) {
        const int err = errno;
        close(file_);
        file_ = -1;
        throw std::runtime_error("SSD1306: Failed to set I2C_SLAVE: " + std::string(std::strerror(err)));
    }

    // Standard 128x64 I2C initialization with internal charge pump enabled
    const uint8_t init_cmds[] = {
        0x00,             // Control byte: command stream
        0xAE, 0xD5, 0x80, // Display OFF, clock divide
        0xA8, 0x3F,       // Multiplex ratio 1/64
        0xD3, 0x00, 0x40, // Display offset 0, start line 0
        0x8D, 0x14,       // Enable charge pump
        0x20, 0x00,       // Horizontal addressing mode
        0xA1, 0xC8,       // Segment re-map, COM scan remapped
        0xDA, 0x12,       // COM pins (128x64)
        0x81, 0xCF,       // Contrast
        0xD9, 0xF1,       // Pre-charge period
        0xDB, 0x40,       // VCOMH deselect level
        0xA4, 0xA6, 0x2E, // RAM content, normal polarity, scroll off
        0xAF              // Display ON
    };

    if (!writeI2c(init_cmds, sizeof(init_cmds))) {
        close(file_);
        file_ = -1;
        throw std::runtime_error("SSD1306 not responding on " + dev);
    }

    if (!font_path.empty()) {
        loadPsfFont(font_path);
    }

    clear();
    display();
}

SSD1306Display::~SSD1306Display()
{
    if (file_ >= 0) {
        close(file_);
        file_ = -1;
    }
}

bool SSD1306Display::loadPsfFont(const std::string &font_path)
{
    gzFile gz = gzopen(font_path.c_str(), "rb");
    if (!gz) {
        return false;
    }

    uint8_t header[4]{};
    const int hdr_read = gzread(gz, header, sizeof(header));
    // Verify PSF1 magic (0x36, 0x04)
    if (hdr_read != 4 || header[0] != 0x36 || header[1] != 0x04 || header[3] == 0) {
        gzclose(gz);
        return false;
    }

    const size_t glyphs = (header[2] & 0x01) ? 512 : 256;
    const int height = header[3];
    std::vector<uint8_t> data(glyphs * static_cast<size_t>(height));
    const int bytes_read = gzread(gz, data.data(), static_cast<unsigned int>(data.size()));
    gzclose(gz);

    if (bytes_read != static_cast<int>(data.size())) {
        return false;
    }

    font_glyphs_ = std::move(data);
    font_height_ = height;
    glyph_count_ = glyphs;
    return true;
}

bool SSD1306Display::writeI2c(const uint8_t *data, size_t len)
{
    if (file_ < 0) {
        return false;
    }
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (write(file_, data, len) == static_cast<ssize_t>(len)) {
            return true;
        }
        usleep(2000);
    }
    return false;
}

void SSD1306Display::setPixel(int x, int y, bool on)
{
    if (x < 0 || x >= kWidth || y < 0 || y >= kHeight) {
        return;
    }
    const size_t idx = static_cast<size_t>(x + (y >> 3) * kWidth);
    const uint8_t bit = static_cast<uint8_t>(1u << (y & 7));
    buffer_[idx] = on ? (buffer_[idx] | bit) : static_cast<uint8_t>(buffer_[idx] & ~bit);
}

void SSD1306Display::fillRoundedRect(int x, int y, int w, int h, int radius, bool on)
{
    if (w <= 0 || h <= 0) {
        return;
    }
    const int r = std::clamp(radius, 0, std::min(w, h) / 2);
    const int r_sq = r * r;
    const int x0 = std::max(0, x), y0 = std::max(0, y);
    const int x1 = std::min(kWidth, x + w), y1 = std::min(kHeight, y + h);

    for (int py = y0; py < y1; ++py) {
        const int dy = py - y;
        const int cy = (dy < r) ? (r - dy) : (dy >= h - r ? dy - (h - r - 1) : 0);
        for (int px = x0; px < x1; ++px) {
            const int dx = px - x;
            const int cx = (dx < r) ? (r - dx) : (dx >= w - r ? dx - (w - r - 1) : 0);
            if (cx == 0 || cy == 0 || (cx * cx + cy * cy <= r_sq)) {
                setPixel(px, py, on);
            }
        }
    }
}

void SSD1306Display::drawText(int x, int y, const std::string &text, bool on)
{
    if (font_glyphs_.empty()) {
        return;
    }
    int cx = x, cy = y;
    for (char c : text) {
        if (c == '\n' || cx + 8 > kWidth) {
            cx = x;
            cy += font_height_;
            if (c == '\n') continue;
        }
        if (cy + font_height_ > kHeight) {
            break;
        }
        const uint8_t uc = static_cast<uint8_t>(c);
        const size_t idx = (uc < glyph_count_) ? uc : static_cast<size_t>(' ');
        const uint8_t *glyph = &font_glyphs_[idx * static_cast<size_t>(font_height_)];
        for (int row = 0; row < font_height_; ++row) {
            for (int col = 0; col < 8; ++col) {
                if (glyph[row] & (0x80u >> col)) {
                    setPixel(cx + col, cy + row, on);
                }
            }
        }
        cx += 8;
    }
}

bool SSD1306Display::display()
{
    const uint8_t addr_cmds[] = {0x00, 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07};
    if (!writeI2c(addr_cmds, sizeof(addr_cmds))) {
        return false;
    }

    constexpr size_t kChunk = 32;
    std::array<uint8_t, 1 + kChunk> packet{0x40};
    for (size_t offset = 0; offset < buffer_.size(); offset += kChunk) {
        const size_t n = std::min(kChunk, buffer_.size() - offset);
        std::copy_n(buffer_.data() + offset, n, packet.data() + 1);
        if (!writeI2c(packet.data(), 1 + n)) {
            return false;
        }
    }
    return true;
}

}  // namespace picar_drivers
