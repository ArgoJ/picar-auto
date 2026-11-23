#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace picar_drivers {

/**
 * @brief I2C driver and 128x64 monochrome framebuffer for SSD1306 OLED displays.
 */
class SSD1306Display {
public:
    static constexpr int kWidth = 128;
    static constexpr int kHeight = 64;
    static constexpr int kBufferSize = kWidth * (kHeight / 8);

    explicit SSD1306Display(
        int bus_number = 1,
        int address = 0x3C,
        const std::string &font_path = "/usr/share/consolefonts/Lat15-VGA8.psf.gz");
    ~SSD1306Display();

    SSD1306Display(const SSD1306Display &) = delete;
    SSD1306Display &operator=(const SSD1306Display &) = delete;

    /** @brief Load an 8-px wide PSF1 console font (.psf or .psf.gz). */
    bool loadPsfFont(const std::string &font_path);

    /** @brief Clear the framebuffer (all pixels off). */
    void clear() { buffer_.fill(0x00); }

    /** @brief Set or clear a pixel at (x, y). */
    void setPixel(int x, int y, bool on = true);

    /** @brief Draw a filled rounded rectangle (radius = 0 for sharp corners). */
    void fillRoundedRect(int x, int y, int w, int h, int radius = 0, bool on = true);

    /** @brief Draw wrapped ASCII text using the loaded PSF1 console font. */
    void drawText(int x, int y, const std::string &text, bool on = true);

    /** @brief Flush the framebuffer to the SSD1306 GDDRAM over I2C. */
    bool display();

private:
    bool writeI2c(const uint8_t *data, size_t len);

    int file_{-1};
    std::array<uint8_t, kBufferSize> buffer_{};
    std::vector<uint8_t> font_glyphs_;
    int font_height_{8};
    size_t glyph_count_{0};
};

}  // namespace picar_drivers
