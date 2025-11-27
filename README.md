# esp-rust-board-discovery

Embedded Rust with Rust ESP Board (ESP32-C3-DevKit-RUST-1)

## About

This project contains examples and tutorials for programming the Rust ESP Board (ESP32-C3-DevKit-RUST-1) using embedded Rust with the `esp-hal` hardware abstraction layer.

## Hardware

**Board:** ESP32-C3-DevKit-RUST-1 (Rust ESP Board)
- **MCU:** ESP32-C3 (RISC-V single-core processor)
- **On-board peripherals:**
  - WS2812 RGB LED on GPIO2
  - SHTC3 temperature/humidity sensor (I2C)
  - ICM42670p 6-axis IMU (I2C)
  - User button on GPIO9
- **I2C pins:**
  - SDA: GPIO10
  - SCL: GPIO8

## Examples

### Basic Examples

#### blinky
Blinks the on-board LED to verify your setup is working.

```bash
cargo run --example blinky
```

#### button
Reads the user button state and prints to console.

```bash
cargo run --example button
```

#### button-interrupt
Demonstrates interrupt-driven button handling for more efficient code.

```bash
cargo run --example button-interrupt
```

### WS2812 RGB LED

#### ws2812
Controls the on-board WS2812 addressable RGB LED on GPIO2. Displays a smooth rainbow animation cycling through all colors using HSV color space with gamma correction.

```bash
cargo run --example ws2812
```

**Features:**
- Uses ESP32 RMT peripheral for precise timing
- HSV to RGB color conversion
- Gamma correction for better color appearance
- Configurable brightness (default: 10/255)

### I2C Examples

#### i2c_scan
Scans the I2C bus for connected devices and prints their addresses. Useful for debugging I2C connections.

```bash
cargo run --example i2c_scan
```

**Expected devices on the Rust ESP Board:**
- `0x44` - SHTC3 temperature/humidity sensor
- `0x68` - ICM42670p IMU

#### shtc3
Reads temperature and humidity from the on-board SHTC3 sensor and prints values continuously.

```bash
cargo run --example shtc3
```

**Output:** Temperature in °C and humidity in %

#### icm42670p
Demonstrates reading both the SHTC3 sensor and the ICM42670p gyroscope simultaneously using shared I2C bus.

```bash
cargo run --example icm42670p
```

**Output:** Temperature, humidity, and gyroscope X/Y/Z values

### Display Examples (SSD1306 OLED)

These examples require an external 128x64 SSD1306 OLED display connected via I2C.

#### Wiring for SSD1306 Display

```
Display Pin -> Rust ESP Board
-----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO8
SDA (green) -> GPIO10
```

#### ssd1306
Displays a Rust logo image on the OLED screen.

```bash
cargo run --example ssd1306
```

**Features:**
- 1-bit black and white graphics
- Buffered rendering
- Static image display

#### ssd1306_text
Demonstrates text rendering and drawing shapes on the OLED display.

```bash
cargo run --example ssd1306_text
```

**Features:**
- Text rendering with built-in fonts
- Drawing primitives (lines, rectangles, circles)
- Shows "Rust ESP Board Demo" with graphics

## Dependencies

Key dependencies used in this project:

- **esp-hal** - Hardware abstraction layer for ESP32
- **embedded-hal** - Standard embedded traits
- **esp-hal-smartled** - WS2812/smart LED support via RMT
- **smart-leds** - Color manipulation and LED traits
- **ssd1306** - OLED display driver
- **embedded-graphics** - 2D graphics library
- **shtcx** - SHTC3 sensor driver
- **icm42670** - ICM42670 IMU driver
- **defmt** - Efficient logging framework

## Building and Flashing

To build and flash any example:

```bash
# Build only
cargo build --example <example_name>

# Build and flash to device
cargo run --example <example_name>
```

## Development Setup

1. Install Rust and cargo
2. Install espflash: `cargo install espflash`
3. Add RISC-V target: `rustup target add riscv32imc-unknown-none-elf`
4. Clone this repository
5. Connect your Rust ESP Board via USB
6. Run examples with `cargo run --example <name>`

## Resources

- [ESP-RS Book](https://docs.esp-rs.org/)
- [ESP-HAL Documentation](https://docs.esp-rs.org/esp-hal/)
- [Rust ESP Board GitHub](https://github.com/esp-rs/esp-rust-board)

## License

This project is licensed under the same terms as the Rust ESP Board examples.
