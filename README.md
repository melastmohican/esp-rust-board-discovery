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

#### bme280_i2c
Reads temperature, humidity, and atmospheric pressure from an external BME280 sensor. This is a popular environmental sensor that measures all three parameters in one device.

This example is configured for the **Adafruit BME280** breakout board connected via **Qwiic/STEMMA QT** cable.

```bash
cargo run --example bme280_i2c
```

**Hardware:**
- Sensor: Adafruit BME280 Temperature Humidity Pressure Sensor
- Connection: Qwiic/STEMMA QT cable (plug and play I2C connection)

**Wiring with Qwiic/STEMMA QT:**

Simply connect the Qwiic/STEMMA QT cable between the board and sensor - no separate wires needed!

```
BME280 Pin -> Rust ESP Board
----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO8
SDA (blue)  -> GPIO10
```

**I2C Address:**
- Adafruit BME280: `0x77` (configured in code)
- Generic modules: Often `0x76` (change to `BME280::new_primary()` in code)

**Output:** Temperature in °C, humidity in %, and atmospheric pressure in hPa

#### hs3003_i2c
Reads temperature and humidity from the Renesas HS3003 sensor using a custom driver implementation. This example is configured for the **Arduino Modulino Thermo** connected via **Qwiic/STEMMA QT** cable.

```bash
cargo run --example hs3003_i2c
```

**Hardware:**
- Sensor: Arduino Modulino Thermo (Renesas HS3003)
- Connection: Qwiic/STEMMA QT cable (plug and play I2C connection)

**Wiring with Qwiic/STEMMA QT:**

Simply connect the Qwiic/STEMMA QT cable between the board and Modulino Thermo - no separate wires needed!

```
Modulino Pin -> Rust ESP Board
------------    --------------
GND (black)  -> GND
VCC (red)    -> 3.3V
SCL (yellow) -> GPIO8
SDA (blue)   -> GPIO10
```

**I2C Address:**
- HS3003: `0x44` (fixed address)

**Features:**
- 14-bit resolution for temperature and humidity
- Temperature: -40°C to +125°C (±0.2°C accuracy)
- Humidity: 0-100% RH (±1.5% accuracy)
- Custom driver implementation (no external crate needed)

**Output:** Temperature in °C and humidity in %

### Display Examples (SSD1306 OLED - I2C)

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

### Display Examples (ST7735S LCD - SPI)

These examples require a Waveshare 0.96 inch LCD module (80x160 pixels) with ST7735S controller connected via SPI.

**Note:** For round 240x240 displays, see the GC9A01 examples below.

#### Wiring for Waveshare 0.96" LCD Module

```
LCD Pin -> ESP32-C3-DevKit-RUST-1
-------    ----------------------
VCC     -> 3.3V
GND     -> GND
DIN     -> GPIO7  (MOSI)
CLK     -> GPIO6  (SCK)
CS      -> GPIO5  (Chip Select)
DC      -> GPIO4  (Data/Command)
RST     -> GPIO3  (Reset)
BL      -> GPIO2  (Backlight)
```

**Pin Functions:**
- **VCC**: Power supply (3.3V)
- **GND**: Ground
- **DIN (MOSI)**: SPI data input - transmits pixel data
- **CLK (SCK)**: SPI clock - synchronizes data transmission
- **CS**: Chip select (active low) - enables the display
- **DC**: Data/Command select (Low=Command, High=Data)
- **RST**: Reset (active low) - resets the display controller
- **BL**: Backlight control (HIGH=on, LOW=off)

**SPI Configuration:**
- SPI Mode: 0 (CPOL=0, CPHA=0)
- Clock Speed: 26 MHz
- Display Resolution: 80x160 pixels (landscape mode)
- Color Format: RGB565 (16-bit color, 65,536 colors)

#### st7735s_spi
Displays Ferris and Rust logo images on the color LCD.

```bash
cargo run --example st7735s_spi
```

**Features:**
- Full RGB565 color support (16-bit, 65K colors)
- Displays raw RGB565 image format (Ferris)
- Displays BMP image format (Rust logo)
- Hardware SPI for fast rendering

#### st7735s_spi_text
Demonstrates text rendering and colorful shapes on the LCD display.

```bash
cargo run --example st7735s_spi_text
```

**Features:**
- Multiple font sizes (6x10, 9x15 bold)
- Text styling with colors and backgrounds
- Drawing colorful primitives (rectangles, circles, lines)
- Multiple colors: white, blue, yellow, green, red, orange
- Shows "Rust ESP Board" title with graphics

### Display Examples (ILI9341 TFT LCD - SPI)

These examples require an ILI9341 TFT LCD module (240x320 pixels) connected via SPI. This is a common rectangular display used in many projects.

**Hardware:** 2.8" TFT SPI 240x320 V1.2 Display Module

This module includes a resistive touchscreen (T_CLK, T_CS, T_DIN, T_DO, T_IRQ pins) which is not used in these basic display examples.

#### Wiring for 2.8" TFT SPI 240x320 V1.2 Module

```
LCD Pin     -> ESP32-C3-DevKit-RUST-1
----------     ----------------------
VCC         -> 3.3V
GND         -> GND
CS          -> GPIO5  (Chip Select)
RESET       -> GPIO3  (Reset)
DC          -> GPIO4  (Data/Command)
SDI (MOSI)  -> GPIO7  (SPI MOSI/Data)
SCK         -> GPIO6  (SPI Clock)
LED         -> 3.3V  (Backlight)
SDO (MISO)  -> GPIO1  (optional)

Touchscreen pins (not connected in these examples):
T_CLK       -> (not used)
T_CS        -> (not used)
T_DIN       -> (not used)
T_DO        -> (not used)
T_IRQ       -> (not used)
```

**Pin Functions:**
- **VCC**: Power supply (3.3V or 5V depending on module)
- **GND**: Ground
- **CS**: Chip select for LCD (active low)
- **RESET**: Reset (active low)
- **DC**: Data/Command select (Low=Command, High=Data)
- **SDI (MOSI)**: SPI data input to display
- **SCK**: SPI clock
- **LED**: Backlight power (can connect to GPIO for PWM control)
- **SDO (MISO)**: SPI data output (optional, rarely needed)

**SPI Configuration:**
- SPI Mode: 0 (CPOL=0, CPHA=0)
- Clock Speed: 40 MHz (ILI9341 supports up to 60 MHz)
- Display Resolution: 240x320 pixels (portrait mode)
- Color Format: RGB565 (16-bit color, 65,536 colors)

#### ili9341_spi
Displays colorful text and Rust logo on the TFT LCD.

```bash
cargo run --example ili9341_spi
```

**Features:**
- Full RGB565 color support (16-bit, 65K colors)
- 240x320 pixel rectangular display
- Multiple colored text examples
- Displays BMP image format (Rust logo)
- High-speed 40 MHz SPI for fast rendering

#### ili9341_spi_text
Demonstrates comprehensive text rendering and colorful shapes on the LCD.

```bash
cargo run --example ili9341_spi_text
```

**Features:**
- Multiple font sizes (6x10, 9x15 bold, 10x20)
- Text styling with colors and backgrounds
- Drawing primitives (rectangles, circles, lines)
- Multiple colors: red, green, blue, yellow, cyan, magenta, white
- Title bar with background color
- Demonstrates rectangular layout for portrait display

#### zermatt
Displays a full-screen 320x240 landscape image of Zermatt on the ILI9341 display.

```bash
cargo run --example zermatt
```

**Features:**
- Full-screen landscape image display (320×240)
- Uses BMP format with tinybmp library
- Demonstrates landscape orientation (90° rotation)
- BGR color order for correct colors
- Includes Python script for JPEG to BMP conversion

**Image Conversion:**
```bash
python3 examples/convert_jpg_to_bmp.py examples/zermatt_320x240.jpg examples/zermatt_320x240.bmp
```

### Display Examples (GC9A01 Round LCD - SPI)

These examples require a GC9A01 round LCD module (240x240 pixels) connected via SPI. This is a circular display commonly used in smartwatches and circular gauge displays.

**Hardware:** UNI128-240240-RGB-7-V1.0 Display Module (7 pins)

**Important Note:** Despite the module having pins labeled **SCL/SDA**, this is an **SPI display**, not I2C! The presence of DC (Data/Command) and CS (Chip Select) pins confirms it's SPI. The pin labels mean:
- **SCL** = SPI Clock (same as SCK/SCLK)
- **SDA** = SPI Data (same as MOSI - Master Out Slave In)

#### Wiring for UNI128-240240-RGB-7-V1.0 Module

```
LCD Pin -> ESP32-C3-DevKit-RUST-1
-------    ----------------------
VCC     -> 3.3V
GND     -> GND
SCL     -> GPIO6  (SPI Clock)
SDA     -> GPIO7  (SPI MOSI/Data)
DC      -> GPIO4  (Data/Command)
CS      -> GPIO5  (Chip Select)
RST     -> GPIO3  (Reset)
```

**Pin Functions:**
- **VCC**: Power supply (3.3V)
- **GND**: Ground
- **SCL**: SPI clock (labeled SCL but it's actually SPI SCK)
- **SDA**: SPI data output (labeled SDA but it's actually SPI MOSI)
- **DC**: Data/Command select (Low=Command, High=Data)
- **CS**: Chip select (active low) - enables the display
- **RST**: Reset (active low) - resets the display controller

**Note:** This 7-pin module has no separate backlight control pin - the backlight is always on when powered.

**SPI Configuration:**
- SPI Mode: 0 (CPOL=0, CPHA=0)
- Clock Speed: 60 MHz (GC9A01 supports up to 62.5 MHz)
- Display Resolution: 240x240 pixels (round/circular)
- Color Format: RGB565 (16-bit color, 65,536 colors)
- Display Shape: Circular (visible area is round)

#### gc9a01_spi
Displays Ferris and Rust logo images on the round color LCD.

```bash
cargo run --example gc9a01_spi
```

**Features:**
- Full RGB565 color support (16-bit, 65K colors)
- 240x240 pixel round display
- Displays raw RGB565 image format (Ferris)
- Displays BMP image format (Rust logo)
- High-speed 60 MHz SPI for fast rendering
- Uses mipidsi driver for robust display control

#### gc9a01_spi_text
Demonstrates text rendering and colorful shapes optimized for the circular display.

```bash
cargo run --example gc9a01_spi_text
```

**Features:**
- Multiple font sizes (6x10, 9x15 bold, 10x20)
- Text styling with colors and backgrounds
- Circular shapes to match the round display form factor
- Drawing primitives optimized for circular layout
- Multiple colors: white, blue, yellow, green, red, cyan, magenta, orange
- Shows "GC9A01 Display" title with circular graphics
- Demonstrates lines radiating from center

## Dependencies

Key dependencies used in this project:

- **esp-hal** - Hardware abstraction layer for ESP32
- **embedded-hal** - Standard embedded traits
- **esp-hal-smartled** - WS2812/smart LED support via RMT
- **smart-leds** - Color manipulation and LED traits
- **ssd1306** - OLED display driver (I2C)
- **st7735-lcd** - ST7735S color LCD driver (SPI)
- **mipidsi** - Universal MIPI display driver (supports GC9A01 and many others)
- **display-interface-spi** - SPI display interface for mipidsi
- **embedded-graphics** - 2D graphics library for displays
- **tinybmp** - BMP image format support
- **shtcx** - SHTC3 sensor driver
- **icm42670** - ICM42670 IMU driver
- **bme280** - BME280 environmental sensor driver
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
2. Install probe-rs: `cargo install probe-rs-tools --locked`
3. Add RISC-V target: `rustup target add riscv32imc-unknown-none-elf`
4. Clone this repository
5. Connect your Rust ESP Board via USB
6. Run examples with `cargo run --example <name>`

**Note:** This project uses probe-rs for flashing and debugging. The `.cargo/config.toml` is configured to use probe-rs as the runner.

## Resources

- [ESP-RS Book](https://docs.esp-rs.org/)
- [ESP-HAL Documentation](https://docs.esp-rs.org/esp-hal/)
- [Rust ESP Board GitHub](https://github.com/esp-rs/esp-rust-board)

## License

This project is licensed under the same terms as the Rust ESP Board examples.
