// =====================================================================
//  Simple FPV Cockpit Display - ESP32-S3 / 2.8" ILI9341 SPI
//
//  A MAVLink artificial horizon for ArduPilot, meant to be filmed by
//  an FPV camera. Shows roll and pitch, airspeed, altitude above home,
//  arm state and flight mode, with pop-up warnings for low battery and
//  low altitude.
//
//  Ported from "Simple FPV Cockpit Display" by Keith Luneau
//  (aka Bonafidepirate), originally written for the LilyGo
//  T-Display-S3 and its 8-bit parallel screen:
//    https://github.com/keithluneau/Simple-FPV-Cockpit-Display
//  MIT License. See LICENSE.
//
//  Full wiring, setup and troubleshooting: see README.md
//
//  QUICK WIRING - DISPLAY (2.8" 240x320 ILI9341 SPI module)
//    Display        ESP32-S3 Super Mini     Header
//    ------------   ---------------------   ------
//    VCC            3V3                     left
//    GND            GND                     left
//    SDO (MISO)     GPIO 13                 left
//    SCK            GPIO 12                 left
//    SDI (MOSI)     GPIO 11                 left
//    CS             GPIO 10                 left
//    RESET          GPIO 9                  left
//    LED            GPIO 8                  left
//    DC / RS        GPIO 4                  right
//
//  QUICK WIRING - FLIGHT CONTROLLER
//    Telemetry TX  ->  GPIO 44 (RX)
//    GND           ->  GND
//    5V            ->  5V / VBUS
//
//    WARNING: do NOT power the board from USB while the flight
//    controller's 5V is connected. Two 5V sources tied together risk
//    feeding current back into the flight controller. Unplug the 5V
//    wire before connecting USB.
//
//  ARDUINO IDE
//    Board: ESP32S3 Dev Module | USB CDC On Boot: Enabled
//    CPU: 240 MHz | PSRAM: QSPI PSRAM on a Super Mini, or Disabled
//    Libraries: LovyanGFX, MAVLink (by Oleg Kalachev)
// =====================================================================

#include <LovyanGFX.hpp>
#include <MAVLink.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// B612, the typeface Airbus and ENAC designed for cockpit screens.
// Generated from the official TTF by tools/ttf2gfx.py; see FONTS below.
#include "B612Bold28.h"
#include "B612Bold20.h"
#include "B612Bold16.h"


// =====================================================================
//  USER SETTINGS
// =====================================================================

// Every setting has its default in a "default:" comment, so you can
// always get back to a known starting point.

// ---------------------------------------------------------------------
//  Display pins and bus
// ---------------------------------------------------------------------

const int TFT_SCLK = 12;   // default: 12
const int TFT_MOSI = 11;   // default: 11
const int TFT_MISO = 13;   // default: 13  (-1 if not connected)
const int TFT_CS   = 10;   // default: 10
const int TFT_DC   = 4;    // default: 4
const int TFT_RST  = 9;    // default: 9   (-1 if tied to the S3 reset)
const int TFT_BL   = 8;    // default: 8   (-1 if LED is tied to 3V3)

// SPI write clock. 40 MHz is reliable on essentially any module and any
// wiring, and is the right place to start.
//
// 80000000 roughly halves the time it takes to push a frame, but it is
// past what the ILI9341 is specified for and depends entirely on your
// wiring. On long dupont jumpers it can drop or add a bit mid-frame,
// which shifts the rest of that frame sideways and looks like the
// picture flickering left and right. Short, direct wiring usually
// takes it. Try it, and come back to 40000000 at the first sign of
// torn pixels, speckles or a sideways jitter.
const uint32_t TFT_SPI_FREQ = 40000000;   // default: 40000000

// Display rotation. 1 = landscape 320x240. 3 = the other way up.
const uint8_t TFT_ROTATION = 1;           // default: 1

// Some ILI9341 modules ship with inverted colors. If the sky is orange
// and the ground is blue, flip this.
const bool TFT_INVERT = false;            // default: false

// ---------------------------------------------------------------------
//  Frame buffering
// ---------------------------------------------------------------------

// The frame is drawn off-screen and pushed in one go. A full 320x240
// 16-bit buffer is 150 KB, which fits in the S3's SRAM.
//    1 band  = 150 KB  - one full-screen buffer (fastest)
//    2 bands =  75 KB
//    4 bands =  38 KB
// If allocation fails the sketch doubles the band count and retries,
// up to 16, and says so over serial. More bands means the vector
// drawing is repeated more times, so only raise it if you must.
const int FRAME_BANDS = 1;                // default: 1

// Keep this false. The S3's PSRAM is much slower to draw into than its
// SRAM, and the buffer fits in SRAM, so PSRAM costs frame rate for no
// benefit. Set true only if you need the SRAM for something else.
const bool USE_PSRAM = false;             // default: false

// ---------------------------------------------------------------------
//  Flight controller link
// ---------------------------------------------------------------------

// RX receives the flight controller's TX.
// TX only needs to be wired if you use the WiFi bridge below; the
// display itself never talks back.
const int MAV_RX_PIN = 44;            // default: 44
const int MAV_TX_PIN = 43;            // default: 43
const uint32_t MAV_BAUD = 115200;     // default: 115200

// Serial receive buffer, in bytes. This has to cover the time the
// board spends pushing a frame, around 25 ms, because nothing is read
// off the port while that happens. The Arduino default of 256 bytes is
// only 22 ms at 115200 and does not cover it once a ground station is
// also pulling parameters.
//
//   buffer   holds at 115200   holds at 921600
//     256           22 ms             3 ms     <- the default; too small
//    2048          178 ms            22 ms     <- too tight at 921600
//    8192          711 ms            89 ms     <- comfortable at both
//
// If you raise SERIALn_BAUD on the flight controller, keep an eye on
// the right-hand column.
const size_t MAV_RX_BUFFER = 8192;    // default: 8192

// Serial transmit buffer, in bytes. Without one, writing to the port
// blocks until the hardware has clocked the bytes out, which at 115200
// is about 9 ms for every 100 bytes - long enough to stall a frame.
// With a buffer, writes return immediately until it is full.
const size_t MAV_TX_BUFFER = 2048;    // default: 2048

// Demo-mode button, active low with an internal pull-up. GPIO 0 is the
// BOOT button on essentially every ESP32-S3 board. -1 disables it.
const int BUTTON_PIN = 0;                 // default: 0
const uint32_t BUTTON_DEBOUNCE_MS = 40;   // default: 40

// ---------------------------------------------------------------------
//  WiFi MAVLink bridge
// ---------------------------------------------------------------------

// Lets Mission Planner, QGroundControl or MAVProxy reach the flight
// controller over WiFi, through this board, while the cockpit display
// carries on as normal. The bridge forwards raw MAVLink bytes in both
// directions and does not care what is in them, so MAVLink 2, signing
// and any dialect all pass through untouched.
//
//   *** THIS NEEDS ONE EXTRA WIRE ***
//
// The display only ever listens, so a display-only build needs just
// the flight controller's TX. A ground station has to talk back, so
// the board's TX must now reach the flight controller's RX:
//
//     ESP32-S3 GPIO 43 (TX)  ->  flight controller telemetry RX
//
// Without that wire you still get live telemetry in Mission Planner,
// but it will sit at "getting params" forever and you will not be able
// to change anything on the aircraft.
//
// Set this false to get the display-only build back, with no WiFi
// started and no radio power drawn.
const bool WIFI_BRIDGE_ENABLE = true;        // default: true

// BRIDGE_AP  - the board makes its own network. Best in the field: no
//              router needed, and the board is always at 192.168.4.1.
// BRIDGE_STA - the board joins a network you already have. Best at the
//              bench, and lets you stay on the internet at the same
//              time. If it cannot join within WIFI_STA_TIMEOUT_MS the
//              sketch gives up and runs as a plain display.
#define BRIDGE_AP   1
#define BRIDGE_STA  2
#define WIFI_BRIDGE_MODE  BRIDGE_AP          // default: BRIDGE_AP

// In BRIDGE_AP mode: the network the board creates.
// In BRIDGE_STA mode: the network it joins.
// The password must be at least 8 characters. In AP mode an empty
// string makes an open network, which also lets anyone within range
// command your aircraft, so set one.
const char* WIFI_SSID = "cockpit-display";   // default: cockpit-display
const char* WIFI_PASS = "fpvcockpit";        // default: fpvcockpit

const uint32_t WIFI_STA_TIMEOUT_MS = 12000;  // default: 12000
const uint8_t  WIFI_AP_CHANNEL = 6;          // default: 6

// UDP port. 14550 is the MAVLink standard and what Mission Planner
// offers when you pick UDP.
const uint16_t WIFI_UDP_PORT = 14550;        // default: 14550

// Telemetry is gathered for up to this long before being sent as one
// UDP packet, instead of firing a packet off per handful of bytes.
// Higher batches harder and adds that much latency to the ground
// station; it has no effect at all on the display.
const uint32_t WIFI_FLUSH_MS = 10;           // default: 10

// A ground station is treated as gone this long after its last packet,
// after which telemetry goes back out as a broadcast.
const uint32_t WIFI_GCS_TIMEOUT_MS = 5000;   // default: 5000

// Show a small WIFI / GCS tag in the top right corner of the screen.
const bool WIFI_SHOW_STATUS = true;          // default: true

// ---------------------------------------------------------------------
//  Units
// ---------------------------------------------------------------------

enum SpeedUnit { SPEED_KMH, SPEED_MS, SPEED_MPH, SPEED_KNOTS };
enum AltUnit   { ALT_METERS, ALT_FEET };

// ONE SWITCH FOR THE WHOLE DISPLAY. Change this single line and the
// speed unit, the altitude unit and the low-altitude warning limit all
// move together, so you never end up warning at 50 metres because the
// number was written for feet.
//
//   UNITS_METRIC     km/h  and metres   (warn below 15 m)
//   UNITS_IMPERIAL   mph   and feet     (warn below 50 ft)
//   UNITS_AVIATION   knots and feet     (warn below 50 ft)
//   UNITS_CUSTOM     set the three values yourself, below
#define UNITS_METRIC    1
#define UNITS_IMPERIAL  2
#define UNITS_AVIATION  3
#define UNITS_CUSTOM    4

#define UNIT_SYSTEM  UNITS_METRIC          // default: UNITS_METRIC

#if   UNIT_SYSTEM == UNITS_METRIC
  const SpeedUnit SPEED_UNIT = SPEED_KMH;
  const AltUnit   ALT_UNIT   = ALT_METERS;
  const float     WARN_ALT   = 15.0f;      // metres
#elif UNIT_SYSTEM == UNITS_IMPERIAL
  const SpeedUnit SPEED_UNIT = SPEED_MPH;
  const AltUnit   ALT_UNIT   = ALT_FEET;
  const float     WARN_ALT   = 50.0f;      // feet
#elif UNIT_SYSTEM == UNITS_AVIATION
  const SpeedUnit SPEED_UNIT = SPEED_KNOTS;
  const AltUnit   ALT_UNIT   = ALT_FEET;
  const float     WARN_ALT   = 50.0f;      // feet
#elif UNIT_SYSTEM == UNITS_CUSTOM
  // Your own combination. WARN_ALT is in whichever ALT_UNIT you pick,
  // and 0 disables the low-altitude warning entirely.
  const SpeedUnit SPEED_UNIT = SPEED_KMH;
  const AltUnit   ALT_UNIT   = ALT_METERS;
  const float     WARN_ALT   = 15.0f;
#else
  #error "UNIT_SYSTEM must be UNITS_METRIC, UNITS_IMPERIAL, UNITS_AVIATION or UNITS_CUSTOM"
#endif

// ---------------------------------------------------------------------
//  Fonts
// ---------------------------------------------------------------------

// B612 is the typeface Airbus, ENAC and the Universite de Toulouse
// designed specifically for aircraft cockpit screens, and released
// under the SIL Open Font License. It is drawn to stay unambiguous at
// a glance under vibration and bad light, which is exactly the problem
// an FPV camera filming a small LCD has. The three headers next to
// this sketch were generated from the official TTF by tools/ttf2gfx.py
// and are bundled, so nothing extra needs installing.
//
// To regenerate at a different size:
//   python3 tools/ttf2gfx.py B612-Bold.ttf 32 B612Bold32 > B612Bold32.h
//
// LovyanGFX's own built-ins also work here, if you would rather not
// use the bundled headers at all:
//   &fonts::FreeSansBold18pt7b   Helvetica-like, a decent stand-in
//   &fonts::DejaVu24             wider
//   &fonts::Orbitron_Light_24    squarer, more sci-fi than aviation
#define FONT_LARGE  (&B612Bold28)   // readouts, flight mode, warning box
#define FONT_MEDIUM (&B612Bold20)   // disarmed status line, DEMO tag
#define FONT_SMALL  (&B612Bold16)   // auto-shrink fallback

// Black outline drawn around text that sits over the sky or ground, in
// pixels, matching the outlines on the horizon markings. This is what
// keeps the lettering readable when an FPV camera films the screen.
// 0 falls back to a solid black box behind the text instead.
const int TEXT_OUTLINE = 2;           // default: 2

// Nudge all centered text down by this many pixels. Proportional fonts
// reserve room for descenders, so digits can sit a pixel or two off
// center in their box. Adjust by eye if your panel looks wrong.
const int TEXT_Y_NUDGE = 0;           // default: 0

// ---------------------------------------------------------------------
//  Motion smoothing
// ---------------------------------------------------------------------

// ArduPilot sends ATTITUDE at a fixed rate (often 10 Hz), while the
// screen redraws much faster. Without smoothing the horizon moves in
// visible steps. This glides the drawn attitude toward the latest
// reported one, a fraction of the remaining distance each frame.
//   1.0  = off, snap straight to the reported attitude
//   0.50 = light, enough to absorb the step when a message lands
//   0.15 = very smooth, noticeably laggy
// Smoothing buys steadiness by lagging, so with forward prediction
// switched on below you want this fairly high. Raising MAVn_EXTRA1 on
// the flight controller beats any value here. See README.
const float ATTITUDE_SMOOTHING = 0.50f;   // default: 0.50

// ---------------------------------------------------------------------
//  Forward prediction (latency cancelling)
// ---------------------------------------------------------------------

// Every ATTITUDE message carries the angular RATES as well as the
// angles. Projecting the last known attitude forward along those rates
// estimates where the aircraft is right now, rather than where it was
// when the message was sent. That removes latency instead of hiding
// it, and it fills the gaps between messages smoothly, which is the
// job smoothing was doing at the cost of lag.
//
// Set this to roughly the delay you can see: the time from the
// aircraft moving to the screen moving. Everything counts - the wait
// for the next message, the serial link, the frame, pushing it over
// SPI, and the panel's own pixel response.
//   0   = off, show the attitude as reported
//   60  = cancels a typical pipeline at 25 Hz
//   100 = aggressive; may overshoot slightly when you stop abruptly
//
// Prediction is an estimate. Too much and the horizon leads the
// aircraft and rebounds when you stop a roll. Start here, and lower it
// if you see the horizon overshoot and settle back.
const float ATTITUDE_LEAD_MS = 60.0f;        // default: 60

// Never project further ahead than this, however stale the data. Caps
// the error if telemetry hiccups mid-manoeuvre.
const float ATTITUDE_MAX_LEAD_MS = 150.0f;   // default: 150

// Attitude changes smaller than this, in degrees, are ignored.
//
// The horizon line is drawn from endpoints 320 px either side of the
// screen centre, so it has a long lever arm: a couple of tenths of a
// degree of gyro noise, with the aircraft standing perfectly still,
// swings the visible ends of the line by a pixel or more. The panel
// also scans its memory out while the next frame is being written into
// it, so a frame that differs slightly from the last shows a seam
// partway down. Together they look like the picture shivering left and
// right. Holding still for small changes removes both.
//
// This is a soft deadband: once a change exceeds the threshold it
// tracks normally, so there is no stick-then-jump. 0 turns it off.
//   0.25 = steady on the bench, no effect you can see in flight
//   0.60 = very steady, slightly notchy in gentle turns
const float ATTITUDE_DEADBAND_DEG = 0.25f;   // default: 0.25

// ---------------------------------------------------------------------
//  Warning pop-up
// ---------------------------------------------------------------------

// Set a limit to 0 to disable that warning.

// Low battery, total pack voltage in volts: cells x volts-per-cell.
// The default is 3.3 V per cell on a 4S LiPo.
const float WARN_PACK_V = 13.20f;     // default: 13.2

// The low-altitude limit (WARN_ALT) lives in the Units section above,
// so it always matches the unit system you chose. It is only active
// while armed, and only once the plane has first climbed above the
// limit, so it doesn't fire on the ground or during launch.

// A warning clears once the value is this fraction above its limit, so
// it doesn't flicker right at the threshold.
const float WARN_HYST = 0.05f;        // default: 0.05

// Which battery to monitor (MAVLink battery id, 0 = first/main).
const uint8_t BATT_ID = 0;            // default: 0

// Pop-up blinking: visible for ON ms, hidden for OFF ms.
// Set WARN_BLINK_OFF_MS to 0 for a steady pop-up.
const uint32_t WARN_BLINK_ON_MS  = 600;   // default: 600
const uint32_t WARN_BLINK_OFF_MS = 300;   // default: 300

// A steady "ARMED" notice shows for this long after arming. A low
// battery or low altitude warning takes priority over it. 0 = off.
const uint32_t ARMED_NOTICE_MS = 5000;    // default: 5000

// ---------------------------------------------------------------------
//  Screen brightness
// ---------------------------------------------------------------------

// R/C channel (1-18) controlling brightness. 0 = disabled, the screen
// stays at BRIGHT_DEFAULT.
const int BRIGHT_RC_CHANNEL = 0;      // default: 0 (disabled)

// Channel values (microseconds) mapping to dimmest and brightest.
// Swap the two numbers to reverse the direction.
const int BRIGHT_RC_LOW_US  = 1000;   // default: 1000
const int BRIGHT_RC_HIGH_US = 2000;   // default: 2000

// Brightness range, 0-255. This is smooth PWM, so the whole range is
// usable. A minimum of 0 turns the backlight fully off.
const uint8_t BRIGHT_MIN = 0;         // default: 0
const uint8_t BRIGHT_MAX = 255;       // default: 255

// Brightness at startup, and whenever R/C control is disabled.
const uint8_t BRIGHT_DEFAULT = 255;   // default: 255

// Brightness curve. 1.0 is linear. Higher values spread the dim end
// over more of the channel's travel, where the eye is most sensitive.
const float BRIGHT_GAMMA = 1.5f;      // default: 1.5

// Smallest brightness change actually applied, so a noisy channel
// doesn't cause constant tiny PWM updates.
const int BRIGHT_DEADBAND = 3;        // default: 3

// ---------------------------------------------------------------------
//  Telemetry and timing
// ---------------------------------------------------------------------

// A readout shows "---" if its data stops arriving for this long.
const uint32_t DATA_TIMEOUT_MS = 2000;        // default: 2000

// The link counts as lost if no autopilot heartbeat arrives for this
// long. The status line then shows "Waiting for MAVLINK".
const uint32_t HEARTBEAT_TIMEOUT_MS = 3000;   // default: 3000

// If no attitude data arrives for this long, the horizon levels itself.
const uint32_t ATTITUDE_TIMEOUT_MS = 1000;    // default: 1000

// Minimum time between redraws. 10 ms caps the loop at 100 fps, which
// is well above what the SPI bus can deliver, so in practice the
// display simply runs as fast as it can and this never gets in the
// way. The real rate is reported over USB serial once a second.
const uint32_t FRAME_INTERVAL_MS = 10;        // default: 10

// ---------------------------------------------------------------------
//  Appearance
// ---------------------------------------------------------------------

// Colors (RGB565).
const uint16_t SKY      = 0x249F;   // default: 0x249F (blue)
const uint16_t GROUND   = 0x8200;   // default: 0x8200 (brown)
const uint16_t WHITE    = 0xFFFF;   // default: 0xFFFF
const uint16_t YELLOW   = 0xFFE0;   // default: 0xFFE0
const uint16_t CYAN     = 0x07FF;   // default: 0x07FF
const uint16_t BLACK    = 0x0000;   // default: 0x0000
const uint16_t RED      = 0xF800;   // default: 0xF800

// Line thicknesses in pixels.
const float T_HORIZON = 5.0f;  // horizon line (default: 5.0)
const float T_LADDER  = 4.0f;  // pitch ladder rungs (default: 4.0)
const float T_BANK    = 4.0f;  // bank reference ticks (default: 4.0)
const float T_SYMBOL  = 5.0f;  // fixed aircraft symbol (default: 5.0)

// Black outline added each side of every marking, so white and yellow
// stay visible against both sky and ground. 0 disables it.
const int OUTLINE = 3;         // default: 3

// Fixed yellow aircraft symbol: two bars with a short drop at the inner
// end of each. Distances in pixels from the screen center.
const int SYMBOL_OUTER = 52;   // center to outer end (default: 52)
const int SYMBOL_INNER = 16;   // center to inner end (default: 16)
const int SYMBOL_DROP  = 9;    // how far each inner end drops (default: 9)
const int SYMBOL_SLANT = 3;    // lean toward center, 0 = straight down
                               // (default: 3)

// How far the horizon moves per degree of pitch, in pixels. At 5.0 the
// 240 px screen shows about +/- 24 degrees. Raise toward 7.0 for a
// tighter, more sensitive scale.
const float PITCH_PIXELS_PER_DEGREE = 5.0f;   // default: 5.0

// Pitch ladder: a rung every LADDER_STEP_DEG degrees, up to
// LADDER_MAX_DEG either side. LADDER_STEP_DEG must be > 0, and
// LADDER_MAX_DEG should be a multiple of it.
const int LADDER_STEP_DEG = 5;        // default: 5
const int LADDER_MAX_DEG  = 10;       // default: 10

// Readout boxes (left: airspeed, right: altitude above home).
// B612Bold28 digits are 18 px wide, so 90 fits four of them.
const int BOX_W = 90;                 // default: 90
const int BOX_H = 38;                 // default: 38
const int BOX_MARGIN = 4;             // default: 4

// Warning pop-up box layout.
const int WARN_W = 210;               // default: 210
const int WARN_HDR_H = 32;            // default: 32
const int WARN_BODY_H = 36;           // default: 36
const int WARN_Y = 4;                 // default: 4

// Shift the bank reference ticks down from the top of the screen.
const int BANK_TICK_Y_SHIFT = 0;      // default: 0


// =====================================================================
//  PROGRAM CODE - you shouldn't need to change anything below here
// =====================================================================

// ---------------------------------------------------------------------
//  LovyanGFX device configuration: ILI9341 on SPI, ESP32-S3
// ---------------------------------------------------------------------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;

public:
  LGFX() {
    {
      auto cfg = _bus.config();
      // SPI2_HOST is the S3's FSPI controller, whose native pins are
      // GPIO 11/12/13.
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = TFT_SPI_FREQ;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = TFT_SCLK;
      cfg.pin_mosi    = TFT_MOSI;
      cfg.pin_miso    = TFT_MISO;
      cfg.pin_dc      = TFT_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }

    {
      auto cfg = _panel.config();
      cfg.pin_cs           = TFT_CS;
      cfg.pin_rst          = TFT_RST;
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;   // native portrait width
      cfg.panel_height     = 320;   // native portrait height
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = (TFT_MISO >= 0);
      cfg.invert           = TFT_INVERT;
      cfg.rgb_order        = false;  // true if red and blue swap
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = false;  // true if you also use the SD slot
      _panel.config(cfg);
    }

    if (TFT_BL >= 0) {
      auto cfg = _light.config();
      cfg.pin_bl      = TFT_BL;
      cfg.invert      = false;
      cfg.freq        = 12000;
      cfg.pwm_channel = 0;
      _light.config(cfg);
      _panel.setLight(&_light);
    }

    setPanel(&_panel);
  }
};

LGFX tft;

// Screen size and center, in the chosen rotation.
const int W = 320;
const int H = 240;
const int CX = W / 2;
const int CY = H / 2;

// ---------------------------------------------------------------------
//  Band canvas
// ---------------------------------------------------------------------
// The frame may be drawn in horizontal strips when memory is tight.
// This wrapper lets all the drawing code use full-screen coordinates:
// it holds one sprite the height of a single band and subtracts the
// band's top edge from every Y before passing it on. Anything outside
// the band is clipped by the sprite, which costs nothing. With
// FRAME_BANDS = 1 the band is the whole screen and this is a no-op.
class BandCanvas {
public:
  lgfx::LGFX_Sprite spr;
  int yOff = 0;      // screen Y of the top of the current band
  int bandH = 0;     // height of one band

  BandCanvas(LovyanGFX* parent) : spr(parent) {}

  bool create(int w, int h) {
    bandH = h;
    return spr.createSprite(w, h) != nullptr;
  }
  void setColorDepth(int d) { spr.setColorDepth(d); }

  void drawFastHLine(int x, int y, int w, uint16_t c) {
    spr.drawFastHLine(x, y - yOff, w, c);
  }
  void fillRect(int x, int y, int w, int h, uint16_t c) {
    spr.fillRect(x, y - yOff, w, h, c);
  }
  void fillCircle(int x, int y, int r, uint16_t c) {
    spr.fillCircle(x, y - yOff, r, c);
  }
  void fillTriangle(int x1, int y1, int x2, int y2,
                    int x3, int y3, uint16_t c) {
    spr.fillTriangle(x1, y1 - yOff, x2, y2 - yOff, x3, y3 - yOff, c);
  }

  void setFont(const lgfx::IFont* f)        { spr.setFont(f); }
  void setTextDatum(textdatum_t d)          { spr.setTextDatum(d); }
  void setTextColor(uint16_t c)             { spr.setTextColor(c); }
  void setTextColor(uint16_t f, uint16_t b) { spr.setTextColor(f, b); }
  int  textWidth(const char* s)             { return spr.textWidth(s); }
  int  fontHeight()                         { return spr.fontHeight(); }
  void drawString(const char* s, int x, int y) {
    spr.drawString(s, x, y - yOff);
  }

  // Push the current band to the display.
  void push() { spr.pushSprite(0, yOff); }
};

BandCanvas canvas(&tft);
int frameBands = FRAME_BANDS;

// Conversion tables, indexed by SPEED_UNIT and ALT_UNIT. Speed factors
// convert from m/s, altitude factors from metres.
const float SPEED_FACTORS[] = {3.6f, 1.0f, 2.23694f, 1.94384f};
const char* const SPEED_UNIT_NAMES[] = {"km/h", "m/s", "mph", "kt"};
const float ALT_FACTORS[] = {1.0f, 3.28084f};
const char* const ALT_UNIT_NAMES[] = {"m", "ft"};

// Sanity checks on the settings.
static_assert(LADDER_STEP_DEG > 0, "LADDER_STEP_DEG must be greater than 0");
static_assert(BRIGHT_MIN <= BRIGHT_MAX, "BRIGHT_MIN must not exceed BRIGHT_MAX");
static_assert(FRAME_BANDS >= 1, "FRAME_BANDS must be at least 1");


// Attitude as last reported by the flight controller, with the angular
// rates that came with it (degrees per second).
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float rollRateDps = 0.0f;
float pitchRateDps = 0.0f;

// Attitude as currently drawn, easing toward the reported values.
float dispRoll = 0.0f;
float dispPitch = 0.0f;

float airspeedMs = 0.0f;   // airspeed in m/s (VFR_HUD)
float homeAltM = 0.0f;     // altitude above home in m (GLOBAL_POSITION_INT)
uint32_t lastVfrMs = 0;
uint32_t lastPosMs = 0;

// Battery, arming and flight mode state.
float packV = 0.0f;        // total pack voltage in V
uint32_t lastPackMs = 0;
bool armed = false;
uint32_t flightMode = 0;   // ArduPilot custom_mode from the heartbeat
uint8_t vehicleType = 0;   // MAV_TYPE from the heartbeat
uint32_t lastHeartbeatMs = 0;
uint32_t armedNoticeUntilMs = 0;

bool altWarnReady = false;   // true once above WARN_ALT while armed
bool lowBattActive = false;
bool lowAltActive = false;

// R/C brightness control.
uint16_t rcBrightUs = 0;
int currentBright = BRIGHT_DEFAULT;

// Demo mode state.
bool demoMode = false;
uint32_t demoStartMs = 0;

uint32_t lastFrame = 0;
uint32_t lastAttitudeMs = 0;
uint32_t attitudeCount = 0;

// Frame timing, reported over USB serial once a second.
uint32_t frameCount = 0;
uint32_t frameTimeAccumUs = 0;

mavlink_message_t mavMsg;
mavlink_status_t mavStatus;

// ---------------------------------------------------------------------
//  WiFi MAVLink bridge
// ---------------------------------------------------------------------
// Raw bytes in both directions. Telemetry from the flight controller is
// gathered for up to WIFI_FLUSH_MS and sent as one UDP packet; anything
// the ground station sends is written straight back out of the serial
// port. Nothing here blocks: if WiFi never comes up, or no ground
// station ever appears, the display is unaffected.

void parseMavlinkBytes(const uint8_t *data, int len);   // defined below

WiFiUDP udp;
bool bridgeUp = false;             // WiFi came up and the socket is open
IPAddress gcsIp;                   // last ground station we heard from
bool gcsKnown = false;
uint32_t lastGcsPacketMs = 0;
uint32_t bytesToGcs = 0;           // counters, reported over USB serial
uint32_t bytesToFc = 0;
uint32_t uplinkDropped = 0;        // datagrams dropped, see pumpLink

// Telemetry waiting to go out as one packet.
uint8_t outbox[1024];
int outboxLen = 0;
uint32_t outboxSince = 0;

bool gcsOnline() {
  return gcsKnown && (millis() - lastGcsPacketMs) < WIFI_GCS_TIMEOUT_MS;
}

// Where to send telemetry: straight to the ground station once one has
// identified itself, otherwise broadcast so it can find us.
IPAddress telemetryTarget() {
  if (gcsOnline()) return gcsIp;

#if WIFI_BRIDGE_MODE == BRIDGE_AP
  IPAddress ip = WiFi.softAPIP();
  return IPAddress(ip[0], ip[1], ip[2], 255);
#else
  IPAddress ip = WiFi.localIP();
  IPAddress nm = WiFi.subnetMask();
  return IPAddress(ip[0] | ~nm[0], ip[1] | ~nm[1],
                   ip[2] | ~nm[2], ip[3] | ~nm[3]);
#endif
}

void flushToGcs() {
  if (!bridgeUp || outboxLen == 0) return;
  if (udp.beginPacket(telemetryTarget(), WIFI_UDP_PORT)) {
    udp.write(outbox, outboxLen);
    udp.endPacket();
    bytesToGcs += outboxLen;
  }
  outboxLen = 0;
}

void queueToGcs(const uint8_t *data, int len) {
  if (!bridgeUp) return;
  while (len > 0) {
    if (outboxLen == 0) outboxSince = millis();
    int space = (int)sizeof(outbox) - outboxLen;
    int n = (len < space) ? len : space;
    memcpy(outbox + outboxLen, data, n);
    outboxLen += n;
    data += n;
    len -= n;
    if (outboxLen >= (int)sizeof(outbox)) flushToGcs();
  }
}

void bridgeBegin() {
  if (!WIFI_BRIDGE_ENABLE) {
    Serial.println("WiFi bridge disabled");
    return;
  }

  WiFi.persistent(false);   // don't wear out flash rewriting credentials
  WiFi.setSleep(false);     // modem sleep adds latency we do not want

#if WIFI_BRIDGE_MODE == BRIDGE_AP
  WiFi.mode(WIFI_AP);
  const char *pw = (strlen(WIFI_PASS) >= 8) ? WIFI_PASS : nullptr;
  if (!WiFi.softAP(WIFI_SSID, pw, WIFI_AP_CHANNEL)) {
    Serial.println("WiFi: softAP failed - running as a plain display");
    return;
  }
  Serial.printf("WiFi AP '%s' up, board at %s, UDP %u\n",
                WIFI_SSID, WiFi.softAPIP().toString().c_str(),
                (unsigned)WIFI_UDP_PORT);
  if (pw == nullptr) {
    Serial.println("WiFi: OPEN network - anyone in range can command "
                   "the aircraft");
  }
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi: joining '%s' ", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - t0 < WIFI_STA_TIMEOUT_MS) {
    delay(200);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    // Not fatal. Give up on the bridge and just be a display.
    Serial.println("WiFi: could not join - running as a plain display");
    WiFi.mode(WIFI_OFF);
    return;
  }
  Serial.printf("WiFi joined, board at %s, UDP %u\n",
                WiFi.localIP().toString().c_str(),
                (unsigned)WIFI_UDP_PORT);
#endif

  udp.begin(WIFI_UDP_PORT);
  bridgeUp = true;
}

// Move bytes both ways. Called from loop() as often as possible, and
// again right after a frame is pushed, since drawing blocks for tens of
// milliseconds. Bounded so it can never spin: whatever is left over is
// picked up on the next call, and the serial receive buffer holds it
// meanwhile.
void pumpLink() {
  uint8_t buf[512];

  // --- flight controller -> display parser and ground station ---
  for (int round = 0; round < 4 && Serial1.available() > 0; round++) {
    int n = 0;
    while (n < (int)sizeof(buf) && Serial1.available() > 0) {
      buf[n++] = (uint8_t)Serial1.read();
    }
    if (n <= 0) break;

    queueToGcs(buf, n);

    // The demo drives the display from its own simulated attitude, so
    // real telemetry is bridged but not drawn.
    if (!demoMode) parseMavlinkBytes(buf, n);
  }

  if (!bridgeUp) return;

  // --- ground station -> flight controller ---
  for (int round = 0; round < 8; round++) {
    int len = udp.parsePacket();
    if (len <= 0) break;

    // Hearing anything at all is what marks the ground station live,
    // even if the datagram itself turns out to be unusable below.
    gcsIp = udp.remoteIP();
    gcsKnown = true;
    lastGcsPacketMs = millis();

    if (len > (int)sizeof(buf)) {
      // Implausibly large for MAVLink; drain it and move on.
      while (udp.read(buf, sizeof(buf)) > 0) {}
      uplinkDropped++;
      continue;
    }

    int n = udp.read(buf, len);
    if (n <= 0) continue;

    // All of it or none of it. A half-written datagram would corrupt
    // the byte stream the flight controller is parsing, whereas losing
    // a whole one is just a lost packet, which MAVLink already expects
    // and the ground station retries. Checking first also means the
    // write never blocks and so never stalls a frame.
    if ((int)Serial1.availableForWrite() >= n) {
      Serial1.write(buf, n);
      bytesToFc += n;
    } else {
      uplinkDropped++;
    }
  }

  if (outboxLen > 0 && millis() - outboxSince >= WIFI_FLUSH_MS) {
    flushToGcs();
  }
}

// Feed received bytes through the MAVLink parser to keep the display
// fed. The bridge forwards the same bytes separately and untouched, so
// anything this function does not understand still reaches the ground
// station intact.
void parseMavlinkBytes(const uint8_t *data, int len) {
  for (int i = 0; i < len; i++) {
    uint8_t b = data[i];

    if (mavlink_parse_char(
          MAVLINK_COMM_0, b, &mavMsg, &mavStatus)) {

      if (mavMsg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&mavMsg, &attitude);

        // MAVLink ATTITUDE angles are in radians, rates in rad/s.
        rollDeg  = attitude.roll  * RAD_TO_DEG;
        pitchDeg = attitude.pitch * RAD_TO_DEG;
        rollRateDps  = attitude.rollspeed  * RAD_TO_DEG;
        pitchRateDps = attitude.pitchspeed * RAD_TO_DEG;

        lastAttitudeMs = millis();
        attitudeCount++;
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
        mavlink_vfr_hud_t vfr;
        mavlink_msg_vfr_hud_decode(&mavMsg, &vfr);

        // Airspeed in m/s. With no airspeed sensor this is ArduPilot's
        // own estimate.
        airspeedMs = vfr.airspeed;
        lastVfrMs = millis();
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t pos;
        mavlink_msg_global_position_int_decode(&mavMsg, &pos);

        // relative_alt is millimetres above home.
        homeAltM = pos.relative_alt / 1000.0f;
        lastPosMs = millis();
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&mavMsg, &hb);

        // Only trust the autopilot's own heartbeat for arm state.
        if (mavMsg.compid == MAV_COMP_ID_AUTOPILOT1 &&
            hb.autopilot != MAV_AUTOPILOT_INVALID) {
          bool nowArmed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
          if (nowArmed && !armed) {
            armedNoticeUntilMs = millis() + ARMED_NOTICE_MS;
          }
          armed = nowArmed;
          flightMode = hb.custom_mode;
          vehicleType = hb.type;
          lastHeartbeatMs = millis();
        }
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
        mavlink_sys_status_t sys;
        mavlink_msg_sys_status_decode(&mavMsg, &sys);

        // Total pack voltage in mV (UINT16_MAX = unknown).
        if (sys.voltage_battery != UINT16_MAX) {
          packV = sys.voltage_battery / 1000.0f;
          lastPackMs = millis();
        }
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
        mavlink_battery_status_t bat;
        mavlink_msg_battery_status_decode(&mavMsg, &bat);

        if (bat.id == BATT_ID) {
          // Total pack voltage is the sum of the entries present
          // (UINT16_MAX = not provided). Works whether the flight
          // controller reports individual cells or one total.
          uint32_t sumMv = 0;
          int n = 0;
          for (int i = 0; i < 10; i++) {
            uint16_t mv = bat.voltages[i];
            if (mv != UINT16_MAX && mv > 0) {
              sumMv += mv;
              n++;
            }
          }

          if (n > 0) {
            packV = sumMv / 1000.0f;
            lastPackMs = millis();
          }
        }
      } else if (mavMsg.msgid == MAVLINK_MSG_ID_RC_CHANNELS) {
        mavlink_rc_channels_t rc;
        mavlink_msg_rc_channels_decode(&mavMsg, &rc);

        const uint16_t ch[18] = {
          rc.chan1_raw,  rc.chan2_raw,  rc.chan3_raw,
          rc.chan4_raw,  rc.chan5_raw,  rc.chan6_raw,
          rc.chan7_raw,  rc.chan8_raw,  rc.chan9_raw,
          rc.chan10_raw, rc.chan11_raw, rc.chan12_raw,
          rc.chan13_raw, rc.chan14_raw, rc.chan15_raw,
          rc.chan16_raw, rc.chan17_raw, rc.chan18_raw};

        if (BRIGHT_RC_CHANNEL >= 1 && BRIGHT_RC_CHANNEL <= 18) {
          rcBrightUs = ch[BRIGHT_RC_CHANNEL - 1];
        }
      }
    }
  }
}

// ---- Drawing primitives ---------------------------------------------

// A line of arbitrary thickness with round end caps, built from two
// filled triangles plus a circle at each end, so there are no gaps on
// diagonals however thick it gets.
void thickLine(int x1, int y1, int x2, int y2,
               float thickness, uint16_t color) {
  float half = thickness * 0.5f;
  int r = (int)(half + 0.5f);
  float dx = (float)(x2 - x1);
  float dy = (float)(y2 - y1);
  float len = sqrtf(dx * dx + dy * dy);

  if (len >= 0.5f) {
    float nx = -dy / len * half;
    float ny =  dx / len * half;

    int ax = (int)lroundf(x1 + nx), ay = (int)lroundf(y1 + ny);
    int bx = (int)lroundf(x2 + nx), by = (int)lroundf(y2 + ny);
    int cx = (int)lroundf(x2 - nx), cy = (int)lroundf(y2 - ny);
    int ex = (int)lroundf(x1 - nx), ey = (int)lroundf(y1 - ny);

    canvas.fillTriangle(ax, ay, bx, by, cx, cy, color);
    canvas.fillTriangle(ax, ay, cx, cy, ex, ey, color);
  }

  if (r > 0) {
    canvas.fillCircle(x1, y1, r, color);
    canvas.fillCircle(x2, y2, r, color);
  }
}

// Thick line with a black outline underneath it.
void markLine(int x1, int y1, int x2, int y2,
              float thickness, uint16_t color) {
  if (OUTLINE > 0) {
    thickLine(x1, y1, x2, y2, thickness + 2 * OUTLINE, BLACK);
  }
  thickLine(x1, y1, x2, y2, thickness, color);
}

// Text with a black outline around it, for lettering that sits over the
// sky or ground. The font and datum must already be set. With
// TEXT_OUTLINE = 0 this falls back to a solid black background instead.
void outlineString(const char* s, int x, int y, uint16_t color) {
  if (TEXT_OUTLINE > 0) {
    static const int ox[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
    static const int oy[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    canvas.setTextColor(BLACK);
    for (int i = 0; i < 8; i++) {
      canvas.drawString(s, x + ox[i] * TEXT_OUTLINE,
                           y + oy[i] * TEXT_OUTLINE);
    }
    canvas.setTextColor(color);
  } else {
    canvas.setTextColor(color, BLACK);
  }
  canvas.drawString(s, x, y);
}

// ---- Screen elements -------------------------------------------------

// Fixed yellow aircraft symbol. Drawn in two passes (all outlines, then
// all yellow) so one segment's outline never cuts into the next's fill.
void drawAircraftSymbol() {
  for (int pass = 0; pass < 2; pass++) {
    bool outlinePass = (pass == 0);
    if (outlinePass && OUTLINE <= 0) continue;

    uint16_t col = outlinePass ? BLACK : YELLOW;
    float t = T_SYMBOL + (outlinePass ? 2 * OUTLINE : 0);

    // Left half: bar, then the drop at its inner end.
    thickLine(CX - SYMBOL_OUTER, CY, CX - SYMBOL_INNER, CY, t, col);
    thickLine(CX - SYMBOL_INNER, CY,
              CX - SYMBOL_INNER + SYMBOL_SLANT, CY + SYMBOL_DROP, t, col);

    // Right half, mirrored.
    thickLine(CX + SYMBOL_OUTER, CY, CX + SYMBOL_INNER, CY, t, col);
    thickLine(CX + SYMBOL_INNER, CY,
              CX + SYMBOL_INNER - SYMBOL_SLANT, CY + SYMBOL_DROP, t, col);
  }
}

// One boxed numeric readout with its unit label underneath.
// leftSide = true puts it at the left edge, false at the right edge.
void drawReadout(bool leftSide, bool valid,
                 float value, const char* label) {
  int bx = leftSide ? BOX_MARGIN : W - BOX_MARGIN - BOX_W;
  int by = CY - BOX_H / 2;

  // Black outer outline, white border, black fill.
  canvas.fillRect(bx - OUTLINE, by - OUTLINE,
                  BOX_W + 2 * OUTLINE, BOX_H + 2 * OUTLINE, BLACK);
  canvas.fillRect(bx, by, BOX_W, BOX_H, WHITE);
  canvas.fillRect(bx + 3, by + 3, BOX_W - 6, BOX_H - 6, BLACK);

  char buf[12];
  if (valid) {
    long v = lroundf(value);
    if (v > 9999) v = 9999;
    if (v < -999) v = -999;
    snprintf(buf, sizeof(buf), "%ld", v);
  } else {
    snprintf(buf, sizeof(buf), "---");
  }

  // The number, centered in the box, shrinking if it would not fit.
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.setFont(FONT_LARGE);
  if (canvas.textWidth(buf) > BOX_W - 10) canvas.setFont(FONT_MEDIUM);
  if (canvas.textWidth(buf) > BOX_W - 10) canvas.setFont(FONT_SMALL);
  canvas.setTextColor(valid ? WHITE : YELLOW);
  canvas.drawString(buf, bx + BOX_W / 2, by + BOX_H / 2 + TEXT_Y_NUDGE);

  // Unit label below the box, outlined because it sits over the scene.
  canvas.setFont(FONT_LARGE);
  canvas.setTextDatum(textdatum_t::top_center);
  int lx = bx + BOX_W / 2;
  int half = canvas.textWidth(label) / 2;
  if (lx - half < 2) lx = 2 + half;
  if (lx + half > W - 2) lx = W - 2 - half;
  outlineString(label, lx, by + BOX_H + OUTLINE + 2, YELLOW);
}

// Airspeed on the left, altitude above home on the right.
void drawReadouts() {
  uint32_t now = millis();
  bool speedOk = lastVfrMs != 0 && now - lastVfrMs < DATA_TIMEOUT_MS;
  bool altOk   = lastPosMs != 0 && now - lastPosMs < DATA_TIMEOUT_MS;

  drawReadout(true, speedOk,
              airspeedMs * SPEED_FACTORS[SPEED_UNIT],
              SPEED_UNIT_NAMES[SPEED_UNIT]);
  drawReadout(false, altOk,
              homeAltM * ALT_FACTORS[ALT_UNIT],
              ALT_UNIT_NAMES[ALT_UNIT]);
}

// Decide which warnings are active. Hysteresis stops a warning
// flickering when a value hovers right at its limit. Called once per
// frame, before the bands are drawn, so every band agrees.
void updateWarnings() {
  uint32_t now = millis();

  // --- Low battery (checked whether or not the plane is armed) ---
  bool packFresh = lastPackMs != 0 && now - lastPackMs < DATA_TIMEOUT_MS;

  if (WARN_PACK_V > 0.0f && packFresh) {
    if (packV < WARN_PACK_V) {
      lowBattActive = true;
    } else if (packV > WARN_PACK_V * (1.0f + WARN_HYST)) {
      lowBattActive = false;
    }
  } else {
    lowBattActive = false;
  }

  // --- Low altitude ---
  // Only while armed, and only once the plane has been above the limit,
  // so it doesn't fire on the ground or during launch.
  bool armedNow = armed && lastHeartbeatMs != 0 &&
                  now - lastHeartbeatMs < HEARTBEAT_TIMEOUT_MS;
  bool altFresh = lastPosMs != 0 && now - lastPosMs < DATA_TIMEOUT_MS;

  if (!armedNow) {
    altWarnReady = false;
    lowAltActive = false;
  } else if (altFresh && WARN_ALT > 0.0f) {
    float alt = homeAltM * ALT_FACTORS[ALT_UNIT];

    if (alt >= WARN_ALT) altWarnReady = true;

    if (altWarnReady) {
      if (alt < WARN_ALT) {
        lowAltActive = true;
      } else if (alt > WARN_ALT * (1.0f + WARN_HYST)) {
        lowAltActive = false;
      }
    }
  } else {
    lowAltActive = false;
  }
}

// What the warning pop-up shows this frame. Decided once per frame
// rather than per band, so the blink phase can't change halfway down
// the screen and tear the box in half.
bool warnVisible = false;
char warnText[24] = "";

void decideWarning() {
  warnVisible = false;

  bool showBatt = lowBattActive;
  bool showAlt = lowAltActive;
  bool showArmed = (int32_t)(armedNoticeUntilMs - millis()) > 0;
  if (!showBatt && !showAlt && !showArmed) return;

  if (showBatt || showAlt) {
    // A real warning takes priority over the ARMED notice.
    uint32_t blinkPeriod = WARN_BLINK_ON_MS + WARN_BLINK_OFF_MS;
    if (blinkPeriod == 0) blinkPeriod = 1;

    // If both are active, alternate between them (two blinks each).
    if (showBatt && showAlt) {
      if ((millis() / (2 * blinkPeriod)) % 2 == 0) showAlt = false;
      else showBatt = false;
    }

    // Blink: hidden during the "off" part of each cycle.
    if (WARN_BLINK_OFF_MS > 0 &&
        millis() % blinkPeriod >= WARN_BLINK_ON_MS) {
      return;
    }

    if (showBatt) {
      snprintf(warnText, sizeof(warnText), "BATT %.1fV", packV);
    } else {
      snprintf(warnText, sizeof(warnText), "ALT %ld %s",
               lroundf(homeAltM * ALT_FACTORS[ALT_UNIT]),
               ALT_UNIT_NAMES[ALT_UNIT]);
    }
  } else {
    snprintf(warnText, sizeof(warnText), "ARMED");
  }

  warnVisible = true;
}

// Red WARNING pop-up, centered in the upper part of the screen.
void drawWarning() {
  if (!warnVisible) return;

  int bx = (W - WARN_W) / 2;
  int by = WARN_Y;
  int totalH = WARN_HDR_H + WARN_BODY_H;

  // Black outline, red box, then a black panel for the body text.
  canvas.fillRect(bx - OUTLINE, by - OUTLINE,
                  WARN_W + 2 * OUTLINE, totalH + 2 * OUTLINE, BLACK);
  canvas.fillRect(bx, by, WARN_W, totalH, RED);
  canvas.fillRect(bx + 3, by + WARN_HDR_H,
                  WARN_W - 6, WARN_BODY_H - 3, BLACK);

  canvas.setTextDatum(textdatum_t::middle_center);

  // "WARNING" header, white on red.
  canvas.setFont(FONT_LARGE);
  canvas.setTextColor(WHITE);
  canvas.drawString("WARNING", bx + WARN_W / 2,
                    by + WARN_HDR_H / 2 + TEXT_Y_NUDGE);

  // Value text, white on black, shrinking if it would not fit.
  canvas.setFont(FONT_LARGE);
  if (canvas.textWidth(warnText) > WARN_W - 14) canvas.setFont(FONT_MEDIUM);
  canvas.setTextColor(WHITE);
  canvas.drawString(warnText, bx + WARN_W / 2,
                    by + WARN_HDR_H + (WARN_BODY_H - 3) / 2 + TEXT_Y_NUDGE);
}

// ArduPlane flight mode name from the heartbeat's custom_mode.
// Other vehicle types or unknown modes show as "MODE n".
const char* flightModeName(uint32_t mode, uint8_t type,
                           char* buf, size_t n) {
  // MAV_TYPE 1 = fixed wing, 19-25 = the VTOL types ArduPlane can use.
  bool isPlane = (type == 1) || (type >= 19 && type <= 25);

  if (isPlane) {
    switch (mode) {
      case 0:  return "MANUAL";
      case 1:  return "CIRCLE";
      case 2:  return "STABILIZE";
      case 3:  return "TRAINING";
      case 4:  return "ACRO";
      case 5:  return "FBWA";
      case 6:  return "FBWB";
      case 7:  return "CRUISE";
      case 8:  return "AUTOTUNE";
      case 10: return "AUTO";
      case 11: return "RTL";
      case 12: return "LOITER";
      case 13: return "TAKEOFF";
      case 14: return "AVOID ADSB";
      case 15: return "GUIDED";
      case 17: return "QSTABILIZE";
      case 18: return "QHOVER";
      case 19: return "QLOITER";
      case 20: return "QLAND";
      case 21: return "QRTL";
      case 22: return "QAUTOTUNE";
      case 23: return "QACRO";
      case 24: return "THERMAL";
      case 25: return "LOITALTQLAND";
      case 26: return "AUTOLAND";
      default: break;
    }
  }

  snprintf(buf, n, "MODE %lu", (unsigned long)mode);
  return buf;
}

// Sky and ground, split along the horizon. Rows that the horizon line
// does not cross are filled as solid blocks rather than one scanline at
// a time, which is most of the screen in anything near level flight.
void fillSkyAndGround(float s, float c, float horizonY) {
  int y0 = canvas.yOff;
  int y1 = canvas.yOff + canvas.bandH;
  if (y1 > H) y1 = H;
  if (y0 >= y1) return;

  // Wings level (or inverted): one horizontal split, two blocks.
  // Above the horizon line is always sky, below it always ground.
  if (fabsf(s) < 0.001f) {
    int hy = (int)lroundf(horizonY);
    if (hy < y0) hy = y0;
    if (hy > y1) hy = y1;
    if (hy > y0) canvas.fillRect(0, y0, W, hy - y0, SKY);
    if (y1 > hy) canvas.fillRect(0, hy, W, y1 - hy, GROUND);
    return;
  }

  // xCross(y) = CX + (y - horizonY) * k, so it advances by k per row.
  const float k = c / s;

  // Color of a row the horizon does not cross. Uses the same floor()
  // and comparison as the per-row loop below, so the blocks and the
  // scanlines can never disagree about a boundary row.
  auto solidAt = [&](int y) -> uint16_t {
    int sp = (int)floorf(CX + ((float)y - horizonY) * k);
    return (sp <= 0) ? ((s > 0) ? SKY : GROUND)
                     : ((s > 0) ? GROUND : SKY);
  };

  // Rows between these two are the only ones the horizon actually
  // crosses; everything above and below is a single color. A near-zero
  // k means a near-vertical horizon, where the divisions below would
  // blow up, so treat the whole band as crossed instead.
  int mixLo = y0, mixHi = y1;

  if (fabsf(k) > 0.0005f) {
    float yAt0 = horizonY - (float)CX / k;
    float yAtW = horizonY + (float)(W - CX) / k;
    float fa = fminf(yAt0, yAtW);
    float fb = fmaxf(yAt0, yAtW);
    if (fa < (float)y0) fa = (float)y0;
    if (fa > (float)y1) fa = (float)y1;
    if (fb < (float)y0) fb = (float)y0;
    if (fb > (float)y1) fb = (float)y1;
    mixLo = (int)floorf(fa);
    mixHi = (int)ceilf(fb);
    if (mixLo < y0) mixLo = y0;
    if (mixHi > y1) mixHi = y1;
    if (mixHi < mixLo) mixHi = mixLo;
  }

  // Solid block above the crossing band.
  if (mixLo > y0) {
    canvas.fillRect(0, y0, W, mixLo - y0, solidAt(y0));
  }

  // The crossing band, one scanline at a time.
  float xCross = CX + ((float)mixLo - horizonY) * k;
  for (int y = mixLo; y < mixHi; y++, xCross += k) {
    int split = (int)floorf(xCross);

    if (s > 0) {
      if (split <= 0) {
        canvas.drawFastHLine(0, y, W, SKY);
      } else if (split >= W) {
        canvas.drawFastHLine(0, y, W, GROUND);
      } else {
        canvas.drawFastHLine(0, y, split, GROUND);
        canvas.drawFastHLine(split, y, W - split, SKY);
      }
    } else {
      if (split <= 0) {
        canvas.drawFastHLine(0, y, W, GROUND);
      } else if (split >= W) {
        canvas.drawFastHLine(0, y, W, SKY);
      } else {
        canvas.drawFastHLine(0, y, split, SKY);
        canvas.drawFastHLine(split, y, W - split, GROUND);
      }
    }
  }

  // Solid block below the crossing band.
  if (y1 > mixHi) {
    canvas.fillRect(0, mixHi, W, y1 - mixHi, solidAt(y1 - 1));
  }
}

// Draw everything, for the band currently selected on the canvas.
// Coordinates are full-screen throughout.
void drawHorizon(float roll, float pitch) {
  float a = -roll * DEG_TO_RAD;
  float s = sinf(a);
  float c = cosf(a);
  float horizonY = CY + pitch * PITCH_PIXELS_PER_DEGREE;

  fillSkyAndGround(s, c, horizonY);

  // Horizon line. Endpoints run well past both screen edges along the
  // roll direction (the sprite clips them), which also avoids the
  // divide-by-cos(roll) problem at +/-90 degrees of bank.
  // lroundf, not a cast: truncation rounds toward zero, so the two ends
  // of the line would round in opposite directions and the line would
  // shift by a pixel as the attitude crossed zero.
  const float L = (float)W;
  markLine((int)lroundf(CX - L * c), (int)lroundf(horizonY - L * s),
           (int)lroundf(CX + L * c), (int)lroundf(horizonY + L * s),
           T_HORIZON, WHITE);

  // Pitch ladder.
  for (int p = -LADDER_MAX_DEG; p <= LADDER_MAX_DEG; p += LADDER_STEP_DEG) {
    if (p == 0) continue;

    float offset = -p * PITCH_PIXELS_PER_DEGREE;

    int lx1 = (int)lroundf(CX - 25 * c - offset * s);
    int ly1 = (int)lroundf(horizonY - 25 * s + offset * c);
    int lx2 = (int)lroundf(CX + 25 * c - offset * s);
    int ly2 = (int)lroundf(horizonY + 25 * s + offset * c);

    markLine(lx1, ly1, lx2, ly2, T_LADDER, WHITE);
  }

  drawAircraftSymbol();

  // Bank reference ticks.
  const int BS = BANK_TICK_Y_SHIFT;
  markLine(CX, 5 + BS, CX, 19 + BS, T_BANK, WHITE);
  markLine(CX - 42, 10 + BS, CX - 33, 22 + BS, T_BANK, WHITE);
  markLine(CX + 42, 10 + BS, CX + 33, 22 + BS, T_BANK, WHITE);
  markLine(CX - 70, 23 + BS, CX - 59, 31 + BS, T_BANK, WHITE);
  markLine(CX + 70, 23 + BS, CX + 59, 31 + BS, T_BANK, WHITE);

  drawReadouts();

  // Status line along the bottom: the flight mode, with "DISARMED" in
  // front of it on the ground, or a waiting message if the heartbeat
  // is lost.
  char status[40];
  bool big = false;
  if (lastHeartbeatMs != 0 &&
      millis() - lastHeartbeatMs < HEARTBEAT_TIMEOUT_MS) {
    char modeBuf[16];
    const char* mode = flightModeName(flightMode, vehicleType,
                                      modeBuf, sizeof(modeBuf));
    if (armed) {
      snprintf(status, sizeof(status), "%s", mode);
      big = true;
    } else {
      snprintf(status, sizeof(status), "DISARMED - %s", mode);
    }
  } else {
    snprintf(status, sizeof(status), "Waiting for MAVLINK");
  }

  canvas.setTextDatum(textdatum_t::bottom_center);
  canvas.setFont(big ? FONT_LARGE : FONT_MEDIUM);
  if (canvas.textWidth(status) > W - 8) canvas.setFont(FONT_MEDIUM);
  if (canvas.textWidth(status) > W - 8) canvas.setFont(FONT_SMALL);
  outlineString(status, CX, H - 4, YELLOW);

  // Small tag so it's obvious the data is simulated.
  if (demoMode) {
    canvas.setFont(FONT_MEDIUM);
    canvas.setTextDatum(textdatum_t::top_left);
    outlineString("DEMO", 6, 4, YELLOW);
  }

  // Bridge state, top right. CYAN once a ground station is actually
  // talking to us, YELLOW when WiFi is up but nothing has called in,
  // RED if WiFi never came up at all.
  if (WIFI_BRIDGE_ENABLE && WIFI_SHOW_STATUS) {
    const char *tag;
    uint16_t col;
    if (!bridgeUp)        { tag = "NO WIFI"; col = RED; }
    else if (gcsOnline()) { tag = "GCS";     col = CYAN; }
    else                  { tag = "WIFI";    col = YELLOW; }
    canvas.setFont(FONT_MEDIUM);
    canvas.setTextDatum(textdatum_t::top_right);
    outlineString(tag, W - 6, 4, col);
  }

  // Warning pop-up goes on top of everything else.
  drawWarning();
}

// Draw one complete frame, band by band.
void drawFrame() {
  updateWarnings();
  decideWarning();

  for (int b = 0; b < frameBands; b++) {
    canvas.yOff = b * canvas.bandH;
    if (canvas.yOff >= H) break;
    drawHorizon(dispRoll, dispPitch);
    canvas.push();
  }
}

// ---- Motion smoothing ------------------------------------------------

// Shortest signed angular distance from one angle to another, in
// degrees, so a roll through +/-180 takes the short way round.
float angleDelta(float from, float to) {
  float d = to - from;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// Where the aircraft probably is right now, as opposed to where it was
// when the last ATTITUDE message was sent: the reported attitude
// projected forward along the reported angular rates, by the age of
// that message plus ATTITUDE_LEAD_MS for the rest of the pipeline.
void predictedAttitude(float *roll, float *pitch) {
  *roll = rollDeg;
  *pitch = pitchDeg;

  if (ATTITUDE_LEAD_MS <= 0.0f || lastAttitudeMs == 0) return;

  float t = (float)(millis() - lastAttitudeMs) + ATTITUDE_LEAD_MS;
  if (t > ATTITUDE_MAX_LEAD_MS) t = ATTITUDE_MAX_LEAD_MS;
  if (t < 0.0f) t = 0.0f;
  t *= 0.001f;   // to seconds

  *roll  += rollRateDps  * t;
  *pitch += pitchRateDps * t;
}

// Ease the drawn attitude toward the predicted one.
void updateDisplayedAttitude() {
  float targetRoll, targetPitch;
  predictedAttitude(&targetRoll, &targetPitch);

  float a = ATTITUDE_SMOOTHING;
  if (a >= 1.0f || a <= 0.0f) {
    dispRoll = targetRoll;
    dispPitch = targetPitch;
    return;
  }

  // Soft deadband: ignore the first ATTITUDE_DEADBAND_DEG of any
  // change, then track the rest normally.
  const float db = ATTITUDE_DEADBAND_DEG;

  float dr = angleDelta(dispRoll, targetRoll);
  if (fabsf(dr) <= db) dr = 0.0f;
  else                 dr -= copysignf(db, dr);

  float dp = targetPitch - dispPitch;
  if (fabsf(dp) <= db) dp = 0.0f;
  else                 dp -= copysignf(db, dp);

  dispRoll += dr * a;
  if (dispRoll > 180.0f) dispRoll -= 360.0f;
  if (dispRoll < -180.0f) dispRoll += 360.0f;

  dispPitch += dp * a;
}

// ---- Backlight -------------------------------------------------------
// Unlike the T-Display-S3's AW9364 (a 16-step pulse-counting chip),
// this module's LED pin is a plain backlight driven by LEDC PWM, so
// brightness is smooth across the whole 0-255 range.

void updateBrightness() {
  if (TFT_BL < 0) return;
  if (BRIGHT_RC_CHANNEL < 1 || BRIGHT_RC_CHANNEL > 18) return;
  if (BRIGHT_RC_HIGH_US == BRIGHT_RC_LOW_US) return;

  // No data, an unused channel (65535) or a failsafe zero: hold.
  if (rcBrightUs < 800 || rcBrightUs > 2200) return;

  float t = (float)((int)rcBrightUs - BRIGHT_RC_LOW_US) /
            (float)(BRIGHT_RC_HIGH_US - BRIGHT_RC_LOW_US);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  float shaped = powf(t, BRIGHT_GAMMA);
  int target = (int)lroundf(BRIGHT_MIN + shaped * (BRIGHT_MAX - BRIGHT_MIN));

  // The two ends of the channel always apply, so the full range stays
  // reachable even with a little noise on the channel value.
  bool atEnd = (t <= 0.01f || t >= 0.99f);

  if (target != currentBright &&
      (atEnd || abs(target - currentBright) >= BRIGHT_DEADBAND)) {
    currentBright = target;
    tft.setBrightness((uint8_t)currentBright);
  }
}

// ---- Demo mode -------------------------------------------------------

// Start or stop the demo. Stopping forgets the simulated data, so the
// display goes back to waiting for real telemetry.
void setDemoMode(bool on) {
  demoMode = on;
  demoStartMs = millis();

  altWarnReady = false;
  lowBattActive = false;
  lowAltActive = false;
  armedNoticeUntilMs = 0;

  if (!on) {
    rollDeg = 0.0f;
    pitchDeg = 0.0f;
    rollRateDps = 0.0f;
    pitchRateDps = 0.0f;
    airspeedMs = 0.0f;
    homeAltM = 0.0f;
    packV = 0.0f;
    armed = false;
    flightMode = 0;
    vehicleType = 0;
    lastAttitudeMs = 0;
    lastVfrMs = 0;
    lastPosMs = 0;
    lastPackMs = 0;
    lastHeartbeatMs = 0;
  }

  Serial.println(on ? "Demo mode ON" : "Demo mode OFF");
}

// Fake flight: rolling and pitching, changing speed and altitude,
// flight modes cycling, and altitude and battery dipping below your
// warning limits so both pop-ups appear.
void updateDemo(uint32_t now) {
  const float TAU = 6.2831853f;
  float t = (now - demoStartMs) / 1000.0f;

  rollDeg    = 40.0f * sinf(TAU * t / 9.0f);
  pitchDeg   = 12.0f * sinf(TAU * t / 6.0f + 1.0f);
  airspeedMs = 27.0f + 8.0f * sinf(TAU * t / 13.0f);

  // Exact derivatives of the two curves above, so forward prediction
  // behaves in demo mode exactly as it does on real telemetry.
  rollRateDps  = 40.0f * (TAU / 9.0f) * cosf(TAU * t / 9.0f);
  pitchRateDps = 12.0f * (TAU / 6.0f) * cosf(TAU * t / 6.0f + 1.0f);

  // Altitude swings between 0.5x and 4.5x the low-altitude limit.
  float altLimitM = (WARN_ALT > 0.0f)
                      ? WARN_ALT / ALT_FACTORS[ALT_UNIT] : 50.0f;
  homeAltM = altLimitM * (2.5f + 2.0f * cosf(TAU * t / 30.0f));

  // Pack voltage swings between about 0.98x and 1.26x the limit.
  float battLimitV = (WARN_PACK_V > 0.0f) ? WARN_PACK_V : 20.0f;
  packV = battLimitV * (1.12f + 0.14f * cosf(TAU * t / 45.0f));

  // Disarmed for the first few seconds, then armed. The flight mode
  // changes every 8 seconds: FBWA, STABILIZE, MANUAL, AUTO.
  static const uint32_t demoModes[] = {5, 2, 0, 10};
  bool nowArmed = t > 3.0f;
  if (nowArmed && !armed) armedNoticeUntilMs = now + ARMED_NOTICE_MS;
  armed = nowArmed;
  vehicleType = 1;   // fixed wing
  flightMode = demoModes[((uint32_t)(t / 8.0f)) % 4];

  // Everything counts as freshly received.
  lastAttitudeMs = now;
  lastVfrMs = now;
  lastPosMs = now;
  lastPackMs = now;
  lastHeartbeatMs = now;
}

// Debounced button: each press toggles demo mode.
void pollButton() {
  if (BUTTON_PIN < 0) return;

  static bool lastRaw = HIGH;
  static bool stable = HIGH;
  static uint32_t changedMs = 0;

  bool raw = digitalRead(BUTTON_PIN);
  uint32_t now = millis();

  if (raw != lastRaw) {
    lastRaw = raw;
    changedMs = now;
  }

  if (raw != stable && now - changedMs >= BUTTON_DEBOUNCE_MS) {
    stable = raw;
    if (stable == LOW) setDemoMode(!demoMode);
  }
}

// Last-resort message straight to the panel, for failures that happen
// before the off-screen buffer exists.
void panelMessage(const char* line1, const char* line2) {
  tft.fillScreen(BLACK);
  tft.setFont(FONT_MEDIUM);
  tft.setTextDatum(textdatum_t::top_left);
  tft.setTextColor(RED);
  tft.drawString(line1, 8, 60);
  if (line2) {
    tft.setTextColor(YELLOW);
    tft.drawString(line2, 8, 100);
  }
}

void setup() {
  Serial.begin(115200);
  if (BUTTON_PIN >= 0) pinMode(BUTTON_PIN, INPUT_PULLUP);

  tft.init();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(BLACK);
  if (TFT_BL >= 0) tft.setBrightness(BRIGHT_DEFAULT);

  canvas.setColorDepth(16);

  bool hasPsram = psramFound();
  if (hasPsram && USE_PSRAM) {
    canvas.spr.setPsram(true);
    Serial.printf("PSRAM found (%u bytes) - buffering there\n",
                  (unsigned)ESP.getPsramSize());
  } else if (hasPsram) {
    Serial.println("PSRAM found but USE_PSRAM is false - buffering in SRAM");
  } else {
    Serial.println("No PSRAM - buffering in SRAM");
  }

  // Allocate the frame buffer. If it will not fit, halve the band
  // height and try again rather than giving up.
  frameBands = FRAME_BANDS;
  bool ok = false;

  while (frameBands <= 16) {
    int bandH = (H + frameBands - 1) / frameBands;
    Serial.printf("Trying %d band(s) of %d px (%d bytes)... ",
                  frameBands, bandH, W * bandH * 2);

    if (canvas.create(W, bandH)) {
      Serial.println("ok");
      ok = true;
      break;
    }

    Serial.println("failed");
    frameBands *= 2;
  }

  if (!ok) {
    Serial.println("Frame buffer allocation failed at every band size!");
    panelMessage("NO MEMORY", "buffer alloc failed");
    while (true) delay(1000);
  }

  if (frameBands != FRAME_BANDS) {
    Serial.printf("Note: FRAME_BANDS raised from %d to %d to fit in RAM.\n",
                  FRAME_BANDS, frameBands);
  }
  Serial.printf("Free heap after allocation: %u bytes (PSRAM free: %u)\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getFreePsram());

  // RX receives the flight controller's TX. TX is only actually used
  // when the WiFi bridge is on. setRxBufferSize must come before
  // begin() to have any effect.
  Serial1.setRxBufferSize(MAV_RX_BUFFER);
  Serial1.setTxBufferSize(MAV_TX_BUFFER);
  Serial1.begin(MAV_BAUD, SERIAL_8N1, MAV_RX_PIN, MAV_TX_PIN);

  // WiFi goes up LAST, on purpose. Its stack wants tens of kilobytes,
  // and the frame buffer needs one big contiguous block, so the frame
  // buffer gets first claim on memory above.
  bridgeBegin();
  Serial.printf("Free heap after WiFi: %u bytes\n",
                (unsigned)ESP.getFreeHeap());

  Serial.println("Cockpit display ready (ESP32-S3 / ILI9341)");
}

void loop() {
  pollButton();

  // Serve the link first: read the flight controller, forward to the
  // ground station, and pass anything it sent back the other way. This
  // runs every pass, far more often than the screen is redrawn.
  pumpLink();

  uint32_t now = millis();

  if (now - lastFrame < FRAME_INTERVAL_MS) return;
  lastFrame = now;

  if (demoMode) updateDemo(now);

  // Hold the last attitude briefly, then level the horizon if telemetry
  // disappears. This is only a visual fallback.
  if (lastAttitudeMs == 0 ||
      now - lastAttitudeMs > ATTITUDE_TIMEOUT_MS) {
    rollDeg = 0;
    pitchDeg = 0;
    // Zero the rates too, or the horizon would keep projecting along
    // the last known rate and drift away while the link is down.
    rollRateDps = 0;
    pitchRateDps = 0;
  }

  updateDisplayedAttitude();
  updateBrightness();

  uint32_t t0 = micros();
  drawFrame();
  frameTimeAccumUs += micros() - t0;
  frameCount++;

  // Pushing a frame blocks for tens of milliseconds. Serve the link
  // again immediately so the bytes that arrived meanwhile go straight
  // out, rather than waiting behind the next frame.
  pumpLink();

  // Debug information over USB serial, once a second.
  static uint32_t lastDebug = 0;

  if (now - lastDebug >= 1000) {
    lastDebug = now;

    float avgMs = frameCount ? (frameTimeAccumUs / 1000.0f / frameCount) : 0.0f;
    unsigned long ageMs = lastAttitudeMs ? (now - lastAttitudeMs) : 0;
    Serial.printf(
      "ATTITUDE %lu Hz (age %lu ms) | roll %.1f (%.0f dps) pitch %.1f "
      "| %lu fps, %.1f ms/frame | link %s %lu B/s out %lu B/s in"
      "%s\n",
      (unsigned long)attitudeCount, ageMs, rollDeg, rollRateDps, pitchDeg,
      (unsigned long)frameCount, avgMs,
      !WIFI_BRIDGE_ENABLE ? "off" : (!bridgeUp ? "down"
                                   : (gcsOnline() ? "GCS" : "idle")),
      (unsigned long)bytesToGcs, (unsigned long)bytesToFc,
      uplinkDropped ? " [UPLINK DROPS]" : "");
    bytesToGcs = 0;
    bytesToFc = 0;
    uplinkDropped = 0;

    attitudeCount = 0;
    frameCount = 0;
    frameTimeAccumUs = 0;
  }
}
