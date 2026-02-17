#pragma once
#include <Arduino.h>
#include <driver/rmt.h>
#include <math.h>

#define NUM_LEDS 240
#define DATA_PIN 10

// ---------- Color helpers ----------
struct RGB {
  uint8_t r, g, b;
  RGB(uint8_t R=0, uint8_t G=0, uint8_t B=0): r(R), g(G), b(B) {}
};

static inline uint32_t packRGB(const RGB &c) {
  return ((uint32_t)c.r << 16) | ((uint32_t)c.g << 8) | c.b;
}
static inline RGB unpackRGB(uint32_t rgb) {
  return RGB((rgb>>16)&0xFF, (rgb>>8)&0xFF, rgb&0xFF);
}

static inline RGB hsv2rgb(uint16_t h, uint8_t s, uint8_t v) {
  // h: 0..359
  float hf = fmodf((float)h, 360.0f) / 60.0f;
  float sf = s / 255.0f;
  float vf = v / 255.0f;

  int i = (int)floorf(hf);
  float f = hf - i;
  float p = vf * (1.0f - sf);
  float q = vf * (1.0f - sf * f);
  float t = vf * (1.0f - sf * (1.0f - f));

  float r=0,g=0,b=0;
  switch (i) {
    case 0: r=vf; g=t;  b=p;  break;
    case 1: r=q;  g=vf; b=p;  break;
    case 2: r=p;  g=vf; b=t;  break;
    case 3: r=p;  g=q;  b=vf; break;
    case 4: r=t;  g=p;  b=vf; break;
    default: r=vf; g=p;  b=q;  break;
  }
  return RGB((uint8_t)roundf(r*255.0f), (uint8_t)roundf(g*255.0f), (uint8_t)roundf(b*255.0f));
}

// ---------- WS2812 RMT driver + effects ----------
class WS2812FX_RMT {
public:
  enum class Mode : uint8_t {
    OFF = 0,
    SOLID,
    BREATH,
    RAINBOW,
    COLOR_WIPE,
    FADE_TO
  };

  WS2812FX_RMT(uint16_t numLeds, uint8_t dataPin,
               rmt_channel_t ch = RMT_CHANNEL_0,
               bool grbOrder = true)
  : _n(numLeds), _pin((gpio_num_t)dataPin), _ch(ch), _grb(grbOrder) {}

  bool begin() {
    if (_n == 0) return false;

    _pixels = (uint8_t*)heap_caps_malloc(_n * 3, MALLOC_CAP_8BIT);
    if (!_pixels) return false;
    memset(_pixels, 0, _n*3);

    _items = (rmt_item32_t*)heap_caps_malloc(_n * 24 * sizeof(rmt_item32_t), MALLOC_CAP_8BIT);
    if (!_items) return false;

    // RMT setup: 80MHz APB / clk_div = 2 => 40MHz tick => 25ns per tick
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel  = _ch;
    config.gpio_num = _pin;
    config.mem_block_num = 3; // enough for 200 leds
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.clk_div = 2;

    esp_err_t e = rmt_config(&config);
    if (e != ESP_OK) return false;
    e = rmt_driver_install(_ch, 0, 0);
    if (e != ESP_OK) return false;

    // WS2812 timing in ticks @25ns:
    // T0H ~0.35us=14 ticks, T0L ~0.8us=32 ticks
    // T1H ~0.7us =28 ticks, T1L ~0.6us=24 ticks
    _t0h = 14; _t0l = 32;
    _t1h = 28; _t1l = 24;

    _mode = Mode::OFF;
    return true;
  }

  void end() {
    if (_pixels) { free(_pixels); _pixels = nullptr; }
    if (_items)  { free(_items);  _items  = nullptr; }
    rmt_driver_uninstall(_ch);
  }

  // ---------- basic strip ops ----------
  uint16_t size() const { return _n; }

  void setBrightness(uint8_t b) { _brightness = b; }
  uint8_t getBrightness() const { return _brightness; }

  void clear() { memset(_pixels, 0, _n*3); }

  void setPixel(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
    if (i >= _n) return;
    // store as RGB in buffer; order handled in show/encode
    _pixels[i*3 + 0] = r;
    _pixels[i*3 + 1] = g;
    _pixels[i*3 + 2] = b;
  }

  void setPixelRGB(uint16_t i, uint32_t rgb) {
    RGB c = unpackRGB(rgb);
    setPixel(i, c.r, c.g, c.b);
  }

  RGB getPixel(uint16_t i) const {
    if (i >= _n) return RGB();
    return RGB(_pixels[i*3+0], _pixels[i*3+1], _pixels[i*3+2]);
  }

  void fill(uint8_t r, uint8_t g, uint8_t b, uint16_t start=0, uint16_t count=0xFFFF) {
    if (start >= _n) return;
    uint16_t end = (count==0xFFFF) ? _n : (uint16_t)min<uint32_t>(_n, (uint32_t)start + count);
    for (uint16_t i=start; i<end; i++) setPixel(i, r,g,b);
  }

  void show() {
    if (!_pixels || !_items) return;

    // Encode pixels -> RMT items
    // Apply brightness scaling during encoding to keep source buffer intact
    const uint16_t itemCount = _n * 24;
    uint16_t idx = 0;

    for (uint16_t i=0; i<_n; i++) {
      uint8_t r = scale8(_pixels[i*3+0], _brightness);
      uint8_t g = scale8(_pixels[i*3+1], _brightness);
      uint8_t b = scale8(_pixels[i*3+2], _brightness);

      // WS2812 expects GRB by default
      uint8_t c0 = _grb ? g : r;
      uint8_t c1 = _grb ? r : g;
      uint8_t c2 = b;

      idx = encodeByte(_items, idx, c0);
      idx = encodeByte(_items, idx, c1);
      idx = encodeByte(_items, idx, c2);
    }

    rmt_write_items(_ch, _items, itemCount, true);
    // Reset time > 50us
    delayMicroseconds(80);
  }

  // ---------- effect control (non-blocking) ----------
  void setSolid(uint32_t rgb) {
    _mode = Mode::SOLID;
    _solid = rgb;
    applySolid();
  }

  void breath(uint32_t rgb, uint32_t periodMs) {
    _mode = Mode::BREATH;
    _solid = rgb;
    _period = max<uint32_t>(periodMs, 200);
    _startMs = millis();
  }

  void rainbow(uint32_t periodMs, bool forward=true) {
    _mode = Mode::RAINBOW;
    _period = max<uint32_t>(periodMs, 200);
    _dir = forward ? 1 : -1;
    _startMs = millis();
  }

  void colorWipe(uint32_t rgb, uint32_t speedMsPerLed) {
    _mode = Mode::COLOR_WIPE;
    _solid = rgb;
    _speed = max<uint32_t>(speedMsPerLed, 1);
    _startMs = millis();
    clear();
  }

  void fadeTo(uint32_t targetRgb, uint32_t durationMs) {
    _mode = Mode::FADE_TO;
    _target = targetRgb;
    _duration = max<uint32_t>(durationMs, 1);
    _startMs = millis();
    // capture current (assume whole strip roughly same; take pixel0 as start)
    _from = packRGB(getPixel(0));
  }

  void off() {
    _mode = Mode::OFF;
    clear();
  }

  Mode mode() const { return _mode; }

  // Call this frequently (e.g. every loop)
  void tick(uint32_t nowMs = 0) {
    if (nowMs == 0) nowMs = millis();

    switch (_mode) {
      case Mode::OFF:
        // nothing
        break;

      case Mode::SOLID:
        // already applied
        break;

      case Mode::BREATH: {
        // b = (sin(2πt/T)+1)/2
        float t = (float)((nowMs - _startMs) % _period) / (float)_period;
        float s = sinf(2.0f * (float)M_PI * t);
        float b = (s + 1.0f) * 0.5f; // 0..1
        RGB c = unpackRGB(_solid);
        uint8_t br = (uint8_t)roundf(b * 255.0f);
        // multiply color by br (not global brightness)
        fill(scale8(c.r, br), scale8(c.g, br), scale8(c.b, br));
      } break;

      case Mode::RAINBOW: {
        // offset cycles over period
        float t = (float)((nowMs - _startMs) % _period) / (float)_period;
        int offset = (int)roundf(t * 360.0f) * _dir;
        for (uint16_t i=0; i<_n; i++) {
          uint16_t h = (uint16_t)((i * 360UL) / _n);
          int hh = (int)h + offset;
          while (hh < 0) hh += 360;
          while (hh >= 360) hh -= 360;
          RGB c = hsv2rgb((uint16_t)hh, 255, 255);
          setPixel(i, c.r, c.g, c.b);
        }
      } break;

      case Mode::COLOR_WIPE: {
        uint32_t elapsed = nowMs - _startMs;
        uint16_t lit = (uint16_t)min<uint32_t>(_n, elapsed / _speed);
        RGB c = unpackRGB(_solid);
        for (uint16_t i=0; i<lit; i++) setPixel(i, c.r, c.g, c.b);
        for (uint16_t i=lit; i<_n; i++) setPixel(i, 0,0,0);
      } break;

      case Mode::FADE_TO: {
        uint32_t e = nowMs - _startMs;
        float a = (e >= _duration) ? 1.0f : (float)e / (float)_duration;
        RGB from = unpackRGB(_from);
        RGB to   = unpackRGB(_target);
        RGB cur(
          (uint8_t)roundf(from.r + (to.r - from.r) * a),
          (uint8_t)roundf(from.g + (to.g - from.g) * a),
          (uint8_t)roundf(from.b + (to.b - from.b) * a)
        );
        fill(cur.r, cur.g, cur.b);
        if (a >= 1.0f) {
          _mode = Mode::SOLID;
          _solid = _target;
        }
      } break;
    }
  }

private:
  uint16_t _n;
  gpio_num_t _pin;
  rmt_channel_t _ch;
  bool _grb = true;

  uint8_t* _pixels = nullptr;
  rmt_item32_t* _items = nullptr;

  uint8_t _brightness = 255;

  // timing ticks
  uint16_t _t0h, _t0l, _t1h, _t1l;

  // fx state
  Mode _mode = Mode::OFF;
  uint32_t _startMs = 0;

  uint32_t _solid = 0x000000;
  uint32_t _target = 0x000000;
  uint32_t _from = 0x000000;

  uint32_t _period = 2000;
  uint32_t _speed = 10;
  uint32_t _duration = 1000;
  int8_t _dir = 1;

  static inline uint8_t scale8(uint8_t v, uint8_t scale) {
    // (v * scale) / 255 with rounding
    return (uint16_t(v) * (uint16_t(scale) + 1)) >> 8;
  }

  inline uint16_t encodeByte(rmt_item32_t* items, uint16_t idx, uint8_t byte) {
    for (int bit=7; bit>=0; bit--) {
      bool one = byte & (1 << bit);
      items[idx].level0 = 1;
      items[idx].duration0 = one ? _t1h : _t0h;
      items[idx].level1 = 0;
      items[idx].duration1 = one ? _t1l : _t0l;
      idx++;
    }
    return idx;
  }

  void applySolid() {
    RGB c = unpackRGB(_solid);
    fill(c.r, c.g, c.b);
  }
};

// ---------- Screen / matrix mapping ----------
class LEDScreen {
public:
  enum class Mapping : uint8_t {
    Progressive,   // each row left->right
    Serpentine     // odd rows reversed
  };

  LEDScreen(uint16_t w, uint16_t h, Mapping m = Mapping::Serpentine)
  : _w(w), _h(h), _map(m) {}

  void attach(WS2812FX_RMT* strip, uint16_t originIndex = 0) {
    _strip = strip;
    _origin = originIndex;
  }

  void clear() {
    if (!_strip) return;
    for (uint16_t y=0; y<_h; y++)
      for (uint16_t x=0; x<_w; x++)
        drawPixel(x,y, 0x000000);
  }

  void drawPixel(int x, int y, uint32_t rgb) {
    if (!_strip) return;
    if (x < 0 || y < 0 || x >= _w || y >= _h) return;
    uint16_t idx = xyToIndex((uint16_t)x, (uint16_t)y);
    _strip->setPixelRGB(_origin + idx, rgb);
  }

  void fillRect(int x, int y, int w, int h, uint32_t rgb) {
    for (int yy=y; yy<y+h; yy++)
      for (int xx=x; xx<x+w; xx++)
        drawPixel(xx,yy,rgb);
  }

  // blit: src is array of packed 0xRRGGBB, row-major
  void blit(const uint32_t* src, uint16_t sw, uint16_t sh, int dx, int dy) {
    for (uint16_t y=0; y<sh; y++) {
      for (uint16_t x=0; x<sw; x++) {
        uint32_t c = src[y*sw + x];
        drawPixel(dx + (int)x, dy + (int)y, c);
      }
    }
  }

private:
  WS2812FX_RMT* _strip = nullptr;
  uint16_t _w, _h;
  Mapping _map;
  uint16_t _origin = 0;

  uint16_t xyToIndex(uint16_t x, uint16_t y) const {
    if (_map == Mapping::Progressive) {
      return y * _w + x;
    } else {
      // Serpentine: odd rows reversed
      if (y & 1) return y * _w + (_w - 1 - x);
      else       return y * _w + x;
    }
  }
};

WS2812FX_RMT strip(NUM_LEDS, DATA_PIN);
void ws2812_init(){
        if (!strip.begin())
    {
        Serial.println("Init failed!");
        while(1);
    }

    strip.setBrightness(120);
    strip.clear();
    strip.show();
}