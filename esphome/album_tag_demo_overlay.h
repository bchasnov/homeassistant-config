#pragma once
// Drawing helpers for album-tag-demo.yaml.
//
// - The mural's four-point sparkle as an anti-aliased parametric shape.
// - An overlay task that animates the loaders (~30 fps) and the heart rain
//   (~60 fps) by sending pixels straight to the panel. It runs on
//   the other core, so it keeps moving while the main loop is stuck in a slow
//   HTTPS download. Every normal screen update goes through update_screen(),
//   which shares a lock with the task so the two never talk to the panel at once.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>

#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esphome/components/display/display.h"
#include "esphome/components/image/image.h"
#include "esphome/core/color.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace album_tag_demo {

using esphome::Color;

inline Color blend(Color a, Color b, float t) {
  t = std::min(1.0f, std::max(0.0f, t));
  return Color((uint8_t) (a.r + (b.r - a.r) * t), (uint8_t) (a.g + (b.g - a.g) * t),
               (uint8_t) (a.b + (b.b - a.b) * t));
}

// RGB565 with the bytes swapped, the order the panel's buffer uses.
inline uint16_t to_panel(Color c) {
  const uint16_t v = ((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3);
  return __builtin_bswap16(v);
}

// ---------------------------------------------------------------------------
// Mural sparkle

// Fitted to a photo of the mural: four tapered arms (each a capsule of radius
// 0.20 at the center ending in a round cap of radius 0.10, so the tips are
// blunt and round, not pointed), blended with smooth fillets (0.5) that give
// the curved, concave sides. That matches the mural's arm half-widths (0.43,
// 0.21, 0.12, 0.10 at 25/50/75/90% of the arm) to within ~0.02. Horizontal
// arms are ~0.87 of the vertical ones. Lengths are in units of `size` (center
// to top tip, in pixels).
struct Sparkle {
  float up = 1.0f, down = 1.0f, left = 0.87f, right = 0.87f;
  float lean = 0.0f;    // radians
  float sway_x = 0.0f;  // how far the vertical arms' tips bend sideways (units of size)
  float sway_y = 0.0f;  // how far the horizontal arms' tips bend up or down
};
constexpr float SPARKLE_BASE = 0.20f;
constexpr float SPARKLE_TIP = 0.10f;
constexpr float SPARKLE_FILLET = 0.5f;

class SparkleField {
 public:
  SparkleField(const Sparkle &s, float size) : s_(s), size_(size), cos_(cosf(s.lean)), sin_(sinf(s.lean)) {}

  // Signed distance in pixels from (x, y), relative to the center; negative inside.
  float distance(float x, float y) const {
    float u = (x * this->cos_ + y * this->sin_) / this->size_;
    float v = (-x * this->sin_ + y * this->cos_) / this->size_;
    // Bend the arms: the further along an arm, the further it's pushed.
    u -= this->s_.sway_x * v * v;
    v -= this->s_.sway_y * u * u;
    const float up = arm_(u, -v, this->s_.up), down = arm_(u, v, this->s_.down);
    const float left = arm_(v, -u, this->s_.left), right = arm_(v, u, this->s_.right);
    return smin_(smin_(smin_(up, right), down), left) * this->size_;
  }

  // Half-width of a box that always contains the shape.
  int reach() const {
    const float arm = std::max({this->s_.up, this->s_.down, this->s_.left, this->s_.right});
    return (int) ceilf(this->size_ * (arm + std::max(fabsf(this->s_.sway_x), fabsf(this->s_.sway_y)))) + 2;
  }

 private:
  // A capsule from the center (radius SPARKLE_BASE) along +y to a round tip
  // at `length` (radius SPARKLE_TIP); after Inigo Quilez's uneven capsule.
  static float arm_(float x, float y, float length) {
    x = fabsf(x);
    const float h = std::max(0.01f, length - SPARKLE_TIP);
    const float b = (SPARKLE_BASE - SPARKLE_TIP) / h;
    const float a = sqrtf(std::max(0.0f, 1 - b * b));
    const float k = -b * x + a * y;
    if (k < 0)
      return sqrtf(x * x + y * y) - SPARKLE_BASE;
    if (k > a * h)
      return sqrtf(x * x + (y - h) * (y - h)) - SPARKLE_TIP;
    return a * x + b * y - SPARKLE_BASE;
  }
  // Smooth minimum: rounds the inside corners where arms meet.
  static float smin_(float a, float b) {
    const float h = std::max(SPARKLE_FILLET - fabsf(a - b), 0.0f) / SPARKLE_FILLET;
    return std::min(a, b) - h * h * SPARKLE_FILLET * 0.25f;
  }

  Sparkle s_;
  float size_, cos_, sin_;
};

// Draws an anti-aliased sparkle centered on (cx, cy), blending edges into bg.
inline void draw_sparkle(esphome::display::Display &it, int cx, int cy, float size, const Sparkle &shape,
                         Color color, Color bg) {
  const SparkleField field(shape, size);
  const int r = field.reach();
  for (int y = cy - r; y <= cy + r; y++) {
    for (int x = cx - r; x <= cx + r; x++) {
      const float cover = 0.5f - field.distance((float) (x - cx), (float) (y - cy));
      if (cover <= 0.0f)
        continue;
      it.draw_pixel_at(x, y, cover >= 1.0f ? color : blend(bg, color, cover));
    }
  }
}

// Signed distance to a unit heart with its bottom tip at the origin and y up
// (after Inigo Quilez); it's about 1.2 wide and 1.1 tall.
inline float heart_distance(float x, float y) {
  x = fabsf(x);
  if (x + y > 1.0f)
    return sqrtf((x - 0.25f) * (x - 0.25f) + (y - 0.75f) * (y - 0.75f)) - 0.35355339f;
  const float d1 = x * x + (y - 1.0f) * (y - 1.0f);
  const float m = 0.5f * std::max(x + y, 0.0f);
  const float d2 = (x - m) * (x - m) + (y - m) * (y - m);
  return sqrtf(std::min(d1, d2)) * (x > y ? 1.0f : -1.0f);
}

// ---------------------------------------------------------------------------
// Overlay task

// Loader colors: a nearly black mesh lit only by the drifting stars.
struct Palette {
  Color dark{4, 9, 8};  // unlit facets, just off the black background
  Color stars[3] = {Color(230, 172, 62), Color(246, 241, 231), Color(242, 78, 62)};  // mustard, cream, red
};

class Overlay {
 public:
  static Overlay &get() {
    static Overlay overlay;
    return overlay;
  }

  // D is the display's concrete type; the heart rain reads its frame buffer
  // (a copy of what's on screen) to restore what's behind each heart.
  template<typename D> void start(D *display) {
    if (this->task_ != nullptr)
      return;
    this->display_ = display;
    this->screen_ = FrameBuffer<D>::get(display);
    this->screen_w_ = display->get_width();
    this->screen_h_ = display->get_height();
    this->chunk_ = static_cast<uint16_t *>(
        heap_caps_malloc(MAX_WIDTH * CHUNK_ROWS * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    xTaskCreatePinnedToCore(&Overlay::task_entry_, "album_overlay", 4096, this, 1, &this->task_, 0);
  }

  void lock() { xSemaphoreTake(this->mutex_, portMAX_DELAY); }
  void unlock() { xSemaphoreGive(this->mutex_); }

  // Called from the display lambda on every frame (with the lock held): the
  // areas that are loading. They're all windows onto one dark, star-lit mesh
  // that spans the screen.
  void set_loading(const std::vector<esphome::display::Rect> &areas) {
    this->loading_.clear();
    for (const auto &a : areas)
      this->loading_.push_back({a.x, a.y, a.w, a.h});
  }

  // Called by the display lambda on full frames, which repaint the loading
  // areas too: the loader then resends everything instead of skipping chunks
  // it thinks are already on the panel.
  void invalidate() { this->resend_all_ = true; }

  // Rains hearts down the whole screen. Tapping again during the rain sends
  // a fresh shower.
  void start_hearts() {
    if (this->screen_ == nullptr)
      return;
    this->lock();
    const uint32_t now = millis_();
    for (auto &heart : this->hearts_) {
      heart.x = (float) (esp_random() % this->screen_w_);
      heart.size = 8.0f + (esp_random() % 900) / 100.0f;
      heart.start_ms = now + esp_random() % 700;
      heart.speed = 220.0f + esp_random() % 140;
      heart.sway = 4.0f + (esp_random() % 1000) / 100.0f;
      heart.sway_hz = 1.0f + (esp_random() % 150) / 100.0f;
      heart.phase = (esp_random() % 628) / 100.0f;
      heart.color = esp_random() % 3;
      heart.drawn = false;
    }
    this->hearts_active_ = true;
    this->unlock();
  }

 private:
  struct Box {
    int x = 0, y = 0, w = 0, h = 0;
  };
  struct Heart {
    float x, size, speed, sway, sway_hz, phase;
    uint32_t start_ms;
    uint8_t color;
    bool drawn;   // whether `last` is on the panel and needs restoring
    Box last;     // where it was drawn last frame
  };
  // Reaches the display's protected frame buffer.
  template<typename D> struct FrameBuffer : D {
    static uint16_t *get(D *display) { return reinterpret_cast<uint16_t *>(static_cast<FrameBuffer *>(display)->buffer_); }
  };
  static constexpr int MAX_WIDTH = 320;
  static constexpr int CHUNK_ROWS = 16;
  static constexpr int MESH_CELL_PX = 46;  // about how big each mesh cell is
  static constexpr uint32_t MESH_FRAME_MS = 33;
  static constexpr uint32_t HEARTS_FRAME_MS = 16;
  static constexpr int STARS = 6;
  static constexpr float LIGHT_RADIUS = 120.0f;  // light reaches no further

  Overlay() { this->mutex_ = xSemaphoreCreateMutex(); }

  static uint32_t millis_() { return (uint32_t) (esp_timer_get_time() / 1000); }

  static void task_entry_(void *arg) { static_cast<Overlay *>(arg)->run_(); }

  void run_() {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
      this->lock();
      const bool hearts = this->hearts_active_;
      const std::vector<Box> loading = this->loading_;
      if (hearts)
        this->draw_hearts_();
      this->unlock();
      uint32_t frame_ms = 0;
      if (hearts) {
        frame_ms = HEARTS_FRAME_MS;
      } else if (!loading.empty()) {
        this->draw_loaders_(loading);  // locks only while sending pixels
        frame_ms = MESH_FRAME_MS;
      }
      if (frame_ms == 0) {
        vTaskDelay(pdMS_TO_TICKS(40));
        wake = xTaskGetTickCount();
      } else {
        // Fixed cadence; if a frame ran long, the next starts right away.
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(frame_ms));
      }
    }
  }

  // Sends a w x rows block to the panel at (x, y). The block starts at
  // (x_offset, y_offset) inside `pixels`, whose rows are `stride` pixels wide
  // (0 = exactly w).
  void blit_(int x, int y, int w, int rows, const uint16_t *pixels, int x_offset = 0, int y_offset = 0,
             int stride = 0) {
    this->display_->draw_pixels_at(x, y, w, rows, reinterpret_cast<const uint8_t *>(pixels),
                                   esphome::display::COLOR_ORDER_RGB, esphome::display::COLOR_BITNESS_565, true,
                                   x_offset, y_offset, stride ? stride - w - x_offset : 0);
  }

  // --- Loaders: a screen-wide low-poly mesh, nearly black, lit only by mural
  // stars sitting on its nodes. Stars twinkle on and off at random nodes with
  // random timing (riding along as the mesh morphs), and each lights the
  // facets around it; every facet has a fixed random tilt, so it catches the
  // light at its own angle and the surface flashes into view, then falls back
  // into the dark. Each loading area is a window onto this world.

  float hash_(int i, int j, int salt) const {
    uint32_t v = (uint32_t) i * 73856093u ^ (uint32_t) j * 19349663u ^ (uint32_t) salt * 83492791u;
    v ^= v >> 13;
    v *= 0x5bd1e995u;
    v ^= v >> 15;
    return (v % 2001) / 1000.0f - 1.0f;
  }

  // Screen-space vertex: fixed jitter plus an irregular drift (two sines with
  // random speeds). Screen-edge vertices slide along their edge; corners stay
  // put; offsets stay under ~0.4 of a cell so triangles never fold over.
  void vertex_(int i, int j, float t, float &x, float &y) const {
    const float cw = (float) this->screen_w_ / this->cols_, ch = (float) this->screen_h_ / this->rows_;
    x = cw * i;
    y = ch * j;
    if (i > 0 && i < this->cols_) {
      x += cw * (0.24f * this->hash_(i, j, 1) +
                 0.09f * sinf(t * (0.6f + 0.4f * this->hash_(i, j, 5)) + 6.28f * this->hash_(i, j, 6)) +
                 0.06f * sinf(t * (1.7f + 0.8f * this->hash_(i, j, 9)) + 6.28f * this->hash_(i, j, 10)));
    }
    if (j > 0 && j < this->rows_) {
      y += ch * (0.24f * this->hash_(i, j, 2) +
                 0.09f * sinf(t * (0.5f + 0.4f * this->hash_(i, j, 7)) + 6.28f * this->hash_(i, j, 8)) +
                 0.06f * sinf(t * (1.5f + 0.8f * this->hash_(i, j, 11)) + 6.28f * this->hash_(i, j, 12)));
    }
  }

  struct Star {
    float x, y, power, size;
    Color color;
  };

  // Star i at time t (seconds): each life it sits on a random interior node,
  // fading in and out with a flicker; lives have random lengths.
  Star star_(int i, float t) const {
    const float life = 1.8f + 0.9f * (this->hash_(i, 0, 21) + 1);  // 1.8-3.6 s
    const float clock = t / life + 0.41f * i;
    const int gen = (int) floorf(clock);
    const float u = clock - gen;
    auto rnd = [&](int k) { return 0.5f + 0.5f * this->hash_(i, gen, 30 + k); };  // 0..1
    const int ni = 1 + std::min(this->cols_ - 2, (int) (rnd(0) * (this->cols_ - 1)));
    const int nj = 1 + std::min(this->rows_ - 2, (int) (rnd(1) * (this->rows_ - 1)));
    Star star;
    this->vertex_(ni, nj, t, star.x, star.y);
    const float flicker = 0.85f + 0.1f * sinf(t * 23 + i * 3) + 0.05f * sinf(t * 37 + i);
    star.power = powf(sinf(M_PI * u), 0.8f) * flicker * (0.7f + 0.3f * rnd(2));
    star.size = (4.0f + 4.0f * rnd(3)) * (0.5f + 0.5f * star.power);
    const Palette p;
    star.color = p.stars[rnd(4) < 0.15f ? 2 : rnd(4) < 0.6f ? 0 : 1];  // red is rare
    return star;
  }

  // Triangles are filled one scanline at a time at pixel centers; edges are
  // always evaluated top to bottom, so neighbours meet with no gaps or overlap.
  struct Triangle {
    float ax, ay, bx, by, cx, cy;  // sorted by y
    uint16_t color;
  };

  void draw_loaders_(const std::vector<Box> &areas) {
    const int64_t started = esp_timer_get_time();
    if (this->cols_ == 0) {
      this->cols_ = std::max(3, (this->screen_w_ + MESH_CELL_PX / 2) / MESH_CELL_PX);
      this->rows_ = std::max(3, (this->screen_h_ + MESH_CELL_PX / 2) / MESH_CELL_PX);
    }
    const uint32_t now = millis_();
    const float t = now / 1000.0f;
    const Palette p;

    Star stars[STARS];
    for (int i = 0; i < STARS; i++)
      stars[i] = this->star_(i, t);

    // Light reaching a point from every star: a tilted facet facing the star
    // catches more, and it falls off with distance like a flashlight.
    auto lit = [&](float x, float y, float nx, float ny, float nz) {
      float r = p.dark.r, g = p.dark.g, b = p.dark.b;
      for (const Star &star : stars) {
        if (star.power <= 0.01f)
          continue;
        const float dx = star.x - x, dy = star.y - y, dz = 34.0f;
        const float flat2 = dx * dx + dy * dy;
        if (flat2 >= LIGHT_RADIUS * LIGHT_RADIUS)
          continue;  // a hard edge, so unlit facets stay exactly dark and unchanged
        const float dist2 = flat2 + dz * dz;
        const float facing = std::max(0.0f, (nx * dx + ny * dy + nz * dz) / sqrtf(dist2));
        const float reach = 1.0f - flat2 / (LIGHT_RADIUS * LIGHT_RADIUS);
        const float amount = 1.5f * star.power * facing * reach * reach * reach;
        r += star.color.r * amount;
        g += star.color.g * amount;
        b += star.color.b * amount;
      }
      return Color((uint8_t) std::min(255.0f, r), (uint8_t) std::min(255.0f, g), (uint8_t) std::min(255.0f, b));
    };

    // This frame's vertices and lit triangles.
    const int stride = this->cols_ + 1;
    std::vector<float> vx(stride * (this->rows_ + 1)), vy(vx.size());
    for (int j = 0; j <= this->rows_; j++)
      for (int i = 0; i <= this->cols_; i++)
        this->vertex_(i, j, t, vx[j * stride + i], vy[j * stride + i]);
    auto &tris = this->triangles_;
    tris.clear();
    auto add = [&](int a, int b, int c, int salt_i, int salt_j, int salt) {
      struct Pt {
        float x, y;
      } pts[3] = {{vx[a], vy[a]}, {vx[b], vy[b]}, {vx[c], vy[c]}};
      std::sort(pts, pts + 3, [](const Pt &l, const Pt &r) { return l.y < r.y; });
      // A fixed random tilt per facet.
      float nx = 0.55f * this->hash_(salt_i, salt_j, salt), ny = 0.55f * this->hash_(salt_i, salt_j, salt + 1);
      const float nz = sqrtf(std::max(0.2f, 1 - nx * nx - ny * ny));
      const Color color = lit((pts[0].x + pts[1].x + pts[2].x) / 3, (pts[0].y + pts[1].y + pts[2].y) / 3, nx, ny, nz);
      tris.push_back({pts[0].x, pts[0].y, pts[1].x, pts[1].y, pts[2].x, pts[2].y, to_panel(color)});
    };
    for (int j = 0; j < this->rows_; j++) {
      for (int i = 0; i < this->cols_; i++) {
        const int a = j * stride + i, b = a + 1, q = a + stride, d = q + 1;
        // Alternate the diagonal so it doesn't read as a plain grid.
        if ((i + j) % 2) {
          add(a, b, d, i, j, 40);
          add(a, d, q, i, j, 42);
        } else {
          add(a, b, q, i, j, 40);
          add(b, d, q, i, j, 42);
        }
      }
    }

    // Tiny glints on vertices a star is lighting up.
    struct Glint {
      int x, y;
      float size;
      Color color;
    };
    std::vector<Glint> glints;
    for (int j = 1; j < this->rows_; j++) {
      for (int i = 1; i < this->cols_; i++) {
        const int k = j * stride + i;
        const Color c = lit(vx[k], vy[k], 0, 0, 1);
        const float level = (c.r + c.g + c.b) / 3.0f / 255.0f;
        if (level > 0.35f)
          glints.push_back({(int) vx[k], (int) vy[k], 1.5f + 3.0f * level, c});
      }
    }

    // Sparkle shapes for drawing: the stars themselves, then the glints.
    auto draw_sparkle_in_chunk = [&](const Box &area, int y0, int rows, float sx, float sy, float size,
                                     Color color) {
      const SparkleField field(Sparkle{}, size);
      const int reach = field.reach();
      const int cx = (int) sx - area.x, cy = (int) sy - area.y;
      for (int y = std::max(cy - reach, y0); y <= std::min(cy + reach, y0 + rows - 1); y++) {
        for (int x = std::max(cx - reach, 0); x <= std::min(cx + reach, area.w - 1); x++) {
          const float cover = 0.5f - field.distance((float) (x - cx), (float) (y - cy));
          if (cover <= 0.0f)
            continue;
          uint16_t &px = this->chunk_[(y - y0) * area.w + x];
          px = cover >= 1.0f ? to_panel(color) : to_panel(blend(from_panel_(px), color, cover));
        }
      }
    };

    int64_t blit_us = 0;
    int sent = 0, skipped = 0;
    if (this->chunk_hashes_.size() != areas.size() || this->resend_all_.exchange(false))
      this->chunk_hashes_.assign(areas.size(), {});
    for (size_t n = 0; n < areas.size(); n++) {
      const Box &area = areas[n];
      if (area.w <= 0 || area.h <= 0 || area.w > MAX_WIDTH)
        continue;
      auto &hashes = this->chunk_hashes_[n];
      if (hashes.size() != (size_t) (area.h + CHUNK_ROWS - 1) / CHUNK_ROWS)
        hashes.assign((area.h + CHUNK_ROWS - 1) / CHUNK_ROWS, 0);
      for (int y0 = 0; y0 < area.h; y0 += CHUNK_ROWS) {
        const int rows = std::min(CHUNK_ROWS, area.h - y0);
        const float top = area.y + y0, bottom = area.y + y0 + rows;
        for (const Triangle &tri : tris) {
          if (tri.cy <= top || tri.ay >= bottom || tri.cy == tri.ay)
            continue;
          for (int y = std::max((int) top, (int) tri.ay); y < std::min((int) bottom, (int) tri.cy + 1); y++) {
            const float yc = y + 0.5f;
            if (yc < tri.ay || yc >= tri.cy)
              continue;
            float xl = tri.ax + (yc - tri.ay) * (tri.cx - tri.ax) / (tri.cy - tri.ay);
            float xr = yc < tri.by ? tri.ax + (yc - tri.ay) * (tri.bx - tri.ax) / (tri.by - tri.ay)
                                   : tri.bx + (yc - tri.by) * (tri.cx - tri.bx) / (tri.cy - tri.by);
            if (xl > xr)
              std::swap(xl, xr);
            const int xs = std::max(area.x, (int) ceilf(xl - 0.5f));
            const int xe = std::min(area.x + area.w, (int) ceilf(xr - 0.5f));
            if (xe > xs) {
              uint16_t *row = &this->chunk_[(y - (int) top) * area.w - area.x];
              std::fill(row + xs, row + xe, tri.color);
            }
          }
        }
        for (const Glint &glint : glints)
          draw_sparkle_in_chunk(area, y0, rows, glint.x, glint.y, glint.size, glint.color);
        for (const Star &star : stars) {
          if (star.power > 0.05f)
            draw_sparkle_in_chunk(area, y0, rows, star.x, star.y, star.size, blend(p.dark, star.color, star.power * 1.3f));
        }

        // Most of the frame is unlit and unchanged; only send what changed.
        uint32_t hash = 2166136261u;
        for (int i = 0; i < rows * area.w; i++)
          hash = (hash ^ this->chunk_[i]) * 16777619u;
        const size_t chunk_index = y0 / CHUNK_ROWS;
        const int64_t blit_start = esp_timer_get_time();
        this->lock();
        // The loaders may have changed, or a full frame repainted them, while
        // this frame was drawing.
        const bool still = n < this->loading_.size() && this->loading_[n].y == area.y && this->loading_[n].h == area.h;
        if (this->resend_all_) {
          std::fill(hashes.begin(), hashes.end(), 0);
        }
        if (!still) {
          // Dropped; the next frame picks up the new areas.
        } else if (hash == hashes[chunk_index]) {
          skipped++;
        } else {
          this->blit_(area.x, area.y + y0, area.w, rows, this->chunk_);
          hashes[chunk_index] = hash;
          sent++;
        }
        this->unlock();
        blit_us += esp_timer_get_time() - blit_start;
      }
    }

    // Frame stats every 5 s.
    this->stat_frames_++;
    this->stat_total_us_ += esp_timer_get_time() - started;
    this->stat_blit_us_ += blit_us;
    this->stat_sent_ += sent;
    this->stat_skipped_ += skipped;
    if (now - this->stat_since_ >= 5000) {
      if (this->stat_since_ != 0 && this->stat_frames_ > 1) {
        esph_log_i("album_overlay", "Loader: %.1f fps, %u ms/frame (%u ms sending incl. waits), %u%% of chunks unchanged",
                   this->stat_frames_ * 1000.0f / (now - this->stat_since_),
                   (unsigned) (this->stat_total_us_ / this->stat_frames_ / 1000),
                   (unsigned) (this->stat_blit_us_ / this->stat_frames_ / 1000),
                   (unsigned) (100 * this->stat_skipped_ / std::max<uint32_t>(1, this->stat_sent_ + this->stat_skipped_)));
      }
      this->stat_since_ = now;
      this->stat_frames_ = this->stat_sent_ = this->stat_skipped_ = 0;
      this->stat_total_us_ = this->stat_blit_us_ = 0;
    }
  }

  static Color from_panel_(uint16_t px) {
    const uint16_t v = __builtin_bswap16(px);
    return Color((v >> 8) & 0xF8, (v >> 3) & 0xFC, (v << 3) & 0xF8);
  }

  // --- Heart rain. Each frame restores the patch each heart covered last frame
  // from the frame buffer, draws every heart touching it, and sends just that
  // patch to the panel.

  // Where a heart is at time `now`, or false once it's fallen off the screen.
  bool heart_box_(const Heart &heart, uint32_t now, Box &box, float &hx, float &hy) const {
    const float t = (now - heart.start_ms) / 1000.0f;
    hy = heart.speed * t;  // bottom tip, falling from the top edge
    hx = heart.x + heart.sway * sinf(2 * M_PI * heart.sway_hz * t + heart.phase);
    if (hy - heart.size * 1.15f >= this->screen_h_)
      return false;
    box.x = std::max(0, (int) floorf(hx - heart.size * 0.65f));
    box.y = std::max(0, (int) floorf(hy - heart.size * 1.15f));
    box.w = std::min(this->screen_w_ - 1, (int) ceilf(hx + heart.size * 0.65f)) - box.x + 1;
    box.h = std::min(this->screen_h_ - 1, (int) ceilf(hy + 1)) - box.y + 1;
    return box.w > 0 && box.h > 0;
  }

  void draw_hearts_() {
    const uint32_t now = millis_();
    const Color colors[3] = {Color(242, 78, 62), Color(230, 172, 62), Color(246, 241, 231)};
    constexpr int N = sizeof(this->hearts_) / sizeof(this->hearts_[0]);
    Box now_box[N];
    float now_x[N], now_y[N];
    bool visible[N];
    bool any_left = false;
    for (int i = 0; i < N; i++) {
      const Heart &heart = this->hearts_[i];
      const bool started = (int32_t) (now - heart.start_ms) >= 0;
      visible[i] = started && this->heart_box_(heart, now, now_box[i], now_x[i], now_y[i]);
      any_left |= visible[i] || !started;
    }

    for (int i = 0; i < N; i++) {
      Heart &heart = this->hearts_[i];
      if (!visible[i] && !heart.drawn)
        continue;
      // Patch = last frame's spot plus this frame's.
      Box patch = heart.drawn ? heart.last : now_box[i];
      if (heart.drawn && visible[i]) {
        const int x2 = std::max(patch.x + patch.w, now_box[i].x + now_box[i].w);
        const int y2 = std::max(patch.y + patch.h, now_box[i].y + now_box[i].h);
        patch.x = std::min(patch.x, now_box[i].x);
        patch.y = std::min(patch.y, now_box[i].y);
        patch.w = x2 - patch.x;
        patch.h = y2 - patch.y;
      }
      for (int y0 = patch.y; y0 < patch.y + patch.h; y0 += CHUNK_ROWS * MAX_WIDTH / std::max(1, patch.w)) {
        const int rows = std::min(CHUNK_ROWS * MAX_WIDTH / std::max(1, patch.w), patch.y + patch.h - y0);
        for (int r = 0; r < rows; r++)
          memcpy(&this->chunk_[r * patch.w], &this->screen_[(y0 + r) * this->screen_w_ + patch.x],
                 patch.w * sizeof(uint16_t));
        // Every visible heart overlapping this patch, so overlaps stay correct.
        for (int j = 0; j < N; j++) {
          if (!visible[j])
            continue;
          const Box &b = now_box[j];
          const int ys = std::max(b.y, y0), ye = std::min(b.y + b.h, y0 + rows);
          const int xs = std::max(b.x, patch.x), xe = std::min(b.x + b.w, patch.x + patch.w);
          const float scale = this->hearts_[j].size;
          for (int y = ys; y < ye; y++) {
            for (int x = xs; x < xe; x++) {
              const float d = heart_distance((x + 0.5f - now_x[j]) / scale, (now_y[j] - (y + 0.5f)) / scale) * scale;
              const float cover = 0.5f - d;
              if (cover <= 0.0f)
                continue;
              uint16_t &px = this->chunk_[(y - y0) * patch.w + (x - patch.x)];
              const Color color = colors[this->hearts_[j].color];
              px = cover >= 1.0f ? to_panel(color) : to_panel(blend(from_panel_(px), color, cover));
            }
          }
        }
        this->blit_(patch.x, y0, patch.w, rows, this->chunk_);
      }
      heart.drawn = visible[i];
      heart.last = now_box[i];
    }
    if (!any_left)
      this->hearts_active_ = false;
  }

  esphome::display::Display *display_{nullptr};
  SemaphoreHandle_t mutex_{nullptr};
  TaskHandle_t task_{nullptr};
  uint16_t *chunk_{nullptr};

  std::vector<Box> loading_;
  int cols_{0}, rows_{0};
  std::vector<Triangle> triangles_;
  std::vector<std::vector<uint32_t>> chunk_hashes_;  // per loading area, last sent hash of each chunk
  std::atomic<bool> resend_all_{false};
  uint32_t stat_since_{0}, stat_frames_{0}, stat_sent_{0}, stat_skipped_{0};
  int64_t stat_total_us_{0}, stat_blit_us_{0};

  uint16_t *screen_{nullptr};
  int screen_w_{0}, screen_h_{0};
  bool hearts_active_{false};
  Heart hearts_[20];
};

// Redraws the screen through the display lambda without racing the overlay task.
inline void update_screen(esphome::display::Display *display) {
  Overlay::get().lock();
  display->update();
  Overlay::get().unlock();
}

}  // namespace album_tag_demo
