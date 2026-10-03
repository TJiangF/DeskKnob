/*
 * DeskKnob UI.
 *
 * A small screen framework on top of LVGL 9 with animated page transitions
 * (inspired by github.com/keysking/kk_ui): a rotary home selector, nested
 * list menus, an integer value editor and a few feature pages.
 *
 * The rotary knob feeds "gear" changes through deskknob_motor; the pressure
 * film = select, the side button = back. All LVGL access happens under the
 * esp_lvgl_port mutex from a single UI task.
 */
#include "ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "lvgl.h"

#include "display.h"
#include "input.h"
#include "motor.h"
#include "led_ring.h"
#include "wifi_mgr.h"
#include "wifi_provision.h"
#include "settings.h"
#include "salary.h"
#include "balance.h"
#include "clock_time.h"
#include "battery.h"
#include "media.h"

static const char *TAG = "ui";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- runtime theme palette ----------------------------------------------
 * C_FG / C_MUTED / C_ACCENT / C_ACCENT2 / C_CARD / C_BG are variables so the
 * whole UI can switch between dark and light at runtime. theme_apply() sets
 * them and rebuilds the current page.
 */
typedef struct {
    lv_color_t bg;
    lv_color_t fg;
    lv_color_t muted;
    lv_color_t card;
    lv_color_t accent;
    lv_color_t accent2;
    uint32_t   id;          /* persisted theme id */
} theme_t;

static const theme_t THEME_DARK = {
    .bg = LV_COLOR_MAKE(0x00, 0x00, 0x00),
    .fg = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
    .muted = LV_COLOR_MAKE(0x8A, 0x8A, 0x8E),
    .card = LV_COLOR_MAKE(0x1C, 0x1C, 0x1E),
    .accent = LV_COLOR_MAKE(0x2E, 0x9B, 0xFF),
    .accent2 = LV_COLOR_MAKE(0x18, 0xE0, 0xA0),
    .id = 0,
};

static const theme_t THEME_LIGHT = {
    .bg = LV_COLOR_MAKE(0xF2, 0xF2, 0xF7),
    .fg = LV_COLOR_MAKE(0x11, 0x11, 0x14),
    .muted = LV_COLOR_MAKE(0x6B, 0x6B, 0x72),
    .card = LV_COLOR_MAKE(0xE0, 0xE0, 0xE6),
    .accent = LV_COLOR_MAKE(0x0A, 0x76, 0xE0),
    .accent2 = LV_COLOR_MAKE(0x0B, 0x9E, 0x74),
    .id = 1,
};

static lv_color_t C_BG;
static lv_color_t C_FG;
static lv_color_t C_MUTED;
static lv_color_t C_CARD;
static lv_color_t C_ACCENT;
static lv_color_t C_ACCENT2;
static uint32_t s_theme_id;

static const theme_t *theme_by_id(uint32_t id)
{
    return (id == THEME_LIGHT.id) ? &THEME_LIGHT : &THEME_DARK;
}

static void theme_load_colors(uint32_t id)
{
    const theme_t *t = theme_by_id(id);
    s_theme_id = id;
    C_BG = t->bg;
    C_FG = t->fg;
    C_MUTED = t->muted;
    C_CARD = t->card;
    C_ACCENT = t->accent;
    C_ACCENT2 = t->accent2;
}

static lv_obj_t *s_scr;

/* ---- navigation ---- */
typedef enum {
    P_HOME = 0,
    P_MUSIC,
    P_TOOLS,
    P_SALARY,
    P_SETTINGS,
    P_THEME,
    P_MOTOR,
    P_LED,
    P_LED_MODE,
    P_RATCHET,
    P_WIFI,
    P_WIFI_PASS,
    P_WIFI_ACTION,
    P_WIFI_SETUP,
    P_BALANCE,
    P_SALARY_COUNTER,
    P_VOLUME,
    P_EDIT,
} page_t;

#define NAV_MAX 8
static page_t s_nav[NAV_MAX];
static int s_nav_depth;      /* number of entries on the stack */
static page_t s_page = P_HOME;

/* Single persistent LVGL screen; each "page" is a full-size child container
 * that we slide in/out. This avoids async screen-load issues on this panel. */
static lv_obj_t *s_root;
static lv_obj_t *s_page_cont;

static void page_build(page_t p);
static void nav_push(page_t p);
static void nav_pop(void);
static void build_wifi_setup(void);
static void lcd_bright_change(int v);
static void pick_settings(int i);
static void wifi_populate(void);
static void transition_to(lv_obj_t *new_cont, bool forward);
static void on_back(void);
static void on_back_hold(void);
static void salary_counter_render(void);
static void build_salary_counter(void);

/* ---- home wheel ---- */
#define HOME_N 4
static lv_obj_t *s_home[HOME_N];
static lv_obj_t *s_home_lbl[HOME_N];
static lv_obj_t *s_home_center;
static lv_obj_t *s_home_time;
static lv_obj_t *s_home_batt_arc;
static int s_home_sel;
static int s_home_pos100;
static bool s_home_active;
static const char *s_home_icon[HOME_N] = {
    LV_SYMBOL_AUDIO, LV_SYMBOL_EDIT, LV_SYMBOL_SETTINGS, LV_SYMBOL_CHARGE
};
static const char *s_home_text[HOME_N] = {"Music", "Tools", "Settings", "Balance"};

/* ---- generic list ---- */
#define LIST_MAX 24
static lv_obj_t *s_list_win;
static lv_obj_t *s_list_cont;
static lv_obj_t *s_list_rows[LIST_MAX];
static lv_obj_t *s_list_lbl[LIST_MAX];
static int s_list_count;
static int s_list_sel;
static void (*s_list_pick)(int index);

/* ---- generic editor ---- */
typedef struct {
    const char *title;
    const char *unit;
    int value;
    int min;
    int max;
    int step;
    void (*on_change)(int v);
} edit_cfg_t;

static edit_cfg_t s_edit;
static int s_edit_initial;

/* ---- wifi password entry ---- */
static char s_wifi_ssid[33];
static bool s_wifi_ssid_stored;
static char s_pwd[65];
static int s_pwd_len;
static int s_pwd_idx;

/* ---- misc state ---- */
static bool s_wifi_scan_seen;
static lv_obj_t *s_status_lbl;

static const char *WIFI_CHARSET =
    "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*-_+=?";
#define WIFI_CHARSET_LEN ((int)strlen(WIFI_CHARSET))

/* ======================================================================
 *  helpers
 * ==================================================================== */

static lv_obj_t *make_screen(void)
{
    lv_obj_t *scr = lv_obj_create(s_root);
    lv_obj_set_size(scr, 240, 240);
    lv_obj_set_pos(scr, 0, 0);
    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(scr, 0, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    return scr;
}

static void page_x_cb(void *o, int32_t v)
{
    lv_obj_set_x((lv_obj_t *)o, v);
}

static void old_page_done_cb(lv_anim_t *a)
{
    lv_obj_t *old = (lv_obj_t *)a->var;
    if (old && old != s_page_cont) {
        lv_obj_del(old);
    }
}

static void transition_to(lv_obj_t *new_cont, bool forward)
{
    lv_obj_t *old = s_page_cont;
    s_page_cont = new_cont;

    const int w = 240;
    int from = forward ? w : -w;

    lv_obj_set_x(new_cont, from);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, new_cont);
    lv_anim_set_exec_cb(&a, page_x_cb);
    lv_anim_set_values(&a, from, 0);
    lv_anim_set_duration(&a, 220);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);

    if (old && old != new_cont) {
        lv_obj_set_x(old, 0);
        lv_anim_t b;
        lv_anim_init(&b);
        lv_anim_set_var(&b, old);
        lv_anim_set_exec_cb(&b, page_x_cb);
        lv_anim_set_values(&b, 0, forward ? -w : w);
        lv_anim_set_duration(&b, 220);
        lv_anim_set_path_cb(&b, lv_anim_path_ease_out);
        lv_anim_set_completed_cb(&b, old_page_done_cb);
        lv_anim_start(&b);
    }
}

static lv_obj_t *add_title(lv_obj_t *scr, const char *text)
{
    lv_obj_t *l = lv_label_create(scr);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(l, C_FG, 0);
    lv_obj_align(l, LV_ALIGN_TOP_MID, 0, 16);
    return l;
}

static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region = h / 60;
    uint8_t rem = (h - region * 60) * 6;
    uint8_t p = (uint8_t)((v * (255 - s)) >> 8);
    uint8_t q = (uint8_t)((v * (255 - ((s * rem) >> 8))) >> 8);
    uint8_t t = (uint8_t)((v * (255 - ((s * (255 - rem)) >> 8))) >> 8);
    switch (region) {
    case 0: *r = v; *g = t; *b = p; break;
    case 1: *r = q; *g = v; *b = p; break;
    case 2: *r = p; *g = v; *b = t; break;
    case 3: *r = p; *g = q; *b = v; break;
    case 4: *r = t; *g = p; *b = v; break;
    default: *r = v; *g = p; *b = q; break;
    }
}

/* ======================================================================
 *  home wheel
 * ==================================================================== */

static int home_wrap_index(int v)
{
    return ((v % HOME_N) + HOME_N) % HOME_N;
}

/* Shortest signed distance between a wheel slot i and the current fractional
 * position, wrapped into [-HOME_N/2, HOME_N/2]. */
static float home_slot_delta(int i, float pos)
{
    float slot = (float)i - pos;
    slot = fmodf(slot, (float)HOME_N);
    if (slot > HOME_N / 2.0f) {
        slot -= HOME_N;
    } else if (slot < -HOME_N / 2.0f) {
        slot += HOME_N;
    }
    return slot;
}

static void home_layout(void)
{
    if (!s_home_active) {
        return;
    }
    const int R = 72;
    const int half = 27;
    float pos = (float)s_home_pos100 / 100.0f;

    /* Who is nearest the top / centered slot right now. */
    int centered = home_wrap_index((int)lroundf(pos));

    for (int i = 0; i < HOME_N; i++) {
        float slot = home_slot_delta(i, pos);
        float ang = (-90.0f + slot * (360.0f / HOME_N)) * (float)M_PI / 180.0f;
        int x = (int)lroundf(R * cosf(ang));
        int y = (int)lroundf(R * sinf(ang));
        lv_obj_set_pos(s_home[i], 120 + x - half, 120 + y - half);

        /* Binary highlight only: the selected icon is larger + glowing, all
         * others are a fixed small size. No continuous scaling while turning. */
        bool selected = (i == centered);
        float sc = selected ? 1.0f : 0.82f;
        lv_obj_set_style_transform_scale(s_home[i], (int)(256 * sc), 0);

        lv_obj_set_style_bg_color(s_home[i], selected ? C_ACCENT : C_CARD, 0);
        lv_obj_set_style_bg_opa(s_home[i], selected ? LV_OPA_COVER : LV_OPA_70, 0);
        lv_obj_set_style_text_color(s_home_lbl[i], selected ? C_FG : C_MUTED, 0);

        /* glow emphasises the selected icon */
        lv_obj_set_style_shadow_width(s_home[i], selected ? 24 : 0, 0);
        lv_obj_set_style_shadow_spread(s_home[i], selected ? 2 : 0, 0);
        lv_obj_set_style_shadow_color(s_home[i], C_ACCENT, 0);
        lv_obj_set_style_shadow_opa(s_home[i], selected ? LV_OPA_80 : LV_OPA_TRANSP, 0);
    }

    lv_label_set_text(s_home_center, s_home_text[centered]);
}

static void home_anim_cb(void *var, int32_t v)
{
    (void)var;
    s_home_pos100 = v;
    home_layout();
}

static void home_select_anim(int steps)
{
    lv_anim_delete(NULL, home_anim_cb);

    /* Animate to the absolute gear position so the wheel tracks the knob.
     * s_home_pos100 keeps growing with the (infinite) gear; the layout wraps
     * it, so we animate directly to gear*100 with no need for shortest-path. */
    (void)steps;
    int target = motor_get_gear() * 100;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &s_home_pos100);
    lv_anim_set_exec_cb(&a, home_anim_cb);
    lv_anim_set_values(&a, s_home_pos100, target);
    lv_anim_set_duration(&a, 220);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

static void build_home(void)
{
    s_scr = make_screen();
    s_home_active = true;
    s_home_sel = 0;
    s_home_pos100 = 0;

    /* 8 tick marks showing the wheel is divided into 8 equal parts */
    for (int k = 0; k < 8; k++) {
        float ang = (-90.0f + k * 45.0f) * (float)M_PI / 180.0f;
        int x = (int)lroundf(104 * cosf(ang));
        int y = (int)lroundf(104 * sinf(ang));
        lv_obj_t *t = lv_obj_create(s_scr);
        lv_obj_set_size(t, 5, 5);
        lv_obj_set_style_radius(t, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(t, C_CARD, 0);
        lv_obj_set_style_bg_opa(t, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(t, 0, 0);
        lv_obj_align(t, LV_ALIGN_CENTER, x, y);
    }

    for (int i = 0; i < HOME_N; i++) {
        lv_obj_t *o = lv_obj_create(s_scr);
        lv_obj_set_size(o, 54, 54);
        lv_obj_set_style_radius(o, 16, 0);
        lv_obj_set_style_border_width(o, 0, 0);
        lv_obj_set_style_shadow_width(o, 0, 0);
        lv_obj_set_style_transform_pivot_x(o, 27, 0);
        lv_obj_set_style_transform_pivot_y(o, 27, 0);
        lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
        s_home[i] = o;

        lv_obj_t *l = lv_label_create(o);
        lv_label_set_text(l, s_home_icon[i]);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_28, 0);
        lv_obj_center(l);
        s_home_lbl[i] = l;
    }

    /* centre label = function of the selected icon */
    s_home_center = lv_label_create(s_scr);
    lv_label_set_text(s_home_center, s_home_text[0]);
    lv_obj_set_style_text_font(s_home_center, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_home_center, C_FG, 0);
    lv_obj_align(s_home_center, LV_ALIGN_CENTER, 0, 0);

    /* bottom-left: clock (curved along the ~7 o'clock arc), pulled 5px inward.
     * Pivot stays at the label centre so the rotation is a clean tilt that
     * follows the rim tangent. */
    s_home_time = lv_label_create(s_scr);
    lv_label_set_text(s_home_time, "12:12");
    lv_obj_set_style_text_font(s_home_time, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_home_time, C_MUTED, 0);
    /* Rotate the text 45 deg so it follows the lower-left arc tangent; rotate
     * about the label centre so the ':' stays at the 225 deg radial point
     * (centre sits on the x = -y diagonal through the screen centre). */
    lv_obj_update_layout(s_home_time);
    lv_obj_set_style_transform_pivot_x(s_home_time, lv_obj_get_width(s_home_time) / 2, 0);
    lv_obj_set_style_transform_pivot_y(s_home_time, lv_obj_get_height(s_home_time) / 2, 0);
    lv_obj_set_style_transform_rotation(s_home_time, 450, 0); /* +45 deg */
    lv_obj_align(s_home_time, LV_ALIGN_CENTER, -72, 72);

    /* bottom-right: curved battery arc, hugging the rim from 4 to 5 o'clock.
     * LVGL angle convention: 0 = 3 o'clock, 90 = 6 o'clock. So the lower-right
     * sector between 5 o'clock (~30 deg) and 4 o'clock (~60 deg) is 30..60.
     * The arc's own rectangle is centred on the screen so its radius matches
     * the wheel rim. */
    s_home_batt_arc = lv_arc_create(s_scr);
    lv_obj_set_size(s_home_batt_arc, 204, 204);   /* radius +2px outward */
    lv_obj_align(s_home_batt_arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation(s_home_batt_arc, 0);
    lv_obj_set_style_arc_width(s_home_batt_arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_home_batt_arc, 6, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_home_batt_arc, C_CARD, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_home_batt_arc, C_ACCENT2, LV_PART_INDICATOR);
    lv_obj_remove_style(s_home_batt_arc, NULL, LV_PART_KNOB);
    lv_obj_remove_flag(s_home_batt_arc, LV_OBJ_FLAG_CLICKABLE);
    /* 5 o'clock .. 4 o'clock (fills upward towards the right) */
    lv_arc_set_bg_angles(s_home_batt_arc, 30, 60);
    lv_arc_set_range(s_home_batt_arc, 0, 100);
    lv_arc_set_value(s_home_batt_arc, battery_get() < 0 ? 0 : battery_get());

    home_layout();
}

/* ======================================================================
 *  generic list
 * ==================================================================== */

static void list_y_cb(void *o, int32_t v)
{
    lv_obj_set_y((lv_obj_t *)o, v);
}

static void list_update(int sel, bool animate)
{
    if (s_list_count == 0) {
        return;
    }
    if (s_list_count <= 0) {
        return;
    }
    /* Wrap so an infinite-rotation knob maps cleanly onto the list. */
    sel = ((sel % s_list_count) + s_list_count) % s_list_count;
    s_list_sel = sel;

    for (int i = 0; i < s_list_count; i++) {
        bool on = (i == sel);
        lv_obj_set_style_bg_color(s_list_rows[i], on ? C_ACCENT : C_CARD, 0);
        lv_obj_set_style_bg_opa(s_list_rows[i], on ? LV_OPA_COVER : LV_OPA_40, 0);
        lv_obj_set_style_text_color(s_list_lbl[i], on ? C_FG : C_MUTED, 0);
        lv_obj_set_style_text_font(s_list_lbl[i], on ? &lv_font_montserrat_20 : &lv_font_montserrat_18, 0);
    }

    int target = (176 / 2) - (sel * 44 + 20);
    if (animate) {
        lv_anim_delete(s_list_cont, list_y_cb);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, s_list_cont);
        lv_anim_set_exec_cb(&a, list_y_cb);
        lv_anim_set_values(&a, lv_obj_get_y(s_list_cont), target);
        lv_anim_set_duration(&a, 200);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_start(&a);
    } else {
        lv_obj_set_y(s_list_cont, target);
    }
}

static void list_begin(const char *title)
{
    s_scr = make_screen();
    add_title(s_scr, title);

    s_list_win = lv_obj_create(s_scr);
    lv_obj_set_size(s_list_win, 214, 176);
    lv_obj_align(s_list_win, LV_ALIGN_CENTER, 0, 14);
    lv_obj_set_style_bg_opa(s_list_win, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_list_win, 0, 0);
    lv_obj_set_style_pad_all(s_list_win, 0, 0);
    lv_obj_set_style_radius(s_list_win, 0, 0);
    lv_obj_set_scrollbar_mode(s_list_win, LV_SCROLLBAR_MODE_OFF);
    lv_obj_remove_flag(s_list_win, LV_OBJ_FLAG_SCROLLABLE);

    s_list_cont = lv_obj_create(s_list_win);
    lv_obj_set_size(s_list_cont, 214, 1);
    lv_obj_set_style_bg_opa(s_list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_list_cont, 0, 0);
    lv_obj_set_style_pad_all(s_list_cont, 0, 0);
    lv_obj_set_scrollbar_mode(s_list_cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_remove_flag(s_list_cont, LV_OBJ_FLAG_SCROLLABLE);

    s_list_count = 0;
    s_list_sel = 0;
}

static void list_add(const char *text)
{
    if (s_list_count >= LIST_MAX) {
        return;
    }
    int i = s_list_count++;
    lv_obj_t *row = lv_obj_create(s_list_cont);
    lv_obj_set_size(row, 214, 40);
    lv_obj_set_pos(row, 0, i * 44 + 2);
    lv_obj_set_style_radius(row, 12, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    s_list_rows[i] = row;

    lv_obj_t *l = lv_label_create(row);
    lv_label_set_text(l, text);
    lv_obj_set_width(l, 200);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(l, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_center(l);
    s_list_lbl[i] = l;
}

static void list_finish(void)
{
    lv_obj_set_height(s_list_cont, s_list_count > 0 ? s_list_count * 44 : 1);
    /* Unlimited rotation: wrap the selection so the knob never hits a wall. */
    motor_set_mode(MOTOR_MODE_TORQUE_INFINITE, 0);
    motor_set_detents(30);
    list_update(0, false);
}

/* ======================================================================
 *  generic editor
 * ==================================================================== */

static lv_obj_t *s_edit_value_lbl;
static lv_obj_t *s_edit_arc;

static void edit_render(void)
{
    if (!s_edit_value_lbl) {
        return;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", s_edit.value);
    lv_label_set_text(s_edit_value_lbl, buf);
    if (s_edit.unit && s_edit.unit[0]) {
        size_t n = strlen(buf);
        snprintf(buf + n, sizeof(buf) - n, " %s", s_edit.unit);
        lv_label_set_text(s_edit_value_lbl, buf);
    }
    int span = s_edit.max - s_edit.min;
    int frac = span > 0 ? (s_edit.value - s_edit.min) * 100 / span : 0;
    if (s_edit_arc) {
        lv_arc_set_value(s_edit_arc, frac);
    }
}

static void edit_change(int delta)
{
    int v = s_edit.value + delta * s_edit.step;
    if (v < s_edit.min) v = s_edit.min;
    if (v > s_edit.max) v = s_edit.max;
    if (v != s_edit.value) {
        s_edit.value = v;
        edit_render();
        if (s_edit.on_change) {
            s_edit.on_change(v);
        }
    }
}

static void build_editor(void)
{
    s_scr = make_screen();
    add_title(s_scr, s_edit.title);

    s_edit_arc = lv_arc_create(s_scr);
    lv_obj_set_size(s_edit_arc, 170, 170);
    lv_obj_align(s_edit_arc, LV_ALIGN_CENTER, 0, 8);
    lv_arc_set_rotation(s_edit_arc, 135);
    lv_arc_set_bg_angles(s_edit_arc, 0, 270);
    lv_arc_set_range(s_edit_arc, 0, 100);
    lv_obj_remove_style(s_edit_arc, NULL, LV_PART_KNOB);
    lv_obj_remove_flag(s_edit_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(s_edit_arc, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_edit_arc, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_edit_arc, C_CARD, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_edit_arc, C_ACCENT, LV_PART_INDICATOR);

    s_edit_value_lbl = lv_label_create(s_scr);
    lv_obj_set_style_text_font(s_edit_value_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(s_edit_value_lbl, C_FG, 0);
    lv_obj_align(s_edit_value_lbl, LV_ALIGN_CENTER, 0, 8);

    lv_obj_t *hint = lv_label_create(s_scr);
    lv_label_set_text(hint, "Turn to set   Press to save   Btn to cancel");
    lv_obj_set_width(hint, 200);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(hint, C_MUTED, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -24);

    edit_render();
}

/* ======================================================================
 *  wifi password entry
 * ==================================================================== */

static void build_wifi_setup(void)
{
    s_scr = make_screen();
    add_title(s_scr, "Phone Setup");

    lv_obj_t *l = lv_label_create(s_scr);
    lv_label_set_text_fmt(l,
        "1. On your phone connect to\n\n"
        "     " WIFI_PROV_AP_SSID "\n\n"
        "2. Open  http://192.168.4.1/\n\n"
        "3. Pick Wi-Fi & enter password");
    lv_obj_set_width(l, 210);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(l, C_FG, 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_center(l);

    lv_obj_t *hint = lv_label_create(s_scr);
    lv_label_set_text(hint, "Press to stop");
    lv_obj_set_style_text_color(hint, C_MUTED, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -26);
}

static lv_obj_t *s_pwd_lbl;
static lv_obj_t *s_pwd_char_lbl;

static void pwd_render(void)
{
    if (!s_pwd_char_lbl) {
        return;
    }
    char shown[70];
    int n = s_pwd_len;
    if (n == 0) {
        shown[0] = '\0';
    } else {
        int i;
        for (i = 0; i < n - 2 && i < 64; i++) shown[i] = '*';
        int k = i;
        for (; i < n && i < 64; i++) shown[i] = s_pwd[i];
        shown[i] = '\0';
        (void)k;
    }
    lv_label_set_text(s_pwd_lbl, shown);

    const char *show;
    if (s_pwd_idx == 0) show = "CONNECT";
    else if (s_pwd_idx == 1) show = "DEL";
    else {
        static char cb[2];
        cb[0] = WIFI_CHARSET[(s_pwd_idx - 2) % WIFI_CHARSET_LEN];
        cb[1] = 0;
        show = cb;
    }
    lv_label_set_text(s_pwd_char_lbl, show);
    lv_obj_set_style_text_color(s_pwd_char_lbl, s_pwd_idx <= 1 ? C_ACCENT : C_ACCENT2, 0);
}

static void build_wifi_pass(void)
{
    s_scr = make_screen();
    char t[40];
    snprintf(t, sizeof(t), "WiFi: %s", s_wifi_ssid);
    add_title(s_scr, t);

    s_pwd_lbl = lv_label_create(s_scr);
    lv_obj_set_style_text_font(s_pwd_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_pwd_lbl, C_FG, 0);
    lv_obj_align(s_pwd_lbl, LV_ALIGN_TOP_MID, 0, 60);

    s_pwd_char_lbl = lv_label_create(s_scr);
    lv_obj_set_style_text_font(s_pwd_char_lbl, &lv_font_montserrat_28, 0);
    lv_obj_align(s_pwd_char_lbl, LV_ALIGN_CENTER, 0, 20);

    lv_obj_t *hint = lv_label_create(s_scr);
    lv_label_set_text(hint, "Turn=pick  Press=add  Btn=del");
    lv_obj_set_style_text_color(hint, C_MUTED, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -26);

    motor_set_mode(MOTOR_MODE_TORQUE_INFINITE, 0);
    motor_set_detents(30);
    pwd_render();
}

/* ======================================================================
 *  feature pages
 * ==================================================================== */

/* Music: a list of transport actions + a dedicated volume page where turning
 * the knob emits volume up/down. */
static void pick_music(int i)
{
    static const media_key_t map[3] = {
        MEDIA_PREV_TRACK, MEDIA_PLAY_PAUSE, MEDIA_NEXT_TRACK
    };
    if (i >= 0 && i < 3) {
        media_send(map[i]);
    } else if (i == 3) {
        nav_push(P_VOLUME);
    } else if (i == 4) {
        media_send(MEDIA_MUTE);
    }
}

static void build_music(void)
{
    list_begin("Music");
    list_add(LV_SYMBOL_PREV "   Previous");
    list_add(LV_SYMBOL_PLAY "   Play / Pause");
    list_add(LV_SYMBOL_NEXT "   Next");
    list_add(LV_SYMBOL_VOLUME_MID "   Volume");
    list_add(LV_SYMBOL_MUTE "   Mute");
    s_list_pick = pick_music;
    list_finish();
}

/* ---- volume page ---- */
static lv_obj_t *s_vol_lbl;
static lv_obj_t *s_vol_bar;
static int s_vol_level;      /* 0..30 */
static int s_vol_last_gear;

static void build_volume(void)
{
    s_scr = make_screen();
    add_title(s_scr, LV_SYMBOL_VOLUME_MID "  Volume");

    s_vol_bar = lv_bar_create(s_scr);
    lv_obj_set_size(s_vol_bar, 190, 14);
    lv_obj_align(s_vol_bar, LV_ALIGN_CENTER, 0, 40);
    lv_bar_set_range(s_vol_bar, 0, 30);
    lv_obj_set_style_bg_color(s_vol_bar, C_CARD, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_vol_bar, C_ACCENT, LV_PART_INDICATOR);

    s_vol_lbl = lv_label_create(s_scr);
    lv_obj_set_style_text_font(s_vol_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(s_vol_lbl, C_FG, 0);
    lv_obj_align(s_vol_lbl, LV_ALIGN_CENTER, 0, -10);

    lv_obj_t *hint = lv_label_create(s_scr);
    lv_label_set_text(hint, "Turn to adjust volume");
    lv_obj_set_style_text_color(hint, C_MUTED, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -26);

    s_vol_level = 15;
    s_vol_last_gear = motor_get_gear();
    lv_bar_set_value(s_vol_bar, s_vol_level, LV_ANIM_OFF);
    lv_label_set_text_fmt(s_vol_lbl, "%d", s_vol_level);

    motor_set_mode(MOTOR_MODE_TORQUE_INFINITE, 0);
    motor_set_detents(30);
}

static void volume_rotate(int delta)
{
    s_vol_level += delta;
    if (s_vol_level < 0) s_vol_level = 0;
    if (s_vol_level > 30) s_vol_level = 30;
    lv_bar_set_value(s_vol_bar, s_vol_level, LV_ANIM_OFF);
    lv_label_set_text_fmt(s_vol_lbl, "%d", s_vol_level);
    if (delta > 0) {
        media_send(MEDIA_VOLUME_UP);
    } else if (delta < 0) {
        media_send(MEDIA_VOLUME_DOWN);
    }
}

static void pick_tools(int i)
{
    if (i == 0) {
        nav_push(P_SALARY);
    }
}

static void build_tools(void)
{
    list_begin("Tools");
    list_add(LV_SYMBOL_EDIT "   Salary Calculator");
    s_list_pick = pick_tools;
    list_finish();
}

/* salary */
typedef enum { SAL_MONTHLY = 0, SAL_HOURS, SAL_DAYS } sal_field_t;

static void salary_edit_monthly(int v)
{
    salary_set_monthly_cents((int64_t)v * 100); /* editor works in yuan */
}
static void salary_edit_hours(int v)
{
    salary_set_hours_x10(v);
}
static void salary_edit_days(int v)
{
    salary_set_days_x10(v);
}

static void open_editor(const char *title, const char *unit, int value, int min, int max,
                        int step, void (*on_change)(int))
{
    s_edit.title = title;
    s_edit.unit = unit;
    s_edit.value = value;
    s_edit.min = min;
    s_edit.max = max;
    s_edit.step = step;
    s_edit.on_change = on_change;
    s_edit_initial = value;
    nav_push(P_EDIT);
    motor_set_mode(MOTOR_MODE_TORQUE_INFINITE, 0);
    motor_set_detents(30);
}

static void pick_salary(int i)
{
    switch (i) {
    case SAL_MONTHLY:
        open_editor("Monthly salary", "CNY", (int)(salary_get_monthly_cents() / 100),
                    0, 100000000, 100, salary_edit_monthly);
        break;
    case SAL_HOURS:
        open_editor("Hours / day", "h", salary_get_hours_x10(), 1, 240, 1, salary_edit_hours);
        break;
    case SAL_DAYS:
        open_editor("Days / month", "d", salary_get_days_x10(), 1, 310, 1, salary_edit_days);
        break;
    case 3: /* start / open the live counter */
        if (!salary_counter_running()) {
            salary_counter_start();
        }
        nav_push(P_SALARY_COUNTER);
        break;
    }
}

static void build_salary(void)
{
    list_begin("Salary");
    char b[56];
    int64_t m = salary_get_monthly_cents();
    snprintf(b, sizeof(b), "Monthly  %lld.%02lld CNY", (long long)(m / 100), (long long)(m % 100));
    list_add(b);
    snprintf(b, sizeof(b), "Hours    %d.%d h/day",
             salary_get_hours_x10() / 10, salary_get_hours_x10() % 10);
    list_add(b);
    snprintf(b, sizeof(b), "Days     %d.%d d/month",
             salary_get_days_x10() / 10, salary_get_days_x10() % 10);
    list_add(b);
    sprintf(b, "Start earning >>");
    list_add(b);
    s_list_pick = pick_salary;
    list_finish();
}

/* ---- live earnings counter ---- */
static lv_obj_t *s_salary_earn_lbl;
static lv_obj_t *s_salary_time_lbl;
static lv_obj_t *s_salary_rate_lbl;

static void build_salary_counter(void)
{
    s_scr = make_screen();
    add_title(s_scr, "Earnings");

    s_salary_earn_lbl = lv_label_create(s_scr);
    lv_obj_set_style_text_font(s_salary_earn_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(s_salary_earn_lbl, C_ACCENT2, 0);
    lv_obj_align(s_salary_earn_lbl, LV_ALIGN_CENTER, 0, -14);

    s_salary_time_lbl = lv_label_create(s_scr);
    lv_obj_set_style_text_font(s_salary_time_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_salary_time_lbl, C_MUTED, 0);
    lv_obj_align(s_salary_time_lbl, LV_ALIGN_CENTER, 0, 30);

    s_salary_rate_lbl = lv_label_create(s_scr);
    lv_obj_set_style_text_font(s_salary_rate_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_salary_rate_lbl, C_MUTED, 0);
    lv_obj_align(s_salary_rate_lbl, LV_ALIGN_BOTTOM_MID, 0, -28);
}

static void salary_counter_render(void)
{
    if (!s_salary_earn_lbl) {
        return;
    }
    if (!salary_counter_running()) {
        salary_counter_start();
    }
    int64_t c = salary_counter_earned_cents();
    lv_label_set_text_fmt(s_salary_earn_lbl, "%lld.%02lld", (long long)(c / 100), (long long)(c % 100));

    double sec = salary_counter_elapsed_sec();
    int h = (int)(sec / 3600.0);
    int mi = (int)((sec - h * 3600.0) / 60.0);
    int s = (int)(sec - h * 3600.0 - mi * 60.0);
    lv_label_set_text_fmt(s_salary_time_lbl, "%02d:%02d:%02d", h, mi, s);

    lv_label_set_text_fmt(s_salary_rate_lbl, "Hold button to reset");
}

/* settings */
static void pick_settings(int i)
{
    if (i == 0) {
        nav_push(P_LED);
    } else if (i == 1) {
        nav_push(P_WIFI);
    } else if (i == 2) {
        open_editor("LCD brightness", "%", display_get_brightness(), 0, 100, 5, lcd_bright_change);
    } else if (i == 3) {
        nav_push(P_RATCHET);
    } else if (i == 4) {
        /* Start the phone provisioning AP and show instructions. */
        wifi_provision_start();
        nav_push(P_WIFI_SETUP);
    } else if (i == 5) {
        nav_push(P_MOTOR);
    } else if (i == 6) {
        nav_push(P_THEME);
    }
}

static void pick_theme(int i)
{
    uint32_t id = (i == 1) ? THEME_LIGHT.id : THEME_DARK.id;
    if (id != s_theme_id) {
        theme_load_colors(id);
        settings_set_int(SET_THEME, (int32_t)id);
        /* Rebuild the settings menu in the new colors. */
        s_nav_depth = s_nav_depth > 1 ? s_nav_depth - 1 : 1;
        page_build(P_SETTINGS);
        transition_to(s_scr, true);
    }
}

static void build_theme(void)
{
    list_begin("Theme");
    list_add("Dark");
    list_add("Light");
    s_list_pick = pick_theme;
    list_finish();
    list_update(s_theme_id == THEME_LIGHT.id ? 1 : 0, false);
}

static void build_settings(void)
{
    list_begin("Settings");
    list_add(LV_SYMBOL_TINT "   LED Ring");
    list_add(LV_SYMBOL_WIFI "   WiFi");
    list_add(LV_SYMBOL_EYE_OPEN "   LCD Brightness");
    list_add(LV_SYMBOL_LOOP "   Ratchet Feel");
    list_add(LV_SYMBOL_KEYBOARD "   WiFi Phone Setup");
    list_add(LV_SYMBOL_SETTINGS "   Motor Tuning");
    list_add(LV_SYMBOL_IMAGE "   Theme");
    s_list_pick = pick_settings;
    list_finish();
}

/* ---- motor tuning page (persisted drive parameters) ---- */
static void mt_voltage(int v)  { motor_set_voltage_limit((float)v / 10.0f);
                                 settings_set_int(SET_MOTOR_VOLTAGE, v); }
static void mt_current(int v)  { motor_set_current_limit((float)v / 100.0f);
                                 settings_set_int(SET_MOTOR_CURRENT, v); }
static void mt_vellim(int v)   { motor_set_velocity_limit((float)v);
                                 settings_set_int(SET_MOTOR_VEL_LIMIT, v); }
static void mt_kp(int v)       { motor_set_kp_angle((float)v / 10.0f);
                                 settings_set_int(SET_MOTOR_KP_ANGLE, v); }
static void mt_pid_p(int v)    { float p, i, d, r; motor_get_pid_velocity(&p, &i, &d, &r);
                                 motor_set_pid_velocity((float)v / 1000.0f, i, d, r);
                                 settings_set_int(SET_MOTOR_PID_P, v); }
static void mt_pid_i(int v)    { float p, i, d, r; motor_get_pid_velocity(&p, &i, &d, &r);
                                 motor_set_pid_velocity(p, (float)v / 100.0f, d, r);
                                 settings_set_int(SET_MOTOR_PID_I, v); }
static void mt_pid_d(int v)    { float p, i, d, r; motor_get_pid_velocity(&p, &i, &d, &r);
                                 motor_set_pid_velocity(p, i, (float)v / 100000.0f, r);
                                 settings_set_int(SET_MOTOR_PID_D, v); }
static void mt_ramp(int v)     { float p, i, d, r; motor_get_pid_velocity(&p, &i, &d, &r);
                                 motor_set_pid_velocity(p, i, d, (float)v);
                                 settings_set_int(SET_MOTOR_PID_RAMP, v); }

static void pick_motor(int i)
{
    float p, iv, d, r;
    motor_get_pid_velocity(&p, &iv, &d, &r);
    switch (i) {
    case 0: open_editor("Drive voltage", "V", (int)(motor_get_voltage_limit() * 10), 5, 33, 1, mt_voltage); break;
    case 1: open_editor("Current limit", "A", (int)(motor_get_current_limit() * 100), 5, 100, 5, mt_current); break;
    case 2: open_editor("Velocity limit", "rad/s", (int)motor_get_velocity_limit(), 5, 200, 5, mt_vellim); break;
    case 3: open_editor("Angle P gain", "x0.1", (int)(motor_get_kp_angle() * 10), 1, 200, 1, mt_kp); break;
    case 4: open_editor("Vel PID P", "x0.001", (int)(p * 1000), 1, 2000, 5, mt_pid_p); break;
    case 5: open_editor("Vel PID I", "x0.01", (int)(iv * 100), 0, 2000, 5, mt_pid_i); break;
    case 6: open_editor("Vel PID D", "x1e-5", (int)(d * 100000), 0, 5000, 5, mt_pid_d); break;
    case 7: open_editor("Vel ramp", "", (int)r, 10, 1000, 10, mt_ramp); break;
    }
}

static void build_motor(void)
{
    list_begin("Motor Tuning");
    char b[40];
    snprintf(b, sizeof(b), "Voltage    %.1f V", (double)motor_get_voltage_limit());
    list_add(b);
    snprintf(b, sizeof(b), "Current    %.2f A", (double)motor_get_current_limit());
    list_add(b);
    snprintf(b, sizeof(b), "Vel limit  %d rad/s", (int)motor_get_velocity_limit());
    list_add(b);
    snprintf(b, sizeof(b), "Angle P    %.1f", (double)motor_get_kp_angle());
    list_add(b);
    float p, iv, d, r;
    motor_get_pid_velocity(&p, &iv, &d, &r);
    snprintf(b, sizeof(b), "PID P      %.3f", (double)p);
    list_add(b);
    snprintf(b, sizeof(b), "PID I      %.2f", (double)iv);
    list_add(b);
    snprintf(b, sizeof(b), "PID D      %.5f", (double)d);
    list_add(b);
    snprintf(b, sizeof(b), "PID ramp   %d", (int)r);
    list_add(b);
    s_list_pick = pick_motor;
    list_finish();
}

/* LED */
static void pick_led_mode(int i)
{
    static const led_mode_t map[4] = {LED_MODE_OFF, LED_MODE_SOLID, LED_MODE_RAINBOW, LED_MODE_BREATHE};
    if (i >= 0 && i < 4) {
        led_ring_set_mode(map[i]);
    }
}

static void led_bright_change(int v)
{
    led_ring_set_brightness(v);
}

static void led_hue_change(int v)
{
    uint8_t r, g, b;
    hsv_to_rgb((uint16_t)v, 255, 255, &r, &g, &b);
    led_ring_set_color(r, g, b);
}

static void pick_led(int i)
{
    if (i == 0) {
        nav_push(P_LED_MODE);
    } else if (i == 1) {
        open_editor("LED brightness", "%", led_ring_get_brightness(), 0, 100, 5, led_bright_change);
    } else if (i == 2) {
        uint8_t r, g, b;
        led_ring_get_color(&r, &g, &b);
        /* approximate current hue from stored color; default 0 */
        open_editor("LED hue", "deg", 0, 0, 359, 10, led_hue_change);
        (void)r; (void)g; (void)b;
    }
}

static void build_led(void)
{
    list_begin("LED Ring");
    list_add(LV_SYMBOL_SETTINGS "   Mode");
    list_add(LV_SYMBOL_PLUS "   Brightness");
    list_add(LV_SYMBOL_TINT "   Color");
    s_list_pick = pick_led;
    list_finish();
}

static void build_led_mode(void)
{
    list_begin("LED Mode");
    list_add("Off");
    list_add("Solid");
    list_add("Rainbow");
    list_add("Breathe");
    s_list_pick = pick_led_mode;
    list_finish();
}

/* LCD brightness */
static void lcd_bright_change(int v)
{
    display_set_brightness(v);
}

/* Ratchet */
static void ratchet_detents_change(int v)
{
    motor_set_detents(v);
}
static void ratchet_k_change(int v)
{
    motor_set_stiffness((float)v / 10.0f);
}

static void pick_ratchet(int i)
{
    if (i == 0) {
        open_editor("Detents", "n", motor_get_detents(), 2, 60, 1, ratchet_detents_change);
    } else {
        open_editor("Stiffness", "x0.1", (int)(motor_get_stiffness() * 10), 5, 150, 1, ratchet_k_change);
    }
}

static void build_ratchet(void)
{
    list_begin("Ratchet");
    list_add(LV_SYMBOL_LOOP "   Detents");
    list_add(LV_SYMBOL_BARS "   Stiffness");
    s_list_pick = pick_ratchet;
    list_finish();
}

/* WiFi */
static void pick_wifi(int i)
{
    if (i < 0) return;
    const char *ssid = wifi_mgr_scan_ssid(i);
    if (ssid[0] == '\0') {
        return;
    }
    strlcpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid));
    s_wifi_ssid_stored = wifi_mgr_has_credentials(ssid);
    if (s_wifi_ssid_stored) {
        nav_push(P_WIFI_ACTION);
    } else {
        s_pwd[0] = '\0';
        s_pwd_len = 0;
        s_pwd_idx = 0;
        nav_push(P_WIFI_PASS);
    }
}

static void build_wifi(void)
{
    list_begin("WiFi");
    if (wifi_mgr_scan_count() > 0 && wifi_mgr_scan_done()) {
        wifi_populate();
    } else {
        list_add("Scanning...");
        s_list_pick = pick_wifi;
        list_finish();
    }
    s_wifi_scan_seen = false;
    wifi_mgr_scan_start();
}

static const char *stored_pass_for(const char *ssid)
{
    for (int i = 0; i < wifi_mgr_stored_count(); i++) {
        if (strcmp(wifi_mgr_stored_ssid(i), ssid) == 0) {
            return wifi_mgr_stored_pass(i);
        }
    }
    return "";
}

static void wifi_action(int i)
{
    if (i == 0) {
        wifi_mgr_connect(s_wifi_ssid, stored_pass_for(s_wifi_ssid), false);
    } else if (i == 1) {
        wifi_mgr_forget(s_wifi_ssid);
    }
    nav_pop();
}

static void build_wifi_action(void)
{
    list_begin(s_wifi_ssid);
    list_add(LV_SYMBOL_WIFI "   Connect");
    list_add(LV_SYMBOL_TRASH "   Forget");
    s_list_pick = wifi_action;
    list_finish();
}

static void pwd_do_connect(void)
{
    bool persist = true;
    wifi_mgr_connect(s_wifi_ssid, s_pwd, persist);
    /* pop back to wifi list */
    while (s_nav_depth > 1 &&
           (s_nav[s_nav_depth - 1] == P_WIFI_PASS || s_nav[s_nav_depth - 1] == P_WIFI_ACTION)) {
        s_nav_depth--;
    }
    page_build(P_WIFI);
    transition_to(s_scr, false);
    s_page = P_WIFI;
}

/* balance */
static lv_obj_t *s_balance_lbl;
static lv_obj_t *s_balance_sub;

static void build_balance(void)
{
    s_scr = make_screen();
    add_title(s_scr, "DeepSeek Balance");

    s_balance_lbl = lv_label_create(s_scr);
    lv_obj_set_style_text_font(s_balance_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(s_balance_lbl, C_ACCENT2, 0);
    lv_obj_align(s_balance_lbl, LV_ALIGN_CENTER, 0, 0);

    s_balance_sub = lv_label_create(s_scr);
    lv_obj_set_style_text_color(s_balance_sub, C_MUTED, 0);
    lv_obj_set_style_text_font(s_balance_sub, &lv_font_montserrat_14, 0);
    lv_obj_align(s_balance_sub, LV_ALIGN_CENTER, 0, 46);

    lv_obj_t *hint = lv_label_create(s_scr);
    lv_label_set_text(hint, "Press to refresh");
    lv_obj_set_style_text_color(hint, C_MUTED, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -26);
}

static void balance_render(void)
{
    if (!s_balance_lbl) {
        return;
    }
    balance_info_t info;
    balance_get(&info);
    if (info.busy) {
        lv_label_set_text(s_balance_lbl, "...");
        lv_label_set_text(s_balance_sub, "fetching");
    } else if (info.valid) {
        char b[32];
        snprintf(b, sizeof(b), "%.2f", info.total);
        lv_label_set_text(s_balance_lbl, b);
        char s[40];
        snprintf(s, sizeof(s), "%s  %s", info.currency, info.available ? "available" : "unavailable");
        lv_label_set_text(s_balance_sub, s);
    } else {
        lv_label_set_text(s_balance_lbl, "--");
        lv_label_set_text(s_balance_sub, info.message[0] ? info.message : "no data");
    }
}

/* ======================================================================
 *  page routing
 * ==================================================================== */

static void page_build(page_t p)
{
    s_home_active = false;
    s_status_lbl = NULL;
    s_edit_value_lbl = NULL;
    s_edit_arc = NULL;
    s_pwd_lbl = NULL;
    s_pwd_char_lbl = NULL;
    s_balance_lbl = NULL;
    s_balance_sub = NULL;
    s_home_time = NULL;
    s_home_batt_arc = NULL;

    switch (p) {
    case P_HOME:        build_home(); break;
    case P_MUSIC:       build_music(); break;
    case P_TOOLS:       build_tools(); break;
    case P_SALARY:      build_salary(); break;
    case P_SETTINGS:    build_settings(); break;
    case P_THEME:       build_theme(); break;
    case P_MOTOR:       build_motor(); break;
    case P_LED:         build_led(); break;
    case P_LED_MODE:    build_led_mode(); break;
    case P_RATCHET:     build_ratchet(); break;
    case P_WIFI:        build_wifi(); break;
    case P_WIFI_PASS:   build_wifi_pass(); break;
    case P_WIFI_ACTION: build_wifi_action(); break;
    case P_WIFI_SETUP:  build_wifi_setup(); break;
    case P_BALANCE:     build_balance(); break;
    case P_SALARY_COUNTER: build_salary_counter(); break;
    case P_VOLUME:      build_volume(); break;
    case P_EDIT:        build_editor(); break;
    default:            build_home(); break;
    }
    s_page = p;
}

static void nav_push(page_t p)
{
    if (s_nav_depth < NAV_MAX) {
        s_nav[s_nav_depth++] = p;
    }
    page_build(p);
    transition_to(s_scr, true);
}

static void nav_pop(void)
{
    if (s_nav_depth > 1) {
        s_nav_depth--;
        page_t prev = s_nav[s_nav_depth - 1];
        page_build(prev);
        transition_to(s_scr, false);
    }
}

static void home_enter(void)
{
    switch (home_wrap_index(s_home_sel)) {
    case 0: nav_push(P_MUSIC); break;
    case 1: nav_push(P_TOOLS); break;
    case 2: nav_push(P_SETTINGS); break;
    case 3: nav_push(P_BALANCE); balance_request_refresh(); break;
    }
}

/* ======================================================================
 *  input handling
 * ==================================================================== */

static void on_rotate(int delta, int gear)
{
    (void)gear;
    switch (s_page) {
    case P_HOME:
        s_home_sel = motor_get_gear();
        home_select_anim(delta);
        break;
    case P_EDIT:
        edit_change(delta);
        break;
    case P_WIFI_PASS: {
        int n = WIFI_CHARSET_LEN + 2;
        s_pwd_idx = ((motor_get_gear() % n) + n) % n;
        pwd_render();
        break;
    }
    case P_VOLUME:
        volume_rotate(delta);
        break;
    case P_MUSIC:
    case P_TOOLS:
    case P_SALARY:
    case P_SETTINGS:
    case P_THEME:
    case P_MOTOR:
    case P_LED:
    case P_LED_MODE:
    case P_RATCHET:
    case P_WIFI_ACTION:
        list_update(motor_get_gear(), true);
        break;
    default:
        break;
    }
}

static void on_press(void)
{
    switch (s_page) {
    case P_HOME:
        home_enter();
        break;
    case P_MUSIC:
    case P_TOOLS:
    case P_SALARY:
    case P_SETTINGS:
    case P_THEME:
    case P_MOTOR:
    case P_LED:
    case P_LED_MODE:
    case P_RATCHET:
    case P_WIFI_ACTION:
        if (s_list_pick) {
            s_list_pick(s_list_sel);
        }
        break;
    case P_EDIT:
        if (s_edit.on_change) {
            s_edit.on_change(s_edit.value);
        }
        nav_pop();
        break;
    case P_WIFI_PASS:
        if (s_pwd_idx == 0) {
            pwd_do_connect();
        } else if (s_pwd_idx == 1) {
            if (s_pwd_len > 0) s_pwd[s_pwd_len--] = '\0';
            pwd_render();
        } else {
            if (s_pwd_len < (int)sizeof(s_pwd) - 1) {
                s_pwd[s_pwd_len++] = WIFI_CHARSET[(s_pwd_idx - 2) % WIFI_CHARSET_LEN];
                s_pwd[s_pwd_len] = '\0';
            }
            pwd_render();
        }
        break;
    case P_BALANCE:
        balance_request_refresh();
        break;
    default:
        break;
    }
}

static void on_back_hold(void)
{
    if (s_page == P_SALARY_COUNTER) {
        salary_counter_reset();
        salary_counter_render();
        return;
    }
    on_back();
}

static void on_back(void)
{
    switch (s_page) {
    case P_HOME:
        break;
    case P_WIFI_SETUP:
        wifi_provision_stop();
        nav_pop();
        break;
    case P_EDIT:
        if (s_edit.on_change) {
            s_edit.on_change(s_edit_initial);
        }
        nav_pop();
        break;
    case P_WIFI_PASS:
        if (s_pwd_len > 0) {
            s_pwd[s_pwd_len--] = '\0';
            pwd_render();
        } else {
            nav_pop();
        }
        break;
    default:
        nav_pop();
        break;
    }
}

/* ======================================================================
 *  periodic updates
 * ==================================================================== */

/* Fill the already-begun WiFi list from cached scan results. */
static void wifi_populate(void)
{
    const char *cur = wifi_mgr_current_ssid();
    int n = wifi_mgr_scan_count();
    int added = 0;
    for (int i = 0; i < n && added < LIST_MAX; i++) {
        const char *ssid = wifi_mgr_scan_ssid(i);
        bool is_cur = wifi_mgr_is_connected() && strcmp(ssid, cur) == 0;
        bool known = wifi_mgr_has_credentials(ssid);
        char row[52];
        snprintf(row, sizeof(row), "%s%s%s", is_cur ? LV_SYMBOL_OK "  " : "",
                 known ? LV_SYMBOL_SAVE "  " : "", ssid);
        list_add(row);
        added++;
    }
    if (added == 0) {
        list_add("No networks found");
    }
    s_list_pick = pick_wifi;
    list_finish();
}

/* Rebuild the WiFi screen from fresh results without starting a new scan. */
static void wifi_refresh_screen(void)
{
    list_begin("WiFi");
    wifi_populate();
    transition_to(s_scr, true);
}

/* Self-healing refresh: if no screen transition animation is running and the
 * active screen is not being animated, periodically invalidate it so a dropped
 * SPI flush cannot leave the panel black. */
static uint32_t s_last_heal_ms;

static void ui_heal_refresh(void)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - s_last_heal_ms < 1000) {
        return;
    }
    s_last_heal_ms = now;
    if (lv_anim_count_running() == 0 && s_page_cont) {
        lv_obj_invalidate(s_page_cont);
    }
}

/* Update the home status row. Clock is currently a fixed placeholder; the
 * battery is a small curved arc with no numeric label. */
static void home_render_status(void)
{
    if (s_home_batt_arc) {
        int lvl = battery_get();
        lv_arc_set_value(s_home_batt_arc, lvl < 0 ? 0 : lvl);
    }
}

static void ui_tick(void)
{
    if (s_page == P_WIFI) {
        if (wifi_mgr_scan_done() && !s_wifi_scan_seen) {
            s_wifi_scan_seen = true;
            wifi_refresh_screen();
        }
    } else if (s_page == P_BALANCE) {
        balance_render();
    } else if (s_page == P_SALARY_COUNTER) {
        salary_counter_render();
    } else if (s_page == P_HOME) {
        home_render_status();
    }
    ui_heal_refresh();
}

/* ======================================================================
 *  main task
 * ==================================================================== */

static void ui_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(200));

    theme_load_colors((uint32_t)settings_get_int(SET_THEME, 0));

    if (display_lock(0)) {
        s_root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(s_root, C_BG, 0);
        lv_obj_set_style_bg_opa(s_root, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(s_root, 0, 0);
        lv_obj_set_style_border_width(s_root, 0, 0);
        lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);
        lv_screen_load(s_root);

        s_nav_depth = 0;
        page_build(P_HOME);
        s_page_cont = s_scr;
        s_nav[s_nav_depth++] = P_HOME;
        motor_set_mode(MOTOR_MODE_TORQUE_INFINITE, 0);
        motor_set_detents(30);
        display_unlock();
    }

    int last_gear = motor_get_gear();
    uint32_t last_alive = 0;

    for (;;) {
        int gear = motor_get_gear();
        if (gear != last_gear) {
            int delta = gear - last_gear;
            last_gear = gear;
            if (display_lock(1000)) {
                on_rotate(delta, gear);
                display_unlock();
            } else {
                ESP_LOGW(TAG, "rotate: display lock timeout");
            }
        }

        input_msg_t ev;
        if (input_wait(&ev, 20)) {
            ESP_LOGI(TAG, "event type=%d page=%d", ev.type, (int)s_page);
            if (display_lock(1000)) {
                if (ev.type == INPUT_EVT_PRESS) {
                    on_press();
                } else if (ev.type == INPUT_EVT_BACK) {
                    on_back();
                } else if (ev.type == INPUT_EVT_BACK_HOLD) {
                    on_back_hold();
                }
                display_unlock();
            } else {
                ESP_LOGW(TAG, "event: display lock timeout");
            }
            /* motor mode may have changed while building a page */
            last_gear = motor_get_gear();
        } else {
            if (display_lock(200)) {
                ui_tick();
                display_unlock();
            }
        }

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_alive > 3000) {
            last_alive = now;
            ESP_LOGI(TAG, "alive page=%d gear=%d depth=%d heap=%u",
                     (int)s_page, motor_get_gear(), s_nav_depth,
                     (unsigned)esp_get_free_heap_size());
        }
    }
}

esp_err_t ui_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(ui_task, "deskknob_ui", 8192, NULL, 5, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
