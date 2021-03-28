#include <stdint.h>
#include <print.h>
#include "annepro2.h"
#include "qmk_ap2_led.h"
#ifdef ANNEPRO2_C18
#include "eeprom_w25x20cl.h"
#endif

// layout using eeprom and bidir-comms to keep user led settings persistent

// eeprom memory layout
typedef union {
    uint32_t raw;
    struct {
        uint8_t magic : 8;
        bool leds_on : 1;
        uint8_t leds_profile : 8;
    };
} user_config_t;

// define out default user_config
user_config_t user_config = {.magic = 0xDE, .leds_on = 0, .leds_profile = 0};

// keep the number of profiles so we can track along with the shine proc
uint8_t numProfiles = 0;

static uint8_t usb_buf[256];
static uint8_t buf_fil = 0;

enum anne_pro_layers {
    _COLEMAK_CAW_L,
    _QWERTY_L,
    _FN1_L,
    _FNW_L,
    _FN2_L,
};

// This is ROW*MATRIX_COLS + COL
#define CAPS_LOCATION (MATRIX_COLS * 2 + 0)
#ifdef _______
#define _______ KC_TRNS
#endif

const uint16_t keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_COLEMAK_CAW_L] = KEYMAP(
		KC_ESC,     KC_1,    KC_2,    KC_3, KC_4, KC_5,   KC_EQL,  KC_6, KC_7, KC_8,    KC_9,    KC_0,       KC_MINS,    KC_BSPC,
		KC_TAB,     KC_Q,    KC_W,    KC_F, KC_P, KC_B,   KC_LBRC, KC_J, KC_L, KC_U,    KC_Y,    KC_SCLN,    KC_QUOT,    KC_BSLS,
		MO(_FNW_L), KC_A,    KC_R,    KC_S, KC_T, KC_G,   KC_RBRC, KC_M, KC_N, KC_E,    KC_I,    KC_O,                   KC_ENT,
		KC_LSFT,    KC_X,    KC_C,    KC_D, KC_V, KC_Z,   KC_SLSH, KC_K, KC_H, KC_COMM, KC_DOT,  KC_RSFT,
		KC_LCTL,    KC_LGUI, KC_LALT,             KC_SPC,                               KC_RALT, MO(_FN1_L), MO(_FN2_L), KC_RCTL),

    [_COLEMAK_L] = KEYMAP(
		KC_ESC,     KC_1,    KC_2,    KC_3, KC_4, KC_5,   KC_6, KC_7, KC_8,    KC_9,   KC_0,    KC_MINS,    KC_EQL,     KC_BSPC,
		KC_TAB,     KC_Q,    KC_W,    KC_F, KC_P, KC_G,   KC_J, KC_L, KC_U,    KC_Y,   KC_SCLN, KC_LBRC,    KC_RBRC,    KC_BSLS,
		MO(_FN1_L), KC_A,    KC_R,    KC_S, KC_T, KC_D,   KC_H, KC_N, KC_E,    KC_I,   KC_O,    KC_QUOT,                KC_ENT,
		KC_LSFT,    KC_Z,    KC_X,    KC_C, KC_V, KC_B,   KC_K, KC_M, KC_COMM, KC_DOT, KC_SLSH, KC_RSFT,
		KC_LCTL,    KC_LGUI, KC_LALT,             KC_SPC,                              KC_RALT, MO(_FN1_L), MO(_FN2_L), KC_RCTL),

    [_COLEMAK_DH_L] = KEYMAP(
		KC_ESC,     KC_1,    KC_2,    KC_3, KC_4, KC_5,   KC_6, KC_7, KC_8,    KC_9,   KC_0,    KC_MINS,    KC_EQL,     KC_BSPC,
		KC_TAB,     KC_Q,    KC_W,    KC_F, KC_P, KC_B,   KC_J, KC_L, KC_U,    KC_Y,   KC_SCLN, KC_LBRC,    KC_RBRC,    KC_BSLS,
		MO(_FN1_L), KC_A,    KC_R,    KC_S, KC_T, KC_G,   KC_M, KC_N, KC_E,    KC_I,   KC_O,    KC_QUOT,                KC_ENT,
		KC_LSFT,    KC_X,    KC_C,    KC_D, KC_V, KC_Z,   KC_K, KC_H, KC_COMM, KC_DOT, KC_SLSH, KC_RSFT,
		KC_LCTL,    KC_LGUI, KC_LALT,             KC_SPC,                              KC_RALT, MO(_FN1_L), MO(_FN2_L), KC_RCTL),

    [_QWERTY_L] = KEYMAP(
        KC_ESC,     KC_1,    KC_2,    KC_3, KC_4, KC_5,   KC_6, KC_7, KC_8,    KC_9,   KC_0,    KC_MINS,    KC_EQL,     KC_BSPC,
        KC_TAB,     KC_Q,    KC_W,    KC_E, KC_R, KC_T,   KC_Y, KC_U, KC_I,    KC_O,   KC_P,    KC_LBRC,    KC_RBRC,    KC_BSLS,
        MO(_FN1_L), KC_A,    KC_S,    KC_D, KC_F, KC_G,   KC_H, KC_J, KC_K,    KC_L,   KC_SCLN, KC_QUOT,                KC_ENT,
        KC_LSFT,    KC_Z,    KC_X,    KC_C, KC_V, KC_B,   KC_N, KC_M, KC_COMM, KC_DOT, KC_SLSH, KC_RSFT,
        KC_LCTL,    KC_LGUI, KC_LALT,             KC_SPC,                              KC_RALT, MO(_FN1_L), MO(_FN2_L), KC_RCTL),

    [_FN1_L] = KEYMAP(
		KC_GRV,  KC_F1,            KC_F2,            KC_F3,              KC_F4,         KC_F5,      KC_F6,      KC_F7,   KC_F8,         KC_F9,        KC_F10,                 KC_F11,  KC_F12,  KC_DEL,
		_______, _______,          KC_HOME,          KC_PGUP,            _______,       _______,    _______,    KC_PSCR, _______,       KC_PSCR,      _______,                _______, _______, _______,
		_______, _______,          KC_END,           KC_PGDN,            _______,       _______,    KC_LEFT,    KC_DOWN, KC_UP,         KC_RGHT,      _______,                _______, _______,
		_______, LCTL(KC_Z),       LCTL(KC_X),       LCTL(KC_C),         LCTL(KC_V),    _______,    _______,    _______, _______,       KC_INS,       KC_DEL,                 _______,
		_______, _______,          _______,          _______,            _______,       _______,    MO(_FN2_L), _______),

    [_FNW_L] = KEYMAP(
		KC_GRV,  KC_F1,            KC_F2,            KC_F3,              KC_F4,         KC_F5,      KC_F6,      KC_F7,   KC_F8,         KC_F9,        KC_F10,                 KC_F11,  KC_F12,  KC_DEL,
		_______, _______,          KC_HOME,          KC_PGUP,            _______,       _______,    _______,    _______, KC_PSCR,       _______,      _______,                _______, _______, _______,
		_______, _______,          KC_END,           KC_PGDN,            _______,       _______,    _______,    KC_LEFT, KC_DOWN,       KC_UP,        KC_RIGHT,               _______, _______,
		_______, LCTL(KC_Z),       LCTL(KC_X),       LCTL(KC_C),         LCTL(KC_V),    _______,    _______,    _______, _______,       KC_INS,       KC_DEL,                 _______,
		_______, _______,          _______,          _______,            _______,       _______,    MO(_FN2_L), _______),

    [_FN2_L] = KEYMAP(
		_______, DF(_COLEMAK_CAW_L),  DF(_QWERTY_L), _______,            _______,       _______,    _______,    _______, KC_AP_LED_OFF, KC_AP_LED_ON, KC_AP_LED_NEXT_PROFILE, _______, _______, _______,
		_______, _______,         KC_UP,             _______,            _______,       _______,    _______,    _______, _______,       _______,      _______,                _______, _______, _______,
		_______, KC_LEFT,         KC_DOWN,           KC_RGHT,            _______,       _______,    _______,    _______, _______,       _______,      _______,                _______, _______,
		_______, _______,         _______,           _______,            _______,       _______,    _______,    _______, _______,       _______,      _______,                _______,
		_______, _______,         _______,           _______,            _______,       MO(_FN1_L), MO(_FN2_L), _______),
};

const uint16_t keymaps_size = sizeof(keymaps);

void matrix_init_user(void) {}

void matrix_scan_user(void) {}

layer_state_t layer_state_set_user(layer_state_t layer) { return layer; }

void raw_hid_receive(uint8_t *data, uint8_t length) {
  uprintf("raw_hid len: %u\n", length);
  if (length == 1)
    annepro2LedSetProfile(data[0]);
  else {
    for (uint8_t i = 0; i < length; i++){
      usb_buf[buf_fil + i] = data[i];
    }
    buf_fil += length;
    if (buf_fil >= 211) {
      sdWrite(&SD0, usb_buf, 211);
      buf_fil = 0;
    }
//    for (int i = 0; i < length; i++) {
//      sdPut(&SD0, data[i]);
//      sdGet(&SD0);
//    }
  }
}

/*!
 * @returns false   processing for this keycode has been completed.
 */
bool process_record_user(uint16_t keycode, keyrecord_t* record)
{
#ifdef ANNEPRO2_C18
    switch (keycode) {
    case KC_AP_LED_OFF:
        if (record->event.pressed) {
            user_config.leds_on = false;
            eeprom_write((void*)&user_config, 0, sizeof(user_config_t));
        }
        return false;
    case KC_AP_LED_ON:
        if (record->event.pressed) {
            user_config.leds_on = true;
            eeprom_write((void*)&user_config, 0, sizeof(user_config_t));
        }
        return false;
    case KC_AP_LED_NEXT_PROFILE:
        if (record->event.pressed) {
            user_config.leds_profile = (user_config.leds_profile + 1) % numProfiles;
            annepro2LedSetProfile(user_config.leds_profile);
            eeprom_write((void*)&user_config, 0, sizeof(user_config_t));
        }
        return false;
    default:
        break;
    }
#endif
    return true;
}

void keyboard_post_init_user(void)
{
    // Customize these values to desired behavior
    debug_enable = true;
    //debug_matrix = true;
    //debug_keyboard=true;
    //debug_mouse=true;

#ifdef ANNEPRO2_C18
    // Read the user config from EEPROM
    eeprom_read((void*)&user_config, 0, sizeof(user_config_t));

    // initialize a new eeprom
    if (user_config.magic != 0xDE)
    {
        user_config.magic = 0xDE;
        user_config.leds_on = false;
        user_config.leds_profile = 0;
        eeprom_write((void*)&user_config, 0, sizeof(user_config_t));
    }

    if (user_config.leds_on) {
        // send profile before so that we don't get a flicker on startup
        annepro2LedSetProfile(user_config.leds_profile);
        annepro2LedEnable();
    } else {
        annepro2LedDisable();
    }
#endif

    numProfiles = annepro2LedGetNumProfiles();
}
