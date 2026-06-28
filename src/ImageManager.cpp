#include "ImageManager.h"

// LittleFS file paths; LVGL POSIX FS driver letter is 'L',
// so "L:/music.png" resolves to "/littlefs/music.png" on ESP32.
// Matching PNG files live in data/ and are flashed with `pio run -t uploadfs`.

const char* Image_music      = "L:/music.png";
const char* Image_settings   = "L:/settings.png";
const char* Image_video      = "L:/video.png";
const char* Image_rotation   = "L:/rotation.png";
const char* Image_play       = "L:/play.png";
const char* Image_calc       = "L:/calculator.png";
const char* Image_pause      = "L:/pause.png";
const char* Image_track      = "L:/track.png";
const char* Image_back       = "L:/back.png";
const char* Image_explorer   = "L:/explorer.png";
const char* Image_tools      = "L:/tools.png";
const char* Image_volume      = "L:/volume.png";
const char* Image_volumeup    = "L:/volumeup.png";
const char* Image_volumedown  = "L:/volumedown.png";
const char* Image_volumemute  = "L:/volumemute.png";
const char* Image_volume120  = "L:/volume120.png";