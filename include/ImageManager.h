#ifndef IMAGE_RESOURCES_H
#define IMAGE_RESOURCES_H

// All icons are PNG files in theLittleFS partition, mounted at /littlefs.
// LVGL POSIX FS driver uses drive letter 'L', so paths look like "L:/music.png".
// To add a new icon: drop the PNG into data/ directory, then run:
//     pio run -t uploadfs
// To use it in code: declare `extern const char* Image_<name>;` here and define
// it in ImageManager.cpp.

extern const char* Image_music;
extern const char* Image_settings;
extern const char* Image_video;
extern const char* Image_rotation;
extern const char* Image_play;
extern const char* Image_calc;
extern const char* Image_pause;
extern const char* Image_track;
extern const char* Image_back;
extern const char* Image_explorer;
extern const char* Image_tools;
extern const char* Image_volume;
extern const char* Image_volumeup;
extern const char* Image_volumedown;
extern const char* Image_volumemute;
extern const char* Image_volume120;
extern const char* Image_torque;
extern const char* Image_wifi;
extern const char* Image_wifi_on;

#endif // IMAGE_RESOURCES_H