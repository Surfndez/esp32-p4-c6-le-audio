#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/* USB Serial/JTAG console through its driver: never blocks a logging task when no host reads. First in app_main. */
void log_sink_usb_init(void);
/* With CONFIG_TAB5_PLAYER_LOG_TO_SD: copy every log line to <dir>/tab5_player.log (written every 2 s, fsync'ed). */
void log_sink_sd_start(const char *dir);

#ifdef __cplusplus
}
#endif
