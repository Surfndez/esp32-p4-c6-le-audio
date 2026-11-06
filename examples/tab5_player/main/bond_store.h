#pragma once
#include "le_audio.h"

/* Loads saved bonds from NVS into RAM and fills kv (read-only unless CONFIG_TAB5_PLAYER_SAVE_BONDS) and identity
 * (address mode the earbuds were paired with). Call once from app_main. true = a saved bond list exists. */
bool bond_store_load(le_audio_kv_t *kv, int *identity);
/* Number of entries in the saved bond list (0 = none). Call after bond_store_load. */
int bond_store_count(void);
