/* Cyberdeck LE Audio: self-disable of the CIS adapter after crashes (description in cis_c6_crash.h). */
#include <string.h>

#include "cis_c6_crash.h"

cis_crash_boot_t cis_crash_boot(const cis_crash_rec_t *stored, const uint8_t img[CIS_CRASH_IMG_LEN],
				bool live_at_reset, bool crash_reset)
{
	cis_crash_boot_t out = { .write = false, .disabled = false };
	const bool same_img = stored != NULL && stored->ver == CIS_CRASH_REC_VER &&
			      memcmp(stored->img, img, CIS_CRASH_IMG_LEN) == 0;

	out.rec.ver = CIS_CRASH_REC_VER;
	memcpy(out.rec.img, img, CIS_CRASH_IMG_LEN);
	/* A new image = a new chance: the counter restarts from zero (written only when there was something to clear). */
	out.rec.crashes = same_img ? stored->crashes : 0;
	if (!same_img && stored != NULL) {
		out.write = true;
	}
	if (live_at_reset && crash_reset && out.rec.crashes < UINT8_MAX) {
		out.rec.crashes++;
		out.write = true;
	}
	out.disabled = out.rec.crashes >= CIS_CRASH_LIMIT;
	return out;
}
