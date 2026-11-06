/*
 * STUB of the CIS engine - ONLY so that the simulator compiles while cis_engine.c does not exist in the parent
 * directory. It handles nothing: the host gets "Unknown HCI Command" for ISO commands and the scenarios
 * end with FAIL and a description. It is NOT an implementation of the engine nor a template for it.
 */
#include "cis_engine.h"

struct cis_engine {
	const cis_platform_ops_t *ops;
};

const char *sim_engine_name(void);
const char *sim_engine_name(void)
{
	return "STUB (cis_ll/cis_engine.c absent - results not meaningful)";
}

uint32_t cis_engine_mem_size(void)
{
	return sizeof(struct cis_engine);
}

cis_engine_t *cis_engine_init(void *mem, uint32_t mem_size, const cis_platform_ops_t *ops)
{
	if (!mem || mem_size < sizeof(struct cis_engine))
		return 0;
	cis_engine_t *e = mem;
	e->ops = ops;
	return e;
}

bool cis_engine_hci_cmd(cis_engine_t *e, const uint8_t *h4, uint16_t len)
{
	(void)e;
	(void)h4;
	(void)len;
	return false;
}

void cis_engine_hci_iso(cis_engine_t *e, const uint8_t *h4, uint16_t len)
{
	(void)e;
	(void)h4;
	(void)len;
}

bool cis_engine_hci_evt_from_controller(cis_engine_t *e, const uint8_t *h4, uint16_t len)
{
	(void)e;
	(void)h4;
	(void)len;
	return false;
}

bool cis_engine_ll_ctrl_rx(cis_engine_t *e, uint16_t acl_handle, const uint8_t *ctr, uint8_t len)
{
	(void)e;
	(void)acl_handle;
	(void)ctr;
	(void)len;
	return false;
}

void cis_engine_ll_ctrl_tx_seen(cis_engine_t *e, uint16_t acl_handle, const uint8_t *ctr, uint8_t len)
{
	(void)e;
	(void)acl_handle;
	(void)ctr;
	(void)len;
}

void cis_engine_on_timer(cis_engine_t *e)
{
	(void)e;
}

void cis_engine_process(cis_engine_t *e)
{
	(void)e;
}

bool cis_engine_active(const cis_engine_t *e)
{
	(void)e;
	return false;
}

bool cis_engine_get_stats(const cis_engine_t *e, uint16_t cis_handle, cis_engine_stats_t *out)
{
	(void)e;
	(void)cis_handle;
	(void)out;
	return false;
}
