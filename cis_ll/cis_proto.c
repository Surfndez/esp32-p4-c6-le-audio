/*
 * Cyberdeck LE Audio: pure CIS protocol - see cis_proto.h.
 *
 * CSA#2 (perm/MAM/prn, ISO subevent mapping) ported from Zephyr
 * subsys/bluetooth/controller/ll_sw/lll_chan.c (Copyright (c) 2018-2021 Nordic Semiconductor ASA,
 * Apache-2.0) - logic 1:1, because that is the only way to hit the same channels as the earbuds; the tests check
 * the Core Vol 6 Part C 3 sample data. The rest was written from scratch per Core 5.4.
 */
#include "cis_proto.h"

#include <string.h>

/* ---------------- LLCP ---------------- */

uint8_t cis_ll_cis_req_pack(const cis_ll_cis_req_t *r, uint8_t out[CIS_LL_CIS_REQ_LEN])
{
	uint8_t *p = out;
	*p++ = CIS_LL_CIS_REQ;
	*p++ = r->cig_id;
	*p++ = r->cis_id;
	*p++ = r->phy_c_to_p;
	*p++ = r->phy_p_to_c;
	/* Max_SDU_C_To_P: 12 bits, then 3 RFU and Framed in the most significant bit */
	*p++ = (uint8_t)r->max_sdu_c_to_p;
	*p++ = (uint8_t)(((r->max_sdu_c_to_p >> 8) & 0x0F) | (r->framed ? 0x80 : 0));
	*p++ = (uint8_t)r->max_sdu_p_to_c;
	*p++ = (uint8_t)((r->max_sdu_p_to_c >> 8) & 0x0F);
	cis_put_le24(p, r->sdu_interval_c_to_p & 0x0FFFFF);
	p += 3;
	cis_put_le24(p, r->sdu_interval_p_to_c & 0x0FFFFF);
	p += 3;
	cis_put_le16(p, r->max_pdu_c_to_p);
	p += 2;
	cis_put_le16(p, r->max_pdu_p_to_c);
	p += 2;
	*p++ = r->nse;
	cis_put_le24(p, r->sub_interval_us);
	p += 3;
	*p++ = (uint8_t)((r->bn_c_to_p & 0x0F) | ((r->bn_p_to_c & 0x0F) << 4));
	*p++ = r->ft_c_to_p;
	*p++ = r->ft_p_to_c;
	cis_put_le16(p, r->iso_interval);
	p += 2;
	cis_put_le24(p, r->cis_offset_min_us);
	p += 3;
	cis_put_le24(p, r->cis_offset_max_us);
	p += 3;
	cis_put_le16(p, r->conn_event_count);
	p += 2;
	return (uint8_t)(p - out);
}

bool cis_ll_cis_req_parse(const uint8_t *c, uint8_t len, cis_ll_cis_req_t *r)
{
	if (len != CIS_LL_CIS_REQ_LEN || c[0] != CIS_LL_CIS_REQ)
		return false;
	r->cig_id = c[1];
	r->cis_id = c[2];
	r->phy_c_to_p = c[3];
	r->phy_p_to_c = c[4];
	r->max_sdu_c_to_p = (uint16_t)(cis_get_le16(&c[5]) & 0x0FFF);
	r->framed = (c[6] & 0x80) != 0;
	r->max_sdu_p_to_c = (uint16_t)(cis_get_le16(&c[7]) & 0x0FFF);
	r->sdu_interval_c_to_p = cis_get_le24(&c[9]) & 0x0FFFFF;
	r->sdu_interval_p_to_c = cis_get_le24(&c[12]) & 0x0FFFFF;
	r->max_pdu_c_to_p = cis_get_le16(&c[15]);
	r->max_pdu_p_to_c = cis_get_le16(&c[17]);
	r->nse = c[19];
	r->sub_interval_us = cis_get_le24(&c[20]);
	r->bn_c_to_p = c[23] & 0x0F;
	r->bn_p_to_c = (uint8_t)(c[23] >> 4);
	r->ft_c_to_p = c[24];
	r->ft_p_to_c = c[25];
	r->iso_interval = cis_get_le16(&c[26]);
	r->cis_offset_min_us = cis_get_le24(&c[28]);
	r->cis_offset_max_us = cis_get_le24(&c[31]);
	r->conn_event_count = cis_get_le16(&c[34]);
	return true;
}

uint8_t cis_ll_cis_rsp_pack(const cis_ll_cis_rsp_t *r, uint8_t out[CIS_LL_CIS_RSP_LEN])
{
	out[0] = CIS_LL_CIS_RSP;
	cis_put_le24(&out[1], r->cis_offset_min_us);
	cis_put_le24(&out[4], r->cis_offset_max_us);
	cis_put_le16(&out[7], r->conn_event_count);
	return CIS_LL_CIS_RSP_LEN;
}

bool cis_ll_cis_rsp_parse(const uint8_t *c, uint8_t len, cis_ll_cis_rsp_t *r)
{
	if (len != CIS_LL_CIS_RSP_LEN || c[0] != CIS_LL_CIS_RSP)
		return false;
	r->cis_offset_min_us = cis_get_le24(&c[1]);
	r->cis_offset_max_us = cis_get_le24(&c[4]);
	r->conn_event_count = cis_get_le16(&c[7]);
	return true;
}

uint8_t cis_ll_cis_ind_pack(const cis_ll_cis_ind_t *r, uint8_t out[CIS_LL_CIS_IND_LEN])
{
	out[0] = CIS_LL_CIS_IND;
	cis_put_le32(&out[1], r->access_address);
	cis_put_le24(&out[5], r->cis_offset_us);
	cis_put_le24(&out[8], r->cig_sync_delay_us);
	cis_put_le24(&out[11], r->cis_sync_delay_us);
	cis_put_le16(&out[14], r->conn_event_count);
	return CIS_LL_CIS_IND_LEN;
}

bool cis_ll_cis_ind_parse(const uint8_t *c, uint8_t len, cis_ll_cis_ind_t *r)
{
	if (len != CIS_LL_CIS_IND_LEN || c[0] != CIS_LL_CIS_IND)
		return false;
	r->access_address = cis_get_le32(&c[1]);
	r->cis_offset_us = cis_get_le24(&c[5]);
	r->cig_sync_delay_us = cis_get_le24(&c[8]);
	r->cis_sync_delay_us = cis_get_le24(&c[11]);
	r->conn_event_count = cis_get_le16(&c[14]);
	return true;
}

uint8_t cis_ll_cis_term_pack(const cis_ll_cis_term_t *r, uint8_t out[CIS_LL_CIS_TERM_LEN])
{
	out[0] = CIS_LL_CIS_TERMINATE_IND;
	out[1] = r->cig_id;
	out[2] = r->cis_id;
	out[3] = r->error_code;
	return CIS_LL_CIS_TERM_LEN;
}

bool cis_ll_cis_term_parse(const uint8_t *c, uint8_t len, cis_ll_cis_term_t *r)
{
	if (len != CIS_LL_CIS_TERM_LEN || c[0] != CIS_LL_CIS_TERMINATE_IND)
		return false;
	r->cig_id = c[1];
	r->cis_id = c[2];
	r->error_code = c[3];
	return true;
}

uint8_t cis_ll_reject_ext_pack(uint8_t reject_opcode, uint8_t error_code, uint8_t out[CIS_LL_REJECT_EXT_LEN])
{
	out[0] = CIS_LL_REJECT_EXT_IND;
	out[1] = reject_opcode;
	out[2] = error_code;
	return CIS_LL_REJECT_EXT_LEN;
}

bool cis_ll_reject_ext_parse(const uint8_t *c, uint8_t len, uint8_t *reject_opcode, uint8_t *error_code)
{
	if (len != CIS_LL_REJECT_EXT_LEN || c[0] != CIS_LL_REJECT_EXT_IND)
		return false;
	*reject_opcode = c[1];
	*error_code = c[2];
	return true;
}

bool cis_ll_unknown_rsp_parse(const uint8_t *c, uint8_t len, uint8_t *unknown_type)
{
	if (len != CIS_LL_UNKNOWN_RSP_LEN || c[0] != CIS_LL_UNKNOWN_RSP)
		return false;
	*unknown_type = c[1];
	return true;
}

bool cis_ll_enc_req_parse(const uint8_t *c, uint8_t len, uint8_t skdm[8], uint8_t ivm[4])
{
	/* opcode, Rand(8), EDIV(2), SKDm(8), IVm(4) */
	if (len != CIS_LL_ENC_REQ_LEN || c[0] != CIS_LL_ENC_REQ)
		return false;
	memcpy(skdm, &c[11], 8);
	memcpy(ivm, &c[19], 4);
	return true;
}

bool cis_ll_enc_rsp_parse(const uint8_t *c, uint8_t len, uint8_t skds[8], uint8_t ivs[4])
{
	if (len != CIS_LL_ENC_RSP_LEN || c[0] != CIS_LL_ENC_RSP)
		return false;
	memcpy(skds, &c[1], 8);
	memcpy(ivs, &c[9], 4);
	return true;
}

/* ---------------- Set CIG Parameters / CIG derivation ---------------- */

#define CIG_FIXED_LEN 15
#define CIG_PER_CIS_LEN 9
#define T_IFS_US 150u
#define T_MSS_US 150u
#define SUB_INTERVAL_MIN_US 400u
#define ISO_INT_UNIT_US 1250u

uint8_t cis_hci_cig_params_parse(const uint8_t *p, uint16_t len, cis_hci_cig_params_t *out)
{
	memset(out, 0, sizeof(*out));
	if (len < CIG_FIXED_LEN)
		return CIS_ERR_INVALID_PARAMS;
	uint8_t count = p[14];
	if (len != CIG_FIXED_LEN + CIG_PER_CIS_LEN * (uint16_t)count || count == 0)
		return CIS_ERR_INVALID_PARAMS;
	/* An engine memory limit, not a host error: Core then requires Memory Capacity Exceeded. */
	if (count > CIS_MAX_CIS)
		return CIS_ERR_MEM_CAPACITY;
	out->cig_id = p[0];
	out->sdu_interval_c_to_p = cis_get_le24(&p[1]);
	out->sdu_interval_p_to_c = cis_get_le24(&p[4]);
	out->sca = p[7];
	out->packing = p[8];
	out->framing = p[9];
	out->latency_c_to_p_ms = cis_get_le16(&p[10]);
	out->latency_p_to_c_ms = cis_get_le16(&p[12]);
	out->cis_count = count;
	for (uint8_t i = 0; i < count; i++) {
		const uint8_t *q = &p[CIG_FIXED_LEN + CIG_PER_CIS_LEN * i];
		cis_hci_cis_cfg_t *c = &out->cis[i];
		c->cis_id = q[0];
		c->max_sdu_c_to_p = cis_get_le16(&q[1]);
		c->max_sdu_p_to_c = cis_get_le16(&q[3]);
		c->phy_c_to_p = q[5];
		c->phy_p_to_c = q[6];
		c->rtn_c_to_p = q[7];
		c->rtn_p_to_c = q[8];
	}
	return CIS_ERR_SUCCESS;
}

uint32_t cis_pdu_air_us(uint16_t payload_with_mic, uint8_t phy)
{
	/* preamble (1 B at 1M, 2 B at 2M) + AA 4 + header 2 + payload + CRC 3 */
	if (phy == CIS_PHY_2M)
		return (2u + 4u + 2u + payload_with_mic + 3u) * 4u;
	return (1u + 4u + 2u + payload_with_mic + 3u) * 8u;
}

/* PHY mask from HCI -> a single PHY. 2M first: a shorter air time means fewer collisions with Wi-Fi on the C6. */
static uint8_t pick_phy(uint8_t mask)
{
	if (mask & CIS_PHY_2M)
		return CIS_PHY_2M;
	if (mask & CIS_PHY_1M)
		return CIS_PHY_1M;
	return 0;
}

static uint8_t clamp_u8(uint32_t v, uint8_t lo, uint8_t hi)
{
	if (v < lo)
		return lo;
	if (v > hi)
		return hi;
	return (uint8_t)v;
}

/* Largest FT at which the transport latency (unframed: CIG_Sync_Delay + FT x ISO - SDU) fits within the host limit.
 * When even FT = 1 does not fit, 1 stays: the host limit is a target, and refusing the CIG at ISO = SDU (where TL =
 * CIG_Sync_Delay, always below 5 ms) could only happen with exotic parameters. */
static uint8_t pick_ft(uint32_t sync_us, uint32_t iso_us, uint32_t sdu_us, uint16_t latency_ms)
{
	uint8_t ft = 1;
	for (uint8_t f = 2; f <= CIS_FT_MAX; f++) {
		uint64_t tl = (uint64_t)sync_us + (uint64_t)f * iso_us - sdu_us;
		if (tl > (uint64_t)latency_ms * 1000u)
			break;
		ft = f;
	}
	return ft;
}

uint8_t cis_cig_derive(const cis_hci_cig_params_t *in, uint8_t max_nse, uint32_t gap_us, uint32_t sub_gap_us,
		cis_cig_derived_t *out)
{
	memset(out, 0, sizeof(*out));
	if (in->cig_id > 0xEF || in->cis_count == 0 || in->sca > 7 || in->packing > 1 || in->framing > 1)
		return CIS_ERR_INVALID_PARAMS;
	if (in->sdu_interval_c_to_p < 0xFF || in->sdu_interval_c_to_p > 0xFFFFF ||
	    in->sdu_interval_p_to_c < 0xFF || in->sdu_interval_p_to_c > 0xFFFFF)
		return CIS_ERR_INVALID_PARAMS;
	if (in->latency_c_to_p_ms < 5 || in->latency_c_to_p_ms > 4000 ||
	    in->latency_p_to_c_ms < 5 || in->latency_p_to_c_ms > 4000)
		return CIS_ERR_INVALID_PARAMS;
	if (in->cis_count > CIS_MAX_CIS)
		return CIS_ERR_MEM_CAPACITY;
	/* Framed mode is out of scope for version 1 (spec: "Out of scope"). */
	if (in->framing)
		return CIS_ERR_UNSUPP_FEATURE;
	if (max_nse == 0)
		max_nse = 1;
	if (max_nse > CIS_NSE_MAX)
		max_nse = CIS_NSE_MAX;

	bool any_c = false, any_p = false;
	for (uint8_t i = 0; i < in->cis_count; i++) {
		const cis_hci_cis_cfg_t *c = &in->cis[i];
		if (c->cis_id > 0xEF || c->max_sdu_c_to_p > 0x0FFF || c->max_sdu_p_to_c > 0x0FFF)
			return CIS_ERR_INVALID_PARAMS;
		for (uint8_t j = 0; j < i; j++)
			if (in->cis[j].cis_id == c->cis_id)
				return CIS_ERR_INVALID_PARAMS;
		if ((c->phy_c_to_p & ~0x07u) || (c->phy_p_to_c & ~0x07u) ||
		    !pick_phy(c->phy_c_to_p) || !pick_phy(c->phy_p_to_c))
			return CIS_ERR_UNSUPP_FEATURE;
		if (c->max_sdu_c_to_p > CIS_SDU_MAX || c->max_sdu_p_to_c > CIS_SDU_MAX)
			return CIS_ERR_UNSUPP_FEATURE;
		any_c = any_c || c->max_sdu_c_to_p;
		any_p = any_p || c->max_sdu_p_to_c;
	}

	/* Unframed: ISO interval = SDU interval (a multiple of 1.25 ms). Different intervals per direction would require
	 * ISO = the LCM of both and BN > 1 per SDU - we do not need that for 10 ms LC3. */
	uint32_t iso_us = any_c || !any_p ? in->sdu_interval_c_to_p : in->sdu_interval_p_to_c;
	if (any_c && any_p && in->sdu_interval_c_to_p != in->sdu_interval_p_to_c)
		return CIS_ERR_UNSUPP_FEATURE;
	if (iso_us % ISO_INT_UNIT_US || iso_us < 4 * ISO_INT_UNIT_US || iso_us > 3200 * ISO_INT_UNIT_US)
		return CIS_ERR_UNSUPP_FEATURE;
	out->iso_interval = (uint16_t)(iso_us / ISO_INT_UNIT_US);
	out->iso_interval_us = iso_us;

	uint32_t total[CIS_MAX_CIS];
	uint8_t min_nse[CIS_MAX_CIS];
	/* radio gaps between consecutive CISes belong to the CIG event (CIG_Sync_Delay) */
	const uint32_t gaps = gap_us * (uint32_t)(in->cis_count - 1u);
	uint32_t sync1 = gaps;
	for (uint8_t i = 0; i < in->cis_count; i++) {
		const cis_hci_cis_cfg_t *c = &in->cis[i];
		cis_cis_derived_t *d = &out->cis[i];
		d->phy_c_to_p = pick_phy(c->phy_c_to_p);
		d->phy_p_to_c = pick_phy(c->phy_p_to_c);
		d->max_sdu_c_to_p = c->max_sdu_c_to_p;
		d->max_sdu_p_to_c = c->max_sdu_p_to_c;
		d->max_pdu_c_to_p = (uint8_t)(c->max_sdu_c_to_p < CIS_MAX_PDU ? c->max_sdu_c_to_p : CIS_MAX_PDU);
		d->max_pdu_p_to_c = (uint8_t)(c->max_sdu_p_to_c < CIS_MAX_PDU ? c->max_sdu_p_to_c : CIS_MAX_PDU);
		/* Core Vol 6 Part G 2.1: BN >= ceil(Max_SDU / Max_PDU x ISO_Interval / SDU_Interval), here ISO = SDU */
		d->bn_c_to_p = (uint8_t)(d->max_pdu_c_to_p ? (c->max_sdu_c_to_p + d->max_pdu_c_to_p - 1) / d->max_pdu_c_to_p : 0);
		d->bn_p_to_c = (uint8_t)(d->max_pdu_p_to_c ? (c->max_sdu_p_to_c + d->max_pdu_p_to_c - 1) / d->max_pdu_p_to_c : 0);
		/* MIC is always counted when a direction carries data: at Set CIG it is not known whether the ACL will be
		 * encrypted. */
		uint32_t mpt_c = cis_pdu_air_us((uint16_t)(d->max_pdu_c_to_p + (d->bn_c_to_p ? CIS_MIC_LEN : 0)), d->phy_c_to_p);
		uint32_t mpt_p = cis_pdu_air_us((uint16_t)(d->max_pdu_p_to_c + (d->bn_p_to_c ? CIS_MIC_LEN : 0)), d->phy_p_to_c);
		uint32_t se = mpt_c + T_IFS_US + mpt_p + T_MSS_US;
		d->se_length_us = se > SUB_INTERVAL_MIN_US ? se : SUB_INTERVAL_MIN_US;
		uint32_t tc = (uint32_t)(c->rtn_c_to_p + 1u) * d->bn_c_to_p;
		uint32_t tp = (uint32_t)(c->rtn_p_to_c + 1u) * d->bn_p_to_c;
		total[i] = tc > tp ? tc : tp;
		if (total[i] == 0)
			total[i] = 1;
		/* NSE >= BN: every payload of an event needs at least one subevent */
		uint8_t bn_max = d->bn_c_to_p > d->bn_p_to_c ? d->bn_c_to_p : d->bn_p_to_c;
		min_nse[i] = bn_max ? bn_max : 1;
		if (min_nse[i] > max_nse)
			return CIS_ERR_UNSUPP_FEATURE;
		const uint8_t n1 = clamp_u8(total[i], min_nse[i], max_nse);
		sync1 += cis_slot_us(n1, d->se_length_us, sub_gap_us);
	}

	/* FT from the latency at the NSE for FT = 1 (the longest CIG); then NSE shrinks as FT grows, so TL stays
	 * within the limit - the same order as in Zephyr ll_cig_parameters_commit (reliability policy). */
	uint32_t sync = gaps;
	for (uint8_t i = 0; i < in->cis_count; i++) {
		cis_cis_derived_t *d = &out->cis[i];
		d->ft_c_to_p = pick_ft(sync1, iso_us, in->sdu_interval_c_to_p, in->latency_c_to_p_ms);
		d->ft_p_to_c = pick_ft(sync1, iso_us, in->sdu_interval_p_to_c, in->latency_p_to_c_ms);
		uint8_t ft_min;
		if (d->bn_c_to_p && d->bn_p_to_c)
			ft_min = d->ft_c_to_p < d->ft_p_to_c ? d->ft_c_to_p : d->ft_p_to_c;
		else
			ft_min = d->bn_p_to_c ? d->ft_p_to_c : d->ft_c_to_p;
		d->nse = clamp_u8((total[i] + ft_min - 1) / ft_min, min_nse[i], max_nse);
		if (d->nse <= min_nse[i] && (d->ft_c_to_p > 1 || d->ft_p_to_c > 1)) {
			/* NSE = BN: each event fits exactly BN payloads, so after one retransmission the queue stays delayed
			 * permanently and FT > 1 only adds latency. We look for the largest FT with a free subevent; when there
			 * is none - FT 1. This applies to every direction with FT > 1, not only to min FT: with different
			 * latencies per direction (e.g. music 40 ms, microphone 10 ms) ft_min = 1, and C->P would have FT 4 with
			 * no subevent for a retransmission. With ft_min = 1 the loop does not start, so FT 1 in both. */
			uint8_t ft_ok = 1;
			for (uint8_t f = (uint8_t)(ft_min - 1); f >= 2; f--) {
				if (clamp_u8((total[i] + f - 1) / f, min_nse[i], max_nse) > min_nse[i]) {
					ft_ok = f;
					break;
				}
			}
			if (d->ft_c_to_p > ft_ok)
				d->ft_c_to_p = ft_ok;
			if (d->ft_p_to_c > ft_ok)
				d->ft_p_to_c = ft_ok;
			d->nse = clamp_u8((total[i] + ft_ok - 1) / ft_ok, min_nse[i], max_nse);
		}
		/* At NSE > 1 subevents are spaced by Sub_Interval = se_length + radio gap (C6: a separate schedule event for
		 * each); the last one has the gap too, because the radio reserves the whole event - including the only one
		 * at NSE 1. */
		d->sub_interval_us = d->nse > 1 ? d->se_length_us + sub_gap_us : 0;
		sync += cis_slot_us(d->nse, d->se_length_us, sub_gap_us);
	}
	/* The sequential packing of all CISes must fit in one ISO interval. */
	if (sync > iso_us)
		return CIS_ERR_UNSUPP_FEATURE;
	out->cig_sync_delay_us = sync;
	uint32_t before = 0;
	for (uint8_t i = 0; i < in->cis_count; i++) {
		cis_cis_derived_t *d = &out->cis[i];
		d->cis_sync_delay_us = sync - before;
		before += cis_slot_us(d->nse, d->se_length_us, sub_gap_us) + gap_us;
		d->tl_c_to_p_us = sync + (uint32_t)d->ft_c_to_p * iso_us - in->sdu_interval_c_to_p;
		d->tl_p_to_c_us = sync + (uint32_t)d->ft_p_to_c * iso_us - in->sdu_interval_p_to_c;
		/* The jitter buffer takes only what the transport does not need; never less than 2 SDUs. */
		const uint32_t mtl_us = (uint32_t)in->latency_c_to_p_ms * 1000u;
		const uint32_t spare = mtl_us > d->tl_c_to_p_us ? (mtl_us - d->tl_c_to_p_us) / in->sdu_interval_c_to_p : 0;
		d->jitter_c_to_p = clamp_u8(spare, CIS_JITTER_MIN, CIS_JITTER_MAX);
	}
	return CIS_ERR_SUCCESS;
}

/* ---------------- access address ---------------- */

bool cis_aa_valid(uint32_t aa)
{
	uint32_t d = aa ^ CIS_ADV_ACCESS_ADDRESS;
	/* not the advertising address and does not differ from it by a single bit */
	if (d == 0 || (d & (d - 1)) == 0)
		return false;
	uint8_t o = (uint8_t)aa;
	if ((uint8_t)(aa >> 8) == o && (uint8_t)(aa >> 16) == o && (uint8_t)(aa >> 24) == o)
		return false;
	unsigned run = 1, trans = 0, trans_ms6 = 0, trans_ls16 = 0;
	for (int i = 30; i >= 0; i--) {
		uint32_t b = (aa >> i) & 1u, prev = (aa >> (i + 1)) & 1u;
		if (b == prev) {
			if (++run > 6)
				return false;
		} else {
			run = 1;
			trans++;
			if (i >= 26)
				trans_ms6++;
			if (i <= 14)
				trans_ls16++;
		}
	}
	if (trans > 24 || trans_ms6 < 2)
		return false;
	/* Rules for LE Coded: cheap, and a CIS should not depend on which PHY the host may choose later. */
	if (trans_ls16 > 11 || __builtin_popcount(aa & 0xFFu) < 3)
		return false;
	return true;
}

uint32_t cis_aa_generate(void (*rnd)(void *ctx, uint8_t *buf, uint16_t len), void *ctx,
		const uint32_t *avoid, uint8_t n_avoid)
{
	/* About 45 % of random words satisfy the rules; 256 tries give a failure probability of ~1e-66. */
	for (int tries = 0; tries < 256; tries++) {
		uint8_t b[4];
		rnd(ctx, b, 4);
		uint32_t aa = cis_get_le32(b);
		if (!cis_aa_valid(aa))
			continue;
		bool clash = false;
		for (uint8_t i = 0; i < n_avoid; i++)
			clash = clash || avoid[i] == aa;
		if (!clash)
			return aa;
	}
	return 0;
}

/* ---------------- CSA#2 (z Zephyr lll_chan.c) ---------------- */

uint16_t cis_csa2_chan_id(uint32_t aa)
{
	return (uint16_t)((aa >> 16) ^ (aa & 0xFFFFu));
}

uint8_t cis_csa2_num_used(const uint8_t chan_map[5])
{
	uint8_t n = 0;
	for (uint8_t ch = 0; ch < 37; ch++)
		n = (uint8_t)(n + ((chan_map[ch >> 3] >> (ch & 7)) & 1));
	return n;
}

static bool chan_used(const uint8_t chan_map[5], uint8_t ch)
{
	return (chan_map[ch >> 3] >> (ch & 7)) & 1;
}

/* Channel at index idx in the used-channels list (ascending). */
static uint8_t chan_remap(const uint8_t chan_map[5], uint16_t idx)
{
	for (uint8_t ch = 0; ch < 37; ch++) {
		if (!chan_used(chan_map, ch))
			continue;
		if (idx == 0)
			return ch;
		idx--;
	}
	return 0;
}

/* Index of channel ch (a used one) in the used-channels list. */
static uint16_t chan_index_of(const uint8_t chan_map[5], uint8_t ch)
{
	uint16_t idx = 0;
	for (uint8_t c = 0; c < ch; c++)
		idx = (uint16_t)(idx + chan_used(chan_map, c));
	return idx;
}

static uint8_t rev8(uint8_t b)
{
	b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
	b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
	b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
	return b;
}

static uint16_t perm(uint16_t i)
{
	return (uint16_t)((rev8((uint8_t)(i >> 8)) << 8) | rev8((uint8_t)i));
}

static uint16_t mam(uint16_t a, uint16_t b)
{
	return (uint16_t)(((uint32_t)a * 17u + b) & 0xFFFFu);
}

static uint16_t prn_s(uint16_t counter, uint16_t chan_id)
{
	uint16_t x = counter ^ chan_id;
	for (int i = 0; i < 3; i++)
		x = mam(perm(x), chan_id);
	return x;
}

uint8_t cis_csa2_chan(uint16_t counter, uint16_t chan_id, const uint8_t chan_map[5])
{
	uint16_t prn_e = prn_s(counter, chan_id) ^ chan_id;
	uint8_t ch = (uint8_t)(prn_e % 37);
	if (chan_used(chan_map, ch))
		return ch;
	uint8_t n = cis_csa2_num_used(chan_map);
	return chan_remap(chan_map, (uint16_t)(((uint32_t)n * prn_e) >> 16));
}

uint8_t cis_csa2_iso_event(uint16_t counter, uint16_t chan_id, const uint8_t chan_map[5], cis_csa2_iso_t *st)
{
	st->prn_lu = prn_s(counter, chan_id);
	uint16_t prn_e = st->prn_lu ^ chan_id;
	uint8_t ch = (uint8_t)(prn_e % 37);
	if (chan_used(chan_map, ch)) {
		st->remap_idx = chan_index_of(chan_map, ch);
		return ch;
	}
	uint8_t n = cis_csa2_num_used(chan_map);
	st->remap_idx = (uint16_t)(((uint32_t)n * prn_e) >> 16);
	return chan_remap(chan_map, st->remap_idx);
}

uint8_t cis_csa2_iso_subevent(uint16_t chan_id, const uint8_t chan_map[5], cis_csa2_iso_t *st)
{
	uint8_t n = cis_csa2_num_used(chan_map);
	if (n == 0)
		return 0;
	st->prn_lu = mam(perm(st->prn_lu), chan_id);
	uint16_t se = st->prn_lu ^ chan_id;
	/* d = max(1, max(min(3, N - 5), min(11, (N - 10) / 2))) on natural numbers */
	uint8_t x = n > 5 ? (uint8_t)(n - 5) : 0;
	uint8_t y = n > 10 ? (uint8_t)((n - 10) >> 1) : 0;
	uint8_t d = 1;
	if ((x < 3 ? x : 3) > d)
		d = x < 3 ? x : 3;
	if ((y < 11 ? y : 11) > d)
		d = y < 11 ? y : 11;
	uint32_t span = (n + 1u) > (2u * d) ? (n + 1u) - 2u * d : 0;
	st->remap_idx = (uint16_t)(((((uint32_t)se * span) >> 16) + d + st->remap_idx) % n);
	return chan_remap(chan_map, st->remap_idx);
}

/* ---------------- PDU header, flushing ---------------- */

uint8_t cis_pdu_hdr_pack(const cis_pdu_hdr_t *h)
{
	return (uint8_t)((h->llid & 0x03) | (h->nesn ? CIS_HDR_NESN : 0) | (h->sn ? CIS_HDR_SN : 0) |
			(h->cie ? CIS_HDR_CIE : 0) | (h->npi ? CIS_HDR_NPI : 0));
}

void cis_pdu_hdr_unpack(uint8_t hdr0, cis_pdu_hdr_t *h)
{
	h->llid = hdr0 & 0x03;
	h->nesn = (hdr0 & CIS_HDR_NESN) != 0;
	h->sn = (hdr0 & CIS_HDR_SN) != 0;
	h->cie = (hdr0 & CIS_HDR_CIE) != 0;
	h->npi = (hdr0 & CIS_HDR_NPI) != 0;
}

void cis_flush_point(uint64_t pn, uint8_t bn, uint8_t ft, uint8_t nse, uint64_t *last_event, uint8_t *last_subevent)
{
	if (bn == 0)
		bn = 1;
	if (ft == 0)
		ft = 1;
	if (nse == 0)
		nse = 1;
	*last_event = pn / bn + ft - 1;
	/* u = NSE - floor(NSE/BN) x (BN - 1 - (pn mod BN)), numbered from 1 */
	uint32_t u = nse - (uint32_t)(nse / bn) * (uint32_t)(bn - 1 - (uint8_t)(pn % bn));
	*last_subevent = (uint8_t)(u ? u - 1 : 0);
}

/* ---------------- unframed SDU <-> PDU ---------------- */

bool cis_unframed_fragment(uint16_t sdu_len, uint8_t max_pdu, uint8_t bn, uint8_t idx,
		uint16_t *offset, uint8_t *len, uint8_t *llid)
{
	if (idx >= bn || sdu_len > (uint32_t)bn * max_pdu)
		return false;
	uint16_t frags = (uint16_t)((sdu_len == 0 || max_pdu == 0) ? 1 : (sdu_len + max_pdu - 1) / max_pdu);
	if (idx < frags) {
		*offset = (uint16_t)(idx * max_pdu);
		uint16_t left = (uint16_t)(sdu_len - *offset);
		*len = (uint8_t)(left < max_pdu ? left : max_pdu);
		*llid = (idx == frags - 1) ? CIS_LLID_COMPLETE_END : CIS_LLID_START_CONT;
	} else {
		/* padding: a null PDU with LLID 0b01 (Zephyr isoal.c: padding) */
		*offset = sdu_len;
		*len = 0;
		*llid = CIS_LLID_START_CONT;
	}
	return true;
}

void cis_unframed_rx_begin(cis_unframed_rx_t *s, uint8_t *buf, uint16_t cap)
{
	memset(s, 0, sizeof(*s));
	s->buf = buf;
	s->cap = cap;
}

void cis_unframed_rx_pdu(cis_unframed_rx_t *s, uint8_t llid, const uint8_t *data, uint8_t len, bool lost)
{
	if (lost) {
		/* a lost payload past the end of the SDU is only padding - it does not corrupt the SDU */
		if (!s->ended)
			s->error = true;
		return;
	}
	if (s->ended) {
		if (len != 0 || llid != CIS_LLID_START_CONT)
			s->error = true;
		return;
	}
	if (llid != CIS_LLID_START_CONT && llid != CIS_LLID_COMPLETE_END) {
		s->error = true;
		return;
	}
	s->started = true;
	s->any_data = true;
	if (len) {
		if ((uint32_t)s->len + len > s->cap) {
			s->error = true;
		} else {
			memcpy(s->buf + s->len, data, len);
			s->len = (uint16_t)(s->len + len);
		}
	}
	if (llid == CIS_LLID_COMPLETE_END)
		s->ended = true;
}

uint8_t cis_unframed_rx_end(cis_unframed_rx_t *s)
{
	if (!s->started) {
		s->len = 0;
		return CIS_ISO_STATUS_LOST;
	}
	if (s->error || !s->ended)
		return CIS_ISO_STATUS_INVALID;
	return CIS_ISO_STATUS_VALID;
}

/* ---------------- HCI ISO ---------------- */

bool cis_hci_iso_parse(const uint8_t *h4, uint16_t len, cis_hci_iso_t *out)
{
	memset(out, 0, sizeof(*out));
	if (len < 5 || h4[0] != CIS_H4_ISO)
		return false;
	uint16_t hdr = cis_get_le16(&h4[1]);
	uint16_t dl = cis_get_le16(&h4[3]);
	if ((hdr & 0x8000) || (dl & 0xC000))
		return false;
	if ((uint32_t)len != 5u + dl)
		return false;
	out->handle = hdr & 0x0FFF;
	out->pb = (uint8_t)((hdr >> 12) & 0x3);
	out->ts_flag = (hdr >> 14) & 1;
	const uint8_t *p = &h4[5];
	uint16_t rem = dl;
	if (out->ts_flag) {
		if (rem < 4)
			return false;
		out->timestamp = cis_get_le32(p);
		p += 4;
		rem = (uint16_t)(rem - 4);
	}
	if (out->pb == CIS_ISO_PB_FIRST || out->pb == CIS_ISO_PB_COMPLETE) {
		if (rem < 4)
			return false;
		out->psn = cis_get_le16(p);
		uint16_t sl = cis_get_le16(p + 2);
		out->sdu_len = sl & 0x0FFF;
		out->status = (uint8_t)(sl >> 14);
		p += 4;
		rem = (uint16_t)(rem - 4);
	}
	out->data = p;
	out->data_len = rem;
	return true;
}

uint16_t cis_hci_iso_build(const cis_hci_iso_t *in, uint8_t *out, uint16_t cap)
{
	bool has_sdu_hdr = in->pb == CIS_ISO_PB_FIRST || in->pb == CIS_ISO_PB_COMPLETE;
	uint32_t dl = (in->ts_flag ? 4u : 0u) + (has_sdu_hdr ? 4u : 0u) + in->data_len;
	if (dl > 0x3FFF || 5u + dl > cap)
		return 0;
	out[0] = CIS_H4_ISO;
	cis_put_le16(&out[1], (uint16_t)((in->handle & 0x0FFF) | ((in->pb & 3u) << 12) | (in->ts_flag ? 0x4000u : 0)));
	cis_put_le16(&out[3], (uint16_t)dl);
	uint8_t *p = &out[5];
	if (in->ts_flag) {
		cis_put_le32(p, in->timestamp);
		p += 4;
	}
	if (has_sdu_hdr) {
		cis_put_le16(p, in->psn);
		cis_put_le16(p + 2, (uint16_t)((in->sdu_len & 0x0FFF) | ((in->status & 3u) << 14)));
		p += 4;
	}
	if (in->data_len)
		memcpy(p, in->data, in->data_len);
	return (uint16_t)(5u + dl);
}

/* ---------------- HCI events ---------------- */

uint16_t cis_hci_evt_cmd_complete(uint8_t *out, uint16_t opcode, const uint8_t *ret, uint8_t ret_len)
{
	out[0] = CIS_H4_EVT;
	out[1] = CIS_EVT_CMD_COMPLETE;
	out[2] = (uint8_t)(3 + ret_len);
	out[3] = 1;   /* Num_HCI_Command_Packets: the controller accepts the next command */
	cis_put_le16(&out[4], opcode);
	if (ret_len)
		memcpy(&out[6], ret, ret_len);
	return (uint16_t)(6 + ret_len);
}

uint16_t cis_hci_evt_cmd_status(uint8_t *out, uint8_t status, uint16_t opcode)
{
	out[0] = CIS_H4_EVT;
	out[1] = CIS_EVT_CMD_STATUS;
	out[2] = 4;
	out[3] = status;
	out[4] = 1;
	cis_put_le16(&out[5], opcode);
	return 7;
}

uint16_t cis_hci_evt_disconn_complete(uint8_t *out, uint8_t status, uint16_t handle, uint8_t reason)
{
	out[0] = CIS_H4_EVT;
	out[1] = CIS_EVT_DISCONN_COMPLETE;
	out[2] = 4;
	out[3] = status;
	cis_put_le16(&out[4], handle);
	out[6] = reason;
	return 7;
}

uint16_t cis_hci_evt_nocp(uint8_t *out, const uint16_t *handles, const uint16_t *counts, uint8_t n)
{
	/* (handle, counter) pairs interleaved - the way controllers send them and NimBLE/Zephyr/BlueZ parse them. */
	out[0] = CIS_H4_EVT;
	out[1] = CIS_EVT_NUM_COMPLETED_PKTS;
	out[2] = (uint8_t)(1 + 4 * n);
	out[3] = n;
	for (uint8_t i = 0; i < n; i++) {
		cis_put_le16(&out[4 + 4 * i], handles[i]);
		cis_put_le16(&out[6 + 4 * i], counts[i]);
	}
	return (uint16_t)(4 + 4 * n);
}

uint16_t cis_hci_evt_cis_established(uint8_t *out, const cis_hci_cis_established_t *ev)
{
	uint8_t *p = out;
	*p++ = CIS_H4_EVT;
	*p++ = CIS_EVT_LE_META;
	*p++ = 29;
	*p++ = CIS_LE_SUBEVT_CIS_ESTABLISHED;
	*p++ = ev->status;
	cis_put_le16(p, ev->handle);
	p += 2;
	cis_put_le24(p, ev->cig_sync_delay_us);
	p += 3;
	cis_put_le24(p, ev->cis_sync_delay_us);
	p += 3;
	cis_put_le24(p, ev->tl_c_to_p_us);
	p += 3;
	cis_put_le24(p, ev->tl_p_to_c_us);
	p += 3;
	*p++ = ev->phy_c_to_p;
	*p++ = ev->phy_p_to_c;
	*p++ = ev->nse;
	*p++ = ev->bn_c_to_p;
	*p++ = ev->bn_p_to_c;
	*p++ = ev->ft_c_to_p;
	*p++ = ev->ft_p_to_c;
	cis_put_le16(p, ev->max_pdu_c_to_p);
	p += 2;
	cis_put_le16(p, ev->max_pdu_p_to_c);
	p += 2;
	cis_put_le16(p, ev->iso_interval);
	p += 2;
	return (uint16_t)(p - out);
}

bool cis_hci_cmd_header(const uint8_t *h4, uint16_t len, uint16_t *opcode, const uint8_t **params, uint8_t *plen)
{
	if (len < 4 || h4[0] != CIS_H4_CMD || (uint32_t)len != 4u + h4[3])
		return false;
	*opcode = cis_get_le16(&h4[1]);
	*plen = h4[3];
	*params = &h4[4];
	return true;
}
