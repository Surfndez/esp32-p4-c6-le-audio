/*
 * cis_proto tests: CSA#2 on the Core Vol 6 Part C 3 sample data (values from the Zephyr lll_chan.c self-test),
 * byte-exact PDU encodings, access address rules on 100 000 generated addresses (independent checker),
 * CIS PDU header, flush point, unframed segmentation, HCI ISO, HCI events, CIG derivation.
 */
#include "../cis_proto.h"
#include "test_util.h"

/* ---------------- CSA#2 ---------------- */
static const uint8_t map37[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x1F };
static const uint8_t map9[5] = { 0x00, 0x06, 0xE0, 0x00, 0x1E };   /* 9, 10, 21, 22, 23, 33, 34, 35, 36 */
static const uint8_t map2[5] = { 0x06, 0x00, 0x00, 0x00, 0x00 };

static void test_csa2(void)
{
	const uint16_t id = 0x305F;
	CHECK_EQ(cis_csa2_chan_id(0x8E89BED6u), 0x305F);
	CHECK_EQ(cis_csa2_num_used(map37), 37);
	CHECK_EQ(cis_csa2_num_used(map9), 9);
	CHECK_EQ(cis_csa2_num_used(map2), 2);

	/* Sample Data 1 and 2 (Core Vol 6 Part C 3.1, 3.2) */
	CHECK_EQ(cis_csa2_chan(0, id, map37), 25);
	CHECK_EQ(cis_csa2_chan(1, id, map37), 20);
	CHECK_EQ(cis_csa2_chan(2, id, map37), 6);
	CHECK_EQ(cis_csa2_chan(3, id, map37), 21);
	CHECK_EQ(cis_csa2_chan(6, id, map9), 23);
	CHECK_EQ(cis_csa2_chan(7, id, map9), 9);
	CHECK_EQ(cis_csa2_chan(8, id, map9), 34);
	CHECK_EQ(cis_csa2_chan(11, id, map2), 1);
	CHECK_EQ(cis_csa2_chan(12, id, map2), 2);

	/* ISO subevents: {counter, map, subevent channels 1..4, remap idx 1..4} */
	static const struct {
		uint16_t counter;
		const uint8_t *map;
		uint8_t ch[4];
		uint8_t idx[4];
	} v[] = {
		{ 0, map37, { 25, 1, 16, 36 }, { 25, 1, 16, 36 } },
		{ 1, map37, { 20, 36, 12, 34 }, { 20, 36, 12, 34 } },
		{ 2, map37, { 6, 18, 32, 21 }, { 6, 18, 32, 21 } },
		{ 3, map37, { 21, 4, 22, 8 }, { 21, 4, 22, 8 } },
		{ 6, map9, { 23, 35, 21, 36 }, { 4, 7, 2, 8 } },
		{ 7, map9, { 9, 22, 36, 33 }, { 0, 3, 8, 5 } },
		{ 8, map9, { 34, 9, 33, 10 }, { 6, 0, 5, 1 } },
		{ 11, map2, { 1, 2, 1, 2 }, { 0, 1, 0, 1 } },
		{ 12, map2, { 2, 1, 2, 1 }, { 1, 0, 1, 0 } },
	};
	for (unsigned i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
		cis_csa2_iso_t st;
		CHECK_EQ(cis_csa2_iso_event(v[i].counter, id, v[i].map, &st), v[i].ch[0]);
		CHECK_EQ(st.remap_idx, v[i].idx[0]);
		for (int s = 1; s < 4; s++) {
			CHECK_EQ(cis_csa2_iso_subevent(id, v[i].map, &st), v[i].ch[s]);
			CHECK_EQ(st.remap_idx, v[i].idx[s]);
		}
	}
	/* prnSubEvent_se for counter 0: 56857 ^ id starts lu; the following se: 11710, 16649, 38198 - verified
	 * indirectly through the channels above; prn_e of event 0 = 56857. */
	{
		cis_csa2_iso_t st;
		cis_csa2_iso_event(0, id, map37, &st);
		CHECK_EQ(st.prn_lu ^ id, 56857);
	}
}

/* ---------------- LLCP PDUs ---------------- */
static void test_llcp_pdus(void)
{
	uint8_t buf[64];
	cis_ll_cis_req_t req = {
		.cig_id = 0x01, .cis_id = 0x02, .phy_c_to_p = CIS_PHY_2M, .phy_p_to_c = CIS_PHY_2M,
		.max_sdu_c_to_p = 120, .max_sdu_p_to_c = 40, .framed = false,
		.sdu_interval_c_to_p = 10000, .sdu_interval_p_to_c = 10000,
		.max_pdu_c_to_p = 120, .max_pdu_p_to_c = 40, .nse = 3, .sub_interval_us = 1060,
		.bn_c_to_p = 2, .bn_p_to_c = 1, .ft_c_to_p = 1, .ft_p_to_c = 2, .iso_interval = 8,
		.cis_offset_min_us = 500, .cis_offset_max_us = 6250, .conn_event_count = 0x1234,
	};
	static const uint8_t want_req[36] = {
		0x1F, 0x01, 0x02, 0x02, 0x02, 0x78, 0x00, 0x28, 0x00, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00,
		0x78, 0x00, 0x28, 0x00, 0x03, 0x24, 0x04, 0x00, 0x12, 0x01, 0x02, 0x08, 0x00,
		0xF4, 0x01, 0x00, 0x6A, 0x18, 0x00, 0x34, 0x12,
	};
	CHECK_EQ(cis_ll_cis_req_pack(&req, buf), 36);
	CHECK_MEM(buf, want_req, 36);
	{
		cis_ll_cis_req_t back;
		memset(&back, 0xEE, sizeof(back));
		CHECK(cis_ll_cis_req_parse(want_req, 36, &back));
		CHECK_EQ(back.cig_id, 1);
		CHECK_EQ(back.bn_c_to_p, 2);
		CHECK_EQ(back.bn_p_to_c, 1);
		CHECK_EQ(back.sub_interval_us, 1060);
		CHECK_EQ(back.cis_offset_max_us, 6250);
		CHECK_EQ(back.conn_event_count, 0x1234);
		CHECK(!back.framed);
		CHECK(!cis_ll_cis_req_parse(want_req, 35, &back));
		CHECK(!cis_ll_cis_req_parse(want_req, 37, &back));
	}
	/* Max_SDU 12 bits + the Framed bit in the most significant bit of the second octet */
	req.max_sdu_c_to_p = 0x0ABC;
	req.framed = true;
	cis_ll_cis_req_pack(&req, buf);
	CHECK_EQ(buf[5], 0xBC);
	CHECK_EQ(buf[6], 0x8A);
	{
		cis_ll_cis_req_t back;
		CHECK(cis_ll_cis_req_parse(buf, 36, &back));
		CHECK_EQ(back.max_sdu_c_to_p, 0x0ABC);
		CHECK(back.framed);
	}

	cis_ll_cis_rsp_t rsp = { .cis_offset_min_us = 500, .cis_offset_max_us = 2000, .conn_event_count = 0x0102 };
	static const uint8_t want_rsp[9] = { 0x20, 0xF4, 0x01, 0x00, 0xD0, 0x07, 0x00, 0x02, 0x01 };
	CHECK_EQ(cis_ll_cis_rsp_pack(&rsp, buf), 9);
	CHECK_MEM(buf, want_rsp, 9);
	{
		cis_ll_cis_rsp_t back;
		CHECK(cis_ll_cis_rsp_parse(want_rsp, 9, &back));
		CHECK_EQ(back.cis_offset_min_us, 500);
		CHECK_EQ(back.cis_offset_max_us, 2000);
		CHECK_EQ(back.conn_event_count, 0x0102);
		CHECK(!cis_ll_cis_rsp_parse(want_rsp, 8, &back));
		CHECK(!cis_ll_cis_rsp_parse(want_req, 9, &back));
	}

	cis_ll_cis_ind_t ind = { .access_address = 0xA1B2C3D4u, .cis_offset_us = 1500, .cig_sync_delay_us = 2120,
		.cis_sync_delay_us = 1060, .conn_event_count = 0x0010 };
	static const uint8_t want_ind[16] = { 0x21, 0xD4, 0xC3, 0xB2, 0xA1, 0xDC, 0x05, 0x00, 0x48, 0x08, 0x00,
		0x24, 0x04, 0x00, 0x10, 0x00 };
	CHECK_EQ(cis_ll_cis_ind_pack(&ind, buf), 16);
	CHECK_MEM(buf, want_ind, 16);
	{
		cis_ll_cis_ind_t back;
		CHECK(cis_ll_cis_ind_parse(want_ind, 16, &back));
		CHECK_EQ(back.access_address, 0xA1B2C3D4u);
		CHECK_EQ(back.cis_offset_us, 1500);
		CHECK_EQ(back.cig_sync_delay_us, 2120);
		CHECK_EQ(back.cis_sync_delay_us, 1060);
		CHECK_EQ(back.conn_event_count, 0x10);
	}

	cis_ll_cis_term_t term = { .cig_id = 1, .cis_id = 2, .error_code = 0x13 };
	static const uint8_t want_term[4] = { 0x22, 0x01, 0x02, 0x13 };
	CHECK_EQ(cis_ll_cis_term_pack(&term, buf), 4);
	CHECK_MEM(buf, want_term, 4);
	{
		cis_ll_cis_term_t back;
		CHECK(cis_ll_cis_term_parse(want_term, 4, &back));
		CHECK_EQ(back.error_code, 0x13);
		CHECK(!cis_ll_cis_term_parse(want_term, 3, &back));
	}

	static const uint8_t want_rej[3] = { 0x11, 0x1F, 0x1A };
	CHECK_EQ(cis_ll_reject_ext_pack(0x1F, 0x1A, buf), 3);
	CHECK_MEM(buf, want_rej, 3);
	{
		uint8_t op = 0, err = 0, t = 0;
		CHECK(cis_ll_reject_ext_parse(want_rej, 3, &op, &err));
		CHECK_EQ(op, 0x1F);
		CHECK_EQ(err, 0x1A);
		static const uint8_t unk[2] = { 0x07, 0x1F };
		CHECK(cis_ll_unknown_rsp_parse(unk, 2, &t));
		CHECK_EQ(t, 0x1F);
		CHECK(!cis_ll_unknown_rsp_parse(unk, 1, &t));
	}

	/* LL_ENC_REQ: opcode, Rand(8), EDIV(2), SKDm(8), IVm(4); LL_ENC_RSP: SKDs(8), IVs(4) */
	{
		uint8_t enc_req[23], enc_rsp[13], skd[8], iv[4];
		enc_req[0] = 0x03;
		for (int i = 1; i < 23; i++)
			enc_req[i] = (uint8_t)i;
		CHECK(cis_ll_enc_req_parse(enc_req, 23, skd, iv));
		CHECK_EQ(skd[0], 11);
		CHECK_EQ(skd[7], 18);
		CHECK_EQ(iv[0], 19);
		CHECK_EQ(iv[3], 22);
		CHECK(!cis_ll_enc_req_parse(enc_req, 22, skd, iv));
		enc_rsp[0] = 0x04;
		for (int i = 1; i < 13; i++)
			enc_rsp[i] = (uint8_t)(0x40 + i);
		CHECK(cis_ll_enc_rsp_parse(enc_rsp, 13, skd, iv));
		CHECK_EQ(skd[0], 0x41);
		CHECK_EQ(iv[3], 0x4C);
	}
}

/* ---------------- access address ---------------- */
/* An independent checker on a bit string (MSB first), a different construction than in cis_proto.c. */
static bool aa_rules_ref(uint32_t aa)
{
	char b[33];
	for (int i = 0; i < 32; i++)
		b[i] = (char)('0' + ((aa >> (31 - i)) & 1));
	b[32] = 0;
	int run = 1, trans = 0, trans_ms6 = 0, trans_ls16 = 0, ones_ls8 = 0;
	for (int i = 1; i < 32; i++) {
		if (b[i] == b[i - 1]) {
			if (++run > 6)
				return false;
		} else {
			run = 1;
			trans++;
			if (i <= 5)
				trans_ms6++;
			if (i >= 17)
				trans_ls16++;
		}
	}
	for (int i = 24; i < 32; i++)
		ones_ls8 += b[i] == '1';
	uint32_t d = aa ^ 0x8E89BED6u;
	int dbits = 0;
	for (int i = 0; i < 32; i++)
		dbits += (d >> i) & 1;
	uint8_t o0 = (uint8_t)aa;
	bool all_eq = ((aa >> 8) & 0xFF) == o0 && ((aa >> 16) & 0xFF) == o0 && (aa >> 24) == o0;
	return dbits > 1 && !all_eq && trans <= 24 && trans_ms6 >= 2 && trans_ls16 <= 11 && ones_ls8 >= 3;
}

static uint64_t g_rng = 0x9E3779B97F4A7C15ull;
static void rng_bytes(void *ctx, uint8_t *buf, uint16_t len)
{
	(void)ctx;
	for (uint16_t i = 0; i < len; i++) {
		g_rng ^= g_rng << 13;
		g_rng ^= g_rng >> 7;
		g_rng ^= g_rng << 17;
		buf[i] = (uint8_t)(g_rng >> 24);
	}
}

static void test_access_address(void)
{
	/* known bad */
	CHECK(!cis_aa_valid(0x8E89BED6u));
	CHECK(!cis_aa_valid(0x8E89BED6u ^ 0x00010000u));
	CHECK(!cis_aa_valid(0x5A5A5A5Au));          /* all octets equal */
	CHECK(!cis_aa_valid(0x0F0F0F0Fu ^ 0x0u));   /* octets equal */
	CHECK(!cis_aa_valid(0x7F2A55A5u));          /* 7 ones at the start */
	CHECK(!cis_aa_valid(0xAAAAAAAAu));          /* 31 transitions */
	CHECK(!cis_aa_valid(0xF85A33C5u ^ 0x0u) == !aa_rules_ref(0xF85A33C5u));
	CHECK(!cis_aa_valid(0x0C5A33C5u) == !aa_rules_ref(0x0C5A33C5u));
	CHECK(!cis_aa_valid(0x8E5A3300u));          /* too few ones in the lower octet */

	/* the validator agrees with the independent checker on random 32-bit values */
	int disagree = 0;
	for (int i = 0; i < 100000; i++) {
		uint32_t x;
		rng_bytes(NULL, (uint8_t *)&x, 4);
		if (cis_aa_valid(x) != aa_rules_ref(x))
			disagree++;
	}
	CHECK_EQ(disagree, 0);

	/* 100 000 generated: each satisfies the rules and differs from the forbidden ones */
	int bad = 0, zero = 0, dup = 0;
	uint32_t avoid[2] = { 0, 0 };
	for (int i = 0; i < 100000; i++) {
		uint32_t aa = cis_aa_generate(rng_bytes, NULL, avoid, 2);
		if (aa == 0) {
			zero++;
			continue;
		}
		if (!aa_rules_ref(aa))
			bad++;
		if (aa == avoid[0] || aa == avoid[1])
			dup++;
		avoid[i & 1] = aa;
	}
	CHECK_EQ(bad, 0);
	CHECK_EQ(zero, 0);
	CHECK_EQ(dup, 0);
}

/* ---------------- PDU header, flushing, segmentation ---------------- */
static void test_pdu_header(void)
{
	cis_pdu_hdr_t h = { .llid = CIS_LLID_START_CONT, .nesn = true, .sn = false, .cie = true, .npi = true };
	CHECK_EQ(cis_pdu_hdr_pack(&h), 0x01 | 0x04 | 0x10 | 0x40);
	cis_pdu_hdr_t u;
	cis_pdu_hdr_unpack(0x0A, &u);   /* LLID 2, SN */
	CHECK_EQ(u.llid, 2);
	CHECK(u.sn && !u.nesn && !u.cie && !u.npi);
	cis_pdu_hdr_unpack(0xFF, &u);
	CHECK_EQ(u.llid, 3);
	CHECK(u.sn && u.nesn && u.cie && u.npi);
	/* RFU bits (5, 7) do not survive pack */
	CHECK_EQ(cis_pdu_hdr_pack(&u), 0x5F);
}

static void test_flush_point(void)
{
	uint64_t ev;
	uint8_t se;
	cis_flush_point(5, 1, 1, 1, &ev, &se);
	CHECK_EQ(ev, 5);
	CHECK_EQ(se, 0);
	cis_flush_point(5, 1, 3, 2, &ev, &se);
	CHECK_EQ(ev, 7);
	CHECK_EQ(se, 1);
	/* BN=2, NSE=4, FT=2: u = NSE - floor(NSE/BN) * (BN - 1 - k mod BN) */
	cis_flush_point(4, 2, 2, 4, &ev, &se);
	CHECK_EQ(ev, 3);
	CHECK_EQ(se, 1);
	cis_flush_point(5, 2, 2, 4, &ev, &se);
	CHECK_EQ(ev, 3);
	CHECK_EQ(se, 3);
	/* BN=2, NSE=3: floor(3/2)=1 -> u = 2 i 3 */
	cis_flush_point(6, 2, 1, 3, &ev, &se);
	CHECK_EQ(ev, 3);
	CHECK_EQ(se, 1);
}

static void test_unframed(void)
{
	uint16_t off;
	uint8_t len, llid;
	/* SDU 300 B, Max_PDU 200, BN 2 -> 200 (START) + 100 (END) */
	CHECK(cis_unframed_fragment(300, 200, 2, 0, &off, &len, &llid));
	CHECK_EQ(off, 0);
	CHECK_EQ(len, 200);
	CHECK_EQ(llid, CIS_LLID_START_CONT);
	CHECK(cis_unframed_fragment(300, 200, 2, 1, &off, &len, &llid));
	CHECK_EQ(off, 200);
	CHECK_EQ(len, 100);
	CHECK_EQ(llid, CIS_LLID_COMPLETE_END);
	/* SDU 100 B in BN 2 -> whole + padding (length 0, LLID 0b01) */
	CHECK(cis_unframed_fragment(100, 200, 2, 0, &off, &len, &llid));
	CHECK_EQ(len, 100);
	CHECK_EQ(llid, CIS_LLID_COMPLETE_END);
	CHECK(cis_unframed_fragment(100, 200, 2, 1, &off, &len, &llid));
	CHECK_EQ(len, 0);
	CHECK_EQ(llid, CIS_LLID_START_CONT);
	/* empty SDU: one END PDU of length 0 */
	CHECK(cis_unframed_fragment(0, 200, 1, 0, &off, &len, &llid));
	CHECK_EQ(len, 0);
	CHECK_EQ(llid, CIS_LLID_COMPLETE_END);
	CHECK(!cis_unframed_fragment(401, 200, 2, 0, &off, &len, &llid));
	CHECK(!cis_unframed_fragment(10, 200, 2, 2, &off, &len, &llid));

	uint8_t buf[400], a[200], b[100];
	memset(a, 0xA1, sizeof(a));
	memset(b, 0xB2, sizeof(b));
	cis_unframed_rx_t s;
	cis_unframed_rx_begin(&s, buf, sizeof(buf));
	cis_unframed_rx_pdu(&s, CIS_LLID_START_CONT, a, 200, false);
	cis_unframed_rx_pdu(&s, CIS_LLID_COMPLETE_END, b, 100, false);
	CHECK_EQ(cis_unframed_rx_end(&s), CIS_ISO_STATUS_VALID);
	CHECK_EQ(s.len, 300);
	CHECK_EQ(buf[0], 0xA1);
	CHECK_EQ(buf[299], 0xB2);

	cis_unframed_rx_begin(&s, buf, sizeof(buf));
	cis_unframed_rx_pdu(&s, CIS_LLID_COMPLETE_END, b, 100, false);
	cis_unframed_rx_pdu(&s, CIS_LLID_START_CONT, NULL, 0, false);   /* padding */
	CHECK_EQ(cis_unframed_rx_end(&s), CIS_ISO_STATUS_VALID);
	CHECK_EQ(s.len, 100);

	cis_unframed_rx_begin(&s, buf, sizeof(buf));
	cis_unframed_rx_pdu(&s, CIS_LLID_START_CONT, a, 200, false);
	cis_unframed_rx_pdu(&s, 0, NULL, 0, true);                       /* end lost */
	CHECK_EQ(cis_unframed_rx_end(&s), CIS_ISO_STATUS_INVALID);

	cis_unframed_rx_begin(&s, buf, sizeof(buf));
	cis_unframed_rx_pdu(&s, 0, NULL, 0, true);
	CHECK_EQ(cis_unframed_rx_end(&s), CIS_ISO_STATUS_LOST);
	CHECK_EQ(s.len, 0);

	/* buffer overflow -> incomplete, no write past the end */
	cis_unframed_rx_begin(&s, buf, 150);
	cis_unframed_rx_pdu(&s, CIS_LLID_START_CONT, a, 200, false);
	cis_unframed_rx_pdu(&s, CIS_LLID_COMPLETE_END, b, 100, false);
	CHECK_EQ(cis_unframed_rx_end(&s), CIS_ISO_STATUS_INVALID);
	CHECK(s.len <= 150);
}

/* ---------------- HCI ISO and events ---------------- */
static void test_hci_iso(void)
{
	uint8_t out[64];
	static const uint8_t data[3] = { 0xAA, 0xBB, 0xCC };
	cis_hci_iso_t p = { .handle = 0x0E00, .pb = CIS_ISO_PB_COMPLETE, .ts_flag = true, .timestamp = 0x11223344,
		.psn = 5, .sdu_len = 3, .status = 0, .data = data, .data_len = 3 };
	static const uint8_t want[16] = { 0x05, 0x00, 0x6E, 0x0B, 0x00, 0x44, 0x33, 0x22, 0x11, 0x05, 0x00, 0x03, 0x00,
		0xAA, 0xBB, 0xCC };
	CHECK_EQ(cis_hci_iso_build(&p, out, sizeof(out)), 16);
	CHECK_MEM(out, want, 16);
	CHECK_EQ(cis_hci_iso_build(&p, out, 15), 0);
	p.status = CIS_ISO_STATUS_INVALID;
	cis_hci_iso_build(&p, out, sizeof(out));
	CHECK_EQ(out[11], 0x03);
	CHECK_EQ(out[12], 0x40);

	cis_hci_iso_t q;
	static const uint8_t host[12] = { 0x05, 0x01, 0x2E, 0x07, 0x00, 0x09, 0x00, 0x03, 0x00, 0x01, 0x02, 0x03 };
	CHECK(cis_hci_iso_parse(host, 12, &q));
	CHECK_EQ(q.handle, 0x0E01);
	CHECK_EQ(q.pb, CIS_ISO_PB_COMPLETE);
	CHECK(!q.ts_flag);
	CHECK_EQ(q.psn, 9);
	CHECK_EQ(q.sdu_len, 3);
	CHECK_EQ(q.data_len, 3);
	CHECK_EQ(q.data ? q.data[2] : 0x100, 0x03);
	CHECK(!cis_hci_iso_parse(host, 11, &q));        /* length does not match the header */
	CHECK(!cis_hci_iso_parse(host + 1, 11, &q));    /* missing type byte */
	CHECK(cis_hci_iso_parse(want, 16, &q));
	CHECK(q.ts_flag);
	CHECK_EQ(q.timestamp, 0x11223344);
	CHECK_EQ(q.psn, 5);
	static const uint8_t cont[7] = { 0x05, 0x00, 0x1E, 0x02, 0x00, 0xAA, 0xBB };
	CHECK(cis_hci_iso_parse(cont, 7, &q));
	CHECK_EQ(q.pb, CIS_ISO_PB_CONT);
	CHECK_EQ(q.data_len, 2);
	static const uint8_t trunc[8] = { 0x05, 0x00, 0x2E, 0x03, 0x00, 0x09, 0x00, 0x03 };
	CHECK(!cis_hci_iso_parse(trunc, 8, &q));         /* the SDU header does not fit in the payload */
}

static void test_hci_events(void)
{
	uint8_t out[64];
	static const uint8_t ret[3] = { 0x00, 0x01, 0x02 };
	static const uint8_t want_cc[9] = { 0x04, 0x0E, 0x06, 0x01, 0x65, 0x20, 0x00, 0x01, 0x02 };
	CHECK_EQ(cis_hci_evt_cmd_complete(out, 0x2065, ret, 3), 9);
	CHECK_MEM(out, want_cc, 9);
	static const uint8_t want_cs[7] = { 0x04, 0x0F, 0x04, 0x00, 0x01, 0x64, 0x20 };
	CHECK_EQ(cis_hci_evt_cmd_status(out, 0, 0x2064), 7);
	CHECK_MEM(out, want_cs, 7);
	static const uint8_t want_dc[7] = { 0x04, 0x05, 0x04, 0x00, 0x01, 0x0E, 0x16 };
	CHECK_EQ(cis_hci_evt_disconn_complete(out, 0, 0x0E01, 0x16), 7);
	CHECK_MEM(out, want_dc, 7);
	static const uint16_t hs[2] = { 0x0E00, 0x0E01 };
	static const uint16_t cs[2] = { 2, 0x0102 };
	static const uint8_t want_nocp[12] = { 0x04, 0x13, 0x09, 0x02, 0x00, 0x0E, 0x02, 0x00, 0x01, 0x0E, 0x02, 0x01 };
	CHECK_EQ(cis_hci_evt_nocp(out, hs, cs, 2), 12);
	CHECK_MEM(out, want_nocp, 12);

	cis_hci_cis_established_t ev = { .status = 0, .handle = 0x0E00, .cig_sync_delay_us = 2120,
		.cis_sync_delay_us = 1060, .tl_c_to_p_us = 1060, .tl_p_to_c_us = 11060, .phy_c_to_p = 2,
		.phy_p_to_c = 2, .nse = 1, .bn_c_to_p = 1, .bn_p_to_c = 1, .ft_c_to_p = 1, .ft_p_to_c = 2,
		.max_pdu_c_to_p = 120, .max_pdu_p_to_c = 40, .iso_interval = 8 };
	static const uint8_t want_est[32] = { 0x04, 0x3E, 0x1D, 0x19, 0x00, 0x00, 0x0E, 0x48, 0x08, 0x00, 0x24, 0x04,
		0x00, 0x24, 0x04, 0x00, 0x34, 0x2B, 0x00, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x78, 0x00, 0x28,
		0x00, 0x08, 0x00 };
	CHECK_EQ(cis_hci_evt_cis_established(out, &ev), CIS_HCI_CIS_ESTABLISHED_LEN);
	CHECK_MEM(out, want_est, 32);

	uint16_t op;
	const uint8_t *prm;
	uint8_t plen;
	static const uint8_t cmd[6] = { 0x01, 0x65, 0x20, 0x01, 0x07, 0x00 };
	CHECK(cis_hci_cmd_header(cmd, 5, &op, &prm, &plen));
	CHECK_EQ(op, 0x2065);
	CHECK_EQ(plen, 1);
	CHECK_EQ(prm[0], 0x07);
	CHECK(!cis_hci_cmd_header(cmd, 6, &op, &prm, &plen));
	CHECK(!cis_hci_cmd_header(cmd, 3, &op, &prm, &plen));
}

/* ---------------- Set CIG Parameters and CIG derivation ---------------- */
static const uint8_t cig_cmd[33] = {
	0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02,
	0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x02, 0x02,
	0x01, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x02, 0x02,
};

static void test_cig(void)
{
	cis_hci_cig_params_t p;
	cis_cig_derived_t d;
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	CHECK_EQ(p.cig_id, 1);
	CHECK_EQ(p.sdu_interval_c_to_p, 10000);
	CHECK_EQ(p.latency_p_to_c_ms, 10);
	CHECK_EQ(p.cis_count, 2);
	CHECK_EQ(p.cis[1].cis_id, 1);
	CHECK_EQ(p.cis[1].max_sdu_c_to_p, 120);
	CHECK_EQ(p.cis[1].rtn_p_to_c, 2);
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 32, &p), CIS_ERR_INVALID_PARAMS);
	{
		uint8_t three[42];
		memcpy(three, cig_cmd, 33);
		three[14] = 3;
		memcpy(three + 33, cig_cmd + 15, 9);
		three[33] = 2;
		CHECK_EQ(cis_hci_cig_params_parse(three, 42, &p), CIS_ERR_MEM_CAPACITY);
	}

	CHECK_EQ(cis_pdu_air_us(124, CIS_PHY_2M), 540);
	CHECK_EQ(cis_pdu_air_us(44, CIS_PHY_2M), 220);
	CHECK_EQ(cis_pdu_air_us(0, CIS_PHY_1M), 80);

	/* one CIS, max_nse = 1 (C6 version 1) */
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.cis_count = 1;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), 0);
	CHECK_EQ(d.iso_interval, 8);
	CHECK_EQ(d.iso_interval_us, 10000);
	CHECK_EQ(d.cis[0].phy_c_to_p, CIS_PHY_2M);
	CHECK_EQ(d.cis[0].max_pdu_c_to_p, 120);
	CHECK_EQ(d.cis[0].max_pdu_p_to_c, 40);
	CHECK_EQ(d.cis[0].bn_c_to_p, 1);
	CHECK_EQ(d.cis[0].bn_p_to_c, 1);
	CHECK_EQ(d.cis[0].nse, 1);
	CHECK_EQ(d.cis[0].ft_c_to_p, 1);
	CHECK_EQ(d.cis[0].ft_p_to_c, 1);
	CHECK_EQ(d.cis[0].sub_interval_us, 0);
	CHECK_EQ(d.cis[0].se_length_us, 1060);
	CHECK_EQ(d.cig_sync_delay_us, 1060);
	CHECK_EQ(d.cis[0].cis_sync_delay_us, 1060);
	CHECK_EQ(d.cis[0].tl_c_to_p_us, 1060);

	/* Different latencies per direction with a single subevent: 40 ms C->P would give FT 4, but NSE = BN = 1 leaves
	 * no subevent for a retransmission - FT 1 also when the other direction has FT 1 (formerly checked only for min
	 * FT). */
	p.latency_c_to_p_ms = 40;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), 0);
	CHECK_EQ(d.cis[0].nse, 1);
	CHECK_EQ(d.cis[0].ft_c_to_p, 1);
	CHECK_EQ(d.cis[0].ft_p_to_c, 1);
	CHECK_EQ(d.cis[0].tl_c_to_p_us, 1060);
	p.latency_c_to_p_ms = 10;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), 0);

	/* C->P jitter buffer from the latency margin: (Max_Transport_Latency - TL) / SDU interval, within
	 * CIS_JITTER_MIN..MAX. The latency from cig_cmd (10 ms): (10000 - 1060) / 10000 = 0 -> minimum 2. */
	CHECK_EQ(d.cis[0].jitter_c_to_p, CIS_JITTER_MIN);
	p.latency_c_to_p_ms = 40;   /* (40000 - 1060) / 10000 = 3 */
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), 0);
	CHECK_EQ(d.cis[0].jitter_c_to_p, 3);
	p.latency_c_to_p_ms = 100;  /* 9 -> maximum 4 */
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), 0);
	CHECK_EQ(d.cis[0].jitter_c_to_p, CIS_JITTER_MAX);
	p.latency_c_to_p_ms = 5;    /* latency smaller than TL: still the minimum */
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), 0);
	CHECK_EQ(d.cis[0].jitter_c_to_p, CIS_JITTER_MIN);
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.cis_count = 1;

	/* max_nse = 4: NSE = (RTN+1)*BN = 3 at FT = 1 */
	CHECK_EQ(cis_cig_derive(&p, 4, 0, 0, &d), 0);
	CHECK_EQ(d.cis[0].nse, 3);
	CHECK_EQ(d.cis[0].sub_interval_us, 1060);
	CHECK_EQ(d.cig_sync_delay_us, 3180);
	CHECK_EQ(d.cis[0].ft_c_to_p, 1);

	/* two CISes, 40 ms latency, RTN 2: the latency gives FT 4, but then NSE = 1 = BN and there is no free
	 * subevent for a retransmission - the queue would become permanently delayed. We take the largest FT at which
	 * NSE > BN: FT 2, NSE 2. Sequential packing. */
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.latency_c_to_p_ms = 40;
	p.latency_p_to_c_ms = 40;
	CHECK_EQ(cis_cig_derive(&p, 4, 0, 0, &d), 0);
	CHECK_EQ(d.cis[0].ft_c_to_p, 2);
	CHECK_EQ(d.cis[0].ft_p_to_c, 2);
	CHECK_EQ(d.cis[0].nse, 2);
	CHECK_EQ(d.cig_sync_delay_us, 4240);
	CHECK_EQ(d.cis[0].cis_sync_delay_us, 4240);
	CHECK_EQ(d.cis[1].cis_sync_delay_us, 2120);
	CHECK_EQ(d.cis[0].tl_c_to_p_us, 4240 + 20000 - 10000);
	/* a radio with a single subevent: FT > 1 gives nothing but latency -> FT 1 */
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), 0);
	CHECK_EQ(d.cis[0].nse, 1);
	CHECK_EQ(d.cis[0].ft_c_to_p, 1);
	CHECK_EQ(d.cis[0].ft_p_to_c, 1);
	CHECK_EQ(d.cig_sync_delay_us, 2120);

	/* gap between CISes (the C6 radio switches the link with a separate event): CIS 1 at 1060 + 2500 us after CIS 0,
	 * CIG_Sync_Delay includes the gap, also in the transport latency; a single CIS - unchanged */
	CHECK_EQ(cis_cig_derive(&p, 1, 2500, 0, &d), 0);
	CHECK_EQ(d.cig_sync_delay_us, 2120 + 2500);
	CHECK_EQ(d.cis[0].cis_sync_delay_us, 2120 + 2500);
	CHECK_EQ(d.cis[1].cis_sync_delay_us, 1060);
	CHECK_EQ(d.cis[0].tl_c_to_p_us, 2120 + 2500 + 10000 - 10000);
	CHECK_EQ(d.cis[1].tl_c_to_p_us, 2120 + 2500 + 10000 - 10000);
	/* a gap at which the CIG does not fit in the ISO interval */
	CHECK_EQ(cis_cig_derive(&p, 1, 8000, 0, &d), CIS_ERR_UNSUPP_FEATURE);
	p.cis_count = 1;
	CHECK_EQ(cis_cig_derive(&p, 1, 2500, 0, &d), 0);
	CHECK_EQ(d.cig_sync_delay_us, 1060);
	CHECK_EQ(d.cis[0].cis_sync_delay_us, 1060);

	/* NSE 2 on a radio that needs a gap between subevents (C6: a separate schedule event for each):
	 * Sub_Interval = se_length + gap, CIS time = NSE x Sub_Interval, CIS 1 after CIS 0 by the CIS time + the CIS gap */
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	CHECK_EQ(cis_cig_derive(&p, 2, 500, 500, &d), 0);
	CHECK_EQ(d.cis[0].nse, 2);
	CHECK_EQ(d.cis[1].nse, 2);
	CHECK_EQ(d.cis[0].sub_interval_us, 1060 + 500);
	CHECK_EQ(d.cig_sync_delay_us, 2 * 2 * 1560 + 500);
	CHECK_EQ(d.cis[0].cis_sync_delay_us, 2 * 2 * 1560 + 500);
	CHECK_EQ(d.cis[1].cis_sync_delay_us, 2 * 1560);
	/* NSE 1 on the same radio: the CIS event is also a separate schedule event, so the same gap follows it -
	 * CIS 1 at 1060 + 500 (+ CIS gap) after CIS 0 (Tab5, 2026-11-01: 1.56 ms works, 1.06 ms does not); Sub_Interval in
	 * LL_CIS_REQ stays 0 */
	CHECK_EQ(cis_cig_derive(&p, 1, 2500, 500, &d), 0);
	CHECK_EQ(d.cis[0].nse, 1);
	CHECK_EQ(d.cis[0].sub_interval_us, 0);
	CHECK_EQ(d.cig_sync_delay_us, 2 * 1560 + 2500);
	CHECK_EQ(d.cis[0].cis_sync_delay_us, 2 * 1560 + 2500);
	CHECK_EQ(d.cis[1].cis_sync_delay_us, 1560);
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 500, &d), 0);
	CHECK_EQ(d.cig_sync_delay_us, 2 * 1560);
	CHECK_EQ(d.cis[1].cis_sync_delay_us, 1560);

	/* only one direction: BN 0 and Max_PDU 0 on the side without data */
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.cis_count = 1;
	p.cis[0].max_sdu_p_to_c = 0;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), 0);
	CHECK_EQ(d.cis[0].bn_p_to_c, 0);
	CHECK_EQ(d.cis[0].max_pdu_p_to_c, 0);

	/* SDU 300 B -> Max_PDU 247... BN 2 */
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.cis_count = 1;
	p.cis[0].max_sdu_c_to_p = 300;
	CHECK_EQ(cis_cig_derive(&p, 2, 0, 0, &d), 0);
	CHECK_EQ(d.cis[0].max_pdu_c_to_p, CIS_MAX_PDU);
	CHECK_EQ(d.cis[0].bn_c_to_p, 2);
	CHECK(d.cis[0].nse >= 2);

	/* things we do not support or that are invalid */
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.framing = 1;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), CIS_ERR_UNSUPP_FEATURE);
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.sdu_interval_c_to_p = 10001;
	p.sdu_interval_p_to_c = 10001;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), CIS_ERR_UNSUPP_FEATURE);
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.cis[0].phy_c_to_p = 0x04;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), CIS_ERR_UNSUPP_FEATURE);
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.cis[0].max_sdu_c_to_p = CIS_SDU_MAX + 1;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), CIS_ERR_UNSUPP_FEATURE);
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.cis[1].cis_id = p.cis[0].cis_id;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), CIS_ERR_INVALID_PARAMS);
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.latency_c_to_p_ms = 4;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), CIS_ERR_INVALID_PARAMS);
	CHECK_EQ(cis_hci_cig_params_parse(cig_cmd, 33, &p), 0);
	p.cig_id = 0xF0;
	CHECK_EQ(cis_cig_derive(&p, 1, 0, 0, &d), CIS_ERR_INVALID_PARAMS);
}

int main(void)
{
	test_csa2();
	test_llcp_pdus();
	test_access_address();
	test_pdu_header();
	test_flush_point();
	test_unframed();
	test_hci_iso();
	test_hci_events();
	test_cig();
	TEST_DONE("test_proto");
}
