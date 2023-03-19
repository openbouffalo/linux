// SPDX-License-Identifier: GPL-2.0
//
// Bouffalo Lab SoC Secure Engine driver
//
// Based on qcom-rng.c
// Copyright (c) 2017-18 Linaro Limited

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <crypto/internal/rng.h>
#include <linux/crypto.h>
#include <linux/hw_random.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/sched.h>

//Register map
//se_sha_0_ctrl
#define REG_SECENG_SHA_0_CTRL                       0 //Offset from base address
#define REG_SECENG_SHA_0_CTRL_MSG_LEN               GENMASK(31, 16)
#define REG_SECENG_SHA_0_CTRL_LINK_MODE             BIT(15)
#define REG_SECENG_SHA_0_CTRL_MODE_EXT              GENMASK(13, 12)
#define REG_SECENG_SHA_0_CTRL_INT_MASK              BIT(11)
#define REG_SECENG_SHA_0_CTRL_INT_SET_1T            BIT(10)
#define REG_SECENG_SHA_0_CTRL_INT_CLR_1T            BIT(9)
#define REG_SECENG_SHA_0_CTRL_INT                   BIT(8)
#define REG_SECENG_SHA_0_CTRL_HASH_SEL              BIT(6)
#define REG_SECENG_SHA_0_CTRL_EN                    BIT(5)
#define REG_SECENG_SHA_0_CTRL_MODE                  GENMASK(4, 2)
#define REG_SECENG_SHA_0_CTRL_INT_TRIG_1T           BIT(1)
#define REG_SECENG_SHA_0_CTRL_BUSY                  BIT(0)

//se_sha_0_msa
#define REG_SECENG_SHA_0_MSA                        4

//se_sha_0_status
#define REG_SECENG_SHA_0_STATUS                     8

//se_sha_0_endian
#define REG_SECENG_SHA_0_ENDIAN                     12
#define REG_SECENG_SHA_0_ENDIAN_VAL                 BIT(0)

//se_sha_0_hash_l_0
#define REG_SECENG_SHA_0_HASH_L_0                   16

//se_sha_0_hash_l_1
#define REG_SECENG_SHA_0_HASH_L_1                   20

//se_sha_0_hash_l_2
#define REG_SECENG_SHA_0_HASH_L_2                   24

//se_sha_0_hash_l_3
#define REG_SECENG_SHA_0_HASH_L_3                   28

//se_sha_0_hash_l_4
#define REG_SECENG_SHA_0_HASH_L_4                   32

//se_sha_0_hash_l_5
#define REG_SECENG_SHA_0_HASH_L_5                   36

//se_sha_0_hash_l_6
#define REG_SECENG_SHA_0_HASH_L_6                   40

//se_sha_0_hash_l_7
#define REG_SECENG_SHA_0_HASH_L_7                   44

//se_sha_0_hash_h_0
#define REG_SECENG_SHA_0_HASH_H_0                   48

//se_sha_0_hash_h_1
#define REG_SECENG_SHA_0_HASH_H_1                   52

//se_sha_0_hash_h_2
#define REG_SECENG_SHA_0_HASH_H_2                   56

//se_sha_0_hash_h_3
#define REG_SECENG_SHA_0_HASH_H_3                   60

//se_sha_0_hash_h_4
#define REG_SECENG_SHA_0_HASH_H_4                   64

//se_sha_0_hash_h_5
#define REG_SECENG_SHA_0_HASH_H_5                   68

//se_sha_0_hash_h_6
#define REG_SECENG_SHA_0_HASH_H_6                   72

//se_sha_0_hash_h_7
#define REG_SECENG_SHA_0_HASH_H_7                   76

//se_sha_0_link
#define REG_SECENG_SHA_0_LINK                       80

//se_sha_0_ctrl_prot
#define REG_SECENG_SHA_0_CTRL_PROT                  252
#define REG_SECENG_SHA_0_CTRL_PROT_ID1_EN           BIT(2)
#define REG_SECENG_SHA_0_CTRL_PROT_ID0_EN           BIT(1)

//se_aes_0_ctrl
#define REG_SECENG_AES_0_CTRL                       256
#define REG_SECENG_AES_0_CTRL_MSG_LEN               GENMASK(31, 16)
#define REG_SECENG_AES_0_CTRL_LINK_MODE             BIT(15)
#define REG_SECENG_AES_0_CTRL_IV_SEL                BIT(14)
#define REG_SECENG_AES_0_CTRL_BLOCK_MODE            GENMASK(13, 12)
#define REG_SECENG_AES_0_CTRL_INT_MASK              BIT(11)
#define REG_SECENG_AES_0_CTRL_INT_SET_1T            BIT(10)
#define REG_SECENG_AES_0_CTRL_INT_CLR_1T            BIT(9)
#define REG_SECENG_AES_0_CTRL_INT                   BIT(8)
#define REG_SECENG_AES_0_CTRL_HW_KEY_EN             BIT(7)
#define REG_SECENG_AES_0_CTRL_DEC_KEY_SEL           BIT(6)
#define REG_SECENG_AES_0_CTRL_DEC_EN                BIT(5)
#define REG_SECENG_AES_0_CTRL_MODE                  GENMASK(4, 3)
#define REG_SECENG_AES_0_CTRL_EN                    BIT(2)
#define REG_SECENG_AES_0_CTRL_TRIG_1T               BIT(1)
#define REG_SECENG_AES_0_CTRL_BUSY                  BIT(0)

//se_aes_0_msa
#define REG_SECENG_AES_0_MSA                        260

//se_aes_0_mda
#define REG_SECENG_AES_0_MDA                        264

//se_aes_0_status
#define REG_SECENG_AES_0_STATUS                     268

//se_aes_0_iv_0
#define REG_SECENG_AES_0_IV_0                       272

//se_aes_0_iv_1
#define REG_SECENG_AES_0_IV_1                       276

//se_aes_0_iv_2
#define REG_SECENG_AES_0_IV_2                       280

//se_aes_0_iv_3
#define REG_SECENG_AES_0_IV_3                       284

//se_aes_0_key_0
#define REG_SECENG_AES_0_KEY_0                      288

//se_aes_0_key_1
#define REG_SECENG_AES_0_KEY_1                      292

//se_aes_0_key_2
#define REG_SECENG_AES_0_KEY_2                      296

//se_aes_0_key_3
#define REG_SECENG_AES_0_KEY_3                      300

//se_aes_0_key_4
#define REG_SECENG_AES_0_KEY_4                      304

//se_aes_0_key_5
#define REG_SECENG_AES_0_KEY_5                      308

//se_aes_0_key_6
#define REG_SECENG_AES_0_KEY_6                      312

//se_aes_0_key_7
#define REG_SECENG_AES_0_KEY_7                      316

//se_aes_0_key_sel
#define REG_SECENG_AES_0_KEY_SEL                    320
#define REG_SECENG_AES_0_KEY_SEL_VAL                GENMASK(1, 0)

//se_aes_1_key_sel
#define REG_SECENG_AES_1_KEY_SEL                    324
#define REG_SECENG_AES_01KEY_SEL_VAL                GENMASK(1, 0)

//se_aes_0_endian
#define REG_SECENG_AES_0_ENDIAN                     328
#define REG_SECENG_AES_0_ENDIAN_CTR_LEN             GENMASK(31, 30)
#define REG_SECENG_AES_0_ENDIAN_TWK                 BIT(4)
#define REG_SECENG_AES_0_ENDIAN_IV                  BIT(3)
#define REG_SECENG_AES_0_ENDIAN_KEY                 BIT(2)
#define REG_SECENG_AES_0_ENDIAN_DIN                 BIT(1)
#define REG_SECENG_AES_0_ENDIAN_DOUT                BIT(0)

//se_aes_sboot
#define REG_SECENG_AES_SBOOT                        332
#define REG_SECENG_AES_SBOOT_UNI_LEN                GENMASK(31, 16)
#define REG_SECENG_AES_SBOOT_XTS_MODE               BIT(15)
#define REG_SECENG_AES_SBOOT_KEY_SEL                BIT(0)

//se_aes_0_link
#define REG_SECENG_AES_0_LINK                       336

//se_aes_0_ctrl_prot
#define REG_SECENG_AES_0_CTRL_PROT                  508
#define REG_SECENG_AES_0_CTRL_PROT_ID1_EN           BIT(2)
#define REG_SECENG_AES_0_CTRL_PROT_ID0_EN           BIT(1)

//se_trng_0_ctrl_0
#define REG_SECENG_TRNG_0_CTRL_0                    512
#define REG_SECENG_TRNG_0_CTRL_0_MANUAL_EN          BIT(15)
#define REG_SECENG_TRNG_0_CTRL_0_MANUAL_RESEED      BIT(14)
#define REG_SECENG_TRNG_0_CTRL_0_MANUAL_FUN_SEL     BIT(13)
#define REG_SECENG_TRNG_0_CTRL_0_INT_MASK           BIT(11)
#define REG_SECENG_TRNG_0_CTRL_0_INT_SET_1T         BIT(10)
#define REG_SECENG_TRNG_0_CTRL_0_INT_CLR_1T         BIT(9)
#define REG_SECENG_TRNG_0_CTRL_0_INT                BIT(8)
#define REG_SECENG_TRNG_0_CTRL_0_HT_ERROR           BIT(4)
#define REG_SECENG_TRNG_0_CTRL_0_DOUT_CLR_1T        BIT(3)
#define REG_SECENG_TRNG_0_CTRL_0_EN                 BIT(2)
#define REG_SECENG_TRNG_0_CTRL_0_TRIG_1T            BIT(1)
#define REG_SECENG_TRNG_0_CTRL_0_BUSY               BIT(0)

//se_trng_0_status
#define REG_SECENG_TRNG_0_STATUS                    516

//se_trng_0_dout_0
#define REG_SECENG_TRNG_0_DOUT_0                    520

//se_trng_0_dout_1
#define REG_SECENG_TRNG_0_DOUT_1                    524

//se_trng_0_dout_2
#define REG_SECENG_TRNG_0_DOUT_2                    528

//se_trng_0_dout_3
#define REG_SECENG_TRNG_0_DOUT_3                    532

//se_trng_0_dout_4
#define REG_SECENG_TRNG_0_DOUT_4                    536

//se_trng_0_dout_5
#define REG_SECENG_TRNG_0_DOUT_5                    540

//se_trng_0_dout_6
#define REG_SECENG_TRNG_0_DOUT_6                    544

//se_trng_0_dout_7
#define REG_SECENG_TRNG_0_DOUT_7                    548

//se_trng_0_test
#define REG_SECENG_TRNG_0_TEST                      552
#define REG_SECENG_TRNG_0_TEST_HT_ALARM_N           GENMASK(11, 4)
#define REG_SECENG_TRNG_0_TEST_HT_DIS               BIT(3)
#define REG_SECENG_TRNG_0_TEST_CP_BYPASS            BIT(2)
#define REG_SECENG_TRNG_0_TEST_CP_TEST_EN           BIT(1)
#define REG_SECENG_TRNG_0_TEST_TEST_EN              BIT(0)

//se_trng_0_ctrl_1
#define REG_SECENG_TRNG_0_CTRL_1_RESEED_N_LSB       556

//se_trng_0_ctrl_2
#define REG_SECENG_TRNG_0_CTRL_2_RESEED_N_MSB       560
#define REG_SECENG_TRNG_0_CTRL_1_RESEED_N_MSB_VALUE GENMASK(15, 0)

//se_trng_0_ctrl_3
#define REG_SECENG_TRNG_0_CTRL_3                    564
#define REG_SECENG_TRNG_0_CTRL_3_ROSC_EN            BIT(31)
#define REG_SECENG_TRNG_0_CTRL_3_HT_OD_EN           BIT(26)
#define REG_SECENG_TRNG_0_CTRL_3_HT_APT_C           GENMASK(25, 16)
#define REG_SECENG_TRNG_0_CTRL_3_HT_RCT_C           GENMASK(15, 8)
#define REG_SECENG_TRNG_0_CTRL_3_CP_RATIO           GENMASK(7, 0)

//se_trng_0_test_out_0
#define REG_SECENG_TRNG_0_TEST_OUT_0                576

//se_trng_0_test_out_1
#define REG_SECENG_TRNG_0_TEST_OUT_1                580

//se_trng_0_test_out_2
#define REG_SECENG_TRNG_0_TEST_OUT_2                584

//se_trng_0_test_out_3
#define REG_SECENG_TRNG_0_TEST_OUT_3                588

//se_trng_0_ctrl_prot
#define REG_SECENG_TRNG_0_CTRL_PROT                 764
#define REG_SECENG_TRNG_0_CTRL_PROT_ID1_EN          BIT(2)
#define REG_SECENG_TRNG_0_CTRL_PROT_ID0_EN          BIT(1)

//se_pka_0_ctrl_0
#define REG_SECENG_PKA_0_CTRL_0                     768
#define REG_SECENG_PKA_0_CTRL_0_STATUS              GENMASK(31, 16)
#define REG_SECENG_PKA_0_CTRL_0_STATUS_CLR_1T       BIT(15)
#define REG_SECENG_PKA_0_CTRL_0_RAM_CLR_MD          BIT(13)
#define REG_SECENG_PKA_0_CTRL_0_ENDIAN              BIT(12)
#define REG_SECENG_PKA_0_CTRL_0_INT_MASK            BIT(11)
#define REG_SECENG_PKA_0_CTRL_0_INT_SET             BIT(10)
#define REG_SECENG_PKA_0_CTRL_0_INT_CLR_1T          BIT(9)
#define REG_SECENG_PKA_0_CTRL_0_INT                 BIT(8)
#define REG_SECENG_PKA_0_CTRL_0_PROT_MD             GENMASK(7, 4)
#define REG_SECENG_PKA_0_CTRL_0_EN                  BIT(3)
#define REG_SECENG_PKA_0_CTRL_0_BUSY                BIT(2)
#define REG_SECENG_PKA_0_CTRL_0_DONE_CLR_1T         BIT(1)
#define REG_SECENG_PKA_0_CTRL_0_DONE                BIT(0)

//se_pka_0_seed
#define REG_SECENG_PKA_0_SEED                       780

//se_pka_0_ctrl_1
#define REG_SECENG_PKA_0_CTRL_1                     784
#define REG_SECENG_PKA_0_CTRL_1_HBYPASS             BIT(3)
#define REG_SECENG_PKA_0_CTRL_1_HBURST              GENMASK(2, 0)

//se_pka_0_rw
#define REG_SECENG_PKA_0_RW                         832
//This is a confusing register which I have not mapped because
//"BL808 Reference Manual 1.2 EN" seems wrong

//se_pka_0_rw_burst
#define REG_SECENG_PKA_0_RW_BURST                   864
//This is a confusing register which I have not mapped because
//"BL808 Reference Manual 1.2 EN" seems wrong

//se_pka_0_ctrl_prot
#define REG_SECENG_PKA_0_CTRL_PROT                  1020
#define REG_SECENG_PKA_0_CTRL_PROT_ID1_EN           BIT(2)
#define REG_SECENG_PKA_0_CTRL_PROT_ID0_EN           BIT(1)

//se_cdet_0_ctrl_0
#define REG_SECENG_CDET_0_CTRL_0                    1024
#define REG_SECENG_CDET_0_CTRL_0_G_LOOP_MIN         GENMASK(31, 24)
#define REG_SECENG_CDET_0_CTRL_0_G_LOOP_MAX         GENMASK(23, 16)
#define REG_SECENG_CDET_0_CTRL_0_STATUS             GENMASK(15, 2)
#define REG_SECENG_CDET_0_CTRL_0_ERROR              BIT(1)
#define REG_SECENG_CDET_0_CTRL_0_EN                 BIT(0)

//se_cdet_0_ctrl_1
#define REG_SECENG_CDET_0_CTRL_1                    1028
#define REG_SECENG_CDET_0_CTRL_1_G_SLP_N            GENMASK(23, 16)
#define REG_SECENG_CDET_0_CTRL_1_T_DLY_N            GENMASK(15, 8)
#define REG_SECENG_CDET_0_CTRL_1_T_LOOP_N           GENMASK(7, 0)

//se_cdet_0_ctrl_prot
#define REG_SECENG_CDET_0_CTRL_PROT                 1276
#define REG_SECENG_CDET_0_CTRL_PROT_ID1_EN          BIT(2)
#define REG_SECENG_CDET_0_CTRL_PROT_ID0_EN          BIT(1)
#define REG_SECENG_CDET_0_CTRL_PROT_PROT_EN         BIT(0)

//se_gmac_0_ctrl_0
#define REG_SECENG_GMAC_0_CTRL_0                    1280
#define REG_SECENG_GMAC_0_CTRL_0_X_ENDIAN           BIT(14)
#define REG_SECENG_GMAC_0_CTRL_0_H_ENDIAN           BIT(13)
#define REG_SECENG_GMAC_0_CTRL_0_T_ENDIAN           BIT(12)
#define REG_SECENG_GMAC_0_CTRL_0_INT_MASK           BIT(11)
#define REG_SECENG_GMAC_0_CTRL_0_INT_SET_1T         BIT(10)
#define REG_SECENG_GMAC_0_CTRL_0_INT_CLR_1T         BIT(9)
#define REG_SECENG_GMAC_0_CTRL_0_INT                BIT(8)
#define REG_SECENG_GMAC_0_CTRL_0_EN                 BIT(2)
#define REG_SECENG_GMAC_0_CTRL_0_TRIG_1T            BIT(1)
#define REG_SECENG_GMAC_0_CTRL_0_BUSY               BIT(0)

//se_gmac_0_lca
#define REG_SECENG_GMAC_0_LCA                       1284

//se_gmac_0_status
#define REG_SECENG_GMAC_0_STATUS                    1288

//se_gmac_0_ctrl_prot
#define REG_SECENG_GMAC_0_CTRL_PROT                 1532
#define REG_SECENG_GMAC_0_CTRL_PROT_ID1_EN          BIT(2)
#define REG_SECENG_GMAC_0_CTRL_PROT_ID0_EN          BIT(1)

//se_ctrl_prot_rd
#define REG_SECENG_CTRL_PROT_RD                     3840
#define REG_SECENG_CTRL_PROT_RD_DBG_DIS             BIT(31)
#define REG_SECENG_CTRL_PROT_RD_GMAC_ID1_EN_RD      BIT(11)
#define REG_SECENG_CTRL_PROT_RD_GMAC_ID0_EN_RD      BIT(10)
#define REG_SECENG_CTRL_PROT_RD_CDET_ID1_EN_RD      BIT(9)
#define REG_SECENG_CTRL_PROT_RD_CDET_ID0_EN_RD      BIT(8)
#define REG_SECENG_CTRL_PROT_RD_PKA_ID1_EN_RD       BIT(7)
#define REG_SECENG_CTRL_PROT_RD_PKA_ID0_EN_RD       BIT(6)
#define REG_SECENG_CTRL_PROT_RD_TRNG_ID1_EN_RD      BIT(5)
#define REG_SECENG_CTRL_PROT_RD_TRNG_ID0_EN_RD      BIT(4)
#define REG_SECENG_CTRL_PROT_RD_AES_ID1_EN_RD       BIT(3)
#define REG_SECENG_CTRL_PROT_RD_AES_ID0_EN_RD       BIT(2)
#define REG_SECENG_CTRL_PROT_RD_SHA_ID1_EN_RD       BIT(1)
#define REG_SECENG_CTRL_PROT_RD_SHA_ID0_EN_RD       BIT(0)

struct bflb_seceng {
	struct mutex lock;
	struct device *dev;
	void __iomem *base;
	struct regmap *map;
	unsigned int initialised;

	struct hwrng hwrng;
};

struct bflb_seceng_ctx {
	struct bflb_seceng *seceng;
};

struct regmap_config bflb_seceng_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.cache_type = REGCACHE_FLAT,
	.max_register = 512 * sizeof(u32),
	.num_reg_defaults_raw = 512,
	.use_relaxed_mmio = true,
	.use_raw_spinlock = true,
};

static struct bflb_seceng *bflb_seceng_dev;

static void bflb_seceng_trng_wait_ready(void)
{
	while (readl_relaxed(bflb_seceng_dev->base + REG_SECENG_TRNG_0_CTRL_0) &
			REG_SECENG_TRNG_0_CTRL_0_BUSY) {
		dev_dbg(bflb_seceng_dev->dev, "Waiting for TRNG ready");
		schedule();
	}
}

static void bflb_seceng_trng_init(void)
{
	regmap_update_bits(bflb_seceng_dev->map, REG_SECENG_TRNG_0_CTRL_0,
		REG_SECENG_TRNG_0_CTRL_0_EN | REG_SECENG_TRNG_0_CTRL_0_INT_CLR_1T,
		FIELD_PREP(REG_SECENG_TRNG_0_CTRL_0_EN, 1) |
				FIELD_PREP(REG_SECENG_TRNG_0_CTRL_0_INT_CLR_1T, 1));

	bflb_seceng_trng_wait_ready();
}

static void bflb_seceng_trng_refresh(void)
{
	regmap_update_bits(bflb_seceng_dev->map, REG_SECENG_TRNG_0_CTRL_0,
		REG_SECENG_TRNG_0_CTRL_0_DOUT_CLR_1T,
		FIELD_PREP(REG_SECENG_TRNG_0_CTRL_0_DOUT_CLR_1T, 1)); //Clear DOUT

	bflb_seceng_trng_wait_ready();

	regmap_update_bits(bflb_seceng_dev->map, REG_SECENG_TRNG_0_CTRL_0,
		REG_SECENG_TRNG_0_CTRL_0_DOUT_CLR_1T,
		FIELD_PREP(REG_SECENG_TRNG_0_CTRL_0_DOUT_CLR_1T, 0)); //Reset clear DOUT

	regmap_update_bits(bflb_seceng_dev->map,
		REG_SECENG_TRNG_0_CTRL_0, REG_SECENG_TRNG_0_CTRL_0_TRIG_1T,
		FIELD_PREP(REG_SECENG_TRNG_0_CTRL_0_TRIG_1T, 1)); //Force TRNG refresh

	bflb_seceng_trng_wait_ready();

	regmap_update_bits(bflb_seceng_dev->map, REG_SECENG_TRNG_0_CTRL_0,
		REG_SECENG_TRNG_0_CTRL_0_INT_CLR_1T,
		FIELD_PREP(REG_SECENG_TRNG_0_CTRL_0_INT_CLR_1T, 1)); //Clear INT

	dev_dbg(bflb_seceng_dev->dev, "Refreshed TRNG");
}

static unsigned int bflb_seceng_trng_read_dout(struct bflb_seceng *seceng,
		unsigned int doutreg)
{
	return readl_relaxed(seceng->base + REG_SECENG_TRNG_0_DOUT_0 +
			(doutreg * 4));
}

static inline unsigned int bflb_seceng_trng_read32(void)
{
	static u8 doutreg = 8;
	u32 val;

	if (doutreg >= 8) {
		//If we have read all available registers (of starting anew),
		//refresh them and start again
		doutreg = 0;

		bflb_seceng_trng_refresh();
	}

	dev_dbg(bflb_seceng_dev->dev, "Selected TRNG DOUT register %u", doutreg);

	//Read selected register
	val = bflb_seceng_trng_read_dout(bflb_seceng_dev, doutreg);
	doutreg++; //Move on to next register

	dev_dbg(bflb_seceng_dev->dev, "TRNG DOUT register produced %u", val);

	return val;
}

static inline u8 bflb_seceng_trng_read8(void)
{
	static unsigned int lastread;
	static u8 shift = 4;

	if (shift == 4) {
		shift = 0;

		lastread = bflb_seceng_trng_read32();
	}

	return (lastread >> (shift++ * 8)) & 0xFF;
}

static void bflb_seceng_trng_fill_buffer(u8 *buff, unsigned long bufflen)
{
	unsigned long i;

	for (i = 0; i < bufflen; i++)
		buff[i] = bflb_seceng_trng_read8();
}

static int bflb_seceng_trng_hwrng_read(struct hwrng *rng, void *data,
		size_t max, bool wait)
{
	//Currently ignoring wait

	mutex_lock(&bflb_seceng_dev->lock);

	dev_dbg(bflb_seceng_dev->dev, "Starting TRNG hwrng read for %lu bytes...",
			max);

	bflb_seceng_trng_fill_buffer(data, max);

	mutex_unlock(&bflb_seceng_dev->lock);

	//We're always going to fill the buffer, so just return what was asked for
	return max;
}

static int bflb_seceng_trng_crypto_generate(struct crypto_rng *tfm,
	const u8 *src, unsigned int slen, u8 *dstn, unsigned int dlen)
{
	struct bflb_seceng_ctx *ctx = crypto_rng_ctx(tfm);
	struct bflb_seceng *seceng = ctx->seceng;

	mutex_lock(&seceng->lock);

	dev_dbg(seceng->dev,
		"Starting TRNG crypto buffer filling read for %u bytes...", dlen);

	bflb_seceng_trng_fill_buffer(dstn, dlen);

	mutex_unlock(&seceng->lock);

	return 0;
}

static int bflb_seceng_trng_crypto_seed(struct crypto_rng *tfm, const u8 *seed,
	unsigned int slen)
{
	return 0;
}

static int bflb_seceng_trng_crypto_init(struct crypto_tfm *tfm)
{
	struct bflb_seceng_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->seceng = bflb_seceng_dev;

	//Skip actual initialisation of the hardware if we have already done so
	if (bflb_seceng_dev->initialised)
		return 0;

	bflb_seceng_trng_init();

	dev_dbg(bflb_seceng_dev->dev, "Initialised TRNG via crypto");

	bflb_seceng_dev->initialised = 1;
	return 0;
}

static int bflb_seceng_trng_hwrng_init(struct hwrng *rng)
{
	if (bflb_seceng_dev->initialised)
		return 0;

	bflb_seceng_trng_init();

	dev_dbg(bflb_seceng_dev->dev, "Initialised TRNG via hwrng");

	bflb_seceng_dev->initialised = 1;
	return 0;
}

static struct rng_alg bflb_seceng_trng_alg = {
	.generate	= bflb_seceng_trng_crypto_generate,
	.seed		= bflb_seceng_trng_crypto_seed,
	.seedsize	= 0,
	.base		= {
		.cra_name		    = "stdrng",
		.cra_driver_name	= "bflb-seceng",
		.cra_flags		    = CRYPTO_ALG_TYPE_RNG,
		.cra_priority		= 300,
		.cra_ctxsize		= sizeof(struct bflb_seceng_ctx),
		.cra_module		    = THIS_MODULE,
		.cra_init		    = bflb_seceng_trng_crypto_init,
	}
};

static struct hwrng bflb_hwrng = {
	.name	= "bflb-seceng",
	.init	= bflb_seceng_trng_hwrng_init,
	.read	= bflb_seceng_trng_hwrng_read,
};

static int bflb_seceng_probe(struct platform_device *pdev)
{
	struct bflb_seceng *seceng;
	int ret;

	seceng = devm_kzalloc(&pdev->dev, sizeof(*seceng), GFP_KERNEL);

	if (!seceng)
		return -ENOMEM;

	seceng->dev = &pdev->dev;

	platform_set_drvdata(pdev, seceng);
	mutex_init(&seceng->lock);

	seceng->base = devm_platform_ioremap_resource(pdev, 0);

	if (IS_ERR(seceng->base))
		return PTR_ERR(seceng->base);

	seceng->map = devm_regmap_init_mmio(&pdev->dev, seceng->base,
		&bflb_seceng_regmap_config);

	if (IS_ERR(seceng->map))
		return dev_err_probe(&pdev->dev, PTR_ERR(seceng->map),
			"Failed to create regmap\n");

	bflb_seceng_dev = seceng; //Assign driver static

	ret = crypto_register_rng(&bflb_seceng_trng_alg);

	if (ret)
		dev_err_probe(&pdev->dev, ret,
		"Failed to register as a crypto random number generator\n");

	seceng->hwrng = bflb_hwrng;

	ret = hwrng_register(&seceng->hwrng);

	if (ret)
		dev_err_probe(&pdev->dev, ret,
			"Failed to register as a hardware random number generator\n");

	dev_info(&pdev->dev, "Bouffalo Lab Secure Engine");

	return ret;
}

static int bflb_seceng_remove(struct platform_device *pdev)
{
	hwrng_unregister(&bflb_seceng_dev->hwrng);
	crypto_unregister_rng(&bflb_seceng_trng_alg);

	bflb_seceng_dev = NULL;

	return 0;
}

static const struct of_device_id __maybe_unused bflb_seceng_of_match[] = {
	{ .compatible = "bflb,seceng", .data = (const void *)0, },
	{}
};
MODULE_DEVICE_TABLE(of, bflb_seceng_of_match);

static struct platform_driver bflb_seceng_driver = {
	.probe  = bflb_seceng_probe,
	.remove =  bflb_seceng_remove,
	.driver = {
		.name = "bflb-seceng",
		.of_match_table = bflb_seceng_of_match,
		.suppress_bind_attrs = true,
	}
};
module_platform_driver(bflb_seceng_driver);

MODULE_DESCRIPTION("Bouffalo BL808 Secure Engine driver");
MODULE_AUTHOR("Alexander Horner <contact@alexhorner.cc>");
MODULE_LICENSE("GPL v2");
