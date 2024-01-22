/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright 2023 Solidigm All Rights Reserved
 */

#include "ftl_nvc_dev.h"
#include "ftl_core.h"
#include "ftl_layout.h"
#include "utils/ftl_layout_tracker_bdev.h"

static bool
is_bdev_compatible(struct spdk_ftl_dev *dev, struct spdk_bdev *bdev)
{
	if (!spdk_bdev_is_md_separate(bdev)) {
		/* It doesn't support separate metadata buffer IO */
		return false;
	}

	if (spdk_bdev_get_md_size(bdev) != sizeof(union ftl_md_vss)) {
		/* Bdev's metadata is invalid size */
		return false;
	}

	if (spdk_bdev_get_dif_type(bdev) != SPDK_DIF_DISABLE) {
		/* Unsupported DIF type used by bdev */
		return false;
	}

	if (ftl_md_xfer_blocks(dev) * spdk_bdev_get_md_size(bdev) > FTL_ZERO_BUFFER_SIZE) {
		FTL_ERRLOG(dev, "Zero buffer too small for bdev %s metadata transfer\n",
			   spdk_bdev_get_name(bdev));
		return false;
	}

	return true;
}

static bool
is_chunk_active(struct spdk_ftl_dev *dev, struct ftl_nv_cache_chunk *chunk)
{
	struct ftl_layout_region *region;
	uint64_t chunk_begin = chunk->offset;
	uint64_t chunk_end = chunk_begin + dev->layout.nvc.chunk_data_blocks - 1;
	uint64_t region_begin, region_end;

	for (uint64_t i = 0; i < FTL_LAYOUT_REGION_TYPE_MAX; i++) {
		if (i == FTL_LAYOUT_REGION_TYPE_DATA_BASE ||
		    i == FTL_LAYOUT_REGION_TYPE_SB_BASE ||
		    i == FTL_LAYOUT_REGION_TYPE_VALID_MAP ||
		    i == FTL_LAYOUT_REGION_TYPE_DATA_NVC) {
			continue;
		}

		region = ftl_layout_region_get(dev, i);
		if (!region) {
			continue;
		}

		region_begin = region->current.offset;
		region_end = region_begin + region->current.blocks - 1;

		if (spdk_max(chunk_begin, region_begin) <= spdk_min(chunk_end, region_end)) {
			return false;
		}
	}

	return true;
}

static void
md_region_setup(struct spdk_ftl_dev *dev, enum ftl_layout_region_type reg_type,
		struct ftl_layout_region *region)
{
	assert(region);
	region->type = reg_type;
	region->mirror_type = FTL_LAYOUT_REGION_TYPE_INVALID;
	region->name = ftl_md_region_name(reg_type);

	region->bdev_desc = dev->nv_cache.bdev_desc;
	region->ioch = dev->nv_cache.cache_ioch;
	region->vss_blksz = dev->nv_cache.md_size;
}

static int
md_region_create(struct spdk_ftl_dev *dev, enum ftl_layout_region_type reg_type,
		 uint32_t reg_version, size_t reg_blks)
{
	const struct ftl_layout_tracker_bdev_region_props *reg_props;

	assert(reg_type < FTL_LAYOUT_REGION_TYPE_MAX);
	reg_blks = ftl_md_region_align_blocks(dev, reg_blks);

	reg_props = ftl_layout_tracker_bdev_add_region(dev->nvc_layout_tracker, reg_type, reg_version,
			reg_blks, 0);
	if (!reg_props) {
		return -1;
	}
	assert(reg_props->type == reg_type);
	assert(reg_props->ver == reg_version);
	assert(reg_props->blk_sz == reg_blks);
	assert(reg_props->blk_offs + reg_blks <= dev->layout.nvc.total_blocks);
	return 0;
}

static int
md_region_open(struct spdk_ftl_dev *dev, enum ftl_layout_region_type reg_type, uint32_t reg_version,
	       size_t entry_size, size_t entry_count, struct ftl_layout_region *region)
{
	const struct ftl_layout_tracker_bdev_region_props *reg_search_ctx = NULL;
	uint64_t reg_blks = ftl_md_region_blocks(dev, entry_size * entry_count);

	assert(reg_type < FTL_LAYOUT_REGION_TYPE_MAX);

	while (true) {
		ftl_layout_tracker_bdev_find_next_region(dev->nvc_layout_tracker, reg_type, &reg_search_ctx);
		if (!reg_search_ctx || reg_search_ctx->ver == reg_version) {
			break;
		}
	}

	if (!reg_search_ctx || reg_search_ctx->blk_sz < reg_blks) {
		/* Region not found or insufficient space */
		return -1;
	}

	if (!region) {
		return 0;
	}

	md_region_setup(dev, reg_type, region);

	region->entry_size = entry_size / FTL_BLOCK_SIZE;
	region->num_entries = entry_count;

	region->current.version = reg_version;
	region->current.offset = reg_search_ctx->blk_offs;
	region->current.blocks = reg_search_ctx->blk_sz;

	return 0;
}

struct ftl_nv_cache_device_type nvc_bdev_vss = {
	.name = "bdev",
	.features = {
	},
	.ops = {
		.is_bdev_compatible = is_bdev_compatible,
		.is_chunk_active = is_chunk_active,
		.md_layout_ops = {
			.region_create = md_region_create,
			.region_open = md_region_open,
		},
	}
};
FTL_NV_CACHE_DEVICE_TYPE_REGISTER(nvc_bdev_vss)
