#include "spdk/hisi_spec.h"
#include "hisi_internal.h"
#include "spdk/log.h"
#include <stdio.h>
#include <string.h>

#define LOG_BUF_SIZE 4096

static void append_to_log_buf(char *log_buf, size_t *log_buf_offset,
                              const char *format, ...) {
    va_list args;
    va_start(args, format);
    int ret = vsnprintf(log_buf + *log_buf_offset,
                        LOG_BUF_SIZE - *log_buf_offset,
                        format, args);
    va_end(args);
    if (ret > 0) {
        *log_buf_offset += (size_t)ret;
        if (*log_buf_offset >= LOG_BUF_SIZE) {
            *log_buf_offset = LOG_BUF_SIZE - 1;
        }
    }
}

static void
hisi_dma_dump_range(struct hisi_dma_dev *hw, uint32_t start, uint32_t end,
                    char *log_buf, size_t *log_buf_offset) {
    #define DUMP_REGNUM_PER_LINE 4
    uint32_t cnt = 0;

    for (uint32_t i = start; i <= end; i += sizeof(uint32_t)) {
        if (cnt % DUMP_REGNUM_PER_LINE == 0) {
            append_to_log_buf(log_buf, log_buf_offset, "      [%4x]:", i);
        }
        append_to_log_buf(log_buf, log_buf_offset, " 0x%08x", hisi_dma_read_dev(hw, i));
        cnt++;
        if (cnt % DUMP_REGNUM_PER_LINE == 0) {
            append_to_log_buf(log_buf, log_buf_offset, "\n");
        }
    }
    if (cnt % DUMP_REGNUM_PER_LINE) {
        append_to_log_buf(log_buf, log_buf_offset, "\n");
    }
}

static void
hisi_dma_dump_common(struct hisi_dma_dev *hw, char *log_buf, size_t *log_buf_offset) {
    struct {
        uint8_t reg_layout;
        uint32_t start;
        uint32_t end;
    } reg_info[] = {
        { HISI_DMA_REG_LAYOUT_HIP08, HISI_DMA_HIP08_DUMP_START_REG, HISI_DMA_HIP08_DUMP_END_REG },
        { HISI_DMA_REG_LAYOUT_HIP09, HISI_DMA_HIP09_DUMP_REGION_A_START_REG, HISI_DMA_HIP09_DUMP_REGION_A_END_REG },
        { HISI_DMA_REG_LAYOUT_HIP09, HISI_DMA_HIP09_DUMP_REGION_B_START_REG, HISI_DMA_HIP09_DUMP_REGION_B_END_REG },
        { HISI_DMA_REG_LAYOUT_HIP09, HISI_DMA_HIP09_DUMP_REGION_C_START_REG, HISI_DMA_HIP09_DUMP_REGION_C_END_REG },
        { HISI_DMA_REG_LAYOUT_HIP09, HISI_DMA_HIP09_DUMP_REGION_D_START_REG, HISI_DMA_HIP09_DUMP_REGION_D_END_REG },
    };

    append_to_log_buf(log_buf, log_buf_offset, "    common-register:\n");
    for (size_t i = 0; i < sizeof(reg_info) / sizeof(reg_info[0]); i++) {
        if (hw->reg_layout != reg_info[i].reg_layout) {
            continue;
        }
        hisi_dma_dump_range(hw, reg_info[i].start, reg_info[i].end, log_buf, log_buf_offset);
    }
}

static void
hisi_dma_dump_read_queue(struct hisi_dma_dev *hw, uint32_t qoff,
                         char *buffer, int max_sz) {
    memset(buffer, 0, max_sz);

    if (qoff == HISI_DMA_QUEUE_SQ_BASE_L_REG ||
        qoff == HISI_DMA_QUEUE_SQ_BASE_H_REG ||
        qoff == HISI_DMA_QUEUE_CQ_BASE_L_REG ||
        qoff == HISI_DMA_QUEUE_CQ_BASE_H_REG) {
        snprintf(buffer, max_sz, "**********");
    } else {
        snprintf(buffer, max_sz, "0x%08x", hisi_dma_read_queue(hw, qoff));
    }
}

static void
hisi_dma_dump_queue(struct hisi_dma_dev *hw, char *log_buf, size_t *log_buf_offset) {
    #define REG_FMT_LEN 32
    char buf[REG_FMT_LEN] = {0};

    append_to_log_buf(log_buf, log_buf_offset, "    queue-register:\n");
    for (uint32_t i = 0; i < HISI_DMA_QUEUE_REGION_SIZE;) {
        hisi_dma_dump_read_queue(hw, i, buf, sizeof(buf));
        append_to_log_buf(log_buf, log_buf_offset, "      [%2x]: %s", i, buf);
        i += sizeof(uint32_t);
        hisi_dma_dump_read_queue(hw, i, buf, sizeof(buf));
        append_to_log_buf(log_buf, log_buf_offset, " %s", buf);
        i += sizeof(uint32_t);
        hisi_dma_dump_read_queue(hw, i, buf, sizeof(buf));
        append_to_log_buf(log_buf, log_buf_offset, " %s", buf);
        i += sizeof(uint32_t);
        hisi_dma_dump_read_queue(hw, i, buf, sizeof(buf));
        append_to_log_buf(log_buf, log_buf_offset, " %s\n", buf);
        i += sizeof(uint32_t);
    }
}

int hisi_dma_dump(struct hisi_dma_dev *hw) {
	char log_buf[LOG_BUF_SIZE] = {0};
	size_t log_buf_offset = 0;

    append_to_log_buf(log_buf, &log_buf_offset,
        "    revision: 0x%x queue_id: %u ring_size: %u\n"
        "    sq_head: %u sq_tail: %u cq_sq_head: %u\n"
        "    cq_head: %u cqs_completed: %u cqe_vld: %u\n"
        "    submitted: %" PRIu64 " completed: %" PRIu64 " errors: %"
        PRIu64 " qfulls: %" PRIu64 "\n",
        hw->revision, hw->queue_id,
        hw->sq_depth_mask > 0 ? hw->sq_depth_mask + 1 : 0,
        hw->sq_head, hw->sq_tail, hw->cq_sq_head,
        hw->cq_head, hw->cqs_completed, hw->cqe_vld,
        hw->submitted, hw->completed, hw->errors, hw->qfulls);

    hisi_dma_dump_queue(hw, log_buf, &log_buf_offset);
    hisi_dma_dump_common(hw, log_buf, &log_buf_offset);

	SPDK_NOTICELOG("%s\n", log_buf);
    return 0;
}
