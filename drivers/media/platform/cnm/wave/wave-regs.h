// SPDX-License-Identifier: (GPL-2.0+ OR MIT)

#ifndef __WAVE_REGS__
#define __WAVE_REGS__

/** Control registers **/

/* Power On Configuration
 * USE_PO_CONF      [3]     1 - Use Power-On-Configuration
 * PO_DEBUG_MODE    [0]     1 - Power On with debug mode
 *
 * Host processor should set 0 for USE_PO_CONF field which is required when
 * VPU initialization.
 */
#define WAVE_REG_POWERON_CONF		0x0000
#define   WAVE_USE_PO_CONF		BIT(3)
#define   WAVE_DEBUG_MODE		BIT(0)
/* Current PC value for check init */
#define WAVE_REG_VCPU_CUR_PC		0x0004
/* Current PC value */
#define WAVE_REG_VCPU_CUR_LR		0x0008

/* V-CPU interrupt mask for STEP debug mode */
#define WAVE_REG_PDBG_STEP_MASK		0x000c
/* V-CPU debug ctrl register */
#define WAVE_REG_PDBG_CTRL			0x0010
/* V-CPU debug index register */
#define WAVE_REG_PDBG_IDX			0x0014
/* V-CPU debug write data register */
#define WAVE_REG_PDBG_WDATA			0x0018
/* V-CPU debug read data register */
#define WAVE_REG_PDBG_RDATA			0x001c

/* Fast IO ctrl register */
#define WAVE_REG_FIO_CTRL			0x0020

#define WAVE_REG_FIO_DATA			0x0024

#define WAVE_REG_RESV_R28			0x0028

#define WAVE_REG_RESV_R2C			0x002c

/* VPU interrupt status */
#define WAVE_REG_VINT_REASON_USR	0x0030
/* VPU interrupt reason clear */
#define WAVE_REG_VINT_REASON_CLR	0x0034
/* Interrupt request sent from host to VPU for the command */
#define WAVE_REG_HOST_INT_REQ		0x0038
/* host清除向vpu发送的处于pending状态的中断请求 */
#define WAVE_REG_VINT_CLEAR			0x003c
/* vpu清除host发起的中断请求 */
#define WAVE_REG_HINT_CLEAR			0x0040
/* vpu interrupt status */
#define WAVE_REG_VINT_STAT			0x0044

/* interrupt enable */
#define WAVE_REG_VINT_ENABLE		0x0048
/* interrupt reason */
#define WAVE_REG_VINT_REASON		0x004c


/* VPU reset request for each clock domain */
#define WAVE_REG_RESET_REQ			0x0050
#define   WAVE_RST_CCLK(_core)	(1 << _core)
#define   WAVE_RST_CCLK_ALL		(0xff)
#define   WAVE_RST_BCLK(_core)	(0x100 << _core)
#define   WAVE_RST_BCLK_ALL		(0xff00)
#define   WAVE_RST_ACLK(_core)	(0x10000 << _core)
#define   WAVE_RST_ACLK_ALL		(0xff0000)
#define   WAVE_RST_VCPU			(0x1000000)
#define   WAVE_RST_VCLK			(0x2000000)
#define   WAVE_RST_MCLK			(0x4000000)
#define   WAVE_RST_ALL			(0xfffffff)

/* VPU reset status for each clock domain */
#define WAVE_REG_RESET_STATUS		0x0054

/* V-CPU restart request */
#define WAVE_REG_VCPU_RESTART		0x0058

/* Clock Gating control */
#define WAVE_REG_CLK_MASK			0x005c
#define   WAVE_CCLK_EN
#define   WAVE_BCLK_EN
#define   WAVE_ACLK_EN
#define   WAVE_MCLK_EN

/*
 * uses between the physical memory and virtual memory for efficient address
 * management.
 *
 * VPU works with the virtual memory addresses that are translated to the
 * phsyical addresses.
 * Host processor needs to assign V-CPU code buffer to VPU_REMAP_PADDR and
 * VPU_REMAP_VADDR to 0, so that VPU can start by reading from VPU_REMAP_PADDR
 * considering it is its the base address 0.
 */

/* REMAP_CTRL TODO(wave420 != wave521)
 * PAGE SIZE:   [8:0]   0x001 - 4K
 *                      0x002 - 8K
 *                      0x004 - 16K
 *                      ...
 *                      0x100 - 1M
 * REGION ATTR1 [10]    0     - Normal
 *                      1     - Make Bus error for the region
 * REGION ATTR2 [11]    0     - Normal
 *                      1     - Bypass region
 * REMAP INDEX  [15:12]       - 0 ~ 3 , 0 - Code, 1-Stack
 * ENDIAN       [19:16]       - See EndianMode in vdi.h
 * AXI-ID       [23:20]       - Upper AXI-ID
 * BUS_ERROR    [29]    0     - bypass
 *                      1     - Make BUS_ERROR for unmapped region
 * BYPASS_ALL   [30]    1     - Bypass all
 * ENABLE       [31]    1     - Update control register[30:16]
 */
#define WAVE_REG_REMAP_CTRL			0x0060

/* remap region base addr in virtual address space */
#define WAVE_REG_REMAP_VADDR		0x0064

/* remap region base addr in physical address space */
#define WAVE_REG_REMAP_PADDR		0x0068

#define WAVE_REG_REMAP_CORE_START	0x006c

/*
 * BUSY_STATUS: indicates which side between VPU and Host processor has
 * the ownership of access to host interface registers.
 * 1: VPU has the ownership for access to host interface registers.
 * 0: Host processor has the ownership for access to host interface registers.
 *
 * Host processor must check the ownership through the BUSY_STATUS register
 * prior to sending command and arguments. Host processor can send a command
 * when VPU_BUSY_STATUS is 0.
 *
 * After setting the command arguments, Host processor must set the BUSY_STATUS
 * register to 1 and issue the command to give the access ownership to VPU.
 *
 * When BUSY_STATUS turns 0 again, Host processor can issue another command for
 * secure operation.
 */
#define WAVE_REG_BUSY_STATUS		0x0070 //
#define   WAVE_REG_BUSY_FLAG	1

/* reserved for report vpu status */
#define WAVE_REG_HALT_STATUS		0x0074
/* reserved for vpu status report */
#define WAVE_REG_VCPU_STATUS		0x0078
/* reserved for vpu status report */
#define WAVE_REG_PRESCAN_STATUS		0x007c


#define WAVE_REG_RET_FIO_STATUS		0x0080
#define WAVE_REG_RET_NAME			0x0090
#define WAVE_REG_RET_PRODUCT_VER	0x0094

/*
    vcpu_config0:
    conf_map_converter_reg,      // [31]
    conf_map_converter_sig,      // [30]
    8'd0,                        // [29:22]
    conf_std_switch_en,          // [21]
    conf_bg_detect,              // [20]
    conf_3dnr_en,                // [19]
    conf_one_axi_en,             // [18]
    conf_sec_axi_en,             // [17]
    conf_bus_info,               // [16]
    conf_afbc_en,                // [15]
    conf_afbc_version_id,        // [14:12]
    conf_fbc_en,                 // [11]
    conf_fbc_version_id,         // [10:08]
    conf_scaler_en,              // [07]
    conf_scaler_version_id,      // [06:04]
    conf_bwb_en,                 // [03]
    3'd0                         // [02:00]
*/
#define WAVE_REG_RET_VCPU_CONFIG0	0x0098

/*
    vpu_config1:
    4'd0,                        // [31:28]
    conf_perf_timer_en,          // [27]
    conf_multi_core_en,          // [26]
    conf_gcu_en,                 // [25]
    conf_cu_report,              // [24]
    4'd0,                        // [23:20]
    conf_vcore_id_3,             // [19]
    conf_vcore_id_2,             // [18]
    conf_vcore_id_1,             // [17]
    conf_vcore_id_0,             // [16]
    conf_bwb_opt,                // [15]
    7'd0,                        // [14:08]
    conf_cod_std_en_reserved_7,  // [7]
    conf_cod_std_en_reserved_6,  // [6]
    conf_cod_std_en_reserved_5,  // [5]
    conf_cod_std_en_reserved_4,  // [4]
    conf_cod_std_en_reserved_3,  // [3]
    conf_cod_std_en_reserved_2,  // [2]
    conf_cod_std_en_vp9,         // [1]
    conf_cod_std_en_hevc         // [0]
 */

#define WAVE_REG_RET_VCPU_CONFIG1	0x009c

/* standard definition */
#define WAVE_REG_CODEC_STD			0x00a0
/* configuration date */
#define WAVE_REG_CONF_DATE			0x00a4
/* revision of H/W config */
#define WAVE_REG_RET_CONF_REV		0x00a8
/* The define value of H/W configuration */
#define WAVE_REG_REG_TYPE			0x00ac


/* Command I/O Register: 0x0100-0x01FF */

/* Host can give any command to VPU by setting COMMAND register(0x100) and
 * interrupt request to VPU for the command issued through HOST_INT_REQ
 * register(0x038) to host interface register, Host processor should check
 * the ownership of host interface registers.
 */

/* WAVE series VPU supports command-queue to maximize performance by pipelining
 * internal commands and by hiding wait cycle taken to receive a command from
 * Host processor.
 *
 * VPU has two queues. One is a command-queue for queueing commands from Host
 * processor, and the other is a report-queue for queueing the results of the
 * commands.
 */

/*
 * 0x00000100 RW 0x0 COMMAND Command
 * 0x00000104 RW 0x0 CMD_OPTION Command Option
 * 0x00000110 RW 0x0 CMD_INSTANCE_INFO Instance information OUTPUT RETURN

 * 0x00000108 RW 0x0 RET_SUCCESS Result of the command
 * 0x0000010C RW 0x0 RET_FAIL_REASON Fail reason of the run command
 * 0x000001E0 RW 0x0 RET_QUEUE_STATUS Queued command information
 * 0x000001E4 RW 0x0 RET_BS_EMPTY Bitstream buffer empty flag
 * 0x000001E8 RW 0x0 RET_QUEUED_CMD_DONE Queued command done flag
 * 0x000001EC RW 0x0 RET_SEEK_INSTANCE_INFO A working instance index on seek stage (for inter-nal use only)
 * 0x000001F0 RW 0x0 RET_PARSING_INSTANCE_INFO A working instance index on prescan stage (for in-ternal use only)
 * 0x000001F4 RW 0x0 RET_DECODING_INSTANCE_INFO A working instance index on decoding stage (for internal use only)
 * 0x000001F8 RW 0x0 RET_ENCODING_INSTANCE_INFO A working instance index on packing stage (for in-ternal use only)
 * 0x000001FC RW 0x0 RET_DONE_INSTANCE_INFO Picture command done interrupt instance index
 */

typedef enum {
    INIT_VPU        = 0x0001,
    WAKEUP_VPU      = 0x0002,
    SLEEP_VPU       = 0x0004,
    CREATE_INSTANCE = 0x0008,
    FLUSH_INSTANCE  = 0x0010,
    DESTROY_INSTANCE= 0x0020,
    INIT_SEQ        = 0x0040,
    SET_FB          = 0x0080,
    DEC_PIC         = 0x0100,
    ENC_PIC         = 0x0100,
    ENC_SET_PARAM   = 0x0200,    // 设置编码参数
    QUERY           = 0x4000,
    UPDATE_BS       = 0x8000,
    RESET_VPU       = 0x10000,
    MAX_VPU_CMD     = 0x10000,
} WAVE_CMD;

/*  */
#define WAVE_REG_CMD_BASE			0x0100
#define WAVE_REG_CMD_OPTION			0x0104

/* result of the cmd */
#define WAVE_REG_RET_SUCCESS		0x0108
/* Fail reason of the run command */
#define WAVE_REG_RET_FAIL_REASON	0x010c

#define WAVE_REG_CMD_INSTANCE_INFO	0x0110


/* INIT_VPU
 * 0x00000110 RW 0x0 ADDR_CODE_BASE Code buffer base address
 * 0x00000114 RW 0x0 CODE_SIZE Code buffer size
 * 0x00000118 RW 0x0 CODE_PARAM Code buffer parameters
 * 0x0000011C RW 0x0 CMD_INIT_ADDR_TEMP_BASE Temporal buffer base address
 * 0x00000120 RW 0x0 CMD_INIT_TEMP_SIZE Temporal buffer size

 * 0x00000124 RW 0x0 CMD_INIT_ADDR_SEC_AXI Secondary AXI base address
 * 0x00000128 RW 0x0 CMD_INIT_SEC_AXI_SIZE Seconary AXI memory size
 * 0x0000012C RW 0x0 CMD_INIT_HW_OPTION VPU hardware option
 * 0x00000130 RW 0x0 CMD_WAKEUP_SYSTEM_CLOCK Time out count
 * 0x00000134 RW 0x0 CMD_INIT_NUM_TASK_BUF Number of task buffer
 * 0x00000138 RW 0x0 CMD_INIT_ADDR_TASK_BUF0 Base address of task buffer 0
 * 0x0000013C RW 0x0 CMD_INIT_ADDR_TASK_BUF1 Base address of task buffer 1
 * 0x00000140 RW 0x0 CMD_INIT_ADDR_TASK_BUF2 Base address of task buffer 2
 * 0x00000144 RW 0x0 CMD_INIT_ADDR_TASK_BUF3 Base address of task buffer 3
 * 0x00000148 RW 0x0 CMD_INIT_ADDR_TASK_BUF4 Base address of task buffer 4
 * 0x0000014C RW 0x0 CMD_INIT_ADDR_TASK_BUF5 Base address of task buffer 5
 * 0x00000150 RW 0x0 CMD_INIT_ADDR_TASK_BUF6 Base address of task buffer 6
 * 0x00000154 RW 0x0 CMD_INIT_ADDR_TASK_BUF7 Base address of task buffer 7
 * 0x00000158 RW 0x0 CMD_INIT_ADDR_TASK_BUF8 Base address of task buffer 8
 * 0x0000015C RW 0x0 CMD_INIT_ADDR_TASK_BUF9 Base address of task buffer 9
 * 0x00000160 RW 0x0 CMD_INIT_ADDR_TASK_BUFA Base address of task buffer A
 * 0x00000164 RW 0x0 CMD_INIT_ADDR_TASK_BUFB Base address of task buffer B
 * 0x00000168 RW 0x0 CMD_INIT_ADDR_TASK_BUFC Base address of task buffer C
 * 0x0000016C RW 0x0 CMD_INIT_ADDR_TASK_BUFD Base address of task buffer D
 * 0x00000170 RW 0x0 CMD_INIT_ADDR_TASK_BUFE Base address of task buffer E
 * 0x00000174 RW 0x0 CMD_INIT_ADDR_TASK_BUFF Base address of task buffer F
 * 0x00000178 RW 0x0 CMD_INIT_TASK_BUFF_SIZE Size of task buffer
 */


/* INIT_VPU parameter registers */
/* Note: WAVE_REG_ADDR_CODE_BASE should be aligned to 4KB */
#define WAVE_REG_ADDR_CODE_BASE                       0x0110
#define WAVE_REG_CODE_SIZE                            0x0114
#define WAVE_REG_CODE_PARAM                           0x0118
#define WAVE_REG_ADDR_TEMP_BASE                       0x011c
#define WAVE_REG_TEMP_SIZE                            0x0120
#define WAVE_REG_ADDR_SEC_AXI                         0x0124
#define WAVE_REG_SEC_AXI_SIZE                         0x0128
#define WAVE_REG_HW_OPTION                            0x012c
#define WAVE_REG_TIMEOUT_CNT                          0x0130


// FIXME
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_5		0x0134
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_0		0x0138
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_1		0x013c
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_2		0x0140
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_3		0x0144
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_4		0x0148
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_5		0x014c
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_6		0x0150
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_7		0x0154
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_8		0x0158
#define WAVE_REG_CMD_ENC_CUSTOM_GOP_PIC_PARAM_9		0x015c



#define WAVE_REG_CMD_INIT_AXI_PARAM                   0x017c


// common parameter register
//

/************************************************************************/
/* ENCODER - ENC_SET_PARAM (CUSTOM_GOP)                                 */
/************************************************************************/

#define WAVE_REG_RET_QUEUE_STATUS                     0x01E0
#define WAVE_REG_RET_BS_EMPTY_INST                    0x01E4
#define WAVE_REG_RET_QUEUE_CMD_DONE_INST              0x01E8
#define WAVE_REG_RET_STAGE0_INSTANCE_INFO             0x01EC
#define WAVE_REG_RET_STAGE1_INSTANCE_INFO             0x01F0
#define WAVE_REG_RET_STAGE2_INSTANCE_INFO             0x01F4

#define WAVE_REG_RET_DONE_INSTANCE_INFO               0x01FC


/* Product information registers */
#define WAVE_REG_PRODUCT_NAME						0x1040
#define WAVE_REG_PRODUCT_NUMBER						0x1044 // chip id


#endif /* __WAVE_REGS__ */
