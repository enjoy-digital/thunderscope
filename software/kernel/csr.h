//--------------------------------------------------------------------------------
// Auto-generated by LiteX (a4ead5ca) on 2023-12-12 13:24:10
//--------------------------------------------------------------------------------
#ifndef __GENERATED_CSR_H
#define __GENERATED_CSR_H

#ifndef CSR_BASE
#define CSR_BASE 0x0L
#endif

/* adc */
#define CSR_ADC_BASE (CSR_BASE + 0x0L)
#define CSR_ADC_CONTROL_ADDR (CSR_BASE + 0x0L)
#define CSR_ADC_CONTROL_SIZE 1
#define CSR_ADC_CONTROL_ACQ_EN_OFFSET 0
#define CSR_ADC_CONTROL_ACQ_EN_SIZE 1
#define CSR_ADC_CONTROL_OSC_EN_OFFSET 1
#define CSR_ADC_CONTROL_OSC_EN_SIZE 1
#define CSR_ADC_CONTROL_RST_OFFSET 2
#define CSR_ADC_CONTROL_RST_SIZE 1
#define CSR_ADC_CONTROL_PWR_DOWN_OFFSET 3
#define CSR_ADC_CONTROL_PWR_DOWN_SIZE 1
#define CSR_ADC_TRIGGER_CONTROL_ADDR (CSR_BASE + 0x4L)
#define CSR_ADC_TRIGGER_CONTROL_SIZE 1
#define CSR_ADC_TRIGGER_CONTROL_ENABLE_OFFSET 0
#define CSR_ADC_TRIGGER_CONTROL_ENABLE_SIZE 1
#define CSR_ADC_HAD1511_CONTROL_ADDR (CSR_BASE + 0x8L)
#define CSR_ADC_HAD1511_CONTROL_SIZE 1
#define CSR_ADC_HAD1511_CONTROL_FRAME_RST_OFFSET 0
#define CSR_ADC_HAD1511_CONTROL_FRAME_RST_SIZE 1
#define CSR_ADC_HAD1511_CONTROL_DELAY_RST_OFFSET 1
#define CSR_ADC_HAD1511_CONTROL_DELAY_RST_SIZE 1
#define CSR_ADC_HAD1511_CONTROL_DELAY_INC_OFFSET 2
#define CSR_ADC_HAD1511_CONTROL_DELAY_INC_SIZE 1
#define CSR_ADC_HAD1511_CONTROL_STAT_RST_OFFSET 3
#define CSR_ADC_HAD1511_CONTROL_STAT_RST_SIZE 1
#define CSR_ADC_HAD1511_STATUS_ADDR (CSR_BASE + 0xcL)
#define CSR_ADC_HAD1511_STATUS_SIZE 1
#define CSR_ADC_HAD1511_DOWNSAMPLING_ADDR (CSR_BASE + 0x10L)
#define CSR_ADC_HAD1511_DOWNSAMPLING_SIZE 1
#define CSR_ADC_HAD1511_RANGE_ADDR (CSR_BASE + 0x14L)
#define CSR_ADC_HAD1511_RANGE_SIZE 1
#define CSR_ADC_HAD1511_RANGE_MIN01_OFFSET 0
#define CSR_ADC_HAD1511_RANGE_MIN01_SIZE 8
#define CSR_ADC_HAD1511_RANGE_MAX01_OFFSET 8
#define CSR_ADC_HAD1511_RANGE_MAX01_SIZE 8
#define CSR_ADC_HAD1511_RANGE_MIN23_OFFSET 16
#define CSR_ADC_HAD1511_RANGE_MIN23_SIZE 8
#define CSR_ADC_HAD1511_RANGE_MAX23_OFFSET 24
#define CSR_ADC_HAD1511_RANGE_MAX23_SIZE 8
#define CSR_ADC_HAD1511_BITSLIP_COUNT_ADDR (CSR_BASE + 0x18L)
#define CSR_ADC_HAD1511_BITSLIP_COUNT_SIZE 1
#define CSR_ADC_HAD1511_SAMPLE_COUNT_ADDR (CSR_BASE + 0x1cL)
#define CSR_ADC_HAD1511_SAMPLE_COUNT_SIZE 1

/* analyzer */
#define CSR_ANALYZER_BASE (CSR_BASE + 0x800L)
#define CSR_ANALYZER_MUX_VALUE_ADDR (CSR_BASE + 0x800L)
#define CSR_ANALYZER_MUX_VALUE_SIZE 1
#define CSR_ANALYZER_TRIGGER_ENABLE_ADDR (CSR_BASE + 0x804L)
#define CSR_ANALYZER_TRIGGER_ENABLE_SIZE 1
#define CSR_ANALYZER_TRIGGER_DONE_ADDR (CSR_BASE + 0x808L)
#define CSR_ANALYZER_TRIGGER_DONE_SIZE 1
#define CSR_ANALYZER_TRIGGER_MEM_WRITE_ADDR (CSR_BASE + 0x80cL)
#define CSR_ANALYZER_TRIGGER_MEM_WRITE_SIZE 1
#define CSR_ANALYZER_TRIGGER_MEM_MASK_ADDR (CSR_BASE + 0x810L)
#define CSR_ANALYZER_TRIGGER_MEM_MASK_SIZE 5
#define CSR_ANALYZER_TRIGGER_MEM_VALUE_ADDR (CSR_BASE + 0x824L)
#define CSR_ANALYZER_TRIGGER_MEM_VALUE_SIZE 5
#define CSR_ANALYZER_TRIGGER_MEM_FULL_ADDR (CSR_BASE + 0x838L)
#define CSR_ANALYZER_TRIGGER_MEM_FULL_SIZE 1
#define CSR_ANALYZER_SUBSAMPLER_VALUE_ADDR (CSR_BASE + 0x83cL)
#define CSR_ANALYZER_SUBSAMPLER_VALUE_SIZE 1
#define CSR_ANALYZER_STORAGE_ENABLE_ADDR (CSR_BASE + 0x840L)
#define CSR_ANALYZER_STORAGE_ENABLE_SIZE 1
#define CSR_ANALYZER_STORAGE_DONE_ADDR (CSR_BASE + 0x844L)
#define CSR_ANALYZER_STORAGE_DONE_SIZE 1
#define CSR_ANALYZER_STORAGE_LENGTH_ADDR (CSR_BASE + 0x848L)
#define CSR_ANALYZER_STORAGE_LENGTH_SIZE 1
#define CSR_ANALYZER_STORAGE_OFFSET_ADDR (CSR_BASE + 0x84cL)
#define CSR_ANALYZER_STORAGE_OFFSET_SIZE 1
#define CSR_ANALYZER_STORAGE_MEM_LEVEL_ADDR (CSR_BASE + 0x850L)
#define CSR_ANALYZER_STORAGE_MEM_LEVEL_SIZE 1
#define CSR_ANALYZER_STORAGE_MEM_DATA_ADDR (CSR_BASE + 0x854L)
#define CSR_ANALYZER_STORAGE_MEM_DATA_SIZE 1

/* ctrl */
#define CSR_CTRL_BASE (CSR_BASE + 0x1000L)
#define CSR_CTRL_RESET_ADDR (CSR_BASE + 0x1000L)
#define CSR_CTRL_RESET_SIZE 1
#define CSR_CTRL_RESET_SOC_RST_OFFSET 0
#define CSR_CTRL_RESET_SOC_RST_SIZE 1
#define CSR_CTRL_RESET_CPU_RST_OFFSET 1
#define CSR_CTRL_RESET_CPU_RST_SIZE 1
#define CSR_CTRL_SCRATCH_ADDR (CSR_BASE + 0x1004L)
#define CSR_CTRL_SCRATCH_SIZE 1
#define CSR_CTRL_BUS_ERRORS_ADDR (CSR_BASE + 0x1008L)
#define CSR_CTRL_BUS_ERRORS_SIZE 1

/* dna */
#define CSR_DNA_BASE (CSR_BASE + 0x1800L)
#define CSR_DNA_ID_ADDR (CSR_BASE + 0x1800L)
#define CSR_DNA_ID_SIZE 2

/* frontend */
#define CSR_FRONTEND_BASE (CSR_BASE + 0x2000L)
#define CSR_FRONTEND_CONTROL_ADDR (CSR_BASE + 0x2000L)
#define CSR_FRONTEND_CONTROL_SIZE 1
#define CSR_FRONTEND_CONTROL_FE_EN_OFFSET 0
#define CSR_FRONTEND_CONTROL_FE_EN_SIZE 1
#define CSR_FRONTEND_CONTROL_COUPLING_OFFSET 8
#define CSR_FRONTEND_CONTROL_COUPLING_SIZE 4
#define CSR_FRONTEND_CONTROL_ATTENUATION_OFFSET 16
#define CSR_FRONTEND_CONTROL_ATTENUATION_SIZE 4
#define CSR_FRONTEND_SPI_CONTROL_ADDR (CSR_BASE + 0x2004L)
#define CSR_FRONTEND_SPI_CONTROL_SIZE 1
#define CSR_FRONTEND_SPI_CONTROL_START_OFFSET 0
#define CSR_FRONTEND_SPI_CONTROL_START_SIZE 1
#define CSR_FRONTEND_SPI_CONTROL_LENGTH_OFFSET 8
#define CSR_FRONTEND_SPI_CONTROL_LENGTH_SIZE 8
#define CSR_FRONTEND_SPI_STATUS_ADDR (CSR_BASE + 0x2008L)
#define CSR_FRONTEND_SPI_STATUS_SIZE 1
#define CSR_FRONTEND_SPI_STATUS_DONE_OFFSET 0
#define CSR_FRONTEND_SPI_STATUS_DONE_SIZE 1
#define CSR_FRONTEND_SPI_STATUS_MODE_OFFSET 1
#define CSR_FRONTEND_SPI_STATUS_MODE_SIZE 1
#define CSR_FRONTEND_SPI_MOSI_ADDR (CSR_BASE + 0x200cL)
#define CSR_FRONTEND_SPI_MOSI_SIZE 1
#define CSR_FRONTEND_SPI_MISO_ADDR (CSR_BASE + 0x2010L)
#define CSR_FRONTEND_SPI_MISO_SIZE 1
#define CSR_FRONTEND_SPI_CS_ADDR (CSR_BASE + 0x2014L)
#define CSR_FRONTEND_SPI_CS_SIZE 1
#define CSR_FRONTEND_SPI_CS_SEL_OFFSET 0
#define CSR_FRONTEND_SPI_CS_SEL_SIZE 5
#define CSR_FRONTEND_SPI_CS_MODE_OFFSET 16
#define CSR_FRONTEND_SPI_CS_MODE_SIZE 1
#define CSR_FRONTEND_SPI_LOOPBACK_ADDR (CSR_BASE + 0x2018L)
#define CSR_FRONTEND_SPI_LOOPBACK_SIZE 1
#define CSR_FRONTEND_SPI_LOOPBACK_MODE_OFFSET 0
#define CSR_FRONTEND_SPI_LOOPBACK_MODE_SIZE 1

/* i2c */
#define CSR_I2C_BASE (CSR_BASE + 0x2800L)
#define CSR_I2C_W_ADDR (CSR_BASE + 0x2800L)
#define CSR_I2C_W_SIZE 1
#define CSR_I2C_W_SCL_OFFSET 0
#define CSR_I2C_W_SCL_SIZE 1
#define CSR_I2C_W_OE_OFFSET 1
#define CSR_I2C_W_OE_SIZE 1
#define CSR_I2C_W_SDA_OFFSET 2
#define CSR_I2C_W_SDA_SIZE 1
#define CSR_I2C_R_ADDR (CSR_BASE + 0x2804L)
#define CSR_I2C_R_SIZE 1
#define CSR_I2C_R_SDA_OFFSET 0
#define CSR_I2C_R_SDA_SIZE 1

/* icap */
#define CSR_ICAP_BASE (CSR_BASE + 0x3000L)
#define CSR_ICAP_ADDR_ADDR (CSR_BASE + 0x3000L)
#define CSR_ICAP_ADDR_SIZE 1
#define CSR_ICAP_DATA_ADDR (CSR_BASE + 0x3004L)
#define CSR_ICAP_DATA_SIZE 1
#define CSR_ICAP_WRITE_ADDR (CSR_BASE + 0x3008L)
#define CSR_ICAP_WRITE_SIZE 1
#define CSR_ICAP_DONE_ADDR (CSR_BASE + 0x300cL)
#define CSR_ICAP_DONE_SIZE 1
#define CSR_ICAP_READ_ADDR (CSR_BASE + 0x3010L)
#define CSR_ICAP_READ_SIZE 1

/* identifier_mem */
#define CSR_IDENTIFIER_MEM_BASE (CSR_BASE + 0x3800L)

/* leds */
#define CSR_LEDS_BASE (CSR_BASE + 0x4000L)
#define CSR_LEDS_OUT_ADDR (CSR_BASE + 0x4000L)
#define CSR_LEDS_OUT_SIZE 1
#define CSR_LEDS_PWM_ENABLE_ADDR (CSR_BASE + 0x4004L)
#define CSR_LEDS_PWM_ENABLE_SIZE 1
#define CSR_LEDS_PWM_WIDTH_ADDR (CSR_BASE + 0x4008L)
#define CSR_LEDS_PWM_WIDTH_SIZE 1
#define CSR_LEDS_PWM_PERIOD_ADDR (CSR_BASE + 0x400cL)
#define CSR_LEDS_PWM_PERIOD_SIZE 1

/* pcie_dma0 */
#define CSR_PCIE_DMA0_BASE (CSR_BASE + 0x4800L)
#define CSR_PCIE_DMA0_WRITER_ENABLE_ADDR (CSR_BASE + 0x4800L)
#define CSR_PCIE_DMA0_WRITER_ENABLE_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_ADDR (CSR_BASE + 0x4804L)
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_SIZE 2
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_ADDRESS_LSB_OFFSET 0
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_ADDRESS_LSB_SIZE 32
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_LENGTH_OFFSET 32
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_LENGTH_SIZE 24
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_IRQ_DISABLE_OFFSET 56
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_IRQ_DISABLE_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_LAST_DISABLE_OFFSET 57
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_LAST_DISABLE_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_WE_ADDR (CSR_BASE + 0x480cL)
#define CSR_PCIE_DMA0_WRITER_TABLE_WE_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_WE_ADDRESS_MSB_OFFSET 0
#define CSR_PCIE_DMA0_WRITER_TABLE_WE_ADDRESS_MSB_SIZE 32
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_PROG_N_ADDR (CSR_BASE + 0x4810L)
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_PROG_N_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_ADDR (CSR_BASE + 0x4814L)
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_INDEX_OFFSET 0
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_INDEX_SIZE 16
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_COUNT_OFFSET 16
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_COUNT_SIZE 16
#define CSR_PCIE_DMA0_WRITER_TABLE_LEVEL_ADDR (CSR_BASE + 0x4818L)
#define CSR_PCIE_DMA0_WRITER_TABLE_LEVEL_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_RESET_ADDR (CSR_BASE + 0x481cL)
#define CSR_PCIE_DMA0_WRITER_TABLE_RESET_SIZE 1
#define CSR_PCIE_DMA0_READER_ENABLE_ADDR (CSR_BASE + 0x4820L)
#define CSR_PCIE_DMA0_READER_ENABLE_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_ADDR (CSR_BASE + 0x4824L)
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_SIZE 2
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_ADDRESS_LSB_OFFSET 0
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_ADDRESS_LSB_SIZE 32
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_LENGTH_OFFSET 32
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_LENGTH_SIZE 24
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_IRQ_DISABLE_OFFSET 56
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_IRQ_DISABLE_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_LAST_DISABLE_OFFSET 57
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_LAST_DISABLE_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_WE_ADDR (CSR_BASE + 0x482cL)
#define CSR_PCIE_DMA0_READER_TABLE_WE_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_WE_ADDRESS_MSB_OFFSET 0
#define CSR_PCIE_DMA0_READER_TABLE_WE_ADDRESS_MSB_SIZE 32
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_PROG_N_ADDR (CSR_BASE + 0x4830L)
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_PROG_N_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_ADDR (CSR_BASE + 0x4834L)
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_INDEX_OFFSET 0
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_INDEX_SIZE 16
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_COUNT_OFFSET 16
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_COUNT_SIZE 16
#define CSR_PCIE_DMA0_READER_TABLE_LEVEL_ADDR (CSR_BASE + 0x4838L)
#define CSR_PCIE_DMA0_READER_TABLE_LEVEL_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_RESET_ADDR (CSR_BASE + 0x483cL)
#define CSR_PCIE_DMA0_READER_TABLE_RESET_SIZE 1
#define CSR_PCIE_DMA0_LOOPBACK_ENABLE_ADDR (CSR_BASE + 0x4840L)
#define CSR_PCIE_DMA0_LOOPBACK_ENABLE_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_ADDR (CSR_BASE + 0x4844L)
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_DEPTH_OFFSET 0
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_DEPTH_SIZE 24
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_SCRATCH_OFFSET 24
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_SCRATCH_SIZE 4
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_LEVEL_MODE_OFFSET 31
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_LEVEL_MODE_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_STATUS_ADDR (CSR_BASE + 0x4848L)
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_STATUS_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_STATUS_LEVEL_OFFSET 0
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_STATUS_LEVEL_SIZE 24
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_ADDR (CSR_BASE + 0x484cL)
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_DEPTH_OFFSET 0
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_DEPTH_SIZE 24
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_SCRATCH_OFFSET 24
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_SCRATCH_SIZE 4
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_LEVEL_MODE_OFFSET 31
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_LEVEL_MODE_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS_ADDR (CSR_BASE + 0x4850L)
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS_LEVEL_OFFSET 0
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS_LEVEL_SIZE 24

/* pcie_endpoint */
#define CSR_PCIE_ENDPOINT_BASE (CSR_BASE + 0x5000L)
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_ADDR (CSR_BASE + 0x5000L)
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_STATUS_OFFSET 0
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_STATUS_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_RATE_OFFSET 1
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_RATE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_WIDTH_OFFSET 2
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_WIDTH_SIZE 2
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_LTSSM_OFFSET 4
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_LTSSM_SIZE 6
#define CSR_PCIE_ENDPOINT_PHY_MSI_ENABLE_ADDR (CSR_BASE + 0x5004L)
#define CSR_PCIE_ENDPOINT_PHY_MSI_ENABLE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_MSIX_ENABLE_ADDR (CSR_BASE + 0x5008L)
#define CSR_PCIE_ENDPOINT_PHY_MSIX_ENABLE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_BUS_MASTER_ENABLE_ADDR (CSR_BASE + 0x500cL)
#define CSR_PCIE_ENDPOINT_PHY_BUS_MASTER_ENABLE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_MAX_REQUEST_SIZE_ADDR (CSR_BASE + 0x5010L)
#define CSR_PCIE_ENDPOINT_PHY_MAX_REQUEST_SIZE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_MAX_PAYLOAD_SIZE_ADDR (CSR_BASE + 0x5014L)
#define CSR_PCIE_ENDPOINT_PHY_MAX_PAYLOAD_SIZE_SIZE 1

/* pcie_msi */
#define CSR_PCIE_MSI_BASE (CSR_BASE + 0x5800L)
#define CSR_PCIE_MSI_ENABLE_ADDR (CSR_BASE + 0x5800L)
#define CSR_PCIE_MSI_ENABLE_SIZE 1
#define CSR_PCIE_MSI_CLEAR_ADDR (CSR_BASE + 0x5804L)
#define CSR_PCIE_MSI_CLEAR_SIZE 1
#define CSR_PCIE_MSI_VECTOR_ADDR (CSR_BASE + 0x5808L)
#define CSR_PCIE_MSI_VECTOR_SIZE 1

/* pcie_phy */
#define CSR_PCIE_PHY_BASE (CSR_BASE + 0x6000L)
#define CSR_PCIE_PHY_PHY_LINK_STATUS_ADDR (CSR_BASE + 0x6000L)
#define CSR_PCIE_PHY_PHY_LINK_STATUS_SIZE 1
#define CSR_PCIE_PHY_PHY_LINK_STATUS_STATUS_OFFSET 0
#define CSR_PCIE_PHY_PHY_LINK_STATUS_STATUS_SIZE 1
#define CSR_PCIE_PHY_PHY_LINK_STATUS_RATE_OFFSET 1
#define CSR_PCIE_PHY_PHY_LINK_STATUS_RATE_SIZE 1
#define CSR_PCIE_PHY_PHY_LINK_STATUS_WIDTH_OFFSET 2
#define CSR_PCIE_PHY_PHY_LINK_STATUS_WIDTH_SIZE 2
#define CSR_PCIE_PHY_PHY_LINK_STATUS_LTSSM_OFFSET 4
#define CSR_PCIE_PHY_PHY_LINK_STATUS_LTSSM_SIZE 6
#define CSR_PCIE_PHY_PHY_MSI_ENABLE_ADDR (CSR_BASE + 0x6004L)
#define CSR_PCIE_PHY_PHY_MSI_ENABLE_SIZE 1
#define CSR_PCIE_PHY_PHY_MSIX_ENABLE_ADDR (CSR_BASE + 0x6008L)
#define CSR_PCIE_PHY_PHY_MSIX_ENABLE_SIZE 1
#define CSR_PCIE_PHY_PHY_BUS_MASTER_ENABLE_ADDR (CSR_BASE + 0x600cL)
#define CSR_PCIE_PHY_PHY_BUS_MASTER_ENABLE_SIZE 1
#define CSR_PCIE_PHY_PHY_MAX_REQUEST_SIZE_ADDR (CSR_BASE + 0x6010L)
#define CSR_PCIE_PHY_PHY_MAX_REQUEST_SIZE_SIZE 1
#define CSR_PCIE_PHY_PHY_MAX_PAYLOAD_SIZE_ADDR (CSR_BASE + 0x6014L)
#define CSR_PCIE_PHY_PHY_MAX_PAYLOAD_SIZE_SIZE 1

/* probe_compensation */
#define CSR_PROBE_COMPENSATION_BASE (CSR_BASE + 0x6800L)
#define CSR_PROBE_COMPENSATION_ENABLE_ADDR (CSR_BASE + 0x6800L)
#define CSR_PROBE_COMPENSATION_ENABLE_SIZE 1
#define CSR_PROBE_COMPENSATION_WIDTH_ADDR (CSR_BASE + 0x6804L)
#define CSR_PROBE_COMPENSATION_WIDTH_SIZE 1
#define CSR_PROBE_COMPENSATION_PERIOD_ADDR (CSR_BASE + 0x6808L)
#define CSR_PROBE_COMPENSATION_PERIOD_SIZE 1

/* xadc */
#define CSR_XADC_BASE (CSR_BASE + 0x7000L)
#define CSR_XADC_TEMPERATURE_ADDR (CSR_BASE + 0x7000L)
#define CSR_XADC_TEMPERATURE_SIZE 1
#define CSR_XADC_VCCINT_ADDR (CSR_BASE + 0x7004L)
#define CSR_XADC_VCCINT_SIZE 1
#define CSR_XADC_VCCAUX_ADDR (CSR_BASE + 0x7008L)
#define CSR_XADC_VCCAUX_SIZE 1
#define CSR_XADC_VCCBRAM_ADDR (CSR_BASE + 0x700cL)
#define CSR_XADC_VCCBRAM_SIZE 1
#define CSR_XADC_EOC_ADDR (CSR_BASE + 0x7010L)
#define CSR_XADC_EOC_SIZE 1
#define CSR_XADC_EOS_ADDR (CSR_BASE + 0x7014L)
#define CSR_XADC_EOS_SIZE 1

#endif