//
// Copyright (c) 2016 Intel Corporation
// SPDX-License-Identifier: MIT

module de10_nano_audio_stream 
#(
        parameter MEM_A_WIDTH,
        parameter MEM_D_WIDTH,
        parameter MEM_BA_WIDTH
)
(
	 //////////// Audio //////////
	input		AUD_ADCDAT,
	inout		AUD_ADCLRCK,
	inout		AUD_BCLK,
	output	AUD_DACDAT,
	inout		AUD_DACLRCK,
	output	AUD_XCK,
	
	 // FPGA I2C IO 
	inout		FPGA_I2C_SCLK,
	inout 	FPGA_I2C_SDAT,
 
    //Clocks and Resets
   input fpga_clk1_50,
   input fpga_clk2_50,         
   input fpga_clk3_50,

   // HPS memory controller ports
   output wire [MEM_A_WIDTH - 1:0]    hps_memory_mem_a,                           
   output wire [MEM_BA_WIDTH - 1:0]   hps_memory_mem_ba,                          
   output wire                        hps_memory_mem_ck,                          
   output wire                        hps_memory_mem_ck_n,                        
   output wire                        hps_memory_mem_cke,                         
   output wire                        hps_memory_mem_cs_n,                        
   output wire                        hps_memory_mem_ras_n,                       
   output wire                        hps_memory_mem_cas_n,                       
   output wire                        hps_memory_mem_we_n,                        
   output wire                        hps_memory_mem_reset_n,                     
   inout  wire [MEM_D_WIDTH - 1:0]    hps_memory_mem_dq,                          
   inout  wire [(MEM_D_WIDTH/8) -1:0] hps_memory_mem_dqs,                         
   inout  wire [(MEM_D_WIDTH/8) -1:0] hps_memory_mem_dqs_n,                       
   output wire                        hps_memory_mem_odt,                         
   output wire [(MEM_D_WIDTH/8) -1:0] hps_memory_mem_dm,                          
   input  wire                        hps_memory_oct_rzqin,                       
    // HPS peripherals
   output wire        hps_emac1_TX_CLK,   
   output wire        hps_emac1_TXD0,     
   output wire        hps_emac1_TXD1,     
   output wire        hps_emac1_TXD2,     
   output wire        hps_emac1_TXD3,     
   input  wire        hps_emac1_RXD0,     
   inout  wire        hps_emac1_MDIO,     
   output wire        hps_emac1_MDC,      
   input  wire        hps_emac1_RX_CTL,   
   output wire        hps_emac1_TX_CTL,   
   input  wire        hps_emac1_RX_CLK,   
   input  wire        hps_emac1_RXD1,     
   input  wire        hps_emac1_RXD2,     
   input  wire        hps_emac1_RXD3, 
   inout  wire        hps_sdio_CMD,       
   inout  wire        hps_sdio_D0,        
   inout  wire        hps_sdio_D1,        
   output wire        hps_sdio_CLK,       
   inout  wire        hps_sdio_D2,        
   inout  wire        hps_sdio_D3,        
   inout  wire        hps_usb1_D0,        
   inout  wire        hps_usb1_D1,        
   inout  wire        hps_usb1_D2,        
   inout  wire        hps_usb1_D3,        
   inout  wire        hps_usb1_D4,        
   inout  wire        hps_usb1_D5,        
   inout  wire        hps_usb1_D6,        
   inout  wire        hps_usb1_D7,        
   input  wire        hps_usb1_CLK,       
   output wire        hps_usb1_STP,       
   input  wire        hps_usb1_DIR,       
   input  wire        hps_usb1_NXT,       
   input  wire        hps_uart0_RX,       
   output wire        hps_uart0_TX,            
   output wire        hps_spim1_CLK,
   output wire        hps_spim1_MOSI,
   input  wire        hps_spim1_MISO,
   output wire        hps_spim1_SS0,
   inout  wire        hps_i2c0_SDA,
   inout  wire        hps_i2c0_SCL,
   inout  wire        hps_i2c1_SDA,
   inout  wire        hps_i2c1_SCL,
   inout  wire        hps_gpio_GPIO09,
   inout  wire        hps_gpio_GPIO35,
   inout  wire        hps_gpio_GPIO40,
   inout  wire        hps_gpio_GPIO53,
   inout  wire        hps_gpio_GPIO54,
   inout  wire        hps_gpio_GPIO61, // ADC IRQ 
   
	// GPIO
	
	inout [35:0] gpio_0,
	inout [35:0] gpio_1,
	
   // FPGA GPIO
   input  wire [1:0]  fpga_key_pio,
   output wire [7:0]  fpga_led_pio,
   input  wire [3:0]  fpga_dipsw_pio,

    // ADC - LTC2308CUF 
   output  wire       adc_convst,
   output  wire       adc_sck,
   output  wire       adc_sdi,
   input   wire       adc_sdo,

   // HDMI
   inout wire         hdmi_i2c_scl,
   inout wire         hdmi_i2c_sda,
   inout wire         hdmi_i2s,
   inout wire         hdmi_lrclk,
   inout wire         hdmi_mclk,
   inout wire         hdmi_sclk,
   output wire        hdmi_tx_clk,
   output wire        hdmi_tx_de,
   output wire        hdmi_tx_hs,
   input wire         hdmi_tx_int,
   output wire        hdmi_tx_vs,
   output wire [23:0] hdmi_tx_d
);

//REG/WIRE Declarations

// i2c connection
wire hdmi_internal_scl_o_e;
wire hdmi_internal_scl_o;
wire hdmi_internal_sda_o_e;
wire hdmi_internal_sda_o;

ALT_IOBUF scl_iobuf (.i(1'b0), .oe(hdmi_internal_scl_o_e), .o(hdmi_internal_scl_o), .io(hdmi_i2c_scl));
ALT_IOBUF sda_iobuf (.i(1'b0), .oe(hdmi_internal_sda_o_e), .o(hdmi_internal_sda_o), .io(hdmi_i2c_sda));

// FPGA I2C Connections
wire fpga_i2c_internal_scl_o_e;
wire fpga_i2c_internal_scl_o;
wire fpga_i2c_internal_sda_o_e;
wire fpga_i2c_internal_sda_o;

ALT_IOBUF fpga_scl_iobuf (.i(1'b0), .oe(fpga_i2c_internal_scl_o_e), .o(fpga_i2c_internal_scl_o), .io(FPGA_I2C_SCLK));
ALT_IOBUF fpga_sda_iobuf (.i(1'b0), .oe(fpga_i2c_internal_sda_o_e), .o(fpga_i2c_internal_sda_o), .io(FPGA_I2C_SDAT));

// additional wire declarations for I2S output controller
	wire				clock_bridge_0_out_clk_clk;
	wire				hps_0_h2f_reset_reset_n;
	wire				hps_0_f2h_dma_req0_dma_req;
	wire				hps_0_f2h_dma_req0_dma_single;
	wire				hps_0_f2h_dma_req0_dma_ack;
	wire				hps_0_f2h_dma_req1_dma_req;
	wire				hps_0_f2h_dma_req1_dma_single;
	wire				hps_0_f2h_dma_req1_dma_ack;
	wire				hps_0_f2h_dma_req2_dma_req;
	wire				hps_0_f2h_dma_req2_dma_single;
	wire				hps_0_f2h_dma_req2_dma_ack;
	wire				hps_0_f2h_dma_req3_dma_req;
	wire				hps_0_f2h_dma_req3_dma_single;
	wire				hps_0_f2h_dma_req3_dma_ack;
// additional wire declarations for I2S output controller 
	wire	[63:0]	i2s_output_apb_0_playback_fifo_data;
	wire				i2s_output_apb_0_playback_fifo_read;
	wire				i2s_output_apb_0_playback_fifo_empty;
	wire				i2s_output_apb_0_playback_fifo_full;
	wire				i2s_output_apb_0_playback_fifo_clk;
	wire				i2s_output_apb_0_playback_dma_enable;
	wire				i2s_playback_enable;
	wire	[63:0]	i2s_output_apb_0_capture_fifo_data;
	wire				i2s_output_apb_0_capture_fifo_write;
	wire				i2s_output_apb_0_capture_fifo_empty;
	wire				i2s_output_apb_0_capture_fifo_full;
	wire				i2s_output_apb_0_capture_fifo_clk;
	wire				i2s_output_apb_0_capture_dma_enable;
	wire				i2s_capture_enable;
	
// additional wire declarations for I2S clock controller 
	wire				i2s_clkctrl_apb_0_ext_bclk;
	wire				i2s_clkctrl_apb_0_ext_playback_lrclk;
	wire				i2s_clkctrl_apb_0_ext_capture_lrclk;
	wire				i2s_clkctrl_apb_0_conduit_master_slave_mode;
	wire				i2s_clkctrl_apb_0_conduit_clk_sel_48_44;
	wire				i2s_clkctrl_apb_0_conduit_bclk;
	wire				i2s_clkctrl_apb_0_conduit_playback_lrclk;
	wire				i2s_clkctrl_apb_0_conduit_capture_lrclk;
	wire				i2s_clkctrl_apb_0_mclk_clk;
	wire				clock_bridge_48_out_clk_clk;
	wire				clock_bridge_44_out_clk_clk;
	
// simple hps audio i/o wire declarations
	reg  signed [31:0]		hps_audio_sample_in;
	wire signed [31:0]		hps_audio_sample_out;
	wire 				hps_audio_strobe;
	
	wire [31:0] mi_connection_memory_read_data;
	wire [9:0]  mi_connection_memory_addr;
	
// SoC sub-system module
soc_system soc_inst (
  //Clocks & Resets
  .clk_clk                               (fpga_clk1_50),
  .reset_reset_n								  (hps_0_h2f_reset_reset_n),
  .hps_0_h2f_reset_reset_n               (hps_0_h2f_reset_reset_n), //hps_fpga_reset_n
  
  //DRAM
  .memory_mem_a                          (hps_memory_mem_a),                               
  .memory_mem_ba                         (hps_memory_mem_ba),                         
  .memory_mem_ck                         (hps_memory_mem_ck),                         
  .memory_mem_ck_n                       (hps_memory_mem_ck_n),                       
  .memory_mem_cke                        (hps_memory_mem_cke),                        
  .memory_mem_cs_n                       (hps_memory_mem_cs_n),                       
  .memory_mem_ras_n                      (hps_memory_mem_ras_n),                      
  .memory_mem_cas_n                      (hps_memory_mem_cas_n),                      
  .memory_mem_we_n                       (hps_memory_mem_we_n),                       
  .memory_mem_reset_n                    (hps_memory_mem_reset_n),                    
  .memory_mem_dq                         (hps_memory_mem_dq),                         
  .memory_mem_dqs                        (hps_memory_mem_dqs),                        
  .memory_mem_dqs_n                      (hps_memory_mem_dqs_n),                      
  .memory_mem_odt                        (hps_memory_mem_odt),                            
  .memory_mem_dm                         (hps_memory_mem_dm),                         
  .memory_oct_rzqin                      (hps_memory_oct_rzqin),      
  
  //HPS Peripherals
  //Emac1
  .hps_0_hps_io_hps_io_emac1_inst_TX_CLK (hps_emac1_TX_CLK), 
  .hps_0_hps_io_hps_io_emac1_inst_TXD0   (hps_emac1_TXD0),   
  .hps_0_hps_io_hps_io_emac1_inst_TXD1   (hps_emac1_TXD1),   
  .hps_0_hps_io_hps_io_emac1_inst_TXD2   (hps_emac1_TXD2),   
  .hps_0_hps_io_hps_io_emac1_inst_TXD3   (hps_emac1_TXD3),   
  .hps_0_hps_io_hps_io_emac1_inst_RXD0   (hps_emac1_RXD0),   
  .hps_0_hps_io_hps_io_emac1_inst_MDIO   (hps_emac1_MDIO),   
  .hps_0_hps_io_hps_io_emac1_inst_MDC    (hps_emac1_MDC),    
  .hps_0_hps_io_hps_io_emac1_inst_RX_CTL (hps_emac1_RX_CTL), 
  .hps_0_hps_io_hps_io_emac1_inst_TX_CTL (hps_emac1_TX_CTL), 
  .hps_0_hps_io_hps_io_emac1_inst_RX_CLK (hps_emac1_RX_CLK), 
  .hps_0_hps_io_hps_io_emac1_inst_RXD1   (hps_emac1_RXD1),   
  .hps_0_hps_io_hps_io_emac1_inst_RXD2   (hps_emac1_RXD2),   
  .hps_0_hps_io_hps_io_emac1_inst_RXD3   (hps_emac1_RXD3),
  //SDMMC
  .hps_0_hps_io_hps_io_sdio_inst_CMD     (hps_sdio_CMD),     
  .hps_0_hps_io_hps_io_sdio_inst_D0      (hps_sdio_D0),      
  .hps_0_hps_io_hps_io_sdio_inst_D1      (hps_sdio_D1),      
  .hps_0_hps_io_hps_io_sdio_inst_CLK     (hps_sdio_CLK),     
  .hps_0_hps_io_hps_io_sdio_inst_D2      (hps_sdio_D2),      
  .hps_0_hps_io_hps_io_sdio_inst_D3      (hps_sdio_D3),
  //USB1
  .hps_0_hps_io_hps_io_usb1_inst_D0      (hps_usb1_D0),      
  .hps_0_hps_io_hps_io_usb1_inst_D1      (hps_usb1_D1),      
  .hps_0_hps_io_hps_io_usb1_inst_D2      (hps_usb1_D2),      
  .hps_0_hps_io_hps_io_usb1_inst_D3      (hps_usb1_D3),      
  .hps_0_hps_io_hps_io_usb1_inst_D4      (hps_usb1_D4),      
  .hps_0_hps_io_hps_io_usb1_inst_D5      (hps_usb1_D5),      
  .hps_0_hps_io_hps_io_usb1_inst_D6      (hps_usb1_D6),      
  .hps_0_hps_io_hps_io_usb1_inst_D7      (hps_usb1_D7),      
  .hps_0_hps_io_hps_io_usb1_inst_CLK     (hps_usb1_CLK),     
  .hps_0_hps_io_hps_io_usb1_inst_STP     (hps_usb1_STP),     
  .hps_0_hps_io_hps_io_usb1_inst_DIR     (hps_usb1_DIR),     
  .hps_0_hps_io_hps_io_usb1_inst_NXT     (hps_usb1_NXT),
  
  //UART0
  .hps_0_hps_io_hps_io_uart0_inst_RX     (hps_uart0_RX),     
  .hps_0_hps_io_hps_io_uart0_inst_TX     (hps_uart0_TX),     
  //I2C 0,1
  .hps_0_hps_io_hps_io_i2c0_inst_SDA     (hps_i2c0_SDA),
  .hps_0_hps_io_hps_io_i2c0_inst_SCL     (hps_i2c0_SCL),
  .hps_0_hps_io_hps_io_i2c1_inst_SDA     (hps_i2c1_SDA),
  .hps_0_hps_io_hps_io_i2c1_inst_SCL     (hps_i2c1_SCL),
  //GPIO
  .hps_0_hps_io_hps_io_gpio_inst_GPIO09  (hps_gpio_GPIO09),
  .hps_0_hps_io_hps_io_gpio_inst_GPIO35  (hps_gpio_GPIO35),
  .hps_0_hps_io_hps_io_gpio_inst_GPIO40  (hps_gpio_GPIO40),
  .hps_0_hps_io_hps_io_gpio_inst_GPIO53  (hps_gpio_GPIO53),
  .hps_0_hps_io_hps_io_gpio_inst_GPIO54  (hps_gpio_GPIO54),
  .hps_0_hps_io_hps_io_gpio_inst_GPIO61  (hps_gpio_GPIO61), 

  //HDMI I2C
  .hps_0_i2c2_out_data   (hdmi_internal_sda_o_e),
  .hps_0_i2c2_sda        (hdmi_internal_sda_o),
  .hps_0_i2c2_clk_clk    (hdmi_internal_scl_o_e),
  .hps_0_i2c2_scl_in_clk (hdmi_internal_scl_o),
  
  // FPGA I2C
  .hps_0_i2c3_scl_in_clk    (fpga_i2c_internal_scl_o),
  .hps_0_i2c3_clk_clk       (fpga_i2c_internal_scl_o_e),
  .hps_0_i2c3_out_data      (fpga_i2c_internal_sda_o_e),
  .hps_0_i2c3_sda           (fpga_i2c_internal_sda_o),  
    
  // HPS DMA Requests
. hps_0_f2h_dma_req0_dma_req( hps_0_f2h_dma_req0_dma_req ),
. hps_0_f2h_dma_req0_dma_single( hps_0_f2h_dma_req0_dma_single ) ,
. hps_0_f2h_dma_req0_dma_ack( hps_0_f2h_dma_req0_dma_ack ),
. hps_0_f2h_dma_req1_dma_req( hps_0_f2h_dma_req1_dma_req ),
. hps_0_f2h_dma_req1_dma_single ( hps_0_f2h_dma_req1_dma_single ) ,
. hps_0_f2h_dma_req1_dma_ack( hps_0_f2h_dma_req1_dma_ack ),
. hps_0_f2h_dma_req2_dma_req( hps_0_f2h_dma_req2_dma_req ),
. hps_0_f2h_dma_req2_dma_single ( hps_0_f2h_dma_req2_dma_single ) ,
. hps_0_f2h_dma_req2_dma_ack( hps_0_f2h_dma_req2_dma_ack ),
. hps_0_f2h_dma_req3_dma_req( hps_0_f2h_dma_req3_dma_req ),
. hps_0_f2h_dma_req3_dma_single( hps_0_f2h_dma_req3_dma_single ) ,
. hps_0_f2h_dma_req3_dma_ack( hps_0_f2h_dma_req3_dma_ack ),

// I2S Output APB
. i2s_output_apb_0_playback_fifo_data ( i2s_output_apb_0_playback_fifo_data ) ,
. i2s_output_apb_0_playback_fifo_read ( i2s_output_apb_0_playback_fifo_read ) ,
. i2s_output_apb_0_playback_fifo_empty ( i2s_output_apb_0_playback_fifo_empty ),
. i2s_output_apb_0_playback_fifo_full ( i2s_output_apb_0_playback_fifo_full ) ,
. i2s_output_apb_0_playback_fifo_clk ( i2s_output_apb_0_playback_fifo_clk ),
. i2s_output_apb_0_playback_dma_req ( hps_0_f2h_dma_req0_dma_single ),
. i2s_output_apb_0_playback_dma_ack ( hps_0_f2h_dma_req0_dma_ack ),
. i2s_output_apb_0_playback_dma_enable ( i2s_output_apb_0_playback_dma_enable ),
. i2s_output_apb_0_capture_fifo_data ( i2s_output_apb_0_capture_fifo_data ), // <-- no loopback
. i2s_output_apb_0_capture_fifo_write ( i2s_output_apb_0_capture_fifo_write ) ,
. i2s_output_apb_0_capture_fifo_empty ( i2s_output_apb_0_capture_fifo_empty ) ,
. i2s_output_apb_0_capture_fifo_full ( i2s_output_apb_0_capture_fifo_full ),
. i2s_output_apb_0_capture_fifo_clk ( i2s_output_apb_0_capture_fifo_clk ),
. i2s_output_apb_0_capture_dma_req ( hps_0_f2h_dma_req1_dma_single ),
. i2s_output_apb_0_capture_dma_ack ( hps_0_f2h_dma_req1_dma_ack ) ,
. i2s_output_apb_0_capture_dma_enable ( i2s_output_apb_0_capture_dma_enable ) ,
// I2S Clock Control APB
. i2s_clkctrl_apb_0_ext_bclk ( i2s_clkctrl_apb_0_ext_bclk ),
. i2s_clkctrl_apb_0_ext_playback_lrclk ( i2s_clkctrl_apb_0_ext_playback_lrclk ),
. i2s_clkctrl_apb_0_ext_capture_lrclk ( i2s_clkctrl_apb_0_ext_capture_lrclk ) ,
. i2s_clkctrl_apb_0_conduit_master_slave_mode ( i2s_clkctrl_apb_0_conduit_master_slave_mode ) ,
. i2s_clkctrl_apb_0_conduit_clk_sel_48_44 ( i2s_clkctrl_apb_0_conduit_clk_sel_48_44 ) ,
. i2s_clkctrl_apb_0_conduit_bclk ( i2s_clkctrl_apb_0_conduit_bclk ),
. i2s_clkctrl_apb_0_conduit_playback_lrclk ( i2s_clkctrl_apb_0_conduit_playback_lrclk ),
. i2s_clkctrl_apb_0_conduit_capture_lrclk ( i2s_clkctrl_apb_0_conduit_capture_lrclk ) ,
. i2s_clkctrl_apb_0_mclk_clk ( i2s_clkctrl_apb_0_mclk_clk ),
// Clock Bridges
. clock_bridge_0_out_clk_clk ( clock_bridge_0_out_clk_clk ),
. clock_bridge_48_out_clk_clk ( clock_bridge_48_out_clk_clk ),
. clock_bridge_44_out_clk_clk ( clock_bridge_44_out_clk_clk ),
 // FPGA to On-Chip Ram Input Sample Buffer: 
 // address[0]: (new samples from HPS written)? 0xabcd:0, 
 // address[1]: current buffer size (set by HPS), 
 // address[2]to[511]: nothing (reserved)
 // addresses[512]to[1023]: samples from HPS, max 512 samples
 // base address is 0xC0004000
 .in_smp_buf_s2_address                    (in_smp_buf_addr),         //               in_smp_buf_s2.address
 .in_smp_buf_s2_chipselect                 (1'b1),                    //                            .chipselect
 .in_smp_buf_s2_clken                      (1'b1),                    //                            .clken
 .in_smp_buf_s2_write                      (in_smp_buf_we),           //                            .write
 .in_smp_buf_s2_readdata                   (in_smp_buf_read_data),    //                            .readdata
 .in_smp_buf_s2_writedata                  (in_smp_buf_write_data),   //                            .writedata
 .in_smp_buf_s2_byteenable                 (4'b1111),   					 //                            .byteenable
 
 // FPGA to On-Chip Ram Output Sample Buffer: 
 // address[0]:  (new samples from FPGA written)? 0xabcd:0,
 // address[1]: current measured buffer inteval time, 
 // address[2]to[511]: nothing (reserved)
 // addresses 512-1023: samples from FPGA, max 512 samples
  // base address is 0xC0005000
 .out_smp_buf_s2_address                    (out_smp_buf_addr),         //              out_smp_buf_s2.address
 .out_smp_buf_s2_chipselect                 (1'b1),                     //                            .chipselect
 .out_smp_buf_s2_clken                      (1'b1),                     //                            .clken
 .out_smp_buf_s2_write                      (out_smp_buf_we),           //                            .write
 .out_smp_buf_s2_readdata                   (out_smp_buf_read_data),    //                            .readdata
 .out_smp_buf_s2_writedata                  (out_smp_buf_write_data),   //                            .writedata
 .out_smp_buf_s2_byteenable                 (4'b1111)   					   //                            .byteenable
);

reg[3:0] bufferTimingFSM = 0;
parameter Wait = 0, NewBufferArrived = 1, ReadNSamples = 2, CopySamples = 3, WaitWriteAck = 4, WriteAck = 5, FinishAck = 6;
wire reset = ~hps_0_h2f_reset_reset_n;

wire signed [31:0] in_smp_buf_read_data;
reg signed [31:0] in_smp_buf_write_data;
reg unsigned [31:0]  in_smp_buf_addr; // 10bit is 0-1024
reg in_smp_buf_we;

wire signed [31:0] out_smp_buf_read_data;
reg  signed [31:0] out_smp_buf_write_data;
reg  [31:0]  out_smp_buf_addr; // 10bit is 0-1024
reg out_smp_buf_we;

reg signed [31:0] in_smp_buf_n_smp = 0; // max 512 samples
reg [31:0] smp_idx = 0;
	
// state machine: copy input to output buffer and write acknowledge 
always @ (posedge fpga_clk1_50 or posedge reset) begin
	if (reset) begin
		bufferTimingFSM <= Wait;
		in_smp_buf_we <= 0;
		in_smp_buf_addr <= 0;
		out_smp_buf_we <= 0;
		out_smp_buf_addr <= 0;
		counter_reset <= 0;
		out_smp_buf_write_data <= 0;
		in_smp_buf_write_data <= 0;
		smp_idx = 0;
	end
	else begin
		case (bufferTimingFSM)
			Wait: begin
				in_smp_buf_we <= 0;
				in_smp_buf_addr <= 0;
				out_smp_buf_we <= 0;
				out_smp_buf_addr <= 1;
				counter_reset <= 0;
				out_smp_buf_write_data <= 0;
				in_smp_buf_write_data <= 0;
				smp_idx = 0;
				
				if ( in_smp_buf_read_data == 32'h0000_abcd ) begin
					
					// read n_samples to transfer in two cycles (NewBufferArrived state)
					in_smp_buf_addr <= 1;
					out_smp_buf_addr <= 1;
					// write counter output
					out_smp_buf_we <= 1;
					out_smp_buf_write_data <= counter_output;
					counter_reset <= 1;
					bufferTimingFSM <= NewBufferArrived;
				end
				else begin
					bufferTimingFSM <= Wait;
				end
			end
			
			NewBufferArrived: begin
				counter_reset <= 0;
				in_smp_buf_we <= 0;
				out_smp_buf_we <= 0;
				// in another two clock cycles we get the first sample
				// set index to first sample to transfer
				in_smp_buf_addr <= 512;
				bufferTimingFSM <= ReadNSamples;
			end
			
			ReadNSamples: begin
			// this state reads number of samples to process
				in_smp_buf_we <= 0;
				in_smp_buf_n_smp <= in_smp_buf_read_data;
				in_smp_buf_addr <= 513;
				bufferTimingFSM <= CopySamples;
			end
			
			CopySamples: begin
				if (smp_idx < in_smp_buf_n_smp-2) begin
					// copy
					out_smp_buf_we <= 1;
					// advance input sample pointer
					smp_idx = smp_idx + 1;
					in_smp_buf_addr <= 513 + smp_idx;
					out_smp_buf_addr <= 511 + smp_idx;
					out_smp_buf_write_data = in_smp_buf_read_data;
				end
				else begin // done (but two samples still wait for transfer)
					out_smp_buf_addr <= 511 + smp_idx + 1;
					out_smp_buf_write_data = in_smp_buf_read_data;
					in_smp_buf_addr <= 0; // one tick delay for write
					bufferTimingFSM <= WaitWriteAck;
				end
			end
			
			WaitWriteAck: begin
				out_smp_buf_addr <= 511 + smp_idx + 2;
				out_smp_buf_write_data = in_smp_buf_read_data;
				bufferTimingFSM <= WriteAck;
				in_smp_buf_we <= 0;
			end
			
			WriteAck: begin
				out_smp_buf_addr <= 0; // one tick delay for write
				out_smp_buf_write_data <= 32'h0000_abcd; // out_smp_buf[0] == 0xabcd signifies that buffer was processed
				// clear new data arrived flag
				in_smp_buf_we <= 1;
				in_smp_buf_write_data <= 0;
				// go to reset
				bufferTimingFSM <= FinishAck; 
			end
			
			FinishAck: begin
				out_smp_buf_we <= 0;
				in_smp_buf_we <= 0;
				bufferTimingFSM <= Wait;
			end
			
			default:
				bufferTimingFSM <= Wait;
		endcase
	end
end

assign fpga_led_pio[7:0] = counter_output[7:0];


reg counter_reset = 0;
wire [31:0] counter_output;
Counter timeBetweenBuffers(
	.clk(fpga_clk1_50),
	.reset(counter_reset),
	.count(counter_output)
);


	wire i2s_playback_fifo_ack48;
	wire i2s_data_out48;
	i2s_shift_out i2s_shift_out48(
		.reset_n							(hps_0_h2f_reset_reset_n),
		.clk								(clock_bridge_48_out_clk_clk),
		
		.fifo_right_data					(i2s_output_apb_0_playback_fifo_data[63:32]),
		.fifo_left_data					(i2s_output_apb_0_playback_fifo_data[31:0]),
		.fifo_ready							(~i2s_output_apb_0_playback_fifo_empty),
		.fifo_ack							(i2s_playback_fifo_ack48),
  
		.enable								(i2s_playback_enable),
		.bclk									(i2s_clkctrl_apb_0_conduit_bclk),
		.lrclk								(i2s_clkctrl_apb_0_conduit_playback_lrclk),
		.data_out							(i2s_data_out48)
	);
	wire i2s_playback_fifo_ack44;
	wire i2s_data_out44;
	i2s_shift_out i2s_shift_out44(
		.reset_n							(hps_0_h2f_reset_reset_n),
		.clk								(clock_bridge_44_out_clk_clk),
		
		.fifo_right_data					(i2s_output_apb_0_playback_fifo_data[63:32]),
		.fifo_left_data					(i2s_output_apb_0_playback_fifo_data[31:0]),
		.fifo_ready							(~i2s_output_apb_0_playback_fifo_empty),
		.fifo_ack							(i2s_playback_fifo_ack44),
  
		.enable								(i2s_playback_enable),
		.bclk									(i2s_clkctrl_apb_0_conduit_bclk),
		.lrclk								(i2s_clkctrl_apb_0_conduit_playback_lrclk),
		.data_out							(i2s_data_out44)
	);

	wire i2s_capture_fifo_write48;
	wire i2s_data_in48;
	wire [63:0] i2s_capture_fifo_data48;
	i2s_shift_in i2s_shift_in48(
		.reset_n							(hps_0_h2f_reset_reset_n),
		.clk								(clock_bridge_48_out_clk_clk),
		
		.fifo_right_data					(i2s_capture_fifo_data48[63:32]),
		.fifo_left_data						(i2s_capture_fifo_data48[31:0]),
		.fifo_ready							(~i2s_output_apb_0_capture_fifo_full),
		.fifo_write							(i2s_capture_fifo_write48),
  
		.enable								(i2s_capture_enable),
		.bclk								(i2s_clkctrl_apb_0_conduit_bclk),
		.lrclk								(i2s_clkctrl_apb_0_conduit_capture_lrclk),
		.data_in							(i2s_data_in48)
	);
	wire i2s_capture_fifo_write44;
	wire i2s_data_in44;
	wire [63:0] i2s_capture_fifo_data44;
	i2s_shift_in i2s_shift_in44(
		.reset_n							(hps_0_h2f_reset_reset_n),
		.clk								(clock_bridge_44_out_clk_clk),
		
		.fifo_right_data					(i2s_capture_fifo_data44[63:32]),
		.fifo_left_data						(i2s_capture_fifo_data44[31:0]),
		.fifo_ready							(~i2s_output_apb_0_capture_fifo_full),
		.fifo_write							(i2s_capture_fifo_write44),
  
		.enable								(i2s_capture_enable),
		.bclk								(i2s_clkctrl_apb_0_conduit_bclk),
		.lrclk								(i2s_clkctrl_apb_0_conduit_capture_lrclk),
		.data_in							(i2s_data_in44)
	);

	// Combinatorics
	assign AUD_XCK = i2s_clkctrl_apb_0_mclk_clk;
	assign i2s_playback_enable = i2s_output_apb_0_playback_dma_enable & ~i2s_output_apb_0_playback_fifo_empty;
	assign i2s_capture_enable = i2s_output_apb_0_capture_dma_enable & ~i2s_output_apb_0_capture_fifo_full;

	// Mux and sync fifo read ack
	reg [2:0] i2s_playback_fifo_ack_synchro;
	assign i2s_playback_fifo_ack = i2s_clkctrl_apb_0_conduit_clk_sel_48_44 ?
		i2s_playback_fifo_ack44 : i2s_playback_fifo_ack48;
	always @(posedge clock_bridge_0_out_clk_clk or negedge hps_0_h2f_reset_reset_n)
		if (~hps_0_h2f_reset_reset_n)
			i2s_playback_fifo_ack_synchro <= 0;
		else
			i2s_playback_fifo_ack_synchro <= {i2s_playback_fifo_ack_synchro[1:0], i2s_playback_fifo_ack};
	assign i2s_output_apb_0_playback_fifo_read = i2s_playback_fifo_ack_synchro[2] & ~i2s_playback_fifo_ack_synchro[1];
	assign i2s_output_apb_0_playback_fifo_clk = clock_bridge_0_out_clk_clk;

	// Mux and sync fifo write
	reg [2:0] i2s_capture_fifo_write_synchro;
	assign i2s_capture_fifo_write = i2s_clkctrl_apb_0_conduit_clk_sel_48_44 ?
		i2s_capture_fifo_write44 : i2s_capture_fifo_write48;
	always @(posedge clock_bridge_0_out_clk_clk or negedge hps_0_h2f_reset_reset_n)
		if (~hps_0_h2f_reset_reset_n)
			i2s_capture_fifo_write_synchro <= 0;
		else
			i2s_capture_fifo_write_synchro <= {i2s_capture_fifo_write_synchro[1:0], i2s_capture_fifo_write};
	assign i2s_output_apb_0_capture_fifo_write = i2s_capture_fifo_write_synchro[2] & ~i2s_capture_fifo_write_synchro[1];
	assign i2s_output_apb_0_capture_fifo_clk = clock_bridge_0_out_clk_clk;

	// Mux capture data
	assign i2s_output_apb_0_capture_fifo_data = i2s_clkctrl_apb_0_conduit_clk_sel_48_44 ?
		i2s_capture_fifo_data44 : i2s_capture_fifo_data48;

	// Mux out
	assign AUD_DACDAT = i2s_clkctrl_apb_0_conduit_clk_sel_48_44 ? i2s_data_out44 : i2s_data_out48;

	// Audio input
	assign i2s_data_in44 = AUD_ADCDAT;
	assign i2s_data_in48 = AUD_ADCDAT;

	//assign i2s_data_in44 = i2s_data_out44; // Loopback for testing
	//assign i2s_data_in48 = i2s_data_out48; // Loopback for testing
		
	// Audio clocks inouts
	assign AUD_BCLK = i2s_clkctrl_apb_0_conduit_master_slave_mode ?
		i2s_clkctrl_apb_0_conduit_bclk : 1'bZ;
	assign AUD_DACLRCK = i2s_clkctrl_apb_0_conduit_master_slave_mode ?
		i2s_clkctrl_apb_0_conduit_playback_lrclk : 1'bZ;
	assign AUD_ADCLRCK = i2s_clkctrl_apb_0_conduit_master_slave_mode ?
		i2s_clkctrl_apb_0_conduit_capture_lrclk : 1'bZ;

	assign i2s_clkctrl_apb_0_ext_bclk = i2s_clkctrl_apb_0_conduit_master_slave_mode ?
		i2s_clkctrl_apb_0_conduit_bclk : AUD_BCLK;
	assign i2s_clkctrl_apb_0_ext_playback_lrclk = i2s_clkctrl_apb_0_conduit_master_slave_mode ?
		i2s_clkctrl_apb_0_conduit_playback_lrclk : AUD_DACLRCK;
	assign i2s_clkctrl_apb_0_ext_capture_lrclk = i2s_clkctrl_apb_0_conduit_master_slave_mode ?
		i2s_clkctrl_apb_0_conduit_capture_lrclk : AUD_DACLRCK; // shouldn't that be AUD_ADCLRCK ?

endmodule
