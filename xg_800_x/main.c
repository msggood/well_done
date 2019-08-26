/***************************************************************************//**
 *   @file   main.c
 *   @brief  Implementation of Main Function.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "initial.h"
#include <stdio.h>
#include <netinet/in.h> 
#include <sys/types.h>    
#include <sys/socket.h>   
#include <stdlib.h>       
#include <string.h>       
#include <unistd.h>
#include <pthread.h>

#include "parameters.h"
#include "reg_defined.h"


extern void reset_all_register();
extern void dma_tx_rx();


#define CMD_SERVER_PORT    6666 
#define LENGTH_OF_LISTEN_QUEUE 20
#define BUFFER_SIZE 1024
#define FILE_NAME_MAX_SIZE 512


void do_cmd(char*received_cmd)
{
	invalid_cmd = 0;
	
	memset(received_cmd+strlen(received_cmd),'\r',1);
	printf("received_cmd=%s\n",received_cmd);
	for(cmd = 0; cmd < cmd_no; cmd++)
	{
		param_no = 0;
		//printf("step4\n");
		cmd_type = console_check_commands(received_cmd, cmd_list[cmd].name,
											  param, &param_no);
		if(cmd_type == UNKNOWN_CMD)
		{
			invalid_cmd++;
		}
		else
		{
			cmd_list[cmd].function(param, param_no);
		}
	}
	if(invalid_cmd == cmd_no)
	{
		printf("Invalid command!\n");
	}
}

void * cmd_socket_server(void *arg)
{
    //设置一个socket地址结构server_addr,代表服务器internet地址, 端口
    struct sockaddr_in server_addr;
    bzero(&server_addr,sizeof(server_addr)); //把一段内存区的内容全部设置为0
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr.sin_port = htons(CMD_SERVER_PORT);
	int ret=0;
	void *p_ret=&ret;
    //创建用于internet的流协议(TCP)socket,用server_socket代表服务器socket
    int server_socket = socket(PF_INET,SOCK_STREAM,0);
    if( ret=server_socket < 0)
    {
        printf("Create Socket Failed!");
		return p_ret;        
    }
    int opt =1;
    setsockopt(server_socket,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));

    //把socket和socket地址结构联系起来
    if( ret=bind(server_socket,(struct sockaddr*)&server_addr,sizeof(server_addr)))
    {
        printf("Server Bind Port : %d Failed!", CMD_SERVER_PORT); 
		return p_ret;        
    }

    //server_socket用于监听
    if ( ret=listen(server_socket, LENGTH_OF_LISTEN_QUEUE) )
    {
        printf("Server Listen Failed!"); 
		return p_ret;         
    }
    while (1) //服务器端要一直运行
    {
        struct sockaddr_in client_addr;
        int length = sizeof(client_addr);

        int new_server_socket = accept(server_socket,(struct sockaddr*)&client_addr,&length);
        if ( new_server_socket < 0)
        {
            printf("Server Accept Failed!\n");
            break;
        }

        char buffer[BUFFER_SIZE];
        bzero(buffer, BUFFER_SIZE);
        length = recv(new_server_socket,buffer,BUFFER_SIZE,0);
        if (length < 0)
        {
            printf("Server Recieve Data Failed!\n");
            break;
        }
		printf("recv:%s\n",buffer);
		do_cmd(buffer);
#if 0		
        bzero(buffer, BUFFER_SIZE);
        if(send(new_server_socket,buffer,BUFFER_SIZE,0)<0)
        {
            printf("Send Failed\n");
            break;
        }
        //            close(fp);
#endif		
        close(new_server_socket);
    }
    //关闭监听用的socket
    close(server_socket);
    return p_ret;
}

void cpu_idle()
{
	while(1)
		sleep(10);
}

int main(void)
{
#if 1

	initial_ad9361();
	double gain_db=40;
	set_rx1_mgc(&gain_db, 1);

	reg_init(BASED_ADDR,4096);
	writew(DMA_LENGTH_OFFSET,0x100);
	printf("dma_length_offset:%d\n",readw(DMA_LENGTH_OFFSET));
	writew(INTR_GAP_OFFSET,0x989680);
	printf("intr_gap:%d\n",readw(INTR_GAP_OFFSET));
	writew(TX_10M_GAP_OFFSET,0x61A800); //0x61A800=> 0.5s,0x186A000;=>2s
	printf("tx_10m_gap:%d\n",readw(TX_10M_GAP_OFFSET));
	writew(TX_LED_GAP_OFFSET,0xD400);
	reg_exit();
#endif

#if 1
	pthread_t cmd_thread;
	pthread_create(&cmd_thread, NULL,cmd_socket_server,NULL);
#endif	
	dma_tx_rx();

	cpu_idle();

	return 0;
}


/***************************************************************************//**
 * @brief main

*******************************************************************************/
void initial_ad9361(void)
{
#ifdef ALTERA_PLATFORM
	if (altera_bridge_init()) {
		printf("Altera Bridge Init Error!\n");
		return -1;
	}
#endif

	// NOTE: The user has to choose the GPIO numbers according to desired
	// carrier board.
	default_init_param.gpio_resetb = GPIO_RESET_PIN;
#ifdef FMCOMMS5
	default_init_param.gpio_sync = GPIO_SYNC_PIN;
	default_init_param.gpio_cal_sw1 = GPIO_CAL_SW1_PIN;
	default_init_param.gpio_cal_sw2 = GPIO_CAL_SW2_PIN;
	default_init_param.rx1rx2_phase_inversion_en = 1;
#else
	default_init_param.gpio_sync = -1;
	default_init_param.gpio_cal_sw1 = -1;
	default_init_param.gpio_cal_sw2 = -1;
#endif

#ifdef LINUX_PLATFORM
	gpio_init(default_init_param.gpio_resetb);
#else
	gpio_init(GPIO_DEVICE_ID);
#endif
	gpio_direction(default_init_param.gpio_resetb, 1);

	spi_init(SPI_DEVICE_ID, 1, 0);

	if (AD9364_DEVICE)
		default_init_param.dev_sel = ID_AD9364;
	if (AD9363A_DEVICE)
		default_init_param.dev_sel = ID_AD9363A;

#if defined FMCOMMS5 || defined ADI_RF_SOM || defined ADI_RF_SOM_CMOS
	default_init_param.xo_disable_use_ext_refclk_enable = 1;
#endif

#ifdef ADI_RF_SOM_CMOS
	default_init_param.swap_ports_enable = 1;
	default_init_param.lvds_mode_enable = 0;
	default_init_param.lvds_rx_onchip_termination_enable = 0;
	default_init_param.full_port_enable = 1;
	default_init_param.digital_interface_tune_fir_disable = 1;
#endif

//	reset_all_register();
	ad9361_init(&ad9361_phy, &default_init_param);

	ad9361_set_tx_fir_config(ad9361_phy, tx_fir_config);
	ad9361_set_rx_fir_config(ad9361_phy, rx_fir_config);

#ifdef FMCOMMS5
#ifdef LINUX_PLATFORM
	gpio_init(default_init_param.gpio_sync);
#endif
	gpio_direction(default_init_param.gpio_sync, 1);
	default_init_param.id_no = 1;
	default_init_param.gpio_resetb = GPIO_RESET_PIN_2;
#ifdef LINUX_PLATFORM
	gpio_init(default_init_param.gpio_resetb);
#endif
	default_init_param.gpio_sync = -1;
	default_init_param.gpio_cal_sw1 = -1;
	default_init_param.gpio_cal_sw2 = -1;
	default_init_param.rx_synthesizer_frequency_hz = 2300000000UL;
	default_init_param.tx_synthesizer_frequency_hz = 2300000000UL;
	gpio_direction(default_init_param.gpio_resetb, 1);
	ad9361_init(&ad9361_phy_b, &default_init_param);

	ad9361_set_tx_fir_config(ad9361_phy_b, tx_fir_config);
	ad9361_set_rx_fir_config(ad9361_phy_b, rx_fir_config);
#endif

#ifndef AXI_ADC_NOT_PRESENT
	#if defined XILINX_PLATFORM || defined LINUX_PLATFORM || defined ALTERA_PLATFORM
		#ifdef DAC_DMA_EXAMPLE
			#ifdef FMCOMMS5
				dac_init(ad9361_phy_b, DATA_SEL_DMA, 0);
			#endif
			dac_init(ad9361_phy, DATA_SEL_DMA, 1);
		#else
			#ifdef FMCOMMS5
				dac_init(ad9361_phy_b, DATA_SEL_DDS, 0);
			#endif
		    dac_init(ad9361_phy, DATA_SEL_DDS, 1);
		#endif
	#endif
#endif

#ifdef FMCOMMS5
	ad9361_do_mcs(ad9361_phy, ad9361_phy_b);
#endif

#ifndef AXI_ADC_NOT_PRESENT
	#if (defined XILINX_PLATFORM || defined ALTERA_PLATFORM) && \
		(defined ADC_DMA_EXAMPLE || defined ADC_DMA_IRQ_EXAMPLE)
		// NOTE: To prevent unwanted data loss, it's recommended to invalidate
		// cache after each adc_capture() call, keeping in mind that the
		// size of the capture and the start address must be alinged to the size
		// of the cache line.
		mdelay(1000);
		adc_capture(16384, ADC_DDR_BASEADDR);
		#ifdef XILINX_PLATFORM
			#ifdef FMCOMMS5
				Xil_DCacheInvalidateRange(ADC_DDR_BASEADDR, 16384 * 16);
			#else
				Xil_DCacheInvalidateRange(ADC_DDR_BASEADDR,
						ad9361_phy->pdata->rx2tx2 ? 16384 * 8 : 16384 * 4);
			#endif
		#endif
	#endif
#endif
//dac_init(ad9361_phy, DATA_SEL_PN15, 1);
#if 0
#ifdef CONSOLE_COMMANDS
	get_help(NULL, 0);
	while(1)
	{
		console_get_command(received_cmd);
		invalid_cmd = 0;
		for(cmd = 0; cmd < cmd_no; cmd++)
		{
			param_no = 0;
			//printf("step4\n");
			cmd_type = console_check_commands(received_cmd, cmd_list[cmd].name,
											  param, &param_no);
			if(cmd_type == UNKNOWN_CMD)
			{
				invalid_cmd++;
			}
			else
			{
				cmd_list[cmd].function(param, param_no);
			}
		}
		if(invalid_cmd == cmd_no)
		{
			printf("Invalid command!\n");
		}
	}
#endif
#endif
	printf("Done.\n");

#ifdef TDD_SWITCH_STATE_EXAMPLE
	uint32_t ensm_mode;
	printf("enter TDD_SWITCH_STATE_EXAMPLE.\n");
	if (!ad9361_phy->pdata->fdd) {
		printf("!ad9361_phy->pdata->fdd.\n");
		if (ad9361_phy->pdata->ensm_pin_ctrl) {
			printf("ad9361_phy->pdata->ensm_pin_ctrl.\n");
			gpio_direction(GPIO_ENABLE_PIN, 1);
			gpio_direction(GPIO_TXNRX_PIN, 1);
			gpio_set_value(GPIO_ENABLE_PIN, 0);
			gpio_set_value(GPIO_TXNRX_PIN, 0);
			udelay(10);
			ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
			printf("TXNRX control - Alert: %s\n",
					ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
			mdelay(1000);

			if (ad9361_phy->pdata->ensm_pin_pulse_mode) {
				while(1) {
					gpio_set_value(GPIO_TXNRX_PIN, 0);
					udelay(10);
					gpio_set_value(GPIO_ENABLE_PIN, 1);
					udelay(10);
					gpio_set_value(GPIO_ENABLE_PIN, 0);
					ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
					printf("TXNRX Pulse control - RX: %s\n",
							ensm_mode == ENSM_MODE_RX ? "OK" : "Error");
					mdelay(1000);

					gpio_set_value(GPIO_ENABLE_PIN, 1);
					udelay(10);
					gpio_set_value(GPIO_ENABLE_PIN, 0);
					ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
					printf("TXNRX Pulse control - Alert: %s\n",
							ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
					mdelay(1000);

					gpio_set_value(GPIO_TXNRX_PIN, 1);
					udelay(10);
					gpio_set_value(GPIO_ENABLE_PIN, 1);
					udelay(10);
					gpio_set_value(GPIO_ENABLE_PIN, 0);
					ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
					printf("TXNRX Pulse control - TX: %s\n",
							ensm_mode == ENSM_MODE_TX ? "OK" : "Error");
					mdelay(1000);

					gpio_set_value(GPIO_ENABLE_PIN, 1);
					udelay(10);
					gpio_set_value(GPIO_ENABLE_PIN, 0);
					ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
					printf("TXNRX Pulse control - Alert: %s\n",
							ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
					mdelay(1000);
				}
			} else {
				while(1) {
					gpio_set_value(GPIO_TXNRX_PIN, 0);
					udelay(10);
					gpio_set_value(GPIO_ENABLE_PIN, 1);
					udelay(10);
					ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
					printf("TXNRX control - RX: %s\n",
							ensm_mode == ENSM_MODE_RX ? "OK" : "Error");
					mdelay(1000);

					gpio_set_value(GPIO_ENABLE_PIN, 0);
					udelay(10);
					ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
					printf("TXNRX control - Alert: %s\n",
							ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
					mdelay(1000);

					gpio_set_value(GPIO_TXNRX_PIN, 1);
					udelay(10);
					gpio_set_value(GPIO_ENABLE_PIN, 1);
					udelay(10);
					ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
					printf("TXNRX control - TX: %s\n",
							ensm_mode == ENSM_MODE_TX ? "OK" : "Error");
					mdelay(1000);

					gpio_set_value(GPIO_ENABLE_PIN, 0);
					udelay(10);
					ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
					printf("TXNRX control - Alert: %s\n",
							ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
					mdelay(1000);
				}
			}
		} else {
			printf("!ad9361_phy->pdata->ensm_pin_ctrl.\n");
			while(1) {
				printf("ad9361_set_en_state_machine_mode.\n");
				ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_MODE_RX);
				ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
				printf("SPI control - RX: %s\n",
						ensm_mode == ENSM_MODE_RX ? "OK" : "Error");
				mdelay(1000);

				ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_MODE_ALERT);
				ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
				printf("SPI control - Alert: %s\n",
						ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
				mdelay(1000);

				ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_MODE_TX);
				ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
				printf("SPI control - TX: %s\n",
						ensm_mode == ENSM_MODE_TX ? "OK" : "Error");
				mdelay(1000);

				ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_MODE_ALERT);
				ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
				printf("SPI control - Alert: %s\n",
						ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
				mdelay(1000);
			}
		}
	}
#endif


#ifdef ALTERA_PLATFORM
	if (altera_bridge_uninit()) {
		printf("Altera Bridge Uninit Error!\n");
		return -1;
	}
#endif

	//return 0;
}
