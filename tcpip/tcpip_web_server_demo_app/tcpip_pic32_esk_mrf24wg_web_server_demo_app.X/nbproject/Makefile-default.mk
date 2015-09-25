#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_mrf24wg_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_mrf24wg_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../../../bsp/pic32_meb/bsp.c ../../../microchip/common/lfsr.c ../../../microchip/common/hashes.c ../../../microchip/common/big_int.c ../../../microchip/common/big_int_helper_c32.S ../../../microchip/common/helpers.c ../berkeley_tcp_client_demo.c ../berkeley_tcp_server_demo.c ../berkeley_udp_client_demo.c ../generic_tcp_client.c ../generic_tcp_server.c ../ping_demo.c ../smtp_demo.c ../../../microchip/system/system_services.c ../../../microchip/system/system_debug.c ../../../microchip/system/system_random.c ../../../microchip/system/system_command.c ../../../microchip/system/system_userio.c ../../../microchip/system/fs/sys_fs.c ../../../microchip/system/fs/mpfs2.c ../../../microchip/system/drivers/usart.c ../../../microchip/system/drivers/lcd.c ../../../microchip/system/drivers/drv_spi.c ../../../microchip/system/drivers/db_appio.c ../../../microchip/system/drivers/spi_ram.c ../../../microchip/system/drivers/spi_eeprom.c ../../../microchip/system/drivers/spi_flash.c ../../../microchip/system/drivers/drv_media.c ../../../microchip/tcpip/tcpip_announce.c ../../../microchip/tcpip/http2.c ../../../microchip/tcpip/arcfour.c ../../../microchip/tcpip/arp.c ../../../microchip/tcpip/berkeley_api.c ../../../microchip/tcpip/dhcp.c ../../../microchip/tcpip/dhcps.c ../../../microchip/tcpip/dns.c ../../../microchip/tcpip/dnss.c ../../../microchip/tcpip/dyn_dns.c ../../../microchip/tcpip/enc28_mac.c ../../../microchip/tcpip/enc28j60.c ../../../microchip/tcpip/encs24j600.c ../../../microchip/tcpip/encx24_mac.c ../../../microchip/tcpip/eth_pic32_ext_phy.c ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c ../../../microchip/tcpip/eth_pic32_int_mac.c ../../../microchip/tcpip/ftp.c ../../../microchip/tcpip/hash_fnv.c ../../../microchip/tcpip/hash_tbl.c ../../../microchip/tcpip/icmp.c ../../../microchip/tcpip/icmpv6.c ../../../microchip/tcpip/ipv4.c ../../../microchip/tcpip/ipv6.c ../../../microchip/tcpip/mac_events_pic32.c ../../../microchip/tcpip/nbns.c ../../../microchip/tcpip/ndp.c ../../../microchip/tcpip/tcpip_reboot.c ../../../microchip/tcpip/rsa.c ../../../microchip/tcpip/smtp.c ../../../microchip/tcpip/snmp.c ../../../microchip/tcpip/snmpv3.c ../../../microchip/tcpip/snmpv3_usm.c ../../../microchip/tcpip/sntp.c ../../../microchip/tcpip/ssl.c ../../../microchip/tcpip/tcp.c ../../../microchip/tcpip/tcpip_heap_alloc.c ../../../microchip/tcpip/tcpip_storage.c ../../../microchip/tcpip/telnet.c ../../../microchip/tcpip/tftpc.c ../../../microchip/tcpip/udp.c ../../../microchip/tcpip/tcpip_mac_object.c ../../../microchip/tcpip/tcpip_manager.c ../../../microchip/tcpip/tcpip_helpers.c ../../../microchip/tcpip/tcpip_commands.c ../../../microchip/tcpip/iperf.c ../../../microchip/tcpip/tcpip_helper_c32.S ../../../microchip/tcpip/tcpip_notify.c ../../../microchip/tcpip/zero_conf_helper.c ../../../microchip/tcpip/zero_conf_link_local.c ../../../microchip/tcpip/zero_conf_multicast_dns.c ../../../microchip/tcpip/wifi/wf_tx_power.c ../../../microchip/tcpip/wifi/wf_connect.c ../../../microchip/tcpip/wifi/wf_connection_algorithm.c ../../../microchip/tcpip/wifi/wf_connection_manager.c ../../../microchip/tcpip/wifi/wf_connection_profile.c ../../../microchip/tcpip/wifi/wf_easy_config.c ../../../microchip/tcpip/wifi/wf_eint.c ../../../microchip/tcpip/wifi/wf_event_handler.c ../../../microchip/tcpip/wifi/wf_init.c ../../../microchip/tcpip/wifi/wf_power_save.c ../../../microchip/tcpip/wifi/wf_scan.c ../../../microchip/tcpip/wifi/wf_spi.c ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c ../../../microchip/tcpip/wifi/mrf24w_events.c ../../../microchip/tcpip/wifi/wf_debug_output.c ../../../microchip/tcpip/wifi/wf_commands.c ../../../microchip/tcpip/wifi/wf_pbkdf2.c ../../../microchip/tcpip/wifi/wf_driver_com_24g.c ../../../microchip/tcpip/wifi/wf_driver_raw_24g.c ../../../microchip/tcpip/wifi/wf_mac_24g.c ../../../microchip/tcpip/wifi/wf_mgmt_msg_24g.c ../../../microchip/tcpip/wifi/wf_param_msg_24g.c ../../../microchip/tcpip/wifi/wf_configData.c ../main_demo.c ../custom_snmp_app.c ../custom_ssl_cert.c ../mpfs_img2.c ../wf_config.c ../custom_http_app.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/979510709/bsp.o ${OBJECTDIR}/_ext/101875047/lfsr.o ${OBJECTDIR}/_ext/101875047/hashes.o ${OBJECTDIR}/_ext/101875047/big_int.o ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o ${OBJECTDIR}/_ext/101875047/helpers.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ${OBJECTDIR}/_ext/1472/ping_demo.o ${OBJECTDIR}/_ext/1472/smtp_demo.o ${OBJECTDIR}/_ext/365611741/system_services.o ${OBJECTDIR}/_ext/365611741/system_debug.o ${OBJECTDIR}/_ext/365611741/system_random.o ${OBJECTDIR}/_ext/365611741/system_command.o ${OBJECTDIR}/_ext/365611741/system_userio.o ${OBJECTDIR}/_ext/97638081/sys_fs.o ${OBJECTDIR}/_ext/97638081/mpfs2.o ${OBJECTDIR}/_ext/792872985/usart.o ${OBJECTDIR}/_ext/792872985/lcd.o ${OBJECTDIR}/_ext/792872985/drv_spi.o ${OBJECTDIR}/_ext/792872985/db_appio.o ${OBJECTDIR}/_ext/792872985/spi_ram.o ${OBJECTDIR}/_ext/792872985/spi_eeprom.o ${OBJECTDIR}/_ext/792872985/spi_flash.o ${OBJECTDIR}/_ext/792872985/drv_media.o ${OBJECTDIR}/_ext/427700826/tcpip_announce.o ${OBJECTDIR}/_ext/427700826/http2.o ${OBJECTDIR}/_ext/427700826/arcfour.o ${OBJECTDIR}/_ext/427700826/arp.o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ${OBJECTDIR}/_ext/427700826/dhcp.o ${OBJECTDIR}/_ext/427700826/dhcps.o ${OBJECTDIR}/_ext/427700826/dns.o ${OBJECTDIR}/_ext/427700826/dnss.o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ${OBJECTDIR}/_ext/427700826/enc28j60.o ${OBJECTDIR}/_ext/427700826/encs24j600.o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o ${OBJECTDIR}/_ext/427700826/ftp.o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ${OBJECTDIR}/_ext/427700826/icmp.o ${OBJECTDIR}/_ext/427700826/icmpv6.o ${OBJECTDIR}/_ext/427700826/ipv4.o ${OBJECTDIR}/_ext/427700826/ipv6.o ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o ${OBJECTDIR}/_ext/427700826/nbns.o ${OBJECTDIR}/_ext/427700826/ndp.o ${OBJECTDIR}/_ext/427700826/tcpip_reboot.o ${OBJECTDIR}/_ext/427700826/rsa.o ${OBJECTDIR}/_ext/427700826/smtp.o ${OBJECTDIR}/_ext/427700826/snmp.o ${OBJECTDIR}/_ext/427700826/snmpv3.o ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o ${OBJECTDIR}/_ext/427700826/sntp.o ${OBJECTDIR}/_ext/427700826/ssl.o ${OBJECTDIR}/_ext/427700826/tcp.o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ${OBJECTDIR}/_ext/427700826/telnet.o ${OBJECTDIR}/_ext/427700826/tftpc.o ${OBJECTDIR}/_ext/427700826/udp.o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ${OBJECTDIR}/_ext/427700826/tcpip_commands.o ${OBJECTDIR}/_ext/427700826/iperf.o ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o ${OBJECTDIR}/_ext/427700826/tcpip_notify.o ${OBJECTDIR}/_ext/427700826/zero_conf_helper.o ${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o ${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o ${OBJECTDIR}/_ext/334706090/wf_tx_power.o ${OBJECTDIR}/_ext/334706090/wf_connect.o ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o ${OBJECTDIR}/_ext/334706090/wf_easy_config.o ${OBJECTDIR}/_ext/334706090/wf_eint.o ${OBJECTDIR}/_ext/334706090/wf_event_handler.o ${OBJECTDIR}/_ext/334706090/wf_init.o ${OBJECTDIR}/_ext/334706090/wf_power_save.o ${OBJECTDIR}/_ext/334706090/wf_scan.o ${OBJECTDIR}/_ext/334706090/wf_spi.o ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o ${OBJECTDIR}/_ext/334706090/mrf24w_events.o ${OBJECTDIR}/_ext/334706090/wf_debug_output.o ${OBJECTDIR}/_ext/334706090/wf_commands.o ${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o ${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o ${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o ${OBJECTDIR}/_ext/334706090/wf_mac_24g.o ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o ${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o ${OBJECTDIR}/_ext/334706090/wf_configData.o ${OBJECTDIR}/_ext/1472/main_demo.o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ${OBJECTDIR}/_ext/1472/mpfs_img2.o ${OBJECTDIR}/_ext/1472/wf_config.o ${OBJECTDIR}/_ext/1472/custom_http_app.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/979510709/bsp.o.d ${OBJECTDIR}/_ext/101875047/lfsr.o.d ${OBJECTDIR}/_ext/101875047/hashes.o.d ${OBJECTDIR}/_ext/101875047/big_int.o.d ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d ${OBJECTDIR}/_ext/101875047/helpers.o.d ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d ${OBJECTDIR}/_ext/1472/ping_demo.o.d ${OBJECTDIR}/_ext/1472/smtp_demo.o.d ${OBJECTDIR}/_ext/365611741/system_services.o.d ${OBJECTDIR}/_ext/365611741/system_debug.o.d ${OBJECTDIR}/_ext/365611741/system_random.o.d ${OBJECTDIR}/_ext/365611741/system_command.o.d ${OBJECTDIR}/_ext/365611741/system_userio.o.d ${OBJECTDIR}/_ext/97638081/sys_fs.o.d ${OBJECTDIR}/_ext/97638081/mpfs2.o.d ${OBJECTDIR}/_ext/792872985/usart.o.d ${OBJECTDIR}/_ext/792872985/lcd.o.d ${OBJECTDIR}/_ext/792872985/drv_spi.o.d ${OBJECTDIR}/_ext/792872985/db_appio.o.d ${OBJECTDIR}/_ext/792872985/spi_ram.o.d ${OBJECTDIR}/_ext/792872985/spi_eeprom.o.d ${OBJECTDIR}/_ext/792872985/spi_flash.o.d ${OBJECTDIR}/_ext/792872985/drv_media.o.d ${OBJECTDIR}/_ext/427700826/tcpip_announce.o.d ${OBJECTDIR}/_ext/427700826/http2.o.d ${OBJECTDIR}/_ext/427700826/arcfour.o.d ${OBJECTDIR}/_ext/427700826/arp.o.d ${OBJECTDIR}/_ext/427700826/berkeley_api.o.d ${OBJECTDIR}/_ext/427700826/dhcp.o.d ${OBJECTDIR}/_ext/427700826/dhcps.o.d ${OBJECTDIR}/_ext/427700826/dns.o.d ${OBJECTDIR}/_ext/427700826/dnss.o.d ${OBJECTDIR}/_ext/427700826/dyn_dns.o.d ${OBJECTDIR}/_ext/427700826/enc28_mac.o.d ${OBJECTDIR}/_ext/427700826/enc28j60.o.d ${OBJECTDIR}/_ext/427700826/encs24j600.o.d ${OBJECTDIR}/_ext/427700826/encx24_mac.o.d ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d ${OBJECTDIR}/_ext/427700826/ftp.o.d ${OBJECTDIR}/_ext/427700826/hash_fnv.o.d ${OBJECTDIR}/_ext/427700826/hash_tbl.o.d ${OBJECTDIR}/_ext/427700826/icmp.o.d ${OBJECTDIR}/_ext/427700826/icmpv6.o.d ${OBJECTDIR}/_ext/427700826/ipv4.o.d ${OBJECTDIR}/_ext/427700826/ipv6.o.d ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d ${OBJECTDIR}/_ext/427700826/nbns.o.d ${OBJECTDIR}/_ext/427700826/ndp.o.d ${OBJECTDIR}/_ext/427700826/tcpip_reboot.o.d ${OBJECTDIR}/_ext/427700826/rsa.o.d ${OBJECTDIR}/_ext/427700826/smtp.o.d ${OBJECTDIR}/_ext/427700826/snmp.o.d ${OBJECTDIR}/_ext/427700826/snmpv3.o.d ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d ${OBJECTDIR}/_ext/427700826/sntp.o.d ${OBJECTDIR}/_ext/427700826/ssl.o.d ${OBJECTDIR}/_ext/427700826/tcp.o.d ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d ${OBJECTDIR}/_ext/427700826/telnet.o.d ${OBJECTDIR}/_ext/427700826/tftpc.o.d ${OBJECTDIR}/_ext/427700826/udp.o.d ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d ${OBJECTDIR}/_ext/427700826/tcpip_commands.o.d ${OBJECTDIR}/_ext/427700826/iperf.o.d ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.d ${OBJECTDIR}/_ext/427700826/tcpip_notify.o.d ${OBJECTDIR}/_ext/427700826/zero_conf_helper.o.d ${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o.d ${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o.d ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d ${OBJECTDIR}/_ext/334706090/wf_connect.o.d ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d ${OBJECTDIR}/_ext/334706090/wf_eint.o.d ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d ${OBJECTDIR}/_ext/334706090/wf_init.o.d ${OBJECTDIR}/_ext/334706090/wf_power_save.o.d ${OBJECTDIR}/_ext/334706090/wf_scan.o.d ${OBJECTDIR}/_ext/334706090/wf_spi.o.d ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d ${OBJECTDIR}/_ext/334706090/wf_debug_output.o.d ${OBJECTDIR}/_ext/334706090/wf_commands.o.d ${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o.d ${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o.d ${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o.d ${OBJECTDIR}/_ext/334706090/wf_mac_24g.o.d ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o.d ${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o.d ${OBJECTDIR}/_ext/334706090/wf_configData.o.d ${OBJECTDIR}/_ext/1472/main_demo.o.d ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d ${OBJECTDIR}/_ext/1472/mpfs_img2.o.d ${OBJECTDIR}/_ext/1472/wf_config.o.d ${OBJECTDIR}/_ext/1472/custom_http_app.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/979510709/bsp.o ${OBJECTDIR}/_ext/101875047/lfsr.o ${OBJECTDIR}/_ext/101875047/hashes.o ${OBJECTDIR}/_ext/101875047/big_int.o ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o ${OBJECTDIR}/_ext/101875047/helpers.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ${OBJECTDIR}/_ext/1472/ping_demo.o ${OBJECTDIR}/_ext/1472/smtp_demo.o ${OBJECTDIR}/_ext/365611741/system_services.o ${OBJECTDIR}/_ext/365611741/system_debug.o ${OBJECTDIR}/_ext/365611741/system_random.o ${OBJECTDIR}/_ext/365611741/system_command.o ${OBJECTDIR}/_ext/365611741/system_userio.o ${OBJECTDIR}/_ext/97638081/sys_fs.o ${OBJECTDIR}/_ext/97638081/mpfs2.o ${OBJECTDIR}/_ext/792872985/usart.o ${OBJECTDIR}/_ext/792872985/lcd.o ${OBJECTDIR}/_ext/792872985/drv_spi.o ${OBJECTDIR}/_ext/792872985/db_appio.o ${OBJECTDIR}/_ext/792872985/spi_ram.o ${OBJECTDIR}/_ext/792872985/spi_eeprom.o ${OBJECTDIR}/_ext/792872985/spi_flash.o ${OBJECTDIR}/_ext/792872985/drv_media.o ${OBJECTDIR}/_ext/427700826/tcpip_announce.o ${OBJECTDIR}/_ext/427700826/http2.o ${OBJECTDIR}/_ext/427700826/arcfour.o ${OBJECTDIR}/_ext/427700826/arp.o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ${OBJECTDIR}/_ext/427700826/dhcp.o ${OBJECTDIR}/_ext/427700826/dhcps.o ${OBJECTDIR}/_ext/427700826/dns.o ${OBJECTDIR}/_ext/427700826/dnss.o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ${OBJECTDIR}/_ext/427700826/enc28j60.o ${OBJECTDIR}/_ext/427700826/encs24j600.o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o ${OBJECTDIR}/_ext/427700826/ftp.o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ${OBJECTDIR}/_ext/427700826/icmp.o ${OBJECTDIR}/_ext/427700826/icmpv6.o ${OBJECTDIR}/_ext/427700826/ipv4.o ${OBJECTDIR}/_ext/427700826/ipv6.o ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o ${OBJECTDIR}/_ext/427700826/nbns.o ${OBJECTDIR}/_ext/427700826/ndp.o ${OBJECTDIR}/_ext/427700826/tcpip_reboot.o ${OBJECTDIR}/_ext/427700826/rsa.o ${OBJECTDIR}/_ext/427700826/smtp.o ${OBJECTDIR}/_ext/427700826/snmp.o ${OBJECTDIR}/_ext/427700826/snmpv3.o ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o ${OBJECTDIR}/_ext/427700826/sntp.o ${OBJECTDIR}/_ext/427700826/ssl.o ${OBJECTDIR}/_ext/427700826/tcp.o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ${OBJECTDIR}/_ext/427700826/telnet.o ${OBJECTDIR}/_ext/427700826/tftpc.o ${OBJECTDIR}/_ext/427700826/udp.o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ${OBJECTDIR}/_ext/427700826/tcpip_commands.o ${OBJECTDIR}/_ext/427700826/iperf.o ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o ${OBJECTDIR}/_ext/427700826/tcpip_notify.o ${OBJECTDIR}/_ext/427700826/zero_conf_helper.o ${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o ${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o ${OBJECTDIR}/_ext/334706090/wf_tx_power.o ${OBJECTDIR}/_ext/334706090/wf_connect.o ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o ${OBJECTDIR}/_ext/334706090/wf_easy_config.o ${OBJECTDIR}/_ext/334706090/wf_eint.o ${OBJECTDIR}/_ext/334706090/wf_event_handler.o ${OBJECTDIR}/_ext/334706090/wf_init.o ${OBJECTDIR}/_ext/334706090/wf_power_save.o ${OBJECTDIR}/_ext/334706090/wf_scan.o ${OBJECTDIR}/_ext/334706090/wf_spi.o ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o ${OBJECTDIR}/_ext/334706090/mrf24w_events.o ${OBJECTDIR}/_ext/334706090/wf_debug_output.o ${OBJECTDIR}/_ext/334706090/wf_commands.o ${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o ${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o ${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o ${OBJECTDIR}/_ext/334706090/wf_mac_24g.o ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o ${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o ${OBJECTDIR}/_ext/334706090/wf_configData.o ${OBJECTDIR}/_ext/1472/main_demo.o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ${OBJECTDIR}/_ext/1472/mpfs_img2.o ${OBJECTDIR}/_ext/1472/wf_config.o ${OBJECTDIR}/_ext/1472/custom_http_app.o

# Source Files
SOURCEFILES=../../../bsp/pic32_meb/bsp.c ../../../microchip/common/lfsr.c ../../../microchip/common/hashes.c ../../../microchip/common/big_int.c ../../../microchip/common/big_int_helper_c32.S ../../../microchip/common/helpers.c ../berkeley_tcp_client_demo.c ../berkeley_tcp_server_demo.c ../berkeley_udp_client_demo.c ../generic_tcp_client.c ../generic_tcp_server.c ../ping_demo.c ../smtp_demo.c ../../../microchip/system/system_services.c ../../../microchip/system/system_debug.c ../../../microchip/system/system_random.c ../../../microchip/system/system_command.c ../../../microchip/system/system_userio.c ../../../microchip/system/fs/sys_fs.c ../../../microchip/system/fs/mpfs2.c ../../../microchip/system/drivers/usart.c ../../../microchip/system/drivers/lcd.c ../../../microchip/system/drivers/drv_spi.c ../../../microchip/system/drivers/db_appio.c ../../../microchip/system/drivers/spi_ram.c ../../../microchip/system/drivers/spi_eeprom.c ../../../microchip/system/drivers/spi_flash.c ../../../microchip/system/drivers/drv_media.c ../../../microchip/tcpip/tcpip_announce.c ../../../microchip/tcpip/http2.c ../../../microchip/tcpip/arcfour.c ../../../microchip/tcpip/arp.c ../../../microchip/tcpip/berkeley_api.c ../../../microchip/tcpip/dhcp.c ../../../microchip/tcpip/dhcps.c ../../../microchip/tcpip/dns.c ../../../microchip/tcpip/dnss.c ../../../microchip/tcpip/dyn_dns.c ../../../microchip/tcpip/enc28_mac.c ../../../microchip/tcpip/enc28j60.c ../../../microchip/tcpip/encs24j600.c ../../../microchip/tcpip/encx24_mac.c ../../../microchip/tcpip/eth_pic32_ext_phy.c ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c ../../../microchip/tcpip/eth_pic32_int_mac.c ../../../microchip/tcpip/ftp.c ../../../microchip/tcpip/hash_fnv.c ../../../microchip/tcpip/hash_tbl.c ../../../microchip/tcpip/icmp.c ../../../microchip/tcpip/icmpv6.c ../../../microchip/tcpip/ipv4.c ../../../microchip/tcpip/ipv6.c ../../../microchip/tcpip/mac_events_pic32.c ../../../microchip/tcpip/nbns.c ../../../microchip/tcpip/ndp.c ../../../microchip/tcpip/tcpip_reboot.c ../../../microchip/tcpip/rsa.c ../../../microchip/tcpip/smtp.c ../../../microchip/tcpip/snmp.c ../../../microchip/tcpip/snmpv3.c ../../../microchip/tcpip/snmpv3_usm.c ../../../microchip/tcpip/sntp.c ../../../microchip/tcpip/ssl.c ../../../microchip/tcpip/tcp.c ../../../microchip/tcpip/tcpip_heap_alloc.c ../../../microchip/tcpip/tcpip_storage.c ../../../microchip/tcpip/telnet.c ../../../microchip/tcpip/tftpc.c ../../../microchip/tcpip/udp.c ../../../microchip/tcpip/tcpip_mac_object.c ../../../microchip/tcpip/tcpip_manager.c ../../../microchip/tcpip/tcpip_helpers.c ../../../microchip/tcpip/tcpip_commands.c ../../../microchip/tcpip/iperf.c ../../../microchip/tcpip/tcpip_helper_c32.S ../../../microchip/tcpip/tcpip_notify.c ../../../microchip/tcpip/zero_conf_helper.c ../../../microchip/tcpip/zero_conf_link_local.c ../../../microchip/tcpip/zero_conf_multicast_dns.c ../../../microchip/tcpip/wifi/wf_tx_power.c ../../../microchip/tcpip/wifi/wf_connect.c ../../../microchip/tcpip/wifi/wf_connection_algorithm.c ../../../microchip/tcpip/wifi/wf_connection_manager.c ../../../microchip/tcpip/wifi/wf_connection_profile.c ../../../microchip/tcpip/wifi/wf_easy_config.c ../../../microchip/tcpip/wifi/wf_eint.c ../../../microchip/tcpip/wifi/wf_event_handler.c ../../../microchip/tcpip/wifi/wf_init.c ../../../microchip/tcpip/wifi/wf_power_save.c ../../../microchip/tcpip/wifi/wf_scan.c ../../../microchip/tcpip/wifi/wf_spi.c ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c ../../../microchip/tcpip/wifi/mrf24w_events.c ../../../microchip/tcpip/wifi/wf_debug_output.c ../../../microchip/tcpip/wifi/wf_commands.c ../../../microchip/tcpip/wifi/wf_pbkdf2.c ../../../microchip/tcpip/wifi/wf_driver_com_24g.c ../../../microchip/tcpip/wifi/wf_driver_raw_24g.c ../../../microchip/tcpip/wifi/wf_mac_24g.c ../../../microchip/tcpip/wifi/wf_mgmt_msg_24g.c ../../../microchip/tcpip/wifi/wf_param_msg_24g.c ../../../microchip/tcpip/wifi/wf_configData.c ../main_demo.c ../custom_snmp_app.c ../custom_ssl_cert.c ../mpfs_img2.c ../wf_config.c ../custom_http_app.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_mrf24wg_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o: ../../../microchip/common/big_int_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.ok ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d" "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -DPIC32_ESK_MEB_ETH_WIFI -I"../configs/pic32_meb/tcpip_profile" -I"../configs/pic32_meb/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/include/common" -I"../../../microchip/include/system" -I"../../../microchip/include/system/fs" -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o ../../../microchip/common/big_int_helper_c32.S  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PIC32MXSK=1,-I".."
	
${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o: ../../../microchip/tcpip/tcpip_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.d" "${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -DPIC32_ESK_MEB_ETH_WIFI -I"../configs/pic32_meb/tcpip_profile" -I"../configs/pic32_meb/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/include/common" -I"../../../microchip/include/system" -I"../../../microchip/include/system/fs" -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o ../../../microchip/tcpip/tcpip_helper_c32.S  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PIC32MXSK=1,-I".."
	
else
${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o: ../../../microchip/common/big_int_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.ok ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d" "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -DPIC32_ESK_MEB_ETH_WIFI -I"../configs/pic32_meb/tcpip_profile" -I"../configs/pic32_meb/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/include/common" -I"../../../microchip/include/system" -I"../../../microchip/include/system/fs" -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o ../../../microchip/common/big_int_helper_c32.S  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.asm.d",--gdwarf-2,-I".."
	
${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o: ../../../microchip/tcpip/tcpip_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.d" "${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -DPIC32_ESK_MEB_ETH_WIFI -I"../configs/pic32_meb/tcpip_profile" -I"../configs/pic32_meb/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/include/common" -I"../../../microchip/include/system" -I"../../../microchip/include/system/fs" -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o ../../../microchip/tcpip/tcpip_helper_c32.S  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/427700826/tcpip_helper_c32.o.asm.d",--gdwarf-2,-I".."
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/979510709/bsp.o: ../../../bsp/pic32_meb/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/979510709 
	@${RM} ${OBJECTDIR}/_ext/979510709/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/979510709/bsp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/979510709/bsp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/979510709/bsp.o.d" -o ${OBJECTDIR}/_ext/979510709/bsp.o ../../../bsp/pic32_meb/bsp.c    -G 64
	
${OBJECTDIR}/_ext/101875047/lfsr.o: ../../../microchip/common/lfsr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/lfsr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/lfsr.o.d" -o ${OBJECTDIR}/_ext/101875047/lfsr.o ../../../microchip/common/lfsr.c    -G 64
	
${OBJECTDIR}/_ext/101875047/hashes.o: ../../../microchip/common/hashes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/hashes.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/hashes.o.d" -o ${OBJECTDIR}/_ext/101875047/hashes.o ../../../microchip/common/hashes.c    -G 64
	
${OBJECTDIR}/_ext/101875047/big_int.o: ../../../microchip/common/big_int.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int.o.d" -o ${OBJECTDIR}/_ext/101875047/big_int.o ../../../microchip/common/big_int.c    -G 64
	
${OBJECTDIR}/_ext/101875047/helpers.o: ../../../microchip/common/helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/helpers.o.d" -o ${OBJECTDIR}/_ext/101875047/helpers.o ../../../microchip/common/helpers.c    -G 64
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o: ../berkeley_tcp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ../berkeley_tcp_client_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o: ../berkeley_tcp_server_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ../berkeley_tcp_server_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o: ../berkeley_udp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ../berkeley_udp_client_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/generic_tcp_client.o: ../generic_tcp_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ../generic_tcp_client.c    -G 64
	
${OBJECTDIR}/_ext/1472/generic_tcp_server.o: ../generic_tcp_server.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ../generic_tcp_server.c    -G 64
	
${OBJECTDIR}/_ext/1472/ping_demo.o: ../ping_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ping_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/ping_demo.o.d" -o ${OBJECTDIR}/_ext/1472/ping_demo.o ../ping_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/smtp_demo.o: ../smtp_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" -o ${OBJECTDIR}/_ext/1472/smtp_demo.o ../smtp_demo.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_services.o: ../../../microchip/system/system_services.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_services.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_services.o.d" -o ${OBJECTDIR}/_ext/365611741/system_services.o ../../../microchip/system/system_services.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_debug.o: ../../../microchip/system/system_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_debug.o.d" -o ${OBJECTDIR}/_ext/365611741/system_debug.o ../../../microchip/system/system_debug.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_random.o: ../../../microchip/system/system_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_random.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_random.o.d" -o ${OBJECTDIR}/_ext/365611741/system_random.o ../../../microchip/system/system_random.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_command.o: ../../../microchip/system/system_command.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_command.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_command.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_command.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_command.o.d" -o ${OBJECTDIR}/_ext/365611741/system_command.o ../../../microchip/system/system_command.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_userio.o: ../../../microchip/system/system_userio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_userio.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_userio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_userio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_userio.o.d" -o ${OBJECTDIR}/_ext/365611741/system_userio.o ../../../microchip/system/system_userio.c    -G 64
	
${OBJECTDIR}/_ext/97638081/sys_fs.o: ../../../microchip/system/fs/sys_fs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/97638081 
	@${RM} ${OBJECTDIR}/_ext/97638081/sys_fs.o.d 
	@${RM} ${OBJECTDIR}/_ext/97638081/sys_fs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/97638081/sys_fs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/97638081/sys_fs.o.d" -o ${OBJECTDIR}/_ext/97638081/sys_fs.o ../../../microchip/system/fs/sys_fs.c    -G 64
	
${OBJECTDIR}/_ext/97638081/mpfs2.o: ../../../microchip/system/fs/mpfs2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/97638081 
	@${RM} ${OBJECTDIR}/_ext/97638081/mpfs2.o.d 
	@${RM} ${OBJECTDIR}/_ext/97638081/mpfs2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/97638081/mpfs2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/97638081/mpfs2.o.d" -o ${OBJECTDIR}/_ext/97638081/mpfs2.o ../../../microchip/system/fs/mpfs2.c    -G 64
	
${OBJECTDIR}/_ext/792872985/usart.o: ../../../microchip/system/drivers/usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/usart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/usart.o.d" -o ${OBJECTDIR}/_ext/792872985/usart.o ../../../microchip/system/drivers/usart.c    -G 64
	
${OBJECTDIR}/_ext/792872985/lcd.o: ../../../microchip/system/drivers/lcd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/lcd.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/lcd.o.d" -o ${OBJECTDIR}/_ext/792872985/lcd.o ../../../microchip/system/drivers/lcd.c    -G 64
	
${OBJECTDIR}/_ext/792872985/drv_spi.o: ../../../microchip/system/drivers/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" -o ${OBJECTDIR}/_ext/792872985/drv_spi.o ../../../microchip/system/drivers/drv_spi.c    -G 64
	
${OBJECTDIR}/_ext/792872985/db_appio.o: ../../../microchip/system/drivers/db_appio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/db_appio.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/db_appio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/db_appio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/db_appio.o.d" -o ${OBJECTDIR}/_ext/792872985/db_appio.o ../../../microchip/system/drivers/db_appio.c    -G 64
	
${OBJECTDIR}/_ext/792872985/spi_ram.o: ../../../microchip/system/drivers/spi_ram.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_ram.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_ram.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/spi_ram.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/spi_ram.o.d" -o ${OBJECTDIR}/_ext/792872985/spi_ram.o ../../../microchip/system/drivers/spi_ram.c    -G 64
	
${OBJECTDIR}/_ext/792872985/spi_eeprom.o: ../../../microchip/system/drivers/spi_eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_eeprom.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/spi_eeprom.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/spi_eeprom.o.d" -o ${OBJECTDIR}/_ext/792872985/spi_eeprom.o ../../../microchip/system/drivers/spi_eeprom.c    -G 64
	
${OBJECTDIR}/_ext/792872985/spi_flash.o: ../../../microchip/system/drivers/spi_flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_flash.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_flash.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/spi_flash.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/spi_flash.o.d" -o ${OBJECTDIR}/_ext/792872985/spi_flash.o ../../../microchip/system/drivers/spi_flash.c    -G 64
	
${OBJECTDIR}/_ext/792872985/drv_media.o: ../../../microchip/system/drivers/drv_media.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_media.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_media.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/drv_media.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/drv_media.o.d" -o ${OBJECTDIR}/_ext/792872985/drv_media.o ../../../microchip/system/drivers/drv_media.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_announce.o: ../../../microchip/tcpip/tcpip_announce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_announce.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_announce.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_announce.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_announce.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_announce.o ../../../microchip/tcpip/tcpip_announce.c    -G 64
	
${OBJECTDIR}/_ext/427700826/http2.o: ../../../microchip/tcpip/http2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/http2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/http2.o.d" -o ${OBJECTDIR}/_ext/427700826/http2.o ../../../microchip/tcpip/http2.c    -G 64
	
${OBJECTDIR}/_ext/427700826/arcfour.o: ../../../microchip/tcpip/arcfour.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arcfour.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/arcfour.o.d" -o ${OBJECTDIR}/_ext/427700826/arcfour.o ../../../microchip/tcpip/arcfour.c    -G 64
	
${OBJECTDIR}/_ext/427700826/arp.o: ../../../microchip/tcpip/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/arp.o.d" -o ${OBJECTDIR}/_ext/427700826/arp.o ../../../microchip/tcpip/arp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/berkeley_api.o: ../../../microchip/tcpip/berkeley_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" -o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ../../../microchip/tcpip/berkeley_api.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dhcp.o: ../../../microchip/tcpip/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcp.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcp.o ../../../microchip/tcpip/dhcp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dhcps.o: ../../../microchip/tcpip/dhcps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcps.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcps.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcps.o ../../../microchip/tcpip/dhcps.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dns.o: ../../../microchip/tcpip/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dns.o ../../../microchip/tcpip/dns.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dnss.o: ../../../microchip/tcpip/dnss.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dnss.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dnss.o.d" -o ${OBJECTDIR}/_ext/427700826/dnss.o ../../../microchip/tcpip/dnss.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dyn_dns.o: ../../../microchip/tcpip/dyn_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ../../../microchip/tcpip/dyn_dns.c    -G 64
	
${OBJECTDIR}/_ext/427700826/enc28_mac.o: ../../../microchip/tcpip/enc28_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ../../../microchip/tcpip/enc28_mac.c    -G 64
	
${OBJECTDIR}/_ext/427700826/enc28j60.o: ../../../microchip/tcpip/enc28j60.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28j60.o ../../../microchip/tcpip/enc28j60.c    -G 64
	
${OBJECTDIR}/_ext/427700826/encs24j600.o: ../../../microchip/tcpip/encs24j600.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" -o ${OBJECTDIR}/_ext/427700826/encs24j600.o ../../../microchip/tcpip/encs24j600.c    -G 64
	
${OBJECTDIR}/_ext/427700826/encx24_mac.o: ../../../microchip/tcpip/encx24_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ../../../microchip/tcpip/encx24_mac.c    -G 64
	
${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o: ../../../microchip/tcpip/eth_pic32_ext_phy.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o ../../../microchip/tcpip/eth_pic32_ext_phy.c    -G 64
	
${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o: ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c    -G 64
	
${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o: ../../../microchip/tcpip/eth_pic32_int_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o ../../../microchip/tcpip/eth_pic32_int_mac.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ftp.o: ../../../microchip/tcpip/ftp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ftp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ftp.o.d" -o ${OBJECTDIR}/_ext/427700826/ftp.o ../../../microchip/tcpip/ftp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/hash_fnv.o: ../../../microchip/tcpip/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ../../../microchip/tcpip/hash_fnv.c    -G 64
	
${OBJECTDIR}/_ext/427700826/hash_tbl.o: ../../../microchip/tcpip/hash_tbl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ../../../microchip/tcpip/hash_tbl.c    -G 64
	
${OBJECTDIR}/_ext/427700826/icmp.o: ../../../microchip/tcpip/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/icmp.o.d" -o ${OBJECTDIR}/_ext/427700826/icmp.o ../../../microchip/tcpip/icmp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/icmpv6.o: ../../../microchip/tcpip/icmpv6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" -o ${OBJECTDIR}/_ext/427700826/icmpv6.o ../../../microchip/tcpip/icmpv6.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ipv4.o: ../../../microchip/tcpip/ipv4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ipv4.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ipv4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ipv4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ipv4.o.d" -o ${OBJECTDIR}/_ext/427700826/ipv4.o ../../../microchip/tcpip/ipv4.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ipv6.o: ../../../microchip/tcpip/ipv6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ipv6.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ipv6.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ipv6.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ipv6.o.d" -o ${OBJECTDIR}/_ext/427700826/ipv6.o ../../../microchip/tcpip/ipv6.c    -G 64
	
${OBJECTDIR}/_ext/427700826/mac_events_pic32.o: ../../../microchip/tcpip/mac_events_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d" -o ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o ../../../microchip/tcpip/mac_events_pic32.c    -G 64
	
${OBJECTDIR}/_ext/427700826/nbns.o: ../../../microchip/tcpip/nbns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/nbns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/nbns.o.d" -o ${OBJECTDIR}/_ext/427700826/nbns.o ../../../microchip/tcpip/nbns.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ndp.o: ../../../microchip/tcpip/ndp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ndp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ndp.o.d" -o ${OBJECTDIR}/_ext/427700826/ndp.o ../../../microchip/tcpip/ndp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_reboot.o: ../../../microchip/tcpip/tcpip_reboot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_reboot.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_reboot.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_reboot.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_reboot.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_reboot.o ../../../microchip/tcpip/tcpip_reboot.c    -G 64
	
${OBJECTDIR}/_ext/427700826/rsa.o: ../../../microchip/tcpip/rsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/rsa.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/rsa.o.d" -o ${OBJECTDIR}/_ext/427700826/rsa.o ../../../microchip/tcpip/rsa.c    -G 64
	
${OBJECTDIR}/_ext/427700826/smtp.o: ../../../microchip/tcpip/smtp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/smtp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/smtp.o.d" -o ${OBJECTDIR}/_ext/427700826/smtp.o ../../../microchip/tcpip/smtp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/snmp.o: ../../../microchip/tcpip/snmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/snmp.o.d" -o ${OBJECTDIR}/_ext/427700826/snmp.o ../../../microchip/tcpip/snmp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/snmpv3.o: ../../../microchip/tcpip/snmpv3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmpv3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/snmpv3.o.d" -o ${OBJECTDIR}/_ext/427700826/snmpv3.o ../../../microchip/tcpip/snmpv3.c    -G 64
	
${OBJECTDIR}/_ext/427700826/snmpv3_usm.o: ../../../microchip/tcpip/snmpv3_usm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d" -o ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o ../../../microchip/tcpip/snmpv3_usm.c    -G 64
	
${OBJECTDIR}/_ext/427700826/sntp.o: ../../../microchip/tcpip/sntp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/sntp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/sntp.o.d" -o ${OBJECTDIR}/_ext/427700826/sntp.o ../../../microchip/tcpip/sntp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ssl.o: ../../../microchip/tcpip/ssl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ssl.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ssl.o.d" -o ${OBJECTDIR}/_ext/427700826/ssl.o ../../../microchip/tcpip/ssl.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcp.o: ../../../microchip/tcpip/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp.o ../../../microchip/tcpip/tcp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o: ../../../microchip/tcpip/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ../../../microchip/tcpip/tcpip_heap_alloc.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_storage.o: ../../../microchip/tcpip/tcpip_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ../../../microchip/tcpip/tcpip_storage.c    -G 64
	
${OBJECTDIR}/_ext/427700826/telnet.o: ../../../microchip/tcpip/telnet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/telnet.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/telnet.o.d" -o ${OBJECTDIR}/_ext/427700826/telnet.o ../../../microchip/tcpip/telnet.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tftpc.o: ../../../microchip/tcpip/tftpc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tftpc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tftpc.o.d" -o ${OBJECTDIR}/_ext/427700826/tftpc.o ../../../microchip/tcpip/tftpc.c    -G 64
	
${OBJECTDIR}/_ext/427700826/udp.o: ../../../microchip/tcpip/udp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/udp.o.d" -o ${OBJECTDIR}/_ext/427700826/udp.o ../../../microchip/tcpip/udp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o: ../../../microchip/tcpip/tcpip_mac_object.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ../../../microchip/tcpip/tcpip_mac_object.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_manager.o: ../../../microchip/tcpip/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ../../../microchip/tcpip/tcpip_manager.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_helpers.o: ../../../microchip/tcpip/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ../../../microchip/tcpip/tcpip_helpers.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_commands.o: ../../../microchip/tcpip/tcpip_commands.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_commands.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_commands.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_commands.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_commands.o ../../../microchip/tcpip/tcpip_commands.c    -G 64
	
${OBJECTDIR}/_ext/427700826/iperf.o: ../../../microchip/tcpip/iperf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/iperf.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/iperf.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/iperf.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/iperf.o.d" -o ${OBJECTDIR}/_ext/427700826/iperf.o ../../../microchip/tcpip/iperf.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_notify.o: ../../../microchip/tcpip/tcpip_notify.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_notify.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_notify.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_notify.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_notify.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_notify.o ../../../microchip/tcpip/tcpip_notify.c    -G 64
	
${OBJECTDIR}/_ext/427700826/zero_conf_helper.o: ../../../microchip/tcpip/zero_conf_helper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_helper.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_helper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/zero_conf_helper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/zero_conf_helper.o.d" -o ${OBJECTDIR}/_ext/427700826/zero_conf_helper.o ../../../microchip/tcpip/zero_conf_helper.c    -G 64
	
${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o: ../../../microchip/tcpip/zero_conf_link_local.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o.d" -o ${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o ../../../microchip/tcpip/zero_conf_link_local.c    -G 64
	
${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o: ../../../microchip/tcpip/zero_conf_multicast_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o.d" -o ${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o ../../../microchip/tcpip/zero_conf_multicast_dns.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_tx_power.o: ../../../microchip/tcpip/wifi/wf_tx_power.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_tx_power.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_tx_power.o ../../../microchip/tcpip/wifi/wf_tx_power.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_connect.o: ../../../microchip/tcpip/wifi/wf_connect.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connect.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connect.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connect.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connect.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connect.o ../../../microchip/tcpip/wifi/wf_connect.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o: ../../../microchip/tcpip/wifi/wf_connection_algorithm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o ../../../microchip/tcpip/wifi/wf_connection_algorithm.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_connection_manager.o: ../../../microchip/tcpip/wifi/wf_connection_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o ../../../microchip/tcpip/wifi/wf_connection_manager.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_connection_profile.o: ../../../microchip/tcpip/wifi/wf_connection_profile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o ../../../microchip/tcpip/wifi/wf_connection_profile.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_easy_config.o: ../../../microchip/tcpip/wifi/wf_easy_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_easy_config.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_easy_config.o ../../../microchip/tcpip/wifi/wf_easy_config.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_eint.o: ../../../microchip/tcpip/wifi/wf_eint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_eint.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_eint.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_eint.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_eint.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_eint.o ../../../microchip/tcpip/wifi/wf_eint.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_event_handler.o: ../../../microchip/tcpip/wifi/wf_event_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_event_handler.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_event_handler.o ../../../microchip/tcpip/wifi/wf_event_handler.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_init.o: ../../../microchip/tcpip/wifi/wf_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_init.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_init.o ../../../microchip/tcpip/wifi/wf_init.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_power_save.o: ../../../microchip/tcpip/wifi/wf_power_save.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_power_save.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_power_save.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_power_save.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_power_save.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_power_save.o ../../../microchip/tcpip/wifi/wf_power_save.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_scan.o: ../../../microchip/tcpip/wifi/wf_scan.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_scan.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_scan.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_scan.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_scan.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_scan.o ../../../microchip/tcpip/wifi/wf_scan.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_spi.o: ../../../microchip/tcpip/wifi/wf_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_spi.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_spi.o ../../../microchip/tcpip/wifi/wf_spi.c    -G 64
	
${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o: ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d" -o ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c    -G 64
	
${OBJECTDIR}/_ext/334706090/mrf24w_events.o: ../../../microchip/tcpip/wifi/mrf24w_events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_events.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d" -o ${OBJECTDIR}/_ext/334706090/mrf24w_events.o ../../../microchip/tcpip/wifi/mrf24w_events.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_debug_output.o: ../../../microchip/tcpip/wifi/wf_debug_output.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_debug_output.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_debug_output.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_debug_output.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_debug_output.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_debug_output.o ../../../microchip/tcpip/wifi/wf_debug_output.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_commands.o: ../../../microchip/tcpip/wifi/wf_commands.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_commands.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_commands.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_commands.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_commands.o ../../../microchip/tcpip/wifi/wf_commands.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o: ../../../microchip/tcpip/wifi/wf_pbkdf2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o ../../../microchip/tcpip/wifi/wf_pbkdf2.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o: ../../../microchip/tcpip/wifi/wf_driver_com_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o ../../../microchip/tcpip/wifi/wf_driver_com_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o: ../../../microchip/tcpip/wifi/wf_driver_raw_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o ../../../microchip/tcpip/wifi/wf_driver_raw_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_mac_24g.o: ../../../microchip/tcpip/wifi/wf_mac_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mac_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mac_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_mac_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_mac_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_mac_24g.o ../../../microchip/tcpip/wifi/wf_mac_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o: ../../../microchip/tcpip/wifi/wf_mgmt_msg_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o ../../../microchip/tcpip/wifi/wf_mgmt_msg_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o: ../../../microchip/tcpip/wifi/wf_param_msg_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o ../../../microchip/tcpip/wifi/wf_param_msg_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_configData.o: ../../../microchip/tcpip/wifi/wf_configData.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_configData.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_configData.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_configData.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_configData.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_configData.o ../../../microchip/tcpip/wifi/wf_configData.c    -G 64
	
${OBJECTDIR}/_ext/1472/main_demo.o: ../main_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/main_demo.o.d" -o ${OBJECTDIR}/_ext/1472/main_demo.o ../main_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/custom_snmp_app.o: ../custom_snmp_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ../custom_snmp_app.c    -G 64
	
${OBJECTDIR}/_ext/1472/custom_ssl_cert.o: ../custom_ssl_cert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" -o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ../custom_ssl_cert.c    -G 64
	
${OBJECTDIR}/_ext/1472/mpfs_img2.o: ../mpfs_img2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/mpfs_img2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/mpfs_img2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/mpfs_img2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/mpfs_img2.o.d" -o ${OBJECTDIR}/_ext/1472/mpfs_img2.o ../mpfs_img2.c    -G 64
	
${OBJECTDIR}/_ext/1472/wf_config.o: ../wf_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/wf_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/wf_config.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/wf_config.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/wf_config.o.d" -o ${OBJECTDIR}/_ext/1472/wf_config.o ../wf_config.c    -G 64
	
${OBJECTDIR}/_ext/1472/custom_http_app.o: ../custom_http_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PIC32MXSK=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_http_app.o ../custom_http_app.c    -G 64
	
else
${OBJECTDIR}/_ext/979510709/bsp.o: ../../../bsp/pic32_meb/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/979510709 
	@${RM} ${OBJECTDIR}/_ext/979510709/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/979510709/bsp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/979510709/bsp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/979510709/bsp.o.d" -o ${OBJECTDIR}/_ext/979510709/bsp.o ../../../bsp/pic32_meb/bsp.c    -G 64
	
${OBJECTDIR}/_ext/101875047/lfsr.o: ../../../microchip/common/lfsr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/lfsr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/lfsr.o.d" -o ${OBJECTDIR}/_ext/101875047/lfsr.o ../../../microchip/common/lfsr.c    -G 64
	
${OBJECTDIR}/_ext/101875047/hashes.o: ../../../microchip/common/hashes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/hashes.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/hashes.o.d" -o ${OBJECTDIR}/_ext/101875047/hashes.o ../../../microchip/common/hashes.c    -G 64
	
${OBJECTDIR}/_ext/101875047/big_int.o: ../../../microchip/common/big_int.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int.o.d" -o ${OBJECTDIR}/_ext/101875047/big_int.o ../../../microchip/common/big_int.c    -G 64
	
${OBJECTDIR}/_ext/101875047/helpers.o: ../../../microchip/common/helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/helpers.o.d" -o ${OBJECTDIR}/_ext/101875047/helpers.o ../../../microchip/common/helpers.c    -G 64
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o: ../berkeley_tcp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ../berkeley_tcp_client_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o: ../berkeley_tcp_server_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ../berkeley_tcp_server_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o: ../berkeley_udp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ../berkeley_udp_client_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/generic_tcp_client.o: ../generic_tcp_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ../generic_tcp_client.c    -G 64
	
${OBJECTDIR}/_ext/1472/generic_tcp_server.o: ../generic_tcp_server.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ../generic_tcp_server.c    -G 64
	
${OBJECTDIR}/_ext/1472/ping_demo.o: ../ping_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ping_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/ping_demo.o.d" -o ${OBJECTDIR}/_ext/1472/ping_demo.o ../ping_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/smtp_demo.o: ../smtp_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" -o ${OBJECTDIR}/_ext/1472/smtp_demo.o ../smtp_demo.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_services.o: ../../../microchip/system/system_services.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_services.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_services.o.d" -o ${OBJECTDIR}/_ext/365611741/system_services.o ../../../microchip/system/system_services.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_debug.o: ../../../microchip/system/system_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_debug.o.d" -o ${OBJECTDIR}/_ext/365611741/system_debug.o ../../../microchip/system/system_debug.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_random.o: ../../../microchip/system/system_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_random.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_random.o.d" -o ${OBJECTDIR}/_ext/365611741/system_random.o ../../../microchip/system/system_random.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_command.o: ../../../microchip/system/system_command.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_command.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_command.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_command.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_command.o.d" -o ${OBJECTDIR}/_ext/365611741/system_command.o ../../../microchip/system/system_command.c    -G 64
	
${OBJECTDIR}/_ext/365611741/system_userio.o: ../../../microchip/system/system_userio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_userio.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_userio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_userio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_userio.o.d" -o ${OBJECTDIR}/_ext/365611741/system_userio.o ../../../microchip/system/system_userio.c    -G 64
	
${OBJECTDIR}/_ext/97638081/sys_fs.o: ../../../microchip/system/fs/sys_fs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/97638081 
	@${RM} ${OBJECTDIR}/_ext/97638081/sys_fs.o.d 
	@${RM} ${OBJECTDIR}/_ext/97638081/sys_fs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/97638081/sys_fs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/97638081/sys_fs.o.d" -o ${OBJECTDIR}/_ext/97638081/sys_fs.o ../../../microchip/system/fs/sys_fs.c    -G 64
	
${OBJECTDIR}/_ext/97638081/mpfs2.o: ../../../microchip/system/fs/mpfs2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/97638081 
	@${RM} ${OBJECTDIR}/_ext/97638081/mpfs2.o.d 
	@${RM} ${OBJECTDIR}/_ext/97638081/mpfs2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/97638081/mpfs2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/97638081/mpfs2.o.d" -o ${OBJECTDIR}/_ext/97638081/mpfs2.o ../../../microchip/system/fs/mpfs2.c    -G 64
	
${OBJECTDIR}/_ext/792872985/usart.o: ../../../microchip/system/drivers/usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/usart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/usart.o.d" -o ${OBJECTDIR}/_ext/792872985/usart.o ../../../microchip/system/drivers/usart.c    -G 64
	
${OBJECTDIR}/_ext/792872985/lcd.o: ../../../microchip/system/drivers/lcd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/lcd.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/lcd.o.d" -o ${OBJECTDIR}/_ext/792872985/lcd.o ../../../microchip/system/drivers/lcd.c    -G 64
	
${OBJECTDIR}/_ext/792872985/drv_spi.o: ../../../microchip/system/drivers/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" -o ${OBJECTDIR}/_ext/792872985/drv_spi.o ../../../microchip/system/drivers/drv_spi.c    -G 64
	
${OBJECTDIR}/_ext/792872985/db_appio.o: ../../../microchip/system/drivers/db_appio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/db_appio.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/db_appio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/db_appio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/db_appio.o.d" -o ${OBJECTDIR}/_ext/792872985/db_appio.o ../../../microchip/system/drivers/db_appio.c    -G 64
	
${OBJECTDIR}/_ext/792872985/spi_ram.o: ../../../microchip/system/drivers/spi_ram.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_ram.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_ram.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/spi_ram.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/spi_ram.o.d" -o ${OBJECTDIR}/_ext/792872985/spi_ram.o ../../../microchip/system/drivers/spi_ram.c    -G 64
	
${OBJECTDIR}/_ext/792872985/spi_eeprom.o: ../../../microchip/system/drivers/spi_eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_eeprom.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/spi_eeprom.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/spi_eeprom.o.d" -o ${OBJECTDIR}/_ext/792872985/spi_eeprom.o ../../../microchip/system/drivers/spi_eeprom.c    -G 64
	
${OBJECTDIR}/_ext/792872985/spi_flash.o: ../../../microchip/system/drivers/spi_flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_flash.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/spi_flash.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/spi_flash.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/spi_flash.o.d" -o ${OBJECTDIR}/_ext/792872985/spi_flash.o ../../../microchip/system/drivers/spi_flash.c    -G 64
	
${OBJECTDIR}/_ext/792872985/drv_media.o: ../../../microchip/system/drivers/drv_media.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_media.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_media.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/drv_media.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/drv_media.o.d" -o ${OBJECTDIR}/_ext/792872985/drv_media.o ../../../microchip/system/drivers/drv_media.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_announce.o: ../../../microchip/tcpip/tcpip_announce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_announce.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_announce.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_announce.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_announce.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_announce.o ../../../microchip/tcpip/tcpip_announce.c    -G 64
	
${OBJECTDIR}/_ext/427700826/http2.o: ../../../microchip/tcpip/http2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/http2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/http2.o.d" -o ${OBJECTDIR}/_ext/427700826/http2.o ../../../microchip/tcpip/http2.c    -G 64
	
${OBJECTDIR}/_ext/427700826/arcfour.o: ../../../microchip/tcpip/arcfour.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arcfour.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/arcfour.o.d" -o ${OBJECTDIR}/_ext/427700826/arcfour.o ../../../microchip/tcpip/arcfour.c    -G 64
	
${OBJECTDIR}/_ext/427700826/arp.o: ../../../microchip/tcpip/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/arp.o.d" -o ${OBJECTDIR}/_ext/427700826/arp.o ../../../microchip/tcpip/arp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/berkeley_api.o: ../../../microchip/tcpip/berkeley_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" -o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ../../../microchip/tcpip/berkeley_api.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dhcp.o: ../../../microchip/tcpip/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcp.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcp.o ../../../microchip/tcpip/dhcp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dhcps.o: ../../../microchip/tcpip/dhcps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcps.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcps.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcps.o ../../../microchip/tcpip/dhcps.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dns.o: ../../../microchip/tcpip/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dns.o ../../../microchip/tcpip/dns.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dnss.o: ../../../microchip/tcpip/dnss.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dnss.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dnss.o.d" -o ${OBJECTDIR}/_ext/427700826/dnss.o ../../../microchip/tcpip/dnss.c    -G 64
	
${OBJECTDIR}/_ext/427700826/dyn_dns.o: ../../../microchip/tcpip/dyn_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ../../../microchip/tcpip/dyn_dns.c    -G 64
	
${OBJECTDIR}/_ext/427700826/enc28_mac.o: ../../../microchip/tcpip/enc28_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ../../../microchip/tcpip/enc28_mac.c    -G 64
	
${OBJECTDIR}/_ext/427700826/enc28j60.o: ../../../microchip/tcpip/enc28j60.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28j60.o ../../../microchip/tcpip/enc28j60.c    -G 64
	
${OBJECTDIR}/_ext/427700826/encs24j600.o: ../../../microchip/tcpip/encs24j600.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" -o ${OBJECTDIR}/_ext/427700826/encs24j600.o ../../../microchip/tcpip/encs24j600.c    -G 64
	
${OBJECTDIR}/_ext/427700826/encx24_mac.o: ../../../microchip/tcpip/encx24_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ../../../microchip/tcpip/encx24_mac.c    -G 64
	
${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o: ../../../microchip/tcpip/eth_pic32_ext_phy.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o ../../../microchip/tcpip/eth_pic32_ext_phy.c    -G 64
	
${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o: ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c    -G 64
	
${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o: ../../../microchip/tcpip/eth_pic32_int_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o ../../../microchip/tcpip/eth_pic32_int_mac.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ftp.o: ../../../microchip/tcpip/ftp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ftp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ftp.o.d" -o ${OBJECTDIR}/_ext/427700826/ftp.o ../../../microchip/tcpip/ftp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/hash_fnv.o: ../../../microchip/tcpip/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ../../../microchip/tcpip/hash_fnv.c    -G 64
	
${OBJECTDIR}/_ext/427700826/hash_tbl.o: ../../../microchip/tcpip/hash_tbl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ../../../microchip/tcpip/hash_tbl.c    -G 64
	
${OBJECTDIR}/_ext/427700826/icmp.o: ../../../microchip/tcpip/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/icmp.o.d" -o ${OBJECTDIR}/_ext/427700826/icmp.o ../../../microchip/tcpip/icmp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/icmpv6.o: ../../../microchip/tcpip/icmpv6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" -o ${OBJECTDIR}/_ext/427700826/icmpv6.o ../../../microchip/tcpip/icmpv6.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ipv4.o: ../../../microchip/tcpip/ipv4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ipv4.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ipv4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ipv4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ipv4.o.d" -o ${OBJECTDIR}/_ext/427700826/ipv4.o ../../../microchip/tcpip/ipv4.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ipv6.o: ../../../microchip/tcpip/ipv6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ipv6.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ipv6.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ipv6.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ipv6.o.d" -o ${OBJECTDIR}/_ext/427700826/ipv6.o ../../../microchip/tcpip/ipv6.c    -G 64
	
${OBJECTDIR}/_ext/427700826/mac_events_pic32.o: ../../../microchip/tcpip/mac_events_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d" -o ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o ../../../microchip/tcpip/mac_events_pic32.c    -G 64
	
${OBJECTDIR}/_ext/427700826/nbns.o: ../../../microchip/tcpip/nbns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/nbns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/nbns.o.d" -o ${OBJECTDIR}/_ext/427700826/nbns.o ../../../microchip/tcpip/nbns.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ndp.o: ../../../microchip/tcpip/ndp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ndp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ndp.o.d" -o ${OBJECTDIR}/_ext/427700826/ndp.o ../../../microchip/tcpip/ndp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_reboot.o: ../../../microchip/tcpip/tcpip_reboot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_reboot.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_reboot.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_reboot.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_reboot.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_reboot.o ../../../microchip/tcpip/tcpip_reboot.c    -G 64
	
${OBJECTDIR}/_ext/427700826/rsa.o: ../../../microchip/tcpip/rsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/rsa.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/rsa.o.d" -o ${OBJECTDIR}/_ext/427700826/rsa.o ../../../microchip/tcpip/rsa.c    -G 64
	
${OBJECTDIR}/_ext/427700826/smtp.o: ../../../microchip/tcpip/smtp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/smtp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/smtp.o.d" -o ${OBJECTDIR}/_ext/427700826/smtp.o ../../../microchip/tcpip/smtp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/snmp.o: ../../../microchip/tcpip/snmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/snmp.o.d" -o ${OBJECTDIR}/_ext/427700826/snmp.o ../../../microchip/tcpip/snmp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/snmpv3.o: ../../../microchip/tcpip/snmpv3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmpv3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/snmpv3.o.d" -o ${OBJECTDIR}/_ext/427700826/snmpv3.o ../../../microchip/tcpip/snmpv3.c    -G 64
	
${OBJECTDIR}/_ext/427700826/snmpv3_usm.o: ../../../microchip/tcpip/snmpv3_usm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d" -o ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o ../../../microchip/tcpip/snmpv3_usm.c    -G 64
	
${OBJECTDIR}/_ext/427700826/sntp.o: ../../../microchip/tcpip/sntp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/sntp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/sntp.o.d" -o ${OBJECTDIR}/_ext/427700826/sntp.o ../../../microchip/tcpip/sntp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/ssl.o: ../../../microchip/tcpip/ssl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ssl.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ssl.o.d" -o ${OBJECTDIR}/_ext/427700826/ssl.o ../../../microchip/tcpip/ssl.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcp.o: ../../../microchip/tcpip/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp.o ../../../microchip/tcpip/tcp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o: ../../../microchip/tcpip/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ../../../microchip/tcpip/tcpip_heap_alloc.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_storage.o: ../../../microchip/tcpip/tcpip_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ../../../microchip/tcpip/tcpip_storage.c    -G 64
	
${OBJECTDIR}/_ext/427700826/telnet.o: ../../../microchip/tcpip/telnet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/telnet.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/telnet.o.d" -o ${OBJECTDIR}/_ext/427700826/telnet.o ../../../microchip/tcpip/telnet.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tftpc.o: ../../../microchip/tcpip/tftpc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tftpc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tftpc.o.d" -o ${OBJECTDIR}/_ext/427700826/tftpc.o ../../../microchip/tcpip/tftpc.c    -G 64
	
${OBJECTDIR}/_ext/427700826/udp.o: ../../../microchip/tcpip/udp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/udp.o.d" -o ${OBJECTDIR}/_ext/427700826/udp.o ../../../microchip/tcpip/udp.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o: ../../../microchip/tcpip/tcpip_mac_object.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ../../../microchip/tcpip/tcpip_mac_object.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_manager.o: ../../../microchip/tcpip/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ../../../microchip/tcpip/tcpip_manager.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_helpers.o: ../../../microchip/tcpip/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ../../../microchip/tcpip/tcpip_helpers.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_commands.o: ../../../microchip/tcpip/tcpip_commands.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_commands.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_commands.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_commands.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_commands.o ../../../microchip/tcpip/tcpip_commands.c    -G 64
	
${OBJECTDIR}/_ext/427700826/iperf.o: ../../../microchip/tcpip/iperf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/iperf.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/iperf.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/iperf.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/iperf.o.d" -o ${OBJECTDIR}/_ext/427700826/iperf.o ../../../microchip/tcpip/iperf.c    -G 64
	
${OBJECTDIR}/_ext/427700826/tcpip_notify.o: ../../../microchip/tcpip/tcpip_notify.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_notify.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_notify.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_notify.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_notify.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_notify.o ../../../microchip/tcpip/tcpip_notify.c    -G 64
	
${OBJECTDIR}/_ext/427700826/zero_conf_helper.o: ../../../microchip/tcpip/zero_conf_helper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_helper.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_helper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/zero_conf_helper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/zero_conf_helper.o.d" -o ${OBJECTDIR}/_ext/427700826/zero_conf_helper.o ../../../microchip/tcpip/zero_conf_helper.c    -G 64
	
${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o: ../../../microchip/tcpip/zero_conf_link_local.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o.d" -o ${OBJECTDIR}/_ext/427700826/zero_conf_link_local.o ../../../microchip/tcpip/zero_conf_link_local.c    -G 64
	
${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o: ../../../microchip/tcpip/zero_conf_multicast_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o.d" -o ${OBJECTDIR}/_ext/427700826/zero_conf_multicast_dns.o ../../../microchip/tcpip/zero_conf_multicast_dns.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_tx_power.o: ../../../microchip/tcpip/wifi/wf_tx_power.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_tx_power.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_tx_power.o ../../../microchip/tcpip/wifi/wf_tx_power.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_connect.o: ../../../microchip/tcpip/wifi/wf_connect.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connect.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connect.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connect.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connect.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connect.o ../../../microchip/tcpip/wifi/wf_connect.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o: ../../../microchip/tcpip/wifi/wf_connection_algorithm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o ../../../microchip/tcpip/wifi/wf_connection_algorithm.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_connection_manager.o: ../../../microchip/tcpip/wifi/wf_connection_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o ../../../microchip/tcpip/wifi/wf_connection_manager.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_connection_profile.o: ../../../microchip/tcpip/wifi/wf_connection_profile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o ../../../microchip/tcpip/wifi/wf_connection_profile.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_easy_config.o: ../../../microchip/tcpip/wifi/wf_easy_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_easy_config.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_easy_config.o ../../../microchip/tcpip/wifi/wf_easy_config.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_eint.o: ../../../microchip/tcpip/wifi/wf_eint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_eint.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_eint.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_eint.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_eint.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_eint.o ../../../microchip/tcpip/wifi/wf_eint.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_event_handler.o: ../../../microchip/tcpip/wifi/wf_event_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_event_handler.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_event_handler.o ../../../microchip/tcpip/wifi/wf_event_handler.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_init.o: ../../../microchip/tcpip/wifi/wf_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_init.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_init.o ../../../microchip/tcpip/wifi/wf_init.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_power_save.o: ../../../microchip/tcpip/wifi/wf_power_save.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_power_save.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_power_save.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_power_save.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_power_save.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_power_save.o ../../../microchip/tcpip/wifi/wf_power_save.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_scan.o: ../../../microchip/tcpip/wifi/wf_scan.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_scan.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_scan.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_scan.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_scan.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_scan.o ../../../microchip/tcpip/wifi/wf_scan.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_spi.o: ../../../microchip/tcpip/wifi/wf_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_spi.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_spi.o ../../../microchip/tcpip/wifi/wf_spi.c    -G 64
	
${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o: ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d" -o ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c    -G 64
	
${OBJECTDIR}/_ext/334706090/mrf24w_events.o: ../../../microchip/tcpip/wifi/mrf24w_events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_events.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d" -o ${OBJECTDIR}/_ext/334706090/mrf24w_events.o ../../../microchip/tcpip/wifi/mrf24w_events.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_debug_output.o: ../../../microchip/tcpip/wifi/wf_debug_output.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_debug_output.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_debug_output.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_debug_output.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_debug_output.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_debug_output.o ../../../microchip/tcpip/wifi/wf_debug_output.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_commands.o: ../../../microchip/tcpip/wifi/wf_commands.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_commands.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_commands.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_commands.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_commands.o ../../../microchip/tcpip/wifi/wf_commands.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o: ../../../microchip/tcpip/wifi/wf_pbkdf2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_pbkdf2.o ../../../microchip/tcpip/wifi/wf_pbkdf2.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o: ../../../microchip/tcpip/wifi/wf_driver_com_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_driver_com_24g.o ../../../microchip/tcpip/wifi/wf_driver_com_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o: ../../../microchip/tcpip/wifi/wf_driver_raw_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_driver_raw_24g.o ../../../microchip/tcpip/wifi/wf_driver_raw_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_mac_24g.o: ../../../microchip/tcpip/wifi/wf_mac_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mac_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mac_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_mac_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_mac_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_mac_24g.o ../../../microchip/tcpip/wifi/wf_mac_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o: ../../../microchip/tcpip/wifi/wf_mgmt_msg_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg_24g.o ../../../microchip/tcpip/wifi/wf_mgmt_msg_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o: ../../../microchip/tcpip/wifi/wf_param_msg_24g.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_param_msg_24g.o ../../../microchip/tcpip/wifi/wf_param_msg_24g.c    -G 64
	
${OBJECTDIR}/_ext/334706090/wf_configData.o: ../../../microchip/tcpip/wifi/wf_configData.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_configData.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_configData.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_configData.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_configData.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_configData.o ../../../microchip/tcpip/wifi/wf_configData.c    -G 64
	
${OBJECTDIR}/_ext/1472/main_demo.o: ../main_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/main_demo.o.d" -o ${OBJECTDIR}/_ext/1472/main_demo.o ../main_demo.c    -G 64
	
${OBJECTDIR}/_ext/1472/custom_snmp_app.o: ../custom_snmp_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ../custom_snmp_app.c    -G 64
	
${OBJECTDIR}/_ext/1472/custom_ssl_cert.o: ../custom_ssl_cert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" -o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ../custom_ssl_cert.c    -G 64
	
${OBJECTDIR}/_ext/1472/mpfs_img2.o: ../mpfs_img2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/mpfs_img2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/mpfs_img2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/mpfs_img2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/mpfs_img2.o.d" -o ${OBJECTDIR}/_ext/1472/mpfs_img2.o ../mpfs_img2.c    -G 64
	
${OBJECTDIR}/_ext/1472/wf_config.o: ../wf_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/wf_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/wf_config.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/wf_config.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/wf_config.o.d" -o ${OBJECTDIR}/_ext/1472/wf_config.o ../wf_config.c    -G 64
	
${OBJECTDIR}/_ext/1472/custom_http_app.o: ../custom_http_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -DPIC32_STARTER_KIT -DMRF24WG -DWIFI_DISABLE_IPV6 -I"../configs/pic32_eth_sk/mrf24w/tcpip_profile" -I"../configs/pic32_eth_sk/mrf24w/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_http_app.o ../custom_http_app.c    -G 64
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_mrf24wg_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PIC32MXSK=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_mrf24wg_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PIC32MXSK=1,--defsym=_min_heap_size=41960,--defsym=_min_stack_size=2048,-L"..",-Map="${DISTDIR}/tcpip_pic32_meb_web_server_demo_app.X.${IMAGE_TYPE}.map",--gc-sections -Os
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_mrf24wg_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_mrf24wg_web_server_demo_app.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=41960,--defsym=_min_stack_size=2048,-L"..",-Map="${DISTDIR}/tcpip_pic32_meb_web_server_demo_app.X.${IMAGE_TYPE}.map",--gc-sections -Os
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_mrf24wg_web_server_demo_app.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
