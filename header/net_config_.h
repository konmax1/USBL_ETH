/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::Network
 * Copyright (c) 2004-2018 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    net_config.h
 * Purpose: Network Library Configuration
 * Rev.:    V7.7.1
 *----------------------------------------------------------------------------*/

#if   defined(__CC_ARM)
  #pragma O3
#elif defined(__clang__)
  #pragma clang diagnostic ignored "-Wundef"
  #pragma clang diagnostic ignored "-Wpadded"
  #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

#include "rl_net_lib.h"

#if   defined(RTE_CMSIS_RTOS)
  #include "net_rtos.h"
#elif defined(RTE_CMSIS_RTOS2)
  #include "net_rtos2.h"
#else
  #error "::CMSIS:RTOS selection invalid"
#endif

#ifdef RTE_Network_Legacy
  #include "rl_net_legacy.h"
#endif

/* Non-Critical configuration upgrade */
#if (ETH0_ENABLE && !defined(ETH0_IP4_FRAG_ENABLE))
  #define ETH0_IP4_FRAG_ENABLE      0
#endif
#if (PPP_ENABLE && !defined(PPP_IP4_FRAG_ENABLE))
  #define PPP_IP4_FRAG_ENABLE       0
#endif
#if (SLIP_ENABLE && !defined(SLIP_IP4_FRAG_ENABLE))
  #define SLIP_IP4_FRAG_ENABLE      0
#endif
#if (SMTP_CLIENT_ENABLE && !defined(SMTP_CLIENT_ATTACH_ENABLE))
  #define SMTP_CLIENT_ATTACH_ENABLE 0
#endif

/* Check configuration integrity */
#if (!defined(NET_THREAD_STACK_SIZE))
  #error "::Network:CORE: Configuration update required"
#endif

#if (ETH0_ENABLE && !defined(ETH0_THREAD_STACK_SIZE))
  #error "::Network:Interface:ETH0: Configuration update required"
#endif

#if (PPP_ENABLE && !defined(PPP_THREAD_STACK_SIZE))
  #error "::Network:Interface:PPP: Configuration update required"
#endif

#if (SLIP_ENABLE && !defined(SLIP_THREAD_STACK_SIZE))
  #error "::Network:Interface:SLIP: Configuration update required"
#endif

#if (TCP_ENABLE && (TCP_MAX_SEG_SIZE > 1440))
  #error "::Network:Socket:TCP: Configuration update required"
#endif

#if (HTTP_SERVER_ENABLE && !defined(HTTP_SERVER_AUTH_ADMIN))
  #error "::Network:Service:HTTP Server: Configuration update required"
#endif

#if (FTP_SERVER_ENABLE && !defined(FTP_SERVER_AUTH_ADMIN))
  #error "::Network:Service:FTP Server: Configuration update required"
#endif

#if (TELNET_SERVER_ENABLE && !defined(TELNET_SERVER_AUTH_ADMIN))
  #error "::Network:Service:Telnet Server: Configuration update required"
#endif

#if (TFTP_SERVER_ENABLE && !defined(TFTP_SERVER_ROOT_ENABLE))
  #error "::Network:Service:TFTP Server: Configuration update required"
#endif

#if (SNTP_CLIENT_ENABLE && !defined(SNTP_CLIENT_NTP_SERVER))
  #error "::Network:Service:SNTP Client: Configuration update required"
#endif

#if (SNMP_AGENT_ENABLE && !defined(SNMP_AGENT_TRAP_IP))
  #error "::Network:Service:SNMP Agent: Configuration update required"
#endif

/* Check Network variant */
#ifndef RTE_Network_IPv6
  #undef ETH0_IP6_ENABLE
  #undef ETH1_IP6_ENABLE
  #undef ETH0_DHCP6_ENABLE
  #undef ETH1_DHCP6_ENABLE
  #define ETH0_IP6_ENABLE       0
  #define ETH1_IP6_ENABLE       0
  #define ETH0_DHCP6_ENABLE     0
  #define ETH1_DHCP6_ENABLE     0
#endif

/* Check IPv4 and IPv6 usage */
#define __IP4_USED ((ETH0_ENABLE && ETH0_IP4_ENABLE) || \
                    (ETH1_ENABLE && ETH1_IP4_ENABLE) || \
                    (PPP_ENABLE)                     || \
                    (SLIP_ENABLE))
#define __IP6_USED ((ETH0_ENABLE && ETH0_IP6_ENABLE) || \
                    (ETH1_ENABLE && ETH1_IP6_ENABLE))

/* Check enabled interfaces */
#if (!ETH0_ENABLE && !ETH1_ENABLE && !PPP_ENABLE && !SLIP_ENABLE)
  #error "::Network:Interface: No interface enabled in configuration"
#endif

#if (ETH0_ENABLE && ETH1_ENABLE)
  #error "::Network:Interface: Only one instance of same interface allowed"
#endif

#if (SLIP_ENABLE && PPP_ENABLE)
  #error "::Network:Interface:PPP and ::Network:Interface:SLIP cannot be used simultaneously"
#endif

/* Check ETH port consistency */
#if ((ETH0_ENABLE && ETH1_ENABLE) && (ETH0_DRIVER == ETH1_DRIVER))
  #error "::Network:Interface:ETH: ETH0 and ETH1 driver numbers must be different"
#endif

/* Check ETH0 enabled protocols */
#if (ETH0_ENABLE && !ETH0_IP4_ENABLE && !ETH0_IP6_ENABLE)
  #error "::Network:Interface:ETH0: IP protocol not enabled"
#endif

/* Check ETH1 enabled protocols */
#if (ETH1_ENABLE && !ETH1_IP4_ENABLE && !ETH1_IP6_ENABLE)
  #error "::Network:Interface:ETH1: IP protocol not enabled"
#endif

/* Check TCP socket configuration */
#define __TCPNS    ((BSD_ENABLE           * BSD_NUM_SOCKS)               + \
                    (HTTP_SERVER_ENABLE   * HTTP_SERVER_NUM_SESSIONS)    + \
                    (TELNET_SERVER_ENABLE * TELNET_SERVER_NUM_SESSISONS) + \
                    (FTP_SERVER_ENABLE    * FTP_SERVER_NUM_SESSIONS * 2) + \
                    (FTP_CLIENT_ENABLE    * 2)                           + \
                    (SMTP_CLIENT_ENABLE   * 1))

#if (__TCPNS > 0 && !TCP_ENABLE)
  #error "::Network:Socket:TCP component required"
#elif (__TCPNS > TCP_NUM_SOCKS)
  #error "::Network:Socket:TCP: Number of TCP Sockets too small"
#endif

/* Calculate number of UDP sockets needed for TFTP server */
#if (TFTP_SERVER_FIREWALL_ENABLE)
  #define __TFTPNS  1
#else
  #define __TFTPNS (1 + TFTP_SERVER_NUM_SESSIONS)
#endif

/* Check UDP socket configuration */
#define __UDPNS    ((BSD_ENABLE         * BSD_NUM_SOCKS)               + \
                    (TFTP_SERVER_ENABLE * __TFTPNS)                    + \
                    (TFTP_CLIENT_ENABLE * 1)                           + \
                    (DNS_CLIENT_ENABLE  * 1)                           + \
                    (SNMP_AGENT_ENABLE  * 1)                           + \
                    (SNTP_CLIENT_ENABLE * 1)                           + \
                    (ETH0_DHCP_ENABLE   * ETH0_ENABLE)                 + \
                    (ETH0_NBNS_ENABLE   * ETH0_ENABLE)                 + \
                    (ETH0_DHCP6_ENABLE  * ETH0_DHCP6_MODE * ETH0_ENABLE))

#if (__UDPNS > 0 && !UDP_ENABLE)
  #error "::Network:Socket:UDP component required"
#elif (__UDPNS > UDP_NUM_SOCKS)
  #error "::Network:Socket:UDP: Number of UDP Sockets too small"
#endif

/* Check Maximum Segment Size of TCP Socket */
#if (TCP_ENABLE && ((TCP_MAX_SEG_SIZE < 536) || (TCP_MAX_SEG_SIZE > 1440)))
  #error "::Network:Socket:TCP: Maximum Segment Size out of range"
#endif

/* Check Receive Window Size of TCP Socket */
#if (TCP_ENABLE && (TCP_RECEIVE_WIN_SIZE < TCP_MAX_SEG_SIZE))
  #error "::Network:Socket:TCP: Receive Window Size too small"
#endif

/* Check BSD Server sockets */
#if (BSD_ENABLE && (BSD_SERVER_SOCKS > BSD_NUM_SOCKS))
  #error "::Network:Socket:BSD: Number of BSD Server sockets too large"
#endif

/* Check BSD Hostname resolver */
#if (BSD_ENABLE && BSD_HOSTNAME_ENABLE && !DNS_CLIENT_ENABLE)
  #error "::Network:Socket:BSD: DNS Client service required"
#endif

/* Check PPP Authentication protocols */
#if (PPP_ENABLE && PPP_AUTH_ENABLE && !(PPP_PAP_ENABLE || PPP_CHAP_ENABLE))
  #error "::Network:Interface:PPP: Authentication needs PAP or/and CHAP enabled"
#endif

/* Check Block Size of TFTP Client */
#if (TFTP_CLIENT_ENABLE && ((TFTP_CLIENT_BLOCK_SIZE < 128) || (TFTP_CLIENT_BLOCK_SIZE > 1428)))
  #error "::Network:Service:TFTP Client: Block Size out of range"
#endif

/* Check SMTP client attachments */
#if (SMTP_CLIENT_ENABLE && SMTP_CLIENT_ATTACH_ENABLE && !defined(RTE_FileSystem_Core))
  #error "::Network:Service:SMTP Client: File System component required"
#endif

/* Check SMTP advanced client */
#if (SMTP_CLIENT_ENABLE && SMTP_CLIENT_ATTACH_ENABLE && !DNS_CLIENT_ENABLE)
  #error "::Network:Service:SMTP Client: DNS Client service required"
#endif

/* Avoid syntax-checker errors in editor */
#ifndef NET_HOST_NAME
  #define NET_HOST_NAME         "my_host"
  #define NET_MEM_POOL_SIZE     8000
  #define NET_START_SERVICE     1
#endif

#ifndef NET_MEM_POOL_SIZE
  #define NET_MEM_POOL_SIZE     (NET_MEM_SIZE*4)
#endif

/* Tick interval is 100 ms */
#define NET_TICK_RATE       10
#define STRLEN(str)         sizeof(str)-1
#define MAX_DELAY(speed)    ((2000000/speed) < 25 ? (2000000/speed) : 25)
#define MAX_TOUT(speed)     ((speed < 10000) ? 5 : 1)

#define EXPAND_SYMBOL(name, port) name##port
#define CREATE_SYMBOL(name, port) EXPAND_SYMBOL(name, port)

/* System configuration */
static uint32_t mem_pool [NET_MEM_POOL_SIZE/4] __attribute__((section(".ARM.__at_0x24070000")));

#if ((HTTP_SERVER_ENABLE && HTTP_SERVER_ROOT_ENABLE) || \
     (FTP_SERVER_ENABLE  && FTP_SERVER_ROOT_ENABLE ) || \
     (TFTP_SERVER_ENABLE && TFTP_SERVER_ROOT_ENABLE))
  /* Buffer for services with root folder enabled */
  static char root_path [NET_ROOT_PATH_SIZE];
#else
  #define root_path      NULL
#endif

NET_SYS_CFG net_sys_config = {
  mem_pool,
  sizeof (mem_pool),
  root_path,
  NET_HOST_NAME,
  NET_START_SERVICE,
  { /* Network interface configuration flags */
  #if (ETH0_ENABLE)
    (0x80 | ETH0_DHCP_ENABLE << 2 | ETH0_IP6_ENABLE << 1 | ETH0_IP4_ENABLE << 0),
  #else
    0x00,
  #endif
  #if (PPP_ENABLE)
    (0x80 | PPP_GET_IP << 3 | PPP_DEFAULT_GW << 2 | 0x01),
  #else
    0x00,
  #endif
  #if (SLIP_ENABLE)
    (0x80 | SLIP_DEFAULT_GW << 2 | 0x01)
  #else
    0x00
  #endif
  }
};

/* netIF_GetOption functions */
NETIF_GETOPT_FUNC netif_getopt_func = {
#if (ETH0_ENABLE)
  net_eth_get_option,
#else
  NULL,
#endif
#if (PPP_ENABLE)
  net_ppp_get_option,
#else
  NULL,
#endif
#if (SLIP_ENABLE)
  net_slip_get_option
#else
  NULL
#endif
};

/* netIF_SetOption functions */
NETIF_SETOPT_FUNC netif_setopt_func = {
#if (ETH0_ENABLE)
  net_eth_set_option,
#else
  NULL,
#endif
#if (PPP_ENABLE)
  net_ppp_set_option,
#else
  NULL,
#endif
#if (SLIP_ENABLE)
  net_slip_set_option
#else
  NULL
#endif
};

/* mbedTLS interface configuration */
#if defined(RTE_Network_Web_Server_RO_TLS) || \
    defined(RTE_Network_Web_Server_FS_TLS) || \
    defined(RTE_Network_SMTP_Client_TLS)
  /* mbedTLS interface functions */
  static NET_TLS_IF tls_if_func = {
    netTLS_GetContext,
    netTLS_Connect,
    netTLS_Listen,
    netTLS_GetBuffer,
    netTLS_Write,
    netTLS_Close
  };
  #define __TLS_USED        1
#else
  #define __TLS_USED        0
#endif

#if (ETH0_ENABLE)
  /* ETH0 device MAC and PHY drivers */
  extern ARM_DRIVER_ETH_MAC ARM_Driver_ETH_MAC_(ETH0_DRIVER);
  extern ARM_DRIVER_ETH_PHY ARM_Driver_ETH_PHY_(ETH0_DRIVER);

  /* ETH0 device local MAC address */
  static uint8_t eth0_mac_addr[NET_ADDR_ETH_LEN];

  /* ETH0 device IPv4 resources */
  #if (ETH0_IP4_ENABLE)
    static NET_ARP_INFO eth0_arp_table[ETH0_ARP_TAB_SIZE];
    static NET_LOCALM   eth0_localm __attribute__((aligned(4)));
    #if (ETH0_IGMP_ENABLE)
    static NET_IGMP_INFO eth0_igmp_table[ETH0_IGMP_TAB_SIZE];
    #endif
    static NET_IP4_CFG eth0_ip4_config = {
      ETH0_IP4_ADDR,
      ETH0_IP4_MASK,
      ETH0_IP4_GATEWAY,
      ETH0_IP4_PRIMARY_DNS,
      ETH0_IP4_SECONDARY_DNS,
    #if (ETH0_IP4_FRAG_ENABLE)
      ETH0_IP4_MTU,
    #else
      0,
    #endif
      /* ARP configuration */
      { eth0_arp_table,
        ETH0_ARP_TAB_SIZE,
        ETH0_ARP_CACHE_TOUT,
        ETH0_ARP_MAX_RETRY,
        ETH0_ARP_RESEND_TOUT * NET_TICK_RATE,
        ETH0_ARP_NOTIFY
      },
      /* IGMP configuration */
    #if (ETH0_IGMP_ENABLE)
      { eth0_igmp_table,
        ETH0_IGMP_TAB_SIZE
      },
    #else
      { NULL, 0 },
    #endif
      /* DHCP configuration */
    #if (ETH0_DHCP_ENABLE)
      { ETH0_DHCP_VCID,
        STRLEN(ETH0_DHCP_VCID),
       (ETH0_DHCP_NTP_SERVERS << 1 | ETH0_DHCP_BOOTFILE)
      }
    #else
      { NULL, 0, 0 }
    #endif
    };
  #endif

  /* ETH0 device IPv6 resources */
  #if (ETH0_IP6_ENABLE)
    static NET_NDP_INFO eth0_ndp_table[ETH0_NDP_TAB_SIZE];
    static NET_LOCALM6  eth0_localm6 __attribute__((aligned(4)));
    static NET_IP6_CFG eth0_ip6_config = {
      ETH0_IP6_ADDR,
      ETH0_IP6_GATEWAY,
      ETH0_IP6_PRIMARY_DNS,
      ETH0_IP6_SECONDARY_DNS,
      ETH0_IP6_PREFIX_LEN,
      /* Neighbor Discovery configuration */
      { eth0_ndp_table,
        ETH0_NDP_TAB_SIZE,
        ETH0_NDP_CACHE_TOUT,
        ETH0_NDP_MAX_RETRY,
        ETH0_NDP_RESEND_TOUT * NET_TICK_RATE
      },
      /* DHCPv6 configuration */
      { (ETH0_DHCP6_ENABLE << 7 | ETH0_DHCP6_MODE),
    #if (ETH0_DHCP6_ENABLE && ETH0_DHCP6_VCLASS_ENABLE)
        ETH0_DHCP6_VCLASS_ENABLE,
        ETH0_DHCP6_VCLASS_EID,
        ETH0_DHCP6_VCLASS_DATA
    #else
        0x00, 0, NULL
    #endif
      }
    };
  #endif

  /* ETH0 device configuration */
  NET_ETH_CFG net_eth0_config = {
    &ARM_Driver_ETH_MAC_(ETH0_DRIVER),
    &ARM_Driver_ETH_PHY_(ETH0_DRIVER),
    eth0_mac_addr,
    ETH0_MAC_ADDR,
  #if (ETH0_IP4_ENABLE)
    &eth0_ip4_config,
  #else
    NULL,
  #endif
  #if (ETH0_IP6_ENABLE)
    &eth0_ip6_config
  #else
    NULL
  #endif
  };
#endif

#if (PPP_ENABLE)
  /* PPP device USART and MODEM drivers */
  extern ARM_DRIVER_USART CREATE_SYMBOL (Driver_USART, PPP_USART_DRIVER);
  extern DRIVER_MODEM     Driver_MODEM;

  /* PPP device resources */
  static NET_LOCALM ppp_localm __attribute__((aligned(4)));

  /* PAP authentication functions */
  #if (PPP_AUTH_ENABLE && PPP_PAP_ENABLE)
  static NET_PPP_AUTH ppp_pap_func[1] = {
    net_pap_init,
    net_pap_uninit,
    net_pap_run,
    net_pap_process
  };
  #else
    #define ppp_pap_func    NULL
  #endif

  /* CHAP authentication functions */
  #if (PPP_AUTH_ENABLE && PPP_CHAP_ENABLE)
  static NET_PPP_AUTH ppp_chap_func[1] = {
    net_chap_init,
    net_chap_uninit,
    net_chap_run,
    net_chap_process
  };
  #else
    #define ppp_chap_func   NULL
  #endif

  /* PPP device configuration */
  NET_PPP_CFG net_ppp_config = {
    /* Modem device */
    { &CREATE_SYMBOL (Driver_USART, PPP_USART_DRIVER),
      &Driver_MODEM,
      PPP_MODEM_INIT_STRING,
      PPP_MODEM_SPEED,
      PPP_MODEM_FLOW_CONTROL,
      MAX_DELAY(PPP_MODEM_SPEED),
      MAX_TOUT(PPP_MODEM_SPEED) * NET_TICK_RATE
    },
    PPP_ACCM,
    PPP_RETRY_TOUT * NET_TICK_RATE,
    PPP_ECHO_TOUT,
    PPP_MAX_RETRY,
    (PPP_CHAP_ENABLE << 1 | PPP_PAP_ENABLE) * PPP_AUTH_ENABLE,
    PPP_IP4_ADDR,
    PPP_IP4_PRIMARY_DNS,
    PPP_IP4_SECONDARY_DNS,
  #if (PPP_IP4_FRAG_ENABLE)
    PPP_IP4_MTU,
  #else
    0,
  #endif
    ppp_pap_func,
    ppp_chap_func
  };
#endif

#if (SLIP_ENABLE)
  /* SLIP device USART and MODEM drivers */
  extern ARM_DRIVER_USART CREATE_SYMBOL (Driver_USART, SLIP_USART_DRIVER);
  extern DRIVER_MODEM     Driver_MODEM;

  /* SLIP device resources */
  static NET_LOCALM slip_localm __attribute__((aligned(4)));

  /* SLIP device configuration */
  NET_SLIP_CFG net_slip_config = {
    /* Modem device */
    { &CREATE_SYMBOL (Driver_USART, SLIP_USART_DRIVER),
      &Driver_MODEM,
      SLIP_MODEM_INIT_STRING,
      SLIP_MODEM_SPEED,
      SLIP_MODEM_FLOW_CONTROL,
      MAX_DELAY(SLIP_MODEM_SPEED),
      MAX_TOUT(SLIP_MODEM_SPEED) * NET_TICK_RATE
    },
    SLIP_IP4_ADDR,
    SLIP_IP4_PRIMARY_DNS,
    SLIP_IP4_SECONDARY_DNS,
  #if (SLIP_IP4_FRAG_ENABLE)
    SLIP_IP4_MTU
  #else
    0
  #endif
  };
#endif

/* Local Machine info IPv4 */
NET_LOCALM *const net_localm[3] = {
#if (ETH0_ENABLE && ETH0_IP4_ENABLE)
  &eth0_localm,
#else
  NULL,
#endif
#if (PPP_ENABLE)
  &ppp_localm,
#else
  NULL,
#endif
#if (SLIP_ENABLE)
  &slip_localm
#else
  NULL
#endif
};

/* Local Machine info IPv6 */
NET_LOCALM6 *const net_localm6[3] = {
#if (ETH0_ENABLE && ETH0_IP6_ENABLE)
  &eth0_localm6,
#else
  NULL,
#endif
  NULL,
  NULL
};

/* IP4 fragmentation and reassembly */ 
#if ((ETH0_ENABLE && ETH0_IP4_FRAG_ENABLE) || \
     (PPP_ENABLE  && PPP_IP4_FRAG_ENABLE)  || \
     (SLIP_ENABLE && SLIP_IP4_FRAG_ENABLE))
  #define IP4_FRAG_ENABLE       1
#ifndef IP4_FRAG_REASS_NUM
  /* Number of reassembly sessions */
  #define IP4_FRAG_REASS_NUM    5
#endif
#ifndef IP4_FRAG_REASS_TOUT
  /* Reassembly timeout in seconds */
  #define IP4_FRAG_REASS_TOUT   10
#endif
#else
  #define IP4_FRAG_ENABLE       0
#endif

#if (IP4_FRAG_ENABLE)
  static NET_IP4_FRAG_INFO ip4_frag_scb[IP4_FRAG_REASS_NUM];
  NET_IP4_FRAG_CFG net_ip4_frag_config = {
    ip4_frag_scb,
    IP4_FRAG_REASS_NUM,
    IP4_FRAG_REASS_TOUT * NET_TICK_RATE
  };
#endif

#if (UDP_ENABLE)
  static NET_UDP_INFO udp_scb[UDP_NUM_SOCKS];
#ifdef RTE_Network_Legacy
  net_udp_cb_t net_udp_cb_legacy[UDP_NUM_SOCKS];
#endif
  NET_UDP_CFG net_udp_config = {
    udp_scb,
    UDP_NUM_SOCKS
  };
#endif

#if (TCP_ENABLE)
  static NET_TCP_INFO tcp_scb[TCP_NUM_SOCKS];
#ifdef RTE_Network_Legacy
  net_tcp_cb_t net_tcp_cb_legacy[TCP_NUM_SOCKS];
#endif
  NET_TCP_CFG net_tcp_config = {
    tcp_scb,
    TCP_NUM_SOCKS,
    TCP_MAX_RETRY,
    TCP_RETRY_TOUT * NET_TICK_RATE,
    TCP_SYN_RETRY_TOUT * NET_TICK_RATE,
    TCP_INITIAL_RETRY_TOUT * NET_TICK_RATE,
    TCP_DEFAULT_TOUT,
    TCP_MAX_SEG_SIZE,
    TCP_RECEIVE_WIN_SIZE,
    TCP_CONNECT_RETRY
  };
#endif

#if (BSD_ENABLE)
  static NET_BSD_INFO bsd_scb[BSD_NUM_SOCKS + BSD_SERVER_SOCKS];
  NET_BSD_CFG net_bsd_config = {
    bsd_scb,
    BSD_NUM_SOCKS + BSD_SERVER_SOCKS,
    BSD_RECEIVE_TOUT * NET_TICK_RATE
  };
#endif

#if (HTTP_SERVER_ENABLE)
  static NET_HTTP_INFO http_scb[HTTP_SERVER_NUM_SESSIONS];
  #if (HTTP_SERVER_AUTH_ENABLE && HTTP_SERVER_AUTH_ADMIN)
    static char http_user [NET_USERNAME_SIZE];
    static char http_passw[NET_PASSWORD_SIZE];
  #endif
  NET_HTTP_CFG net_http_config = {
    http_scb,
    HTTP_SERVER_NUM_SESSIONS,
    HTTP_SERVER_AUTH_ENABLE,
    HTTP_SERVER_PORT_NUM,
    HTTP_SERVER_ID,
  #if (HTTP_SERVER_ROOT_ENABLE && !defined(RTE_Network_Web_Server_RO))
    HTTP_SERVER_ROOT_FOLDER,
  #else
    NULL,
  #endif
  #if (HTTP_SERVER_AUTH_ENABLE)
    HTTP_SERVER_AUTH_REALM,
  #else
    "",
  #endif
  #if (HTTP_SERVER_AUTH_ENABLE && HTTP_SERVER_AUTH_ADMIN)
    HTTP_SERVER_AUTH_USER,
    HTTP_SERVER_AUTH_PASS,
    http_user,
    http_passw,
  #else
    NULL,NULL,NULL,NULL,
  #endif
  #if defined(RTE_Network_Web_Server_RO_TLS) || defined(RTE_Network_Web_Server_FS_TLS)
    &tls_if_func
  #else
    NULL
  #endif
  };
#endif

#if (TELNET_SERVER_ENABLE)
  static NET_TELNET_INFO telnet_scb[TELNET_SERVER_NUM_SESSISONS];
  #if (TELNET_SERVER_AUTH_ENABLE && TELNET_SERVER_AUTH_ADMIN)
    static char telnet_user [NET_USERNAME_SIZE];
    static char telnet_passw[NET_PASSWORD_SIZE];
  #endif
  NET_TELNET_CFG net_telnet_config = {
    telnet_scb,
    TELNET_SERVER_NUM_SESSISONS,
    TELNET_SERVER_AUTH_ENABLE,
    TELNET_SERVER_NO_ECHO,
    TELNET_SERVER_PORT_NUM,
    TELNET_SERVER_TOUT,
  #if (TELNET_SERVER_AUTH_ENABLE && TELNET_SERVER_AUTH_ADMIN)
    TELNET_SERVER_AUTH_USER,
    TELNET_SERVER_AUTH_PASS,
    telnet_user,
    telnet_passw
  #else
    NULL,NULL,NULL,NULL
  #endif
  };
#endif

#if (TFTP_SERVER_ENABLE)
  static NET_TFTP_INFO tftp_scb[TFTP_SERVER_NUM_SESSIONS];
  NET_TFTP_CFG net_tftp_config = {
    tftp_scb,
    TFTP_SERVER_NUM_SESSIONS,
    TFTP_SERVER_MAX_RETRY,
    TFTP_SERVER_PORT_NUM,
    TFTP_SERVER_TOUT,
    TFTP_SERVER_FIREWALL_ENABLE,
  #if (TFTP_SERVER_ROOT_ENABLE)
    TFTP_SERVER_ROOT_FOLDER
  #else
    NULL
  #endif
  };
#endif

#if (TFTP_CLIENT_ENABLE)
  NET_TFTPC_CFG net_tftpc_config = {
    TFTP_CLIENT_BLOCK_SIZE,
    TFTP_CLIENT_RETRY_TOUT,
    TFTP_CLIENT_MAX_RETRY
  };
#endif

#if (FTP_SERVER_ENABLE)
  static NET_FTP_INFO ftp_scb[FTP_SERVER_NUM_SESSIONS];
  #if (FTP_SERVER_AUTH_ENABLE && FTP_SERVER_AUTH_ADMIN)
    static char ftp_user [NET_USERNAME_SIZE];
    static char ftp_passw[NET_PASSWORD_SIZE];
  #endif
  NET_FTP_CFG net_ftp_config = {
    ftp_scb,
    FTP_SERVER_NUM_SESSIONS,
    FTP_SERVER_AUTH_ENABLE,
    FTP_SERVER_PORT_NUM,
    FTP_SERVER_TOUT,
    STRLEN(FTP_SERVER_MESSAGE),
    FTP_SERVER_MESSAGE,
  #if (FTP_SERVER_ROOT_ENABLE)
    FTP_SERVER_ROOT_FOLDER,
  #else
    NULL,
  #endif
  #if (FTP_SERVER_AUTH_ENABLE && FTP_SERVER_AUTH_ADMIN)
    FTP_SERVER_AUTH_USER,
    FTP_SERVER_AUTH_PASS,
    ftp_user,
    ftp_passw
  #else
    NULL,NULL,NULL,NULL
  #endif
  };
#endif

#if (FTP_CLIENT_ENABLE)
  NET_FTPC_CFG net_ftpc_config = {
    FTP_CLIENT_TOUT,
    FTP_CLIENT_PASSIVE_MODE
  };
#endif

#if (DNS_CLIENT_ENABLE)
  static NET_DNS_INFO dns_table[DNS_CLIENT_TAB_SIZE];
  NET_DNS_CFG net_dns_config = {
    dns_table,
    DNS_CLIENT_TAB_SIZE
  };
#endif

#if (SMTP_CLIENT_ENABLE)
  NET_SMTP_CFG net_smtp_config = {
    SMTP_CLIENT_TOUT,
  #if (SMTP_CLIENT_ATTACH_ENABLE)
    /* Attachment support functions */
    { net_smtp_mail_attach,
      net_smtp_mime_header,
      net_smtp_parse_fname
    },
    /* File system interface */
    { netSMTPc_fopen,
      netSMTPc_fread,
      netSMTPc_fclose
    },
  #else
    { NULL,NULL,NULL },
    { NULL,NULL,NULL },
  #endif
  #if defined(RTE_Network_SMTP_Client_TLS)
    &tls_if_func
  #else
    NULL
  #endif
  };
#endif

#if (SNMP_AGENT_ENABLE)
  NET_SNMP_CFG net_snmp_config = {
    SNMP_AGENT_PORT_NUM,
    SNMP_AGENT_TRAP_PORT,
    SNMP_AGENT_TRAP_IP,
    SNMP_AGENT_COMMUNITY
  };
#endif

#if (SNTP_CLIENT_ENABLE)
  NET_SNTP_CFG net_sntp_config = {
    SNTP_CLIENT_NTP_SERVER,
    SNTP_CLIENT_BCAST_MODE
  };
#endif

#if !(ETH0_ENABLE && ETH0_IP4_ENABLE)
  #undef ETH0_IP4_ENABLE
  #undef ETH0_IGMP_ENABLE
  #undef ETH0_NBNS_ENABLE
  #undef ETH0_DHCP_ENABLE
#endif

#if !(ETH0_ENABLE && ETH0_IP6_ENABLE)
  #undef ETH0_IP6_ENABLE
  #undef ETH0_DHCP6_ENABLE
#endif


/* Network scheduler function table */
const net_sys_fn_t net_sys_func[][2] = {
#if (ETH0_ENABLE)
  { net_eth_iface_init,    net_eth_iface_run    },
#endif
#if (PPP_ENABLE)
  { net_ppp_iface_init,    net_ppp_iface_run    },
#endif
#if (SLIP_ENABLE)
  { net_slip_iface_init,   net_slip_iface_run   },
#endif
  { net_loop_iface_init,   net_loop_iface_run   },
#if (IP4_FRAG_ENABLE)
  { net_ip4_frag_init,     net_ip4_frag_run     },
#endif
  { net_ping_client_init,  net_ping_client_run  },
#if (ETH0_IGMP_ENABLE)
  { net_igmp_host_init,    net_igmp_host_run    },
#endif
#if (UDP_ENABLE)
  { net_udp_socket_init,   NULL                 },
#endif
#if (TCP_ENABLE)
  { net_tcp_socket_init,   net_tcp_socket_run   },
#endif
#if (BSD_ENABLE)
  { net_bsd_socket_init,   net_bsd_socket_run   },
 #if (BSD_HOSTNAME_ENABLE)
  { net_bsd_host_init,     NULL                 },
 #endif
#endif
#if (__TLS_USED)
  /* Must initialize before services */
  { netTLS_InterfaceInit,  NULL                 },
#endif
#if (HTTP_SERVER_ENABLE)
  { net_http_server_init,  net_http_server_run  },
#endif
#if (TELNET_SERVER_ENABLE)
  { net_telnet_server_init,net_telnet_server_run},
#endif
#if (TFTP_SERVER_ENABLE)
  { net_tftp_server_init,  net_tftp_server_run  },
#endif
#if (TFTP_CLIENT_ENABLE)
  { net_tftp_client_init,  net_tftp_client_run  },
#endif
#if (FTP_SERVER_ENABLE)
  { net_ftp_server_init,   net_ftp_server_run   },
#endif
#if (FTP_CLIENT_ENABLE)
  { net_ftp_client_init,   net_ftp_client_run   },
#endif
#if (ETH0_NBNS_ENABLE)
  { net_nbns_client_init,  NULL                 },
#endif
#if (ETH0_DHCP_ENABLE)
  { net_dhcp_client_init,  net_dhcp_client_run  },
#endif
#if (ETH0_DHCP6_ENABLE)
  { net_dhcp6_client_init, net_dhcp6_client_run },
#endif
#if (DNS_CLIENT_ENABLE)
  { net_dns_client_init,   net_dns_client_run   },
#endif
#if (SMTP_CLIENT_ENABLE)
  { net_smtp_client_init,  net_smtp_client_run  },
#endif
#if (SNMP_AGENT_ENABLE)
  { net_snmp_agent_init,   net_snmp_agent_run   },
#endif
#if (SNTP_CLIENT_ENABLE)
  { net_sntp_client_init,  net_sntp_client_run  },
#endif
  { NET_SYS_FUNC_END,      NET_SYS_FUNC_END     }
};

/* Network uninit function table */
const net_sys_fn_t net_sys_func_un[] = {
#if (ETH0_ENABLE)
  net_eth_iface_uninit,
#endif
#if (PPP_ENABLE)
  net_ppp_iface_uninit,
#endif
#if (SLIP_ENABLE)
  net_slip_iface_uninit,
#endif
  net_loop_iface_uninit,
#if (IP4_FRAG_ENABLE)
  net_ip4_frag_uninit,
#endif
  net_ping_client_uninit,
#if (ETH0_IGMP_ENABLE)
  net_igmp_host_uninit,
#endif
#if (UDP_ENABLE)
  net_udp_socket_uninit,
#endif
#if (TCP_ENABLE)
  net_tcp_socket_uninit,
#endif
#if (BSD_ENABLE)
  net_bsd_socket_uninit,
 #if (BSD_HOSTNAME_ENABLE)
  net_bsd_host_uninit,
 #endif
#endif
#if (__TLS_USED)
  netTLS_InterfaceUninit,
#endif
#if (HTTP_SERVER_ENABLE)
  net_http_server_uninit,
#endif
#if (TELNET_SERVER_ENABLE)
  net_telnet_server_uninit,
#endif
#if (TFTP_SERVER_ENABLE)
  net_tftp_server_uninit,
#endif
#if (TFTP_CLIENT_ENABLE)
  net_tftp_client_uninit,
#endif
#if (FTP_SERVER_ENABLE)
  net_ftp_server_uninit,
#endif
#if (FTP_CLIENT_ENABLE)
  net_ftp_client_uninit,
#endif
#if (ETH0_NBNS_ENABLE)
  net_nbns_client_uninit,
#endif
#if (ETH0_DHCP_ENABLE)
  net_dhcp_client_uninit,
#endif
#if (ETH0_DHCP6_ENABLE)
  net_dhcp6_client_uninit,
#endif
#if (DNS_CLIENT_ENABLE)
  net_dns_client_uninit,
#endif
#if (SMTP_CLIENT_ENABLE)
  net_smtp_client_uninit,
#endif
#if (SNMP_AGENT_ENABLE)
  net_snmp_agent_uninit,
#endif
#if (SNTP_CLIENT_ENABLE)
  net_sntp_client_uninit,
#endif
  NET_SYS_FUNC_END
};

/* Executable image size optimization */
#if !(ETH0_ENABLE)
/* Empty functions when Ethernet Interface is disabled */
bool net_eth_is_my_addr (const NET_FRAME *frame) {
  (void)frame; return (false); }
const uint8_t *net_eth_get_addr (const __ADDR *addr) {
  (void)addr; return (NULL); }
bool net_eth_send_frame (NET_FRAME *frame, uint8_t ip_ver) {
  (void)frame; (void)ip_ver; return (false); }
netStatus netETH_SendRaw (uint32_t if_num, const uint8_t *buf, uint32_t len) {
  (void)if_num; (void)buf; (void)len; return (netError); }
#ifdef RTE_Network_IPv6
void net_ndp_process (NET_FRAME *frame) {
  (void)frame; }
#endif
#endif

#if !(ETH0_IGMP_ENABLE)
/* Empty functions when IP Multicasting is not enabled */
bool net_igmp_is_member (const uint8_t *ip4_addr) {
  (void)ip4_addr; return (false); }
uint32_t net_igmp_collect_addr (uint8_t *buf) {
  (void)buf; return (0); }
void net_igmp_process (NET_FRAME *frame) {
  (void)frame; }
#endif

#if (ETH0_IP4_ENABLE && !ETH0_DHCP_ENABLE)
/* Empty functions when DHCP not enabled */
netStatus netDHCP_Enable (uint32_t if_num) {
  (void)if_num; return (netError); }
netStatus netDHCP_Disable (uint32_t if_num) {
  (void)if_num; return (netError); }
netStatus netDHCP_SetOption (uint32_t if_num, uint8_t option,
                             const uint8_t *val, uint32_t len) {
  (void)if_num; (void)option; (void)val; (void)len; return (netError); }
#endif

#if (ETH0_IP6_ENABLE && !ETH0_DHCP6_ENABLE)
/* Empty functions when DHCPv6 not enabled */
netStatus netDHCP6_Enable (uint32_t if_num, netDHCP6_Mode mode) {
  (void)if_num; (void)mode; return (netError); }
netStatus netDHCP6_Disable (uint32_t if_num) {
  (void)if_num; return (netError); }
#endif

#if !(PPP_ENABLE)
/* Empty functions when PPP Interface is disabled */
bool net_ppp_send_frame (NET_FRAME *frame, uint16_t prot) {
  (void)frame; (void)prot; return (false); }
#endif

#if !(SLIP_ENABLE)
/* Empty functions when SLIP Interface is disabled */
bool net_slip_send_frame (NET_FRAME *frame, uint8_t ip_ver) {
  (void)frame; (void)ip_ver; return (false); }
#endif

#if !(IP4_FRAG_ENABLE)
/* Empty functions when IP fragmentation not enabled */
NET_FRAME *net_ip4_frag_add (NET_FRAME *frame) {
  return (frame); }
NET_FRAME *net_ip4_frag_get (NET_FRAME *frame, uint16_t mtu) {
  (void)mtu; return (frame); }
#endif

#if !(UDP_ENABLE)
/* Empty function when UDP socket not enabled */
void net_udp_process (NET_FRAME *frame, uint8_t ip_ver) {
  (void)frame; (void)ip_ver; }
#endif

#if !(TCP_ENABLE)
/* Empty function when TCP socket not enabled */
void net_tcp_process (NET_FRAME *frame, uint8_t ip_ver) {
  (void)frame; (void)ip_ver; }
#endif

#if !(SNTP_CLIENT_ENABLE)
/* Empty function when SNTP not enabled */
netStatus netSNTPc_GetTime (const NET_ADDR *addr, netSNTPc_cb_t cb_func) {
  (void)addr; (void)cb_func; return (netError); }
#endif

#if (HTTP_SERVER_ENABLE && defined(RTE_Network_Web_Server_RO))
/* Empty interface functions for Compact Web server */
__weak void *netHTTPs_fopen (const char *fname) {
  (void)fname; return (NULL); }
__weak void netHTTPs_fclose (void *file) {
  (void)file; }
__weak uint32_t netHTTPs_fread (void *file, uint8_t *buf, uint32_t len) {
  (void)file; (void)buf; (void)len; return (0); }
__weak char *netHTTPs_fgets (void *file, char *buf, uint32_t size) {
  (void)file; (void)buf; (void)size; return (NULL); }
__weak void netHTTPs_fstat (const char *fname, uint32_t *fsize, uint32_t *ftime) {
  (void)fname; (void)fsize; (void)ftime; }
#endif
