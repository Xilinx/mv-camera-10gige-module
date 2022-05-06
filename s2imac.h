/******************************************************************************/
/*  Linux driver header for the s2imac device.                                */
/*  Copyright (c) 2022. Sensor to Image GmbH.						          */
/*                                                                            */
/*  This program is free software; you can redistribute it and/or modify      */
/*  it under the terms of the GNU General Public License version 2 as         */
/*  published by the Free Software Foundation.								  */
/*----------------------------------------------------------------------------*/
/*    File :  s2imac.h                                                        */
/*    Date :  2021-09-29                                                      */
/*     Rev :  1.4                                                             */
/*  Author :  RW                                                              */
/*----------------------------------------------------------------------------*/
/*  GigE Vision control protocol                                              */
/*----------------------------------------------------------------------------*/
/*  0.1  |  2018-03-15  |  RW  |  Initial release                             */
/*  0.2  |  2018-05-29  |  RW  |  Version to 1.1.4                            */
/*  0.3  |  2018-07-05  |  RW  |  Version to 1.1.5                            */
/*  0.4  |  2018-09-05  |  RW  |  Add TIMEOUT_READ_WRITE_PHY define           */
/*       |              |      |  Version to 1.1.6                            */
/*  0.5  |  2018-10-18  |  RW  |  Version to 1.1.7                            */
/*  0.6  |  2018-11-05  |  RW  |  Added MDIO_CLOCK and MDIO_CLOCK_DIV defines */
/*       |              |      |  Version to 1.1.8                            */
/*  0.7  |  2019-01-04  |  RW  |  Add rx/tx buffer to net_local structure     */
/*       |              |      |  Add zynqmp arch                             */
/*       |              |      |  Version to 1.1.9                            */
/*  0.8  |  2019-08-08  |  RW  |  Version to 1.2.0                            */
/*  0.9  |  2019-08-23  |  RW  |  Version to 1.2.1                            */
/*  0.10 |  2019-09-02  |  RW  |  Version to 1.2.2                            */
/*  1.0  |  2019-11-18  |  RW  |  Version to 1.3.0                            */
/*  1.1  |  2020-12-15  |  RW  |  Set little-endian mode for linux            */ 
/*       |              |      |  Added config-interface to device tree       */ 
/*       |              |      |  Version to 1.3.1                            */
/*  1.2  |  2021-01-29  |  RW  |  Version to 1.3.2                            */
/*  1.3  |  2021-08-31  |  RW  |  Added Xilinx 10/25-Gigabit Ethernet         */
/*       |              |      |  PCS/PMA IP core                             */ 
/*       |              |      |  Version to 1.4.0                            */
/*  1.4  |  2021-09-29  |  RW  |  Version to 1.4.1                            */
/******************************************************************************/
#ifndef _S2IMAC_H_
#define _S2IMAC_H_

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/io.h>

#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
#define HAVE_TRANS_START_HELPER
#endif

#define DRIVER_NAME "s2imac"
#define DRIVER_VERS "1.4.1"

#define TIMEOUT_READ_WRITE_PHY  2000
 
//#define NBASE_T_MARVELL 1
 
//#define CONFIG_S2IMAC_POLLING 1

/* MDIO register basis */
#define MDIO_BASE   (lp->base_addr[S2IMAC_GIGEMAP] + 0x1000)
#define MDIO_BASE1  (lp->base_addr[S2IMAC_GIGEMAP] + 0x3000)

/* Tx/Rx buffer basis */
#define TXBUF         (lp->base_addr[S2IMAC_GIGEMAP] + 0x4000)
#define RXBUF         (lp->base_addr[S2IMAC_GIGEMAP] + 0x8000)

/* MAC registers definition */
#define SPEED   (lp->base_addr[S2IMAC_GIGEMAP] + (0x100 << 2))
#define RCW1    (lp->base_addr[S2IMAC_GIGEMAP] + (0x240 << 2))
#define TC      (lp->base_addr[S2IMAC_GIGEMAP] + (0x280 << 2))
#define EMMC    (lp->base_addr[S2IMAC_GIGEMAP] + (0x300 << 2))
#define MC      (lp->base_addr[S2IMAC_GIGEMAP] + (0x340 << 2))
#define UAW0    (lp->base_addr[S2IMAC_GIGEMAP] + (0x380 << 2))
#define UAW1    (lp->base_addr[S2IMAC_GIGEMAP] + (0x384 << 2))
#define AFM     (lp->base_addr[S2IMAC_GIGEMAP] + (0x390 << 2))




#define MDIO_RXTX_JUMBO   (1 << 30)
#define MDIO_RXTX_ENABLE  (1 << 28)
#define MDIO_RXTX_HALFDUPLEX  (1 << 26)

#define MDIO_EMMC_10BASET (0)
#define MDIO_EMMC_100BASET  (1 << 30)
#define MDIO_EMMC_1000BASET (1 << 31)

#define MDIO_ENABLE_MASK  (1 << 6)
#define MDIO_CLOCK_DIV_MASK ((1 << 6) - 1)

#define MDIO_ENABLE_MASK1  (1 << 5)
#define MDIO_CLOCK_DIV_MASK1 ((1 << 5) - 1)

#define MDIO_CLOCK  (2500000) /* 2.5 MHz */
#define MDIO_CLOCK_DIV  (MDIO_CLOCK_DIV_MASK & \
      ((lp->coreclk / (2 * MDIO_CLOCK)) - 1))

#define MDIO_CLOCK_DIV1  (MDIO_CLOCK_DIV_MASK1 & \
      ((lp->coreclk / (2 * MDIO_CLOCK)) - 1))
      
/* direct registers definition */
#define GCSR        (lp->base_addr[S2IMAC_GIGEMAP] + 0xC000)
#define CLK_FREQ    (lp->base_addr[S2IMAC_GIGEMAP] + 0xC008)
#define TOCNT_DIV   (lp->base_addr[S2IMAC_GIGEMAP] + 0xC014)
#define MDIO_ACC    (lp->base_addr[S2IMAC_GIGEMAP] + 0xC028)
#define ETHSIZE     (lp->base_addr[S2IMAC_GIGEMAP] + 0xC02C)
#define MAC_HIGH    (lp->base_addr[S2IMAC_GIGEMAP] + 0xC030)
#define MAC_LOW     (lp->base_addr[S2IMAC_GIGEMAP] + 0xC034)
#define TX_LEN      (lp->base_addr[S2IMAC_GIGEMAP] + 0xC048)
#define RX_LEN      (lp->base_addr[S2IMAC_GIGEMAP] + 0xC04C)
#define MAC_RX_HIGH (lp->base_addr[S2IMAC_GIGEMAP] + 0xC090)
#define MAC_RX_LOW  (lp->base_addr[S2IMAC_GIGEMAP] + 0xC094)

#ifndef CONFIG_S2IMAC_POLLING
#define INT_MASK    (lp->base_addr[S2IMAC_GIGEMAP] + 0xC0AC)
#define INT_REQ     (lp->base_addr[S2IMAC_GIGEMAP] + 0xC0B0)
#endif

#define ID_REG      (lp->base_addr[S2IMAC_GIGEMAP] + 0xC0B4)


#define GCSR_CONNECTED      0x00000001
#define GCSR_IP_OK          0x00000002
#define GCSR_LITTLE_ENDIAN  0x01000000

#define GCSR_RST_PHY    (1 << 2)

#define MDIO_ACC_ENA_NBLOCK (1 << 31)
#define MDIO_ACC_NBLOCK   (1 << 16)
#define MDIO_ACC_MIIRDY   (1 << 17)

#ifndef CONFIG_S2IMAC_POLLING
#define INT_ENABLE_RX   (1)
#define INT_ENABLE_TX   (1 << 1)
#define INT_ENABLE_IRQS   (1 << 31)
#endif

/* max MTU size of an ... */
#define XTE_MTU     1500  /* Ethernet frame */
#define XTE_JUMBO_MTU   8982  /* jumbo Ethernet frame, not supported by the GigE core */
#define XTE_HDR_SIZE    14  /* Ethernet header */
#define XTE_HDR_VLAN_SIZE 18  /* Ethernet header with VLAN */
#define XTE_TRL_SIZE    4 /* Ethernet trailer (FCS) */
#define XTE_MAX_FRAME_SIZE  (XTE_MTU + XTE_HDR_SIZE + XTE_TRL_SIZE)
#define XTE_MAX_VLAN_FRAME_SIZE (XTE_MTU + XTE_HDR_VLAN_SIZE + XTE_TRL_SIZE)
#define XTE_MAX_JUMBO_FRAME_SIZE (XTE_JUMBO_MTU + XTE_HDR_SIZE + XTE_TRL_SIZE)

/* Tx timeout is 60 seconds. */
#define TX_TIMEOUT    (60*HZ)

/* BUFFER_ALIGN(adr) calculates the number of bytes to the next alignment. */
#define ALIGNMENT   4
#define BUFFER_ALIGN(adr) ((ALIGNMENT - ((ulong)adr)) % ALIGNMENT)

/* Read/Write access to the registers */
#ifndef in_be32
#if defined(CONFIG_ARCH_ZYNQ) || defined(CONFIG_ARCH_ZYNQMP)
#define in_be32(offset)   __raw_readl(offset)
#define out_be32(offset, val) __raw_writel(val, offset)
#endif
#endif

#define DWORD_SWAP(x) ((((x) << 24) & 0xff000000L) | \
       (((x) <<  8) & 0x00ff0000L) | \
       (((x) >>  8) & 0x0000ff00L) | \
       (((x) >> 24) & 0x000000ffL) )

# define WORD_SWAP(x) ((((x) << 8) & 0xff00) | (((x) >> 8) & 0x00ff)) 

// Types of the Ethernet PHYs
#define PHY_BROADCOM    0x00
#define PHY_MARVELL     0x01
#define PHY_NATIONAL    0x02
#define PHY_MICREL      0x03
#define PHY_10G_AEL2005 0x04
#define PHY_10G_TN80XX  0x05
#define PHY_10G_PCS_PMA 0x06
#define PHY_1G_PCS_PMA  0x07
#define PHY_TI          0x08
#define PHY_NBASET_MRVL 0x09
#define PHY_NBASET_AQR  0x0A
#define PHY_SWITCH_MRVL 0x0B
#define PHY_25G_PCS_PMA 0x0C
#define PHY_NONE        0xFF

// NBASE-T MAC TX link speed throttling constants
#define NBASET_SPD_10G  0
#define NBASET_SPD_5G   1
#define NBASET_SPD_2500 3
#define NBASET_SPD_1000 9

// Host interface mode
typedef enum phy_if_mode {DEFAULT, XAUI, RXAUI, USXGMII} phy_if_mode;

/**send_data
 * struct net_local - Our private per device data
 * @ndev:   instance of the network device
 * @base_addr[]:  base address of the s2imac device
 * @coreclk:    ip core clock frequency in Hz
 * @regmem_end:   physical end of GigEV register bank
 * @regmem_start: physical start of GigEV register bank
 * @reset_lock:   lock used for synchronization
 * @deferred_skb: holds an skb (for transmission at a later time) when the
 *      Tx buffer is not free
 * @phy_dev:    pointer to the PHY device
 * @phy_node:   pointer to the PHY device node
 * @last_link:    last link status
 * @cur_speed:    current speed
 */
struct net_local {

  struct net_device *ndev;

#define S2IMAC_GIGEMAP        0
#define S2IMAC_VIDEOMAP       1
#define S2IMAC_FRAMEBUFFERMAP 2
#define MAX_S2IMAC_MAPS       3
  
  void __iomem *base_addr[MAX_S2IMAC_MAPS];
  unsigned int coreclk;

  unsigned long video_end;
  unsigned long video_start;

  unsigned long framebuffer_end;
  unsigned long framebuffer_start;
  

  spinlock_t reset_lock;
  struct sk_buff *deferred_skb;

  /* FIXME: use mdio bus: struct phy_device *phy_dev; */
  /* FIXME: use mdio bus: struct device_node *phy_node; */
  u32 phy_addr;   /* FIXME: really global, use mdio bus */

  int last_link;
  int cur_speed;

  u8 phy_type; 
  u8 init_phy; 

  u32 *gige_txbuf;
  u32 *gige_rxbuf;

#ifdef CONFIG_S2IMAC_POLLING
  struct timer_list irq_timer;  /* ISR polling timer */
#endif

  struct timer_list phy_timer;  /* PHY monitoring timer */
  
  phy_if_mode config_interface;

  void __iomem *phy_base_addr;

  unsigned long phy_base_end;
  unsigned long phy_base_start;
};

/* some useful structs and typedefs for s2imac.c */
typedef struct {
  __u32 address;
  __u32 data;
} s2igen_content_t;

/* some useful defines for s2imac.c */
#define SIOS2IGENRD   SIOCDEVPRIVATE+0  /* generic read */
#define SIOS2IGENWR   SIOCDEVPRIVATE+1  /* generic write */

/* old defines */
#define SIOS2IREGRD   SIOCDEVPRIVATE+2  /* register read */
#define SIOS2IREGWR   SIOCDEVPRIVATE+3  /* register write */

/* new defines */
#define SIOS2IVIDEORD   SIOCDEVPRIVATE+2    /* video read */
#define SIOS2IVIDEOWR   SIOCDEVPRIVATE+3    /* video write */
#define SIOS2IFBUFRD    SIOCDEVPRIVATE+4    /* framebuffer read */
#define SIOS2IFBUFWR    SIOCDEVPRIVATE+5    /* framebuffer write */

#endif
