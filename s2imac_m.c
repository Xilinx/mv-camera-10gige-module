/******************************************************************************/
/*  Linux driver for the s2imac device.                                       */
/*  Copyright (c) 2022. Sensor to Image GmbH.                                 */
/*                                                                            */
/*  This program is free software; you can redistribute it and/or modify      */
/*  it under the terms of the GNU General Public License version 2 as         */
/*  published by the Free Software Foundation.                                */
/*----------------------------------------------------------------------------*/
/*    File :  s2imac_m.c                                                      */
/*    Date :  2021-09-29                                                      */
/*     Rev :  1.4                                                             */
/*  Author :  RW                                                              */
/*----------------------------------------------------------------------------*/
/*  GigE Vision control protocol                                              */
/*----------------------------------------------------------------------------*/
/*  0.1  |  2010-10     |  RW  |  Initial release                             */
/*  0.2  |  2011-01     |  RW  |  Code improvements and coding style          */
/*       |              |      |  corrections                                 */  
/*  0.3  |  2011-02     |  RW  |  Change over to an interrupt driven network  */
/*       |              |      |  driver                                      */
/*  0.4  |  2013-07     |  RW  |  Added broadcom BCM54610 phy support         */
/*  0.5  |  2013-12     |  RW  |  Remove swab bytes in send and receive data  */
/*       |              |      |  (is made in fpga)                           */
/*  0.6  |  2016-04     |  RW  |  Support zynq                                */
/*  0.7  |  2016-11-23  |  RW  |  Bug fix:when data with odd length is sent   */
/*       |              |      |  Version to 1.1.0                            */
/*  0.8  |  2016-12-06  |  RW  |  Add support for Marwell 88e1512 PHY         */
/*       |              |      |  Problem (crash) fixed when driver is        */
/*       |              |      |  unloaded as a module                        */
/*       |              |      |  Check for maximum MTU size of 1500 bytes    */
/*       |              |      |  in s2imac_change_mtu function               */
/*       |              |      |  Display driver version and copyright when   */
/*       |              |      |  started                                     */
/*       |              |      |  Version to 1.1.1                            */
/*  0.8  |  2017-05-16  |  RW  |  Description was changed                     */
/*  0.9  |  2017-12-04  |  RW  |  Fix compile error for linux 4.7 and greater */
/*       |              |      |  Version to 1.1.2                            */
/*  0.10 |  2018-03-15  |  RW  |  Add Marvel NBASE T Phy                      */
/*       |              |      |  Version to 1.1.3                            */
/*  0.11 |  2018-05-29  |  RW  |  Wait after close the network device         */
/*  0.12 |  2018-07-05  |  RW  |  Remove wait after close the network device  */
/*       |              |      |  Clear rx buffer when close the device       */
/*  0.13 |  2018-09-05  |  RW  |  Add timeout in read/write ethernet PHY regs */
/*       |              |      |  remove not used code in ethtool_get_settings*/
/*  0.14 |  2018-10-29  |  RW  |  Clear XGigE interrupts and clear RX_LEN     */
/*       |              |      |  register in s2imac_enable_interrupts        */
/*       |              |      |  Clear always the rx interrupt in            */
/*       |              |      |  s2imac_rx_handler function                  */
/*  0.15 |  2018-11-05  |  RW  |  Set current parameter to phy_setup function */
/*       |              |      |  Fixed the build warnings                    */
/*       |              |      |  Moved MDIO_CLOCK and MDIO_CLOCK_DIV defines */
/*       |              |      |  to s2imac.h header file                     */
/*       |              |      |  Removed unused code                         */
/*  0.16 |  2019-01-04  |  RW  |  Add rx/tx buffer to net_local structure     */
/*       |              |      |  Set the base address pointers to 0 if unmap */
/*       |              |      |  Get the available register name from the    */
/*       |              |      |  device tree (core, video, framebuffer)      */
/*       |              |      |  e.q. reg-names = "core", "framebuffer";     */
/*       |              |      |  Change the error handling in s2imac_of_probe*/
/*       |              |      |  if no phy found chancel s2imac_of_probe     */
/*  0.17 |  2019-08-08  |  RW  |  Don't set the gcsr connected bit in         */
/*       |              |      |  set_mac_speed function                      */ 
/*  0.18 |  2019-08-23  |  RW  |  If link is up than set gcsr connected bit   */
/*       |              |      |  in open function                            */ 
/*       |              |      |  Fixed bug to get iospace for video and      */ 
/*       |              |      |  framebuffer                                 */ 
/*  0.19 |  2019-09-02  |  RW  |  Added s2imac_set_rx_mode function to enable */ 
/*       |              |      |  or disable multicast mode                   */ 
/*       |              |      |  Change error outputs for register mapping   */ 
/*  1.0  |  2019-11-18  |  RW  |  Added NBASE-T Marvell PH support            */ 
/*       |              |      |  Rename file to s2imac_m.c                   */ 
/*  1.1  |  2020-12-15  |  RW  |  Set little-endian mode for linux            */ 
/*       |              |      |  Added config-interface to device tree       */ 
/*  1.2  |  2021-01-29  |  RW  |  Add new timer functions was changed by      */
/*       |              |      |  kernel 4.15.0                               */
/*       |              |      |  Struct ethtool_ops was changed by kernel    */
/*       |              |      |  add new get and set link functions          */ 
/*  1.3  |  2021-08-31  |  RW  |  Added Xilinx 10/25-Gigabit Ethernet         */ 
/*       |              |      |  PCS/PMA IP core                             */ 
/*  1.4  |  2021-09-29  |  RW  |  struct ethtool_link_ksettings was changed by*/
/*       |              |      |  kernel 4.9.212                              */
/******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/io.h>
#include <linux/delay.h>

#include "s2imac.h"

#ifdef NBASE_T_MARVELL
/* include Marvel NBASE T Phy functions */
#include "m88x33xx/m88x33xx.h"
#endif

#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/version.h>

/*************************/
/* s2imac driver calls */
/*************************/

#ifndef CONFIG_S2IMAC_POLLING
/**
 * s2imac_enable_interrupts - Enable the interrupts for the s2imac device
 * @drvdata:  Pointer to the s2imac device private data
 *
 * This function enables the Tx and Rx interrupts for the s2imac device along
 * with the Global Interrupt Enable.
 */
static void s2imac_enable_interrupts(struct net_local *lp)
{
  u32 reg_data;

  /* clear XGigE interrupts */
  out_be32 ((u32 *) INT_REQ,  INT_ENABLE_TX | INT_ENABLE_RX);
  
  /* get interrput mask register */
  reg_data = in_be32((u32 *) INT_MASK);

  /* Enable the Tx interrupts, enable Rx interrupts and enable the global interrupt */
  out_be32 ((u32 *) INT_MASK, reg_data | INT_ENABLE_TX | INT_ENABLE_RX | INT_ENABLE_IRQS);
  
  /* clear RX_LEN register */  
  out_be32 ((u32 *) RX_LEN, 0);
}

/**
 * s2imac_disable_interrupts - Disable the interrupts for the s2imac device
 * @drvdata:  Pointer to the s2imac device private data
 *
 * This function disables the Tx and Rx interrupts for the s2imac device,
 * along with the Global Interrupt Enable.
 */
static void s2imac_disable_interrupts(struct net_local *lp)
{
  u32 reg_data;
  
  /* get interrput mask register */
  reg_data = in_be32((u32 *) INT_MASK);

  /* disable the Tx interrupts, disable Rx interrupts and disable the global interrupt */
  out_be32 ((u32 *) INT_MASK, reg_data & (~INT_ENABLE_TX) & (~INT_ENABLE_RX) & (~INT_ENABLE_IRQS));
}
#endif

/* Read Ethernet PHY register */
static u16 s2imac_phy_read (struct net_local *lp, u8 reg_addr)
{
  u32 ret;
  int retries = 0;
  
  if (in_be32 ((u32 *) MDIO_ACC) & MDIO_ACC_NBLOCK) {
    /*
     * Non-blocking mode
     * (only this software thread waits for MDIO access)
     */
    ret = in_be32 ((u32 *) (MDIO_BASE + ((lp->phy_addr & 0x1F) << 7)
          + ((reg_addr & 0x1F) << 2)));

    /*
     * Wait here polling, until the value is ready to be read.
     * Should we avoid endless loop due to hardware?
     */
    do {
      ret = in_be32 ((u32 *) MDIO_ACC);
      retries++;
      if(retries > TIMEOUT_READ_WRITE_PHY)
        break;
    } while (!(ret & MDIO_ACC_MIIRDY));
    /*
     * read the value
     */
    ret = in_be32 ((u32 *) MDIO_ACC);
  } else {
    /*
     * Blocking mode
     * (whole system waits for the bus transaction to finish)
     */
    ret = in_be32 ((u32 *) (MDIO_BASE + ((lp->phy_addr & 0x1F) << 7)
          + ((reg_addr & 0x1F) << 2)));
  }
  return (u16) (ret & 0xFFFF);
}

static u16 s2imac_phy_read1 (struct net_local *lp, u8 reg_addr)
{
  u32 ret;
  int retries = 0;

  if (in_be32 ((u32 *) MDIO_ACC) & MDIO_ACC_NBLOCK) {
    /*
     * Non-blocking mode
     * (only this software thread waits for MDIO access)
     */
    ret = in_be32 ((u32 *) (MDIO_BASE1 + ((lp->phy_addr & 0x1F) << 7)
          + ((reg_addr & 0x1F) << 2)));

    /*
     * Wait here polling, until the value is ready to be read.
     * Should we avoid endless loop due to hardware?
     */
    do {
      ret = in_be32 ((u32 *) MDIO_ACC);
      retries++;
      if(retries > TIMEOUT_READ_WRITE_PHY)
        break;
    } while (!(ret & MDIO_ACC_MIIRDY));
    /*
     * read the value
     */
    ret = in_be32 ((u32 *) MDIO_ACC);
  } else {
    /*
     * Blocking mode
     * (whole system waits for the bus transaction to finish)
     */
    ret = in_be32 ((u32 *) (MDIO_BASE1 + ((lp->phy_addr & 0x1F) << 7)
          + ((reg_addr & 0x1F) << 2)));
  }
  return (u16) (ret & 0xFFFF);
}

/* Write Ethernet PHY register */
static void s2imac_phy_write (struct net_local *lp, u8 reg_addr, u16 reg_data)
{
  u32 ret;
  int retries = 0;

  if (in_be32 ((u32 *) MDIO_ACC) & MDIO_ACC_NBLOCK) {
    /*
     * Non-blocking mode
     * (only this software thread waits for MDIO access)
     */
    out_be32 ((u32 *) (MDIO_BASE + (lp->phy_addr << 7)
           + ((reg_addr) << 2)), reg_data);

    /*
     * Wait here polling, until the value is ready written.
     * Should we avoid endless loop due to hardware?
     */
    do {
      ret = in_be32 ((u32 *) MDIO_ACC);
      retries++;
      if(retries > TIMEOUT_READ_WRITE_PHY)
        break;
    } while (!(ret & MDIO_ACC_MIIRDY));
  } else {
    /*
     * Blocking mode
     * (whole system waits for the bus transaction to finish)
     */
    out_be32 ((u32 *) (MDIO_BASE + (lp->phy_addr << 7)
           + ((reg_addr) << 2)), reg_data);
  }
}

static void s2imac_phy_write_addr (struct net_local *lp, u8 reg_addr, u16 reg_data)
{
  u32 ret;
  int retries = 0;

  if (in_be32 ((u32 *) MDIO_ACC) & MDIO_ACC_NBLOCK) {
    /*
     * Non-blocking mode
     * (only this software thread waits for MDIO access)
     */
    out_be32 ((u32 *) (MDIO_BASE1 + (lp->phy_addr << 7)
           + ((reg_addr) << 2)), reg_data);

    /*
     * Wait here polling, until the value is ready written.
     * Should we avoid endless loop due to hardware?
     */
    do {
      ret = in_be32 ((u32 *) MDIO_ACC);
      retries++;
      if(retries > TIMEOUT_READ_WRITE_PHY)
        break;
    } while (!(ret & MDIO_ACC_MIIRDY));
  } else {
    /*
     * Blocking mode
     * (whole system waits for the bus transaction to finish)
     */
    out_be32 ((u32 *) (MDIO_BASE1 + (lp->phy_addr << 7)
           + ((reg_addr) << 2)), reg_data);
  }
}

/* Extended PHY read */
u16 s2imac_mdio_read(struct net_local *lp, u8 reg, u16 addr)
{
    s2imac_phy_write_addr(lp, reg, addr);
    return s2imac_phy_read1(lp, reg);
}

/* Extended PHY write */
void s2imac_mdio_write(struct net_local *lp, u8 reg, u16 addr, u16 val)
{
    s2imac_phy_write_addr(lp, reg, addr);
    s2imac_phy_write(lp, reg, val);
    return;
}

/*
 * Detect the PHY address by scanning addresses 0 to 31 and
 * looking at the MII status register (register 1) and assuming
 * the PHY supports 10Mbps full/half duplex. Feel free to change
 * this code to match your PHY, or hardcode the address if needed.
 *
 * Use MII register 1 (MII status register) to detect PHY
 *
 * Mask used to verify certain PHY features (or register contents)
 * in the register above:
 *  0x1000: 10Mbps full duplex support
 *  0x0800: 10Mbps half duplex support
 *  0x0008: Auto-negotiation support
 */
#define PHY_DETECT_REG    1
#define PHY_DETECT_MASK   0x1808
static int detect_phy (struct net_local *lp, struct device *dev)
{
  u16 phy_reg;

  for (lp->phy_addr = 31; (int)lp->phy_addr >= 0; lp->phy_addr--) {
    phy_reg = s2imac_phy_read (lp, PHY_DETECT_REG);
    if (phy_reg != 0xFFFF) {
//    if ((phy_reg != 0xFFFF) && ((phy_reg & PHY_DETECT_MASK)
//              == PHY_DETECT_MASK)) {
      /* Found a valid PHY address */
      dev_info (dev, "PHY detected at address %d.\n",
          lp->phy_addr);
      return lp->phy_addr;
    }
  }
  /* default to zero */
  dev_info (dev, "No PHY detected. Assuming a PHY at address  %d.\n", lp->phy_addr);
  return lp->phy_addr;
}

static void set_mac_speed (struct net_local *lp)
{
  u32 cfg  = 0x80000000;      /* Default 1 Gbps */
  u32 rxtx = 0x10000000;      /* Enabled by default */
  struct net_device *ndev = lp->ndev;
  u32 id;
//  u16 tmp;
//  int i, retries;
//  unsigned retries = 10;

  /* wait for link up */
//  while (retries-- && ((s2imac_phy_read (lp, 1) & 0x24) != 0x24)) ;

  /* get PHY id */
  id = (s2imac_phy_read (lp, 2) << 16)
      | s2imac_phy_read (lp, 3);

  /*
   * Marwell 88e1111 id 0x1410cc2 - ml50x, ml605, ZC702, ZC706, / 88e1512 id 0x1410dd1 - SVDK
   */
  /* FIXME this part will be replaced by PHY lib */
  if ((id == 0x1410cc2) || (id == 0x1410dd1)) {
    switch ((s2imac_phy_read (lp, 0x11)) & 0xE000) {
    case 0x0000:
      /* 10BASE-T, half-duplex */
      cfg = MDIO_EMMC_10BASET;
      rxtx = (MDIO_RXTX_ENABLE | MDIO_RXTX_HALFDUPLEX);
      lp->cur_speed = 10;
      dev_info (&ndev->dev, "speed set to 10BASE-T/HD\n");
      break;
    case 0x2000:
      /* 10BASE-T, full-duplex */
      cfg = MDIO_EMMC_10BASET;
      rxtx = MDIO_RXTX_ENABLE;
      lp->cur_speed = 10;
      dev_info (&ndev->dev, "speed set to 10BASE-T/FD\n");
      break;
    case 0x4000:
      /* 100BASE-TX, half-duplex */
      cfg = MDIO_EMMC_100BASET;
      rxtx = (MDIO_RXTX_ENABLE | MDIO_RXTX_HALFDUPLEX);
      lp->cur_speed = 100;
      dev_info (&ndev->dev, "speed set to 100BASE-T/HD\n");
      break;
    case 0x6000:
      /* 100BASE-TX, full-duplex */
      cfg = MDIO_EMMC_100BASET;
      rxtx = MDIO_RXTX_ENABLE;
      lp->cur_speed = 100;
      dev_info (&ndev->dev, "speed set to 100BASE-T/FD\n");
      break;
    case 0x8000:
      /* 1000BASE-T, half-duplex */
      cfg = MDIO_EMMC_1000BASET;
      rxtx = (MDIO_RXTX_ENABLE | MDIO_RXTX_HALFDUPLEX);
      lp->cur_speed = 1000;
      dev_info (&ndev->dev, "speed set to 1000BASE-T/HD\n");
      break;
    case 0xA000:
      /* 1000BASE-T, full-duplex */
      cfg = MDIO_EMMC_1000BASET;
      rxtx = MDIO_RXTX_ENABLE;
      lp->cur_speed = 1000;
      dev_info (&ndev->dev, "speed set to 1000BASE-T/FD\n");
      break;
    default:
      return;
    }
    lp->phy_type = PHY_MARVELL;
  }
  else if(id == 0x0143BD63){
    /*
    * Broadcom BCM54610 id 0x0143BD63
    */
    switch ((s2imac_phy_read (lp, 0x19)) & 0x0700) {
    case 0x0100:
      /* 10BASE-T, half-duplex */
      cfg = MDIO_EMMC_10BASET;
      rxtx = (MDIO_RXTX_ENABLE | MDIO_RXTX_HALFDUPLEX);
      lp->cur_speed = 10;
      dev_info (&ndev->dev, "speed set to 10BASE-T/HD\n");
      break;
    case 0x0200:
      /* 10BASE-T, full-duplex */
      cfg = MDIO_EMMC_10BASET;
      rxtx = MDIO_RXTX_ENABLE;
      lp->cur_speed = 10;
      dev_info (&ndev->dev, "speed set to 10BASE-T/FD\n");
      break;
    case 0x0300:
      /* 100BASE-TX, half-duplex */
      cfg = MDIO_EMMC_100BASET;
      rxtx = (MDIO_RXTX_ENABLE | MDIO_RXTX_HALFDUPLEX);
      lp->cur_speed = 100;
      dev_info (&ndev->dev, "speed set to 100BASE-T/HD\n");
      break;
    case 0x0500:
      /* 100BASE-TX, full-duplex */
      cfg = MDIO_EMMC_100BASET;
      rxtx = MDIO_RXTX_ENABLE;
      lp->cur_speed = 100;
      dev_info (&ndev->dev, "speed set to 100BASE-T/FD\n");
      break;
    case 0x0600:
      /* 1000BASE-T, half-duplex */
      cfg = MDIO_EMMC_1000BASET;
      rxtx = (MDIO_RXTX_ENABLE | MDIO_RXTX_HALFDUPLEX);
      lp->cur_speed = 1000;
      dev_info (&ndev->dev, "speed set to 1000BASE-T/HD\n");
      break;
    case 0x0700:
      /* 1000BASE-T, full-duplex */
      cfg = MDIO_EMMC_1000BASET;
      rxtx = MDIO_RXTX_ENABLE;
      lp->cur_speed = 1000;
      dev_info (&ndev->dev, "speed set to 1000BASE-T/FD\n");
      break;
    default:
      return;
    }
    /* Set PHY autonegotiation registers */
    s2imac_phy_write (lp, 4, 0x01E1);
    s2imac_phy_write (lp, 9, 0x0300);
    /* Setup PHY LEDs
    *   - activity LED indicates even link utilization
    *   - LED1 = off, LED2 = activity
    *   - LED3 = off, LED4 = link quality
    */
    s2imac_phy_write (lp, 0x1C, 0xA40B); 
    s2imac_phy_write (lp, 0x1C, 0xB43E);
    s2imac_phy_write (lp, 0x1C, 0xB87E);
    
    lp->phy_type = PHY_BROADCOM;
  }
  else {

    if(lp->phy_base_addr == 0) {
      if(lp->init_phy == 0) {
          out_be32 ((u32 *) MC, MDIO_ENABLE_MASK1 | MDIO_CLOCK_DIV1);
        lp->phy_addr = detect_phy (lp, &ndev->dev);
      }

      /* check if phy is NBASE-T Marvell PHY */
      id = ((s2imac_mdio_read (lp, 1, 2) << 16) | s2imac_mdio_read (lp, 1, 3)) & 0xFFFFFFFC;
    } else 
      id = 0;
    
    if(id == 0x2B09A8) {
#ifdef NBASE_T_MARVELL

      if(lp->init_phy == 0) {

        dev_info (&ndev->dev, "Init NBASE-T Marvell PHY [%X] ...\n", id);
        
        s2imac_mdio_write(lp, 31, 0xF001, (s2imac_mdio_read(lp, 31, 0xF001) & 0xFFF8) | 0x0001);  // XAUI mode
        s2imac_mdio_write(lp, 31, 0xF001,  s2imac_mdio_read(lp, 31, 0xF001)           | 0x8000);  // SW reset
  
        i = m88x33xx_init(lp, RXAUI);
        if(i != 0)
          dev_err (&ndev->dev, "Error m88x33xx_init: %d\n", i);
        else {
          lp->init_phy = 1;
          
          retries = 50;
          while (retries-- && ((s2imac_mdio_read (lp, 0x03, 0x8008) & 0x0400) == 0)) 
            //udelay(200000);
            mdelay(200);
        }
      }
      
      /* Setup XGMAC speed throttling and GEV link speed register */
      tmp = s2imac_mdio_read(lp, 0x03, 0x8008);
      switch (tmp & 0xC000) {
          /* 10Mbps */
          case 0x0000:    out_be32 ((u32 *) SPEED, NBASET_SPD_1000);
                          lp->cur_speed = 10;
                          dev_info (&ndev->dev, "speed set to 10BASE-T/FD\n");
                          break;
          /* 100Mbps */
          case 0x4000:    out_be32 ((u32 *) SPEED, NBASET_SPD_1000);
                          lp->cur_speed = 100;
                          dev_info (&ndev->dev, "speed set to 100BASE-T/FD\n");
                          break;
          /* 1Gbps */
          case 0x8000:    out_be32 ((u32 *) SPEED, NBASET_SPD_1000);
                          lp->cur_speed = 1000;
                          dev_info (&ndev->dev, "speed set to 1000BASE-T/FD\n");
                          break;
          /* 10Gbps + NBASE-T */
          case 0xC000:    switch (tmp & 0x000C) {
          /* 2.5Gbps */
          case 0x0004:    out_be32 ((u32 *) SPEED, NBASET_SPD_2500);
                          lp->cur_speed = 2500;
                          dev_info (&ndev->dev, "speed set to 2500BASE-T/FD\n");
                          break;
          /* 5Gbps */
          case 0x0008:    out_be32 ((u32 *) SPEED, NBASET_SPD_5G);
                          lp->cur_speed = 5000;
                          dev_info (&ndev->dev, "speed set to 5000BASE-T/FD\n");
                          break;
          /* 10Gbps */
          default:        out_be32 ((u32 *) SPEED, NBASET_SPD_10G);
                          lp->cur_speed = 10000;
                          dev_info (&ndev->dev, "speed set to 10000BASE-T/FD\n");
                          break;
        }
        break;
      }
      
      lp->phy_type = PHY_NBASET_MRVL;
      rxtx = MDIO_RXTX_ENABLE;
#endif    
    } else {      
    
      // Xilinx 10/25-Gigabit Ethernet PCS/PMA IP core
       lp->phy_type = PHY_25G_PCS_PMA;
      // 10/25GBASE-SR, full-duplex
      rxtx = MDIO_RXTX_ENABLE;

      dev_info (&ndev->dev, "PHY: Xilinx 10/25-Gigabit Ethernet PCS/PMA IP core\n");

      if (in_be32 ((u32 *)(lp->phy_base_addr + 0x0498)) & 0x00000001) {
        lp->cur_speed = 10000;
        dev_info (&ndev->dev, "speed set to 10000BASE-T/FD\n");
      } else {
        lp->cur_speed = 25000;
        dev_info (&ndev->dev, "speed set to 25000BASE-T/FD\n");
      }
    }
  }
  
  if((lp->phy_type == PHY_MARVELL) || (lp->phy_type == PHY_BROADCOM)) {
    out_be32 ((u32 *) EMMC, cfg);
  }
    
  /* Enable jumbo frames for Tx and Rx */
  out_be32 ((u32 *) TC, rxtx | MDIO_RXTX_JUMBO);
  out_be32 ((u32 *) RCW1, rxtx | MDIO_RXTX_JUMBO);
}

/*
 * Perform any necessary special phy setup. In the gmii case, nothing needs to
 * be done.
 */
static void phy_setup (struct net_local *lp, struct net_device *dev)
{
  u32 reg;
  unsigned retries = 10;

  /* non-blocking PHY access enabled (if supported) */
  out_be32 ((u32 *) MDIO_ACC, MDIO_ACC_ENA_NBLOCK);

  /* wait for end of PHY reset */
  do {
    reg = in_be32 ((u32 *) GCSR);
  } while (reg & GCSR_RST_PHY);

  /*
   * Setup timers and clock generators
   *   - timeout counter clock period 1 ms
   *   - PHY MDC frequency (max 2.5 MHz according to IEEE Std 802.3-2002,
   *     BCM5461A supports 12.5 MHz)
   */
  out_be32 ((u32 *) CLK_FREQ, lp->coreclk);
  out_be32 ((u32 *) TOCNT_DIV, (lp->coreclk / 1000) - 1);
  out_be32 ((u32 *) MC, MDIO_ENABLE_MASK | MDIO_CLOCK_DIV);

  /* wait for link up */
  while (retries-- && ((s2imac_phy_read (lp, 1) & 0x24) != 0x24)) ;
}

/*
 * The PHY registers read here should be standard registers in all PHY chips
 */
static int get_phy_status (struct net_device *ndev, int *linkup)
{
  struct net_local *lp = (struct net_local *)netdev_priv (ndev);
  u16 reg;

  if((lp->phy_type == PHY_MARVELL) || (lp->phy_type == PHY_BROADCOM)) {
    reg = s2imac_phy_read (lp, MII_BMSR);
    *linkup = (reg & BMSR_LSTATUS) != 0;
  } else if(lp->phy_type == PHY_25G_PCS_PMA) {
    // 10GBASE-R Xilinx 10/25Gbps PCS/PMA IP core
    *linkup = in_be32 ((u32 *)(lp->phy_base_addr + 0x040C)) & 0x00000001;
  } else {
    if((s2imac_mdio_read (lp, 0x03, 0x8008) & 0x0400) == 0)
      *linkup = 0;
    else
      *linkup = 1;
  }
//BMSR_ANEGCOMPLETE
  return 0;
}

/*
 * This routine is used for two purposes. The first is to keep s2imac
 * EMAC's duplex setting in sync with the PHY's. The second is to keep
 * the system apprised of the state of the link. Note that this driver
 * does not configure the PHY. Either the PHY should be configured for
 * auto-negotiation or it should be handled by something like mii-tool.
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)) 
static void poll_gmii (unsigned long data)
#else
static void poll_gmii (struct timer_list *data)
#endif
{
  struct net_device *ndev;
  struct net_local *lp;
  int phy_carrier;
  int netif_carrier;
  u32 reg;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)) 
  ndev = (struct net_device *)data;
  lp = (struct net_local *)netdev_priv (ndev);
#else  
  lp = from_timer(lp, data, phy_timer);
  ndev = lp->ndev;
#endif  

  /* first, find out what's going on with the PHY */
  if (get_phy_status (ndev, &phy_carrier)) {
    dev_err (&ndev->dev, "terminating link monitoring\n");
    return;
  }

  netif_carrier = netif_carrier_ok (ndev) != 0;

 
  if (phy_carrier != netif_carrier) {
    if (phy_carrier) {

      /* Reset PHY/MAC
      *  (workaround to avoid problems with out of DCM
      *  specification clocks in the 10 Mbps mode)
      */
      reg = in_be32 ((u32 *) GCSR);
      reg |= GCSR_RST_PHY;
      out_be32 ((u32 *) GCSR, reg);

      /* wait for end of PHY reset */
      do {
        reg = in_be32 ((u32 *) GCSR);
      } while (reg & GCSR_RST_PHY);

      set_mac_speed (lp);

      /* enable receive and transmit clients */
      reg = in_be32 ((u32 *) GCSR);
      reg |= (GCSR_CONNECTED | GCSR_LITTLE_ENDIAN);
      out_be32 ((u32 *) GCSR, reg);

			dev_info (&ndev->dev, "PHY link carrier restored\n");
      netif_carrier_on (ndev);

    } else {

      /* Periodically restart autonegotiation */
      if((lp->phy_type == PHY_MARVELL) || (lp->phy_type == PHY_BROADCOM)) {
        if (s2imac_phy_read(lp, 0x01) & 0x0020)
          s2imac_phy_write(lp, 0x00, s2imac_phy_read(lp, 0x00) | 0x0200);
      }

      /* disable receive and transmit clients */
      reg = in_be32 ((u32 *) GCSR);
      reg &= ~GCSR_CONNECTED;
      out_be32 ((u32 *) GCSR, reg);

      dev_info (&ndev->dev, "PHY link carrier lost\n");
      netif_carrier_off (ndev);
    }
  }

  /* Set up the timer so we'll get called again in 2 seconds. */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)) 
  lp->phy_timer.expires = jiffies + 2 * HZ;
  add_timer (&lp->phy_timer);
#else
  mod_timer(&lp->phy_timer, jiffies + 2 * HZ);
#endif
}

/**
 * s2imac_send_data - Send an Ethernet frame
 * @lp:   Pointer to the s2imac device private data
 * @data: Pointer to the data to be sent
 * @byte_count: Total frame size, &lp->ndev->devincluding header
 *
 * This function checks if the Tx buffer of the s2imac device is free to send
 * data. If so, it fills the Tx buffer with data for transmission. Otherwise, it
 * returns an error.
 *
 * Return:  0 upon success or -1 if the buffer(s) are full.
 *
 * Note:  The maximum Tx packet size can not be more than Ethernet header
 *    (14 Bytes) + Maximum MTU (1500 bytes). This is excluding FCS.
 */
static int s2imac_send_data (struct net_local *lp, u8 * data,
           unsigned int byte_count)
{
  u16 *to_u16_ptr, val;
  u32 len, i, len_rx, *to_u32_ptr, align_buffer,j;
  u8 *to_u8_ptr, *from_u8_ptr;

#ifndef CONFIG_S2IMAC_POLLING
  if(in_be32 ((u32 *) TX_LEN))
    return -1; /* Buffer was full, return failure */
#endif

  /* If the length is too large, truncate it */
  if (byte_count > ETH_FRAME_LEN)
    byte_count = ETH_FRAME_LEN;
  
  len = ((byte_count - 2) / 4);
  to_u16_ptr = (u16 *)data;

  // first 2 bytes of buffer not used 
  val = *to_u16_ptr++;
  lp->gige_txbuf[0] = (u32) (((u32) val << 16) | (u32) 0x0000);

  to_u32_ptr = (u32 *)to_u16_ptr;
	
  len_rx = 2;
  for (i = 1; i <= len; i++) {
    lp->gige_txbuf[i] = *to_u32_ptr++;
    len_rx += 4;
  }

  // if there are data available, send it 
  if(len_rx != byte_count)  {
      len = byte_count - len_rx;
      align_buffer = 0;
      to_u8_ptr = (u8 *)&align_buffer;
      from_u8_ptr = (u8 *)to_u32_ptr;
      for (j = 0; j < len; j++) 
        *to_u8_ptr++ = *from_u8_ptr++;
      lp->gige_txbuf[i] = align_buffer;
  }
  
  /* send reply */
  while (in_be32 ((u32 *) TX_LEN)) { 
  };
  out_be32 ((u32 *) TX_LEN, byte_count);

  //dev_info (&lp->ndev->dev, "send: %d\n",in_be32 ((u32 *) TX_LEN));

  return 0;
}

#ifndef CONFIG_S2IMAC_POLLING
/**********************/
/* Interrupt Handlers */
/**********************/

/**
 * s2imac_tx_handler - Interrupt handler for frames sent
 * @dev:  Pointer to the network device
 *
 * This function updates the number of packets transmitted and handles the
 * deferred skb, if there is one.
 */
static void s2imac_tx_handler(struct net_device *dev)
{
  struct net_local *lp = (struct net_local *) netdev_priv(dev);
      
  if (lp->deferred_skb) {
    dev->stats.tx_packets++;
    if (s2imac_send_data(lp,
          (u8 *) lp->deferred_skb->data,
          lp->deferred_skb->len) == 0) {
      dev->stats.tx_bytes += lp->deferred_skb->len;
      dev_kfree_skb_irq(lp->deferred_skb);
      lp->deferred_skb = NULL;
#ifdef HAVE_TRANS_START_HELPER
      netif_trans_update(dev);
#else
      dev->trans_start = jiffies;
#endif
      netif_wake_queue(dev);
    }
  }
  /* clear tx interrupt */
  out_be32 ((u32 *) INT_REQ,  INT_ENABLE_TX);
}

/**
 * s2imac_rx_handler- Interrupt handler for frames received
 * @dev:  Pointer to the network device
 *
 * This function allocates memory for a socket buffer, fills it with data
 * received and hands it over to the TCP/IP stack.
 */
static void s2imac_rx_handler(struct net_device *dev)
{
  struct net_local *lp = (struct net_local *) netdev_priv(dev);
  struct sk_buff *skb;
  unsigned int align;
  u32 len, l, i, align_buffer,len_rx;
  u8 *to_u8_ptr, *from_u8_ptr;
  u32 *to_u32_ptr;

  len = in_be32 ((u32 *) RX_LEN);
	
//  dev_info (&dev->dev, "[INFO] - IRQ IN: %d\n", len);

  if (len) {
    skb = dev_alloc_skb (len + ALIGNMENT);
    if (!skb) {
      dev->stats.rx_dropped++;
      dev_err (&dev->dev,
         "Could not allocate receive buffer\n");
      return;
    }

    /*
     * A new skb should have the data halfword aligned, but this code is
     * here just in case that isn't true. Calculate how many
     * bytes we should reserve to get the data to start on a word
     * boundary
     */
    align = BUFFER_ALIGN (skb->data);
    if (align)
      skb_reserve (skb, align);

    skb_reserve (skb, 2);

    l = (len / 4);

    to_u32_ptr = (u32 *)skb->data;
    len_rx = 0;
    for (i = 0; i < l; i++) 
    {
      align_buffer = ((lp->gige_rxbuf[i] >> 16) | (lp->gige_rxbuf[i + 1] << 16));
      *to_u32_ptr++ = align_buffer;
      len_rx += 4;
    }

    if(len != len_rx)
    {
      l = len - len_rx;
      align_buffer = ((lp->gige_rxbuf[i] >> 16) | (lp->gige_rxbuf[i + 1] << 16));
      to_u8_ptr = (u8 *)to_u32_ptr;
      from_u8_ptr = (u8 *)&align_buffer;
      for (i = 0; i < l; i++) 
        *to_u8_ptr++ = *from_u8_ptr++;
    }

    skb_put (skb, len); /* Tell the skb how much data we got */
    skb->dev = dev; /* Fill out required meta-data */

    skb->protocol = eth_type_trans (skb, dev);
    skb->ip_summed = CHECKSUM_NONE;

    dev->stats.rx_packets++;
    dev->stats.rx_bytes += len;

    if (!skb_defer_rx_timestamp(skb))
      netif_rx (skb); /* Send the packet upstream */
  }
  
  /* clear rx interrupt */
  out_be32 ((u32 *) INT_REQ,  INT_ENABLE_RX);
  out_be32 ((u32 *) RX_LEN, 0);
}

/**
 * s2imac_interrupt - Interrupt handler for this driver
 * @irq:  Irq of the s2imac device
 * @dev_id: Void pointer to the network device instance used as callback
 *    reference
 *
 * This function handles the Tx and Rx interrupts of the s2imac device.
 */
static irqreturn_t s2imac_interrupt(int irq, void *dev_id)
{
  struct net_device *dev = dev_id;
  struct net_local *lp = (struct net_local *) netdev_priv(dev);
  u32 reg_data;
  unsigned long flags;

  spin_lock_irqsave (&lp->reset_lock, flags);

  /* read the irq status */
  reg_data = in_be32((u32 *) INT_REQ);

  /* check if rx interrupt */
  if((reg_data & INT_ENABLE_RX) == INT_ENABLE_RX)
    s2imac_rx_handler(dev);

  /* check if tx interrupt */
  if((reg_data & INT_ENABLE_TX) == INT_ENABLE_TX)
    s2imac_tx_handler(dev);

  spin_unlock_irqrestore (&lp->reset_lock, flags);

  return IRQ_HANDLED;
}

#else

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)) 
static void s2imac_irq_timer (unsigned long data)
#else
static void s2imac_irq_timer (struct timer_list *data)
#endif
{
  struct net_device *ndev;
  struct net_local *lp;
//  struct net_device *ndev = (struct net_device *)data;
//  struct net_local *lp = (struct net_local *)netdev_priv (ndev);
  struct sk_buff *skb;
  unsigned int align;
  u32 len, l, i, align_buffer,len_rx;
  u16 *to_u16_ptr;
  u16 *from_u16_ptr;
  u32 *to_u32_ptr;
  u16 w1, w2;
  u8 *to_u8_ptr, *from_u8_ptr;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)) 
  ndev = (struct net_device *)data;
  lp = (struct net_local *)netdev_priv (ndev);
#else  
  lp = from_timer(lp, data, irq_timer);
  ndev = lp->ndev;
#endif  
  
  
  len = in_be32 ((u32 *) RX_LEN);
  if (len) {
    skb = dev_alloc_skb (len + ALIGNMENT);
    if (!skb) {
      ndev->stats.rx_dropped++;
      dev_err (&ndev->dev,
        "Could not allocate receive buffer\n");
      return;
    }

    /*
     * A new skb should have the data halfword aligned, but this code is
     * here just in case that isn't true. Calculate how many
     * bytes we should reserve to get the data to start on a word
     * boundary
     */
    align = BUFFER_ALIGN (skb->data);
    if (align)
      skb_reserve (skb, align);

    skb_reserve (skb, 2);

    l = (len / 4);

    to_u32_ptr = (u32 *)skb->data;
    len_rx = 0;
    for (i = 0; i < l; i++) 
    {
      align_buffer = ((lp->gige_rxbuf[i] >> 16) | (lp->gige_rxbuf[i + 1] << 16));
      *to_u32_ptr++ = align_buffer;
      len_rx += 4;
    }

    if(len != len_rx)
    {
      l = len - len_rx;
      align_buffer = ((lp->gige_rxbuf[i] >> 16) | (lp->gige_rxbuf[i + 1] << 16));
      to_u8_ptr = (u8 *)to_u32_ptr;
      from_u8_ptr = (u8 *)&align_buffer;
      for (i = 0; i < l; i++) 
        *to_u8_ptr++ = *from_u8_ptr++;
    }

    skb_put (skb, len); /* Tell the skb how much data we got */
    skb->dev = ndev;  /* Fill out required meta-data */

    skb->protocol = eth_type_trans (skb, ndev);
    skb->ip_summed = CHECKSUM_NONE;

    ndev->stats.rx_packets++;
    ndev->stats.rx_bytes += len;

    if (!skb_defer_rx_timestamp(skb))
      netif_rx (skb);   /* Send the packet upstream */

    out_be32 ((u32 *) RX_LEN, 0);
  }

  mod_timer (&lp->irq_timer, jiffies);
}
#endif

/**
 * s2imac_update_address - Update the MAC address in the device
 * @lp:   Pointer to the s2imac device private data
 * @address_ptr:Pointer to the MAC address (MAC address is a 48-bit value)
 *
 * Tx must be idle and Rx should be idle for deterministic results.
 * It is recommended that this function should be called after the
 * initialization and before transmission of any packets from the device.
 * The MAC address can be programmed using any of the two transmit
 * buffers (if configured).
 */
static void s2imac_update_address (struct net_local *lp, u8 * address_ptr)
{
  u32 val;
  u32 mac_l, mac_h;

  /* set up unicast MAC address filter */
  val = ((address_ptr[3] << 24) | (address_ptr[2] << 16) |
         (address_ptr[1] << 8) | (address_ptr[0]));
  out_be32 ((u32 *) UAW0, val);
  val = (address_ptr[5] << 8) | address_ptr[4];
  out_be32 ((u32 *) UAW1, val);

  mac_h = (address_ptr[0] << 8) | address_ptr[1];
  mac_l = ((address_ptr[2] << 24) | (address_ptr[3] << 16) |
     (address_ptr[4] << 8) | (address_ptr[5]));
  out_be32 ((u32 *) MAC_HIGH, mac_h); /* set gige_mac_h register */
  out_be32 ((u32 *) MAC_LOW, mac_l);  /* set gige_mac_l register */
  out_be32 ((u32 *) MAC_RX_HIGH, mac_h);  /* set gige_rx_mac_h register */
  out_be32 ((u32 *) MAC_RX_LOW, mac_l); /* set gige_rx_mac_l register */
}

/**
 * s2imac_set_mac_address - Set the MAC address for this device
 * @dev:  Pointer to the network device instance
 * @addr: Void pointer to the sockaddr structure
 *
 * This function copies the HW address from the sockaddr strucutre to the
 * net_device structure and updates the address in HW.
 *
 * Return:  Error if the net device is busy or 0 if the addr is set
 *    successfully
 */
static int s2imac_set_mac_address (struct net_device *ndev, void *address)
{
  struct net_local *lp = (struct net_local *)netdev_priv (ndev);
  struct sockaddr *addr = address;

  if (netif_running (ndev))
    return -EBUSY;

  memcpy (ndev->dev_addr, addr->sa_data, ndev->addr_len);
  s2imac_update_address (lp, ndev->dev_addr);
  return 0;
}

/**
 * s2imac_tx_timeout - Callback for Tx Timeout
 * @dev:  Pointer to the network device
 *
 * This function is called when Tx time out occurs for s2imac device.
 */
static void s2imac_tx_timeout (struct net_device *ndev, unsigned int val)
{
  struct net_local *lp = (struct net_local *)netdev_priv (ndev);
  unsigned long flags;
#ifdef HAVE_TRANS_START_HELPER
  struct netdev_queue *txq = netdev_get_tx_queue(ndev, 0);
#endif
  
  dev_err (&ndev->dev, "Exceeded transmit timeout of %lu ms\n",
     TX_TIMEOUT * 1000UL / HZ);

  ndev->stats.tx_errors++;

  /* Reset the device */
  spin_lock_irqsave (&lp->reset_lock, flags);

  /* Shouldn't really be necessary, but shouldn't hurt */
  netif_stop_queue (ndev);

#ifndef CONFIG_S2IMAC_POLLING
  s2imac_disable_interrupts(lp);
  s2imac_enable_interrupts(lp);
#endif  

  //out_be32 ((u32 *) TX_LEN, byte_count);

  if (lp->deferred_skb) {
    dev_kfree_skb (lp->deferred_skb);
    lp->deferred_skb = NULL;
    ndev->stats.tx_errors++;
  }

  /* To exclude tx timeout */
#ifdef HAVE_TRANS_START_HELPER
  txq->trans_start = 0xffffffff - TX_TIMEOUT - TX_TIMEOUT;
#else
  ndev->trans_start = 0xffffffff - TX_TIMEOUT - TX_TIMEOUT;
#endif 
  /* We're all ready to go. Start the queue */
  netif_wake_queue (ndev);
  spin_unlock_irqrestore (&lp->reset_lock, flags);
}

/**
 * s2imac_open - Open the network device
 * @dev:  Pointer to the network device
 *
 * This function sets the MAC address, requests an IRQ and enables interrupts
 * for the s2imac device and starts the Tx queue.
 * It also connects to the phy device, if MDIO is included in s2imac device.
 */
static int s2imac_open (struct net_device *ndev)
{
  struct net_local *lp = (struct net_local *)netdev_priv (ndev);
  int link;
  u32 reg;
#ifndef CONFIG_S2IMAC_POLLING
  int retval;
  s2imac_disable_interrupts(lp);
#endif

  /* Just to be safe, stop the device first */
  netif_stop_queue (ndev);

  /* Set the MAC address each time opened */
  s2imac_update_address (lp, ndev->dev_addr);

  /* transmit and receive packet buffers */
  lp->gige_txbuf = (u32 *)TXBUF;
  lp->gige_rxbuf = (u32 *)RXBUF;

  /* setup phy */
  set_mac_speed (lp);

  /* if link if up enable connected bit */
  get_phy_status (ndev, &link);

  /* first read from PHY_25G_PCS_PMA always 0, need to fix */
  if(lp->phy_type == PHY_25G_PCS_PMA)
    link = 1;

  if(link)
  {
    /* enable receive and transmit clients */
    reg = in_be32 ((u32 *) GCSR);
    reg |= (GCSR_CONNECTED | GCSR_LITTLE_ENDIAN);
    out_be32 ((u32 *) GCSR, reg);
  }  
  
#ifndef CONFIG_S2IMAC_POLLING
  /* grab the IRQ */
  retval = request_irq(ndev->irq, s2imac_interrupt, 0, ndev->name, ndev);
  if (retval) {
    dev_err(&lp->ndev->dev, "Could not allocate interrupt %d\n",
      ndev->irq);
    return retval;
  }
  
  /* Enable Interrupts */
  s2imac_enable_interrupts(lp);
#else
  mod_timer (&lp->irq_timer, jiffies);
#endif

  /* We're ready to go */
  netif_start_queue (ndev);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)) 
  /* Set up the PHY monitoring timer. */
  lp->phy_timer.expires = jiffies + 2 * HZ;
  lp->phy_timer.data = (unsigned long)ndev;
  lp->phy_timer.function = &poll_gmii;
  init_timer (&lp->phy_timer);
  add_timer (&lp->phy_timer);
#else
   timer_setup(&lp->phy_timer, &poll_gmii, 0);
   mod_timer(&lp->phy_timer, jiffies + 2 * HZ);
#endif
  return 0;
}

/**
 * s2imac_close - Close the network device
 * @dev:  Pointer to the network device
 *
 * This function stops the Tx queue, disables interrupts and frees the IRQ for
 * the s2imac device.
 * It also disconnects the phy device associated with the s2imac device.
 */
static int s2imac_close (struct net_device *ndev)
{
  struct net_local *lp = (struct net_local *)netdev_priv (ndev);
  u32 reg;

  netif_stop_queue (ndev);

#ifndef CONFIG_S2IMAC_POLLING
  s2imac_disable_interrupts(lp);
  free_irq(ndev->irq, ndev);
#endif
	
  /* Shut down the PHY monitoring timer. */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)) 
  del_timer_sync (&lp->phy_timer);
#else
  del_timer(&lp->phy_timer);
#endif  
  /* disable receive and transmit clients */
  reg = in_be32 ((u32 *) GCSR);
  reg &= ~GCSR_CONNECTED;
  out_be32 ((u32 *) GCSR, reg);

  return 0;
}

/**
 * s2imac_get_stats - Get the stats for the net_device
 * @dev:  Pointer to the network device
 *
 * This function returns the address of the 'net_device_stats' structure for the
 * given network device. This structure holds usage statistics for the network
 * device.
 *
 * Return:  Pointer to the net_device_stats structure.
 */
static struct net_device_stats *s2imac_get_stats (struct net_device *ndev)
{
  return &ndev->stats;
}

/**
 * s2imac_send - Transmit a frame
 * @orig_skb: Pointer to the socket buffer to be transmitted
 * @dev:  Pointer to the network device
 *
 * This function checks if the Tx buffer of the s2imac device is free to send
 * data. If so, it fills the Tx buffer with data from socket buffer data,
 * updates the stats and frees the socket buffer. The Tx completion is signaled
 * by an interrupt. If the Tx buffer isn't free, then the socket buffer is
 * deferred and the Tx queue is stopped so that the deferred socket buffer can
 * be transmitted when the s2imac device is free to transmit data.
 *
 * Return:  0, always.
 */
static int s2imac_send (struct sk_buff *orig_skb, struct net_device *ndev)
{
  struct net_local *lp = (struct net_local *)netdev_priv (ndev);
  struct sk_buff *new_skb;
  unsigned int len;
  unsigned long flags;

  len = orig_skb->len;

  new_skb = orig_skb;

  spin_lock_irqsave (&lp->reset_lock, flags);

  if (s2imac_send_data (lp, (u8 *) new_skb->data, len) != 0) {
    /*
     * If the s2imac Tx buffer is busy, stop the Tx queue and
     * defer the skb for transmission at a later point when the
     * current transmission is complete
     */
    netif_stop_queue (ndev);
    lp->deferred_skb = new_skb;
    /* Take the time stamp now, since we can't do this in an ISR. */
    skb_tx_timestamp(new_skb);
    spin_unlock_irqrestore (&lp->reset_lock, flags);
    return 0;
  }
  spin_unlock_irqrestore (&lp->reset_lock, flags);

  skb_tx_timestamp(new_skb);
  ndev->stats.tx_packets++;
  ndev->stats.tx_bytes += len;
  dev_kfree_skb (new_skb);
#ifdef HAVE_TRANS_START_HELPER
  netif_trans_update(ndev);
#else
  ndev->trans_start = jiffies;
#endif  
  return 0;
}

/**
 * s2imac_remove_ndev - Free the network device
 * @ndev: Pointer to the network device to be freed
 *
 * This function un maps the IO region of the s2imac device and frees the net
 * device.
 */
static void s2imac_remove_ndev (struct net_device *ndev)
{
  if (ndev) {
    struct net_local *lp = (struct net_local *)netdev_priv (ndev);

    if (lp->base_addr[S2IMAC_GIGEMAP])
    {
      iounmap ((void __iomem __force *)(lp->base_addr[S2IMAC_GIGEMAP]));
      lp->base_addr[S2IMAC_GIGEMAP] = NULL;
    } 
    if (lp->base_addr[S2IMAC_VIDEOMAP])
    {
      iounmap ((void __iomem __force *)(lp->base_addr[S2IMAC_VIDEOMAP]));
      lp->base_addr[S2IMAC_VIDEOMAP] = NULL;
    }
      
    if (lp->base_addr[S2IMAC_FRAMEBUFFERMAP])
    {
      iounmap ((void __iomem __force *)(lp->base_addr[S2IMAC_FRAMEBUFFERMAP]));
      lp->base_addr[S2IMAC_FRAMEBUFFERMAP] = NULL;
    }
      
    free_netdev (ndev);
  }
}

#ifndef ETHTOOL_GLINKSETTINGS
static int s2imac_ethtool_get_settings (struct net_device *ndev,
          struct ethtool_cmd *ecmd)
{
  struct net_local *lp = (struct net_local *)netdev_priv (ndev);
  u16 gmii_status = 0;

  memset (ecmd, 0, sizeof (struct ethtool_cmd));

//  if((lp->phy_type == PHY_MARVELL) || (lp->phy_type == PHY_BROADCOM)) 
    gmii_status = s2imac_phy_read (lp, MII_BMSR);

  ecmd->duplex = DUPLEX_FULL;

  ecmd->supported |= SUPPORTED_MII;

  ecmd->port = PORT_MII;

  ecmd->speed = lp->cur_speed;

  if (gmii_status & BMSR_ANEGCAPABLE) {
    ecmd->supported |= SUPPORTED_Autoneg;
  }
  if (gmii_status & BMSR_ANEGCOMPLETE) {
    ecmd->autoneg = AUTONEG_ENABLE;
    ecmd->advertising |= ADVERTISED_Autoneg;
  } else {
    ecmd->autoneg = AUTONEG_DISABLE;
  }
  ecmd->phy_address = lp->phy_addr;
  ecmd->transceiver = XCVR_INTERNAL;

  ecmd->supported |= SUPPORTED_10baseT_Full | SUPPORTED_100baseT_Full | SUPPORTED_1000baseT_Full;
  return 0;
}

static int s2imac_ethtool_set_settings (struct net_device *ndev,
          struct ethtool_cmd *ecmd)
{
  struct net_local *lp = (struct net_local *)netdev_priv (ndev);

  if ((ecmd->duplex != DUPLEX_FULL) ||
      (ecmd->transceiver != XCVR_INTERNAL) ||
      (ecmd->phy_address && (ecmd->phy_address != lp->phy_addr))) {
    return -EOPNOTSUPP;
  }

  if ((ecmd->speed != 10000) && (ecmd->speed != 1000) && (ecmd->speed != 100) &&
      (ecmd->speed != 10)) {
    dev_err (&ndev->dev, "speed not supported: %d\n", ecmd->speed);
    return -EOPNOTSUPP;
  }

  if (ecmd->speed != lp->cur_speed) {
    lp->cur_speed = ecmd->speed;
    dev_err (&ndev->dev, "can not change speed to: %d\n", ecmd->speed);
  }
  return 0;
}
#else
static int s2imac_get_link_ksettings(struct net_device *dev,
				       struct ethtool_link_ksettings *cmd)
{
	struct net_local *lp = (struct net_local *)netdev_priv (dev);
  u16 gmii_status = 0;
	u32 supported = 0;

	if (!lp->init_phy)
		return -ENODEV;

  memset (cmd, 0, sizeof (struct ethtool_link_ksettings));

  gmii_status = s2imac_phy_read (lp, MII_BMSR);

  cmd->base.speed = lp->cur_speed;
  cmd->base.duplex = DUPLEX_FULL;
  cmd->base.port = PORT_MII;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 212)
	cmd->base.transceiver = XCVR_INTERNAL;
#endif  
  cmd->base.phy_address = lp->phy_addr;

  if (gmii_status & BMSR_ANEGCOMPLETE) {
    cmd->base.autoneg = AUTONEG_ENABLE;
  } else {
    cmd->base.autoneg = AUTONEG_DISABLE;
  }
  
  supported |= SUPPORTED_MII;
  if (gmii_status & BMSR_ANEGCAPABLE) {
    supported |= SUPPORTED_Autoneg;
  }
  supported |= SUPPORTED_10baseT_Full | SUPPORTED_100baseT_Full | SUPPORTED_1000baseT_Full;
	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
						supported);
	return 0;
}
static int s2imac_set_link_ksettings(struct net_device *dev,
				       const struct ethtool_link_ksettings *cmd)
{
	struct net_local *lp = (struct net_local *)netdev_priv (dev);

	if (!lp->init_phy)
		return -ENODEV;
  
	if (cmd->base.phy_address != lp->phy_addr)
		return -EINVAL;
  
  if ((cmd->base.duplex != DUPLEX_FULL) ||
      (cmd->base.transceiver != XCVR_INTERNAL) ||
      (cmd->base.phy_address && (cmd->base.phy_address != lp->phy_addr))) {
    return -EOPNOTSUPP;
  }

  if ((cmd->base.speed != 10000) && (cmd->base.speed != 1000) && (cmd->base.speed != 100) &&
      (cmd->base.speed != 10)) {
    dev_err (&dev->dev, "speed not supported: %d\n", cmd->base.speed);
    return -EOPNOTSUPP;
  }

  if (cmd->base.speed != lp->cur_speed) {
    lp->cur_speed = cmd->base.speed;
    dev_err (&dev->dev, "can not change speed to: %d\n", cmd->base.speed);
  }
    
	return 0;
}
#endif

static struct net_device_ops s2imac_netdev_ops;

/* From include/linux/ethtool.h */
static struct ethtool_ops ethtool_ops = {
#ifndef ETHTOOL_GLINKSETTINGS
  .get_settings = s2imac_ethtool_get_settings,
  .set_settings = s2imac_ethtool_set_settings,
#else
	.get_link_ksettings	= s2imac_get_link_ksettings,
	.set_link_ksettings	= s2imac_set_link_ksettings,
#endif  

  .get_link = ethtool_op_get_link
};

/**
 * s2imac_of_probe - Probe method for the s2imac device.
 * @ofdev:  Pointer to OF device structure
 * @match:  Pointer to the structure used for matching a device
 *
 * This function probes for the s2imac device in the device tree.
 * It initializes the driver data structure and the hardware, sets the MAC
 * address and registers the network device.
 *
 * Return:  0, if the driver is bound to the s2imac device, or
 *    a negative error if there is failure.
 */
static int s2imac_of_probe(struct platform_device *ofdev)
//static int __devinit s2imac_of_probe (struct of_device *ofdev)
//              const struct of_device_id *match)
{
#ifndef CONFIG_S2IMAC_POLLING
#ifdef CONFIG_MICROBLAZE
  struct resource r_irq; /* Interrupt resources */
#else  
  struct resource *r_irq1; /* Interrupt resources */
#endif
#endif
  struct resource r_mem;  /* IO mem resources */
  struct net_device *ndev;
  struct net_local *lp;
//  struct device_node *np = ofdev->node;
  struct device_node *np = ofdev->dev.of_node;
  struct device *dev = &ofdev->dev;
  const void *mac_address;
  const unsigned int *clk;
  int rc = 0;
  u32 mac_l, mac_h;
  int res_cnt = 0;
  const unsigned int *cfg_int;
  const unsigned int *phy_base_addr;
  const unsigned int *phy_base_addr_len;
  unsigned int phy_base_addr_len_val;
	
#ifdef CONFIG_MICROBLAZE
  struct device_node *cpu;
#endif

  dev_info (dev, "probing \'%s\'\n", np->name);

  /* Create an ethernet device instance */
  ndev = alloc_etherdev (sizeof (struct net_local));
  if (!ndev) {
    dev_err (dev, "Could not allocate network device\n");
    return -ENOMEM;
  }

  dev_set_drvdata (dev, ndev);
  SET_NETDEV_DEV (ndev, dev);

  lp = netdev_priv (ndev);
  lp->ndev = ndev;
  lp->init_phy = 0;
  
  lp->base_addr[S2IMAC_GIGEMAP] = NULL;
  lp->base_addr[S2IMAC_VIDEOMAP] = NULL;
  lp->base_addr[S2IMAC_FRAMEBUFFERMAP] = NULL;
  
  ndev->mem_start = 0;
  ndev->mem_end = 0;
  
  lp->video_start = 0;
  lp->video_end = 0;

  lp->framebuffer_start = 0;
  lp->framebuffer_end = 0;

  lp->phy_base_addr = NULL;
  lp->phy_base_start = 0;
  lp->phy_base_end = 0;
  
  /* Get clock configuration for the device */
  clk = of_get_property (np, "clock-frequency", NULL);
  if (clk == NULL || (!(u32) be32_to_cpup(clk))) {
    dev_warn (dev, "no clock-frequency property set\n");
    rc = -ENODEV;

#ifndef CONFIG_MICROBLAZE

      goto error;

#else /* is CONFIG_MICROBLAZE */
    cpu = of_find_node_by_type (NULL, "cpu");
    if (!cpu) {
      dev_err (dev, "you don't have a cpu?\n");
      goto error;
    }

    clk = of_get_property (cpu, "clock-frequency", NULL);
    if (!clk) {
      dev_err (dev, "no CPU clock-frequency property set\n");
      goto error;
    }
    dev_warn (dev, "use CPU clock-frequency\n");
    rc = 0;
#endif
  }
  lp->coreclk = be32_to_cpup(clk);
   
  dev_info (dev, "core clock is %d Hz\n", lp->coreclk);
  
  cfg_int = of_get_property (np, "config-interface", NULL);
  if (cfg_int == NULL) {
    dev_warn (dev, "no config-interface property set, set to DEFAULT\n");
		lp->config_interface = DEFAULT;
	}
	else {
		if(strcmp( (char *)cfg_int, "DEFAULT") == 0)
		  lp->config_interface = DEFAULT;
		else if(strcmp( (char *)cfg_int, "XAUI") == 0)
		  lp->config_interface = XAUI;
		else if(strcmp( (char *)cfg_int, "RXAUI") == 0)
		  lp->config_interface = RXAUI;
		else if(strcmp( (char *)cfg_int, "USXGMII") == 0)
		  lp->config_interface = USXGMII;
		else
		  lp->config_interface = DEFAULT;
    dev_info (dev, "config-interface is %s [%d]\n", (char *)cfg_int, lp->config_interface);
	}
	
  /* Get iospace for the gige register */
  if(platform_get_resource_byname(ofdev, IORESOURCE_MEM, "core"))
  {
    rc = of_address_to_resource (ofdev->dev.of_node, S2IMAC_GIGEMAP, &r_mem);
    if (rc) {
      dev_err (dev, "Invalid address for core register\n");
      goto error;
    }

    if (!request_mem_region (r_mem.start,
          r_mem.end - r_mem.start + 1,
          DRIVER_NAME)) {
      dev_err (dev, "Couldn't lock core memory region at %llX\n",
        r_mem.start);
      rc = -EBUSY;
      goto error;
    }

    ndev->mem_start = r_mem.start;
    ndev->mem_end = r_mem.end;

    /* Get the virtual base address for the gige register */
    lp->base_addr[S2IMAC_GIGEMAP] =
        ioremap (ndev->mem_start, ndev->mem_end - ndev->mem_start + 1);

    if (NULL == lp->base_addr[S2IMAC_GIGEMAP]) {
      dev_err (dev, "Could not allocate iomem for core\n");
      rc = -EIO;
      goto error;
    }
        
    dev_info (dev, "core found at 0x%lX mapped to 0x%llX\n",
      ndev->mem_start,
      (uint64_t)lp->base_addr[S2IMAC_GIGEMAP]);
  }
  
  /* Get iospace for the video register */
  if(platform_get_resource_byname(ofdev, IORESOURCE_MEM, "video"))
  {
    rc = of_address_to_resource (ofdev->dev.of_node, S2IMAC_VIDEOMAP, &r_mem);
    if (rc) {
      dev_err (dev, "Invalid address for video register\n");
      goto error;
    }

    if (!request_mem_region (r_mem.start,
          r_mem.end - r_mem.start + 1,
          DRIVER_NAME)) {
      dev_err (dev, "Couldn't lock video memory region at %p\n",
        (void *)r_mem.start);
      rc = -EBUSY;
      goto error;
    }
    lp->video_start = r_mem.start;
    lp->video_end = r_mem.end;
    
    /* Get the virtual base address for the video register */
    lp->base_addr[S2IMAC_VIDEOMAP] =
        ioremap (lp->video_start, lp->video_end - lp->video_start + 1);

    if (NULL == lp->base_addr[S2IMAC_VIDEOMAP]) {
      dev_err (dev, "Could not allocate iomem for video\n");
      rc = -EIO;
      goto error;
    }
    
    dev_info (dev, "video found at 0x%lX mapped to 0x%llX\n",
      lp->video_start,
      (uint64_t)lp->base_addr[S2IMAC_VIDEOMAP]);
  }
  else
    res_cnt = 1;
  
#if defined(CONFIG_ARCH_ZYNQ) || defined(CONFIG_ARCH_ZYNQMP)
  /* Get iospace for the framebuffer register */
  if(platform_get_resource_byname(ofdev, IORESOURCE_MEM, "framebuffer"))
  {
    rc = of_address_to_resource (ofdev->dev.of_node, S2IMAC_FRAMEBUFFERMAP - res_cnt, &r_mem);
    if (rc) {
      dev_err (dev, "Invalid address for framebuffer register\n");
      goto error;
    }

    if (!request_mem_region (r_mem.start,
          r_mem.end - r_mem.start + 1,
          DRIVER_NAME)) {
      dev_err (dev, "Couldn't lock framebuffer memory region at %p\n",
        (void *)r_mem.start);
      rc = -EBUSY;
      goto error;
    }
    lp->framebuffer_start = r_mem.start;
    lp->framebuffer_end = r_mem.end;
    
    /* Get the virtual base address for the framebuffer register */
    lp->base_addr[S2IMAC_FRAMEBUFFERMAP] =
        ioremap (lp->framebuffer_start, lp->framebuffer_end - lp->framebuffer_start + 1);
      
    if (NULL == lp->base_addr[S2IMAC_FRAMEBUFFERMAP]) {
      dev_err (dev, "Could not allocate iomem for framebuffer\n");
      rc = -EIO;
      goto error;
    }

    dev_info (dev, "framebuffer found at 0x%lX mapped to 0x%llX\n",
      lp->framebuffer_start,
      (uint64_t)lp->base_addr[S2IMAC_FRAMEBUFFERMAP]);
  }
#endif

#ifndef CONFIG_S2IMAC_POLLING
#ifdef CONFIG_MICROBLAZE
  /* Get IRQ for the device */
  rc = of_irq_to_resource(ofdev->dev.of_node, 0, &r_irq);
  if (rc == NO_IRQ) {
    dev_err(dev, "no IRQ found\n");
    goto error;
  }
  ndev->irq = r_irq.start;
#else
  r_irq1 = platform_get_resource(ofdev, IORESOURCE_IRQ, 0);
  if (!r_irq1) {
    dev_err(dev, "no IRQ found\n");
    goto error;
  }
  ndev->irq = r_irq1->start;
#endif
  dev_info(dev,"Irq is %d\n", ndev->irq);
#endif

  spin_lock_init (&lp->reset_lock);

  /* check if mac address is already set */
  mac_h = in_be32 ((u32 *) MAC_HIGH);
  mac_l = in_be32 ((u32 *) MAC_LOW);
  
  if ((mac_h == 0) && (mac_l == 0)) {
    mac_address = of_get_mac_address (np, ndev->dev_addr);
    if (mac_address)
      /* Copy the MAC address from OF node. */
      memcpy (ndev->dev_addr, mac_address, 6);
    else
      dev_warn (dev, "No MAC address found\n");

		/* Set the MAC address in the device */
    s2imac_update_address (lp, ndev->dev_addr);
  } else {
    ndev->dev_addr[0] = (u8) (mac_h >> 8);
    ndev->dev_addr[1] = (u8) (mac_h & 0xff);
    ndev->dev_addr[2] = (u8) (mac_l >> 24);
    ndev->dev_addr[3] = (u8) (mac_l >> 16);
    ndev->dev_addr[4] = (u8) (mac_l >> 8);
    ndev->dev_addr[5] = (u8) (mac_l & 0xff);
  }

  dev_info (dev, "MAC address is now %02x:%02x:%02x:%02x:%02x:%02x\n",
      ndev->dev_addr[0], ndev->dev_addr[1], ndev->dev_addr[2],
      ndev->dev_addr[3], ndev->dev_addr[4], ndev->dev_addr[5]);
  
  /* Get phy base address length*/
  phy_base_addr_len = of_get_property (np, "phy-base-address-length", NULL);
  if (phy_base_addr_len == NULL) {
    dev_warn (dev, "no phy-address-length set, set to 0\n");
    phy_base_addr_len_val = 0;
  }
  else 
    phy_base_addr_len_val = be32_to_cpup(phy_base_addr_len);

  /* Get phy base address */
  phy_base_addr = of_get_property (np, "phy-base-address", NULL);
  if (phy_base_addr == NULL) {
    dev_warn (dev, "no phy-base-address set, set to 0\n");
    lp->phy_base_addr = 0;
  }
  else {
    r_mem.start = be32_to_cpup(phy_base_addr);
    if(r_mem.start)
    {
      r_mem.end = r_mem.start + (phy_base_addr_len_val - 1);
      
      if (!request_mem_region (r_mem.start,
            r_mem.end - r_mem.start + 1,
            DRIVER_NAME)) {
        dev_err (dev, "Couldn't lock phy base address region at %p\n",
          (void *)r_mem.start);
        rc = -EBUSY;
        goto error;
      }
      
      lp->phy_base_start = r_mem.start;
      lp->phy_base_end = r_mem.end;

      /* Get the virtual base address for the phy base address */
      lp->phy_base_addr = ioremap (lp->phy_base_start, lp->phy_base_end - lp->phy_base_start + 1);

      if (NULL == lp->phy_base_addr) {
        dev_err (dev, "Could not allocate iomem for phy base address\n");
        rc = -EIO;
        goto error;
      }
          
      dev_info (dev, "phy base address found at 0x%lX mapped to 0x%llX\n",
        lp->phy_base_start,
        (uint64_t)lp->phy_base_addr);
    }
  }
  
  /* Scan to find the PHY */
  phy_setup (lp, ndev);
  lp->phy_addr = detect_phy (lp, dev);
  if(lp->phy_addr == -1)
    goto error;

#ifdef CONFIG_S2IMAC_POLLING
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)) 
  init_timer (&lp->irq_timer);
  lp->irq_timer.function = s2imac_irq_timer;
  lp->irq_timer.data = (unsigned long)ndev;
#else
   timer_setup(&lp->irq_timer, s2imac_irq_timer, 0);
#endif
#endif

  ndev->netdev_ops = &s2imac_netdev_ops;
  ndev->flags &= ~IFF_MULTICAST;
  ndev->watchdog_timeo = TX_TIMEOUT;
  ndev->ethtool_ops = &ethtool_ops;

  /* Set ethtool IOCTL handler vectors. */
//  SET_ETHTOOL_OPS (ndev, &ethtool_ops);

  /* Finally, register the device */
  rc = register_netdev (ndev);
  if (rc) {
    dev_err (dev, "Cannot register network device, aborting\n");
    goto error;
  }
  
  return 0;

error:

  if(lp->base_addr[S2IMAC_FRAMEBUFFERMAP])
  {
    iounmap (lp->base_addr[S2IMAC_FRAMEBUFFERMAP]);
    lp->base_addr[S2IMAC_FRAMEBUFFERMAP] = NULL;
  }
  
  if(lp->base_addr[S2IMAC_VIDEOMAP])
  {
    iounmap (lp->base_addr[S2IMAC_VIDEOMAP]);
    lp->base_addr[S2IMAC_VIDEOMAP] = NULL;
  }
  
  if(lp->base_addr[S2IMAC_GIGEMAP])
  {
    iounmap (lp->base_addr[S2IMAC_GIGEMAP]);
    lp->base_addr[S2IMAC_GIGEMAP] = NULL;
  }
  
  if(lp->framebuffer_start)
  {
    release_mem_region (lp->framebuffer_start,
            lp->framebuffer_end - lp->framebuffer_start + 1);
    lp->framebuffer_start = 0;
  }
  
  if(lp->video_start)
  {
    release_mem_region (lp->video_start,
            lp->video_end - lp->video_start + 1);
    lp->video_start = 0;
  }
  
  if(ndev->mem_start)
  {
    release_mem_region (ndev->mem_start,
            ndev->mem_end - ndev->mem_start + 1);
    ndev->mem_start = 0;
  } 

  if(lp->phy_base_start)
  {
    release_mem_region (lp->phy_base_start,
            lp->phy_base_end - lp->phy_base_start + 1);
    lp->phy_base_start = 0;
  }
  
  s2imac_remove_ndev (ndev);
  
  return rc;
}

/**
 * s2imac_of_remove - Unbind the driver from the s2imac device.
 * @of_dev: Pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 *
 * Return:  0, always.
 */
//static int __devexit s2imac_of_remove (struct of_device *of_dev)
static int s2imac_of_remove (struct platform_device *of_dev)
{
  struct device *dev = &of_dev->dev;
  struct net_device *ndev = dev_get_drvdata (dev);
  struct net_local *lp = (struct net_local *)netdev_priv (ndev);

  unregister_netdev (ndev);

  s2imac_remove_ndev (ndev);

  if(lp->framebuffer_start)
  {
    release_mem_region (lp->framebuffer_start,
            lp->framebuffer_end - lp->framebuffer_start + 1);
    lp->framebuffer_start = 0;
  }
  
  if(lp->video_start)
  {
    release_mem_region (lp->video_start,
            lp->video_end - lp->video_start + 1);
    lp->video_start = 0;
  }
  
  if(ndev->mem_start)
  {
    release_mem_region (ndev->mem_start,
            ndev->mem_end - ndev->mem_start + 1);
    ndev->mem_start = 0;
  }

  if(lp->phy_base_start)
  {
    release_mem_region (lp->phy_base_start,
            lp->phy_base_end - lp->phy_base_start + 1);
    lp->phy_base_start = 0;
  }
  
  dev_set_drvdata (dev, NULL);

  return 0;
}

static int s2imac_change_mtu (struct net_device *ndev, int new_mtu)
{
  int device_enable = 0;
#ifdef CONFIG_XILINX_GIGE_VLAN
  int head_size = XTE_HDR_VLAN_SIZE;
#else
  int head_size = XTE_HDR_SIZE;
#endif
  int max_frame = new_mtu + head_size + XTE_TRL_SIZE;
  int min_frame = 1 + head_size + XTE_TRL_SIZE;

  if (max_frame < min_frame)
    return -EINVAL;

  if (max_frame > XTE_MAX_FRAME_SIZE) {
    dev_info (&ndev->dev, "Wrong MTU packet size. Use %d size\n",
      XTE_MTU);
    new_mtu = XTE_MTU;
  }

  ndev->mtu = new_mtu;  /* change mtu in net_device structure */

  /* stop driver */
  if (netif_running (ndev)) {
    device_enable = 1;
    s2imac_close (ndev);
  }

  if (device_enable)
    s2imac_open (ndev); /* open the device */
  return 0;
}

static int s2imac_ioctl (struct net_device *ndev, struct ifreq *rq, void __user *data, int cmd)
{
	struct net_local *lp = (struct net_local *)netdev_priv (ndev);
	s2igen_content_t *gen_content;
	int ret = 0;

	gen_content = kmalloc(sizeof(s2igen_content_t), GFP_KERNEL);
	if (!gen_content)
		return -ENOMEM;

	ret = copy_from_user (gen_content, data, sizeof (s2igen_content_t));
	if (ret) {
		printk(KERN_ERR "copy_from_user fails ret = %d", ret);
		printk(KERN_ERR "gen_content address = %u data = %u\n", gen_content->address, gen_content->data);
		kfree(gen_content);
	    return -EFAULT;
	}

  /* check address range 1st slot --> device base address */
  if((cmd == SIOS2IGENRD) || (cmd == SIOS2IGENWR)){
    if (gen_content->address > (ndev->mem_end - ndev->mem_start + 1))
	{
		printk(KERN_ERR "gen_content->address failed");
		kfree(gen_content);
		return -EINVAL;
	}
  }

  /* check address range 2st slot --> register base address */
  if((cmd == SIOS2IVIDEORD) || (cmd == SIOS2IVIDEOWR)){

    if (gen_content->address > (lp->video_end - lp->video_start + 1)) {
		kfree(gen_content);
		return -EINVAL;
	}
  }
  
  /* check address range 3st slot --> register base address */
  if((cmd == SIOS2IFBUFRD) || (cmd == SIOS2IFBUFWR)){

    if (gen_content->address > (lp->framebuffer_end - lp->framebuffer_start + 1)) {
		kfree(gen_content);
		return -EINVAL;
	}
  }

  switch (cmd) {
  case SIOS2IGENRD: /* core read */

    gen_content->data =
        in_be32 ((u32 *) (lp->base_addr[S2IMAC_GIGEMAP] + gen_content->address));

	ret = copy_to_user(data, gen_content, sizeof (s2igen_content_t));
    if (ret) {
		printk(KERN_ERR "copy_to_user failed ret = %d", ret);
		kfree(gen_content);
		return -EFAULT;
	}
	break;

  case SIOS2IGENWR: /* core write */

    out_be32 ((u32 *) (lp->base_addr[S2IMAC_GIGEMAP] + gen_content->address),
        gen_content->data);
	break;

  case SIOS2IVIDEORD: /* video register read */

    gen_content->data =
        in_be32 ((u32 *) (lp->base_addr[S2IMAC_VIDEOMAP] + gen_content->address));

    if (copy_to_user
        (data, gen_content, sizeof (s2igen_content_t))) {
		kfree(gen_content);
		return -EFAULT;
	}
	break;

  case SIOS2IVIDEOWR: /* video register write */

    out_be32 ((u32 *) (lp->base_addr[S2IMAC_VIDEOMAP] + gen_content->address),
        gen_content->data);
	break;

  case SIOS2IFBUFRD: /* framebuffer register read */

    gen_content->data =
        in_be32 ((u32 *) (lp->base_addr[S2IMAC_FRAMEBUFFERMAP] + gen_content->address));

    if (copy_to_user
        (data, gen_content, sizeof (s2igen_content_t))) {
		kfree(gen_content);
		return -EFAULT;
	}
	break;

  case SIOS2IFBUFWR: /* framebuffer register write */

    out_be32 ((u32 *) (lp->base_addr[S2IMAC_FRAMEBUFFERMAP] + gen_content->address),
        gen_content->data);
	break;

  default:
	kfree(gen_content);
    return -EOPNOTSUPP;
  }
	kfree(gen_content);
	return 0;
}

/**
 * s2imac_set_rx_mode - enable/disable promiscuous and multicast modes
 * @ndev: network interface device structure
 */
static void s2imac_set_rx_mode(struct net_device *ndev)
{
  struct net_local *lp = netdev_priv(ndev);
  u32 reg;

  /* get promiscuous mode of the MAC */
  reg = in_be32 ((u32 *) AFM);
  
  /* Enable multicast mode */
  if ((ndev->flags & IFF_ALLMULTI) || (ndev->flags & IFF_MULTICAST)){
    reg |= 0x80000000;
  /* Disable multicast mode */
  } else {
    reg = ~0x80000000;
  }

  /* enable/disable promiscuous mode of the MAC */
  out_be32 ((u32 *) AFM, reg);
}

static struct net_device_ops s2imac_netdev_ops = {
  .ndo_open = s2imac_open,
  .ndo_stop = s2imac_close,
  .ndo_start_xmit = s2imac_send,
  .ndo_siocdevprivate = s2imac_ioctl,
  .ndo_change_mtu = s2imac_change_mtu,
  .ndo_set_mac_address = s2imac_set_mac_address,
  .ndo_tx_timeout = s2imac_tx_timeout,
  .ndo_get_stats = s2imac_get_stats,
  .ndo_set_rx_mode  = s2imac_set_rx_mode,
};

/* Match table for OF platform binding */
//static struct of_device_id s2imac_of_match[] __devinitdata = {
static struct of_device_id s2imac_of_match[] = {
  { .compatible = "s2i,s2imac-1.00.a", },
  { .compatible = "xlnx,s2imac-epc-1.02.a", },
  { .compatible = "xlnx,s2imac-epc-2.0", },
  { .compatible = "s2i,s2imac-epc", .type = "network", },
  { /* end of list */ },
};

MODULE_DEVICE_TABLE (of, s2imac_of_match);

//static struct of_platform_driver s2imac_of_driver = {
static struct platform_driver s2imac_of_driver = {
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table = s2imac_of_match,
  },
  .probe = s2imac_of_probe,
  .remove = s2imac_of_remove,
};

/**
 * s2imac_init - Initial driver registration call
 *
 * Return:  0 upon success, or a negative error upon failure.
 */
static int __init s2imac_init (void)
{
  /* No kernel boot options used, we just need to register the driver */
//  return of_register_platform_driver (&s2imac_of_driver);

  pr_info("S2I Ethernet MAC Driver - %s\n",
    DRIVER_VERS);
  pr_info("Copyright(c) 2010 - 2021 Sensor to Image GmbH.\n");
  
  return platform_driver_register (&s2imac_of_driver);
}

/**
 * s2imac_cleanup - Driver un-registration call
 */
static void __exit s2imac_cleanup (void)
{
//  of_unregister_platform_driver (&s2imac_of_driver);
  platform_driver_unregister (&s2imac_of_driver);
}

module_init (s2imac_init);
module_exit (s2imac_cleanup);

MODULE_AUTHOR ("Sensor to Image GmbH.");
MODULE_DESCRIPTION ("S2I Ethernet MAC Driver");
MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Enclustra");
//MODULE_VERSION("1.0");
