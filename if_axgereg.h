/*-
 * Copyright (c) 2013 Kevin Lo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY Bill Paul AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL Bill Paul OR THE VOICES IN HIS HEAD
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */


#define AX88179_PHY_ID			0x03
#define AXGE_MCAST_FILTER_SIZE		8
#define AXGE_MAXGE_MCAST			64
#define AXGE_EEPROM_LEN			0x40
#define AXGE_RX_CHECKSUM			1
#define AXGE_TX_CHECKSUM			2

#define AXGE_BULKIN_24K			0x18;	/* 24k */

#define AXGE_ACCESS_MAC			0x01
#define AXGE_ACCESS_PHY			0x02
#define AXGE_ACCESS_WAKEUP		0x03
#define AXGE_ACCESS_EEPROM		0x04
#define AXGE_ACCESS_EFUSE			0x05
#define AXGE_RELOAD_EEPROM_EFUSE		0x06
#define AXGE_WRITE_EFUSE_EN		0x09
#define AXGE_WRITE_EFUSE_DIS		0x0A
#define AXGE_ACCESS_MFAB			0x10

#define PHYSICAL_LINK_STATUS		0x02
	#define	AXGE_USB_SS		0x04
	#define	AXGE_USB_HS		0x02
	#define	AXGE_USB_FS		0x01

#define GENERAL_STATUS			0x03
/* Check AX88179 version. UA1:Bit2 = 0,  UA2:Bit2 = 1 */
	#define	AXGE_SECLD		0x04



#define AXGE_SROM_ADDR			0x07
#define AXGE_SROM_CMD			0x0a
	#define EEP_RD			0x04	/* EEprom read command */
	#define EEP_WR			0x08	/* EEprom write command */
	#define EEP_BUSY		0x10	/* EEprom access module busy */


#define AXGE_SROM_DATA_LOW		0x08
#define AXGE_SROM_DATA_HIGH		0x09

#define AXGE_RX_CTL			0x0b
	#define AXGE_RX_CTL_DROPCRCERR		0x0100 /* Drop CRC error packet */
	#define AXGE_RX_CTL_IPE			0x0200 /* Enable IP header in receive buffer aligned on 32-bit aligment */
	#define AXGE_RX_CTL_TXPADCRC		0x0400 /* checksum value in rx header 3 */
	#define AXGE_RX_CTL_START			0x0080 /* Ethernet MAC start */
	#define AXGE_RX_CTL_AP			0x0020 /* Accept physcial address from Multicast array */
	#define AXGE_RX_CTL_AM			0x0010
	#define AXGE_RX_CTL_AB			0x0008 
	#define AXGE_RX_CTL_HA8B		0x0004
	#define AXGE_RX_CTL_AMALL		0x0002 /* Accetp all multicast frames */
	#define AXGE_RX_CTL_PRO			0x0001 /* Promiscuous Mode */
	#define AXGE_RX_CTL_STOP		0x0000 /* Stop MAC */

#define AXGE_NODE_ID			0x10
#define AXGE_MULTI_FILTER_ARRY		0x16

#define AXGE_MEDIUM_STATUS_MODE			0x22
	#define AXGE_MEDIUM_GIGAMODE	0x0001
	#define AXGE_MEDIUM_FULL_DUPLEX	0x0002
	#define AXGE_MEDIUM_ALWAYS_ONE	0x0004
	#define AXGE_MEDIUM_EN_125MHZ	0x0008
	#define AXGE_MEDIUM_RXFLOW_CTRLEN	0x0010
	#define AXGE_MEDIUM_TXFLOW_CTRLEN	0x0020
	#define AXGE_MEDIUM_RECEIVE_EN	0x0100
	#define AXGE_MEDIUM_PS		0x0200
	#define AXGE_MEDIUM_JUMBO_EN	0x8040

#define AXGE_MONITOR_MODE			0x24
	#define AXGE_MONITOR_MODE_RWLC		0x02
	#define AXGE_MONITOR_MODE_RWMP		0x04
	#define AXGE_MONITOR_MODE_RWWF		0x08
	#define AXGE_MONITOR_MODE_RW_FLAG		0x10
	#define AXGE_MONITOR_MODE_PMEPOL		0x20
	#define AXGE_MONITOR_MODE_PMETYPE		0x40

#define AXGE_GPIO_CTRL			0x25
	#define AXGE_GPIO_CTRL_GPIO3EN		0x80
	#define AXGE_GPIO_CTRL_GPIO2EN		0x40
	#define AXGE_GPIO_CTRL_GPIO1EN		0x20

#define AXGE_PHYPWR_RSTCTL		0x26
	#define AXGE_PHYPWR_RSTCTL_BZ		0x0010
	#define AXGE_PHYPWR_RSTCTL_IPRL		0x0020
	#define AXGE_PHYPWR_RSTCTL_AUTODETACH	0x1000

#define AXGE_RX_BULKIN_QCTRL		0x2e
	#define AXGE_RX_BULKIN_QCTRL_TIME		0x01
	#define AXGE_RX_BULKIN_QCTRL_IFG		0x02
	#define AXGE_RX_BULKIN_QCTRL_SIZE		0x04

#define AXGE_RX_BULKIN_QTIMR_LOW		0x2f
#define AXGE_RX_BULKIN_QTIMR_HIGH			0x30
#define AXGE_RX_BULKIN_QSIZE			0x31
#define AXGE_RX_BULKIN_QIFG			0x32

#define AXGE_CLK_SELECT			0x33
	#define AXGE_CLK_SELECT_BCS		0x01
	#define AXGE_CLK_SELECT_ACS		0x02
	#define AXGE_CLK_SELECT_ACSREQ		0x10
	#define AXGE_CLK_SELECT_ULR		0x08

#define AXGE_RXCOE_CTL			0x34
	#define AXGE_RXCOE_IP			0x01
	#define AXGE_RXCOE_TCP			0x02
	#define AXGE_RXCOE_UDP			0x04
	#define AXGE_RXCOE_ICMP			0x08
	#define AXGE_RXCOE_IGMP			0x10
	#define AXGE_RXCOE_TCPV6			0x20
	#define AXGE_RXCOE_UDPV6			0x40
	#define AXGE_RXCOE_ICMV6			0x80

	#define AXGE_RXCOE_DEF_CSUM	(AXGE_RXCOE_IP	| AXGE_RXCOE_TCP  | \
					 AXGE_RXCOE_UDP	| AXGE_RXCOE_ICMV6 | \
					 AXGE_RXCOE_TCPV6	| AXGE_RXCOE_UDPV6)

#define AXGE_TXCOE_CTL			0x35
	#define AXGE_TXCOE_IP			0x01
	#define AXGE_TXCOE_TCP			0x02
	#define AXGE_TXCOE_UDP			0x04
	#define AXGE_TXCOE_ICMP			0x08
	#define AXGE_TXCOE_IGMP			0x10
	#define AXGE_TXCOE_TCPV6			0x20
	#define AXGE_TXCOE_UDPV6			0x40
	#define AXGE_TXCOE_ICMV6			0x80
	#define AXGE_TXCOE_DEF_CSUM	(AXGE_TXCOE_TCP   | AXGE_TXCOE_UDP | \
					 AXGE_TXCOE_TCPV6 | AXGE_TXCOE_UDPV6)

#define AXGE_PAUSE_WATERLVL_HIGH		0x54
#define AXGE_PAUSE_WATERLVL_LOW		0x55


#define AXGE_EEP_EFUSE_CORRECT		0x00
#define AX88179_EEPROM_MAGIC			0x17900b95


/*****************************************************************************/
/* GMII register definitions */
#define GMII_PHY_CONTROL			0x00	/* control reg */
	/* Bit definitions: GMII Control */
	#define GMII_CONTROL_RESET		0x8000	/* reset bit in control reg */
	#define GMII_CONTROL_LOOPBACK		0x4000	/* loopback bit in control reg */
	#define GMII_CONTROL_10MB		0x0000	/* 10 Mbit */
	#define GMII_CONTROL_100MB		0x2000	/* 100Mbit */
	#define GMII_CONTROL_1000MB		0x0040	/* 1000Mbit */
	#define GMII_CONTROL_SPEED_BITS		0x2040	/* speed bit mask */
	#define GMII_CONTROL_ENABLE_AUTO	0x1000	/* autonegotiate enable */
	#define GMII_CONTROL_POWER_DOWN		0x0800
	#define GMII_CONTROL_ISOLATE		0x0400	/* islolate bit */
	#define GMII_CONTROL_START_AUTO		0x0200	/* restart autonegotiate */
	#define GMII_CONTROL_FULL_DUPLEX	0x0100

#define GMII_PHY_STATUS				0x01	/* status reg */
	/* Bit definitions: GMII Status */
	#define GMII_STATUS_100MB_MASK		0xE000	/* any of these indicate 100 Mbit */
	#define GMII_STATUS_10MB_MASK		0x1800	/* either of these indicate 10 Mbit */
	#define GMII_STATUS_AUTO_DONE		0x0020	/* auto negotiation complete */
	#define GMII_STATUS_AUTO		0x0008	/* auto negotiation is available */
	#define GMII_STATUS_LINK_UP		0x0004	/* link status bit */
	#define GMII_STATUS_EXTENDED		0x0001	/* extended regs exist */
	#define GMII_STATUS_100T4		0x8000	/* capable of 100BT4 */
	#define GMII_STATUS_100TXFD		0x4000	/* capable of 100BTX full duplex */
	#define GMII_STATUS_100TX		0x2000	/* capable of 100BTX */
	#define GMII_STATUS_10TFD		0x1000	/* capable of 10BT full duplex */
	#define GMII_STATUS_10T			0x0800	/* capable of 10BT */

#define GMII_PHY_OUI				0x02	/* most of the OUI bits */
#define GMII_PHY_MODEL				0x03	/* model/rev bits, and rest of OUI */
#define GMII_PHY_ANAR				0x04	/* AN advertisement reg */
	/* Bit definitions: Auto-Negotiation Advertisement */
	#define GMII_ANAR_ASYM_PAUSE		0x0800	/* support asymetric pause */
	#define GMII_ANAR_PAUSE			0x0400	/* support pause packets */
	#define GMII_ANAR_100T4			0x0200	/* support 100BT4 */
	#define GMII_ANAR_100TXFD		0x0100	/* support 100BTX full duplex */
	#define GMII_ANAR_100TX			0x0080	/* support 100BTX half duplex */
	#define GMII_ANAR_10TFD			0x0040	/* support 10BT full duplex */
	#define GMII_ANAR_10T			0x0020	/* support 10BT half duplex */
	#define GMII_SELECTOR_FIELD		0x001F	/* selector field. */

#define GMII_PHY_ANLPAR				0x05	/* AN Link Partner */
	/* Bit definitions: Auto-Negotiation Link Partner Ability */
	#define GMII_ANLPAR_100T4		0x0200	/* support 100BT4 */
	#define GMII_ANLPAR_100TXFD		0x0100	/* support 100BTX full duplex */
	#define GMII_ANLPAR_100TX		0x0080	/* support 100BTX half duplex */
	#define GMII_ANLPAR_10TFD		0x0040	/* support 10BT full duplex */
	#define GMII_ANLPAR_10T			0x0020	/* support 10BT half duplex */
	#define GMII_ANLPAR_PAUSE		0x0400	/* support pause packets */
	#define GMII_ANLPAR_ASYM_PAUSE		0x0800	/* support asymetric pause */
	#define GMII_ANLPAR_ACK			0x4000	/* means LCB was successfully rx'd */
	#define GMII_SELECTOR_8023		0x0001;

#define GMII_PHY_ANER				0x06	/* AN expansion reg */
#define GMII_PHY_1000BT_CONTROL			0x09	/* control reg for 1000BT */
#define GMII_PHY_1000BT_STATUS			0x0A	/* status reg for 1000BT */

#define GMII_PHY_PHYSR				0x11	/* PHY specific status register */
	#define GMII_PHY_PHYSR_SMASK		0xc000
	#define GMII_PHY_PHYSR_GIGA		0x8000
	#define GMII_PHY_PHYSR_100		0x4000
	#define GMII_PHY_PHYSR_FULL		0x2000
	#define GMII_PHY_PHYSR_LINK		0x400

/* Bit definitions: 1000BaseT AUX Control */
#define GMII_1000_AUX_CTRL_MASTER_SLAVE		0x1000
#define GMII_1000_AUX_CTRL_FD_CAPABLE		0x0200	/* full duplex capable */
#define GMII_1000_AUX_CTRL_HD_CAPABLE		0x0100	/* half duplex capable */

/* Bit definitions: 1000BaseT AUX Status */
#define GMII_1000_AUX_STATUS_FD_CAPABLE		0x0800	/* full duplex capable */
#define GMII_1000_AUX_STATUS_HD_CAPABLE		0x0400	/* half duplex capable */

/*Cicada MII Registers */
#define GMII_AUX_CTRL_STATUS			0x1C
#define GMII_AUX_ANEG_CPLT			0x8000
#define GMII_AUX_FDX				0x0020
#define GMII_AUX_SPEED_1000			0x0010
#define GMII_AUX_SPEED_100			0x0008

#define GMII_LED_ACTIVE				0x1a
	#define GMII_LED_ACTIVE_MASK		0xff8f
	#define GMII_LED0_ACTIVE		(1 << 4)
	#define GMII_LED1_ACTIVE		(1 << 5)
	#define GMII_LED2_ACTIVE		(1 << 6)

#define GMII_LED_LINK				0x1c
	#define GMII_LED_LINK_MASK		0xf888
	#define GMII_LED0_LINK_10		(1 << 0)
	#define GMII_LED0_LINK_100		(1 << 1)
	#define GMII_LED0_LINK_1000		(1 << 2)
	#define GMII_LED1_LINK_10		(1 << 4)
	#define GMII_LED1_LINK_100		(1 << 5)
	#define GMII_LED1_LINK_1000		(1 << 6)
	#define GMII_LED2_LINK_10		(1 << 8)
	#define GMII_LED2_LINK_100		(1 << 9)
	#define GMII_LED2_LINK_1000		(1 << 10)

	#define	LED_VALID	(1 << 15) /* UA2 LED Setting */

	#define	LED0_ACTIVE	(1 << 0)
	#define	LED0_LINK_10	(1 << 1)
	#define	LED0_LINK_100	(1 << 2)
	#define	LED0_LINK_1000	(1 << 3)
	#define	LED0_FD		(1 << 4)
	#define LED0_USB3_MASK	0x001f

	#define	LED1_ACTIVE	(1 << 5)
	#define	LED1_LINK_10	(1 << 6)
	#define	LED1_LINK_100	(1 << 7)
	#define	LED1_LINK_1000	(1 << 8)
	#define	LED1_FD		(1 << 9)
	#define LED1_USB3_MASK	0x03e0

	#define	LED2_ACTIVE	(1 << 10)
	#define	LED2_LINK_1000	(1 << 13)
	#define	LED2_LINK_100	(1 << 12)
	#define	LED2_LINK_10	(1 << 11)
	#define	LED2_FD		(1 << 14)
	#define LED2_USB3_MASK	0x7c00

#define GMII_PHYPAGE				0x1e

#define GMII_PHY_PAGE_SELECT			0x1f
	#define GMII_PHY_PAGE_SELECT_EXT	0x0007
	#define GMII_PHY_PAGE_SELECT_PAGE0	0X0000
	#define GMII_PHY_PAGE_SELECT_PAGE1	0X0001
	#define GMII_PHY_PAGE_SELECT_PAGE2	0X0002
	#define GMII_PHY_PAGE_SELECT_PAGE3	0X0003
	#define GMII_PHY_PAGE_SELECT_PAGE4	0X0004
	#define GMII_PHY_PAGE_SELECT_PAGE5	0X0005
	#define GMII_PHY_PAGE_SELECT_PAGE6	0X0006
/******************************************************************************/
#define	AXGE_CONFIG_IDX		0	/* config number 1 */
#define	AXGE_IFACE_IDX		0

#define AXGE_RXHDR_CRC_ERR                        0x80000000
#define AXGE_RXHDR_L4_ERR         (1 << 8)
#define AXGE_RXHDR_L3_ERR         (1 << 9)


#define AXGE_RXHDR_L4_TYPE_ICMP           2
#define AXGE_RXHDR_L4_TYPE_IGMP           3
#define AXGE_RXHDR_L4_TYPE_TCMPV6         5

#define AXGE_RXHDR_L3_TYPE_IP             1
#define AXGE_RXHDR_L3_TYPE_IPV6           2

#define AXGE_RXHDR_L4_TYPE_MASK                   0x1c
#define AXGE_RXHDR_L4_TYPE_UDP                    4
#define AXGE_RXHDR_L4_TYPE_TCP                    16
#define AXGE_RXHDR_L3CSUM_ERR                     2
#define AXGE_RXHDR_L4CSUM_ERR                     1
#define AXGE_RXHDR_CRC_ERR                        0x80000000
#define AXGE_RXHDR_DROP_ERR                       0x40000000

struct axge_csum_hdr {
	uint16_t cstatus;
#define	AXGE_CSUM_HDR_L4_CSUM_ERR	0x0001
#define	AXGE_CSUM_HDR_L3_CSUM_ERR	0x0002
#define	AXGE_CSUM_HDR_L4_TYPE_UDP	0x0004
#define	AXGE_CSUM_HDR_L4_TYPE_TCP	0x0010
#define	AXGE_CSUM_HDR_L4_TYPE_MASK	0x001C
#define AXGE_CSUM_HDR_L3_TYPE_IPV4	0x0020
	uint16_t len;
#define	AXGE_CSUM_HDR_LEN_MASK		0x1FFF
#define	AXGE_CSUM_HDR_CRC_ERR		0x1000
#define	AXGE_CSUM_HDR_MII_ERR		0x2000
#define	AXGE_CSUM_HDR_RUNT		0x4000
#define	AXGE_CSUM_HDR_BMCAST		0x8000
} __packed;

#define AXGE_CSUM_RXBYTES(x)	((x) & AXGE_CSUM_HDR_LEN_MASK)

#define	GET_MII(sc)		uether_getmii(&(sc)->sc_ue)

/* The interrupt endpoint is currently unused by the ASIX part. */
enum {
	AXGE_BULK_DT_WR,
	AXGE_BULK_DT_RD,
	AXGE_N_TRANSFER,
};

struct axge_softc {
	struct usb_ether	sc_ue;
	struct mtx		sc_mtx;
	struct usb_xfer		*sc_xfer[AXGE_N_TRANSFER];
	int			sc_phyno;

	int			sc_flags;
#define	AXGE_FLAG_LINK		0x0001	/* got a link */
};

#define	AXGE_LOCK(_sc)			mtx_lock(&(_sc)->sc_mtx)
#define	AXGE_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define	AXGE_LOCK_ASSERT(_sc, t)	mtx_assert(&(_sc)->sc_mtx, t)
