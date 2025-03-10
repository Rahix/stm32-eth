#![no_std]

/// Re-export
#[cfg(feature = "stm32f7xx-hal")]
pub use stm32f7xx_hal as hal;
/// Re-export
#[cfg(feature = "stm32f7xx-hal")]
pub use stm32f7xx_hal::pac as stm32;

/// Re-export
#[cfg(feature = "stm32f4xx-hal")]
pub use stm32f4xx_hal as hal;
/// Re-export
#[cfg(feature = "stm32f4xx-hal")]
pub use stm32f4xx_hal::stm32;

use hal::rcc::Clocks;
use stm32::{Interrupt, ETHERNET_DMA, ETHERNET_MAC, NVIC};

mod ring;
#[cfg(feature = "smi")]
pub mod smi;
pub use ring::RingEntry;
mod desc;
mod rx;
pub use rx::{RxDescriptor, RxError, RxRingEntry};
use rx::{RxPacket, RxRing};
mod tx;
use tx::TxRing;
pub use tx::{TxDescriptor, TxError, TxRingEntry};
pub mod setup;
use setup::{
    AlternateVeryHighSpeed, MiiCol, MiiCrs, MiiRxClk, MiiRxD0, MiiRxD1, MiiRxD2, MiiRxD3, MiiRxDv,
    MiiRxEr, MiiTxClk, MiiTxD0, MiiTxD1, MiiTxD2, MiiTxD3, MiiTxEn, RmiiCrsDv, RmiiRefClk,
    RmiiRxD0, RmiiRxD1, RmiiTxD0, RmiiTxD1, RmiiTxEN,
};
pub use setup::{EthPins, EthPinsMii};

#[cfg(feature = "smoltcp-phy")]
pub use smoltcp;
#[cfg(feature = "smoltcp-phy")]
mod smoltcp_phy;
#[cfg(feature = "smoltcp-phy")]
pub use smoltcp_phy::{EthRxToken, EthTxToken};

/// From the datasheet: *VLAN Frame maxsize = 1522*
const MTU: usize = 1522;

mod consts {
    /* For HCLK 60-100 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_42: u8 = 0;
    /* For HCLK 100-150 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_62: u8 = 1;
    /* For HCLK 20-35 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_16: u8 = 2;
    /* For HCLK 35-60 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_26: u8 = 3;
    /* For HCLK over 150 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_102: u8 = 4;
}
use self::consts::*;

/// HCLK must be at least 25MHz to use the ethernet peripheral.
#[derive(Debug)]
pub struct WrongClock;

/// Ethernet driver for *STM32* chips.
pub struct Eth<'rx, 'tx> {
    eth_mac: ETHERNET_MAC,
    eth_dma: ETHERNET_DMA,
    rx_ring: RxRing<'rx>,
    tx_ring: TxRing<'tx>,
}

impl<'rx, 'tx> Eth<'rx, 'tx> {
    /// Initialize and start tx and rx DMA engines.
    ///
    /// Make sure that the buffers reside in a memory region that is
    /// accessible by the peripheral. Core-Coupled Memory (CCM) is
    /// usually not accessible. HCLK must be at least 25MHz.
    ///
    /// Uses an interrupt free critical section to turn on the ethernet clock
    /// for STM32F7xx.
    ///
    /// Other than that, initializes and starts the Ethernet hardware
    /// so that you can [`send()`](#method.send) and
    /// [`recv_next()`](#method.recv_next).
    pub fn new<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>(
        eth_mac: ETHERNET_MAC,
        eth_dma: ETHERNET_DMA,
        rx_buffer: &'rx mut [RxRingEntry],
        tx_buffer: &'tx mut [TxRingEntry],
        clocks: Clocks,
        pins: EthPins<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>,
    ) -> Result<Self, WrongClock>
    where
        REFCLK: RmiiRefClk + AlternateVeryHighSpeed,
        CRS: RmiiCrsDv + AlternateVeryHighSpeed,
        TXEN: RmiiTxEN + AlternateVeryHighSpeed,
        TXD0: RmiiTxD0 + AlternateVeryHighSpeed,
        TXD1: RmiiTxD1 + AlternateVeryHighSpeed,
        RXD0: RmiiRxD0 + AlternateVeryHighSpeed,
        RXD1: RmiiRxD1 + AlternateVeryHighSpeed,
    {
        setup::setup(true);
        pins.setup_pins();
        let mut eth = Eth {
            eth_mac,
            eth_dma,
            rx_ring: RxRing::new(rx_buffer),
            tx_ring: TxRing::new(tx_buffer),
        };
        eth.init(clocks)?;
        eth.rx_ring.start(&eth.eth_dma);
        eth.tx_ring.start(&eth.eth_dma);
        Ok(eth)
    }

    #[rustfmt::skip]
    pub fn new_mii<
        TXCLK, TXD0, TXD1, TXD2, TXD3, TXEN,
        RXCLK, RXD0, RXD1, RXD2, RXD3, RXDV, RXER, CRS, COL,
    >(
        eth_mac: ETHERNET_MAC,
        eth_dma: ETHERNET_DMA,
        rx_buffer: &'rx mut [RxRingEntry],
        tx_buffer: &'tx mut [TxRingEntry],
        clocks: Clocks,
        pins: EthPinsMii<
            TXCLK, TXD0, TXD1, TXD2, TXD3, TXEN,
            RXCLK, RXD0, RXD1, RXD2, RXD3, RXDV, RXER, CRS, COL,
        >,
    ) -> Result<Self, WrongClock>
    where
        TXCLK: MiiTxClk, TXD0: MiiTxD0, TXD1: MiiTxD1, TXD2: MiiTxD2, TXD3: MiiTxD3, TXEN: MiiTxEn,
        RXCLK: MiiRxClk, RXD0: MiiRxD0, RXD1: MiiRxD1, RXD2: MiiRxD2, RXD3: MiiRxD3, RXDV: MiiRxDv, RXER: MiiRxEr,
        CRS: MiiCrs, COL: MiiCol,

        TXCLK: AlternateVeryHighSpeed,
        TXD0: AlternateVeryHighSpeed,
        TXD1: AlternateVeryHighSpeed,
        TXD2: AlternateVeryHighSpeed,
        TXD3: AlternateVeryHighSpeed,
        TXEN: AlternateVeryHighSpeed,
        RXCLK: AlternateVeryHighSpeed,
        RXD0: AlternateVeryHighSpeed,
        RXD1: AlternateVeryHighSpeed,
        RXD2: AlternateVeryHighSpeed,
        RXD3: AlternateVeryHighSpeed,
        RXDV: AlternateVeryHighSpeed,
        RXER: AlternateVeryHighSpeed,
        CRS: AlternateVeryHighSpeed,
        COL: AlternateVeryHighSpeed,
    {
        setup::setup(false);
        pins.setup_pins();
        let mut eth = Eth {
            eth_mac,
            eth_dma,
            rx_ring: RxRing::new(rx_buffer),
            tx_ring: TxRing::new(tx_buffer),
        };
        eth.init(clocks)?;
        eth.rx_ring.start(&eth.eth_dma);
        eth.tx_ring.start(&eth.eth_dma);
        Ok(eth)
    }

    fn init(&mut self, clocks: Clocks) -> Result<(), WrongClock> {
        let clock_range = match clocks.hclk().0 {
            0..=24_999_999 => return Err(WrongClock),
            25_000_000..=34_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_16,
            35_000_000..=59_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_26,
            60_000_000..=99_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_42,
            100_000_000..=149_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_62,
            _ => ETH_MACMIIAR_CR_HCLK_DIV_102,
        };
        self.reset_dma_and_wait();

        // set clock range in MAC MII address register
        self.eth_mac
            .macmiiar
            .modify(|_, w| unsafe { w.cr().bits(clock_range) });

        // Configuration Register
        self.eth_mac.maccr.modify(|_, w| {
            // CRC stripping for Type frames
            w.cstf()
                .set_bit()
                // Fast Ethernet speed
                .fes()
                .set_bit()
                // Duplex mode
                .dm()
                .set_bit()
                // Automatic pad/CRC stripping
                .apcs()
                .set_bit()
                // Retry disable in half-duplex mode
                .rd()
                .set_bit()
                // Receiver enable
                .re()
                .set_bit()
                // Transmitter enable
                .te()
                .set_bit()
        });
        // frame filter register
        self.eth_mac.macffr.modify(|_, w| {
            // Receive All
            w.ra()
                .set_bit()
                // Promiscuous mode
                .pm()
                .set_bit()
        });
        // Flow Control Register
        self.eth_mac.macfcr.modify(|_, w| {
            // Pause time
            w.pt().bits(0x100)
        });
        // operation mode register
        self.eth_dma.dmaomr.modify(|_, w| {
            // Dropping of TCP/IP checksum error frames disable
            w.dtcefd()
                .set_bit()
                // Receive store and forward
                .rsf()
                .set_bit()
                // Disable flushing of received frames
                .dfrf()
                .set_bit()
                // Transmit store and forward
                .tsf()
                .set_bit()
                // Forward error frames
                .fef()
                .set_bit()
                // Operate on second frame
                .osf()
                .set_bit()
        });
        // bus mode register
        self.eth_dma.dmabmr.modify(|_, w| unsafe {
            // Address-aligned beats
            w.aab()
                .set_bit()
                // Fixed burst
                .fb()
                .set_bit()
                // Rx DMA PBL
                .rdp()
                .bits(32)
                // Programmable burst length
                .pbl()
                .bits(32)
                // Rx Tx priority ratio 2:1
                .pm()
                .bits(0b01)
                // Use separate PBL
                .usp()
                .set_bit()
        });
        Ok(())
    }

    /// reset DMA bus mode register
    fn reset_dma_and_wait(&self) {
        self.eth_dma.dmabmr.modify(|_, w| w.sr().set_bit());

        // Wait until done
        while self.eth_dma.dmabmr.read().sr().bit_is_set() {}
    }

    /// Enable RX and TX interrupts
    ///
    /// In your handler you must call
    /// [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html) to
    /// clear interrupt pending bits. Otherwise the interrupt will
    /// reoccur immediately.
    pub fn enable_interrupt(&self) {
        self.eth_dma.dmaier.modify(|_, w| {
            w
                // Normal interrupt summary enable
                .nise()
                .set_bit()
                // Receive Interrupt Enable
                .rie()
                .set_bit()
                // Transmit Interrupt Enable
                .tie()
                .set_bit()
        });

        // Enable ethernet interrupts
        unsafe {
            NVIC::unmask(Interrupt::ETH);
        }
    }

    /// Calls [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html)
    pub fn interrupt_handler(&self) {
        eth_interrupt_handler(&self.eth_dma);
    }

    /// Is Rx DMA currently running?
    ///
    /// It stops if the ring is full. Call `recv_next()` to free an
    /// entry and to demand poll from the hardware.
    pub fn rx_is_running(&self) -> bool {
        self.rx_ring.running_state(&self.eth_dma).is_running()
    }

    /// Receive the next packet (if any is ready), or return `None`
    /// immediately.
    pub fn recv_next(&mut self) -> Result<RxPacket, RxError> {
        self.rx_ring.recv_next(&self.eth_dma)
    }

    /// Is Tx DMA currently running?
    pub fn tx_is_running(&self) -> bool {
        self.tx_ring.is_running(&self.eth_dma)
    }

    /// Send a packet
    pub fn send<F: FnOnce(&mut [u8]) -> R, R>(
        &mut self,
        length: usize,
        f: F,
    ) -> Result<R, TxError> {
        let result = self.tx_ring.send(length, f);
        self.tx_ring.demand_poll(&self.eth_dma);
        result
    }

    #[cfg(feature = "smi")]
    /// Borrow access to the MAC's SMI.
    ///
    /// Allows for controlling and monitoring any PHYs that may be accessible via the MDIO/MDC
    /// pins.
    ///
    /// Exclusive access to the `MDIO` and `MDC` is required to ensure that are not used elsewhere
    /// for the duration of SMI communication.
    pub fn smi<'eth, 'pins, Mdio, Mdc>(
        &'eth mut self,
        mdio: &'pins mut Mdio,
        mdc: &'pins mut Mdc,
    ) -> smi::Smi<'eth, 'pins, Mdio, Mdc>
    where
        Mdio: smi::MdioPin,
        Mdc: smi::MdcPin,
    {
        smi::Smi::new(&self.eth_mac.macmiiar, &self.eth_mac.macmiidr, mdio, mdc)
    }
}

/// Call in interrupt handler to clear interrupt reason, when
/// [`enable_interrupt()`](struct.Eth.html#method.enable_interrupt).
///
/// There are two ways to call this:
///
/// * Via the [`Eth`](struct.Eth.html) driver instance that your interrupt handler has access to.
/// * By unsafely getting `Peripherals`.
///
/// TODO: could return interrupt reason
pub fn eth_interrupt_handler(eth_dma: &ETHERNET_DMA) {
    eth_dma
        .dmasr
        .write(|w| w.nis().set_bit().rs().set_bit().ts().set_bit());
}
