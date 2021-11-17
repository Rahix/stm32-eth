#[cfg(feature = "stm32f4xx-hal")]
use stm32f4xx_hal::{
    bb,
    gpio::{
        gpioa::{PA0, PA1, PA3, PA7},
        gpiob::{PB0, PB1, PB8, PB10, PB11, PB12, PB13},
        gpioc::{PC2, PC3, PC4, PC5},
        gpiog::{PG11, PG13, PG14},
        Floating, Input,
        Speed::VeryHigh,
    },
    stm32::{RCC, SYSCFG},
};

#[cfg(feature = "stm32f7xx-hal")]
use cortex_m::interrupt;
#[cfg(feature = "stm32f7xx-hal")]
use stm32f7xx_hal::{
    gpio::{
        gpioa::{PA1, PA7},
        gpiob::{PB11, PB12, PB13},
        gpioc::{PC4, PC5},
        gpiog::{PG11, PG13, PG14},
        Floating, Input,
        Speed::VeryHigh,
    },
    pac::{RCC, SYSCFG},
};

// Enable syscfg and ethernet clocks. Reset the Ethernet MAC.
pub(crate) fn setup(rmii: bool) {
    #[cfg(feature = "stm32f4xx-hal")]
    unsafe {
        const SYSCFG_BIT: u8 = 14;
        const ETH_MAC_BIT: u8 = 25;
        const ETH_TX_BIT: u8 = 26;
        const ETH_RX_BIT: u8 = 27;
        const MII_RMII_BIT: u8 = 23;

        //NOTE(unsafe) This will only be used for atomic writes with no side-effects
        let rcc = &*RCC::ptr();
        let syscfg = &*SYSCFG::ptr();

        // Enable syscfg clock
        bb::set(&rcc.apb2enr, SYSCFG_BIT);

        if rcc.ahb1enr.read().ethmacen().bit_is_set() {
            // pmc must be changed with the ethernet controller disabled or under reset
            bb::clear(&rcc.ahb1enr, ETH_MAC_BIT);
        }
        // select MII or RMII mode
        // 0 = MII, 1 = RMII
        if rmii {
            bb::set(&syscfg.pmc, MII_RMII_BIT);
        } else {
            bb::clear(&syscfg.pmc, MII_RMII_BIT);
        }

        // enable ethernet clocks
        bb::set(&rcc.ahb1enr, ETH_MAC_BIT);
        bb::set(&rcc.ahb1enr, ETH_TX_BIT);
        bb::set(&rcc.ahb1enr, ETH_RX_BIT);

        // reset pulse
        bb::set(&rcc.ahb1rstr, ETH_MAC_BIT);
        bb::clear(&rcc.ahb1rstr, ETH_MAC_BIT);
    }
    #[cfg(feature = "stm32f7xx-hal")]
    //stm32f7xx-hal does not currently have bitbanding
    interrupt::free(|_| unsafe {
        //NOTE(unsafe) Interrupt free and we only modify mac bits
        let rcc = &*RCC::ptr();
        let syscfg = &*SYSCFG::ptr();
        // enable syscfg clock
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

        if rcc.ahb1enr.read().ethmacen().bit_is_set() {
            // pmc must be changed with the ethernet controller disabled or under reset
            rcc.ahb1enr.modify(|_, w| w.ethmacen().clear_bit());
        }

        // select MII or RMII mode
        // 0 = MII, 1 = RMII
        syscfg.pmc.modify(|_, w| w.mii_rmii_sel().bit(rmii));

        // enable ethernet clocks
        rcc.ahb1enr.modify(|_, w| {
            w.ethmacen()
                .set_bit()
                .ethmactxen()
                .set_bit()
                .ethmacrxen()
                .set_bit()
        });

        //reset pulse
        rcc.ahb1rstr.modify(|_, w| w.ethmacrst().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());
    });
}

/// RMII Reference Clock.
pub unsafe trait RmiiRefClk {}

/// RMII RX Data Valid.
pub unsafe trait RmiiCrsDv {}

/// RMII TX Enable.
pub unsafe trait RmiiTxEN {}

/// RMII TXD0.
pub unsafe trait RmiiTxD0 {}

/// RMII TXD1.
pub unsafe trait RmiiTxD1 {}

/// RMII RXD0.
pub unsafe trait RmiiRxD0 {}

/// RMII RXD1.
pub unsafe trait RmiiRxD1 {}

pub unsafe trait MiiTxClk {}
pub unsafe trait MiiTxD0 {}
pub unsafe trait MiiTxD1 {}
pub unsafe trait MiiTxD2 {}
pub unsafe trait MiiTxD3 {}
pub unsafe trait MiiTxEn {}

pub unsafe trait MiiRxClk {}
pub unsafe trait MiiRxD0 {}
pub unsafe trait MiiRxD1 {}
pub unsafe trait MiiRxD2 {}
pub unsafe trait MiiRxD3 {}
pub unsafe trait MiiRxDv {}
pub unsafe trait MiiRxEr {}
pub unsafe trait MiiCrs {}
pub unsafe trait MiiCol {}

/// Trait needed to setup the pins for the Ethernet peripheral.
pub trait AlternateVeryHighSpeed {
    /// Puts the pin in the Alternate Function 11 with Very High Speed.
    fn into_af11_very_high_speed(self);
}

#[cfg(feature = "stm32f4xx-hal")]
impl<const P: char, const N: u8> AlternateVeryHighSpeed for stm32f4xx_hal::gpio::Pin<Input<Floating>, P, N> {
    fn into_af11_very_high_speed(self) {
        self.into_alternate::<11>().set_speed(VeryHigh);
    }
}

pub struct EthPins<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1> {
    pub ref_clk: REFCLK,
    pub crs: CRS,
    pub tx_en: TXEN,
    pub tx_d0: TXD0,
    pub tx_d1: TXD1,
    pub rx_d0: RXD0,
    pub rx_d1: RXD1,
}

impl<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1> EthPins<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>
where
    REFCLK: RmiiRefClk + AlternateVeryHighSpeed,
    CRS: RmiiCrsDv + AlternateVeryHighSpeed,
    TXEN: RmiiTxEN + AlternateVeryHighSpeed,
    TXD0: RmiiTxD0 + AlternateVeryHighSpeed,
    TXD1: RmiiTxD1 + AlternateVeryHighSpeed,
    RXD0: RmiiRxD0 + AlternateVeryHighSpeed,
    RXD1: RmiiRxD1 + AlternateVeryHighSpeed,
{
    /// Pin setup.
    ///
    /// Set RMII pins to
    /// * Alternate function 11
    /// * High-speed
    ///
    /// This function consumes the pins so that you cannot use them
    /// anywhere else by accident.
    pub fn setup_pins(self) {
        self.ref_clk.into_af11_very_high_speed();
        self.crs.into_af11_very_high_speed();
        self.tx_en.into_af11_very_high_speed();
        self.tx_d0.into_af11_very_high_speed();
        self.tx_d1.into_af11_very_high_speed();
        self.rx_d0.into_af11_very_high_speed();
        self.rx_d1.into_af11_very_high_speed();
    }
}

#[rustfmt::skip]
pub struct EthPinsMii<
    TXCLK, TXD0, TXD1, TXD2, TXD3, TXEN,
    RXCLK, RXD0, RXD1, RXD2, RXD3, RXDV, RXER, CRS, COL,
> {
    pub tx_clk: TXCLK,
    pub txd0: TXD0,
    pub txd1: TXD1,
    pub txd2: TXD2,
    pub txd3: TXD3,
    pub tx_en: TXEN,

    pub rx_clk: RXCLK,
    pub rxd0: RXD0,
    pub rxd1: RXD1,
    pub rxd2: RXD2,
    pub rxd3: RXD3,
    pub rx_dv: RXDV,
    pub rx_er: RXER,
    pub crs: CRS,
    pub col: COL,
}

#[rustfmt::skip]
impl<
    TXCLK, TXD0, TXD1, TXD2, TXD3, TXEN,
    RXCLK, RXD0, RXD1, RXD2, RXD3, RXDV, RXER, CRS, COL,
> EthPinsMii<
    TXCLK, TXD0, TXD1, TXD2, TXD3, TXEN,
    RXCLK, RXD0, RXD1, RXD2, RXD3, RXDV, RXER, CRS, COL,
>
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
    pub fn setup_pins(self) {
        self.tx_clk.into_af11_very_high_speed();
        self.txd0.into_af11_very_high_speed();
        self.txd1.into_af11_very_high_speed();
        self.txd2.into_af11_very_high_speed();
        self.txd3.into_af11_very_high_speed();
        self.tx_en.into_af11_very_high_speed();

        self.rx_clk.into_af11_very_high_speed();
        self.rxd0.into_af11_very_high_speed();
        self.rxd1.into_af11_very_high_speed();
        self.rxd2.into_af11_very_high_speed();
        self.rxd3.into_af11_very_high_speed();
        self.rx_dv.into_af11_very_high_speed();
        self.rx_er.into_af11_very_high_speed();
        self.crs.into_af11_very_high_speed();
        self.col.into_af11_very_high_speed();
    }
}

macro_rules! impl_pins {
    ( $($traity:ident: [$($pin:ty,)+],)+ ) => {
        $(
            $(
                unsafe impl $traity for $pin {}
            )+
        )+
    };
}

#[cfg(feature = "device-selected")]
impl_pins!(
    RmiiRefClk: [
        PA1<Input<Floating>>,
    ],
    RmiiCrsDv: [
        PA7<Input<Floating>>,
    ],
    RmiiTxEN: [
        PB11<Input<Floating>>,
        PG11<Input<Floating>>,
    ],
    RmiiTxD0: [
        PB12<Input<Floating>>,
        PG13<Input<Floating>>,
    ],
    RmiiTxD1: [
        PB13<Input<Floating>>,
        PG14<Input<Floating>>,
    ],
    RmiiRxD0: [
        PC4<Input<Floating>>,
    ],
    RmiiRxD1: [
        PC5<Input<Floating>>,
    ],
);

#[cfg(feature = "device-selected")]
impl_pins!(
    MiiTxClk: [
        PC3<Input<Floating>>,
    ],
    MiiTxD0: [
        PB12<Input<Floating>>,
    ],
    MiiTxD1: [
        PB13<Input<Floating>>,
    ],
    MiiTxD2: [
        PC2<Input<Floating>>,
    ],
    MiiTxD3: [
        PB8<Input<Floating>>,
    ],
    MiiTxEn: [
        PB11<Input<Floating>>,
    ],
    MiiRxClk: [
        PA1<Input<Floating>>,
    ],
    MiiRxD0: [
        PC4<Input<Floating>>,
    ],
    MiiRxD1: [
        PC5<Input<Floating>>,
    ],
    MiiRxD2: [
        PB0<Input<Floating>>,
    ],
    MiiRxD3: [
        PB1<Input<Floating>>,
    ],
    MiiRxDv: [
        PA7<Input<Floating>>,
    ],
    MiiRxEr: [
        PB10<Input<Floating>>,
    ],
    MiiCrs: [
        PA0<Input<Floating>>,
    ],
    MiiCol: [
        PA3<Input<Floating>>,
    ],
);
