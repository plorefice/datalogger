use core::mem::MaybeUninit;
use sntp::{
    net::iface::{
        EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache, Route, Routes,
    },
    net::socket::{SocketSet, SocketSetItem, UdpPacketMetadata, UdpSocketBuffer},
    net::time::Instant,
    net::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
    Client,
};
use stm32_eth::{Eth, RingEntry, RxDescriptor, TxDescriptor};
use stm32f4xx_hal::{
    gpio::{
        gpioa::{PA1, PA2, PA7},
        gpiob::{PB11, PB12, PB13},
        gpioc::{PC1, PC4, PC5},
        Alternate,
        Speed::VeryHigh,
        AF11,
    },
    stm32::{ETHERNET_DMA, ETHERNET_MAC, RCC, SYSCFG},
};

type PINS = (
    PA1<Alternate<AF11>>,
    PA2<Alternate<AF11>>,
    PA7<Alternate<AF11>>,
    PB11<Alternate<AF11>>,
    PB12<Alternate<AF11>>,
    PB13<Alternate<AF11>>,
    PC1<Alternate<AF11>>,
    PC4<Alternate<AF11>>,
    PC5<Alternate<AF11>>,
);

/// Container for all the network resources.
/// TODO: refactor this ugly interface.
pub struct Netlink {
    pub(crate) iface: EthernetInterface<'static, 'static, 'static, Eth<'static, 'static>>,
    pub(crate) sntp: Client,
    pub(crate) sockets: SocketSet<'static, 'static, 'static>,
}

/// Performs all the heavy lifting required to bring up the Ethernet interface
/// and allocate all the data structures required for smoltcp to function correctly.
///
/// Every single buffer allocated by this function is static, meaning that it won't be
/// stack-allocated. This is good, because we can catch OOM conditions during compilation.
pub fn setup(syscfg: SYSCFG, pins: PINS, mac: ETHERNET_MAC, dma: ETHERNET_DMA) -> Netlink {
    // Initialize ethernet MAC
    {
        let rcc = unsafe { &*RCC::ptr() };
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());
        syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());
        rcc.ahb1enr.modify(|_, w| {
            w.ethmacen()
                .set_bit()
                .ethmactxen()
                .set_bit()
                .ethmacrxen()
                .set_bit()
        });
        rcc.ahb1rstr.modify(|_, w| w.ethmacrst().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());
    }

    // Setup ethernet pin speed
    pins.0.set_speed(VeryHigh);
    pins.1.set_speed(VeryHigh);
    pins.2.set_speed(VeryHigh);
    pins.3.set_speed(VeryHigh);
    pins.4.set_speed(VeryHigh);
    pins.5.set_speed(VeryHigh);
    pins.6.set_speed(VeryHigh);
    pins.7.set_speed(VeryHigh);
    pins.8.set_speed(VeryHigh);

    // Configure Ethernet peripheral
    let eth = {
        static mut RX_RING: MaybeUninit<[RingEntry<RxDescriptor>; 8]> = MaybeUninit::uninit();
        static mut TX_RING: MaybeUninit<[RingEntry<TxDescriptor>; 2]> = MaybeUninit::uninit();

        unsafe {
            RX_RING.as_mut_ptr().write(Default::default());
            TX_RING.as_mut_ptr().write(Default::default());
        };

        Eth::new(
            mac,
            dma,
            unsafe { &mut (*RX_RING.as_mut_ptr())[..] },
            unsafe { &mut (*TX_RING.as_mut_ptr())[..] },
        )
    };

    // Start receiving packets
    eth.enable_interrupt();

    // Create smoltcp interface
    let iface = {
        let ethernet_addr = EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x02]);

        let neighbor_cache = unsafe {
            static mut CACHE_ENTRIES: [Option<(IpAddress, Neighbor)>; 8] = [None; 8];
            NeighborCache::new(&mut CACHE_ENTRIES[..])
        };

        static mut IP_ADDRESS: MaybeUninit<[IpCidr; 1]> = MaybeUninit::uninit();
        unsafe {
            IP_ADDRESS
                .as_mut_ptr()
                .write([IpCidr::new(IpAddress::v4(192, 168, 2, 2), 24)]);
        }

        let mut routes = {
            static mut ROUTES_STORAGE: [Option<(IpCidr, Route)>; 1] = [None; 1];
            Routes::new(unsafe { &mut ROUTES_STORAGE[..] })
        };

        let default_v4_gw = Ipv4Address::new(192, 168, 2, 1);
        routes.add_default_ipv4_route(default_v4_gw).unwrap();

        EthernetInterfaceBuilder::new(eth)
            .ethernet_addr(ethernet_addr)
            .neighbor_cache(neighbor_cache)
            .ip_addrs(unsafe { &mut (*IP_ADDRESS.as_mut_ptr())[..] })
            .routes(routes)
            .finalize()
    };

    // Create socket set
    let mut sockets = unsafe {
        static mut SOCKET_ENTRIES: [Option<SocketSetItem>; 1] = [None; 1];
        SocketSet::new(&mut SOCKET_ENTRIES[..])
    };

    // Create SNTP client
    let sntp = {
        let sntp_rx_buffer = unsafe {
            static mut UDP_METADATA: [UdpPacketMetadata; 1] = [UdpPacketMetadata::EMPTY; 1];
            static mut UDP_DATA: [u8; 128] = [0; 128];
            UdpSocketBuffer::new(&mut UDP_METADATA[..], &mut UDP_DATA[..])
        };

        let sntp_tx_buffer = unsafe {
            static mut UDP_METADATA: [UdpPacketMetadata; 1] = [UdpPacketMetadata::EMPTY; 1];
            static mut UDP_DATA: [u8; 128] = [0; 128];
            UdpSocketBuffer::new(&mut UDP_METADATA[..], &mut UDP_DATA[..])
        };

        let server = IpAddress::v4(62, 112, 134, 4);

        Client::new(
            &mut sockets,
            sntp_rx_buffer,
            sntp_tx_buffer,
            server,
            Instant::from_secs(0),
        )
    };

    Netlink {
        iface,
        sockets,
        sntp,
    }
}
