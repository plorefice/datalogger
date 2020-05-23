use crate::time::Instant;
use core::mem::MaybeUninit;
use smolapps::{
    net::socket::{
        RawPacketMetadata, RawSocketBuffer, SocketHandle, SocketSet, SocketSetItem,
        UdpPacketMetadata, UdpSocket, UdpSocketBuffer,
    },
    net::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
    net::{
        dhcp::Dhcpv4Client,
        iface::{
            EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache, Route, Routes,
        },
    },
    sntp, tftp,
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

const SNTP_SERVER_ADDR: [u8; 4] = [62, 112, 134, 4];

/// Container for all the network resources.
///
/// Everything that needs to run just in the network loop goes here.
pub struct Netlink<'a> {
    pub iface: EthernetInterface<'a, 'a, 'a, Eth<'a, 'a>>,
    pub sockets: SocketSet<'a, 'a, 'a>,
    pub dhcp: Dhcpv4Client,
    pub sntp: sntp::Client,
    pub tftp: tftp::Server,
}

/// Performs all the heavy lifting required to bring up the Ethernet interface
/// and allocate all the data structures required for the network stack to function correctly.
///
/// Every single buffer allocated by this function is `static`, meaning that it won't be
/// stack-allocated. This is good, because we can catch OOM conditions during compilation.
pub fn setup(syscfg: SYSCFG, pins: PINS, mac: ETHERNET_MAC, dma: ETHERNET_DMA) -> Netlink<'static> {
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
    // NOTE(unsafe) initialization of MaybeUninit static variable and static mut dereference
    let eth = {
        static mut RX_RING: MaybeUninit<[RingEntry<RxDescriptor>; 8]> = MaybeUninit::uninit();
        static mut TX_RING: MaybeUninit<[RingEntry<TxDescriptor>; 4]> = MaybeUninit::uninit();

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

    // Create socket set
    // NOTE(unsafe) initialization of MaybeUninit static variable and static mut dereference
    let mut sockets = unsafe {
        static mut SOCKET_ENTRIES: [Option<SocketSetItem>; 4] = [None, None, None, None];
        SocketSet::new(&mut SOCKET_ENTRIES[..])
    };

    let iface = setup_interface(eth);

    let dhcp = setup_dhcp_client(&mut sockets);
    let sntp = setup_sntp_client(&mut sockets);
    let tftp = setup_tftp_server(&mut sockets);

    Netlink {
        iface,
        sockets,
        dhcp,
        sntp,
        tftp,
    }
}

/// Creates the `smoltcp` interface.
fn setup_interface<'a>(eth: Eth<'a, 'a>) -> EthernetInterface<'a, 'a, 'a, Eth<'a, 'a>> {
    static mut IP_ADDRESS: MaybeUninit<[IpCidr; 1]> = MaybeUninit::uninit();

    // NOTE(unsafe) static variable initialization
    let neighbor_cache = unsafe {
        static mut CACHE_ENTRIES: [Option<(IpAddress, Neighbor)>; 8] = [None; 8];
        NeighborCache::new(&mut CACHE_ENTRIES[..])
    };

    // NOTE(unsafe) initialization of MaybeUninit static variable
    unsafe {
        IP_ADDRESS
            .as_mut_ptr()
            .write([IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 0)]);
    }

    let routes = {
        static mut ROUTES_STORAGE: [Option<(IpCidr, Route)>; 1] = [None; 1];
        // NOTE(unsafe) static variable initialization
        Routes::new(unsafe { &mut ROUTES_STORAGE[..] })
    };

    EthernetInterfaceBuilder::new(eth)
        .ethernet_addr(EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x02]))
        .neighbor_cache(neighbor_cache)
        .ip_addrs(unsafe { &mut (*IP_ADDRESS.as_mut_ptr())[..] })
        .routes(routes)
        .finalize()
}

/// Spawns a DHCP client.
fn setup_dhcp_client<'a, 'b, 'c>(sockets: &mut SocketSet<'a, 'b, 'c>) -> Dhcpv4Client
where
    'b: 'c,
{
    // NOTE(unsafe) static variable initialization
    let rx_buffer = unsafe {
        static mut RAW_METADATA: [RawPacketMetadata; 1] = [RawPacketMetadata::EMPTY; 1];
        static mut RAW_DATA: [u8; 900] = [0; 900];
        RawSocketBuffer::new(&mut RAW_METADATA[..], &mut RAW_DATA[..])
    };

    // NOTE(unsafe) static variable initialization
    let tx_buffer = unsafe {
        static mut RAW_METADATA: [RawPacketMetadata; 1] = [RawPacketMetadata::EMPTY; 1];
        static mut RAW_DATA: [u8; 600] = [0; 600];
        RawSocketBuffer::new(&mut RAW_METADATA[..], &mut RAW_DATA[..])
    };

    Dhcpv4Client::new(sockets, rx_buffer, tx_buffer, Instant::now().into())
}

/// Spawns an SNTP client.
fn setup_sntp_client<'a, 'b, 'c>(sockets: &mut SocketSet<'a, 'b, 'c>) -> sntp::Client
where
    'b: 'c,
{
    // NOTE(unsafe) static variable initialization
    let rx_buffer = unsafe {
        static mut UDP_METADATA: [UdpPacketMetadata; 1] = [UdpPacketMetadata::EMPTY; 1];
        static mut UDP_DATA: [u8; 128] = [0; 128];
        UdpSocketBuffer::new(&mut UDP_METADATA[..], &mut UDP_DATA[..])
    };

    // NOTE(unsafe) static variable initialization
    let tx_buffer = unsafe {
        static mut UDP_METADATA: [UdpPacketMetadata; 1] = [UdpPacketMetadata::EMPTY; 1];
        static mut UDP_DATA: [u8; 128] = [0; 128];
        UdpSocketBuffer::new(&mut UDP_METADATA[..], &mut UDP_DATA[..])
    };

    sntp::Client::new(
        sockets,
        rx_buffer,
        tx_buffer,
        Ipv4Address::from_bytes(&SNTP_SERVER_ADDR[..]).into(),
        Instant::now().into(),
    )
}

/// Spawns a TFTP server.
fn setup_tftp_server(sockets: &mut SocketSet) -> tftp::Server {
    // NOTE(unsafe) static variable initialization
    let rx_buffer = unsafe {
        static mut UDP_METADATA: [UdpPacketMetadata; 2] = [UdpPacketMetadata::EMPTY; 2];
        static mut UDP_DATA: [u8; 1200] = [0; 1200];
        UdpSocketBuffer::new(&mut UDP_METADATA[..], &mut UDP_DATA[..])
    };

    // NOTE(unsafe) static variable initialization
    let tx_buffer = unsafe {
        static mut UDP_METADATA: [UdpPacketMetadata; 2] = [UdpPacketMetadata::EMPTY; 2];
        static mut UDP_DATA: [u8; 1200] = [0; 1200];
        UdpSocketBuffer::new(&mut UDP_METADATA[..], &mut UDP_DATA[..])
    };

    tftp::Server::new(sockets, rx_buffer, tx_buffer, Instant::now().into())
}

/// Creates a UDP socket for heartbeat packets.
pub fn create_heartbeat_socket(sockets: &mut SocketSet) -> SocketHandle {
    // NOTE(unsafe) static variable initialization
    let buffer = unsafe {
        static mut UDP_METADATA: [UdpPacketMetadata; 1] = [UdpPacketMetadata::EMPTY; 1];
        static mut UDP_DATA: [u8; 64] = [0; 64];
        UdpSocketBuffer::new(&mut UDP_METADATA[..], &mut UDP_DATA[..])
    };

    let socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut [UdpPacketMetadata::EMPTY; 0][..], &mut [0; 0][..]),
        buffer,
    );

    sockets.add(socket)
}
