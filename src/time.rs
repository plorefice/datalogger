use cast::{i64, u32};
use core::{
    cell::RefCell,
    cmp,
    convert::{Infallible, TryInto},
    fmt, ops,
    sync::atomic::{AtomicU32, Ordering},
    time,
};
use cortex_m::{
    interrupt::{self, Mutex},
    peripheral::NVIC,
};
use smolapps::net;
use stm32f4xx_hal::{
    rcc::Clocks,
    stm32::{Interrupt, TIM2},
    time::U32Ext,
    timer::{Event, Timer},
};

/// Current monotonic time expressed in milliseconds.
static TICK: AtomicU32 = AtomicU32::new(0);

/// System wall clock backing storage.
static CLOCK: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

/// A measurement of a monotonically nondecreasing clock. Opaque and useful only with `Duration`.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Instant {
    inner: i64,
}

impl Instant {
    /// Returns an instant corresponding to "now".
    pub fn now() -> Self {
        Instant {
            inner: i64(TICK.load(Ordering::Acquire)),
        }
    }

    /// Returns the amount of time elapsed since this instant was created.
    pub fn elapsed(&self) -> Duration {
        let diff = Instant::now().inner.wrapping_sub(self.inner);
        assert!(diff >= 0, "instant now is earlier than self");
        Duration::from_millis(u32(diff).unwrap())
    }

    /// Returns the amount of time elapsed from another instant to this one.
    pub fn duration_since(&self, earlier: Instant) -> Duration {
        let diff = self.inner.wrapping_sub(earlier.inner);
        assert!(diff >= 0, "second instant is later than self");
        Duration::from_millis(u32(diff).unwrap())
    }

    /// Returns the origin of time.
    pub const fn zero() -> Instant {
        Instant { inner: 0 }
    }
}

impl From<Instant> for net::time::Instant {
    fn from(i: Instant) -> Self {
        net::time::Instant::from_millis(i.inner)
    }
}

impl Into<Instant> for net::time::Instant {
    fn into(self) -> Instant {
        Instant { inner: self.millis }
    }
}

impl fmt::Debug for Instant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Instant").field(&self.inner).finish()
    }
}

impl ops::AddAssign<Duration> for Instant {
    fn add_assign(&mut self, dur: Duration) {
        self.inner = self.inner.wrapping_add(dur.as_millis() as i64);
    }
}

impl ops::Add<Duration> for Instant {
    type Output = Self;

    fn add(mut self, dur: Duration) -> Self {
        self += dur;
        self
    }
}

impl ops::SubAssign<Duration> for Instant {
    fn sub_assign(&mut self, dur: Duration) {
        self.inner = self.inner.wrapping_sub(dur.as_millis() as i64);
    }
}

impl ops::Sub<Duration> for Instant {
    type Output = Self;

    fn sub(mut self, dur: Duration) -> Self {
        self -= dur;
        self
    }
}

impl ops::Sub<Instant> for Instant {
    type Output = Duration;

    fn sub(self, other: Instant) -> Duration {
        self.duration_since(other)
    }
}

impl Ord for Instant {
    fn cmp(&self, rhs: &Self) -> cmp::Ordering {
        self.inner.wrapping_sub(rhs.inner).cmp(&0)
    }
}

impl PartialOrd for Instant {
    fn partial_cmp(&self, rhs: &Self) -> Option<cmp::Ordering> {
        Some(self.cmp(rhs))
    }
}

/// A measurement of the system clock, useful for talking to external entities
/// like the file system or other processes.
///
/// Counterpart of [`SystemTime`] present in the Standard Library.
///
/// [`SystemTime`]: https://doc.rust-lang.org/std/time/struct.SystemTime.html
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct SystemTime {
    inner: u64,
}

impl SystemTime {
    /// Returns the system time corresponding to "now".
    pub fn now() -> Self {
        Self {
            inner: interrupt::free(|cs| *CLOCK.borrow(cs).borrow()),
        }
    }

    /// Returns this system time expressed in seconds since Unix epoch.
    pub fn as_secs(&self) -> u64 {
        self.inner / 1_000
    }

    /// Ticks the system wall clock by one millisecond.
    pub fn tick() {
        interrupt::free(|cs| *CLOCK.borrow(cs).borrow_mut() += 1);
    }

    /// Sets the system wall clock to the specified `time`, espressed in **milliseconds**
    /// since the Unix epoch.
    pub fn adjust(time: u64) {
        interrupt::free(|cs| *CLOCK.borrow(cs).borrow_mut() = time);
    }
}

impl fmt::Debug for SystemTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("SystemTime").field(&self.inner).finish()
    }
}

/// A `Duration` type to represent a span of time, typically used for system timeouts.
#[derive(Clone, Copy, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct Duration {
    inner: time::Duration,
}

impl Duration {
    /// Creates a new `Duration` from the specified number of milliseconds.
    pub const fn from_millis(ms: u32) -> Self {
        Duration {
            inner: time::Duration::from_millis(ms as u64),
        }
    }

    /// Creates a new `Duration` from the specified number of seconds.
    pub const fn from_secs(s: u32) -> Self {
        Self::from_millis(s * 1_000)
    }

    /// Returns the total number of whole milliseconds contained by this `Duration`.
    pub fn as_millis(self) -> u64 {
        self.inner.as_millis() as u64
    }
}

impl From<net::time::Duration> for Duration {
    fn from(d: net::time::Duration) -> Self {
        Duration { inner: d.into() }
    }
}

impl TryInto<u32> for Duration {
    type Error = Infallible;

    fn try_into(self) -> Result<u32, Infallible> {
        Ok(self.as_millis() as u32)
    }
}

impl ops::AddAssign for Duration {
    fn add_assign(&mut self, dur: Duration) {
        self.inner += dur.inner;
    }
}

impl ops::Add<Duration> for Duration {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Duration {
            inner: self.inner + other.inner,
        }
    }
}

impl ops::SubAssign for Duration {
    fn sub_assign(&mut self, rhs: Duration) {
        self.inner -= rhs.inner;
    }
}

impl ops::Sub<Duration> for Duration {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Duration {
            inner: self.inner - rhs.inner,
        }
    }
}

/// Implementation of the `Monotonic` trait based on a hardware timer with millisecond precision.
pub struct Ticker {
    inner: Timer<TIM2>,
}

impl Ticker {
    /// Initializes the ticker to use the TIM2 peripheral under the hood.
    pub fn init(tim: TIM2, clocks: Clocks) -> Self {
        // NOTE(unsafe) we are not in an interrupt context
        unsafe { NVIC::unmask(Interrupt::TIM2) };

        let mut inner = Timer::tim2(tim, 1.khz(), clocks);
        inner.listen(Event::TimeOut);

        Self { inner }
    }

    /// Ticks the internal timer, increasing the current monotonic time by one millisecond.
    pub fn tick(&mut self) {
        self.inner.clear_interrupt(Event::TimeOut);
        TICK.fetch_add(1, Ordering::Release);
    }
}

impl rtfm::Monotonic for Ticker {
    type Instant = Instant;

    fn ratio() -> rtfm::Fraction {
        rtfm::Fraction {
            numerator: 168,
            denominator: 1,
        }
    }

    fn now() -> Self::Instant {
        Instant::now()
    }

    unsafe fn reset() {
        TICK.store(0, Ordering::Release);
    }

    fn zero() -> Self::Instant {
        Instant::zero()
    }
}
