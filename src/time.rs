use atsamd_hal::{
    clock::Tcc2Tc3Clock,
    prelude::*,
    samd21::timer::TimerCounter3,
    target_device::{PM, TC3},
};
use cast::{i64, u32};
use core::{
    cell::RefCell,
    cmp,
    convert::{Infallible, TryInto},
    fmt, ops, time,
};
use cortex_m::interrupt::{self, Mutex};

/// Current monotonic time expressed in milliseconds.
static TICK: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

/// A measurement of a monotonically nondecreasing clock. Opaque and useful only with `Duration`.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Instant {
    inner: i64,
}

impl Instant {
    /// Returns an instant corresponding to "now".
    pub fn now() -> Self {
        Instant {
            inner: i64(interrupt::free(|cs| *TICK.borrow(cs).borrow())),
        }
    }

    /// Returns the amount of time elapsed since this instant was created.
    pub fn elapsed(self) -> Duration {
        let diff = Instant::now().inner.wrapping_sub(self.inner);
        assert!(diff >= 0, "instant now is earlier than self");
        Duration::from_millis(u32(diff).unwrap())
    }

    /// Returns the amount of time elapsed from another instant to this one.
    pub fn duration_since(self, earlier: Instant) -> Duration {
        let diff = self.inner.wrapping_sub(earlier.inner);
        assert!(diff >= 0, "second instant is later than self");
        Duration::from_millis(u32(diff).unwrap())
    }

    /// Returns the origin of time.
    pub const fn zero() -> Instant {
        Instant { inner: 0 }
    }
}

impl fmt::Debug for Instant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Instant").field(&self.inner).finish()
    }
}

impl ops::AddAssign<Duration> for Instant {
    fn add_assign(&mut self, dur: Duration) {
        self.inner = self.inner.wrapping_add(i64(dur.as_millis()).unwrap());
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
        self.inner = self.inner.wrapping_sub(i64(dur.as_millis()).unwrap());
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
    inner: TimerCounter3,
}

impl Ticker {
    /// Initializes the ticker to use the TC3 peripheral under the hood.
    pub fn init(tc: TC3, clock: &Tcc2Tc3Clock, pm: &mut PM) -> Self {
        let mut inner = TimerCounter3::tc3_(clock, tc, pm);
        inner.start(1.khz());
        inner.enable_interrupt();

        Self { inner }
    }

    /// Ticks the internal timer, increasing the current monotonic time by one millisecond.
    pub fn tick(&mut self) {
        if self.inner.wait().is_ok() {
            interrupt::free(|cs| {
                *TICK.borrow(cs).borrow_mut() += 1;
            });
        }
    }
}

impl rtfm::Monotonic for Ticker {
    type Instant = Instant;

    fn ratio() -> rtfm::Fraction {
        rtfm::Fraction {
            numerator: 48,
            denominator: 1,
        }
    }

    fn now() -> Self::Instant {
        Instant::now()
    }

    unsafe fn reset() {
        interrupt::free(|cs| *TICK.borrow(cs).borrow_mut() = 0);
    }

    fn zero() -> Self::Instant {
        Instant::zero()
    }
}
