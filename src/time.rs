use cast::{u32, u64};
use core::{
    cell::RefCell,
    cmp::Ordering,
    convert::{Infallible, TryInto},
    fmt, ops, time,
};
use cortex_m::{
    interrupt::{self, Mutex},
    peripheral::NVIC,
};
use stm32f4xx_hal::{
    rcc::Clocks,
    stm32::{Interrupt, TIM2},
    time::U32Ext,
    timer::{Event, Timer},
};

/// Current time expressed in milliseconds.
/// It needs to be updated from interrupt context, so it is protected by a `Mutex`.
static NOW: Mutex<RefCell<i64>> = Mutex::new(RefCell::new(0));

/// A measurement of the amount of time elapsed since an arbitrary start.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Instant {
    inner: i64,
}

impl Instant {
    /// Returns an instant corresponding to "now".
    pub fn now() -> Self {
        Instant {
            inner: interrupt::free(|cs| *NOW.borrow(cs).borrow()),
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
    pub fn zero() -> Instant {
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
    fn cmp(&self, rhs: &Self) -> Ordering {
        self.inner.wrapping_sub(rhs.inner).cmp(&0)
    }
}

impl PartialOrd for Instant {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}

/// A `Duration` type to represent a span of time.
#[derive(Clone, Copy, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct Duration {
    inner: time::Duration,
}

impl Duration {
    /// Creates a new `Duration` from the specified number of milliseconds.
    pub fn from_millis(ms: u32) -> Self {
        Duration {
            inner: time::Duration::from_millis(u64(ms)),
        }
    }

    /// Returns the total number of whole milliseconds contained by this Duration.
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
pub struct SystemTimer {
    inner: Timer<TIM2>,
}

impl SystemTimer {
    /// Initializes the system timer.
    pub fn init(tim: TIM2, clocks: Clocks) -> Self {
        // NOTE(unsafe) this is used only during initialization
        unsafe { NVIC::unmask(Interrupt::TIM2) };

        let mut inner = Timer::tim2(tim, 1.khz(), clocks);
        inner.listen(Event::TimeOut);

        Self { inner }
    }

    /// Ticks the system timer, increasing the current time by one millisecond.
    pub fn tick(&mut self) {
        interrupt::free(|cs| {
            self.inner.clear_interrupt(Event::TimeOut);
            *NOW.borrow(cs).borrow_mut() += 1;
        });
    }
}

impl rtfm::Monotonic for SystemTimer {
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
        interrupt::free(|cs| *NOW.borrow(cs).borrow_mut() = 0);
    }

    fn zero() -> Self::Instant {
        Instant::zero()
    }
}
