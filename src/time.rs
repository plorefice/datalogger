use cast::{i32, u16, u32, u64};
use core::{
    cmp::Ordering,
    convert::{Infallible, TryInto},
    fmt,
    marker::PhantomData,
    ops, time,
};
use stm32f4xx_hal::{rcc::Clocks, stm32::RCC};

/// Concrete hardware timer to use as provider.
type TIM = stm32f4xx_hal::stm32::TIM2;

/// A measurement of the number of microseconds elapsed since an arbitrary point in time.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Instant {
    inner: i32,
}

impl Instant {
    /// Returns an instant corresponding to "now".
    pub fn now() -> Self {
        // NOTE(unsafe) TIM must only be used by this module
        let tim = unsafe { &*TIM::ptr() };

        Instant {
            inner: i32(tim.cnt.read().bits()).unwrap(),
        }
    }

    /// Returns the amount of time elapsed since this instant was created.
    pub fn elapsed(&self) -> Duration {
        let diff = Instant::now().inner.wrapping_sub(self.inner);
        assert!(diff >= 0, "instant now is earlier than self");
        Duration::from_micros(u32(diff).unwrap())
    }

    /// Returns the amount of time elapsed from another instant to this one.
    pub fn duration_since(&self, earlier: Instant) -> Duration {
        let diff = self.inner.wrapping_sub(earlier.inner);
        assert!(diff >= 0, "second instant is later than self");
        Duration::from_micros(u32(diff).unwrap())
    }
}

impl fmt::Debug for Instant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Instant")
            .field(&(self.inner as u32))
            .finish()
    }
}

impl ops::AddAssign<Duration> for Instant {
    fn add_assign(&mut self, dur: Duration) {
        self.inner = self.inner.wrapping_add(dur.inner.as_micros() as i32);
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
        self.inner = self.inner.wrapping_sub(dur.inner.as_micros() as i32);
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

    /// Creates a new `Duration` from the specified number of microseconds.
    pub fn from_micros(us: u32) -> Self {
        Duration {
            inner: time::Duration::from_micros(u64(us)),
        }
    }
}

impl TryInto<u32> for Duration {
    type Error = Infallible;

    fn try_into(self) -> Result<u32, Infallible> {
        Ok(self.inner.as_micros() as u32)
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

/// Adds the `ms` and `us` method to the `u32` type.
pub trait U32Ext {
    /// Converts the `u32` value into milliseconds.
    fn ms(self) -> Duration;

    /// Converts the `u32` value into microseconds.
    fn us(self) -> Duration;
}

impl U32Ext for u32 {
    fn ms(self) -> Duration {
        Duration::from_millis(self)
    }

    fn us(self) -> Duration {
        Duration::from_micros(self)
    }
}

/// Implementation of the `Monotonic` trait based on a hardware timer with microsecond precision.
///
/// The `F` parameter must be set to the system clock frequency in Mhz using `typenum`.
pub struct SystemTimer<F> {
    _marker: PhantomData<F>,
}

impl<F> SystemTimer<F> {
    pub fn constrain(tim: TIM, clocks: Clocks) {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // Enable and reset peripheral to a clean slate state
        rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());

        // Pause and reset counter
        tim.cr1.modify(|_, w| w.cen().clear_bit());
        tim.cnt.reset();

        // Configure prescaler to achieve a counter frequency of 1 Mhz
        let pclk_mul = if clocks.ppre1() == 1 { 1 } else { 2 };
        let ticks = u16(clocks.pclk1().0 * pclk_mul / 1_000_000).unwrap();
        tim.psc.write(|w| w.psc().bits(ticks - 1));

        // Trigger prescaler update
        tim.egr.write(|w| w.ug().set_bit());

        // Start counter
        tim.cr1.modify(|_, w| w.cen().set_bit());
    }
}

impl<F> rtfm::Monotonic for SystemTimer<F>
where
    F: typenum::Unsigned,
{
    type Instant = Instant;

    fn ratio() -> rtfm::Fraction {
        rtfm::Fraction {
            numerator: F::U32,
            denominator: 1,
        }
    }

    fn now() -> Self::Instant {
        Instant::now()
    }

    unsafe fn reset() {
        { &*TIM::ptr() }.cnt.reset();
    }

    fn zero() -> Self::Instant {
        Instant { inner: 0 }
    }
}
