use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex as EmbassyMutex;

pub type Mutex<T> = EmbassyMutex<CriticalSectionRawMutex, T>;

/// This is a macro that improves on the embassy join function
/// with the advantage of joining any number of futures.
#[macro_export]
macro_rules! join {
    ($f0:expr, $f1: expr, $($rest:expr),+ $(,)?) => {{
        join!($f0, join!($f1, $($rest),+))
    }};
    ($f0:expr, $f1: expr $(,)?) => {{
        use embassy_futures::join::join;
        join($f0, $f1)
    }};
    ($f0:expr $(,)?) => {{
        $f0
    }};
}
