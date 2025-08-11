use spin::Once;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClkError {
    InvalidClockRate,
    RegisterOperationFailed,
    InvalidPeripheralId,
    ResetTimeout,
    NotInitialized,
}

pub type ClkResult<T = ()> = Result<T, ClkError>;

pub trait Clk {
    fn init(&self) -> ClkResult;
    fn mmc_get_clk(&self) -> ClkResult<u64>;
    fn mmc_set_clk(&self, clk: u64) -> ClkResult<u64>;
}

static INIT_CLK: Once = Once::new();
static mut GLOBAL_CLK_INSTANCE: Option<&'static dyn Clk> = None;

pub fn init_global_clk(clk: &'static dyn Clk) {
    INIT_CLK.call_once(|| unsafe {
        GLOBAL_CLK_INSTANCE = Some(clk);
    });
}

pub fn global_clk() -> Result<&'static dyn Clk, ClkError> {
    unsafe {
        match GLOBAL_CLK_INSTANCE {
            Some(clk) => Ok(clk),
            None => Err(ClkError::NotInitialized),
        }
    }
}

pub fn mmc_get_clk() -> ClkResult<u64> {
    global_clk()?.mmc_get_clk()
}

pub fn mmc_set_clk(clk: u64) -> ClkResult<u64> {
    global_clk()?.mmc_set_clk(clk)
}
