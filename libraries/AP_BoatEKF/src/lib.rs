#![no_std]

use panic_abort as _;

#[unsafe(no_mangle)]
pub extern "C" fn call_from_rust() {}

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
