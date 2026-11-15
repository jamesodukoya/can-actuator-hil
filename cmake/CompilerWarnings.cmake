# Strict set for project code; -Wdouble-promotion catches silent float->double math on the STM32.
function(sbw_set_strict_warnings target)
  target_compile_options(${target} PRIVATE
    -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion -Wshadow
    -Wdouble-promotion -Wold-style-cast)
endfunction()

# Lighter set for tests: GoogleTest macros promote floats to double by design.
function(sbw_set_test_warnings target)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endfunction()

# Code that will run on the microcontrollers: no exceptions, no RTTI, no fused multiply-add,
# so results are bit-identical on x86-64 and aarch64.
function(sbw_set_embedded_flags target)
  target_compile_options(${target} PRIVATE -fno-exceptions -fno-rtti -ffp-contract=off)
endfunction()
