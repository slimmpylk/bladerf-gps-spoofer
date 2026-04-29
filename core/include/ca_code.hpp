#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// ca_code.hpp — GPS L1 C/A Gold Code Generator
//
// WHAT THIS FILE IS:
//   A header file. In C++ you split every component into two files:
//     .hpp  (header)  — declares WHAT exists (types, function signatures)
//     .cpp  (source)  — defines HOW it works (actual implementation)
//
// WHY SPLIT LIKE THIS?
//   When main.cpp does #include "ca_code.hpp", the compiler only needs to
//   know the function signatures — it doesn't need to recompile ca_code.cpp.
//   This is why C++ builds are incremental: change ca_code.cpp and only that
//   file recompiles. Without the split, every change would recompile everything.
//
// #pragma once:
//   Tells the preprocessor: "only include this file once per translation unit,
//   even if someone accidentally #includes it twice." The older equivalent is
//   #ifndef / #define / #endif guards, but #pragma once is cleaner.
//
// REFERENCE: IS-GPS-200N §3.3.2, Table 3-Ia
// ─────────────────────────────────────────────────────────────────────────────

// Standard library includes.
// WHY <array>?   std::array<T,N> is a fixed-size array whose size is known at
//                compile time. It's safer than C-style int arr[1023] because
//                it knows its own size, has .at() bounds-checking, and works
//                with range-for loops.
// WHY <cstdint>? Gives us int8_t, uint8_t, int32_t etc. — integer types with
//                explicit widths. If you write 'int' you don't know if it's
//                16, 32, or 64 bits. int8_t is always exactly 8 bits. In
//                embedded/RF work this precision matters.
#include <array>
#include <cstdint>
#include <span>     // C++20: a non-owning view into a contiguous sequence

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// Constants
//
// WHY constexpr instead of #define?
//
//   #define CA_CODE_LENGTH 1023
//   vs
//   constexpr int kCACodeLength = 1023;
//
//   #define is a text substitution done by the preprocessor before the compiler
//   even runs. It has no type, no scope, can't be debugged. If you write
//   #define X 1023 and then do X + 0.5 somewhere, you get a silent float.
//
//   constexpr is a typed, scoped, compile-time constant. The compiler handles
//   it, it shows up in the debugger, and it participates in type checking.
//   Always prefer constexpr over #define for constants in C++.
//
// WHY 'k' prefix?
//   Google C++ Style Guide convention: constants start with 'k'. Not required
//   but makes it immediately obvious in code that something is a constant.
// ─────────────────────────────────────────────────────────────────────────────

constexpr int kCACodeLength   = 1023;   // chips per epoch (IS-GPS-200N §3.3.2)
constexpr int kNumPRN         = 32;     // legacy L1 C/A PRN assignments
constexpr int kLFSRStages     = 10;     // both G1 and G2 are 10-stage registers

// ─────────────────────────────────────────────────────────────────────────────
// Type alias
//
// WHY using CACode = ... instead of just using the array type directly?
//   Readability. "CACode prn1_code" is clearer than
//   "std::array<int8_t, 1023> prn1_code" scattered across the codebase.
//   If we ever change the representation (e.g. to bitset), we change one line.
//
// WHY int8_t (signed 8-bit integer) for chips?
//   Each chip value is either 0 or 1 (raw LFSR output) or -1 / +1 (signal
//   domain). int8_t holds both representations, uses 1 byte per chip, and the
//   full code (1023 chips) fits in 1 KB of memory. We could use bool for the
//   raw domain, but int8_t avoids conversion overhead when we multiply later.
// ─────────────────────────────────────────────────────────────────────────────
using CACode = std::array<int8_t, kCACodeLength>;

// ─────────────────────────────────────────────────────────────────────────────
// G2 tap assignments — IS-GPS-200N Table 3-Ia
//
// Each PRN's unique code is determined by which two stages of G2 are XOR-ed
// together before combining with G1's output.
//
// WHY a struct instead of two separate arrays?
//   Keeps the two taps for each PRN together. If you had int g2_tap1[32] and
//   int g2_tap2[32] separately, you could accidentally index them differently.
//   A struct pairs them permanently.
//
// WHY const at the end of the struct? (it's inside the G2Taps table below,
//   which is constexpr — this makes the whole table evaluated at compile time)
// ─────────────────────────────────────────────────────────────────────────────
struct G2Taps {
    int tap_i;   // first G2 register stage (1-indexed, per IS-GPS-200N)
    int tap_j;   // second G2 register stage
};

// The full table — index [0] is unused (PRNs are 1-based, not 0-based).
// Making this constexpr means the entire table lives in read-only memory
// (like a constant in the executable) and never needs to be computed at runtime.
//
// WHY std::array<G2Taps, 33> and not G2Taps table[33]?
//   std::array knows its size (.size() = 33) and works with range-for.
//   A C-style array is just a pointer with no built-in bounds knowledge.
constexpr std::array<G2Taps, 33> kG2TapTable = {{
    {0,  0},   // [0]  unused — PRNs are 1-indexed
    {2,  6},   // [1]  PRN 1
    {3,  7},   // [2]  PRN 2
    {4,  8},   // [3]  PRN 3
    {5,  9},   // [4]  PRN 4
    {1,  9},   // [5]  PRN 5
    {2, 10},   // [6]  PRN 6
    {1,  8},   // [7]  PRN 7
    {2,  9},   // [8]  PRN 8
    {3, 10},   // [9]  PRN 9
    {2,  3},   // [10] PRN 10
    {3,  4},   // [11] PRN 11
    {5,  6},   // [12] PRN 12
    {6,  7},   // [13] PRN 13
    {7,  8},   // [14] PRN 14
    {8,  9},   // [15] PRN 15
    {9, 10},   // [16] PRN 16
    {1,  4},   // [17] PRN 17
    {2,  5},   // [18] PRN 18
    {3,  6},   // [19] PRN 19
    {4,  7},   // [20] PRN 20
    {5,  8},   // [21] PRN 21
    {6,  9},   // [22] PRN 22
    {1,  3},   // [23] PRN 23
    {4,  6},   // [24] PRN 24
    {5,  7},   // [25] PRN 25
    {6,  8},   // [26] PRN 26
    {7,  9},   // [27] PRN 27
    {8, 10},   // [28] PRN 28
    {1,  6},   // [29] PRN 29
    {2,  7},   // [30] PRN 30
    {3,  8},   // [31] PRN 31
    {4,  9},   // [32] PRN 32
}};


// ─────────────────────────────────────────────────────────────────────────────
// Function declarations
//
// This is where we DECLARE that these functions exist. The actual code
// (implementation) is in ca_code.cpp.
//
// WHY declare here and define in .cpp?
//   Any file that #includes ca_code.hpp can call these functions without
//   the compiler needing to see how they're implemented. It just needs to
//   know: "generate_ca_code exists, takes an int, returns a CACode."
//   The linker handles connecting calls to implementations later.
// ─────────────────────────────────────────────────────────────────────────────

// Generate the full 1023-chip C/A code for a given PRN (1–32).
//
// WHY [[nodiscard]]?
//   The compiler will warn if you call this function and throw away the result.
//   Example: if someone accidentally writes
//       generate_ca_code(5);   // result discarded — almost certainly a bug
//   they get a compiler warning. [[nodiscard]] is a free safety net.
//
// WHY return by value (CACode) instead of taking an output pointer?
//   Modern C++ compilers use Return Value Optimisation (RVO) — the returned
//   std::array is constructed directly in the caller's memory, no copy happens.
//   Returning by value is both safe (no dangling pointers) and efficient.
[[nodiscard]] CACode generate_ca_code(int prn);

// Convert a raw {0,1} chip to signal-domain {-1, +1}.
// Mapping: 0 → +1,  1 → -1  (NRZ-L encoding per IS-GPS-200N)
//
// WHY inline?
//   This is a tiny one-liner called millions of times per second in the
//   modulator. 'inline' suggests to the compiler: "embed this code at the
//   call site instead of doing a real function call." Modern compilers do
//   this automatically for small functions, but inline makes the intent clear.
//
// WHY constexpr here too?
//   If the input is a compile-time constant (e.g. chip_to_signal(0)),
//   constexpr lets the compiler evaluate it at compile time — the call
//   disappears entirely from the generated machine code.
[[nodiscard]] constexpr int8_t chip_to_signal(int8_t chip) noexcept {
    // noexcept = this function never throws an exception.
    // WHY mark noexcept? Two reasons:
    //   1. The compiler can generate slightly faster code (no exception
    //      handling machinery needed).
    //   2. It documents the contract: callers know this is safe to call
    //      in exception-sensitive contexts (like signal handlers).
    return (chip == 0) ? int8_t{1} : int8_t{-1};
}

// Validate that prn is in the range [1, 32].
// Called at the start of generate_ca_code() to catch misuse early.
[[nodiscard]] constexpr bool is_valid_prn(int prn) noexcept {
    return prn >= 1 && prn <= kNumPRN;
}

} // namespace gps
//
// WHY namespace gps?
//   Namespaces prevent name collisions. If some library you include also has
//   a function called generate_ca_code(), without a namespace you'd get a
//   linker error. With namespaces it's gps::generate_ca_code() vs
//   their_lib::generate_ca_code() — unambiguous.
//   Convention: use short lowercase names for project namespaces.
