// ─────────────────────────────────────────────────────────────────────────────
// ca_code.cpp — GPS L1 C/A Gold Code Generator Implementation
//
// This is the .cpp file — it contains the actual implementation of everything
// declared in ca_code.hpp.
//
// RULE: .cpp files #include their own .hpp first. This ensures the header
//       is self-contained (compiles on its own without hidden dependencies).
// ─────────────────────────────────────────────────────────────────────────────

#include "ca_code.hpp"

// WHY #include <stdexcept>?
//   We throw std::invalid_argument if someone passes an invalid PRN.
//   This header declares that exception type.
//   In C++ you must include the header for every type/function you use.
#include <stdexcept>

// WHY #include <string>?
//   std::string is used to build the error message below.
#include <string>

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// generate_ca_code()
//
// Implements the two-LFSR Gold code generator per IS-GPS-200N §3.3.2.
//
// The algorithm:
//   1. Initialise G1 and G2 registers to all-ones.
//   2. For 1023 clock cycles:
//        a. Read G1 output from stage 10.
//        b. Read G2 output = XOR of stages tap_i and tap_j (PRN-specific).
//        c. CA chip = G1_out XOR G2_out.
//        d. Compute G1 feedback = G1[3] XOR G1[10], shift register right.
//        e. Compute G2 feedback = G2[2] XOR G2[3] XOR G2[6] XOR G2[8]
//                                  XOR G2[9] XOR G2[10], shift right.
//   3. Return the 1023-chip array.
// ─────────────────────────────────────────────────────────────────────────────

CACode generate_ca_code(int prn) {

    // ── Input validation ───────────────────────────────────────────────────
    // WHY validate here and not in the header?
    //   The header declares the contract ("call me with prn 1-32").
    //   The implementation enforces it. This is the "fail fast" principle:
    //   detect misuse as early as possible, not silently produce wrong data.
    //
    // WHY throw instead of returning an error code or bool?
    //   Returning an error code means every caller must check it:
    //       CACode code = generate_ca_code(prn);
    //       if (error) { ... }  // easy to forget
    //   An exception is impossible to ignore — if it isn't caught, the program
    //   terminates with a clear message. For programmer errors (invalid PRN),
    //   exceptions are the right tool.
    if (!is_valid_prn(prn)) {
        throw std::invalid_argument(
            "PRN " + std::to_string(prn) + " is out of range [1, 32]"
        );
    }

    // ── LFSR state ─────────────────────────────────────────────────────────
    // Both registers are 10-stage arrays initialised to all-ones.
    //
    // WHY std::array<int, 10> instead of a single uint10_t or uint16_t?
    //
    //   Option A: pack bits into a uint16_t and use bitwise ops.
    //             Fast but hard to read. The feedback equation becomes
    //             g1 = ((g1 >> 1) | (((g1 >> 2) ^ (g1 >> 9)) & 1) << 9)
    //             — nearly impossible to verify against the spec.
    //
    //   Option B: use an array where each element is one stage.
    //             Slower by a tiny constant factor, but the code reads
    //             exactly like the IS-GPS-200N specification. Maintainable.
    //             At 1023 iterations this "slowness" is completely negligible
    //             (microseconds). Clarity wins over micro-optimisation.
    //
    // We use 0-indexed arrays (stages [0]..[9] = spec stages 1..10).
    // The comment on each line makes the mapping explicit.

    std::array<int, kLFSRStages> g1, g2;
    g1.fill(1);   // .fill() sets every element — cleaner than a loop
    g2.fill(1);

    // Retrieve the tap pair for this PRN from the compile-time table.
    // Subtract 1: table uses 1-based stage numbers, array is 0-based.
    //
    // WHY auto& instead of G2Taps taps?
    //   'auto' lets the compiler deduce the type (G2Taps).
    //   The '&' makes it a reference — we read from the table in-place
    //   without copying the struct. For a tiny struct this doesn't matter
    //   much, but the habit of passing/reading by reference avoids needless
    //   copies when things get larger.
    const auto& taps = kG2TapTable[prn];
    const int ti = taps.tap_i - 1;   // 0-based index into g2 array
    const int tj = taps.tap_j - 1;

    // ── Output storage ─────────────────────────────────────────────────────
    // CACode is std::array<int8_t, 1023>.
    // WHY declare it here and fill it in the loop (not pre-initialise)?
    //   We'll write every element exactly once in the loop below.
    //   Pre-initialising with fill(0) would be redundant work.
    //   The compiler may optimise it away, but it's better to be explicit
    //   about intent: "we fill this fully in the loop."
    CACode code;

    // ── Main generation loop ───────────────────────────────────────────────
    // WHY int i and not size_t i?
    //   kCACodeLength (1023) is a signed int. Comparing a signed int to an
    //   unsigned size_t triggers a compiler warning. Using int throughout
    //   avoids the warning and is fine for small counts like 1023.
    for (int i = 0; i < kCACodeLength; ++i) {

        // Step 1: Read outputs from current register state BEFORE clocking.
        //
        // WHY before clocking?
        //   The IS-GPS-200N spec (and real hardware) outputs the chip from
        //   the current state, then advances the register. If you clock first,
        //   the first chip is wrong. This was the bug in our original JS demo.
        const int g1_out = g1[9];                // stage 10 output (0-indexed: 9)
        const int g2_out = g2[ti] ^ g2[tj];      // XOR of the two tap stages

        // Step 2: Compute the C/A chip.
        // XOR (^) in binary: same bits → 0, different bits → 1.
        // This is correct for the {0,1} domain used by the LFSRs.
        // Conversion to signal domain {-1, +1} happens later in the modulator.
        code[i] = static_cast<int8_t>(g1_out ^ g2_out);
        //
        // WHY static_cast<int8_t>?
        //   g1_out and g2_out are plain int. XOR of two ints is an int.
        //   Assigning int to int8_t without a cast generates a compiler warning
        //   (narrowing conversion). static_cast makes the narrowing intentional
        //   and explicit. The value is always 0 or 1, so no data is lost.

        // Step 3: Compute feedback bits for both registers.
        //
        // G1 polynomial: x^10 + x^3 + 1
        //   Feedback = stage[3] XOR stage[10]  (1-indexed)
        //            = g1[2]    XOR g1[9]       (0-indexed)
        //
        // G2 polynomial: x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1
        //   Feedback = stage[2]^stage[3]^stage[6]^stage[8]^stage[9]^stage[10]
        //            = g2[1]^g2[2]^g2[5]^g2[7]^g2[8]^g2[9]  (0-indexed)
        const int g1_fb = g1[2] ^ g1[9];
        const int g2_fb = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];

        // Step 4: Shift registers right and insert feedback at the left (stage 1).
        //
        // A shift register clocks like this:
        //   [s1, s2, s3, s4, s5, s6, s7, s8, s9, s10]
        //   After one clock:
        //   [fb, s1, s2, s3, s4, s5, s6, s7, s8,  s9]
        //   s10 is discarded (we already read it as output above).
        //
        // In code: shift everything right by one position, put feedback at [0].
        //
        // WHY a loop going backwards?
        //   If you went forwards: g1[1] = g1[0] (ok), then g1[2] = g1[1]
        //   But g1[1] is already overwritten! You'd copy the same value.
        //   Going backwards from the end: g1[9] = g1[8], g1[8] = g1[7], ...
        //   ensures each stage reads the previous stage's OLD value.
        for (int s = kLFSRStages - 1; s > 0; --s) {
            g1[s] = g1[s - 1];
            g2[s] = g2[s - 1];
        }
        g1[0] = g1_fb;
        g2[0] = g2_fb;
    }

    return code;
    // The compiler applies Return Value Optimisation (RVO) here:
    // the 1023-byte array is constructed directly in the caller's stack frame,
    // not copied. This is a C++17 guarantee (mandatory copy elision).
}

} // namespace gps
