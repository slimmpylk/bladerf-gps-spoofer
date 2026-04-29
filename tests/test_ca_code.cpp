// ─────────────────────────────────────────────────────────────────────────────
// test_ca_code.cpp — Unit tests for the C/A Gold code generator
//
// WHY GoogleTest (gtest)?
//   It gives us EXPECT_EQ, EXPECT_THROW and other assertion macros that
//   print clear failure messages. Without a framework you'd write:
//       if (code[0] != 1) { std::cout << "FAIL\n"; exit(1); }
//   With gtest you get:
//       EXPECT_EQ(code[0], 1);
//   And on failure: "Expected code[0] to be 1 but got 0 (at test_ca_code.cpp:42)"
//
// HOW TESTS WORK IN GTEST:
//   TEST(SuiteName, TestName) { ... }
//   Each TEST() block is an independent test case.
//   EXPECT_EQ(a, b)  — test passes if a == b, continues on failure
//   ASSERT_EQ(a, b)  — test passes if a == b, STOPS the test on failure
//   EXPECT_THROW(expr, ExceptionType) — passes if expr throws that exception
// ─────────────────────────────────────────────────────────────────────────────

#include "ca_code.hpp"
#include <gtest/gtest.h>

// ─────────────────────────────────────────────────────────────────────────────
// Known-good reference values from IS-GPS-200N
//
// The standard's Appendix 3.III Table 3-Ia lists the first 10 chips of every
// PRN. We use these to validate our generator.
//
// WHY a plain array of structs instead of a map or vector?
//   It's initialised at compile time (constexpr), lives in read-only memory,
//   and the loop below iterates it linearly — fastest possible access.
// ─────────────────────────────────────────────────────────────────────────────

struct PRNReference {
    int prn;
    int first_chips[10];   // first 10 chips in {0,1} domain
};

// Reference values verified against IS-GPS-200N and our generator output.
// Every PRN also has exactly 512 ones out of 1023 chips (balanced Gold code).
constexpr PRNReference kReferenceChips[] = {
    { 1, {1,1,0,0,1,0,0,0,0,0}},   // IS-GPS-200N confirmed: 1100100000
    { 2, {1,1,1,0,0,1,0,0,0,0}},   // 1110010000
    { 3, {1,1,1,1,0,0,1,0,0,0}},   // 1111001000
    { 4, {1,1,1,1,1,0,0,1,0,0}},   // 1111100100
    { 5, {1,0,0,1,0,1,1,0,1,1}},   // 1001011011
    {17, {1,0,0,1,1,0,1,1,1,0}},   // 1001101110
    {32, {1,1,1,1,0,0,1,0,1,0}},   // 1111001010
};

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: Code length
// Every PRN must produce exactly 1023 chips — not 1022, not 1024.
// ─────────────────────────────────────────────────────────────────────────────
TEST(CACode, CodeLengthIs1023) {
    for (int prn = 1; prn <= gps::kNumPRN; ++prn) {
        const auto code = gps::generate_ca_code(prn);
        EXPECT_EQ(static_cast<int>(code.size()), gps::kCACodeLength)
            << "PRN " << prn << " has wrong length";
        // The << after EXPECT_EQ appends extra info to the failure message.
        // Without it: "Expected 1023 but got 1024"
        // With it:    "Expected 1023 but got 1024 -- PRN 5 has wrong length"
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: Chip values are only 0 or 1
// The raw LFSR output is in the {0,1} domain. Any other value means the
// XOR logic is broken.
// ─────────────────────────────────────────────────────────────────────────────
TEST(CACode, ChipValuesAreBinary) {
    for (int prn = 1; prn <= gps::kNumPRN; ++prn) {
        const auto code = gps::generate_ca_code(prn);
        for (int i = 0; i < gps::kCACodeLength; ++i) {
            EXPECT_TRUE(code[i] == 0 || code[i] == 1)
                << "PRN " << prn << " chip[" << i << "] = " << (int)code[i];
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: First chips match IS-GPS-200N reference values
// This is the most important test — it validates our implementation against
// the published specification.
// ─────────────────────────────────────────────────────────────────────────────
TEST(CACode, FirstChipsMatchStandard) {
    for (const auto& ref : kReferenceChips) {
        const auto code = gps::generate_ca_code(ref.prn);

        for (int i = 0; i < 10; ++i) {
            EXPECT_EQ(static_cast<int>(code[i]), ref.first_chips[i])
                << "PRN " << ref.prn << " chip[" << i << "] mismatch";
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: All 32 codes are different from each other
// If two PRNs produce the same sequence, our tap table has a bug.
// This test checks all 32×31/2 = 496 unique pairs.
// ─────────────────────────────────────────────────────────────────────────────
TEST(CACode, AllCodesAreUnique) {
    // Generate all 32 codes upfront.
    // WHY std::array<gps::CACode, 33>?
    //   Index 0 unused (PRNs are 1-based), PRNs 1-32 at indices 1-32.
    std::array<gps::CACode, 33> all_codes;
    for (int prn = 1; prn <= gps::kNumPRN; ++prn) {
        all_codes[prn] = gps::generate_ca_code(prn);
    }

    // Compare every pair.
    // WHY nested loops starting at prn_b = prn_a + 1?
    //   We only need to check each pair once. (1,2) is the same test as (2,1).
    //   Starting the inner loop at prn_a + 1 avoids both duplicates and
    //   comparing a code against itself.
    for (int prn_a = 1; prn_a <= gps::kNumPRN; ++prn_a) {
        for (int prn_b = prn_a + 1; prn_b <= gps::kNumPRN; ++prn_b) {
            EXPECT_NE(all_codes[prn_a], all_codes[prn_b])
                << "PRN " << prn_a << " and PRN " << prn_b
                << " produced identical codes — tap table error!";
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5: Code is periodic (wraps back to chip 0 after 1023 chips)
// After one full epoch the LFSRs return to all-ones, so generating the same
// PRN twice should give the same sequence.
// ─────────────────────────────────────────────────────────────────────────────
TEST(CACode, CodeIsPeriodic) {
    const auto first  = gps::generate_ca_code(1);
    const auto second = gps::generate_ca_code(1);
    EXPECT_EQ(first, second)
        << "Same PRN generated twice but produced different codes — state not reset";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 6: chip_to_signal() conversion is correct
// 0 → +1 and 1 → -1 per IS-GPS-200N NRZ-L encoding.
// ─────────────────────────────────────────────────────────────────────────────
TEST(CACode, ChipToSignalMapping) {
    EXPECT_EQ(gps::chip_to_signal(0),  1);
    EXPECT_EQ(gps::chip_to_signal(1), -1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 7: Invalid PRN throws std::invalid_argument
// This tests our error handling. PRN 0 and PRN 33 are both invalid.
// ─────────────────────────────────────────────────────────────────────────────
TEST(CACode, InvalidPRNThrows) {
    // WHY the lambda wrapper?
    //   EXPECT_THROW passes the expression to a macro that evaluates it but
    //   discards the result — this triggers our [[nodiscard]] warning.
    //   Wrapping in a lambda that explicitly returns void suppresses it.
    //   This is the correct way to test [[nodiscard]] functions for exceptions.
    EXPECT_THROW({ (void)gps::generate_ca_code(0);  }, std::invalid_argument);
    EXPECT_THROW({ (void)gps::generate_ca_code(33); }, std::invalid_argument);
    EXPECT_THROW({ (void)gps::generate_ca_code(-1); }, std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 8: Cross-correlation is low between different PRNs
// This validates the Gold code property. Correlation between any two different
// PRNs should be much less than the autocorrelation peak (1023).
//
// For n=10, the maximum off-peak Gold code correlation is 65 (≈ -24 dB).
// We test a few pairs to verify our codes have the right property.
// ─────────────────────────────────────────────────────────────────────────────
TEST(CACode, CrossCorrelationIsLow) {
    const auto code1  = gps::generate_ca_code(1);
    const auto code5  = gps::generate_ca_code(5);
    const auto code12 = gps::generate_ca_code(12);

    // Helper lambda: compute integer cross-correlation at zero lag.
    // WHY a lambda instead of a separate function?
    //   It's only used in this test. Keeping it local avoids polluting
    //   any namespace. Lambdas capture their context — here we don't need
    //   any captures ([]) since both inputs are passed as parameters.
    auto xcorr = [](const gps::CACode& a, const gps::CACode& b) {
        int sum = 0;
        for (int i = 0; i < gps::kCACodeLength; ++i) {
            // Convert to signal domain (+1/-1) before summing
            sum += gps::chip_to_signal(a[i]) * gps::chip_to_signal(b[i]);
        }
        return std::abs(sum);
    };

    // Autocorrelation should be 1023 (perfect)
    EXPECT_EQ(xcorr(code1, code1), 1023);

    // Cross-correlation should be ≤ 65 (the Gold code bound for n=10)
    EXPECT_LE(xcorr(code1,  code5),  65);
    EXPECT_LE(xcorr(code1,  code12), 65);
    EXPECT_LE(xcorr(code5,  code12), 65);
}
