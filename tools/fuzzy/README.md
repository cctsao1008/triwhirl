# Fuzzy reference validation

This directory validates TriWhirl's deterministic embedded fuzzy-inference semantics against a pinned FuzzyLite host oracle.

## Boundary

FuzzyLite is **host/SITL/reference tooling only**. It is not part of the ESP32 production firmware dependency graph. The production fuzzy runtime remains TriWhirl-owned, allocation-free code under `components/triwhirl_core/include/triwhirl/fuzzy.hpp`.

The repository pins `third_party/fuzzylite` to an exact upstream commit. FuzzyLite is GPLv3/proprietary dual-licensed, while TriWhirl is MIT; do not link FuzzyLite into distributed production firmware without an explicit licensing review.

A normal firmware checkout/build does not need the submodule initialized. Only reference work needs:

```bash
git submodule update --init third_party/fuzzylite
```

## Upright-balance structural fixture

`controllers/fuzzy/balance.fll` is a normalized **structural parity fixture**, not a tuned controller and not evidence of closed-loop stability. Its domains are:

```text
theta_error    [-1, +1]
theta_dot      [-1, +1]
wheel_velocity [-1, +1]
       -> target_velocity [-1, +1]
```

Each input uses the same symmetric five-term partition as the embedded implementation: `NL`, `NS`, `ZE`, `PS`, `PL`. The 125 zero-order Takagi-Sugeno rules use product firing and weighted-average defuzzification.

At the linguistic centers the deliberately neutral reference surface is:

```text
u_norm = -(theta_error_norm + theta_dot_norm + wheel_velocity_norm) / 3
```

That equation exists only to make the first three-input inference surface explicit and independently checkable. Physical normalization scales, rule shaping, stability, capture behavior and momentum management are later control-design work.

## Reproduce the checked-in FLL

```bash
python3 tools/fuzzy/generate_balance_reference.py --output /tmp/balance.fll
diff -u controllers/fuzzy/balance.fll /tmp/balance.fll
```

## Build and run the oracle comparison

```bash
cmake -S tools/fuzzy -B build/fuzzy-reference -DCMAKE_BUILD_TYPE=Release
cmake --build build/fuzzy-reference --parallel
./build/fuzzy-reference/fuzzy_reference_test controllers/fuzzy/balance.fll
```

The test checks all 125 linguistic centers and a deterministic 21x21x21 dense grid. It requires finite bounded output, full rule coverage, normalized clamping parity, antisymmetry, and a maximum embedded-vs-FuzzyLite inference error no greater than `1e-5`.
