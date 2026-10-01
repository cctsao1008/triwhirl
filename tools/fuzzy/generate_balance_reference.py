#!/usr/bin/env python3
"""Generate the normalized TriWhirl upright-balance FuzzyLite reference model.

This is a structural inference-parity fixture. It is intentionally not a
closed-loop tuning result and must not be treated as a hardware-ready balance
controller.
"""

from __future__ import annotations

import argparse
from pathlib import Path

TERMS = ("NL", "NS", "ZE", "PS", "PL")


def output_term(level: int) -> str:
    if level == 0:
        return "Z"
    return ("P" if level > 0 else "N") + str(abs(level))


def append_input(lines: list[str], name: str) -> None:
    lines.extend(
        [
            f"InputVariable: {name}",
            "  enabled: true",
            "  range: -1.000000000 1.000000000",
            "  lock-range: true",
            "  term: NL Trapezoid -1.000000000 -1.000000000 -1.000000000 -0.500000000",
            "  term: NS Triangle -1.000000000 -0.500000000 0.000000000",
            "  term: ZE Triangle -0.500000000 0.000000000 0.500000000",
            "  term: PS Triangle 0.000000000 0.500000000 1.000000000",
            "  term: PL Trapezoid 0.500000000 1.000000000 1.000000000 1.000000000",
        ]
    )


def build_fll() -> str:
    lines = [
        "Engine: TriWhirlBalanceReference",
        "description: Structural normalized upright-balance parity baseline; not validated for closed-loop stability",
    ]

    for name in ("theta_error", "theta_dot", "wheel_velocity"):
        append_input(lines, name)

    lines.extend(
        [
            "OutputVariable: target_velocity",
            "  enabled: true",
            "  range: -1.000000000 1.000000000",
            "  lock-range: true",
            "  aggregation: none",
            "  defuzzifier: WeightedAverage TakagiSugeno",
            "  default: nan",
            "  lock-previous: false",
        ]
    )

    for level in range(-6, 7):
        lines.append(
            f"  term: {output_term(level)} Constant {level / 6.0:.9f}"
        )

    lines.extend(
        [
            "RuleBlock: balance",
            "  enabled: true",
            "  conjunction: AlgebraicProduct",
            "  disjunction: none",
            "  implication: none",
            "  activation: General",
        ]
    )

    # At each linguistic center, this neutral structural fixture implements:
    #   u_norm = -(theta_error_norm + theta_dot_norm + wheel_velocity_norm) / 3
    # It exists only to prove inference parity. It is not a tuned controller.
    for i, theta in enumerate(TERMS):
        for j, rate in enumerate(TERMS):
            for k, wheel in enumerate(TERMS):
                level = 6 - (i + j + k)
                lines.append(
                    "  rule: if theta_error is "
                    f"{theta} and theta_dot is {rate} and wheel_velocity is {wheel} "
                    f"then target_velocity is {output_term(level)}"
                )

    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        help="write the generated FLL to this path instead of stdout",
    )
    args = parser.parse_args()

    content = build_fll()
    if args.output is None:
        print(content, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content, encoding="utf-8")


if __name__ == "__main__":
    main()
