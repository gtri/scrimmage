#!/usr/bin/env python3
"""
Compare a mission's current summary.csv against a stored benchmark.

Usage:
    compare_benchmarks.py <mission_name> <baseline.csv> <current.csv>

Prints a GitHub-flavored markdown table to stdout.
Exit code 0 always — comparison is informational only.
"""

import csv
import sys


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {r["team_id"]: r for r in rows}


def fmt(val):
    """Format a CSV value: strip trailing zeros from floats, leave non-numeric as-is."""
    try:
        f = float(val)
        # Use up to 4 significant figures, strip trailing zeros
        return f"{f:.4g}"
    except (ValueError, TypeError):
        return val


def compare(mission_name, baseline_path, current_path):
    baseline = load(baseline_path)
    current = load(current_path)

    if not baseline and not current:
        print(f"## {mission_name}")
        print()
        print("_(both baseline and current have no data rows)_")
        print()
        return

    all_teams = sorted(set(baseline) | set(current), key=int)
    sample = (list(baseline.values()) or list(current.values()))[0]
    all_fields = [h for h in sample if h != "team_id"]

    print(f"## {mission_name}")
    print()
    print("| team | metric | baseline | current | delta |")
    print("|------|--------|----------|---------|-------|")

    for team in all_teams:
        b_row = baseline.get(team, {})
        c_row = current.get(team, {})
        for field in all_fields:
            b_val = b_row.get(field, "N/A")
            c_val = c_row.get(field, "N/A")
            try:
                delta = float(c_val) - float(b_val)
                delta_str = f"{delta:+.4g}"
                flag = "✅" if delta == 0 else ("⬆️" if delta > 0 else "⬇️")
            except (ValueError, TypeError):
                delta_str = "—"
                flag = "❓"
            print(f"| {team} | {field} | {fmt(b_val)} | {fmt(c_val)} | {delta_str} {flag} |")

    print()


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <mission_name> <baseline.csv> <current.csv>",
              file=sys.stderr)
        sys.exit(1)

    mission_name = sys.argv[1]
    baseline_path = sys.argv[2]
    current_path = sys.argv[3]
    compare(mission_name, baseline_path, current_path)


if __name__ == "__main__":
    main()
