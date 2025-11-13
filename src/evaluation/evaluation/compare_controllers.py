#!/usr/bin/env python3

import csv
import sys
import os
from tabulate import tabulate


def load_csv(csv_path):
    """Load controller comparison data from CSV"""
    if not os.path.exists(csv_path):
        print(f"❌ Error: CSV file not found: {csv_path}")
        return None

    data = []
    with open(csv_path, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            data.append(row)

    return data


def analyze_controllers(data):
    """Analyze and compare controller performance"""
    if not data:
        print("No data to analyze")
        return

    print("\n" + "="*80)
    print("🏁 CONTROLLER PERFORMANCE COMPARISON")
    print("="*80 + "\n")

    # Prepare table data
    table_data = []
    headers = ["Controller", "Total Lap Time", "Score"]

    # Collect sector names from first row
    first_row = data[0]
    sector_names = []
    for key in first_row.keys():
        if key.endswith('_time') and key != 'total_lap_time':
            sector_name = key.replace('_time', '')
            sector_names.append(sector_name)

    # Add sector columns to headers
    for sector in sector_names:
        headers.append(f"{sector} Time")

    # Build table rows
    for row in data:
        row_data = [
            row['controller_name'],
            f"{float(row['total_lap_time']):.3f}s",
            f"{float(row['overall_score']):.2f}"
        ]

        # Add sector times
        for sector in sector_names:
            row_data.append(f"{float(row[f'{sector}_time']):.3f}s")

        table_data.append(row_data)

    # Sort by total lap time (fastest first)
    table_data.sort(key=lambda x: float(x[1].replace('s', '')))

    # Print main table
    print(tabulate(table_data, headers=headers, tablefmt="grid"))

    # Find best performances
    print("\n" + "="*80)
    print("🏆 BEST PERFORMANCES")
    print("="*80 + "\n")

    # Best overall lap time
    best_lap = min(data, key=lambda x: float(x['total_lap_time']))
    print(f"🥇 Fastest Lap: {best_lap['controller_name']} - {float(best_lap['total_lap_time']):.3f}s")
    print(f"   Score: {float(best_lap['overall_score']):.2f}")

    # Best sector times
    print("\n📊 Sector Analysis:")
    for sector in sector_names:
        best_sector = min(data, key=lambda x: float(x[f'{sector}_time']))
        avg_speed = float(best_sector[f'{sector}_avg_speed'])
        print(f"  {sector}: {best_sector['controller_name']} - {float(best_sector[f'{sector}_time']):.3f}s (avg speed: {avg_speed:.2f} m/s)")

    # Performance comparison (relative to best)
    print("\n" + "="*80)
    print("📈 PERFORMANCE GAP (relative to fastest)")
    print("="*80 + "\n")

    best_time = float(best_lap['total_lap_time'])
    gap_table = []
    for row in data:
        lap_time = float(row['total_lap_time'])
        gap = lap_time - best_time
        gap_pct = (gap / best_time) * 100
        gap_table.append([
            row['controller_name'],
            f"{lap_time:.3f}s",
            f"+{gap:.3f}s" if gap > 0 else "---",
            f"+{gap_pct:.2f}%" if gap > 0 else "---"
        ])

    gap_table.sort(key=lambda x: float(x[1].replace('s', '')))
    print(tabulate(gap_table, headers=["Controller", "Lap Time", "Gap", "Gap %"], tablefmt="grid"))

    print("\n" + "="*80 + "\n")


def main(args=None):
    # Default CSV path
    default_csv = '/home/joon/joon_sim_ws/evaluation_results/controller_comparison.csv'

    # ✅ Support for obstacle map comparison
    default_obs_csv = '/home/joon/joon_sim_ws/evaluation_results/controller_comparison_obs.csv'

    # Allow custom CSV path from command line
    if args and len(args) > 0:
        csv_path = args[0]
    else:
        # Interactive selection if both files exist
        import os
        if os.path.exists(default_csv) and os.path.exists(default_obs_csv):
            print("\n📊 Multiple comparison files found:")
            print("  1. Normal map comparison")
            print("  2. Obstacle map comparison")
            choice = input("Select which to analyze (1/2, or press Enter for normal): ").strip()
            csv_path = default_obs_csv if choice == '2' else default_csv
        elif os.path.exists(default_obs_csv):
            csv_path = default_obs_csv
        else:
            csv_path = default_csv

    print(f"📂 Loading data from: {csv_path}")

    data = load_csv(csv_path)
    if data:
        analyze_controllers(data)
    else:
        print("❌ Failed to load data")
        sys.exit(1)


if __name__ == '__main__':
    main(sys.argv[1:])
