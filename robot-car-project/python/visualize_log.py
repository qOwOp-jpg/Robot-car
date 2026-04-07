import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def parse_sections(text: str):
    meta = {}
    dashboard = []
    section = None

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue

        if line == "BEGIN_META":
            section = "meta"
            continue
        if line == "END_META":
            section = None
            continue
        if line == "BEGIN_DASHBOARD":
            section = "dashboard"
            continue
        if line == "END_DASHBOARD":
            section = None
            continue
        if line in ("NO_DATA",):
            raise ValueError("No telemetry data in log file")
        if line in ("RESET_DONE",):
            continue

        if section == "meta":
            parts = [p.strip() for p in line.split(",", 1)]
            if len(parts) == 2:
                meta[parts[0]] = parts[1]
            continue

        if section == "dashboard":
            if line.startswith("idx,"):
                continue
            parts = [p.strip() for p in line.split(",")]

            # idx,time_s,state,state_name,error,left_pwm,right_pwm,P,I,D
            if len(parts) == 10:
                dashboard.append(
                    {
                        "idx": int(parts[0]),
                        "time_s": float(parts[1]),
                        "state": int(parts[2]),
                        "state_name": parts[3],
                        "error": float(parts[4]),
                        "left_pwm": int(parts[5]),
                        "right_pwm": int(parts[6]),
                        "P": float(parts[7]),
                        "I": float(parts[8]),
                        "D": float(parts[9]),
                    }
                )
                continue

    return meta, dashboard


def write_csv(path: Path, rows, fieldnames):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def plot_dashboard(dashboard, outdir: Path):
    if not dashboard:
        raise ValueError("Dashboard section parsed as empty. Check logfile format.")

    t = [r["time_s"] for r in dashboard]
    err = [r["error"] for r in dashboard]
    left = [r["left_pwm"] for r in dashboard]
    right = [r["right_pwm"] for r in dashboard]
    p = [r["P"] for r in dashboard]
    i = [r["I"] for r in dashboard]
    d = [r["D"] for r in dashboard]

    plt.figure(figsize=(10, 4.5))
    plt.plot(t, err)
    plt.xlabel("Time (s)")
    plt.ylabel("Error")
    plt.title("Dashboard - Raw Error")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(outdir / "dashboard_error.png", dpi=150)
    plt.close()

    plt.figure(figsize=(10, 4.5))
    plt.plot(t, left, label="Left PWM")
    plt.plot(t, right, label="Right PWM")
    plt.xlabel("Time (s)")
    plt.ylabel("PWM")
    plt.title("Dashboard - Motor Power (Left vs Right)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "dashboard_motors.png", dpi=150)
    plt.close()

    plt.figure(figsize=(10, 4.5))
    plt.plot(t, p, label="P")
    plt.plot(t, i, label="I")
    plt.plot(t, d, label="D")
    plt.xlabel("Time (s)")
    plt.ylabel("Contribution")
    plt.title("Dashboard - PID Terms")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "dashboard_pid.png", dpi=150)
    plt.close()


def smooth_series(values, alpha=0.30):
    if not values:
        return []
    out = [values[0]]
    for v in values[1:]:
        out.append((1.0 - alpha) * out[-1] + alpha * v)
    return out


def integrate_dashboard_path(dashboard):
    if not dashboard:
        return []

    pts = [(0.0, 0.0)]
    x = 0.0
    y = 0.0
    theta = 0.0

    lefts = smooth_series([r["left_pwm"] / 255.0 for r in dashboard], alpha=0.30)
    rights = smooth_series([r["right_pwm"] / 255.0 for r in dashboard], alpha=0.30)
    errs = smooth_series([r["error"] for r in dashboard], alpha=0.22)

    for i, row in enumerate(dashboard):
        if i == 0:
            dt = dashboard[1]["time_s"] - dashboard[0]["time_s"] if len(dashboard) > 1 else 0.2
        else:
            dt = dashboard[i]["time_s"] - dashboard[i - 1]["time_s"]
        if dt <= 0:
            dt = 0.2

        left = lefts[i]
        right = rights[i]
        err = errs[i]
        state = row["state_name"].upper()

        forward = 0.5 * (left + right)
        diff = right - left

        if abs(diff) < 0.10 and abs(err) < 0.18:
            turn_rate = 0.28 * err + 0.55 * diff
        else:
            turn_rate = 1.45 * diff + 0.42 * err

        if state != "FOLLOW":
            forward *= 0.78
            turn_rate *= 1.08

        if state in ("OBSTACLE", "LOST", "ALIGN_LINE", "FIND_LINE"):
            turn_rate *= 1.08

        speed_scale = 1.10
        theta += turn_rate * dt
        x += math.cos(theta) * forward * dt * speed_scale
        y += math.sin(theta) * forward * dt * speed_scale
        pts.append((x, y))

    return pts


def plot_map(dashboard, outdir: Path):
    dash_pts = integrate_dashboard_path(dashboard)

    plt.figure(figsize=(7, 7))
    has_any = False

    if dash_pts:
        xs = [p[0] for p in dash_pts]
        ys = [p[1] for p in dash_pts]
        plt.plot(xs, ys, label="Dashboard-based rough path")
        plt.scatter([xs[0]], [ys[0]], marker="o", label="Start")
        plt.scatter([xs[-1]], [ys[-1]], marker="x", label="End")
        has_any = True

    plt.xlabel("Relative X")
    plt.ylabel("Relative Y")
    plt.title("Cartographer - Approximate Map")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    if has_any:
        plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "cartographer_path.png", dpi=150)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile", help="Raw serial log text file")
    args = parser.parse_args()

    log_path = Path(args.logfile)
    outdir = log_path.parent

    text = log_path.read_text(encoding="utf-8", errors="ignore")
    meta, dashboard = parse_sections(text)

    if not dashboard:
        raise ValueError(
            "Dashboard parsed as empty. Your logfile uses the new 10-column format, so use this fixed script instead of the old robot_car_v3.py."
        )

    write_csv(
        outdir / "dashboard.csv",
        dashboard,
        ["idx", "time_s", "state", "state_name", "error", "left_pwm", "right_pwm", "P", "I", "D"],
    )

    with (outdir / "meta.txt").open("w", encoding="utf-8") as f:
        for k, v in meta.items():
            f.write(f"{k}={v}\n")

    plot_dashboard(dashboard, outdir)
    plot_map(dashboard, outdir)

    print("Saved:")
    print(outdir / "dashboard.csv")
    print(outdir / "dashboard_error.png")
    print(outdir / "dashboard_motors.png")
    print(outdir / "dashboard_pid.png")
    print(outdir / "cartographer_path.png")
    print(outdir / "meta.txt")


if __name__ == "__main__":
    main()
