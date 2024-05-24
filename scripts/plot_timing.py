import argparse
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from cycler import cycler

def set_aspect_ratio(ax, ratio=1):
    x_left, x_right = ax.get_xlim()     
    y_low, y_high = ax.get_ylim()     

    x_diff = x_right - x_left
    y_diff = y_low - y_high

    ax.set_aspect(ratio * abs(x_diff / y_diff))

def plot_pie(dataframes, entity_count):
    # Plot Pi charts 
    fig_pie, axes_pie = plt.subplots(1, 2)
    pi_labels = ["Autonomy", "Control", "Motion", "Other"]
    for (device, (ax, df)) in zip(["cpu", "gpu"], zip(axes_pie, dataframes)):
        collected = df.groupby("entity_count")
        avgs = collected.mean()
        vars = collected.var()

        autonomy_time = avgs.loc[entity_count]["autonomies"]
        control_time = avgs.loc[entity_count]["controllers"]
        sensor_time = avgs.loc[entity_count]["sensors"]
        interaction_times = avgs.loc[entity_count]["interactions"]
        networks_time = avgs.loc[entity_count]["networks"]
        motion_time = avgs.loc[entity_count][device + "_motion_models"]
        total_time = avgs.loc[entity_count]["total"]  

        misc_time = total_time
        misc_time -= autonomy_time
        misc_time -= control_time
        #misc_time -= sensor_time
        #misc_time -= interaction_times
        #misc_time -= networks_time
        misc_time -= motion_time

        times = [autonomy_time, 
                 control_time,
                 motion_time,
                 misc_time]

        explode = (0, 0, 0.075, 0)
        ax.pie(times, labels=pi_labels, autopct="%.1f%%", explode=explode,
               textprops={'size': 'smaller'})
        ax.set_xlabel(device.upper())

    fig_pie.supxlabel("Agent Count: {}".format(entity_count))
    fig_pie.tight_layout()
    fig_pie.savefig(os.environ['HOME'] + "/time_pie_" + str(entity_count) + ".png")

def plot_speedup(dataframes):
    cpu_data = dataframes[0]
    gpu_data = dataframes[1]

    mean_cpu_data = cpu_data.groupby("entity_count").mean()
    mean_gpu_data = gpu_data.groupby("entity_count").mean()

    mean_total_speedup = mean_cpu_data["total"].to_numpy() / mean_gpu_data["total"].to_numpy()
    mean_motion_speedup = mean_cpu_data["cpu_motion_models"].to_numpy() / mean_gpu_data["gpu_motion_models"].to_numpy()

    entity_counts = mean_cpu_data.index.to_numpy(dtype=float)
    
    fig_speedup, ax_speedup = plt.subplots()
    plot_cycler = (cycler(color=['r', 'b']) + cycler(linestyle=["--", "-."]) + cycler(marker=["o", "s"]))
    ax_speedup.set_prop_cycle(plot_cycler)

    ax_speedup.plot(entity_counts, mean_total_speedup, label="Total Simulation", markersize=5)
    ax_speedup.plot(entity_counts, mean_motion_speedup, label="Motion Updates", markersize=5)

    ax_speedup.grid()
    ax_speedup.legend()
    ax_speedup.set_xlabel("Number of Agents")
    ax_speedup.set_ylabel("CPU Sim Time / GPU Sim Time")
    ax_speedup.set_xscale("log")
    ax_speedup.set_aspect(0.5)
    fig_speedup.tight_layout()
    fig_speedup.savefig(os.environ['HOME'] + "/speedup.png", dpi=600)


def main(): 
    #plt.rcParams.update({
    #    "text.usetex": True,
    #    "font.family": "Helvetica"
    #    })

    parser = argparse.ArgumentParser(
            usage="%(prog)s [cpu_log_dir] [gpu_log_dir]",
            description="Plot timing information for gpu and cpu comparisions"
            )

    parser.add_argument("cpu_dir")
    parser.add_argument("gpu_dir")
    args = parser.parse_args()


    fig_mean, ax_mean = plt.subplots()
    ax_mean.set_xlabel("Number of Agents")
    ax_mean.set_ylabel("Total Motion Update Time (s)")
    ax_mean.set_yscale("log")
    #ax_mean.set_xscale("log", base=2)
    ax_mean.set_xscale("log")
    ax_mean.grid()

    fig_mean_frac, ax_mean_frac = plt.subplots()
    ax_mean_frac.set_xlabel("Number of Agents")
    ax_mean_frac.set_ylabel("% of Total Sim Time")
    #ax_mean_frac.set_xscale("log", base=2)
    ax_mean_frac.set_xscale("log")
    ax_mean_frac.grid()

    fig_speedup, ax_speedup = plt.subplots()
    ax_speedup.set_xlabel("Number of Agents")
    ax_speedup.set_ylabel("GPU Speedup")
    ax_speedup.set_xscale("log")

    fig_total, ax_total = plt.subplots()
    ax_total.set_xlabel("Number of Agents")
    ax_total.set_ylabel("Time (ms)")


    dataframes = []

    plot_cycler = (cycler(color=['r', 'b']) + cycler(linestyle=["--", "-."]) + cycler(marker=["o", "s"]))

    ax_mean.set_prop_cycle(plot_cycler)
    ax_mean_frac.set_prop_cycle(plot_cycler)

    markersize = 5

    for device in ["cpu", "gpu"]:
        if device == "cpu":
            log_dir = args.cpu_dir
        elif device == "gpu":
            log_dir = args.gpu_dir

        df = pd.DataFrame(columns = ["entity_count", "cpu_motion_models", "cpu_motion_models_frac", "gpu_motion_models", "gpu_motion_models_frac"])
        for (dirpath, dirnames, filenames) in os.walk(log_dir):
            if "summary.csv" in filenames and "timer_log.csv" in filenames:
                summary_path = os.path.join(dirpath, "summary.csv")
                timer_log_path = os.path.join(dirpath, "timer_log.csv")

                summary = pd.read_csv(summary_path)
                timer_log = pd.read_csv(timer_log_path)
                total_sim_time = timer_log["total"]
                normalized_times = timer_log.div(total_sim_time[0])
                normalized_times = normalized_times.add_suffix("_frac")

                data = summary.join(timer_log)
                data = data.join(normalized_times)
                df = pd.concat([df, data], ignore_index=True)

        dataframes.append(df)
        collected = df.groupby("entity_count")
        avgs = collected.mean()
        vars = collected.var()

        motion_model = device + "_motion_models"

        entity_counts = avgs.index.to_numpy(dtype=float)
        mean_device_time = avgs[motion_model].to_numpy(dtype=float)
        std_device_time = np.sqrt(vars[motion_model].to_numpy(dtype=float))

        mean_device_time_frac = avgs[motion_model + "_frac"]
        std_device_time_frac = np.sqrt(vars[motion_model + "_frac"].to_numpy(dtype=float))

        #ax_mean.errorbar(entity_counts, mean_device_time, std_device_time, linestyle="None", marker=".", label=device)
        #ax_mean_frac.errorbar(entity_counts, mean_device_time_frac, std_device_time_frac, linestyle="None", marker=".", label=device) 

        ax_mean.plot(entity_counts, 1e-3*mean_device_time, label=device.upper(), markersize=markersize)
        ax_mean_frac.plot(entity_counts, 100*mean_device_time_frac, label=device.upper(), markersize=markersize)

        ax_mean.legend()
        ax_mean.grid()
        ax_mean.set_aspect(0.5)

        ax_mean_frac.legend()
        ax_mean_frac.grid()
        ax_mean_frac.set_aspect(0.5)


        fig_mean.tight_layout()
        fig_mean_frac.tight_layout()
        
        fig_mean.savefig(os.environ['HOME'] + "/mean_device_time.png", dpi=600)
        fig_mean_frac.savefig(os.environ['HOME'] + "/mean_frac_device_time.png", dpi=600)

    plot_pie(dataframes, entity_counts[0])
    plot_pie(dataframes, entity_counts[-1])

    plot_speedup(dataframes)

if __name__ == '__main__':
    main()
