# network_monitor.py
#
# A Python script to monitor and plot network bandwidth on Linux.
#
# This script can:
#   1. Let the user interactively select an active interface to monitor.
#   2. Directly monitor a specific interface passed as an argument.
#   3. Monitor bandwidth (upload and download speeds) for the chosen interface.
#   4. Log the bandwidth data to a CSV file in a specified directory.
#   5. Plot the results from a log file into a chart.
#   6. Simultaneously capture network traffic using tshark (if installed).
#
# Dependencies:
#   - tshark: The command-line utility for Wireshark.
#     (Install on Debian/Ubuntu: sudo apt-get install tshark)
#   - psutil: To get network interface information and stats.
#   - pandas: To read and manipulate the data for plotting.
#   - matplotlib: To create the plot.
#   To configure tshark to run without sudo
#   sudo usermod -aG wireshark <USER>
#   sudo setcap cap_net_raw,cap_net_admin+eip /usr/bin/dumpcap
#
# Installation:
#   pip install psutil pandas matplotlib
#
# Usage:
#
#   To save timestamped log and pcap files to a specific directory:
#   python net_monitor.py monitor -n wlp0s20f3 -d /tmp/netlogs --pcap
#
#   To plot the most recent log file from a directory:
#   python network_monitor.py plot -d /tmp/netlogs
#


 


import psutil
import time
import argparse
import csv
import os
import subprocess
import shutil
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

def get_active_interfaces():
    """
    Gets a list of all active (up) network interfaces.

    Returns:
        list: A list of names for all active interfaces.
    """
    active_interfaces = []
    stats = psutil.net_if_stats()
    for name, stat in stats.items():
        if stat.isup:
            active_interfaces.append(name)
    return active_interfaces

def select_interface_interactively():
    """
    Lists active interfaces and prompts the user to select one.

    Returns:
        str: The name of the selected interface, or None if no selection is made.
    """
    interfaces = get_active_interfaces()
    if not interfaces:
        print("Error: No active network interfaces found.")
        return None

    print("Please select an interface to monitor:")
    for i, name in enumerate(interfaces):
        print(f"  {i+1}: {name}")

    while True:
        try:
            choice = input(f"Enter number (1-{len(interfaces)}), or Ctrl+C to exit: ")
            choice_index = int(choice) - 1
            if 0 <= choice_index < len(interfaces):
                return interfaces[choice_index]
            else:
                print("Invalid number. Please try again.")
        except (ValueError, IndexError):
            print("Invalid input. Please enter a number from the list.")
        except (KeyboardInterrupt, EOFError):
            print("\nSelection cancelled.")
            return None

def monitor_bandwidth(interface, log_file, interval, pcap_file=None):
    """
    Monitors bandwidth and optionally captures packets with tshark.

    Args:
        interface (str): The name of the interface to monitor.
        log_file (str): The path to the CSV log file.
        interval (int): The time interval in seconds between measurements.
        pcap_file (str, optional): The path to save the packet capture.
    """
    if pcap_file and not shutil.which("tshark"):
        print("Error: 'tshark' is not installed or not in your PATH.")
        print("Please install it to use the packet capture feature (e.g., 'sudo apt-get install tshark').")
        pcap_file = None # Disable pcap functionality

    tshark_process = None
    if pcap_file:
        tshark_command = ['tshark', '-i', interface, '-w', pcap_file]
        try:
            tshark_process = subprocess.Popen(tshark_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print(f"Starting tshark packet capture on '{interface}'.")
        except (FileNotFoundError, PermissionError) as e:
            print(f"Error starting tshark: {e}")
            print("Continuing with bandwidth monitoring only.")
            tshark_process = None

    print(f"Monitoring bandwidth on '{interface}' every {interval}s. Press Ctrl+C to stop.")
    
    header = ['timestamp', 'bytes_in', 'bytes_out', 'rx_mbps', 'tx_mbps']
    try:
        with open(log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
    except IOError as e:
        print(f"Error: Could not write to log file '{log_file}'. {e}")
        if tshark_process: tshark_process.terminate()
        return

    try:
        last_io = psutil.net_io_counters(pernic=True)[interface]
        last_time = time.time()

        while True:
            time.sleep(interval)
            
            try:
                current_io = psutil.net_io_counters(pernic=True)[interface]
                current_time = time.time()
            except KeyError:
                print(f"\nError: Interface '{interface}' not found. It might have been disconnected.")
                break

            time_diff = current_time - last_time
            if time_diff == 0: continue

            bytes_in = current_io.bytes_recv - last_io.bytes_recv
            bytes_out = current_io.bytes_sent - last_io.bytes_sent
            rx_mbps = (bytes_in * 8) / (time_diff * 1024 * 1024)
            tx_mbps = (bytes_out * 8) / (time_diff * 1024 * 1024)
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            print(f"\r{timestamp} | Download: {rx_mbps:6.2f} Mbps | Upload: {tx_mbps:6.2f} Mbps", end="")

            with open(log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, bytes_in, bytes_out, rx_mbps, tx_mbps])

            last_io = current_io
            last_time = current_time

    except KeyboardInterrupt:
        print(f"\n\nMonitoring stopped. Data logged to '{log_file}'.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        if tshark_process:
            tshark_process.terminate()
            try:
                tshark_process.wait(timeout=5)
                print(f"Stopped tshark. Packet capture saved to '{pcap_file}'.")
            except subprocess.TimeoutExpired:
                print("tshark did not terminate gracefully, killing process.")
                tshark_process.kill()


def plot_from_log(log_file, output_file):
    """
    Plots bandwidth data from a CSV log file using pandas and matplotlib.

    Args:
        log_file (str): The path to the CSV log file.
        output_file (str): The path to save the output plot image.
    """
    try:
        df = pd.read_csv(log_file)
    except FileNotFoundError:
        print(f"Error: Log file not found at '{log_file}'")
        return
    
    if df.empty:
        print(f"Log file '{log_file}' is empty. Nothing to plot.")
        return

    df['timestamp'] = pd.to_datetime(df['timestamp'])

    try:
        plt.style.use('seaborn-v0_8-grid')
    except (IOError, OSError):
        print("Warning: 'seaborn-v0_8-grid' style not found. Falling back to 'ggplot'.")
        plt.style.use('ggplot')

    fig, ax = plt.subplots(figsize=(15, 7))

    ax.plot(df['timestamp'], df['rx_mbps'], label='Download (Mbps)', color='deepskyblue')
    ax.plot(df['timestamp'], df['tx_mbps'], label='Upload (Mbps)', color='tomato')

    peak_rx = df['rx_mbps'].max()
    peak_tx = df['tx_mbps'].max()
    avg_rx = df['rx_mbps'].mean()
    avg_tx = df['tx_mbps'].mean()

    title = (
        f"Bandwidth Over Time: {os.path.basename(log_file)}\n"
        f"Peak Download: {peak_rx:.2f} Mbps | Peak Upload: {peak_tx:.2f} Mbps\n"
        f"Avg Download: {avg_rx:.2f} Mbps | Avg Upload: {avg_tx:.2f} Mbps"
    )
    ax.set_title(title, fontsize=16)
    ax.set_xlabel('Time', fontsize=12)
    ax.set_ylabel('Bandwidth (Mbps)', fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)

    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    fig.autofmt_xdate()

    plt.tight_layout()

    try:
        plt.savefig(output_file, dpi=150)
        print(f"Plot successfully saved to '{output_file}'")
    except Exception as e:
        print(f"Error saving plot: {e}")

    plt.show()

def main():
    """Main function to parse arguments and run the script."""
    parser = argparse.ArgumentParser(
        description="Monitor and plot network bandwidth on Linux.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    subparsers = parser.add_subparsers(dest="command", required=True, help="Available commands")

    # --- Monitor command ---
    parser_monitor = subparsers.add_parser(
        "monitor",
        help="Monitor bandwidth and log to a file.",
        description="Monitors bandwidth and can simultaneously capture packets to a .pcap file."
    )
    parser_monitor.add_argument("-n", "--interface", help="Network interface name (e.g., wlp0s20f3). Skips interactive selection.")
    parser_monitor.add_argument("-d", "--dir", default=".", help="Output directory for log/pcap files. If specified, filenames will be timestamped.")
    parser_monitor.add_argument("-o", "--output", default="bandwidth.log", help="Output log file name (default: bandwidth.log). Ignored if -d is used.")
    parser_monitor.add_argument("-i", "--interval", type=int, default=1, help="Monitoring interval in seconds (default: 1).")
    parser_monitor.add_argument("--pcap", action='store_true', help="Enable packet capture. Creates a timestamped .pcap file if -d is used, or 'capture.pcap' otherwise.")

    # --- Plot command ---
    parser_plot = subparsers.add_parser(
        "plot",
        help="Plot bandwidth from a log file.",
        description="Reads a CSV log file and creates a plot. Can find the latest log in a directory."
    )
    parser_plot.add_argument("-f", "--file", help="The specific log file to plot from. Overrides automatic detection via --dir.")
    parser_plot.add_argument("-d", "--dir", help="Directory to search for the most recent .log file to plot.")
    parser_plot.add_argument("-o", "--output", default=None, help="Output plot image file name. Defaults to a name based on the log file.")

    args = parser.parse_args()

    if args.command == "monitor":
        interface_name = args.interface or select_interface_interactively()

        if interface_name:
            output_dir = args.dir
            if not os.path.isdir(output_dir):
                try:
                    os.makedirs(output_dir, exist_ok=True)
                    print(f"Created output directory: {output_dir}")
                except OSError as e:
                    print(f"Error: Could not create directory '{output_dir}'. {e}")
                    return

            if output_dir != '.':
                timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                base_name = f"{timestamp}_{interface_name.replace(':', '-')}"
                log_file_path = os.path.join(output_dir, f"{base_name}_bandwidth.log")
                pcap_file_path = os.path.join(output_dir, f"{base_name}_capture.pcap") if args.pcap else None
            else:
                log_file_path = os.path.join(output_dir, args.output)
                pcap_file_path = os.path.join(output_dir, "capture.pcap") if args.pcap else None
            
            print(f"Logging bandwidth to: {log_file_path}")
            if pcap_file_path: print(f"Capturing packets to: {pcap_file_path}")

            monitor_bandwidth(interface_name, log_file_path, args.interval, pcap_file_path)
        else:
            print("No interface selected. Exiting.")

    elif args.command == "plot":
        log_file_to_plot = args.file
        if not log_file_to_plot and args.dir:
            try:
                log_files = [f for f in os.listdir(args.dir) if f.endswith('.log')]
                if not log_files:
                    print(f"Error: No .log files found in directory '{args.dir}'.")
                    return
                latest_log_file = max([os.path.join(args.dir, f) for f in log_files], key=os.path.getmtime)
                log_file_to_plot = latest_log_file
                print(f"Found latest log file to plot: {os.path.basename(log_file_to_plot)}")
            except FileNotFoundError:
                print(f"Error: Directory not found at '{args.dir}'")
                return

        if not log_file_to_plot:
            parser.error("You must specify a log file with -f/--file or a directory with -d/--dir.")
            return

        output_file_path = args.output
        if not output_file_path:
            base_name = os.path.splitext(os.path.basename(log_file_to_plot))[0]
            plot_output_dir = os.path.dirname(log_file_to_plot) or '.'
            output_file_path = os.path.join(plot_output_dir, f"{base_name}_plot.png")

        plot_from_log(log_file_to_plot, output_file_path)

if __name__ == "__main__":
    main()


