import os
import pandas as pd
import matplotlib.pyplot as plt
from fpdf import FPDF

# Set the logs directory
LOGS_DIR = "logs"

def process_logs():
    # Iterate through all folders in the logs directory
    for folder in os.listdir(LOGS_DIR):
        log_folder_path = os.path.join(LOGS_DIR, folder)
        
        # Skip if it's not a directory
        if not os.path.isdir(log_folder_path):
            continue

        # Check if the report already exists
        report_path = os.path.join(log_folder_path, "report.pdf")
        if os.path.exists(report_path):
            print(f"Report already exists for {folder}. Skipping...")
            continue

        # Paths to CSV files
        error_file = os.path.join(log_folder_path, "aruco_pose_errors.csv")
        detection_file = os.path.join(log_folder_path, "detected_markers.csv")

        if not os.path.exists(error_file) or not os.path.exists(detection_file):
            print(f"Missing data files in {folder}. Skipping...")
            continue

        # Analyze data and generate the report
        analyze_and_generate_report(log_folder_path, error_file, detection_file, report_path)


def analyze_and_generate_report(folder_path, error_file, detection_file, report_path):
    # Load data
    errors_df = pd.read_csv(error_file)
    detections_df = pd.read_csv(detection_file)

    # Error Analysis
    error_stats = {}
    if not errors_df.empty:
        for marker_id in errors_df["marker_id"].unique():
            marker_data = errors_df[errors_df["marker_id"] == marker_id]
            mean = marker_data[["dx", "dy", "dz", "rx", "ry", "rz"]].mean()
            std = marker_data[["dx", "dy", "dz", "rx", "ry", "rz"]].std()
            error_stats[marker_id] = {"mean": mean, "std": std}

        # Overall stats
        overall_mean = errors_df[["dx", "dy", "dz", "rx", "ry", "rz"]].mean()
        overall_std = errors_df[["dx", "dy", "dz", "rx", "ry", "rz"]].std()
    else:
        overall_mean, overall_std = None, None

    # Detection Analysis
    detections_df["num_markers"] = detections_df["detected_marker_ids"].apply(
        lambda x: len(x.split(";")) if pd.notna(x) else 0
    )
    detection_mean = detections_df["num_markers"].mean()
    detection_std = detections_df["num_markers"].std()

    # Plot Pose Error Data
    if not errors_df.empty:
        plt.figure(figsize=(10, 6))
        for marker_id in errors_df["marker_id"].unique():
            marker_data = errors_df[errors_df["marker_id"] == marker_id]
            timestamps = marker_data["timestamp"]
            errors = (marker_data["dx"]**2 + marker_data["dy"]**2 + marker_data["dz"]**2)**0.5
            plt.plot(timestamps, errors, label=f"Marker {marker_id}", marker="o")

        plt.xlabel("Time (s)")
        plt.ylabel("Pose Error (m)")
        plt.title("Pose Error Over Time")
        plt.legend()
        error_plot_path = os.path.join(folder_path, "error_plot.png")
        plt.savefig(error_plot_path)
        plt.close()
    else:
        error_plot_path = None

    # Plot Detection Data
    if not detections_df.empty:
        plt.figure(figsize=(10, 6))
        timestamps = detections_df["timestamp"]
        detection_counts = detections_df["num_markers"]

        plt.plot(timestamps, detection_counts, marker="o", color="green", label="Number of Detected Markers")
        plt.xlabel("Time (s)")
        plt.ylabel("Number of Markers")
        plt.title("Number of Detected Markers Over Time")
        plt.legend()
        detection_plot_path = os.path.join(folder_path, "detection_plot.png")
        plt.savefig(detection_plot_path)
        plt.close()
    else:
        detection_plot_path = None

    # Generate PDF Report
    pdf = FPDF()
    pdf.add_page()
    pdf.set_font("Arial", size=12)

    # Title
    pdf.cell(200, 10, txt=f"Test Report: {os.path.basename(folder_path)}", ln=True, align="C")

    # Error Stats
    pdf.cell(200, 10, txt="Pose Error Statistics:", ln=True)
    if error_stats:
        for marker_id, stats in error_stats.items():
            pdf.set_font("Arial", size=10)
            pdf.cell(200, 10, txt=f"Marker {marker_id}: ", ln=True)
            pdf.cell(200, 10, txt=f"     Mean={stats['mean'].values}", ln=True)
            pdf.cell(200, 10, txt=f"     Std={stats['std'].values}", ln=True)
    else:
        pdf.cell(200, 10, txt="No markers detected.", ln=True)

    # Overall stats
    if overall_mean is not None:
        pdf.cell(200, 10, txt=f"Overall Mean: {overall_mean.values}, Overall Std: {overall_std.values}", ln=True)

    # Detection Stats
    pdf.cell(200, 10, txt="Detection Statistics:", ln=True)
    pdf.cell(200, 10, txt=f"Mean Detected Markers: {detection_mean}, Std: {detection_std}", ln=True)

    # Add Pose Error Plot
    if error_plot_path:
        pdf.add_page()
        pdf.image(error_plot_path, x=10, y=10, w=180)

    # Add Detection Plot
    if detection_plot_path:
        pdf.add_page()
        pdf.image(detection_plot_path, x=10, y=10, w=180)

    # Save the PDF
    pdf.output(report_path)
    print(f"Report created: {report_path}")


if __name__ == "__main__":
    process_logs()
