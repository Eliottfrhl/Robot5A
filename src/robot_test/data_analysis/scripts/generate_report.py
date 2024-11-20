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

        # Paths to required files
        error_file = os.path.join(log_folder_path, "aruco_pose_errors.csv")
        detection_file = os.path.join(log_folder_path, "detected_markers.csv")
        info_file = os.path.join(log_folder_path, "test_information.txt")

        if not os.path.exists(error_file) or not os.path.exists(detection_file) or not os.path.exists(info_file):
            print(f"Missing required files in {folder}. Skipping...")
            continue

        # Analyze data and generate the report
        analyze_and_generate_report(log_folder_path, error_file, detection_file, info_file, report_path)


def analyze_and_generate_report(folder_path, error_file, detection_file, info_file, report_path):
    # Load data
    errors_df = pd.read_csv(error_file).round(4)
    detections_df = pd.read_csv(detection_file)

    # Load context information
    with open(info_file, "r") as file:
        test_info = file.read()

    # Error Analysis
    error_stats = {}
    if not errors_df.empty:
        for marker_id in errors_df["marker_id"].unique():
            marker_data = errors_df[errors_df["marker_id"] == marker_id]
            mean = marker_data[["dx", "dy", "dz", "rx", "ry", "rz"]].mean().round(4)
            std = marker_data[["dx", "dy", "dz", "rx", "ry", "rz"]].std().round(4)
            error_stats[marker_id] = {"mean": mean, "std": std}

        overall_mean = errors_df[["dx", "dy", "dz", "rx", "ry", "rz"]].mean().round(4)
        overall_std = errors_df[["dx", "dy", "dz", "rx", "ry", "rz"]].std().round(4)
    else:
        overall_mean, overall_std = None, None

    # Detection Analysis
    detections_df["num_markers"] = detections_df["detected_marker_ids"].apply(
        lambda x: len(x.split(";")) if pd.notna(x) else 0
    )
    detection_mean = detections_df["num_markers"].mean().round(4)
    detection_std = detections_df["num_markers"].std().round(4)

    # Plot 1: Pose Errors
    pose_plot_path = os.path.join(folder_path, "pose_errors.png")
    plt.figure(figsize=(10, 6))
    for marker_id in errors_df["marker_id"].unique():
        marker_data = errors_df[errors_df["marker_id"] == marker_id]
        timestamps = marker_data["timestamp"]
        errors = (marker_data["dx"]**2 + marker_data["dy"]**2 + marker_data["dz"]**2)**0.5
        plt.plot(timestamps, errors, label=f"Marker {marker_id}", marker="o")
    plt.title("Pose Errors Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Pose Error (m)")
    plt.legend()
    plt.savefig(pose_plot_path)
    plt.close()

    # Plot 2: Number of Detected Markers
    detection_plot_path = os.path.join(folder_path, "detection_plot.png")
    plt.figure(figsize=(10, 6))
    plt.plot(detections_df["timestamp"], detections_df["num_markers"], marker="o", color="blue", label="Detected Markers")
    plt.title("Number of Detected Markers Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Number of Markers")
    plt.legend()
    plt.savefig(detection_plot_path)
    plt.close()

    # Orientation Plots: rx, ry, rz
    orientation_plot_paths = {}
    for axis in ["rx", "ry", "rz"]:
        plot_path = os.path.join(folder_path, f"orientation_{axis}_errors.png")
        plt.figure(figsize=(10, 6))
        for marker_id in errors_df["marker_id"].unique():
            marker_data = errors_df[errors_df["marker_id"] == marker_id]
            plt.plot(marker_data["timestamp"], marker_data[axis], label=f"Marker {marker_id}", marker="o")
        plt.title(f"Orientation Errors ({axis.upper()}) Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel(f"Orientation Error ({axis})")
        plt.legend()
        plt.savefig(plot_path)
        plt.close()
        orientation_plot_paths[axis] = plot_path

    # Generate PDF Report
    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=15)

    # Page 1: Context Information and Plots
    pdf.add_page()
    pdf.set_font("Arial", size=12)
    pdf.multi_cell(0, 10, test_info.strip())  # Add all lines from test_information.txt
    pdf.ln(25)
    pdf.cell(200, 10, txt="Pose Errors Over Time", ln=True, align="C")
    pdf.image(pose_plot_path, x=10, y=110, w=180)
    pdf.add_page()
    pdf.cell(200, 10, txt="Number of Detected Markers Over Time", ln=True, align="C")
    pdf.image(detection_plot_path, x=10, y=50, w=180)

    # Page 2-3: Error Statistics and Detection Analysis
    pdf.add_page()
    pdf.set_font("Arial", size=14, style="B")
    pdf.cell(200, 10, txt="Error Statistics", ln=True, align="L")
    pdf.set_font("Arial", size=12)
    for marker_id, stats in error_stats.items():
        pdf.cell(200, 10, txt=f"Marker {marker_id}:", ln=True)
        pdf.cell(200, 10, txt=f"    Mean: {stats['mean'].to_dict()}", ln=True)
        pdf.cell(200, 10, txt=f"    Std: {stats['std'].to_dict()}", ln=True)
    if overall_mean is not None:
        pdf.cell(200, 10, txt="Overall Pose Errors (Mean and Std):", ln=True)
        for col in overall_mean.index:
            pdf.cell(200, 10, txt=f"    {col}: Mean={overall_mean[col]:.4f}, Std={overall_std[col]:.4f}", ln=True)
    pdf.cell(200, 10, txt="Detection Analysis:", ln=True, align="L")
    pdf.cell(200, 10, txt=f"Mean Number of Detected Markers: {detection_mean:.4f}", ln=True)
    pdf.cell(200, 10, txt=f"Standard Deviation: {detection_std:.4f}", ln=True)

    # Page 4-5: Orientation Error Plots
    for axis, path in orientation_plot_paths.items():
        pdf.add_page()
        pdf.cell(200, 10, txt=f"Orientation Errors ({axis.upper()}) Over Time", ln=True, align="C")
        pdf.image(path, x=10, y=50, w=180)

    # Save the PDF
    pdf.output(report_path)
    print(f"Report created: {report_path}")


if __name__ == "__main__":
    process_logs()
