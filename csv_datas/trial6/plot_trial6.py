import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_joint_data(file_number, start_time=None, end_time=None):
    """
    Plots joint data from a CSV file.
    Updated to handle new column names with '_1' suffixes and different torque variables.
    """
    # Use the current directory as the base path for flexibility
    base_path = "."
    csv_path = os.path.join(base_path, f"cdsl_data_{file_number}.csv")

    try:
        # Define the new list of required columns
        required_cols = {
            "joint_pos_desired_1", "joint_pos_1", "joint_vel_desired_1",
            "joint_vel_filtered_1", "input torque_1", "h term torque_1", "current_time"
        }

        # Read the CSV file
        df = pd.read_csv(csv_path)

        # Strip whitespace that might be in the column names
        df.columns = [col.strip() for col in df.columns]

        # Check if the required columns are present
        if not required_cols.issubset(df.columns):
            # If headers are missing or incorrect, assume a headerless CSV with 7 columns
            print("Warning: CSV file does not contain the expected headers. Reading data without headers.")
            df = pd.read_csv(csv_path, header=None)
            df = df.iloc[:, :7].copy()
            # Assign the new column names
            df.columns = [
                "joint_pos_desired_1", "joint_pos_1", "joint_vel_desired_1",
                "joint_vel_filtered_1", "input torque_1", "h term torque_1", "current_time"
            ]

    except Exception as e:
        raise RuntimeError(f"Cannot read or process the CSV file: {e}")

    # Convert all columns to numeric type (coerce errors to NaN)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # Drop rows with NaN values in any of the required columns
    df = df.dropna(subset=list(required_cols))

    # Filter the data by time if start_time or end_time is provided
    if start_time is not None:
        df = df[df["current_time"] >= start_time]
    if end_time is not None:
        df = df[df["current_time"] <= end_time]

    if df.empty:
        raise RuntimeError("No data is available in the selected time range.")

    # Create 3 subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # Plot 1: Desired Joint Velocity vs. Filtered Joint Velocity
    axs[0].plot(df["current_time"].to_numpy(), df["joint_vel_desired_1"].to_numpy(), label="joint_vel_desired_1", color="tab:blue")
    axs[0].plot(df["current_time"].to_numpy(), df["joint_vel_filtered_1"].to_numpy(), label="joint_vel_filtered_1", color="tab:orange")
    axs[0].set_ylabel("joint velocity")
    axs[0].set_title("Joint Velocity Desired vs Filtered")
    axs[0].grid(True)
    axs[0].legend()

    # Plot 2: Joint Position vs. Desired Joint Position
    axs[1].plot(df["current_time"].to_numpy(), df["joint_pos_1"].to_numpy(), label="joint_pos_1", color="tab:green")
    axs[1].plot(df["current_time"].to_numpy(), df["joint_pos_desired_1"].to_numpy(), label="joint_pos_desired_1", color="tab:red")
    axs[1].set_ylabel("joint position")
    axs[1].set_title("Joint Position vs Desired Joint Position")
    axs[1].grid(True)
    axs[1].legend()

    # Plot 3: Torque Components vs. Time (Modified)
    axs[2].plot(df["current_time"].to_numpy(), df["input torque_1"].to_numpy(), label="input torque_1", color="tab:purple", linewidth=2.5)
    axs[2].plot(df["current_time"].to_numpy(), df["h term torque_1"].to_numpy(), label="h term torque_1", color="tab:blue", linestyle=':', linewidth=4.5)
    axs[2].set_xlabel("time")
    axs[2].set_ylabel("torque")
    axs[2].set_title("Torque Components vs Time")
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    
    # === CHANGED PART ===
    # Instead of saving the plot, display it on the screen.
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        file_number_arg = sys.argv[1]
        try:
            plot_joint_data(file_number_arg)
        except (RuntimeError, FileNotFoundError) as e:
            print(f"Error: {e}")
    else:
        print("Usage: python3 your_script_name.py <file_number>")