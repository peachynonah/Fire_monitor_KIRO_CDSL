import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_joint_data(file_number, start_time=None, end_time=None):
    """
    Plots joint data from a CSV file.
    Updated to display 4 separate plots.
    """
    base_path = "."
    csv_path = os.path.join(base_path, f"cdsl_data_{file_number}.csv")

    try:
        required_cols = {
            "joint_pos_desired_1", "joint_pos_1", "joint_vel_desired_1",
            "joint_vel_filtered_1", "input torque_1_w.DOB[Nm]", 
            "DOB Torque Term[Nm]", "current_time"
        }

        new_column_names = [
            "joint_pos_desired_1", "joint_pos_1", "joint_vel_desired_1",
            "joint_vel_filtered_1", "input torque_1_w.DOB[Nm]", 
            "DOB Torque Term[Nm]", "current_time"
        ]

        df = pd.read_csv(csv_path)
        df.columns = [col.strip() for col in df.columns]

        if not required_cols.issubset(df.columns):
            print("Warning: CSV file does not contain the expected headers. Reading data without headers.")
            df = pd.read_csv(csv_path, header=None)
            df = df.iloc[:, :7].copy()
            df.columns = new_column_names

    except Exception as e:
        raise RuntimeError(f"Cannot read or process the CSV file: {e}")

    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=list(required_cols))

    if start_time is not None:
        df = df[df["current_time"] >= start_time]
    if end_time is not None:
        df = df[df["current_time"] <= end_time]

    if df.empty:
        raise RuntimeError("No data is available in the selected time range.")

    fig, axs = plt.subplots(4, 1, figsize=(10, 15), sharex=True)

    # Plot 1
    axs[0].plot(df["current_time"].to_numpy(), df["joint_vel_desired_1"].to_numpy(),
                label="joint_vel_desired_1", color="tab:blue")
    axs[0].plot(df["current_time"].to_numpy(), df["joint_vel_filtered_1"].to_numpy(),
                label="joint_vel_filtered_1", color="tab:orange")
    axs[0].set_ylabel("joint velocity (rad/s)")
    axs[0].set_title("Joint Velocity Desired vs Filtered")
    axs[0].grid(True)
    axs[0].legend()

    # Plot 2
    axs[1].plot(df["current_time"].to_numpy(), df["joint_pos_1"].to_numpy(),
                label="joint_pos_1", color="tab:green")
    axs[1].plot(df["current_time"].to_numpy(), df["joint_pos_desired_1"].to_numpy(),
                label="joint_pos_desired_1", color="tab:red")
    axs[1].set_ylabel("joint position (rad)")
    axs[1].set_title("Joint Position vs Desired Joint Position")
    axs[1].grid(True)
    axs[1].legend()

    # Plot 3
    axs[2].plot(df["current_time"].to_numpy(), df["input torque_1_w.DOB[Nm]"].to_numpy(),
                label="input torque_1_w.DOB[Nm]", color="tab:purple", linewidth=2.5)
    axs[2].set_ylabel("torque (Nm)")
    axs[2].set_title("Input Torque (with DOB) vs Time")
    axs[2].grid(True)
    axs[2].legend()

    # Plot 4: (input torque - DOB term) and DOB term together
    time = df["current_time"].to_numpy()
    diff = (df["input torque_1_w.DOB[Nm]"] + df["DOB Torque Term[Nm]"]).to_numpy()

    axs[3].plot(time, diff,
                label="input torque_1_w.DOB[Nm] + DOB Torque Term[Nm]",
                color="tab:cyan", linewidth=2.5)
    axs[3].plot(time, df["DOB Torque Term[Nm]"].to_numpy(),
                label="DOB Torque Term[Nm]",
                color="#008384", linewidth=2.5)
    axs[3].set_xlabel("time (s)")
    axs[3].set_ylabel("torque (Nm)")
    axs[3].set_title("Input?DOB Term & DOB Term vs Time")
    axs[3].grid(True)
    axs[3].legend()

    plt.tight_layout()
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
