import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_joint_data(file_number, start_time=None, end_time=None):
    """
    Plots data for 2 joints from a CSV file in a 3x2 grid.
    Updated to convert pandas Series to NumPy arrays for plotting.
    """
    # Use the current directory as the base path
    base_path = "."
    csv_path = os.path.join(base_path, f"cdsl_data_{file_number}.csv")

    try:
        # Define the list of required columns based on the new header
        new_column_names = [
            "joint1_pos_desired", "joint1_pos",
            "joint2_pos_desired", "joint2_pos",
            "joint1_vel_desired", "joint1_vel_filtered",
            "joint2_vel_desired", "joint2_vel_filtered",
            "target_torque_joint1",
            "target_torque_joint2",
            "propo_term_torque_joint1", "deriv_term_torque_joint1",
            "propo_term_torque_joint2", "deriv_term_torque_joint2",
            "current_time"
        ]
        required_cols = set(new_column_names)

        # Read the CSV file
        df = pd.read_csv(csv_path)

        # Strip whitespace from column names to prevent errors
        df.columns = [col.strip() for col in df.columns]

        # Check if the required columns are present
        if not required_cols.issubset(df.columns):
            print("Warning: CSV file does not contain the expected headers. Reading data without headers.")
            df = pd.read_csv(csv_path, header=None)
            df = df.iloc[:, :len(new_column_names)].copy()
            df.columns = new_column_names

    except Exception as e:
        raise RuntimeError(f"Cannot read or process the CSV file: {e}")

    # Convert all columns to numeric type
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # Drop rows with NaN values
    df = df.dropna(subset=list(required_cols))

    # Filter by time
    if start_time is not None:
        df = df[df["current_time"] >= start_time]
    if end_time is not None:
        df = df[df["current_time"] <= end_time]

    if df.empty:
        raise RuntimeError("No data is available in the selected time range.")

    # Create a 3x2 grid of subplots
    fig, axs = plt.subplots(3, 2, figsize=(15, 12), sharex=True)
    fig.suptitle(f'Joint Data Analysis for File {file_number}', fontsize=16)


    # --- Row 1: Position Plots ---
    # Plot 1,1: Joint 1 Position
    axs[0, 0].plot(df["current_time"].to_numpy(), df["joint1_pos_desired"].to_numpy(), label="Desired Pos (J1)", linestyle='--', linewidth = 2.5)
    axs[0, 0].plot(df["current_time"].to_numpy(), df["joint1_pos"].to_numpy(), label="Actual Pos (J1)")
    axs[0, 0].set_ylabel("Position (rad)")
    axs[0, 0].set_title("Joint 1: Position")
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    # Plot 1,2: Joint 2 Position
    axs[0, 1].plot(df["current_time"].to_numpy(), df["joint2_pos_desired"].to_numpy(), label="Desired Pos (J2)", linestyle='--', linewidth = 2.5)
    axs[0, 1].plot(df["current_time"].to_numpy(), df["joint2_pos"].to_numpy(), label="Actual Pos (J2)")
    axs[0, 1].set_ylabel("Position (rad)")
    axs[0, 1].set_title("Joint 2: Position")
    axs[0, 1].grid(True)
    axs[0, 1].legend()


    # --- Row 2: Velocity Plots ---
    # Plot 2,1: Joint 1 Velocity
    axs[1, 0].plot(df["current_time"].to_numpy(), df["joint1_vel_desired"].to_numpy(), label="Desired Vel (J1)", linestyle='--', linewidth = 2.5)
    axs[1, 0].plot(df["current_time"].to_numpy(), df["joint1_vel_filtered"].to_numpy(), label="Filtered Vel (J1)")
    axs[1, 0].set_ylabel("Velocity (rad/s)")
    axs[1, 0].set_title("Joint 1: Velocity")
    axs[1, 0].grid(True)
    axs[1, 0].legend()

    # Plot 2,2: Joint 2 Velocity
    axs[1, 1].plot(df["current_time"].to_numpy(), df["joint2_vel_desired"].to_numpy(), label="Desired Vel (J2)", linestyle='--', linewidth = 2.5)
    axs[1, 1].plot(df["current_time"].to_numpy(), df["joint2_vel_filtered"].to_numpy(), label="Filtered Vel (J2)")
    axs[1, 1].set_ylabel("Velocity (rad/s)")
    axs[1, 1].set_title("Joint 2: Velocity")
    axs[1, 1].grid(True)
    axs[1, 1].legend()


    # --- Row 3: Torque Plots ---
    # Plot 3,1: Joint 1 Torque
    axs[2, 0].plot(df["current_time"].to_numpy(), df["target_torque_joint1"].to_numpy(), label="Target Torque (J1)")
    axs[2, 0].plot(df["current_time"].to_numpy(), df["propo_term_torque_joint1"].to_numpy(), label="Proportional Term (J1)", linestyle='--', linewidth = 2.5)
    axs[2, 0].plot(df["current_time"].to_numpy(), df["deriv_term_torque_joint1"].to_numpy(), label="Derivative Term (J1)", linestyle=':', linewidth = 4.5)
    axs[2, 0].set_xlabel("Time (s)")
    axs[2, 0].set_ylabel("Torque (permil)")
    axs[2, 0].set_title("Joint 1: Torque Components")
    axs[2, 0].grid(True)
    axs[2, 0].legend()

    # Plot 3,2: Joint 2 Torque
    axs[2, 1].plot(df["current_time"].to_numpy(), df["target_torque_joint2"].to_numpy(), label="Target Torque (J2)")
    axs[2, 1].plot(df["current_time"].to_numpy(), df["propo_term_torque_joint2"].to_numpy(), label="Proportional Term (J2)", linestyle='--')
    axs[2, 1].plot(df["current_time"].to_numpy(), df["deriv_term_torque_joint2"].to_numpy(), label="Derivative Term (J2)", linestyle=':')
    axs[2, 1].set_xlabel("Time (s)")
    axs[2, 1].set_ylabel("Torque (permil)")
    axs[2, 1].set_title("Joint 2: Torque Components")
    axs[2, 1].grid(True)
    axs[2, 1].legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Display the plot
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