import pandas as pd
import matplotlib.pyplot as plt

# Read CSV data from string
df = pd.read_csv("../mujoco_data/data.csv")

# Define the types and their corresponding names
type_names = {"acc": "Acceleration", "vel": "Velocity", "force": "Force"}

# Create three separate figures for acceleration, velocity, and force
for type_key, type_name in type_names.items():
    # Create a figure with 7 subplots (one for each joint), sharing the x-axis
    fig, axes = plt.subplots(7, 1, figsize=(8, 14), sharex=True)

    for i in range(7):
        # Plot the data for each joint
        axes[i].plot(df["time"], df[f"joint_{type_key}_{i}"], label=f"Joint {i}")
        axes[i].set_title(f"Joint {i}")
        axes[i].set_ylabel(type_name)
        axes[i].grid(True)

    # Set the x-label only on the bottom subplot
    axes[6].set_xlabel("Time (s)")

    # Add a super title for the figure
    fig.suptitle(f"Joint {type_name} over Time", fontsize=16)

    # Adjust the layout to prevent overlapping
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    # Save the figure to a PNG file
    fig.savefig(f"joint_{type_name.lower()}_over_time.png", dpi=300)

# Show all figures (optional, can be removed if only saving is desired)
plt.show()
