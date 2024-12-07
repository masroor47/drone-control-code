import re
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

def parse_sensor_line(line):
    temp_pressure_pattern = r'I \((\d+)\) Sensors: Temp: (\d+\.\d+)°C, Pressure: (\d+\.\d+) hPa'
    temp_pressure_match = re.search(temp_pressure_pattern, line)
    accel_pattern = r'I \((\d+)\) Sensors: Raw: Accel: X=(-?\d+) Y=(-?\d+) Z=(-?\d+) \| Gyro: X=(-?\d+) Y=(-?\d+) Z=(-?\d+)'
    raw_match = re.search(accel_pattern, line)
    if raw_match:
        print(raw_match)
        return {
            'timestamp': int(raw_match.group(1)),
            'r_accel_x': int(raw_match.group(2)),
            'r_accel_y': int(raw_match.group(3)),
            'r_accel_z': int(raw_match.group(4)),
            'r_gyro_x': int(raw_match.group(5)),
            'r_gyro_y': int(raw_match.group(6)),
            'r_gyro_z': int(raw_match.group(7)),
        }
    elif temp_pressure_match:
        return {
            'timestamp': int(temp_pressure_match.group(1)),
            'temperature': float(temp_pressure_match.group(2)),
            'pressure': float(temp_pressure_match.group(3)),
        }
    return None

# def process_imu_file(filename):
#     data = []
#     with open(filename, 'r') as f:
#         for line in f:
#             parsed = parse_imu_line(line)
#             if parsed:
#                 data.append(parsed)
    
#     return pd.DataFrame(data)
def process_sensor_file(filename):
    data_by_timestamp = {}
    
    with open(filename, 'r') as f:
        for line in f:
            parsed = parse_sensor_line(line)
            if parsed:
                timestamp = parsed['timestamp']
                if timestamp not in data_by_timestamp:
                    data_by_timestamp[timestamp] = {}
                data_by_timestamp[timestamp].update(parsed)
                print(data_by_timestamp[timestamp])
    
    # Convert to DataFrame
    df = pd.DataFrame(list(data_by_timestamp.values()))
    
    # Sort and reorder columns
    if not df.empty:
        df = df.sort_values('timestamp')
        column_order = ['timestamp', 
                       'r_accel_x', 'r_accel_y', 'r_accel_z',
                       'r_gyro_x', 'r_gyro_y', 'r_gyro_z',
                       'temperature', 'pressure']
        existing_columns = [col for col in column_order if col in df.columns]
        df = df[existing_columns]
    
    return df

def plot_sensor_data(df):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))

    ax1.plot(df['timestamp'], df['r_accel_x'], label='X')
    ax1.plot(df['timestamp'], df['r_accel_y'], label='Y')
    ax1.plot(df['timestamp'], df['r_accel_z'], label='Z')
    ax1.set_title('Accelerometer Data')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('Acceleration')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(df['timestamp'], df['r_gyro_x'], label='X')
    ax2.plot(df['timestamp'], df['r_gyro_y'], label='Y')
    ax2.plot(df['timestamp'], df['r_gyro_z'], label='Z')
    ax2.set_title('Gyroscope Data')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Angular Velocity')
    ax2.legend()
    ax2.grid(True)

    # Plot pressure, and altitude data on the same plot with different y-axes
    df.dropna(subset=['pressure'], inplace=True)
    ax3.plot(df['timestamp'], df['pressure'], label='Pressure', color='tab:blue')
    ax3.set_title('Pressure and Altitude Data')
    ax3.set_xlabel('Time (ms)')
    ax3.set_ylabel('Pressure (hPa)', color='tab:blue')
    ax3.tick_params(axis='y', labelcolor='tab:blue')
    ax3.legend(loc='upper left')

    # Create a second y-axis for altitude
    ax3_alt = ax3.twinx()
    # remove nan values from pressure
    altitude = 44330 * (1.0 - (df['pressure'] / 1013.25) ** 0.1903)
    ax3_alt.plot(df['timestamp'], altitude, label='Altitude', color='tab:red')
    ax3_alt.set_ylabel('Altitude (m)', color='tab:red')
    ax3_alt.tick_params(axis='y', labelcolor='tab:red')
    ax3_alt.legend(loc='upper right')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    df = process_sensor_file('./data/sensor_logs.txt')
    print(df.head(5))
    
    # Basic statistics
    print("\nAccelerometer Statistics:")
    print(df[['r_accel_x', 'r_accel_y', 'r_accel_z']].describe())
    print("\nGyroscope Statistics:")
    print(df[['r_gyro_x', 'r_gyro_y', 'r_gyro_z']].describe())
    print("\nTemperature and Pressure Statistics:")
    print(df[['temperature', 'pressure']].describe())
    
    # Plot the data
    plot_sensor_data(df)
    
    # Optionally save to CSV for further analysis
    # df.to_csv('imu_data_processed.csv', index=False)