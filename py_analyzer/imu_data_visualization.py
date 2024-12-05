import re
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

def parse_imu_line(line):
    # Regular expression to match the IMU data format
    # raw_pattern = r'Raw: Accel: X=(-?\d+) Y=(-?\d+) Z=(-?\d+) \| Gyro: X=(-?\d+) Y=(-?\d+) Z=(-?\d+)' 
    # filtered_pattern = r'Filtered: Accel: X=(-?\d+) Y=(-?\d+) Z=(-?\d+) \| Gyro: X=(-?\d+) Y=(-?\d+) Z=(-?\d+)'
    temp_pressure_pattern = r'Temp: (\d+\.\d+)°C, Pressure: (\d+\.\d+) hPa'
    # Temp: 25.91°C, Pressure: 1005.9 hPa
    # pattern = r'Accel: X=(-?\d+) Y=(-?\d+) Z=(-?\d+) \| Gyro: X=(-?\d+) Y=(-?\d+) Z=(-?\d+)'
    # raw_match = re.search(raw_pattern, line)
    # filtered_match = re.search(filtered_pattern, line)
    temp_pressure_match = re.search(temp_pressure_pattern, line)
    
    # if raw_match or filtered_match:
    if temp_pressure_match:
        return {
            # 'r_accel_x': int(raw_match.group(1)),
            # 'r_accel_y': int(raw_match.group(2)),
            # 'r_accel_z': int(raw_match.group(3)),
            # 'r_gyro_x': int(raw_match.group(4)),
            # 'r_gyro_y': int(raw_match.group(5)),
            # 'r_gyro_z': int(raw_match.group(6)),
            # 'f_accel_x': int(filtered_match.group(1)),
            # 'f_accel_y': int(filtered_match.group(2)),
            # 'f_accel_z': int(filtered_match.group(3)),
            # 'f_gyro_x': int(filtered_match.group(4)),
            # 'f_gyro_y': int(filtered_match.group(5)),
            # 'f_gyro_z': int(filtered_match.group(6)),
            'temperature': float(temp_pressure_match.group(1)),
            'pressure': float(temp_pressure_match.group(2)),
            'timestamp': datetime.now()  # Add timestamp for time-series plotting
        }
    # if not raw_match:
    #     print("Raw data not found in line: ", line)
    # if not filtered_match:
    #     print("Filtered data not found in line: ", line)
    return None

def process_imu_file(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            parsed = parse_imu_line(line)
            if parsed:
                data.append(parsed)
    
    return pd.DataFrame(data)

def plot_imu_data(df):
    # Create a figure with subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # ax1.plot(df['timestamp'], df['temperature'], label='Temperature')
    # ax1.set_title('Temperature Data')
    # ax1.set_xlabel('Time')
    # ax1.set_ylabel('Temperature (°C)')
    # ax1.legend()

    ax1.plot(df['timestamp'], df['pressure'], label='Pressure')
    ax1.set_title('Pressure Data')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Pressure (hPa)')
    ax1.legend()

    # float altitude = 44330 * (1.0f - pow(pressure / 1013.25f, 0.1903f));
    altitude = 44330 * (1.0 - (df['pressure'] / 1013.25) ** 0.1903)
    ax2.plot(df['timestamp'], altitude, label='Altitude')
    ax2.set_title('Altitude Data')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Altitude (m)')
    ax2.legend()



    
    # Plot accelerometer data
    # ax1.plot(df['timestamp'], df['r_accel_x'], label='X')
    # ax1.plot(df['timestamp'], df['r_accel_y'], label='Y')
    # ax1.plot(df['timestamp'], df['r_accel_z'], label='Z')
    # ax1.set_title('Accelerometer Data')
    # ax1.set_xlabel('Time')
    # ax1.set_ylabel('Acceleration')
    # ax1.legend()
    # ax1.grid(True)
    
    # Plot gyroscope data
    # ax2.plot(df['timestamp'], df['r_gyro_x'], label='X')
    # ax2.plot(df['timestamp'], df['r_gyro_y'], label='Y')
    # ax2.plot(df['timestamp'], df['r_gyro_z'], label='Z')
    # ax2.set_title('Gyroscope Data')
    # ax2.set_xlabel('Time')
    # ax2.set_ylabel('Angular Velocity')
    # ax2.legend()
    # ax2.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Read and process the data
    df = process_imu_file('imu_data_temp_press.txt')
    print(df.head(5))
    
    # Basic statistics
    # print("\nAccelerometer Statistics:")
    # print(df[['accel_x', 'accel_y', 'accel_z']].describe())
    # print("\nGyroscope Statistics:")
    # print(df[['gyro_x', 'gyro_y', 'gyro_z']].describe())
    
    # Plot the data
    plot_imu_data(df)
    
    # Optionally save to CSV for further analysis
    df.to_csv('imu_data_processed.csv', index=False)