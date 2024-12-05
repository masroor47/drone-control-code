import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
from scipy import stats

def parse_imu_line(line):
    """Parse a single line of IMU data containing either raw or filtered measurements."""
    timestamp_match = re.search(r'I \((\d+)\)', line)
    if not timestamp_match:
        return None
    
    timestamp = int(timestamp_match.group(1))
    
    raw_pattern = r'Raw: Accel: X=(-?\d+) Y=(-?\d+) Z=(-?\d+) \| Gyro: X=(-?\d+) Y=(-?\d+) Z=(-?\d+)'
    filtered_pattern = r'Filtered: Accel: X=(-?\d+\.\d+) Y=(-?\d+\.\d+) Z=(-?\d+\.\d+) \| Gyro: X=(-?\d+\.\d+) Y=(-?\d+\.\d+) Z=(-?\d+\.\d+)'
    
    raw_match = re.search(raw_pattern, line)
    filtered_match = re.search(filtered_pattern, line)
    
    data = {}
    if raw_match:
        data = {
            'timestamp': timestamp,
            'type': 'raw',
            'accel_x': float(raw_match.group(1)),
            'accel_y': float(raw_match.group(2)),
            'accel_z': float(raw_match.group(3)),
            'gyro_x': float(raw_match.group(4)),
            'gyro_y': float(raw_match.group(5)),
            'gyro_z': float(raw_match.group(6))
        }
    elif filtered_match:
        data = {
            'timestamp': timestamp,
            'type': 'filtered',
            'accel_x': float(filtered_match.group(1)),
            'accel_y': float(filtered_match.group(2)),
            'accel_z': float(filtered_match.group(3)),
            'gyro_x': float(filtered_match.group(4)),
            'gyro_y': float(filtered_match.group(5)),
            'gyro_z': float(filtered_match.group(6))
        }
    return data if data else None

def process_imu_file(filename):
    """Process the IMU data file and return a synchronized DataFrame with both raw and filtered data."""
    data_dict = {}  # Use timestamp as key to match raw and filtered data
    
    with open(filename, 'r') as f:
        for line in f:
            parsed = parse_imu_line(line)
            if parsed:
                timestamp = parsed['timestamp']
                data_type = parsed['type']
                
                if timestamp not in data_dict:
                    data_dict[timestamp] = {'timestamp': timestamp}
                
                # Add prefix to measurements based on type
                prefix = 'raw_' if data_type == 'raw' else 'filtered_'
                for key in ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']:
                    data_dict[timestamp][prefix + key] = parsed[key]
    
    # Convert to DataFrame and handle missing values
    df = pd.DataFrame(list(data_dict.values()))
    df['timestamp'] = pd.to_datetime(df['timestamp'], unit='ms')
    return df

def calculate_differences(df):
    """Calculate differences between raw and filtered data."""
    diff_columns = {
        'accel_x_diff': ('raw_accel_x', 'filtered_accel_x'),
        'accel_y_diff': ('raw_accel_y', 'filtered_accel_y'),
        'accel_z_diff': ('raw_accel_z', 'filtered_accel_z'),
        'gyro_x_diff': ('raw_gyro_x', 'filtered_gyro_x'),
        'gyro_y_diff': ('raw_gyro_y', 'filtered_gyro_y'),
        'gyro_z_diff': ('raw_gyro_z', 'filtered_gyro_z')
    }
    
    for new_col, (raw_col, filtered_col) in diff_columns.items():
        df[new_col] = df[raw_col] - df[filtered_col]
        
    return df

def plot_comparison(df, sensor_type='accel'):
    """Plot raw vs filtered data with differences."""
    axes = ['x', 'y', 'z']
    fig, axs = plt.subplots(len(axes), 2, figsize=(15, 12))
    
    for i, axis in enumerate(axes):
        raw_col = f'raw_{sensor_type}_{axis}'
        filtered_col = f'filtered_{sensor_type}_{axis}'
        diff_col = f'{sensor_type}_{axis}_diff'
        
        # Plot raw vs filtered
        axs[i, 0].plot(df['timestamp'], df[raw_col], label='Raw', alpha=0.6)
        axs[i, 0].plot(df['timestamp'], df[filtered_col], label='Filtered', alpha=0.6)
        axs[i, 0].set_title(f'{sensor_type.capitalize()} {axis.upper()} - Raw vs Filtered')
        axs[i, 0].set_xlabel('Time')
        axs[i, 0].set_ylabel('Value')
        axs[i, 0].legend()
        axs[i, 0].grid(True)
        
        # Plot difference
        axs[i, 1].plot(df['timestamp'], df[diff_col], label='Difference', color='red')
        axs[i, 1].set_title(f'{sensor_type.capitalize()} {axis.upper()} - Difference')
        axs[i, 1].set_xlabel('Time')
        axs[i, 1].set_ylabel('Difference')
        axs[i, 1].grid(True)
        
    plt.tight_layout()
    return fig

def analyze_filtering_effectiveness(df):
    """Analyze the effectiveness of the filtering."""
    analysis = {}
    
    for sensor in ['accel', 'gyro']:
        for axis in ['x', 'y', 'z']:
            raw_col = f'raw_{sensor}_{axis}'
            filtered_col = f'filtered_{sensor}_{axis}'
            
            # Calculate metrics
            raw_std = df[raw_col].std()
            filtered_std = df[filtered_col].std()
            noise_reduction = (raw_std - filtered_std) / raw_std * 100
            correlation = df[raw_col].corr(df[filtered_col])
            
            # Store results
            analysis[f'{sensor}_{axis}'] = {
                'raw_std': raw_std,
                'filtered_std': filtered_std,
                'noise_reduction_percent': noise_reduction,
                'correlation': correlation
            }
    
    return pd.DataFrame(analysis).round(4)

def main():
    # Read and process the data
    df = process_imu_file('imu_data_calib_filter_trans_a=0.1.txt')
    
    # Calculate differences
    df = calculate_differences(df)
    
    # Plot comparisons
    accel_fig = plot_comparison(df, 'accel')
    accel_fig.suptitle('Accelerometer Data Comparison', y=1.02)
    plt.show()
    
    gyro_fig = plot_comparison(df, 'gyro')
    gyro_fig.suptitle('Gyroscope Data Comparison', y=1.02)
    plt.show()
    
    # Analyze filtering effectiveness
    effectiveness = analyze_filtering_effectiveness(df)
    print("\nFiltering Effectiveness Analysis:")
    print(effectiveness)
    
    # Save processed data
    df.to_csv('imu_data_comparison.csv', index=False)
    effectiveness.to_csv('filtering_effectiveness.csv')

if __name__ == "__main__":
    main()