import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os

def load_and_filter(csv_path):
    df = pd.read_csv(csv_path)
    df.sort_values('timestamp', inplace=True)
    return df

def plot_comparison(cmd_df, fbk_df, field, motor_id, save_dir=None):
    # 反馈数据插值对齐命令时间
    fbk_interp = fbk_df.set_index('timestamp').reindex(cmd_df['timestamp'], method='nearest').reset_index()

    plt.figure(figsize=(10, 5))
    plt.plot(cmd_df['timestamp'], cmd_df[field], label=f'Command {field}', linestyle='--')
    plt.plot(fbk_interp['timestamp'], fbk_interp[field], label=f'Feedback {field}', alpha=0.7)
    plt.xlabel('Timestamp')
    plt.ylabel(field)
    plt.title(f'Motor {motor_id} - {field} Command vs Feedback')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, f'motor_{motor_id}_{field}.png')
        plt.savefig(save_path)
    else:
        plt.show()
    plt.close()

def analyze_motor(cmd_df_all, fbk_df_all, motor_id, save_dir=None):
    cmd_df = cmd_df_all[cmd_df_all['motor_id'] == motor_id]
    fbk_df = fbk_df_all[fbk_df_all['motor_id'] == motor_id]
    if cmd_df.empty or fbk_df.empty:
        print(f"[!] 电机 {motor_id} 无数据，跳过。")
        return
    for field in ['position', 'speed', 'torque']:
        plot_comparison(cmd_df, fbk_df, field, motor_id, save_dir)

def main():
    parser = argparse.ArgumentParser(description="Compare motor command and feedback logs.")
    parser.add_argument('--cmd_csv', type=str, required=True, help="Path to command_log_*.csv")
    parser.add_argument('--fbk_csv', type=str, required=True, help="Path to feedback_log_*.csv")
    parser.add_argument('--motor_id', type=int, help="Optional: Motor ID to analyze")
    parser.add_argument('--save_dir', type=str, default=None, help="Optional: Directory to save figures (instead of showing them)")
    args = parser.parse_args()

    cmd_df = load_and_filter(args.cmd_csv)
    fbk_df = load_and_filter(args.fbk_csv)

    if args.motor_id is not None:
        analyze_motor(cmd_df, fbk_df, args.motor_id, args.save_dir)
    else:
        motor_ids = sorted(set(cmd_df['motor_id']).intersection(set(fbk_df['motor_id'])))
        print(f"[i] 自动检测到 {len(motor_ids)} 个电机: {motor_ids}")
        for motor_id in motor_ids:
            analyze_motor(cmd_df, fbk_df, motor_id, args.save_dir)

if __name__ == '__main__':
    main()
