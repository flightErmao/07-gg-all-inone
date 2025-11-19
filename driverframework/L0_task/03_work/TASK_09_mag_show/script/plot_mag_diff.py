#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磁力计差值数据可视化脚本
读取清洗后的CSV数据，绘制多个差值对比图表
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

def load_data(csv_path):
    """加载清洗后的CSV数据"""
    df = pd.read_csv(csv_path)
    return df

def calculate_stats(data):
    """计算统计信息（平均值和方差）"""
    stats = {}
    for mag_id in data['磁力计编号'].unique():
        mag_data = data[data['磁力计编号'] == mag_id]
        stats[mag_id] = {
            'DIFF_X': {'mean': mag_data['DIFF_X'].mean(), 'std': mag_data['DIFF_X'].std()},
            'DIFF_Y': {'mean': mag_data['DIFF_Y'].mean(), 'std': mag_data['DIFF_Y'].std()},
            'DIFF_Z': {'mean': mag_data['DIFF_Z'].mean(), 'std': mag_data['DIFF_Z'].std()},
            'DIFF_Magnitude': {'mean': mag_data['DIFF_Magnitude'].mean(), 'std': mag_data['DIFF_Magnitude'].std()},
        }
    return stats

def plot_axis_diff_by_magnetometer(data, stats, output_dir):
    """绘制每个磁力计各轴差值图（X、Y、Z用不同颜色）"""
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    
    colors = {'X': '#FF6B6B', 'Y': '#4ECDC4', 'Z': '#45B7D1'}
    axes_labels = {'X': 'DIFF_X', 'Y': 'DIFF_Y', 'Z': 'DIFF_Z'}
    
    for idx, mag_id in enumerate(sorted(data['磁力计编号'].unique())):
        ax = axes[idx]
        mag_data = data[data['磁力计编号'] == mag_id].sort_values('测试次数')
        test_numbers = mag_data['测试次数'].values
        
        for axis_name, col_name in axes_labels.items():
            values = mag_data[col_name].values
            ax.plot(test_numbers, values, marker='o', linewidth=2, markersize=6, 
                   label=f'{axis_name}轴', color=colors[axis_name])
            
            # 添加平均值和方差虚线
            mean_val = stats[mag_id][col_name]['mean']
            std_val = stats[mag_id][col_name]['std']
            ax.axhline(y=mean_val, color=colors[axis_name], linestyle='--', 
                     alpha=0.5, linewidth=1.5, label=f'{axis_name}均值: {mean_val:.2f}')
            ax.axhline(y=mean_val + std_val, color=colors[axis_name], linestyle=':', 
                     alpha=0.3, linewidth=1)
            ax.axhline(y=mean_val - std_val, color=colors[axis_name], linestyle=':', 
                     alpha=0.3, linewidth=1)
        
        ax.set_xlabel('测试次数', fontsize=12)
        ax.set_ylabel('差值 (uT)', fontsize=12)
        ax.set_title(f'{mag_id}号磁力计 - 各轴差值对比', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=9)
        ax.set_xticks(test_numbers)
    
    plt.tight_layout()
    output_path = output_dir / '01_各轴差值对比_按磁力计.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"已保存: {output_path}")
    plt.close()

def plot_magnitude_diff_by_magnetometer(data, stats, output_dir):
    """绘制模长差值图（不同磁力计用不同颜色）"""
    fig, ax = plt.subplots(figsize=(12, 7))
    
    colors = {17846: '#E74C3C', 17741: '#3498DB'}
    
    for mag_id in sorted(data['磁力计编号'].unique()):
        mag_data = data[data['磁力计编号'] == mag_id].sort_values('测试次数')
        test_numbers = mag_data['测试次数'].values
        magnitude_diff = mag_data['DIFF_Magnitude'].values
        
        ax.plot(test_numbers, magnitude_diff, marker='o', linewidth=2.5, 
               markersize=8, label=f'{mag_id}号磁力计', color=colors[mag_id])
        
        # 添加平均值和方差虚线
        mean_val = stats[mag_id]['DIFF_Magnitude']['mean']
        std_val = stats[mag_id]['DIFF_Magnitude']['std']
        ax.axhline(y=mean_val, color=colors[mag_id], linestyle='--', 
                  alpha=0.6, linewidth=2, 
                  label=f'{mag_id}号均值: {mean_val:.2f}±{std_val:.2f}')
        ax.fill_between([1, 10], mean_val - std_val, mean_val + std_val, 
                       color=colors[mag_id], alpha=0.1)
    
    ax.set_xlabel('测试次数', fontsize=13)
    ax.set_ylabel('模长差值 (uT)', fontsize=13)
    ax.set_title('模长差值对比 - 不同磁力计', fontsize=15, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=11)
    ax.set_xticks(range(1, 11))
    
    plt.tight_layout()
    output_path = output_dir / '02_模长差值对比_按磁力计.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"已保存: {output_path}")
    plt.close()

def plot_axis_diff_combined(data, stats, output_dir):
    """绘制所有磁力计各轴差值综合对比图"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    
    axes_config = [
        ('DIFF_X', 'X轴差值', '#FF6B6B'),
        ('DIFF_Y', 'Y轴差值', '#4ECDC4'),
        ('DIFF_Z', 'Z轴差值', '#45B7D1')
    ]
    
    colors = {17846: '#E74C3C', 17741: '#3498DB'}
    
    for idx, (col_name, title, default_color) in enumerate(axes_config):
        ax = axes[idx]
        
        for mag_id in sorted(data['磁力计编号'].unique()):
            mag_data = data[data['磁力计编号'] == mag_id].sort_values('测试次数')
            test_numbers = mag_data['测试次数'].values
            values = mag_data[col_name].values
            
            ax.plot(test_numbers, values, marker='o', linewidth=2, markersize=6,
                   label=f'{mag_id}号磁力计', color=colors[mag_id])
            
            # 添加平均值和方差虚线
            mean_val = stats[mag_id][col_name]['mean']
            std_val = stats[mag_id][col_name]['std']
            ax.axhline(y=mean_val, color=colors[mag_id], linestyle='--', 
                      alpha=0.5, linewidth=1.5)
        
        ax.set_xlabel('测试次数', fontsize=11)
        ax.set_ylabel(f'{title} (uT)', fontsize=11)
        ax.set_title(f'{title}对比', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=9)
        ax.set_xticks(range(1, 11))
    
    plt.tight_layout()
    output_path = output_dir / '03_各轴差值综合对比.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"已保存: {output_path}")
    plt.close()

def plot_statistics_summary(data, stats, output_dir):
    """绘制统计信息汇总图（平均值和方差）"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    axes_config = [
        ('DIFF_X', 'X轴差值', 0, 0),
        ('DIFF_Y', 'Y轴差值', 0, 1),
        ('DIFF_Z', 'Z轴差值', 1, 0),
        ('DIFF_Magnitude', '模长差值', 1, 1)
    ]
    
    colors = {17846: '#E74C3C', 17741: '#3498DB'}
    mag_ids = sorted(data['磁力计编号'].unique())
    
    for col_name, title, row, col in axes_config:
        ax = axes[row, col]
        
        means = [stats[mag_id][col_name]['mean'] for mag_id in mag_ids]
        stds = [stats[mag_id][col_name]['std'] for mag_id in mag_ids]
        x_pos = np.arange(len(mag_ids))
        
        bars = ax.bar(x_pos, means, yerr=stds, capsize=5, 
                     color=[colors[mag_id] for mag_id in mag_ids],
                     alpha=0.7, edgecolor='black', linewidth=1.5)
        
        # 添加数值标签
        for i, (mean, std) in enumerate(zip(means, stds)):
            ax.text(i, mean + std + 2, f'{mean:.2f}±{std:.2f}', 
                   ha='center', va='bottom', fontsize=10, fontweight='bold')
        
        ax.set_xlabel('磁力计编号', fontsize=11)
        ax.set_ylabel(f'{title} (uT)', fontsize=11)
        ax.set_title(f'{title}统计信息', fontsize=12, fontweight='bold')
        ax.set_xticks(x_pos)
        ax.set_xticklabels([f'{mag_id}号' for mag_id in mag_ids])
        ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    output_path = output_dir / '04_统计信息汇总.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"已保存: {output_path}")
    plt.close()

def print_statistics(stats):
    """打印统计信息到控制台"""
    print("\n" + "="*60)
    print("统计信息汇总")
    print("="*60)
    
    for mag_id in sorted(stats.keys()):
        print(f"\n{mag_id}号磁力计:")
        print(f"  X轴差值:  均值={stats[mag_id]['DIFF_X']['mean']:8.2f} uT, "
              f"标准差={stats[mag_id]['DIFF_X']['std']:6.2f} uT")
        print(f"  Y轴差值:  均值={stats[mag_id]['DIFF_Y']['mean']:8.2f} uT, "
              f"标准差={stats[mag_id]['DIFF_Y']['std']:6.2f} uT")
        print(f"  Z轴差值:  均值={stats[mag_id]['DIFF_Z']['mean']:8.2f} uT, "
              f"标准差={stats[mag_id]['DIFF_Z']['std']:6.2f} uT")
        print(f"  模长差值: 均值={stats[mag_id]['DIFF_Magnitude']['mean']:8.2f} uT, "
              f"标准差={stats[mag_id]['DIFF_Magnitude']['std']:6.2f} uT")
    
    print("="*60 + "\n")

def main():
    # 设置路径
    script_dir = Path(__file__).parent
    data_dir = script_dir.parent / 'data'
    output_dir = script_dir.parent / 'data' / 'plots'
    output_dir.mkdir(exist_ok=True)
    
    csv_path = data_dir / '01-mag_ci_cleaned.csv'
    
    if not csv_path.exists():
        print(f"错误: 找不到数据文件 {csv_path}")
        return
    
    # 加载数据
    print(f"正在加载数据: {csv_path}")
    data = load_data(csv_path)
    
    # 计算统计信息
    print("正在计算统计信息...")
    stats = calculate_stats(data)
    
    # 打印统计信息
    print_statistics(stats)
    
    # 绘制图表
    print("正在生成图表...")
    plot_axis_diff_by_magnetometer(data, stats, output_dir)
    plot_magnitude_diff_by_magnetometer(data, stats, output_dir)
    plot_axis_diff_combined(data, stats, output_dir)
    plot_statistics_summary(data, stats, output_dir)
    
    print(f"\n所有图表已保存到: {output_dir}")

if __name__ == '__main__':
    main()

