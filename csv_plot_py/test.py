import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
df = pd.read_csv('/home/rain/Zbot/Project/Proj1205_test_libtorch/csv_plot_py/delete.csv')  # 替换为你的文件路径

# 设置图表样式
plt.figure(figsize=(15, 10))

# 创建6个子图
for i in range(1, 7):
    plt.subplot(6, 1, i)  # 3行2列，第i个子图
    
    # libtorch_out
    plt.plot(df['timestamp'], df[f'libtorch_out{i}'], 'b-', linewidth=1.5, label=f'libtorch_out{i}')
    
    # pytorch_out
    plt.plot(df['timestamp'], df[f'pytorch_out{i}'], 'r--', linewidth=1.5, label=f'pytorch_out{i}')
    
    # 设置标签和标题
    plt.xlabel('timestamp')
    plt.ylabel('Value')
    plt.title(f'Axis {i}: libtorch_out{i}  and pytorch_out{i} ')
    plt.legend()
    plt.grid(True, alpha=0.3)

# 调整布局
plt.tight_layout()

# 保存为PNG文件
output_file = 'plot_output.png'
plt.savefig(output_file, dpi=300, bbox_inches='tight')
print(f"图表已保存为: {output_file}")

# 显示图表
plt.show()
