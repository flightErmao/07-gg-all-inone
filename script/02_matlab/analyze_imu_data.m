%% IMU数据频谱分析脚本
% 作者: 自动生成
% 日期: 2024
% 功能: 读取IMU数据并进行FFT频谱分析

clear; clc; close all;

%% 1. 读取CSV数据文件
fprintf('正在读取IMU数据文件...\n');

% 文件路径
csv_file = '../01_python/01_mlog_parse/out/mlog_msg_0_Minifly_Sensor_IMU.csv';

% 检查文件是否存在
if ~exist(csv_file, 'file')
    error('CSV文件不存在: %s', csv_file);
end

% 读取CSV文件
try
    data = readtable(csv_file);
    fprintf('成功读取数据，共 %d 行数据\n', height(data));
catch ME
    error('读取CSV文件失败: %s', ME.message);
end

%% 2. 提取数据列
fprintf('正在提取数据列...\n');

% 提取时间戳
timestamp = data.timestamp;

% 提取原始加速度数据 (m/s²)
acc_raw_x = data.acc_raw_0_;
acc_raw_y = data.acc_raw_1_;
acc_raw_z = data.acc_raw_2_;

% 提取原始陀螺仪数据 (rad/s)
gyro_raw_x = data.gyro_raw_0_;
gyro_raw_y = data.gyro_raw_1_;
gyro_raw_z = data.gyro_raw_2_;

% 提取滤波后加速度数据
acc_filter_x = data.acc_filter_0_;
acc_filter_y = data.acc_filter_1_;
acc_filter_z = data.acc_filter_2_;

% 提取滤波后陀螺仪数据
gyro_filter_x = data.gyro_filter_0_;
gyro_filter_y = data.gyro_filter_1_;
gyro_filter_z = data.gyro_filter_2_;

% 提取时间间隔
delta_ts = data.delta_ts;

% 计算采样频率
fs = 1 / mean(delta_ts); % 平均采样频率
fprintf('平均采样频率: %.2f Hz\n', fs);

%% 3. 数据预处理
fprintf('正在进行数据预处理...\n');

% 去除NaN值
valid_idx = ~isnan(timestamp) & ~isnan(acc_raw_x) & ~isnan(gyro_raw_x);
timestamp = timestamp(valid_idx);
acc_raw_x = acc_raw_x(valid_idx);
acc_raw_y = acc_raw_y(valid_idx);
acc_raw_z = acc_raw_z(valid_idx);
gyro_raw_x = gyro_raw_x(valid_idx);
gyro_raw_y = gyro_raw_y(valid_idx);
gyro_raw_z = gyro_raw_z(valid_idx);
acc_filter_x = acc_filter_x(valid_idx);
acc_filter_y = acc_filter_y(valid_idx);
acc_filter_z = acc_filter_z(valid_idx);
gyro_filter_x = gyro_filter_x(valid_idx);
gyro_filter_y = gyro_filter_y(valid_idx);
gyro_filter_z = gyro_filter_z(valid_idx);

% 计算时间向量（秒）
time_vec = (timestamp - timestamp(1)) / 1e6; % 假设时间戳单位为微秒

fprintf('有效数据点数: %d\n', length(time_vec));

%% 4. 绘制时域信号
fprintf('正在绘制时域信号...\n');

figure('Name', 'IMU时域信号', 'Position', [100, 100, 1200, 800]);

% 加速度时域图
subplot(3,2,1);
plot(time_vec, acc_raw_x, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, acc_filter_x, 'b-', 'LineWidth', 1);
title('X轴加速度');
xlabel('时间 (s)');
ylabel('加速度 (m/s²)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,2);
plot(time_vec, acc_raw_y, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, acc_filter_y, 'b-', 'LineWidth', 1);
title('Y轴加速度');
xlabel('时间 (s)');
ylabel('加速度 (m/s²)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,3);
plot(time_vec, acc_raw_z, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, acc_filter_z, 'b-', 'LineWidth', 1);
title('Z轴加速度');
xlabel('时间 (s)');
ylabel('加速度 (m/s²)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

% 陀螺仪时域图
subplot(3,2,4);
plot(time_vec, gyro_raw_x, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, gyro_filter_x, 'b-', 'LineWidth', 1);
title('X轴陀螺仪');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,5);
plot(time_vec, gyro_raw_y, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, gyro_filter_y, 'b-', 'LineWidth', 1);
title('Y轴陀螺仪');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,6);
plot(time_vec, gyro_raw_z, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, gyro_filter_z, 'b-', 'LineWidth', 1);
title('Z轴陀螺仪');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

%% 6. 计算并绘制频谱图
fprintf('正在计算频谱分析...\n');

% 加速度频谱分析
figure('Name', '加速度频谱分析', 'Position', [200, 200, 1200, 800]);

% X轴加速度频谱
subplot(3,2,1);
[f_acc_x_raw, Pxx_acc_x_raw] = compute_fft_spectrum(acc_raw_x, fs);
[f_acc_x_filt, Pxx_acc_x_filt] = compute_fft_spectrum(acc_filter_x, fs);
semilogx(f_acc_x_raw, Pxx_acc_x_raw, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_acc_x_filt, Pxx_acc_x_filt, 'b-', 'LineWidth', 1);
title('X轴加速度频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,2);
[f_acc_y_raw, Pxx_acc_y_raw] = compute_fft_spectrum(acc_raw_y, fs);
[f_acc_y_filt, Pxx_acc_y_filt] = compute_fft_spectrum(acc_filter_y, fs);
semilogx(f_acc_y_raw, Pxx_acc_y_raw, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_acc_y_filt, Pxx_acc_y_filt, 'b-', 'LineWidth', 1);
title('Y轴加速度频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,3);
[f_acc_z_raw, Pxx_acc_z_raw] = compute_fft_spectrum(acc_raw_z, fs);
[f_acc_z_filt, Pxx_acc_z_filt] = compute_fft_spectrum(acc_filter_z, fs);
semilogx(f_acc_z_raw, Pxx_acc_z_raw, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_acc_z_filt, Pxx_acc_z_filt, 'b-', 'LineWidth', 1);
title('Z轴加速度频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

% 陀螺仪频谱分析
figure('Name', '陀螺仪频谱分析', 'Position', [300, 300, 1200, 800]);

subplot(3,2,1);
[f_gyro_x_raw, Pxx_gyro_x_raw] = compute_fft_spectrum(gyro_raw_x, fs);
[f_gyro_x_filt, Pxx_gyro_x_filt] = compute_fft_spectrum(gyro_filter_x, fs);
semilogx(f_gyro_x_raw, Pxx_gyro_x_raw, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_gyro_x_filt, Pxx_gyro_x_filt, 'b-', 'LineWidth', 1);
title('X轴陀螺仪频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,2);
[f_gyro_y_raw, Pxx_gyro_y_raw] = compute_fft_spectrum(gyro_raw_y, fs);
[f_gyro_y_filt, Pxx_gyro_y_filt] = compute_fft_spectrum(gyro_filter_y, fs);
semilogx(f_gyro_y_raw, Pxx_gyro_y_raw, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_gyro_y_filt, Pxx_gyro_y_filt, 'b-', 'LineWidth', 1);
title('Y轴陀螺仪频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,3);
[f_gyro_z_raw, Pxx_gyro_z_raw] = compute_fft_spectrum(gyro_raw_z, fs);
[f_gyro_z_filt, Pxx_gyro_z_filt] = compute_fft_spectrum(gyro_filter_z, fs);
semilogx(f_gyro_z_raw, Pxx_gyro_z_raw, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_gyro_z_filt, Pxx_gyro_z_filt, 'b-', 'LineWidth', 1);
title('Z轴陀螺仪频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;

%% 7. 对比分析图
figure('Name', '原始vs滤波对比分析', 'Position', [400, 400, 1200, 600]);

% 加速度对比
subplot(2,3,1);
plot(f_acc_x_raw, Pxx_acc_x_raw, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_acc_x_filt, Pxx_acc_x_filt, 'b-', 'LineWidth', 1.5);
title('X轴加速度频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

subplot(2,3,2);
plot(f_acc_y_raw, Pxx_acc_y_raw, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_acc_y_filt, Pxx_acc_y_filt, 'b-', 'LineWidth', 1.5);
title('Y轴加速度频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

subplot(2,3,3);
plot(f_acc_z_raw, Pxx_acc_z_raw, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_acc_z_filt, Pxx_acc_z_filt, 'b-', 'LineWidth', 1.5);
title('Z轴加速度频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

% 陀螺仪对比
subplot(2,3,4);
plot(f_gyro_x_raw, Pxx_gyro_x_raw, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_gyro_x_filt, Pxx_gyro_x_filt, 'b-', 'LineWidth', 1.5);
title('X轴陀螺仪频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

subplot(2,3,5);
plot(f_gyro_y_raw, Pxx_gyro_y_raw, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_gyro_y_filt, Pxx_gyro_y_filt, 'b-', 'LineWidth', 1.5);
title('Y轴陀螺仪频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

subplot(2,3,6);
plot(f_gyro_z_raw, Pxx_gyro_z_raw, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_gyro_z_filt, Pxx_gyro_z_filt, 'b-', 'LineWidth', 1.5);
title('Z轴陀螺仪频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('原始', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

%% 8. 统计分析
fprintf('正在进行统计分析...\n');

% 计算统计信息
stats = struct();

% 加速度统计
stats.acc_raw_x = struct('mean', mean(acc_raw_x), 'std', std(acc_raw_x), 'rms', rms(acc_raw_x));
stats.acc_raw_y = struct('mean', mean(acc_raw_y), 'std', std(acc_raw_y), 'rms', rms(acc_raw_y));
stats.acc_raw_z = struct('mean', mean(acc_raw_z), 'std', std(acc_raw_z), 'rms', rms(acc_raw_z));

stats.acc_filter_x = struct('mean', mean(acc_filter_x), 'std', std(acc_filter_x), 'rms', rms(acc_filter_x));
stats.acc_filter_y = struct('mean', mean(acc_filter_y), 'std', std(acc_filter_y), 'rms', rms(acc_filter_y));
stats.acc_filter_z = struct('mean', mean(acc_filter_z), 'std', std(acc_filter_z), 'rms', rms(acc_filter_z));

% 陀螺仪统计
stats.gyro_raw_x = struct('mean', mean(gyro_raw_x), 'std', std(gyro_raw_x), 'rms', rms(gyro_raw_x));
stats.gyro_raw_y = struct('mean', mean(gyro_raw_y), 'std', std(gyro_raw_y), 'rms', rms(gyro_raw_y));
stats.gyro_raw_z = struct('mean', mean(gyro_raw_z), 'std', std(gyro_raw_z), 'rms', rms(gyro_raw_z));

stats.gyro_filter_x = struct('mean', mean(gyro_filter_x), 'std', std(gyro_filter_x), 'rms', rms(gyro_filter_x));
stats.gyro_filter_y = struct('mean', mean(gyro_filter_y), 'std', std(gyro_filter_y), 'rms', rms(gyro_filter_y));
stats.gyro_filter_z = struct('mean', mean(gyro_filter_z), 'std', std(gyro_filter_z), 'rms', rms(gyro_filter_z));

% 显示统计结果
fprintf('\n=== 统计分析结果 ===\n');
fprintf('采样频率: %.2f Hz\n', fs);
fprintf('数据长度: %.2f 秒\n', time_vec(end));
fprintf('数据点数: %d\n', length(time_vec));

fprintf('\n--- 加速度统计 (m/s²) ---\n');
fprintf('原始数据 - X轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_raw_x.mean, stats.acc_raw_x.std, stats.acc_raw_x.rms);
fprintf('原始数据 - Y轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_raw_y.mean, stats.acc_raw_y.std, stats.acc_raw_y.rms);
fprintf('原始数据 - Z轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_raw_z.mean, stats.acc_raw_z.std, stats.acc_raw_z.rms);

fprintf('滤波数据 - X轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_filter_x.mean, stats.acc_filter_x.std, stats.acc_filter_x.rms);
fprintf('滤波数据 - Y轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_filter_y.mean, stats.acc_filter_y.std, stats.acc_filter_y.rms);
fprintf('滤波数据 - Z轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_filter_z.mean, stats.acc_filter_z.std, stats.acc_filter_z.rms);

fprintf('\n--- 陀螺仪统计 (rad/s) ---\n');
fprintf('原始数据 - X轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_raw_x.mean, stats.gyro_raw_x.std, stats.gyro_raw_x.rms);
fprintf('原始数据 - Y轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_raw_y.mean, stats.gyro_raw_y.std, stats.gyro_raw_y.rms);
fprintf('原始数据 - Z轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_raw_z.mean, stats.gyro_raw_z.std, stats.gyro_raw_z.rms);

fprintf('滤波数据 - X轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_filter_x.mean, stats.gyro_filter_x.std, stats.gyro_filter_x.rms);
fprintf('滤波数据 - Y轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_filter_y.mean, stats.gyro_filter_y.std, stats.gyro_filter_y.rms);
fprintf('滤波数据 - Z轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_filter_z.mean, stats.gyro_filter_z.std, stats.gyro_filter_z.rms);

%% 9. 保存结果
fprintf('\n正在保存分析结果...\n');

% 保存工作空间变量
save('imu_analysis_results.mat', 'data', 'stats', 'fs', 'time_vec', ...
     'acc_raw_x', 'acc_raw_y', 'acc_raw_z', 'gyro_raw_x', 'gyro_raw_y', 'gyro_raw_z', ...
     'acc_filter_x', 'acc_filter_y', 'acc_filter_z', 'gyro_filter_x', 'gyro_filter_y', 'gyro_filter_z');

fprintf('分析完成！结果已保存到 imu_analysis_results.mat\n');
fprintf('生成了以下图表：\n');
fprintf('1. IMU时域信号图\n');
fprintf('2. 加速度频谱分析图\n');
fprintf('3. 陀螺仪频谱分析图\n');
fprintf('4. 原始vs滤波对比分析图\n');

%% 10. FFT频谱分析函数
function [f, Pxx] = compute_fft_spectrum(signal, fs)
    % 计算信号的FFT频谱
    N = length(signal);
    
    % 使用Welch方法计算功率谱密度
    [Pxx, f] = pwelch(signal, [], [], [], fs);
    
    % 转换为dB
    Pxx_db = 10*log10(Pxx);
end
