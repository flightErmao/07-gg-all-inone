%% IMU数据频谱分析脚本
% 作者: 自动生成
% 日期: 2024
% 功能: 读取IMU数据并进行FFT频谱分析

clear; clc; close all;

%% 1. 读取CSV数据文件
fprintf('正在读取IMU数据文件...\n');

% 文件路径
csv_file = '../../01_python/01_mlog_parse/out/mlog_msg_0_Minifly_Sensor_IMU.csv';

% 显示当前工作目录和文件路径用于调试
fprintf('当前工作目录: %s\n', pwd);
fprintf('查找文件: %s\n', csv_file);
fprintf('绝对路径: %s\n', fullfile(pwd, csv_file));

% 检查文件是否存在
if ~exist(csv_file, 'file')
    % 尝试其他可能的路径
    alternative_paths = {
        '../01_python/01_mlog_parse/out/mlog_msg_0_Minifly_Sensor_IMU.csv'; ...
        'script/01_python/01_mlog_parse/out/mlog_msg_0_Minifly_Sensor_IMU.csv'; ...
        '../../script/01_python/01_mlog_parse/out/mlog_msg_0_Minifly_Sensor_IMU.csv'
    };
    
    fprintf('尝试其他可能的路径:\n');
    for i = 1:length(alternative_paths)
        alt_path = alternative_paths{i};
        if exist(alt_path, 'file')
            fprintf('找到文件: %s\n', alt_path);
            csv_file = alt_path;
            break;
        else
            fprintf('未找到: %s\n', alt_path);
        end
    end
    
    if ~exist(csv_file, 'file')
        error('CSV文件不存在。请检查文件路径。\n尝试的路径:\n%s', strjoin(alternative_paths, '\n'));
    end
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

% 提取滤波前加速度数据 (m/s²)
acc_before_x = data.acc_filter_before_0_;
acc_before_y = data.acc_filter_before_1_;
acc_before_z = data.acc_filter_before_2_;

% 提取滤波后加速度数据 (m/s²)
acc_after_x = data.acc_filter_after_0_;
acc_after_y = data.acc_filter_after_1_;
acc_after_z = data.acc_filter_after_2_;

% 提取滤波前陀螺仪数据 (rad/s)
gyro_before_x = data.gyro_filter_before_0_;
gyro_before_y = data.gyro_filter_before_1_;
gyro_before_z = data.gyro_filter_before_2_;

% 提取滤波后陀螺仪数据 (rad/s)
gyro_after_x = data.gyro_filter_after_0_;
gyro_after_y = data.gyro_filter_after_1_;
gyro_after_z = data.gyro_filter_after_2_;

% 提取时间间隔
delta_ts = data.delta_ts;

% 计算采样频率
% 由于timestamp单位是ms，delta_ts也是ms，需要转换为秒
delta_ts_sec = delta_ts / 1000; % 转换为秒
fs = 1 / mean(delta_ts_sec); % 平均采样频率
fprintf('平均采样频率: %.2f Hz\n', fs);
fprintf('平均采样周期: %.3f ms\n', mean(delta_ts));

%% 3. 数据预处理
fprintf('正在进行数据预处理...\n');

% 去除NaN值
valid_idx = ~isnan(timestamp) & ~isnan(acc_before_x) & ~isnan(gyro_before_x);
timestamp = timestamp(valid_idx);
acc_before_x = acc_before_x(valid_idx);
acc_before_y = acc_before_y(valid_idx);
acc_before_z = acc_before_z(valid_idx);
acc_after_x = acc_after_x(valid_idx);
acc_after_y = acc_after_y(valid_idx);
acc_after_z = acc_after_z(valid_idx);
gyro_before_x = gyro_before_x(valid_idx);
gyro_before_y = gyro_before_y(valid_idx);
gyro_before_z = gyro_before_z(valid_idx);
gyro_after_x = gyro_after_x(valid_idx);
gyro_after_y = gyro_after_y(valid_idx);
gyro_after_z = gyro_after_z(valid_idx);

% 计算时间向量（秒）
time_vec = (timestamp - timestamp(1)) / 1000; % 时间戳单位为毫秒，转换为秒

fprintf('有效数据点数: %d\n', length(time_vec));

%% 4. 绘制时域信号
fprintf('正在绘制时域信号...\n');

figure('Name', 'IMU时域信号', 'Position', [100, 100, 1200, 800]);

% 加速度时域图
subplot(3,2,1);
plot(time_vec, acc_before_x, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, acc_after_x, 'b-', 'LineWidth', 1);
title('X轴加速度');
xlabel('时间 (s)');
ylabel('加速度 (m/s²)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,2);
plot(time_vec, acc_before_y, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, acc_after_y, 'b-', 'LineWidth', 1);
title('Y轴加速度');
xlabel('时间 (s)');
ylabel('加速度 (m/s²)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,3);
plot(time_vec, acc_before_z, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, acc_after_z, 'b-', 'LineWidth', 1);
title('Z轴加速度');
xlabel('时间 (s)');
ylabel('加速度 (m/s²)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

% 陀螺仪时域图
subplot(3,2,4);
plot(time_vec, gyro_before_x, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, gyro_after_x, 'b-', 'LineWidth', 1);
title('X轴陀螺仪');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,5);
plot(time_vec, gyro_before_y, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, gyro_after_y, 'b-', 'LineWidth', 1);
title('Y轴陀螺仪');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,6);
plot(time_vec, gyro_before_z, 'r-', 'LineWidth', 1);
hold on;
plot(time_vec, gyro_after_z, 'b-', 'LineWidth', 1);
title('Z轴陀螺仪');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

%% 6. 计算并绘制频谱图
fprintf('正在计算频谱分析...\n');

% 加速度频谱分析
figure('Name', '加速度频谱分析', 'Position', [200, 200, 1200, 800]);

% X轴加速度频谱
subplot(3,2,1);
[f_acc_x_before, Pxx_acc_x_before] = compute_fft_spectrum(acc_before_x, fs);
[f_acc_x_after, Pxx_acc_x_after] = compute_fft_spectrum(acc_after_x, fs);
semilogx(f_acc_x_before, Pxx_acc_x_before, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_acc_x_after, Pxx_acc_x_after, 'b-', 'LineWidth', 1);
title('X轴加速度频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,2);
[f_acc_y_before, Pxx_acc_y_before] = compute_fft_spectrum(acc_before_y, fs);
[f_acc_y_after, Pxx_acc_y_after] = compute_fft_spectrum(acc_after_y, fs);
semilogx(f_acc_y_before, Pxx_acc_y_before, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_acc_y_after, Pxx_acc_y_after, 'b-', 'LineWidth', 1);
title('Y轴加速度频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,3);
[f_acc_z_before, Pxx_acc_z_before] = compute_fft_spectrum(acc_before_z, fs);
[f_acc_z_after, Pxx_acc_z_after] = compute_fft_spectrum(acc_after_z, fs);
semilogx(f_acc_z_before, Pxx_acc_z_before, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_acc_z_after, Pxx_acc_z_after, 'b-', 'LineWidth', 1);
title('Z轴加速度频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

% 陀螺仪频谱分析
figure('Name', '陀螺仪频谱分析', 'Position', [300, 300, 1200, 800]);

subplot(3,2,1);
[f_gyro_x_before, Pxx_gyro_x_before] = compute_fft_spectrum(gyro_before_x, fs);
[f_gyro_x_after, Pxx_gyro_x_after] = compute_fft_spectrum(gyro_after_x, fs);
semilogx(f_gyro_x_before, Pxx_gyro_x_before, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_gyro_x_after, Pxx_gyro_x_after, 'b-', 'LineWidth', 1);
title('X轴陀螺仪频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,2);
[f_gyro_y_before, Pxx_gyro_y_before] = compute_fft_spectrum(gyro_before_y, fs);
[f_gyro_y_after, Pxx_gyro_y_after] = compute_fft_spectrum(gyro_after_y, fs);
semilogx(f_gyro_y_before, Pxx_gyro_y_before, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_gyro_y_after, Pxx_gyro_y_after, 'b-', 'LineWidth', 1);
title('Y轴陀螺仪频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

subplot(3,2,3);
[f_gyro_z_before, Pxx_gyro_z_before] = compute_fft_spectrum(gyro_before_z, fs);
[f_gyro_z_after, Pxx_gyro_z_after] = compute_fft_spectrum(gyro_after_z, fs);
semilogx(f_gyro_z_before, Pxx_gyro_z_before, 'r-', 'LineWidth', 1);
hold on;
semilogx(f_gyro_z_after, Pxx_gyro_z_after, 'b-', 'LineWidth', 1);
title('Z轴陀螺仪频谱');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;

%% 7. 对比分析图
figure('Name', '滤波前后对比分析', 'Position', [400, 400, 1200, 600]);

% 加速度对比
subplot(2,3,1);
plot(f_acc_x_before, Pxx_acc_x_before, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_acc_x_after, Pxx_acc_x_after, 'b-', 'LineWidth', 1.5);
title('X轴加速度频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

subplot(2,3,2);
plot(f_acc_y_before, Pxx_acc_y_before, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_acc_y_after, Pxx_acc_y_after, 'b-', 'LineWidth', 1.5);
title('Y轴加速度频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

subplot(2,3,3);
plot(f_acc_z_before, Pxx_acc_z_before, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_acc_z_after, Pxx_acc_z_after, 'b-', 'LineWidth', 1.5);
title('Z轴加速度频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

% 陀螺仪对比
subplot(2,3,4);
plot(f_gyro_x_before, Pxx_gyro_x_before, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_gyro_x_after, Pxx_gyro_x_after, 'b-', 'LineWidth', 1.5);
title('X轴陀螺仪频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

subplot(2,3,5);
plot(f_gyro_y_before, Pxx_gyro_y_before, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_gyro_y_after, Pxx_gyro_y_after, 'b-', 'LineWidth', 1.5);
title('Y轴陀螺仪频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

subplot(2,3,6);
plot(f_gyro_z_before, Pxx_gyro_z_before, 'r-', 'LineWidth', 1.5);
hold on;
plot(f_gyro_z_after, Pxx_gyro_z_after, 'b-', 'LineWidth', 1.5);
title('Z轴陀螺仪频谱对比');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB)');
legend('滤波前', '滤波后', 'Location', 'best');
grid on;
xlim([0.1, fs/2]);

%% 8. 详细频谱分析和滤波效果评估
fprintf('正在进行详细频谱分析和滤波效果评估...\n');

% 计算统计信息
stats = struct();

% 加速度统计
stats.acc_before_x = struct('mean', mean(acc_before_x), 'std', std(acc_before_x), 'rms', rms(acc_before_x));
stats.acc_before_y = struct('mean', mean(acc_before_y), 'std', std(acc_before_y), 'rms', rms(acc_before_y));
stats.acc_before_z = struct('mean', mean(acc_before_z), 'std', std(acc_before_z), 'rms', rms(acc_before_z));

stats.acc_after_x = struct('mean', mean(acc_after_x), 'std', std(acc_after_x), 'rms', rms(acc_after_x));
stats.acc_after_y = struct('mean', mean(acc_after_y), 'std', std(acc_after_y), 'rms', rms(acc_after_y));
stats.acc_after_z = struct('mean', mean(acc_after_z), 'std', std(acc_after_z), 'rms', rms(acc_after_z));

% 陀螺仪统计
stats.gyro_before_x = struct('mean', mean(gyro_before_x), 'std', std(gyro_before_x), 'rms', rms(gyro_before_x));
stats.gyro_before_y = struct('mean', mean(gyro_before_y), 'std', std(gyro_before_y), 'rms', rms(gyro_before_y));
stats.gyro_before_z = struct('mean', mean(gyro_before_z), 'std', std(gyro_before_z), 'rms', rms(gyro_before_z));

stats.gyro_after_x = struct('mean', mean(gyro_after_x), 'std', std(gyro_after_x), 'rms', rms(gyro_after_x));
stats.gyro_after_y = struct('mean', mean(gyro_after_y), 'std', std(gyro_after_y), 'rms', rms(gyro_after_y));
stats.gyro_after_z = struct('mean', mean(gyro_after_z), 'std', std(gyro_after_z), 'rms', rms(gyro_after_z));


% 对每个轴进行频谱分析
fprintf('\n=== 详细频谱分析结果 ===\n');

% 加速度频谱分析
acc_analysis = struct();
for axis = {'x', 'y', 'z'}
    axis_name = axis{1};
    before_var = ['acc_before_' axis_name];
    after_var = ['acc_after_' axis_name];
    f_before_var = ['f_acc_' axis_name '_before'];
    f_after_var = ['f_acc_' axis_name '_after'];
    Pxx_before_var = ['Pxx_acc_' axis_name '_before'];
    Pxx_after_var = ['Pxx_acc_' axis_name '_after'];
    
    % 滤波前分析
    [signal_power_before, noise_power_before, snr_before, dominant_freq_before, signal_bandwidth_before] = ...
        analyze_spectrum(eval(f_before_var), eval(Pxx_before_var), fs);
    
    % 滤波后分析
    [signal_power_after, noise_power_after, snr_after, dominant_freq_after, signal_bandwidth_after] = ...
        analyze_spectrum(eval(f_after_var), eval(Pxx_after_var), fs);
    
    % 降噪效果
    noise_reduction_db = 10 * log10(noise_power_before / noise_power_after);
    signal_preservation = signal_power_after / signal_power_before;
    
    acc_analysis.(axis_name) = struct(...
        'before', struct('signal_power', signal_power_before, 'noise_power', noise_power_before, ...
                        'snr_db', snr_before, 'dominant_freq', dominant_freq_before, 'signal_bandwidth', signal_bandwidth_before), ...
        'after', struct('signal_power', signal_power_after, 'noise_power', noise_power_after, ...
                       'snr_db', snr_after, 'dominant_freq', dominant_freq_after, 'signal_bandwidth', signal_bandwidth_after), ...
        'noise_reduction_db', noise_reduction_db, 'signal_preservation', signal_preservation);
    
    fprintf('\n--- 加速度 %s轴 ---\n', upper(axis_name));
    fprintf('滤波前: 主频率=%.2fHz, 信噪比=%.2fdB, 信号功率=%.2e, 噪声功率=%.2e\n', ...
            dominant_freq_before, snr_before, signal_power_before, noise_power_before);
    fprintf('滤波后: 主频率=%.2fHz, 信噪比=%.2fdB, 信号功率=%.2e, 噪声功率=%.2e\n', ...
            dominant_freq_after, snr_after, signal_power_after, noise_power_after);
    fprintf('降噪效果: %.2fdB, 信号保持率: %.2f%%\n', noise_reduction_db, signal_preservation * 100);
end

% 陀螺仪频谱分析
gyro_analysis = struct();
for axis = {'x', 'y', 'z'}
    axis_name = axis{1};
    before_var = ['gyro_before_' axis_name];
    after_var = ['gyro_after_' axis_name];
    f_before_var = ['f_gyro_' axis_name '_before'];
    f_after_var = ['f_gyro_' axis_name '_after'];
    Pxx_before_var = ['Pxx_gyro_' axis_name '_before'];
    Pxx_after_var = ['Pxx_gyro_' axis_name '_after'];
    
    % 滤波前分析
    [signal_power_before, noise_power_before, snr_before, dominant_freq_before, signal_bandwidth_before] = ...
        analyze_spectrum(eval(f_before_var), eval(Pxx_before_var), fs);
    
    % 滤波后分析
    [signal_power_after, noise_power_after, snr_after, dominant_freq_after, signal_bandwidth_after] = ...
        analyze_spectrum(eval(f_after_var), eval(Pxx_after_var), fs);
    
    % 降噪效果
    noise_reduction_db = 10 * log10(noise_power_before / noise_power_after);
    signal_preservation = signal_power_after / signal_power_before;
    
    gyro_analysis.(axis_name) = struct(...
        'before', struct('signal_power', signal_power_before, 'noise_power', noise_power_before, ...
                        'snr_db', snr_before, 'dominant_freq', dominant_freq_before, 'signal_bandwidth', signal_bandwidth_before), ...
        'after', struct('signal_power', signal_power_after, 'noise_power', noise_power_after, ...
                       'snr_db', snr_after, 'dominant_freq', dominant_freq_after, 'signal_bandwidth', signal_bandwidth_after), ...
        'noise_reduction_db', noise_reduction_db, 'signal_preservation', signal_preservation);
    
    fprintf('\n--- 陀螺仪 %s轴 ---\n', upper(axis_name));
    fprintf('滤波前: 主频率=%.2fHz, 信噪比=%.2fdB, 信号功率=%.2e, 噪声功率=%.2e\n', ...
            dominant_freq_before, snr_before, signal_power_before, noise_power_before);
    fprintf('滤波后: 主频率=%.2fHz, 信噪比=%.2fdB, 信号功率=%.2e, 噪声功率=%.2e\n', ...
            dominant_freq_after, snr_after, signal_power_after, noise_power_after);
    fprintf('降噪效果: %.2fdB, 信号保持率: %.2f%%\n', noise_reduction_db, signal_preservation * 100);
end

% 显示统计结果
fprintf('\n=== 统计分析结果 ===\n');
fprintf('采样频率: %.2f Hz\n', fs);
fprintf('数据长度: %.2f 秒\n', time_vec(end));
fprintf('数据点数: %d\n', length(time_vec));

fprintf('\n--- 加速度统计 (m/s²) ---\n');
fprintf('滤波前 - X轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_before_x.mean, stats.acc_before_x.std, stats.acc_before_x.rms);
fprintf('滤波前 - Y轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_before_y.mean, stats.acc_before_y.std, stats.acc_before_y.rms);
fprintf('滤波前 - Z轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_before_z.mean, stats.acc_before_z.std, stats.acc_before_z.rms);

fprintf('滤波后 - X轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_after_x.mean, stats.acc_after_x.std, stats.acc_after_x.rms);
fprintf('滤波后 - Y轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_after_y.mean, stats.acc_after_y.std, stats.acc_after_y.rms);
fprintf('滤波后 - Z轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.acc_after_z.mean, stats.acc_after_z.std, stats.acc_after_z.rms);

fprintf('\n--- 陀螺仪统计 (rad/s) ---\n');
fprintf('滤波前 - X轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_before_x.mean, stats.gyro_before_x.std, stats.gyro_before_x.rms);
fprintf('滤波前 - Y轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_before_y.mean, stats.gyro_before_y.std, stats.gyro_before_y.rms);
fprintf('滤波前 - Z轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_before_z.mean, stats.gyro_before_z.std, stats.gyro_before_z.rms);

fprintf('滤波后 - X轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_after_x.mean, stats.gyro_after_x.std, stats.gyro_after_x.rms);
fprintf('滤波后 - Y轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_after_y.mean, stats.gyro_after_y.std, stats.gyro_after_y.rms);
fprintf('滤波后 - Z轴: 均值=%.4f, 标准差=%.4f, RMS=%.4f\n', stats.gyro_after_z.mean, stats.gyro_after_z.std, stats.gyro_after_z.rms);

%% 9. 0-300Hz频段能量分布详细分析
fprintf('\n=== 0-300Hz频段能量分布详细分析 ===\n');

% 对每个轴进行0-300Hz频段能量分布分析
fprintf('\n--- 加速度计0-300Hz频段能量分布 ---\n');
acc_energy_analysis = struct();
for axis = {'x', 'y', 'z'}
    axis_name = axis{1};
    f_before_var = ['f_acc_' axis_name '_before'];
    f_after_var = ['f_acc_' axis_name '_after'];
    Pxx_before_var = ['Pxx_acc_' axis_name '_before'];
    Pxx_after_var = ['Pxx_acc_' axis_name '_after'];
    
    % 滤波前能量分布分析
    [energy_before, freq_bands] = analyze_energy_distribution_0_300hz(eval(f_before_var), eval(Pxx_before_var));
    
    % 滤波后能量分布分析
    [energy_after, ~] = analyze_energy_distribution_0_300hz(eval(f_after_var), eval(Pxx_after_var));
    
    acc_energy_analysis.(axis_name) = struct('before', energy_before, 'after', energy_after);
    
    fprintf('\n加速度计%s轴频段能量分布:\n', upper(axis_name));
    fprintf('频段\t\t滤波前能量\t滤波后能量\t能量变化\t滤波前占比\t滤波后占比\n');
    fprintf('----\t\t--------\t--------\t--------\t--------\t--------\n');
    
    for i = 1:length(freq_bands)
        band_name = freq_bands{i, 1};
        field_name = freq_bands{i, 4};
        
        energy_before_val = energy_before.(field_name).energy;
        energy_after_val = energy_after.(field_name).energy;
        energy_change = energy_after_val - energy_before_val;
        energy_change_pct = (energy_change / energy_before_val) * 100;
        
        before_pct = energy_before.(field_name).percentage;
        after_pct = energy_after.(field_name).percentage;
        
        fprintf('%s\t%.2e\t%.2e\t%.2e (%.1f%%)\t%.1f%%\t\t%.1f%%\n', ...
                band_name, energy_before_val, energy_after_val, energy_change, energy_change_pct, before_pct, after_pct);
    end
end

fprintf('\n--- 陀螺仪0-300Hz频段能量分布 ---\n');
gyro_energy_analysis = struct();
for axis = {'x', 'y', 'z'}
    axis_name = axis{1};
    f_before_var = ['f_gyro_' axis_name '_before'];
    f_after_var = ['f_gyro_' axis_name '_after'];
    Pxx_before_var = ['Pxx_gyro_' axis_name '_before'];
    Pxx_after_var = ['Pxx_gyro_' axis_name '_after'];
    
    % 滤波前能量分布分析
    [energy_before, freq_bands] = analyze_energy_distribution_0_300hz(eval(f_before_var), eval(Pxx_before_var));
    
    % 滤波后能量分布分析
    [energy_after, ~] = analyze_energy_distribution_0_300hz(eval(f_after_var), eval(Pxx_after_var));
    
    gyro_energy_analysis.(axis_name) = struct('before', energy_before, 'after', energy_after);
    
    fprintf('\n陀螺仪%s轴频段能量分布:\n', upper(axis_name));
    fprintf('频段\t\t滤波前能量\t滤波后能量\t能量变化\t滤波前占比\t滤波后占比\n');
    fprintf('----\t\t--------\t--------\t--------\t--------\t--------\n');
    
    for i = 1:length(freq_bands)
        band_name = freq_bands{i, 1};
        field_name = freq_bands{i, 4};
        
        energy_before_val = energy_before.(field_name).energy;
        energy_after_val = energy_after.(field_name).energy;
        energy_change = energy_after_val - energy_before_val;
        energy_change_pct = (energy_change / energy_before_val) * 100;
        
        before_pct = energy_before.(field_name).percentage;
        after_pct = energy_after.(field_name).percentage;
        
        fprintf('%s\t%.2e\t%.2e\t%.2e (%.1f%%)\t%.1f%%\t\t%.1f%%\n', ...
                band_name, energy_before_val, energy_after_val, energy_change, energy_change_pct, before_pct, after_pct);
    end
end

% 绘制0-300Hz频段能量分布对比图
figure('Name', '0-300Hz频段能量分布对比', 'Position', [500, 500, 1400, 1000]);

% 加速度计能量分布对比
subplot(2,3,1);
plot_energy_distribution_comparison(acc_energy_analysis.x.before, acc_energy_analysis.x.after, 'X轴加速度');
subplot(2,3,2);
plot_energy_distribution_comparison(acc_energy_analysis.y.before, acc_energy_analysis.y.after, 'Y轴加速度');
subplot(2,3,3);
plot_energy_distribution_comparison(acc_energy_analysis.z.before, acc_energy_analysis.z.after, 'Z轴加速度');

% 陀螺仪能量分布对比
subplot(2,3,4);
plot_energy_distribution_comparison(gyro_energy_analysis.x.before, gyro_energy_analysis.x.after, 'X轴陀螺仪');
subplot(2,3,5);
plot_energy_distribution_comparison(gyro_energy_analysis.y.before, gyro_energy_analysis.y.after, 'Y轴陀螺仪');
subplot(2,3,6);
plot_energy_distribution_comparison(gyro_energy_analysis.z.before, gyro_energy_analysis.z.after, 'Z轴陀螺仪');

% 频段能量变化总结
fprintf('\n--- 0-300Hz频段能量变化总结 ---\n');
fprintf('分析各频段的滤波效果:\n');

% 计算平均能量变化
freq_band_names = {'band_0_10Hz', 'band_10_20Hz', 'band_20_50Hz', 'band_50_100Hz', 'band_100_200Hz', 'band_200_300Hz'};
freq_band_labels = {'0-10Hz', '10-20Hz', '20-50Hz', '50-100Hz', '100-200Hz', '200-300Hz'};

for i = 1:length(freq_band_names)
    band_name = freq_band_names{i};
    band_label = freq_band_labels{i};
    
    % 计算加速度计平均能量变化
    acc_energy_change = 0;
    for axis = {'x', 'y', 'z'}
        axis_name = axis{1};
        before_energy = acc_energy_analysis.(axis_name).before.(band_name).energy;
        after_energy = acc_energy_analysis.(axis_name).after.(band_name).energy;
        if before_energy > 0
            acc_energy_change = acc_energy_change + (after_energy - before_energy) / before_energy;
        end
    end
    acc_energy_change = acc_energy_change / 3 * 100; % 平均百分比变化
    
    % 计算陀螺仪平均能量变化
    gyro_energy_change = 0;
    for axis = {'x', 'y', 'z'}
        axis_name = axis{1};
        before_energy = gyro_energy_analysis.(axis_name).before.(band_name).energy;
        after_energy = gyro_energy_analysis.(axis_name).after.(band_name).energy;
        if before_energy > 0
            gyro_energy_change = gyro_energy_change + (after_energy - before_energy) / before_energy;
        end
    end
    gyro_energy_change = gyro_energy_change / 3 * 100; % 平均百分比变化
    
    fprintf('%s频段:\n', band_label);
    fprintf('  加速度计能量变化: %.1f%%\n', acc_energy_change);
    fprintf('  陀螺仪能量变化: %.1f%%\n', gyro_energy_change);
    
    if acc_energy_change < -10
        fprintf('  ✓ 加速度计该频段能量显著减少，滤波效果良好\n');
    elseif acc_energy_change < 0
        fprintf('  ⚠ 加速度计该频段能量略有减少，滤波效果一般\n');
    else
        fprintf('  ✗ 加速度计该频段能量增加，滤波可能过度或无效\n');
    end
    
    if gyro_energy_change < -10
        fprintf('  ✓ 陀螺仪该频段能量显著减少，滤波效果良好\n');
    elseif gyro_energy_change < 0
        fprintf('  ⚠ 陀螺仪该频段能量略有减少，滤波效果一般\n');
    else
        fprintf('  ✗ 陀螺仪该频段能量增加，滤波可能过度或无效\n');
    end
    fprintf('\n');
end

%% 10. 信号dB含义和滤波效果详细解释
fprintf('\n=== 信号dB含义和滤波效果详细解释 ===\n');

fprintf('\n--- 信号dB的含义 ---\n');
fprintf('• 功率谱密度(dB) = 10*log10(功率谱密度)\n');
fprintf('• dB是相对单位，表示信号功率的对数比值\n');
fprintf('• 每增加3dB，信号功率增加一倍\n');
fprintf('• 每增加10dB，信号功率增加10倍\n');
fprintf('• 负dB值表示信号功率小于参考值\n');

fprintf('\n--- 四轴飞行器IMU信号特征 ---\n');
fprintf('• 信号频段(0-50Hz): 四轴飞行器主要运动频率\n');
fprintf('  - 悬停: 0-5Hz\n');
fprintf('  - 姿态调整: 5-20Hz\n');
fprintf('  - 机动飞行: 20-50Hz\n');
fprintf('• 噪声频段(50-500Hz): 传感器噪声和振动\n');
fprintf('  - 电机振动: 50-200Hz\n');
fprintf('  - 传感器噪声: 200-500Hz\n');
fprintf('• 高频噪声(>500Hz): 电子噪声和量化误差\n');

fprintf('\n--- 滤波前后信号变化分析 ---\n');
% 计算平均信号带宽变化
avg_signal_bandwidth_acc_before = mean([acc_analysis.x.before.signal_bandwidth, acc_analysis.y.before.signal_bandwidth, acc_analysis.z.before.signal_bandwidth]);
avg_signal_bandwidth_acc_after = mean([acc_analysis.x.after.signal_bandwidth, acc_analysis.y.after.signal_bandwidth, acc_analysis.z.after.signal_bandwidth]);
avg_signal_bandwidth_gyro_before = mean([gyro_analysis.x.before.signal_bandwidth, gyro_analysis.y.before.signal_bandwidth, gyro_analysis.z.before.signal_bandwidth]);
avg_signal_bandwidth_gyro_after = mean([gyro_analysis.x.after.signal_bandwidth, gyro_analysis.y.after.signal_bandwidth, gyro_analysis.z.after.signal_bandwidth]);

fprintf('• 信号带宽变化:\n');
fprintf('  - 加速度计: %.1fHz → %.1fHz (变化: %.1fHz)\n', avg_signal_bandwidth_acc_before, avg_signal_bandwidth_acc_after, avg_signal_bandwidth_acc_after - avg_signal_bandwidth_acc_before);
fprintf('  - 陀螺仪: %.1fHz → %.1fHz (变化: %.1fHz)\n', avg_signal_bandwidth_gyro_before, avg_signal_bandwidth_gyro_after, avg_signal_bandwidth_gyro_after - avg_signal_bandwidth_gyro_before);

if avg_signal_bandwidth_acc_after < avg_signal_bandwidth_acc_before
    fprintf('  ✓ 加速度计信号带宽减小，滤波有效去除了高频噪声\n');
else
    fprintf('  ⚠ 加速度计信号带宽变化不明显，可能需要调整滤波参数\n');
end

if avg_signal_bandwidth_gyro_after < avg_signal_bandwidth_gyro_before
    fprintf('  ✓ 陀螺仪信号带宽减小，滤波有效去除了高频噪声\n');
else
    fprintf('  ⚠ 陀螺仪信号带宽变化不明显，可能需要调整滤波参数\n');
end

%% 10. 四轴飞行器IMU滤波建议
fprintf('\n=== 四轴飞行器IMU滤波建议 ===\n');

% 分析滤波效果
avg_noise_reduction_acc = mean([acc_analysis.x.noise_reduction_db, acc_analysis.y.noise_reduction_db, acc_analysis.z.noise_reduction_db]);
avg_noise_reduction_gyro = mean([gyro_analysis.x.noise_reduction_db, gyro_analysis.y.noise_reduction_db, gyro_analysis.z.noise_reduction_db]);
avg_signal_preservation_acc = mean([acc_analysis.x.signal_preservation, acc_analysis.y.signal_preservation, acc_analysis.z.signal_preservation]);
avg_signal_preservation_gyro = mean([gyro_analysis.x.signal_preservation, gyro_analysis.y.signal_preservation, gyro_analysis.z.signal_preservation]);

fprintf('平均降噪效果: 加速度 %.2fdB, 陀螺仪 %.2fdB\n', avg_noise_reduction_acc, avg_noise_reduction_gyro);
fprintf('平均信号保持率: 加速度 %.1f%%, 陀螺仪 %.1f%%\n', avg_signal_preservation_acc * 100, avg_signal_preservation_gyro * 100);

% 滤波效果评估
if avg_noise_reduction_acc > 3 && avg_signal_preservation_acc > 0.8
    fprintf('✓ 加速度滤波效果良好：降噪明显且信号保持良好\n');
elseif avg_noise_reduction_acc > 1
    fprintf('⚠ 加速度滤波效果一般：有一定降噪但可能影响信号质量\n');
else
    fprintf('✗ 加速度滤波效果较差：降噪不明显或信号损失严重\n');
end

if avg_noise_reduction_gyro > 3 && avg_signal_preservation_gyro > 0.8
    fprintf('✓ 陀螺仪滤波效果良好：降噪明显且信号保持良好\n');
elseif avg_noise_reduction_gyro > 1
    fprintf('⚠ 陀螺仪滤波效果一般：有一定降噪但可能影响信号质量\n');
else
    fprintf('✗ 陀螺仪滤波效果较差：降噪不明显或信号损失严重\n');
end

% 针对四轴飞行器的具体建议
fprintf('\n--- 四轴飞行器IMU滤波优化建议 ---\n');

% 基于采样频率的建议
if fs >= 1000
    fprintf('• 采样频率较高(%.0fHz)，建议使用低通滤波器截止频率: 50-100Hz\n', fs);
    fprintf('• 可考虑使用2阶或3阶巴特沃斯滤波器\n');
elseif fs >= 500
    fprintf('• 采样频率中等(%.0fHz)，建议使用低通滤波器截止频率: 25-50Hz\n', fs);
    fprintf('• 可考虑使用2阶巴特沃斯滤波器\n');
else
    fprintf('• 采样频率较低(%.0fHz)，建议使用低通滤波器截止频率: 10-25Hz\n', fs);
    fprintf('• 注意避免过度滤波影响控制响应\n');
end

% 基于噪声分析的建议
if avg_noise_reduction_acc < 2
    fprintf('• 加速度计噪声较大，建议：\n');
    fprintf('  - 检查硬件连接和电源稳定性\n');
    fprintf('  - 考虑使用卡尔曼滤波器进行更智能的滤波\n');
    fprintf('  - 增加滤波器的阶数或降低截止频率\n');
end

if avg_noise_reduction_gyro < 2
    fprintf('• 陀螺仪噪声较大，建议：\n');
    fprintf('  - 检查陀螺仪校准和温度补偿\n');
    fprintf('  - 考虑使用互补滤波器结合加速度计数据\n');
    fprintf('  - 使用自适应滤波器根据飞行状态调整参数\n');
end

% 控制性能相关建议
fprintf('\n--- 控制性能优化建议 ---\n');
fprintf('• 对于姿态控制，建议陀螺仪截止频率: 20-40Hz\n');
fprintf('• 对于位置控制，建议加速度计截止频率: 10-20Hz\n');
fprintf('• 考虑使用互补滤波器或扩展卡尔曼滤波器进行传感器融合\n');
fprintf('• 根据飞行模式动态调整滤波参数（悬停vs机动飞行）\n');

% 实时性考虑
fprintf('\n--- 实时性考虑 ---\n');
fprintf('• 滤波算法延迟应小于控制周期(通常1-5ms)\n');
fprintf('• 考虑使用IIR滤波器减少计算量\n');
fprintf('• 避免使用过长的滤波器窗口影响实时性\n');

%% 11. 滤波效果总结报告
fprintf('\n=== 滤波效果总结报告 ===\n');

% 计算综合滤波效果指标
fprintf('\n--- 综合滤波效果指标 ---\n');
fprintf('采样频率: %.0f Hz (%.1f ms采样周期)\n', fs, 1000/fs);
fprintf('数据时长: %.2f 秒\n', time_vec(end));
fprintf('数据点数: %d\n', length(time_vec));

% 降噪效果总结
fprintf('\n--- 降噪效果总结 ---\n');
fprintf('加速度计平均降噪: %.2f dB\n', avg_noise_reduction_acc);
fprintf('陀螺仪平均降噪: %.2f dB\n', avg_noise_reduction_gyro);

if avg_noise_reduction_acc > 6
    fprintf('✓ 加速度计降噪效果优秀 (>6dB)\n');
elseif avg_noise_reduction_acc > 3
    fprintf('✓ 加速度计降噪效果良好 (3-6dB)\n');
elseif avg_noise_reduction_acc > 1
    fprintf('⚠ 加速度计降噪效果一般 (1-3dB)\n');
else
    fprintf('✗ 加速度计降噪效果较差 (<1dB)\n');
end

if avg_noise_reduction_gyro > 6
    fprintf('✓ 陀螺仪降噪效果优秀 (>6dB)\n');
elseif avg_noise_reduction_gyro > 3
    fprintf('✓ 陀螺仪降噪效果良好 (3-6dB)\n');
elseif avg_noise_reduction_gyro > 1
    fprintf('⚠ 陀螺仪降噪效果一般 (1-3dB)\n');
else
    fprintf('✗ 陀螺仪降噪效果较差 (<1dB)\n');
end

% 信号保持率总结
fprintf('\n--- 信号保持率总结 ---\n');
fprintf('加速度计信号保持率: %.1f%%\n', avg_signal_preservation_acc * 100);
fprintf('陀螺仪信号保持率: %.1f%%\n', avg_signal_preservation_gyro * 100);

if avg_signal_preservation_acc > 0.9
    fprintf('✓ 加速度计信号保持优秀 (>90%%)\n');
elseif avg_signal_preservation_acc > 0.8
    fprintf('✓ 加速度计信号保持良好 (80-90%%)\n');
elseif avg_signal_preservation_acc > 0.7
    fprintf('⚠ 加速度计信号保持一般 (70-80%%)\n');
else
    fprintf('✗ 加速度计信号保持较差 (<70%%)\n');
end

if avg_signal_preservation_gyro > 0.9
    fprintf('✓ 陀螺仪信号保持优秀 (>90%%)\n');
elseif avg_signal_preservation_gyro > 0.8
    fprintf('✓ 陀螺仪信号保持良好 (80-90%%)\n');
elseif avg_signal_preservation_gyro > 0.7
    fprintf('⚠ 陀螺仪信号保持一般 (70-80%%)\n');
else
    fprintf('✗ 陀螺仪信号保持较差 (<70%%)\n');
end

% 滤波质量综合评估
fprintf('\n--- 滤波质量综合评估 ---\n');
filter_quality_score = (avg_noise_reduction_acc + avg_noise_reduction_gyro) / 2 + ...
                      (avg_signal_preservation_acc + avg_signal_preservation_gyro) * 10;
fprintf('滤波质量评分: %.1f/20\n', filter_quality_score);

if filter_quality_score > 15
    fprintf('🏆 滤波质量优秀！滤波器参数设置合理\n');
elseif filter_quality_score > 12
    fprintf('👍 滤波质量良好，可考虑微调参数\n');
elseif filter_quality_score > 8
    fprintf('⚠ 滤波质量一般，建议重新调整滤波器参数\n');
else
    fprintf('❌ 滤波质量较差，需要重新设计滤波器\n');
end

%% 12. 保存结果
fprintf('\n正在保存分析结果...\n');

% 保存工作空间变量
save('imu_analysis_results.mat', 'data', 'stats', 'fs', 'time_vec', ...
     'acc_before_x', 'acc_before_y', 'acc_before_z', 'acc_after_x', 'acc_after_y', 'acc_after_z', ...
     'gyro_before_x', 'gyro_before_y', 'gyro_before_z', 'gyro_after_x', 'gyro_after_y', 'gyro_after_z', ...
     'acc_analysis', 'gyro_analysis', 'acc_energy_analysis', 'gyro_energy_analysis', 'filter_quality_score');

fprintf('分析完成！结果已保存到 imu_analysis_results.mat\n');
fprintf('生成了以下图表：\n');
fprintf('1. IMU时域信号图\n');
fprintf('2. 加速度频谱分析图\n');
fprintf('3. 陀螺仪频谱分析图\n');
fprintf('4. 滤波前后对比分析图\n');
fprintf('5. 0-300Hz频段能量分布对比图\n');
fprintf('6. 详细的频谱分析和滤波效果评估报告\n');
fprintf('7. 信号dB含义和滤波效果详细解释\n');

%% 13. FFT频谱分析函数
function [f, Pxx] = compute_fft_spectrum(signal, ~)
    % 计算信号的FFT频谱
    
    % 使用Welch方法计算功率谱密度
    [Pxx, f] = pwelch(signal, [], [], []);
    
    % 转换为dB
    Pxx = 10*log10(Pxx);
end

%% 12. 频谱特征分析函数
function [signal_power, noise_power, snr_db, dominant_freq, signal_bandwidth] = analyze_spectrum(f, Pxx, ~)
    % 分析频谱特征
    % 对于1kHz采样频率的四轴飞行器IMU数据
    
    % 信号功率：低频部分（0-50Hz）- 四轴飞行器主要运动频率
    low_freq_idx = f <= 50;
    signal_power = sum(Pxx(low_freq_idx)) * (f(2) - f(1));
    
    % 噪声功率：中高频部分（50-500Hz）- 传感器噪声和振动
    mid_freq_idx = f > 50 & f <= 500;
    noise_power = sum(Pxx(mid_freq_idx)) * (f(2) - f(1));
    
    % 高频噪声（500Hz以上）- 用于分析但不影响主要计算
    % high_freq_idx = f > 500; % 暂时注释掉未使用的变量
    % high_noise_power = sum(Pxx(high_freq_idx)) * (f(2) - f(1));
    
    % 信噪比（信号与中频噪声的比值）
    if noise_power > 0
        snr_db = 10 * log10(signal_power / noise_power);
    else
        snr_db = Inf;
    end
    
    % 主频率（功率谱密度最大的频率）
    [~, max_idx] = max(Pxx);
    dominant_freq = f(max_idx);
    
    % 信号带宽（包含90%信号功率的频率范围）
    cumulative_power = cumsum(Pxx) * (f(2) - f(1));
    total_power = cumulative_power(end);
    signal_90_idx = find(cumulative_power >= 0.05 * total_power & cumulative_power <= 0.95 * total_power);
    if ~isempty(signal_90_idx)
        signal_bandwidth = f(signal_90_idx(end)) - f(signal_90_idx(1));
    else
        signal_bandwidth = 0;
    end
end

%% 13. 0-300Hz频段能量分布分析函数
function [energy_distribution, freq_bands] = analyze_energy_distribution_0_300hz(f, Pxx)
    % 分析0-300Hz频段的能量分布
    % 将0-300Hz分为多个子频段进行详细分析
    
    % 定义频段
    freq_bands = {
        '0-10Hz', 0, 10, 'band_0_10Hz';      % 悬停和慢速运动
        '10-20Hz', 10, 20, 'band_10_20Hz';    % 姿态调整
        '20-50Hz', 20, 50, 'band_20_50Hz';    % 机动飞行
        '50-100Hz', 50, 100, 'band_50_100Hz';  % 电机振动低频
        '100-200Hz', 100, 200, 'band_100_200Hz'; % 电机振动高频
        '200-300Hz', 200, 300, 'band_200_300Hz'  % 传感器噪声
    };
    
    energy_distribution = struct();
    
    for i = 1:size(freq_bands, 1)
        f_low = freq_bands{i, 2};
        f_high = freq_bands{i, 3};
        field_name = freq_bands{i, 4};
        
        % 找到对应频段的索引
        band_idx = f >= f_low & f < f_high;
        
        if any(band_idx)
            % 计算该频段的能量
            band_energy = sum(Pxx(band_idx)) * (f(2) - f(1));
            
            % 计算该频段的平均功率谱密度
            band_avg_psd = mean(Pxx(band_idx));
            
            % 计算该频段的峰值频率
            [~, max_idx] = max(Pxx(band_idx));
            band_freqs = f(band_idx);
            band_peak_freq = band_freqs(max_idx);
            
            % 计算该频段的功率占比
            total_energy_0_300 = sum(Pxx(f <= 300)) * (f(2) - f(1));
            band_percentage = (band_energy / total_energy_0_300) * 100;
            
            energy_distribution.(field_name) = struct(...
                'energy', band_energy, ...
                'avg_psd', band_avg_psd, ...
                'peak_freq', band_peak_freq, ...
                'percentage', band_percentage);
        else
            energy_distribution.(field_name) = struct(...
                'energy', 0, ...
                'avg_psd', 0, ...
                'peak_freq', 0, ...
                'percentage', 0);
        end
    end
end

%% 14. 能量分布对比图绘制函数
function plot_energy_distribution_comparison(energy_before, energy_after, title_str)
    % 绘制能量分布对比图
    
    % 频段名称和对应的字段名
    freq_bands = {'0-10Hz', '10-20Hz', '20-50Hz', '50-100Hz', '100-200Hz', '200-300Hz'};
    field_names = {'band_0_10Hz', 'band_10_20Hz', 'band_20_50Hz', 'band_50_100Hz', 'band_100_200Hz', 'band_200_300Hz'};
    
    % 提取数据
    before_percentages = zeros(1, length(field_names));
    after_percentages = zeros(1, length(field_names));
    
    for i = 1:length(field_names)
        before_percentages(i) = energy_before.(field_names{i}).percentage;
        after_percentages(i) = energy_after.(field_names{i}).percentage;
    end
    
    % 绘制柱状图
    x = 1:length(freq_bands);
    width = 0.35;
    
    bar(x - width/2, before_percentages, width, 'FaceColor', [0.8, 0.2, 0.2], 'DisplayName', '滤波前');
    hold on;
    bar(x + width/2, after_percentages, width, 'FaceColor', [0.2, 0.2, 0.8], 'DisplayName', '滤波后');
    
    % 设置图形属性
    set(gca, 'XTick', x, 'XTickLabel', freq_bands);
    xlabel('频段');
    ylabel('能量占比 (%)');
    title(title_str);
    legend('Location', 'best');
    grid on;
    
    % 添加数值标签
    for i = 1:length(freq_bands)
        text(i - width/2, before_percentages(i) + 0.5, sprintf('%.1f', before_percentages(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 8);
        text(i + width/2, after_percentages(i) + 0.5, sprintf('%.1f', after_percentages(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 8);
    end
    
    hold off;
end
