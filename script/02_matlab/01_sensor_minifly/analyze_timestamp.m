% Analyze time_stamp differences between consecutive frames and plot
close all; clear; clc;

% Locate CSV next to this script
scriptDir = fileparts(mfilename('fullpath'));
csvPath   = fullfile(scriptDir, 'imuraw.csv');

% Read data
tbl = readtable(csvPath);
if ~ismember('time_stamp', tbl.Properties.VariableNames)
	error('Column time_stamp not found in %s', csvPath);
end

% Use uint64 to handle wrap-around safely (original is u32)
ts = uint64(tbl.time_stamp(:));
N  = numel(ts);

if N < 2
	error('Not enough samples: need at least 2 rows, found %d', N);
end

% Compute difference between consecutive frames: ts(i) - ts(i-1)
% with modulo 2^32 wrap handling
MOD = uint64(2^32);
frameDiff = nan(N,1);
for i = 2:N
	frameDiff(i) = double(mod(ts(i) - ts(i-1), MOD));
end

% Print the difference sequence
fprintf('时间戳差值序列:\n');
for i = 2:N
	fprintf('帧 %d: 差值 = %d\n', i, frameDiff(i));
end

% Plot
figure('Name','time_stamp frame difference','Color','w');
plot(2:N, frameDiff(2:N), 'r-', 'LineWidth', 1.5);
xlabel('帧索引');
ylabel('时间戳差值 [ticks]');
title('相邻帧时间戳差值 (带u32溢出处理)');
grid on;

% Display summary statistics
fprintf('\n统计信息:\n');
fprintf('总帧数: %d\n', N);
fprintf('平均差值: %.2f\n', mean(frameDiff(2:N), 'omitnan'));
fprintf('最大差值: %.0f\n', max(frameDiff(2:N), [], 'omitnan'));
fprintf('最小差值: %.0f\n', min(frameDiff(2:N), [], 'omitnan')); 