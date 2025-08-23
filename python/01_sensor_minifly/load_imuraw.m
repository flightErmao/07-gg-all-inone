% Clear environment (close figures, clear variables, clear command window)
close all; clear; clc;

% Resolve CSV path next to this script
scriptDir = fileparts(mfilename('fullpath'));
csvPath   = fullfile(scriptDir, 'imuraw.csv');

% Read CSV into table
imuRawData = readtable(csvPath);

% Expose columns as individual variables in workspace
a_x = imuRawData.a_x; %#ok<NASGU>
a_y = imuRawData.a_y; %#ok<NASGU>
a_z = imuRawData.a_z; %#ok<NASGU>

v_x = imuRawData.v_x; %#ok<NASGU>
v_y = imuRawData.v_y; %#ok<NASGU>
v_z = imuRawData.v_z; %#ok<NASGU>

time_stamp = imuRawData.time_stamp; %#ok<NASGU>

% Summary
fprintf('Loaded %d rows from %s\n', height(imuRawData), csvPath); 