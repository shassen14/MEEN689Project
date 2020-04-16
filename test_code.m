clear all; clc; close all;

% load file
matFile = 'data_collection/walk2_100.mat';
load(matFile)


longTable = timetable2table(Position(:, 1));
longArray = table2array(longTable)

