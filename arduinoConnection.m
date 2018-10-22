clear all
close all
clc

% Connect to arduino
ard = arduino;
dev = i2cdev(ard,'0x68');
