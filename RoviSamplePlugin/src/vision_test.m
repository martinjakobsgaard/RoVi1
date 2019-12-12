clc; clear;

T1 = readtable('sparse_test_with_0.DAT');
mean_no_noise = mean(T1.Error);

%boxplot(T1.Error);
%title('Boxplot of error with no noise')
%ylabel('Error [m]') 

T2 = readtable ('sparse_test_with_5.DAT');
mean_low_noise = mean(T2.Error);

Use_count_5 = sum(T2.Error<0.05);

%boxplot(T2.Error);
%title('Boxplot of error with a low amount of added noise.')
%ylabel('Error [m]') 

T3 = readtable ('sparse_test_with_10.DAT');
mean_high_noise = mean(T3.Error);

Use_count_10 = sum(T3.Error<0.05);

boxplot(T3.Error);
title('Boxplot of error with a high amount of added noise.')
ylabel('Error [m]') 