clc; clear;

T1 = readtable('sparse_test_0_new.DAT');
mean_no_noise = mean(T1.Error);
Use_count_0 = sum(T1.Error<0.05);

%boxplot(T1.Error);
%title('Boxplot of error with no noise')
%ylabel('Error [m]') 

T2 = readtable ('sparse_test_5_new.DAT');
mean_low_noise = mean(T2.Error);

Use_count_5 = sum(T2.Error<0.05);

boxplot(T2.Error);
title('Boxplot of error with a low amount of added noise.')
ylabel('Error [m]') 

T3 = readtable ('sparse_test_10_new.DAT');
mean_high_noise = mean(T3.Error);

Use_count_10 = sum(T3.Error<0.05);

%boxplot(T3.Error);
%title('Boxplot of error with a high amount of added noise.')
%ylabel('Error [m]') 


%%This part is based on https://in.mathworks.com/matlabcentral/answers/289642-multiple-boxplots-on-same-figure
%Errors = [T1.Error' T2.Error' T3.Error'];

%group = [ones(size(T1.Error)); 2 * ones(size(T2.Error)); 3 * ones(size(T3.Error))];
     
%figure
%boxplot([T1.Error; T2.Error; T3.Error],group)
%title('Boxplot of error with a low amount of added noise.')
%ylabel('Error [m]') 
%xlabel('Deviation')
%set(gca,'XTickLabel',{'0 ','5','10'})

