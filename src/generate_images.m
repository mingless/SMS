files = dir('*.mat');
for file = files'
    clearvars -except files file
    load(file.name);
%     strcat(file.name(1:end-4),'.tex')
    figure('Position',  [403 246 620 420]);
    title('obiekt z regulatorem PID');
    subplot('Position', [0.1 0.12 0.8 0.15]);
    stairs(u);
    ylabel('$u$');
    xlabel('$k$');
    decimal_comma(gca, 'XY');
    subplot('Position', [0.1 0.37 0.8 0.6]);
    plot(y);
    ylabel('$y$');
    decimal_comma(gca, 'XY');
    hold on;
    if exist('yzad','var')
        stairs(yzad,':');
        matlab2tikz(strcat('../sprawozdanie/Projekt56/im/',file.name(1:end-4),'.tex'),...
            'encoding','UTF-8','extraCode',strcat('%err = ',num2str(sum((yzad-y).^2))),...
            'parseStrings',false);
    else
        matlab2tikz(strcat('../sprawozdanie/Projekt56/im/',file.name(1:end-4),'.tex'),...
            'encoding','UTF-8','parseStrings',false);
    end
end
close all;
