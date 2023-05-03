function plot_function(data, title_name, main_label,labels_names, lineNames, timeVec, linewidth, colors,f)
    % Utility function for the stacked plots.
    font = 15; stackedfont = 11.8;

    % Diminish linewidth for better comparison
    linewidth = linewidth - 1;
    persistent n
    if isempty(n)
        n = 0;
    end
    % Create zero vec for plots
    len = length(timeVec) -1;
    zero_vec = zeros(len + 1,1);
    % The inserted string with the names of the labels is splitted and the number of "lines" of plots is counted.
    splittedLabels = split(labels_names,';');
    splittedLineNames = split(lineNames,';');
    % Maximum an Minimum y value for limits
    [yMin,yMax] = bounds([data,zero_vec],'all');
    yMin = yMin - 0.1*yMin;
    yMax = yMax + 0.1*yMax;

    fig = figure(n+5);
    hold off,  
    D = seconds(0:len)/f;
    numberOflines = size(data,2);
    
    if numberOflines == 2
    t=tiledlayout(1,1);
    nexttile
        TT = array2timetable([data,zero_vec,zero_vec],...
                            'RowTimes', D, ...
                            'VariableNames',[splittedLabels;'zero line';'zero line ']);
        s = stackedplot(TT, {[1 3],[2 4]},'LineWidth',linewidth);
        % Change the line style and Colors
        s.LineProperties(1).LineStyle = {'-','--'};
        s.LineProperties(1).Color = [colors(1,:);[0 0 0]];
        s.LineProperties(2).LineStyle = {'-','--'};
        s.LineProperties(2).Color = [colors(3,:);[0 0 0]];
        % Change yticks
        s.AxesProperties(1).YLimits = [yMin yMax]; % Changes the limits of the first subplot
        s.AxesProperties(2).YLimits = [yMin yMax]; % Changes the limits of the second subplot
        % Set our labels
        s.DisplayLabels = splittedLabels;
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    xlabel(t,"\bf{time [s]}",'fontsize',font); 
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font);
    big_label.FontSize = font;

    elseif numberOflines == 3
    t=tiledlayout(1,1);
    nexttile
        TT = array2timetable([data,zero_vec,zero_vec,zero_vec],...
                        'RowTimes', D, ...
                        'VariableNames',[splittedLabels;'zero line';'zero line ';'zero line  ']);
        s = stackedplot(TT, {[1 4],[2 5],[3 6]},'LineWidth',linewidth);
        % Change the line style and Colors
        s.LineProperties(1).LineStyle = {'-','--'};
        s.LineProperties(1).Color = [colors(1,:);[0 0 0]];
        s.LineProperties(2).LineStyle = {'-','--'};
        s.LineProperties(2).Color = [colors(2,:);[0 0 0]];
        s.LineProperties(3).LineStyle = {'-','--'};
        s.LineProperties(3).Color = [colors(3,:);[0 0 0]];
        % Change yticks
        s.AxesProperties(1).YLimits = [yMin yMax]; % Changes the limits of the first subplot
        s.AxesProperties(2).YLimits = [yMin yMax]; % Changes the limits of the second subplot
        s.AxesProperties(3).YLimits = [yMin yMax]; % Changes the limits of the third subplot
        % Set our labels
        s.DisplayLabels = splittedLabels;
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    xlabel(t,"\bf{time [s]}",'fontsize',font); 
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font);
    big_label.FontSize = font;
 
    elseif numberOflines == 4
    t=tiledlayout(3,1);
    nexttile
        TT = array2timetable([data(:,1:2),zero_vec,zero_vec],...
        'RowTimes', D, ...
        'VariableNames',[splittedLabels(1:2);'zero line';'zero line ']);
        s = stackedplot(TT, {[1 3],[2 4]},'LineWidth',linewidth);
        % Change the line style and Colors
        s.LineProperties(1).LineStyle = {'-','--'};
        s.LineProperties(1).Color = [colors(1,:);[0 0 0]];
        s.LineProperties(2).LineStyle = {'-','--'};
        s.LineProperties(2).Color = [colors(2,:);[0 0 0]];
        % Change yticks
        [yMin,yMax] = bounds(data(:,1:3),'all');
        s.AxesProperties(1).YLimits = [yMin yMax]; % Changes the limits of the first subplot
        s.AxesProperties(2).YLimits = [yMin yMax]; % Changes the limits of the second subplot
        % Set our labels
        s.DisplayLabels = splittedLineNames;
        % Font of the stackedplot
        s.FontSize = stackedfont; 
        % Update figures counter n
        n = n + 1;
        grid on
    nexttile
        plot(timeVec,data(:,3),'Color',colors(3,:)),grid on
        yline(0,'LineStyle','--','Color','k','LineWidth',1)
        ylabel('\boldmath{$e_{tot} = \sqrt{x^2 + y^2}$}','FontSize',font,'Rotation',0)
        legend(splittedLabels{3},'zero line','FontSize',font)
        ylim([yMin yMax]);
    nexttile
        plot(timeVec,data(:,4),'Color',colors(4,:)), grid on
        yline(0,'LineStyle','--','Color','k','LineWidth',1)
        ylabel('\boldmath{$e_{\theta} [rad] $}','FontSize',font,'Rotation',0)
        legend(splittedLabels{4},'zero line','FontSize',font)

    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    xlabel(t,"\bf{time [s]}",'fontsize',font); 
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font);
    big_label.FontSize = font;
       

    elseif numberOflines == 6
    t=tiledlayout(1,1);
    nexttile
        TT = array2timetable([data,zero_vec,zero_vec,zero_vec],...
        'RowTimes', D, ...
        'VariableNames',[splittedLineNames;'zero line';'zero line ';'zero line  ']);
        s = stackedplot(TT, {[1 4 7],[2 5 8],[3 6 9]},'LineWidth',linewidth);
        % Change the line style and Colors
        s.LineProperties(1).LineStyle = {'-','-.','--'};
        s.LineProperties(1).Color = [colors(1:2,:);[0 0 0]];
        s.LineProperties(2).LineStyle ={'-','-.','--'};
        s.LineProperties(2).Color = [colors(3:4,:);[0 0 0]];
        s.LineProperties(3).LineStyle = {'-','-.','--'};
        s.LineProperties(3).Color =[colors(5:6,:);[0 0 0]];
        % Change yticks
        [mintheta,maxtheta] = bounds(data(:,3));
        mintheta = mintheta + 0.2*mintheta;
        maxtheta = maxtheta + 0.2*maxtheta;
        s.AxesProperties(1).YLimits = [yMin yMax]; % Changes the limits of the first subplot
        s.AxesProperties(2).YLimits = [yMin yMax]; % Changes the limits of the second subplot
        s.AxesProperties(3).YLimits = [mintheta maxtheta]; % Changes the limits of the third subplot
        % Set our labels
        s.DisplayLabels = splittedLabels;
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    xlabel(t,"\bf{time [s]}",'fontsize',font); 
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font);
    big_label.FontSize = font;

    elseif numberOflines == 8
    t=tiledlayout(3,1);
    nexttile
        TT = array2timetable([data(:,1,2,5,6),zero_vec,zero_vec,zero_vec,zero_vec],...
        'RowTimes', D, ...
        'VariableNames',[splittedLineNames;'zero line';'zero line ';'zero line  ';'zero line   ']);
        s = stackedplot(TT, {[1 5 9],[2 6 10],[3 7 11],[4 8 12]},'LineWidth',linewidth);
        % Change the line style and Colors
        s.LineProperties(1).LineStyle = {'-','-.','--'};
        s.LineProperties(1).Color = [colors(1:2,:);[0 0 0]];
        s.LineProperties(2).LineStyle ={'-','-.','--'};
        s.LineProperties(2).Color = [colors(3:4,:);[0 0 0]];
        s.LineProperties(3).LineStyle = {'-','-.','--'};
        s.LineProperties(3).Color =[colors(5:6,:);[0 0 0]];
        s.LineProperties(4).LineStyle = {'-','-.','--'};
        s.LineProperties(4).Color =[colors(7:8,:);[0 0 0]];
        % Change yticks
        s.AxesProperties(1).YLimits = [yMin yMax]; % Changes the limits of the first subplot
        s.AxesProperties(2).YLimits = [yMin yMax]; % Changes the limits of the second subplot
        s.AxesProperties(3).YLimits = [yMin yMax]; % Changes the limits of the third subplot
        s.AxesProperties(4).YLimits = [yMin yMax]; % Changes the limits of the fourth subplot
        % Set our labels
        s.DisplayLabels = splittedLineNames;
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
        nexttile
            plot(timeVec,data(:,3),'Color',colors(3,:)),grid on
            yline(0,'LineStyle','--','Color','k','LineWidth',1)
            ylabel('\boldmath{$e_{tot} = \sqrt{x^2 + y^2}$}','FontSize',font,'Rotation',0)
            legend(splittedLabels{3},'zero line','FontSize',font)
            ylim([yMin yMax])
        nexttile
            plot(timeVec,data(:,4),'Color',colors(4,:)), grid on
            yline(0,'LineStyle','--','Color','k','LineWidth',1)
            ylabel('\boldmath{$e_{\theta} [rad] $}','FontSize',font,'Rotation',0)
            legend(splittedLabels{4},'zero line','FontSize',font)

    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    xlabel(t,"\bf{time [s]}",'fontsize',font); 
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font);
    big_label.FontSize = font;

    else 
        disp("error")
    end

    grid on
    
end 