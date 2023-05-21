function plot_function(data, title_name, main_label,labels_names, lineNames, stackedtitle,timeVec, linewidth, colors,f)
    % Utility function for the stacked plots.
    font = 18; stackedfont = 12;

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
    splitstackedtitle = split(stackedtitle,';');
    % Maximum an Minimum y value for limits
    [yMin,yMax] = bounds([data,zero_vec],'all');
    yMin = yMin - 0.1*yMin;
    yMax = yMax + 0.1*yMax;

    fig = figure(n+6);
    set(gcf, 'Position', get(0, 'Screensize'));
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
        s.XLabel = "";
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
        grid on
    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    xlabel(t,"\bf{time [s]}",'fontsize',font); 
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font+1);
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
        s.XLabel = 'time [s]';
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
        grid on
    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    xlabel(t,"\bf{time [s]}",'fontsize',font); 
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font+1);
    big_label.FontSize = font;
 
    elseif numberOflines == 4
    n = n + 1;
    % Maximum an Minimum y value for limits
    [yMin13,yMax13] = bounds([data(:,1:3),zero_vec],'all')
    yMin13 = yMin13 - 0.1*yMin13;
    yMax13 = yMax13 + 0.1*yMax13;
    t=tiledlayout(4,1);
    nexttile
    plot(timeVec,data(:,1),'Color',colors(1,:)),grid on
    yline(0,'LineStyle','--','Color','k','LineWidth',1)
    ylabel('\boldmath{$e_{x} [m]$}','FontSize',font,'Rotation',0)
    xlabel('time [s]','FontSize',font)
    legend(splittedLabels{1},'zero line','FontSize',stackedfont+2)
    ylim([yMin13 yMax13])
    nexttile
    plot(timeVec,data(:,2),'Color',colors(2,:)),grid on
    yline(0,'LineStyle','--','Color','k','LineWidth',1)
    ylabel('\boldmath{$e_{y} [m]$}','FontSize',font,'Rotation',0)
    xlabel('time [s]','FontSize',font)
    legend(splittedLabels{2},'zero line','FontSize',stackedfont+2)
    ylim([yMin13 yMax13])

    nexttile
        plot(timeVec,data(:,3),'Color',colors(3,:)),grid on
        yline(0,'LineStyle','--','Color','k','LineWidth',1)
        ylabel('\boldmath{$e_{tot} [m]$}','FontSize',font,'Rotation',0)
        xlabel('time [s]','FontSize',font)
        legend(splittedLabels{3},'zero line','FontSize',stackedfont+2)
        ylim([yMin13 yMax13])
        %title('\bf{$e_{tot} = \sqrt{e_{x}^2 + e_{y}^2}$}','FontSize',font)
    nexttile
        plot(timeVec,data(:,4),'Color',colors(4,:)), grid on
        yline(0,'LineStyle','--','Color','k','LineWidth',1)
        ylabel('\boldmath{$e_{\theta} [rad] $}','FontSize',font,'Rotation',0)
        legend(splittedLabels{4},'zero line','FontSize',stackedfont+2)
        xlabel('time [s]','FontSize',font)
        %title('\bf{$e_{\theta}$}','FontSize',font)

    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font+1);
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
        [mintheta,maxtheta] = bounds(data(:,3),'all');
        mintheta = mintheta + 0.2*mintheta;
        maxtheta = maxtheta + 0.2*maxtheta;
        s.AxesProperties(1).YLimits = [yMin yMax]; % Changes the limits of the first subplot
        s.AxesProperties(2).YLimits = [yMin yMax]; % Changes the limits of the second subplot
        s.AxesProperties(3).YLimits = [mintheta maxtheta]; % Changes the limits of the third subplot
        % Set our labels
        s.DisplayLabels = splittedLabels;
        % Font of the stackedplot
        s.FontSize = stackedfont;
        s.XLabel = '';
        % Update figures counter n
        n = n + 1;
        grid on
    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    xlabel(t,"\bf{time [s]}",'fontsize',font); 
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font+1);
    big_label.FontSize = font;

    elseif numberOflines == 8
    [minxytot,maxxytot] = bounds(data(:,[1 2 3 5 6 7]),'all');
    minxytot = minxytot + 0.2*minxytot;
    maxxytot = maxxytot + 0.2*maxxytot;
    t = tiledlayout(4,1);
    t1 = nexttile(1);
        TT = array2timetable([data(:,[1 5]),zero_vec],...
        'RowTimes', D, ...
        'VariableNames',{splittedLineNames{1};splittedLineNames{5};...
                         'zero line'})
        s = stackedplot(TT, {[1 2 3]},'LineWidth',linewidth);

        % Change the line style and Colors
        s.LineProperties(1).LineStyle = {'-','-.','--'};
        s.LineProperties(1).Color = [colors(1:2,:);[0 0 0]];
        % Change yticks
        s.AxesProperties(1).YLimits = [minxytot maxxytot]; % Changes the limits of the first subplot
        % Set our labels
        s.DisplayLabels = splittedLabels(1);
        s.XLabel = 'time [s]';
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
        grid on
        %title('$e_{x}$')
        annotation('textbox',[0.031666666666667,0.804425863991081,0.084,0.074], ...
        'String','\boldmath{$e_{x}$}','EdgeColor','none','FontSize',font+3)

        t2 = nexttile(2);
        TT = array2timetable([data(:,[2 6]),zero_vec],...
        'RowTimes', D, ...
        'VariableNames',{splittedLineNames{2};splittedLineNames{6};...
                         'zero line'})
        s = stackedplot(TT, {[1 2 3]},'LineWidth',linewidth);
        % Change the line style and Colors
        s.LineProperties(1).LineStyle ={'-','-.','--'};
        s.LineProperties(1).Color = [colors(3:4,:);[0 0 0]];
        % Change yticks
        s.AxesProperties(1).YLimits = [minxytot maxxytot]; % Changes the limits of the first subplot
        % Set our labels
        s.DisplayLabels = splittedLabels(2);
        s.XLabel = 'time [s]';
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
        grid on
        %title('$e_{y}$')
        annotation('textbox',[0.032708333333333,0.599297658862876,0.084,0.074], ...
        'String','\boldmath{$e_{y}$}','EdgeColor','none','FontSize',font+3)
        
        t3 = nexttile(3);
        TT = array2timetable([data(:,[3 7]),zero_vec],...
        'RowTimes', D, ...
        'VariableNames',{splittedLineNames{3};splittedLineNames{7};...
                         'zero line'});
        s = stackedplot(TT, {[1 2 3]},'LineWidth',linewidth);
        % Change the line style and Colors
        s.LineProperties(1).LineStyle = {'-','-.','--'};
        s.LineProperties(1).Color = [colors(5:6,:);[0 0 0]];
        % Change yticks
        s.AxesProperties(1).YLimits = [minxytot maxxytot]; % Changes the limits of the first subplot
        % Set our labels
        s.DisplayLabels =splittedLabels(3);
        s.XLabel = 'time [s]';
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
        grid on
        %title(strcat('\bf{',splitstackedtitle{1},'}'))
        annotation('textbox',[0.0353125,0.365183946488294,0.084,0.074], ...
        'String',splitstackedtitle(1),'EdgeColor','none','FontSize',font+3)

        annotation('textbox',[0.0884375,0.424749163879598,0.087083333333334,0.054568561872909], ...
        'String',splitstackedtitle(3),'EdgeColor','none','FontSize',font+3)

        t4 = nexttile(4);
        TT = array2timetable([data(:,[4 8]),zero_vec],...
        'RowTimes', D, ...
        'VariableNames',{splittedLineNames{4};splittedLineNames{8};...
                         'zero line';});
        s = stackedplot(TT, {[1 2 3]},'LineWidth',linewidth);
        % Change the line style and Colors
        s.LineProperties(1).LineStyle = {'-','-.','--'};
        s.LineProperties(1).Color = [colors(7:8,:);[0 0 0]];
        % Change yticks
        [min4,max4] = bounds(data(:,[4 8]),'all');
        min4 = min4 + 0.2*min4;
        max4 = max4 + 0.2*max4;
        s.AxesProperties(1).YLimits = [min4 max4]; % Changes the limits of the first subplot
        % Set our labels
        s.XLabel = 'time [s]';
        s.DisplayLabels = splittedLabels(4);
        % Font of the stackedplot
        s.FontSize = stackedfont;
        % Update figures counter n
        n = n + 1;
        %title(strcat('\bf{',splitstackedtitle{2},'}'))
        annotation('textbox',[0.034270833333333,0.143333333333333,0.084,0.074], ...
        'String',splitstackedtitle(2),'EdgeColor','none','FontSize',font+3)

        annotation('textbox',[0.083229166666667,0.20958751393534,0.087083333333334,0.054568561872909], ...
        'String',splitstackedtitle(4),'EdgeColor','none','FontSize',font+3)

    big_label = ylabel(t,main_label);
    % Changing the size of all text and set title.
    %xlabel(t,"\bf{time [s]}",'fontsize',font); 
    title(t,strcat('\bf{',title_name,'}'),'FontSize',font+1);
    big_label.FontSize = font;

    else 
        disp("error")
    end

    grid on
    
end 