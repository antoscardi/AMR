function plot_function(data, title_name, labels_names, lineNames, timeVec, linewidth, colors, counter)
    % Utility function for the stacked plots.
    persistent n
    if isempty(n)
        n = 0;
    end
    % The inserted string with the names of the labels is splitted and the number of "lines" of plots is counted.
    splittedLabels = split(labels_names,';'); number = length(splittedLabels);
    splittedLineNames = split(lineNames,';');  
    figure(n+5), hold off,
    if size(data,1) > 4
        len = length(timeVec) -1;
        D = milliseconds(0:len)*10;
        TT = array2timetable(data,...
                           'RowTimes', D, ...
                            'VariableNames',splittedLineNames);
        s = stackedplot(TT, {[1,4],[2 5],[3 6]},'LineWidth',linewidth);
        % Change the line style and Colors
        s.LineProperties(1).LineStyle = {'-','-.'};
        s.LineProperties(1).Color = colors(1:2,:);
        s.LineProperties(2).LineStyle = {'-','-.'};
        s.LineProperties(2).Color = colors(3:4,:);
        s.LineProperties(3).LineStyle = {'-','-.'};
        s.LineProperties(3).Color = colors(5:6,:);
        % Set our labels
        s.DisplayLabels = splittedLabels;
        % Update counter for the next time this function is called.
        counter = counter + size(data,2);
        n = n + 1;
    else
         s = stackedplot(timeVec, data', 'LineWidth',linewidth);
         % Adding the labels and the grid.
         s.DisplayLabels = splittedLabels;
        % Changing color for each line of polt.
        for i = counter:counter + number -1
        s.LineProperties(i-counter+1).Color = colors(i,:);
        end
        % Update counters for the next time this function is called.
        counter = counter + number; % for colors 
        n = n + 1; % for figure number
    end
    grid on
    % Changing the size of all text and set title.
    fontsize(10, 'points');
    xlabel("time [s]"); s.Title = title_name;
   
end 