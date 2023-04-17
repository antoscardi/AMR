function plot_function(data, title_name, labels_names, timeVec, linewidth, colors, counter)
    % Utility function for the stacked plots.
    persistent n
    if isempty(n)
        n = 0;
    end
    % The inserted string with the names of the labels is splitted and the number of "lines" of plots is counted.
    splittedLabels = split(labels_names,';'); number = length(splittedLabels); 
    doTable = true;
    figure(n+5), hold off,
    if doTable == true
    %if strcmp(class(data),'table')
        % Splitting data in 3 tables
        % splitdata = splitvars(data,'Var1','NewVariableNames',{'a', 'b', 'c', 'd', 'e', 'f'})
        % table1 = mergevars(splitdata, [1 4]);
        % table2 = mergevars(table1, [2 4]);
        % table3 = mergevars(table2, [3 4])
        % table3.Properties.RowNames = {'a','b','c', 'd', 'e', 'f'}
        % s = stackedplot(table3, 'LineWidth',linewidth);
        % %s.DisplayLabels = splittedLabels;
        % s.LineProperties(1).LineStyle={'-.','-'};
        % s.LineProperties(2).LineStyle={'-.','-'};
        % s.LineProperties(3).LineStyle={'-.','-'};
        % s.LineProperties(1).Color = [colors(1,:);colors(2,:)];
        % s.LineProperties(2).Color = [colors(3,:);colors(4,:)];
        % s.LineProperties(3).Color = [colors(5,:);colors(6,:)];
        % s.DisplayLabels = splittedLabels;
        D = milliseconds(0:2000)*10;
        TT = array2timetable(data,...
                           'RowTimes', D, ...
                            'VariableNames', {'x_NOPT_PERT','y_NOPT_PERT','theta_NOPT_PERT','x_OPT_PERT', 'y_OPT_PERT', 'theta_OPT_PERT'});
        s = stackedplot(TT, {[1,4],[2 5],[3 6]})
        s.LineProperties(1).LineStyle={'-.','-'};
        s.LineProperties(2).LineStyle={'-.','-'};
        s.LineProperties(3).LineStyle={'-.','-'};
        %s.LineProperties(1).Color = [[1, 0, 0];[0, 0.5, 0]];
        %s.LineProperties(2).Color = [[1, 0, 0];[0, 0.5, 0]];
        %s.LineProperties(3).Color = [[1, 0, 0];[0, 0.5, 0]];
        % To specify x-ticks 
        
        s.DisplayLabels = splittedLabels;
    else
         s = stackedplot(timeVec, data', 'LineWidth',linewidth);
         % Adding the labels and the grid.
         s.DisplayLabels = splittedLabels;
    end
    grid on
    % Changing the size of all text and set title.
    %fontsize(10, 'points');
    xlabel("time [s]"); s.Title = title_name;
    % Changing color for each line of polt.
    for i = counter:counter + number -1
    s.LineProperties(i-counter+1).Color = colors(i,:);
    end 
    % Update counter for the next time this function is called.
    counter = counter + number;
    n = n + 1;
end 