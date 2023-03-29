function plot_function(data, title_name, labels_names, timeVec, linewidth, colors, counter)
    % Utility function for the stacked plots.
  
    % The inserted string with the names of the labels is splitted and the number of "lines" of plots is counted.
    splittedLabels = split(labels_names,';'); number = length(splittedLabels);   
    figure(), 
    s = stackedplot(timeVec, data', 'LineWidth',linewidth);
    % Adding the labels and the grid.
    s.DisplayLabels = splittedLabels; grid on
    % Changing the size of all text and set title.
    fontsize(10, 'points'), xlabel("time [s]"); s.Title = title_name;
    % Changing color for each line of polt.
    for i = counter:counter + number -1
    s.LineProperties(i-counter+1).Color = colors(i,:);
    end 
    % Update counter for the next time this function is called.
    counter = counter + number;

end 