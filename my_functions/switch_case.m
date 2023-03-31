function [choosenCase, perturbed_params] = switch_case(wheelDistance, wheelRadius)
%% Function that makes you choose from the Command Window which of the two cases you want to use for
% the perturbed value of the parameters you need.
% Generate uniform distribution and sample the exstremes: 80% or 120% of the nominal value
wheelRadius80 = wheelRadius*0.8;
wheelRadius120 = wheelRadius*1.2;
wheelDistance80 = wheelDistance*0.8;
wheelDistance120 = wheelDistance*1.2;

% Choose if you want to see the case in which the params are the 80% or the 120% of the nominal values.
choose = input('Enter either 80 or 120: ');
switch choose
    case 80
        %% FIRST CASE
        choosenCase = 80;
        perturbed_params = [wheelRadius80; wheelDistance80];
        disp('You have choosen to use 80% of the nominal value')
    case 120
        %% SECOND CASE
        choosenCase= 120;
        perturbed_params = [wheelRadius120; wheelDistance120];
        disp('You choose to use 120% of the nominal value')
    otherwise
        disp('You did not choose anything')
        [choosenCase, perturbed_params] = switch_case(wheelDistance,wheelRadius);
end
end

