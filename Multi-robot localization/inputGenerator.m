function u = inputGenerator(num, funV, funOmega, option)
%% Function to generate the u vector of inputs, funV and funOmega are matlab functions

% Initialization of the cell array to return
u = {};

% Same inputs for all
if strcmp(option, 'same')
    for i=1:num
        u{1,i} = funV;
        u{2,i} = funOmega;
    end
end

% Random inputs
if strcmp(option, 'random')

    range = [1,2];
    kV = range(1) + (range(2)-range(1))*rand(num,1);
    kOmega = range(1) + (range(2)-range(1))*rand(num,1);
    
    for i=1:num
       u{1,i} = @(t) kV(i)*funV(t); 
       u{2,i} = @(t) kOmega(i)*funOmega(t);
    end
    
    
end

% HARDCODED INPUTS (deprecated)
% SimSets.u{1,1} = @(t) 1;
% SimSets.u{1,2} = @(t) 1;
% % SimSets.u{1,3} = @(t) 1;
% SimSets.u{2,1} = @(t) 2*sin(2*pi*t/10).*cos(2*pi*t/2);
% SimSets.u{2,2} = @(t) 2*sin(2*pi*t/10).*cos(2*pi*t/2);
% % SimSets.u{2,3} = @(t) 2*sin(2*pi*t/10).*cos(2*pi*t/2);