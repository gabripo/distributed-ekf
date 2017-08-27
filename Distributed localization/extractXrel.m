function rel = extractXrel(xrel, numvehicle)
%% Function to reorder the relative mesurements array to have a simple extraction of data

rel = [];

% Trimming the entire data to bearing angle measurements, then reorder it
idx = cat(1, 1:4:4*nchoosek(numvehicle,2), 2:4:4*nchoosek(numvehicle,2));
z_b_all = xrel(:,cat(1, idx(:), []));
clear idx

z_b = extractBear(z_b_all, numvehicle);

% Trimming the entire data to distance measurements, then reorder it
z_d_all = xrel(:,3:4:end);

z_d = extractDist(z_d_all, numvehicle);

% Trimming the entire data to orientation measurements, then reorder it
z_o_all = xrel(:,4:4:end);

z_o = extractOri(z_o_all, numvehicle);

% Final result
rel = [z_b, z_d, z_o];

function z_b = extractBear(z_b_all, num)
%% Function to extract the bearing angles between vehicles

% z_b = [];

% Initializing related to single vehicle matrices
ZB = cell(1,num);
% for i=1:num
%    ZB{i} = zeros(size(z_b_all, 1), 2*nchoosek(num,2)/num);
% end
% clear i

% Temporary variable to trim (it seems to be unuseful but it is due to
% debugging purpose)
z_b_temp = z_b_all;

for i=num-1:-1:1
    
    for k=2:2:i*2
        ZB{i+1}(:,size(ZB{i+1},2)+1) = z_b_temp(:,k);
        ZB{k/2}(:,size(ZB{k/2},2)+1) = z_b_temp(:,k-1);
    end
    clear k
    
    % Trimming z_b_temp
    z_b_temp = z_b_temp(:,i*2+1:end);
    
end

% z_b = cell2mat(ZB);
z_b = ZB;

function z_d = extractDist(z_d_all, num)
%% Function to extract the distances between vehicles

% z_d = [];

% Initializing related to single vehicle matrices
ZD = cell(1,num);

z_d_temp = z_d_all;

for i=num-1:-1:1
    
    for j=1:i
        % Main measurement
        ZD{i+1}(:,size(ZD{i+1},2)+1) = z_d_temp(:,j);
        
        % Opposite measurement (supplementar vehicle)
        ZD{j}(:,size(ZD{j},2)+1) = -z_d_temp(:,j);
    end
    clear j
    
    % Trimming z_d_temp
    z_d_temp = z_d_temp(:,i+1:end);
    
end

% z_d = cell2mat(ZD);
z_d = ZD;

function z_o = extractOri(z_o_all, num)
%% Function to extract the distances between vehicles

% z_o = [];

% Initializing related to single vehicle matrices
ZO = cell(1,num);

z_o_temp = z_o_all;

for i=num-1:-1:1
    
    for j=1:i
        % Main measurement
        ZO{i+1}(:,size(ZO{i+1},2)+1) = z_o_temp(:,j);
        
        % Opposite measurement (supplementar vehicle)
        ZO{j}(:,size(ZO{j},2)+1) = -z_o_temp(:,j);
    end
    clear j
    
    % Trimming z_d_temp
    z_o_temp = z_o_temp(:,i+1:end);
    
end

% z_o = cell2mat(ZO);
z_o = ZO;