function rel = extractXrel(xrel, numvehicle)
%% Function to reorder the relative mesurements array to have a simple extraction of data

rel = [];

% Trimming the entire data to bearing angle measurements
idx = cat(1, 1:4:4*nchoosek(numvehicle,2), 2:4:4*nchoosek(numvehicle,2));
z_b_all = xrel(:,cat(1, idx(:), []));
clear idx

z_b = extractBear(z_b_all, numvehicle);

% Trimming the entire data to distance measurements
z_d = xrel(:,3:4:end);

% Trimming the entire data to orientation measurements
z_o = xrel(:,4:4:end);



rel = [z_b, z_d, z_o];

function z_b = extractBear(z_b_all, num)

z_b = [];

% Initializing related to single vehicle matrices
ZB = cell(1,num);
% for i=1:num
%    ZB{i} = zeros(size(z_b_all, 1), 2*nchoosek(num,2)/num);
% end
% clear i

% 
z_b_temp = z_b_all;

for i=num-1:-1:1
    
    for k=2:2:i*2
        ZB{i+1}(:,size(ZB{i+1},2)+1) = z_b_temp(:,k);
%         ZB{k/2}(:,k/2+(num-1)-i) = z_b_temp(:,k/2);
        ZB{k/2}(:,size(ZB{k/2},2)+1) = z_b_temp(:,k-1);
    end
    clear k
    
%     ZB{1}(:,num-i) = z_b_temp(:,1);
    
    
%     for j=1:2:i*2
%         ZB{(j+1)/2}(:,) = z_b_temp(:,j);
%     end
%     clear j
    
    
    % Trimming z_b_all 
    z_b_temp = z_b_temp(:,i*2+1:end);
    
end


% res = z_b_all;