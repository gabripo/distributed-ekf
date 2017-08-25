function comparePlot(real, ekf, strReal, strEkf)

% String cell to write as legend
strCell = cell(0);

% co = [0 0 1;
%       0 0 1;
%       1 0 0;
%       1 0 0;
%       0 1 0
%       0 1 0];
% set(groot,'defaultAxesLineStyleOrder',{'-','--'},...
%     'defaultAxesColorOrder',[0 0 1])
% figure(1); clf;
% hold on
% set(gca,'linestyleorder',{'-',':','-',':','-',':'})
% set(gca,'colororder', co)
for i=1:size(real, 2)/3
   plot(real(:,i*3-2), real(:,i*3-1), ekf(i*3-2,:), ekf(i*3-1,:));
   strCell = cat(2, strCell, {strcat(strReal,sprintf(' %d', i)), strcat(strEkf,sprintf(' %d', i))});
   hold all
end
legend(strCell{:,:})
hold off
% set(groot,'defaultAxesLineStyleOrder','remove')
% set(groot,'defaultAxesColorOrder','remove')