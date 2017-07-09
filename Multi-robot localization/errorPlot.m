function errorPlot(time, real, ekf)

% String cell to write as legend
strCell = cell(0);

% figure(2); clf
for i=1:size(real, 2)/3
   plot(time, ekf(i*3-2,:) - real(:,i*3-2)', time, ekf(i*3-1,:) - real(:,i*3-1)')
   strCell = cat(2, strCell, {strcat('X-error of ',sprintf(' %d', i)), strcat('Y-error of',sprintf(' %d', i))});
   hold on
end
legend(strCell{:,:})
hold off