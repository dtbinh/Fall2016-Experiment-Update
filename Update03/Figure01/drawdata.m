% draw data
% % estimation
plot(filterOut(:,6),'color',colorKhepera(1,:));
hold on
grid on
[lengthData,~] = size(DataSet);
text(lengthData,filterOut(lengthData,6),num2str(filterOut(lengthData,6)));
% 
% error

% plot(filterOut(:,5),'color',colorKhepera(1,:));
% hold on
% grid on
