fig = figure; axis equal
hold on; grid on;


y_1=Costs(:,1);
y_2=Costs(:,2);
y_3=Costs(:,3);
y_4=Costs(:,4);
x=1:length(y_1);
plot(x,y_1,x,y_2, x,y_3, x,y_4);
legend('grounding cost','control deviation cost','chattering cost', 'dynamic obstacle cost');
xticks([1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39])
xticklabels({'-90, 1','-75','-60','-45','-30','-15','0','15','30','45','60','75','90','-90, 0.5','-75','-60','-45','-30','-15','0','15','30','45','60','75','90','-90, 0','-75','-60','-45','-30','-15','0','15','30','45','60','75','90',})
xlabel('control behaviour number n');
ylabel('cost value');
grid