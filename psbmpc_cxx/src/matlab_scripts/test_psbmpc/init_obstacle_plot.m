
th = 0 : 0.01 : 2.01 * pi;
x_a = 20 * cos(th); y_a = 20 * sin(th);

disp(WPs_i);
plot(ax1, WPs_i(2, :), WPs_i(1, :), 'mx', 'Linewidth', 2);

h_X_i{i} = [];
h_X_i_ptch{i} = [];
h_P_i{i} = [];
h_d_i{i} = [];

legend_strs = strings(n_obst + 1, 1);
legend_strs(1) = 'Safe dist.';
count = 2;
if i == n_obst
    for i = 1 : n_obst
        legend_strs(count) = ['i=' num2str(i)]; 
        count = count + 1;
    end
end

