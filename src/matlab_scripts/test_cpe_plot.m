run test_mrou_plot
run test_ownship_plot

figure;
hold on; grid on;
plot(P_c_CE, 'r');
plot(P_c_MCSKF, 'b');
legend('CE', 'MCSKF4D', 'Interpreter', 'latex');kc
