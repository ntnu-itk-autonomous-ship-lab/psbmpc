figure; axis equal
hold on; 
grid on;

P(P == -1) = NaN;
test_polygon=polyshape(P);

plot(test_polygon);