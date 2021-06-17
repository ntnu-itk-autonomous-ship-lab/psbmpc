function [r_ellipse, phi] = create_probability_contour(P)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[nrows,ncols] = size(P);

% Scale covariance with inverse Chi-squared value, to get 99% error ellipse
chi_val = chi2inv(0.997, 2);
if nrows > 2
    P_xy = chi_val*[P(1,1) P(1,2); P(2,1) P(2,2)];
else
    P_xy = chi_val*P;
end

[eigenvec, eigenval ] = eig(P_xy);

% Get the index of the largest eigenvector
[largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
if length(largest_eigenvec_ind_c) > 1
    largest_eigenvec_ind_c = largest_eigenvec_ind_c(1);
end
largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

% Get the largest eigenvalue
largest_eigenval = max(max(eigenval));

% Get the smallest eigenvector and eigenvalue
if largest_eigenvec_ind_c == 1
    smallest_eigenval = max(eigenval(:,2));
    smallest_eigenvec = eigenvec(:,2);
else
    smallest_eigenval = max(eigenval(:,1));
    smallest_eigenvec = eigenvec(1,:);
end

% Calculate the angle between the x-axis and the largest eigenvector
angle = atan2(largest_eigenvec(2), largest_eigenvec(1));

% This angle is between -pi and pi.
% Let's shift it such that the angle is between 0 and 2pi
if(angle < 0)
    angle = angle + 2*pi;
end

theta_grid = linspace(0,2*pi);
phi = angle;


a=sqrt(largest_eigenval);
b=sqrt(smallest_eigenval);

% the ellipse in "body" x and y coordinates 
ellipse_x_r  = a*cos( theta_grid );
ellipse_y_r  = b*sin( theta_grid );

R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

% Rotate to NED by angle phi, N_ell_points x 2
r_ellipse = [ellipse_x_r; ellipse_y_r]' * R;
end

