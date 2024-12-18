function ee = errorEllipse(C, mu, conf)
    % ERROR_ELLIPSE - Plots an error ellipse for a 2D Gaussian distribution
    % C - Covariance matrix
    % mu - Mean (center) of the distribution
    % conf - Confidence interval (e.g., 0.95 for 95% confidence)

    if nargin < 3
        conf = 0.9973;
    end
    if nargin < 2
        mu = [0, 0];
    end

    % Chi-square value for the given confidence interval
    s = -2 * log(1 - conf);
    
    [V, D] = eig(C * s);
    
    % Create an ellipse with unit radius
    t = linspace(0, 2*pi, 100);
    a = cos(t);
    b = sin(t);
    ellipse = [a; b]' * sqrt(D) * V';
    
    % Ensure mu is a row vector
    mu = mu(:)';

    % Shift the ellipse to the mean
    ee = ellipse + repmat(mu, size(ellipse, 1), 1);
end
