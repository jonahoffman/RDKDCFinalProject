function mu = manipulability(J, measure)
% Outputs the manipulability calculated dependendent on the chosen measure.
% Options: sigmamin, detjac, invcond. 
    
    % Find singular values of the Jacobian:
    S = svd(J);
    if strcmpi(measure, 'sigmamin')
        % Minimum singular value:
        mu = min(S); 
    elseif strcmpi(measure, 'detjac')
        % Determinant of the jacobian:
        mu = det(J);
    elseif strcmpi(measure, 'invcond')
        % Formula for the inverse condition:
        mu = min(S) ./ max(S); 
    else
        % Only accept the three defined strings:
        disp("Invalid manipulability measure.");
    end
end 