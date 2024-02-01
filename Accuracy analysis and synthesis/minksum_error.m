function [k,convexHullPoints,AB] = minksum_error(v1,v2)

% % 如果v1，v2过小，就避免添加v1和v2
% if norm(v1) < 1e-10
%     v1 = zeros(2,3);
% end
% if norm(v2) < 1e-10
%     v2 = zeros(2,3);
% end
A = v1;  % "Points" as x and y coordinates
B = v2;
sA = size(A);
sB = size(B);
AB = reshape(A, [1, sA]) + reshape(B, [sB(1), 1, sB(2)]);
AB = reshape(AB, [], 3);

    if size(AB,1) == 4
        k = [1 2 4 3];
        convexHullPoints = AB;
    else
        k = convhull(AB);
        convexHullPoints = AB(unique(k(:)), :);
    end
end