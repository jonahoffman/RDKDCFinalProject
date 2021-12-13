function SE3 = create_homog(R, p)
    SE3 = zeros(4,4);
    SE3(4,4) = 1;
    SE3(1:3, 1:3) = R;
    SE3(1:3, 4) = p;
end
    