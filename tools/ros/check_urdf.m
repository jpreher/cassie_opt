function [ ] = check_urdf( urdf_path )
% Function for checking the validity of an URDF file.
% This function will halt your program with an error if the model is
% invalid.
res = system(sprintf('check_urdf %s\n', urdf_path));

if (res > 0)
   error('The URDF file you specified is not valid!');
end

end

