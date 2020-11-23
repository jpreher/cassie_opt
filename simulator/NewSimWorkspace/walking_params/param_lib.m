function params = param_lib(vdx, vdy, leg, interp_lib)

nvx = size(interp_lib,1);
nvy = size(interp_lib,2);
vd_x = zeros(nvx,1);
vd_y = zeros(nvy,1);

for i = 1:nvx
    vd_x(i) = interp_lib{i,1}.vd_x;
end
for i = 1:nvy
    vd_y(i) = interp_lib{1,i}.vd_y;
end

right = struct();
right.amat_array = {};
right.avelocity_array = {};
right.addq_array = {};
right.af_array = {};
right.x0_array = {};
%
left = struct();
left.amat_array = {};
left.avelocity_array = {};
left.addq_array = {};
left.af_array = {};
left.x0_array = {};

for i = 1:nvx
    for j = 1:nvy
        right.amat_array{i,j} = interp_lib{i,j}.right.aposition;
        left.amat_array{i,j} = interp_lib{i,j}.left.aposition;
        
        right.avelocity_array{i,j} = interp_lib{i,j}.right.avelocity;
        left.avelocity_array{i,j} = interp_lib{i,j}.left.avelocity;
        
        right.addq_array{i,j} = interp_lib{i,j}.right.addq;
        left.addq_array{i,j} = interp_lib{i,j}.left.addq;
        
        right.af_array{i,j} = interp_lib{i,j}.right.af;
        left.af_array{i,j} = interp_lib{i,j}.left.af;
        
        right.x0_array{i,j} = interp_lib{i,j}.right.x0;
        left.x0_array{i,j} = interp_lib{i,j}.left.x0;
    end
end

params = struct();
params.pposition = interp_lib{1,1}.right.pposition;


if strcmp(leg, 'Left')
    amat = bilinear_interp(vd_x, vd_y, left.amat_array, vdx, vdy);
    params.aposition = reshape(amat,9,7);
    
    ddqmat = bilinear_interp(vd_x, vd_y, left.addq_array, vdx, vdy);
    params.ddqd = reshape(ddqmat,22,7);
    
    fdmat = bilinear_interp(vd_x, vd_y, left.af_array, vdx, vdy);
    params.fd = reshape(fdmat,7,7);
    
    params.x0 = bilinear_interp(vd_x, vd_y, left.x0_array, vdx, vdy)';
else
    amat = bilinear_interp(vd_x, vd_y, right.amat_array, vdx, vdy);
    params.aposition = reshape(amat,9,7);
    
    ddqmat = bilinear_interp(vd_x, vd_y, right.addq_array, vdx, vdy);
    params.ddqd = reshape(ddqmat,22,7);
    
    fdmat = bilinear_interp(vd_x, vd_y, right.af_array, vdx, vdy);
    params.fd = reshape(fdmat,7,7);
        
    
    params.x0 = bilinear_interp(vd_x, vd_y, right.x0_array, vdx, vdy)';
    
end


end

function Xj = find_index(X, Xi)
    Xj = 1;
    if (Xi <= X(1))
        Xi = X(1);
        Xj = 1;
    elseif (Xi >= X(length(X)))
        Xi = X(length(X) - 1);
        Xj = length(X) - 1;
    else
        for i = 1:length(X)-1
            if (Xi >= X(i)) && (Xi <= X(i+1))
                Xj = i;
            end
        end
    end
end

function Zi = bilinear_interp(X, Y, Z, Xi, Yi)
    Xj = find_index(X,Xi);
    Yj = find_index(Y,Yi);
    
    % X direction
    SXj = (Xi - X(Xj)) / (X(Xj+1) - X(Xj));
    SYj = (Yi - Y(Yj)) / (Y(Yj+1) - Y(Yj));
    
    Zi = ( Z{Xj,Yj} * (1.0 - SXj) + Z{Xj+1,Yj} * SXj ) * (1.0 - SYj) ...
            + ( Z{Xj,Yj+1} * (1.0 - SXj) + Z{Xj+1,Yj+1} * SXj ) * SYj;
end