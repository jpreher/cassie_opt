function [lib] = create_lib_period2(paths)
    if nargin < 1
        paths = '.';
    end
    list = dir(strcat(paths,'/params/stepping'));
    lib = {struct()};

    for i = 3:length(list)
        load(strcat(pwd,'/params/stepping/', list(i).name, '/optData.mat'));
        params = data.params;
        full_name = strsplit(list(i).name,'_');

        % Get designed speed
        lib{i-2}.vd_x = sscanf(full_name{3}, '%f');
        lib{i-2}.vd_y = sscanf(full_name{4}, '%f');
        
        fprintf("Iteration %d of %d, speed [%d, %d]\n", i-2, length(list)-2, lib{i-2}.vd_x, lib{i-2}.vd_y);

        % Right single support
        [a_p, a_v, a_ddq, a_u, a_f] = get_logger_fits(data.logger(1));
        lib{i-2}.right.aposition = params{1}.aposition';
        lib{i-2}.right.pposition = fliplr(params{1}.pposition');
        lib{i-2}.right.apbase    = a_p(:)';
        lib{i-2}.right.avelocity = a_v(:)';
        lib{i-2}.right.addq      = a_ddq(:)';
        lib{i-2}.right.af        = a_f(:)';
        lib{i-2}.right.au        = a_u(:)';
        lib{i-2}.right.x0        = params{1}.x0';

        % Left single support
        [a_p, a_v, a_ddq, a_u, a_f] = get_logger_fits(data.logger(2));
        lib{i-2}.left.aposition = params{2}.aposition';
        lib{i-2}.left.pposition = fliplr(params{2}.pposition');
        lib{i-2}.left.apbase    = a_p(:)';
        lib{i-2}.left.avelocity = a_v(:)';
        lib{i-2}.left.addq      = a_ddq(:)';
        lib{i-2}.left.af        = a_f(:)';
        lib{i-2}.left.au        = a_u(:)';
        lib{i-2}.left.x0        = params{2}.x0';
    end

    no_moves = false;
    nested_lib = cell(2,2);
    while ~no_moves
        no_moves = true;

        % Find and sort by x speeds
        for i = 1:length(lib)-1
            tmp = lib{i};
            if lib{i+1}.vd_x < tmp.vd_x
                lib{i} = lib{i+1};
                lib{i+1} = tmp;
                no_moves = false;
            end
        end

        % Now dump into our nested array
        j = 1; k = 1;
        vx = lib{j}.vd_x;

        % Nest each value into (x,y) pairs
        for i = 1:length(lib)
            if lib{i}.vd_x ~= vx
                j = j+1;
                k = 1;
                vx = lib{i}.vd_x;
            end
            nested_lib{j,k} = lib{i};
            k = k+1;
        end
    end

    no_moves = false;
    while ~no_moves
        no_moves = true;

        % Sort the y
        for i = 1:size(nested_lib,1)
            for j = 1:size(nested_lib,2)-1
                tmp = nested_lib{i,j};
                if nested_lib{i,j+1}.vd_y < tmp.vd_y
                    nested_lib{i,j} = nested_lib{i,j+1};
                    nested_lib{i,j+1} = tmp;
                    no_moves = false;
                end
            end
        end
    end
    
    export_path = strcat(paths,'/params/stepping');
    libName = string(datetime('now', 'Format', 'yyyyMMdd'));
    yaml_write_file(strcat(export_path, '/', libName,'.yaml'), nested_lib);
    save(strcat(export_path, '/', libName), 'nested_lib');
end

function [a_p, a_v, a_ddq, a_u, a_f] = get_logger_fits(log)
    M = 6;
    ts = log.flow.t - log.flow.t(1);
    a_v = [...
        get_a_fit( ts, M,  log.flow.states.dx(1,:) );
        get_a_fit( ts, M,  log.flow.states.dx(2,:) )];
    a_ddq = zeros(length(log.flow.states.x(:,1)), M+1);
    a_u = zeros(length(log.flow.inputs.Control.u(:,1)), M+1);
    parfor i = 1:length(log.flow.states.x(:,1))
        a_ddq(i,:) = get_a_fit( ts, M,  log.flow.states.ddx(i,:) );
    end
    parfor i = 1:length(log.flow.inputs.Control.u(:,1))
        a_u(i,:) = get_a_fit( ts, M,  log.flow.inputs.Control.u(i,:) );
    end
    a_u = a_u .* [25; 25; 16; 16; 50; 25; 25; 16; 16; 50];

    a_f = zeros(7,M+1);
    parfor i = 1:2
        a_f(i,:) = get_a_fit( ts, M,  log.flow.inputs.ConstraintWrench.fSpringTransmissions(i,:) );
    end
    
    h_rf = h_RightSole_RightSS(log.flow.states.x(:,1), zeros(5,1));
    h_lf = h_LeftSole_LeftSS(log.flow.states.x(:,1), zeros(5,1));
    parfor i = 1:5
        if isfield(log.flow.inputs.ConstraintWrench, 'fRightSole')
            a_f(i+2,:) = get_a_fit( ts, M,  log.flow.inputs.ConstraintWrench.fRightSole(i,:) );
        elseif isfield(log.flow.inputs.ConstraintWrench, 'fLeftSole')
            a_f(i+2,:) = get_a_fit( ts, M,  log.flow.inputs.ConstraintWrench.fLeftSole(i,:) );
        end
    end
    if isfield(log.flow.inputs.ConstraintWrench, 'fRightSole')
        a_p = [...
                get_a_fit( ts, M,  log.flow.states.x(1,:) - h_rf(1) );
                get_a_fit( ts, M,  log.flow.states.x(2,:) - h_rf(2) )];
    else
        a_p = [...
                get_a_fit( ts, M,  log.flow.states.x(1,:) - h_lf(1) );
                get_a_fit( ts, M,  log.flow.states.x(2,:) - h_lf(2) )];
    end
end


