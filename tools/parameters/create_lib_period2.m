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

        % Right single support
        [a_v, a_ddq, a_f] = get_logger_fits(data.logger(1));
        lib{i-2}.right.aposition = params{1}.aposition';
        lib{i-2}.right.pposition = fliplr(params{1}.pposition');
        lib{i-2}.right.avelocity = a_v(:)';
        lib{i-2}.right.addq      = a_ddq(:)';
        lib{i-2}.right.af        = a_f(:)';
        lib{i-2}.right.x0        = params{1}.x0';

        % Left single support
        [a_v, a_ddq, a_f] = get_logger_fits(data.logger(2));
        lib{i-2}.left.aposition = params{2}.aposition';
        lib{i-2}.left.pposition = fliplr(params{2}.pposition');
        lib{i-2}.left.avelocity = a_v(:)';
        lib{i-2}.left.addq      = a_ddq(:)';
        lib{i-2}.left.af        = a_f(:)';
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

function [a_v, a_ddq, a_f] = get_logger_fits(log)
    ts = log.flow.t - log.flow.t(1);
    a_v = [...
        get_a_fit( ts, 6,  log.flow.states.dx(1,:) );
        get_a_fit( ts, 6,  log.flow.states.dx(2,:) )];
    a_ddq = [];
    for i = 1:length(log.flow.states.x(:,1))
        a_ddq = [a_ddq;
            get_a_fit( ts, 6,  log.flow.states.ddx(i,:) )];
    end

    a_f = [];
    for i = 1:2
        a_f = [a_f;
            get_a_fit( ts, 6,  log.flow.inputs.ConstraintWrench.fSpringTransmissions(i,:) )];
    end
    for i = 1:5
        if isfield(log.flow.inputs.ConstraintWrench, 'fRightSole')
            a_f = [a_f;
                get_a_fit( ts, 6,  log.flow.inputs.ConstraintWrench.fRightSole(i,:) )];
        elseif isfield(log.flow.inputs.ConstraintWrench, 'fLeftSole')
            a_f = [a_f;
                get_a_fit( ts, 6,  log.flow.inputs.ConstraintWrench.fLeftSole(i,:) )];
        end
    end
end


