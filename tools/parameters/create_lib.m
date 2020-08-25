function [lib] = create_lib(paths)
    if nargin < 1
        paths = '.';
    end
    list = dir(strcat(paths,'/params/stepping'));
    lib = {struct()};

    for i = 3:length(list)
        load(strcat(pwd,'/params/stepping/', list(i).name, '/optData.mat'));
        params = data.params{1,1};
        full_name = strsplit(list(i).name,'_');
        
        [a_v, a_ddq, a_f] = get_logger_fits(data.logger);
        
        lib{i-2}.vd_x = sscanf(full_name{3}, '%f');
        lib{i-2}.vd_y = sscanf(full_name{4}, '%f');
        lib{i-2}.aposition = params.aposition';
        lib{i-2}.pposition = params.pposition';
        lib{i-2}.avelocity = a_v(:)';
        lib{i-2}.addq      = a_ddq(:)';
        lib{i-2}.af        = a_f(:)';
        lib{i-2}.x0 = params.x0';
    end
    
    no_moves = false;
    while ~no_moves
        no_moves = true;
        for i = 1:length(lib)-1
            tmp = lib{i};
            if lib{i+1}.vd_x < tmp.vd_x
                lib{i} = lib{i+1};
                lib{i+1} = tmp;
                no_moves = false;
            end
        end
    end
    
    gaitlib = struct(...
        'vd', zeros(1,length(lib)), ...
        'pposition', zeros(length(lib),length(params.pposition)), ...
        'amat', zeros(length(lib),length(params.aposition)), ...
        'avel', zeros(length(lib),length(lib{1}.avelocity)), ...
        'addq', zeros(length(lib),length(lib{1}.addq)), ...
        'af',   zeros(length(lib),length(lib{1}.af)));
    for i = 1:length(lib)
        gaitlib.vd(i) = lib{i}.vd_x;
        gaitlib.pposition(i,:) = lib{i}.pposition;
        gaitlib.amat(i,:) = lib{i}.aposition;
        gaitlib.avel(i,:) = lib{i}.avelocity;
        gaitlib.addq(i,:) = lib{i}.addq;
        gaitlib.af(i,:) = lib{i}.af;
    end
    
    export_path = strcat(paths,'/params/stepping_v4');
    libName = string(datetime('now', 'Format', 'yyyyMMdd'));
    yaml_write_file(strcat(export_path, '/', libName,'.yaml'), gaitlib);
    save(strcat(export_path, '/', libName), 'gaitlib');
end

function [a_v, a_ddq, a_f] = get_logger_fits(log)
    a_v = [...
        get_a_fit( log.flow.t, 6,  log.flow.states.dx(1,:) );
        get_a_fit( log.flow.t, 6,  log.flow.states.dx(2,:) )];
    a_ddq = [];
    a_f = [];
    for i = 1:length(log.flow.states.x(:,1))
        a_ddq = [a_ddq;
            get_a_fit( log.flow.t, 6,  log.flow.states.ddx(i,:) )];
%         a_f = [a_f;
%             get_a_fit( log.flow.t, 6,  log.flow.states.ddx(i,:) )]; % TODO HERE!
    end
    a_f = [];
end


