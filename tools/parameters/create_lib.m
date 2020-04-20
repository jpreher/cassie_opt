function[lib] = create_lib(paths)

if nargin < 1
    paths = '.';
end

list = dir(strcat(paths,'/params/stepping_v4'));
lib = struct();
lib.avelocity = [];
lib.aposition = [];
lib.pposition = [];
lib.x0 = [];

for i = 3:length(list)
    load(strcat(pwd,'/params/stepping_v4/', list(i).name, '/optData.mat'));
    params = data.params{1,1};
    full_name = strsplit(list(i).name,'_');
    vd_x = sscanf(full_name{3}, '%f');
    vd_y = sscanf(full_name{4}, '%f');
    
    lib.avelocity = [lib.avelocity; vd_x vd_y params.avelocity'];
    lib.aposition = [lib.aposition; vd_x vd_y params.aposition'];
    lib.pposition = [lib.pposition; vd_x vd_y params.pposition'];
    lib.x0 = [lib.x0; vd_x, vd_y , params.x0'];
end

hws = get_param('gaitLib', 'modelworkspace');
apos_num = 72;
ppos_num = 2;
x0_num = 44;

for_vel = unique(lib.avelocity(:,1));
sid_vel = unique(lib.avelocity(:,2));

for i = 1:length(for_vel)
    opts = find(~(lib.avelocity(:,1)-for_vel(i)));
    for j = 1:length(sid_vel)
        ind = find(~(lib.avelocity(opts,2)-sid_vel(j)));
        avel_raw(i,j) = lib.avelocity(opts(ind), 3);
        for k = 1:apos_num
            apos_raw(i,j,k) = lib.aposition(opts(ind), k+2);
        end
        for k = 1:ppos_num
            ppos_raw(i,j,k) = lib.pposition(opts(ind), k+2);
        end
        for k = 1:x0_num
            x0_raw(i,j,k) = lib.x0(opts(ind), k+2);
        end
    end
end
        

hws.assignin('avel_raw', avel_raw)
hws.assignin('apos_raw', apos_raw)
hws.assignin('ppos_raw', ppos_raw)
hws.assignin('x0_raw', x0_raw)



end