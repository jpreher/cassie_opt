function [ params ] = remap_symmetric_param( params, behavior, logger )

ndomain = numel(logger);

assert(ndomain == numel(params), 'There is a size mismatch in remapping the parameters.');

for i = 1:ndomain
    p = fields(params{i,1});
    
    % Step through the fields and remap
    param = struct();
    for j = 1:numel(p)
        pj = p{j};
                
        if ~isempty(strfind(pj,'Left'))
            pj = strrep(pj,'Left','Right');
        elseif ~isempty(strfind(pj,'Right'))
            pj = strrep(pj,'Right','Left');
        end
        
        param.(pj) = params{i}.(p{j});
    end
    
    % Remap outputs if exist
    if isfield(params{i}, 'pposition')
        % Assumes node is every other domain
        domain = behavior.hybridSystem.Gamma.Nodes.Domain{i};
        
        % Flip all non-pitch outputs
        degree = domain.VirtualConstraints.position.PolyDegree;
        
        labels = domain.VirtualConstraints.position.OutputLabel;

        yawOutput = strfind(labels, 'Yaw');
        rollOutput = strfind(labels, 'Roll');
        posYOutput = strfind(labels, 'PosY');
        
%         yawIndices  = find(~cellfun(@isempty,yawOutput));
%         rollIndices = find(~cellfun(@isempty,rollOutput));
%         YposIndices = find(~cellfun(@isempty,posYOutput));

        apos_fixed = reshape(params{i}.aposition, domain.VirtualConstraints.position.Dimension, degree+1);
        
%         apos_fixed(yawIndices,:)  = -apos_fixed(yawIndices,:);
%         apos_fixed(rollIndices,:) = -apos_fixed(rollIndices,:);
%         apos_fixed(YposIndices,:) = -apos_fixed(YposIndices,:);

        param.aposition = apos_fixed(:);
        
        param.pvelocity       = params{i}.pposition;
        params{i,1}.pvelocity = params{i}.pposition;
    end
    
    if isfield(params{i}, 'name')
        name = params{i}.name;
        if ~isempty(strfind(name,'Left'))
            param.name = strrep(name,'Left','Right');
        elseif ~isempty(strfind(name,'Right'))
            param.name = strrep(name,'Right','Left');
        end
    end
    
    if isfield(param, 'x0')
        param = rmfield(param, 'x0');
    end
    
    % Assign
    params{ndomain+i,1} = param;
end

end

