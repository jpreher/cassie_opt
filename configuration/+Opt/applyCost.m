%% Function: applyCost
%
% Description: Applies a desired cost function to continous domains of the
%   provided nlp. "CostType" should be a cell array of text strings
%   corresponding to the desired cost seen in the switch statement below.
%
% _________________________________________________________________________

function [ nlp ] = applyCost( behavior, nlp, CostType, weight, vars )

% Common variables
x  = behavior.robotModel.States.x;
dx = behavior.robotModel.States.dx;
vhip = dx('BasePosX');
mg   = 9.81 * sum([behavior.robotModel.Links(:).Mass]); 

% Assign cost to each vertex
vertices = fields(behavior.vertices);
for i = 1:numel(vertices)
    phaseName = behavior.vertices.(vertices{i}).Name;
    phaseIndex = nlp.getPhaseIndex(phaseName);
    phase = nlp.Phase(phaseIndex);
    domain = phase.Plant;
    
    u  = domain.Inputs.Control.u;
    Be = domain.Gmap.Control.u;
%     p  = domain.Params.pposition;
%     a = SymVariable(tomatrix(domain.Params.aposition(:))); %a  = domain.Params.aposition;

    switch CostType{i}
        case 'mCOT'
            cot     = sqrt(sum((tovector(u)*(Be'*dx)).^2)).^2 / (mg * vhip);
            cot_fun = SymFunction(['cot_' phase.Name], cot, {u, dx});
            addRunningCost(nlp.Phase(phaseIndex), cot_fun, {'u', 'dx'});
            
        case 'TorqueSquare'
            u2 = weight.* tovector(norm(u.*[16, 16, 25, 25, 50, 16, 16, 25, 25, 50]').^2);
            u2_fun = SymFunction(['torque_', phase.Name], u2, {u, dx});
            addRunningCost(nlp.Phase(phaseIndex), u2_fun, {'u', 'dx'});
            
        case 'SpringSquare'
            u2 = weight.* ( dx('LeftShinPitch').^2 + dx('LeftAchillesSpring').^2 + dx('RightShinPitch').^2 + dx('RightAchillesSpring').^2 );
            u2_fun = SymFunction(['spring_movement_', phase.Name], u2, {dx});
            addRunningCost(nlp.Phase(phaseIndex), u2_fun, {'dx'});
            
        case 'TorqueSquarePlanar'
            u2 = weight.* tovector(norm(u.*[1, 1, 1, 25, 25, 50]').^2);
            u2_fun = SymFunction(['torque_', phase.Name], u2, {u, dx});
            addRunningCost(nlp.Phase(phaseIndex), u2_fun, {'u', 'dx'});
            
        case 'Zero'
            Czero = 0;
            Czero_fun = SymFunction('Czero', Czero, {x});
            addRunningCost(nlp.Phase(phaseIndex), Czero_fun, {'x'});
            
        case 'SLIP'
            tau = domain.VirtualConstraints.position.PhaseFuncs{1};
            slip_params = load('cassie_slip');
            a_slip_bezier  = slip_params.slip_a_mat;
            
            pCOMslip  = bezier(a_slip_bezier, tau);
            pCOMrobot = domain.getComPosition';
            
            SLIP_norm = (pCOMrobot(1) - pCOMslip(1)).^2 + ...
                        (pCOMrobot(3) - pCOMslip(2)).^2;
            SLIP_fun = SymFunction(['slip_' phase.Name], SLIP_norm, {x, p});
            addRunningCost(nlp.Phase(phaseIndex), SLIP_fun, {'x', 'pposition'});
            
        case 'BaseMovement'
            if nargin < 5
                vars = [0;0];
            end
            
            if strcmp(behavior.robotModel.Joints(6).Name, 'BaseRotZ')
                qbIndices = 1:6;
                auxdata = [vars; zeros(4,1)];
            else
                qbIndices = 1:3;
                auxdata = [vars;0];
            end
            
            vd = SymVariable('vd',[length(auxdata),1]);
            
            baseMov = tovector(sum((weight.*(vd - dx(qbIndices))).^2));
            baseMovFun = SymFunction(['BaseMovement_', phase.Name], baseMov, {dx}, {vd});
            addRunningCost(nlp.Phase(phaseIndex), baseMovFun, {'dx'}, {auxdata});
            
        case 'NSFMovement'
            if isempty(strfind(domain.Name, 'Right'))
                % stanceFoot = 'Left';
                p_nsf = getCartesianPosition(domain, domain.ContactPoints.RightSole);
            else
                % stanceFoot = 'Right';
                p_nsf = getCartesianPosition(domain, domain.ContactPoints.LeftSole);
            end
            v_nsf = jacobian(p_nsf, x) * dx;
            %v_nsf(3)=[]; %-don't care nsf_velZ
            
            T = 0.4;
            aux_data = [vars;0];
            vd = SymVariable('vd',[length(aux_data),1]);
            nsf_vel_norm = tovector(sum(((vd*2/T - v_nsf)).^2));
            nsf_vel_norm_Fun = SymFunction(['NSFMovement_', phase.Name], nsf_vel_norm, {x, dx}, {vd});
            addRunningCost(nlp.Phase(phaseIndex), nsf_vel_norm_Fun, {'x','dx'}, {aux_data});
            
        case 'Movement'
            baseWeight = [0; 0; 1; 0; 0; 0];
            FeetWeight = [1; 1; 1];
            
            baseMov = dx(1:6).*baseWeight;
            
            p_nsf = getCartesianPosition(domain, domain.ContactPoints.LeftMidFoot);
            v_nsf = jacobian(p_nsf, x) * dx;
            nsfMove = v_nsf.*FeetWeight;
            
            Mov = tovector( sum(baseMov.^2) + sum(nsfMove.^2) );
            Mov_Fun = SymFunction(['Movement_', phase.Name], Mov, {x, dx});
            addRunningCost(nlp.Phase(phaseIndex), Mov_Fun, {'x','dx'});
            
        case 'ROMcontrol'
            nodeLength = nlp.Phase(phaseIndex).NumNode;
            u_dist_norm_fun = nlp.Phase(phaseIndex).ConstrTable.u_dist_norm_RightSS(1).SymFun;
            for kk = 1:4:nodeLength
                aux_data = {kk, nodeLength};
                addNodeCost(nlp.Phase(phaseIndex), u_dist_norm_fun, ...
                            {'x','dx', 'pposition', 'aposition', 'u', 'T', 'prom'},...
                            kk, aux_data);
            end
        case 'NSF_x'
            p_nsf = getCartesianPosition(domain, domain.ContactPoints.LeftSole);
            p_x = p_nsf(1).^2;
            p_x_Fun = SymFunction(['NSF_x_', phase.Name], p_x*weight, {x});
            addRunningCost(nlp.Phase(phaseIndex), p_x_Fun, {'x'});
        case 'Orientation'
            orientation_fun = SymFunction(['orientation_center_', phase.Name], weight(1)*(x(4)).^2 + weight(2)*(x(5)).^2 + weight(3)*(x(6)).^2, {x});
            addRunningCost(nlp.Phase(phaseIndex), orientation_fun, {'x'});
        case 'StanceKneeSpeed'
            if isempty(strfind(domain.Name, 'Right'))
                stanceKnee = dx('RightKneePitch');
            else
                stanceKnee = dx('LeftKneePitch');
            end
            stanceKneeSpeed_fun = SymFunction(['stance_knee_speed_', phase.Name], weight * stanceKnee.^2, {dx});
            addRunningCost(nlp.Phase(phaseIndex), stanceKneeSpeed_fun, {'dx'});
        case 'StanceMoment'
            if isempty(strfind(domain.Name, 'Right'))
                F  = behavior.vertices.l_SS.HolonomicConstraints.LeftSole.Input;
                stance_moment = SymVariable('fSole', [5,1]);
                stanceKneeSpeed_fun = SymFunction(['stance_moment_', phase.Name], weight * F(5), {F});
                addRunningCost(nlp.Phase(phaseIndex), stanceKneeSpeed_fun, {'fLeftSole'});
            else                
                F  = behavior.vertices.r_SS.HolonomicConstraints.RightSole.Input;
                stance_moment = SymVariable('fSole', [5,1]);
                stanceKneeSpeed_fun = SymFunction(['stance_moment_', phase.Name], weight * F(5), {F});
                addRunningCost(nlp.Phase(phaseIndex), stanceKneeSpeed_fun, {'fRightSole'});
            end
            
    end
end

end