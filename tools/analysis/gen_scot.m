%% function: gen_scot
%
% Description: Generates the specific cost of transport(SCOT) for given
% data.
%
% Author: Jenna Reher, (jreher@caltech.edu)
% _____________________________________________________________________

function [scot_pos,scot_abs,scot_sum ] = gen_scot(record_domains, model, controller, calcs_steps)


ref = Ref();
ref.h = struct();
ref.h.calcs = {};
ref.h.calc = {};


odeopts = odeset('MaxStep', 1e-2, 'OutputFcn', @outputfcn, ...
    'RelTol',1e-6,'AbsTol',1e-6);

[nDomains, nSteps] = size(calcs_steps);

dt = 0.001;

mass = model.getTotalMass;
grav = 9.81;

scot_abs = zeros(2,nSteps);
scot_pos = zeros(2,nSteps);
scot_sum = zeros(2,nSteps);



for k = 1:nSteps
    calcs = cell(nDomains/2,1);
    for j = 1:(nDomains/2)
        % right leg stance
        t0 = calcs_steps{j,k}.t(1);
        tf = calcs_steps{j,k}.t(end);
        
        ref.h.calcs = {};
        ref.h.calc = {};
        
        
        tspan = t0:dt:tf;
        if tspan(end) == tf
            tspan(end+1) = tf;
        end
        
        
        current_domain = record_domains{j,k};
        x0 = calcs_steps{j,k}.x(:,1);
        sol = ode113(@(t, x, ref) calcDynamics(current_domain, t, x, model, controller, ref), ...
            tspan, x0, odeopts, ref);
        
        
        calcs{j} = horzcat_fields(cell2mat(ref.h.calcs));
        
        
    end
    
    out = horzcat_fields_domains([calcs{:}],true);
    
    pcom_0 = pe_com_vec(out.x(:,1));
    pcom_f = pe_com_vec(out.x(:,end));
    
    dist = pcom_f(1) - pcom_0(1);
    
    numSample = numel(out.t);
    pnorm_abs = zeros(numSample,1);
    pnorm_sum = zeros(numSample,1);
    pnorm_pos = zeros(numSample,1);
    for i = 1:numSample
        dq = out.dqe(:,i);
        u = out.uq(:,i);
        
        
        power = dq.*u;
        pnorm_pos(i) = sum(power(power>0));
        pnorm_sum(i) = sum(power);
        pnorm_abs(i) = sum(abs(dq.*u));
    end
    
    PnormSumInt = trapz(out.t,pnorm_sum);
    PnormPosInt = trapz(out.t,pnorm_pos);
    PnormAbsInt = trapz(out.t,pnorm_abs);
    
    
    scot_pos(1,k) = PnormPosInt/(mass*grav*dist);
    scot_sum(1,k) = PnormSumInt/(mass*grav*dist);
    scot_abs(1,k) = PnormAbsInt/(mass*grav*dist);
    
    calcs = cell(nDomains/2,1);
    for j = (1+nDomains/2):nDomains
        % right leg stance
        t0 = calcs_steps{j,k}.t(1);
        tf = calcs_steps{j,k}.t(end);
        
        ref.h.calcs = {};
        ref.h.calc = {};
        
        
        tspan = t0:dt:tf;
        if tspan(end) == tf
            tspan(end+1) = tf;
        end
        
        
        current_domain = record_domains{j,k};
        x0 = calcs_steps{j,k}.x(:,1);
        sol = ode113(@(t, x, ref) calcDynamics(current_domain, t, x, model, controller, ref), ...
            tspan, x0, odeopts, ref);
        
        
        calcs{j - nDomains/2} = horzcat_fields(cell2mat(ref.h.calcs));
        
    end
    
    
    out = horzcat_fields_domains([calcs{:}],true);
    
    pcom_0 = pe_com_vec(out.x(:,1));
    pcom_f = pe_com_vec(out.x(:,end));
    
    dist = pcom_f(1) - pcom_0(1);
    
    numSample = numel(out.t);
    pnorm_abs = zeros(numSample,1);
    pnorm_sum = zeros(numSample,1);
    pnorm_pos = zeros(numSample,1);
    for i = 1:numSample
        dq = out.dqe(:,i);
        u = out.uq(:,i);
        
        
        power = dq.*u;
        pnorm_pos(i) = sum(power(power>0));
        pnorm_sum(i) = sum(power);
        pnorm_abs(i) = sum(abs(dq.*u));
    end
    
    PnormSumInt = trapz(out.t,pnorm_sum);
    PnormPosInt = trapz(out.t,pnorm_pos);
    PnormAbsInt = trapz(out.t,pnorm_abs);
    
    
    scot_pos(2,k) = PnormPosInt/(mass*grav*dist);
    scot_sum(2,k) = PnormSumInt/(mass*grav*dist);
    scot_abs(2,k) = PnormAbsInt/(mass*grav*dist);
end

end

