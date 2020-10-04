classdef GaitParams < handle
    
    properties
        controller = 'QPwalk';
        ep = 1e-2;
        w_slack = 1e3;
        stance_leg = 'Right';
        gait_param = [];
        timer = Timer();
        phase = [];
        interp_lib = [];
        avg_vel = [0;0];
        prev_avg_vel = [0;0];
        k_step = 0;
        vd = [0;0];
        v_step_avg_allocator = [0;0];
        v_step_avg_count = 0;
        velocityLP;
        v_prev = [0;0];
        
        sagittal_offset = 0;
        Kp_x = 0;
        Kp_y = 0;
        Kd_x = 0;
        Kd_y = 0;
        
    end
    
    methods
        
    end
end

