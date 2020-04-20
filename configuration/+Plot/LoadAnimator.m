function [conGUI] = LoadAnimator(behavior, logger, varargin)
    
    
    if isa(logger, 'SimLogger')
        np = length(logger);
        
        t = [];
        q = [];
        for i=1:np
            t = [t, logger(i).flow.t]; %#ok<*AGROW>
            q = [q, logger(i).flow.states.x];
        end
    else
        np = length(logger);
        
        t = [];
        q = [];
        for i=1:np
            t = [t, logger(i).tspan]; %#ok<*AGROW>
            q = [q, logger(i).states.x];
        end
    end

    robot_disp = Plot.LoadDisplay(behavior, varargin{:});
    
    anim = frost.Animator.AbstractAnimator(robot_disp, t, q);
    anim.isLooping = false;
    anim.speed = 0.25;
    anim.pov = frost.Animator.AnimatorPointOfView.Free;
    anim.Animate(true);
    conGUI = frost.Animator.AnimatorControls();
    conGUI.anim = anim;
end