function params = NewStepPreProcess(sys, t, x, controller, params) %#ok<INUSL>
    
    y = struct2array(sys.VirtualConstraints);
    
%     for i=1:numel(y)
%         if strcmp(y(i).PhaseType,'TimeBased')
%             params.(y(i).PhaseParamName) = t + params.(y(i).PhaseParamName);
%         end
%     end
%     controller.Param.lastParam = params;
    
end