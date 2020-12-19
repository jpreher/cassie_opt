%% Class: behavior
% 
% Description:  
%             - 3D CassieFull model (compliant)  
%             - two domain stepping
%   
% NOTE: The "is_symmetric" flag controls whether the walking gait is cyclic
%   with strictly the right leg as stance through applying a corresponding 
%   reset map. In general, use symmetric behaviors for optimization and the
%   left/right leg case for simulation and evaluation of the gait.
% 
% Author: Jenna Reher, jreher@caltech.edu
%         Wenlong Ma, wma@caltech.edu
% _________________________________________________________________________

classdef behavior < handle
    properties
        name;           % Name of this behavior
        isSymmetric;    % Flag true = symmetric behavior
        robotModel;     % The robot model associated
        vertices;       % Continuous domains
        edges;          % Discrete domains
        hybridSystem;   % Hybrid system for this behavior1
        constraints;    % Optimization constraints
    end
    
    methods
        function obj = init(obj, is_symmetric, delay_coriolis, omit_coriolis)
            % Assign name
            obj.name = 'walking';
            obj.isSymmetric = is_symmetric;
            
            % Load the namespace domains
            import([obj.name, '.Vertices.*']);
                
            % Load in the model
            robotName = 'cassie_v4';
            urdf = strcat('modules/cassie_description/urdf/', robotName, '.urdf');
            obj.robotModel = Cassie_v4(urdf);
            obj.robotModel.configureDynamics('DelayCoriolisSet', delay_coriolis, 'OmitCoriolisSet', omit_coriolis);
            
            % Load the controller
            controller  = IOFeedback('IO');
            
            %% Either load the full model(for sim&opt) or symmetric(for opt)
            if is_symmetric
                % --------------------------------------------------
                %  vertex1: RightDS                                |
                %  ->edge1: LeftLift                               |
                %  vertex2: RightSS                                |
                %  ->edge2: LeftImpactRelabel                      |
                % --------------------------------------------------
                obj.vertices.r_DS  = RightDS(obj.robotModel);
                obj.vertices.r_SS  = RightSS(obj.robotModel);
                
                obj.edges.l_lift   = Edges.Lift(obj.vertices.r_SS);
                obj.edges.l_impact = Edges.Impact(obj.vertices.r_DS, 'Left', true);
                
                obj.hybridSystem = HybridSystem(obj.name);
                obj.hybridSystem = addVertex(obj.hybridSystem , 'RightDS', ...
                                             'Domain', obj.vertices.r_DS, ...
                                             'Control', controller);
                obj.hybridSystem = addVertex(obj.hybridSystem , 'RightSS', ...
                                             'Domain', obj.vertices.r_SS, ...
                                             'Control', controller);
                srcs = {'RightDS', 'RightSS'};
                tars = {'RightSS', 'RightDS'};
                obj.hybridSystem = addEdge(obj.hybridSystem, srcs, tars);
                obj.hybridSystem = setEdgeProperties(obj.hybridSystem, srcs, tars, ...
                                                     'Guard', {obj.edges.l_lift, obj.edges.l_impact});

            else
                %% -------------------------------------------------
                %  vertex1: RightDS                                |
                %  ->edge1: LeftLift                               |
                %  vertex2: RightSS                                |
                %  ->edge2: LeftImpact                             |
                % --------------------------------------------------
                %  vertex1: LeftDS                                 |
                %  ->edge1: RightLift                              |
                %  vertex2: LeftSS                                 |
                %  ->edge2: RightImpact                            |
                % --------------------------------------------------
                obj.vertices.r_DS = RightDS(obj.robotModel);
                obj.vertices.r_SS = RightSS(obj.robotModel);
                obj.vertices.l_DS = LeftDS(obj.robotModel);
                obj.vertices.l_SS = LeftSS(obj.robotModel);
                
                obj.edges.l_lift   = Edges.Lift(obj.vertices.r_SS);
                obj.edges.l_impact = Edges.Impact(obj.vertices.l_DS, 'Left'); 
                obj.edges.r_lift   = Edges.Lift(obj.vertices.l_SS);
                obj.edges.r_impact = Edges.Impact(obj.vertices.r_DS, 'Right');
                
                obj.hybridSystem = HybridSystem(obj.name);
                obj.hybridSystem = addVertex(obj.hybridSystem , 'RightDS', ...
                                             'Domain', obj.vertices.r_DS, ...
                                             'Control', controller);
                obj.hybridSystem = addVertex(obj.hybridSystem, 'RightSS', ...
                                         'Domain', obj.vertices.r_SS, ...
                                         'Control', controller);
                obj.hybridSystem = addVertex(obj.hybridSystem , 'LeftDS', ...
                                             'Domain', obj.vertices.l_DS, ...
                                             'Control', controller);
                obj.hybridSystem = addVertex(obj.hybridSystem, 'LeftSS', ...
                                         'Domain', obj.vertices.l_SS, ...
                                         'Control', controller);
                                    
                srcs = {'RightDS', 'RightSS', 'LeftDS', 'LeftSS'};
                tars = {'RightSS', 'LeftDS', 'LeftSS', 'RightDS'};
                obj.hybridSystem = addEdge(obj.hybridSystem, srcs, tars);
                obj.hybridSystem = setEdgeProperties(obj.hybridSystem, srcs, tars, ...
                                            'Guard', {obj.edges.l_lift, ...
                                                      obj.edges.l_impact, ...
                                                      obj.edges.r_lift, ...
                                                      obj.edges.r_impact});
            
            end
        end
    end
end