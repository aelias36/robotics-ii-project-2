classdef ctrl_bug
    properties
        v_max
        T
        angle_eps
        min_obstacle_dist
    end
    
    methods
        function obj = ctrl_bug()
        end
        
        function v_xy = step(obj, xy, goal, ranges, angles)
            p_goal = goal - xy;
            goal_direction = normalize(p_goal);
            ranges(isnan(ranges)) = inf;

            % Check if path ahead is clear
             goal_angle = atan2(p_goal(2), p_goal(1));
             goal_idx = abs(wrapToPi(angles - goal_angle)) <= obj.angle_eps;
%             has_obstacle = min(ranges(goal_idx)) < obj.min_obstacle_dist;
            has_obstacle = min(ranges) < obj.min_obstacle_dist | ...
                min(ranges(goal_idx)) < norm(p_goal);

            if has_obstacle
                % Move CCW around obstacles
                

                [~,min_dist_idx] = min(ranges);
                min_dist_angle = angles(min_dist_idx);
                
                tangent_angle = min_dist_angle - pi/2;

                v_xy = obj.v_max * [cos(tangent_angle) sin(tangent_angle)];
%                 step_ahead_dist = obj.min_obstacle_dist + obj.v_max * obj.T;
%                 step_ahead_idx = ranges > step_ahead_dist;
%                 step_ahead_angles = angles(step_ahead_idx);
% 
%                 [~, idx] = max(wrapTo2Pi(step_ahead_angles - goal_angle + obj.angle_eps)); %%%%%%%%%%%%%%
%                 step_angle = step_ahead_angles(idx);
%                 
%                 v_xy = obj.v_max * [cos(step_angle) sin(step_angle)];
                return

                  
            end
            

            if norm(p_goal) > obj.v_max * obj.T
                v_xy = obj.v_max * goal_direction;
            else
                v_xy = p_goal / obj.T;
            end

            assert(norm(v_xy) <= obj.v_max + eps)
        end
    end
end

function n = normalize(v)
    n = v / norm(v);
end