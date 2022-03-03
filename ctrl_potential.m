classdef ctrl_potential
    properties
        v_max
        k_goal
        avoidance_strength
        avoidance_distance
        robot_radius
    end
    
    methods
        function obj = ctrl_potential()
        end

        function v = v_avoid_fun(obj, ranges)
            ranges = ranges - obj.robot_radius;
            ranges(ranges<0) = obj.avoidance_distance/100;
            
            v = -obj.avoidance_strength./(ranges.^2) .* (1./ranges - 1/obj.avoidance_distance);
            v(ranges > obj.avoidance_distance) = 0;
            v(isnan(v)) = 0;
        end
        
        function v_xy = step(obj, xy, goal, ranges, angles)
            % Find velocity contribution from goal seeking
            p_goal = goal - xy;
            v_goal = obj.k_goal*p_goal;

            % Find velocity contribution from obstacle avoidance
            avoidance_vecs = -[cos(angles) sin(angles)];
            avoidance_vels = obj.v_avoid_fun(ranges);
            v_avoid = -avoidance_vecs' * avoidance_vels;

            % Find total velocity
            v_xy = v_goal + v_avoid';

            % Clip to within velocity bounds
            if norm(v_xy) > obj.v_max
                v_xy = obj.v_max * normalize(v_xy);
            end
            


            %assert(norm(v_xy) <= obj.v_max + 10*eps)
        end
    end
end

function n = normalize(v)
    n = v / norm(v);
end