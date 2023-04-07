function new_waypoints = updateGoalPoint(waypoints,new_goal)
% updateGoalPoint 
%   update the goal point of the trajectory with a new goal that can change
%   with time.

new_waypoints = waypoints;
new_waypoints(:,end) = new_goal;

end