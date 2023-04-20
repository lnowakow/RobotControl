%% Visualize Result

vizStep = 500;
R1_qTask = out.R1configReal.Data';
R1_posTask = out.R1poseReal.Data';
R1_trajReal = out.R1poseDesired.Data';
R1_waypoints = P1_R1_waypoints;

robot1.DataFormat = 'column';

R2_qTask = out.R2configReal.Data';
R2_posTask = transformPoints(out.R2poseReal.Data');
R2_trajReal = transformPoints(out.R2poseDesired.Data');
R2_waypoints = transformPoints(P2_R2_waypoints);

robot2.DataFormat = 'column';


plotSimulationResults(robot1, vizStep, R1_qTask, ...
    R1_posTask, R1_trajReal, R1_waypoints, ...
    robot2, R2_qTask, R2_posTask, R2_trajReal, R2_waypoints, out.tout(end));

function outData = transformPoints(inData)
    R1_T_R2 = [-1,  0,  0, 0.7395*2+0.2;
                0, -1,  0,            0;
                0,  0,  1,            0;
                0,  0,  0,            1];

    [R, C] = size(inData);
    outData = zeros(size(inData));
    for idx = 1:C
        point = [inData(:,idx);1];
        Tpoint = R1_T_R2*point;
        outData(:,idx) = Tpoint(1:3);
    end
end


function [] = plotSimulationResults(robot1, vizStep, R1_qTask, ...
    R1_posTask, R1_trajReal, R1_waypoints, ...
    robot2, R2_qTask, ...
    R2_posTask, R2_trajReal, R2_waypoints, time)
    figure;
    
    max_frames = size(R1_qTask(:,1:vizStep:end),2);

    num_frames = size(R1_qTask,2);
    v = VideoWriter('simVideo.avi');
    v.FrameRate = max_frames / time;
    open(v)
    % Iterate through all joint configurations 
    for i = 1:vizStep:num_frames
        
        disp(strcat('Simulation step:  ',num2str(i),'/',num2str(num_frames)))
        % Change robot configuration for projection on camera
        show(robot1, R1_qTask(:,i),'Frames','on','PreservePlot',false,'visuals','on');
        hold on
        plot3(R1_posTask(1,:),R1_posTask(2,:),R1_posTask(3,:),'b-');
        plot3(R1_trajReal(1,1:vizStep:i),R1_trajReal(2,1:vizStep:i),R1_trajReal(3,1:vizStep:i),'r-');
        plot3(R1_waypoints(1,:),R1_waypoints(2,:),R1_waypoints(3,:),'ko','LineWidth',2);
        % Robot 2
        show(robot2, R2_qTask(:,i),'Frames','on','PreservePlot',false,'visuals','on', ...
            'Position',[0.7395*2+0.2 0 0 pi]);
        plot3(R2_posTask(1,:),R2_posTask(2,:),R2_posTask(3,:),'b-');
        plot3(R2_trajReal(1,1:vizStep:i),R2_trajReal(2,1:vizStep:i),R2_trajReal(3,1:vizStep:i),'r-');
        plot3(R2_waypoints(1,:),R2_waypoints(2,:),R2_waypoints(3,:),'ko','LineWidth',2);
        
        % Draw robot trajectories from simulation
        % Draw environment and its trajectories
        %camproj('orthographic')
        axis([-0.2 2.0 -0.2 0.2 0 1.5])
        %pbaspect([1 1 0.5])
        %view(0,0)
        drawnow
        frame = getframe(gcf);
        writeVideo(v,frame);

    end
    hold off
    
    
    close(v)
end