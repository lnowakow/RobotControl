%% Generate Report of Simulation
close all
time = out.tout;

R1Hm = out.R1HmConst.Data(1);
R1Dm = out.R1DmConst.Data(1);
R1Km = out.R1KmConst.Data(1);
R1_handle = sprintf("R1_Hm%d-Dm%d-Km%d_", R1Hm,R1Dm,R1Km);

R2Hm = out.R1HmConst.Data(1);
R2Dm = out.R1DmConst.Data(1);
R2Km = out.R1KmConst.Data(1);
R2_handle = sprintf("R2_Hm%d-Dm%d-Km%d_", R2Hm,R2Dm,R2Km);

image_dir = "images/";
sim_folder = R1_handle+R2_handle+"Ks500/";
if ~exist(image_dir+sim_folder, "dir")
    mkdir(image_dir+sim_folder)
end

desiredPoseLabels = ["X desired", "Y desired", "Z desired"];
realPoseLabels = ["X real", "Y real", "Z real"];
configLabels = ["q1", "q2", "q3"];
forceLabels = ["Fx", "Fy", "Fz"];
phaseLabels = ["Robot 1 Move", "Robot 2 Move", "Grasp Start"];

%% R1 Desired Trajectory v Real Trajectory
fig_title = "R1 Desired v Real Trajectory";
fig_subtitle = sprintf("Hm=%d, Dm=%d, Km=%d, Ks=500Nm", R1Hm, R1Dm, R1Km);
f1 = figure("Name",fig_title);
plot(time, out.R1poseReal.Data, "LineWidth",2)
hold on
plot(time, out.R1poseDesired.Data, ':', "LineWidth",2);
xline([0, out.phase2time.Data(end), out.phase3time.Data(end)], ...
    'k--', phaseLabels, ...
    "LabelVerticalAlignment","bottom", ...
    "LabelOrientation","aligned", ...
    "LabelHorizontalAlignment","right")
ax = gca;
ax.XAxis.Exponent = 0;
legend([realPoseLabels, desiredPoseLabels])
title(fig_title)
subtitle(fig_subtitle)
xlabel("Time [s]")
ylabel("Cartesian Positions [m]")
xtickformat('%.0f')
hold off
saveas(f1, image_dir+sim_folder+R1_handle+fig_title+'.jpg')

%% R2 Desired Trajectory v Real Trajectory
fig_title = "R2 Desired v Real Trajectory";
fig_subtitle = sprintf("Hm=%d, Dm=%d, Km=%d, Ks=500Nm", R2Hm, R2Dm, R2Km);
f1 = figure("Name",fig_title);
plot(time, out.R2poseReal.Data, "LineWidth",2)
hold on
plot(time, out.R2poseDesired.Data, ':', "LineWidth",2);
xline([0, out.phase2time.Data(end), out.phase3time.Data(end)], ...
    'k--', phaseLabels, ...
    "LabelVerticalAlignment","bottom", ...
    "LabelOrientation","aligned", ...
    "LabelHorizontalAlignment","right")
ax = gca;
ax.XAxis.Exponent = 0;
legend([realPoseLabels, desiredPoseLabels])
title(fig_title)
subtitle(fig_subtitle)
xlabel("Time [s]")
ylabel("Cartesian Positions [m]")
xtickformat('%.0f')
hold off
saveas(f1, image_dir+sim_folder+R2_handle+fig_title+'.jpg')

%% Spring Length
fig_title = "Spring Length during Interaction";
fig_subtitle = sprintf("Hm=%d, Dm=%d, Km=%d, Ks=500Nm", R2Hm, R2Dm, R2Km);

interaction_time = out.tout(out.tout >= out.phase3time.Data(end));
spring_l = out.spring_length.Data(end-size(interaction_time)+1:end);

[settling_time, max_disp] = findSettlingTime(spring_l, interaction_time);
TsLabel = sprintf("Ts=%.4f s", settling_time);

f1 = figure("Name",fig_title);
plot(interaction_time, spring_l, "LineWidth",2)
hold on
xline(interaction_time(1)+settling_time,'k--',TsLabel, ...
    "LabelVerticalAlignment","bottom", ...
    "LabelOrientation","horizontal", ...
    "LabelHorizontalAlignment","right")
ax = gca;
ax.XAxis.Exponent = 0;
title(fig_title)
subtitle(fig_subtitle)
xlabel("Time [s]")
ylabel("Spring Length [m]")
xtickformat('%.0f')
hold off
saveas(f1, image_dir+sim_folder+fig_title+'.jpg')

%% R1 Interaction Forces
fig_title = "Spring Force Exerted on Each Manipulator";
fig_subtitle = sprintf("Hm=%d, Dm=%d, Km=%d, Ks=500Nm", R2Hm, R2Dm, R2Km);

interaction_time = out.tout(out.tout >= out.phase3time.Data(end));
force = squeeze( ...
    out.R1interactionForce.Data(:,1,end-size(interaction_time)+1:end));

[settling_time, max_disp] = findSettlingTime(spring_l, interaction_time);
TsLabel = sprintf("Ts=%.4f s", settling_time);

f1 = figure("Name",fig_title);
plot(interaction_time, force, "LineWidth",2)
hold on
xline(interaction_time(1)+settling_time,'k--',TsLabel, ...
    "LabelVerticalAlignment","bottom", ...
    "LabelOrientation","horizontal", ...
    "LabelHorizontalAlignment","right")
ax = gca;
ax.XAxis.Exponent = 0;
title(fig_title)
subtitle(fig_subtitle)
legend(forceLabels)
xlabel("Time [s]")
ylabel("Interaction Force [N]")
xtickformat('%.0f')
hold off
saveas(f1, image_dir+sim_folder+fig_title+'.jpg')



