close all
%% R1 Interaction Forces
fig_title = "Effect of Rod Stiffness on Interaction Stability";
interaction_time = out_ks10000.tout(out_ks1000.tout >= out_ks1000.phase3time.Data(end));
force = squeeze( ...
    out_ks10000.R1interactionForce.Data(1,1,end-size(interaction_time)+1:end));

[force_settling_time, max_force] = findSettlingTime(spring_l, interaction_time);
TsLabel10000 = sprintf("Ts=%.4f s", force_settling_time);

f1 = figure("Name",fig_title);
plot(interaction_time, force, "LineWidth",2,"Color","r")
hold on
interaction_time = out_ks1000.tout(out_ks1000.tout >= out_ks1000.phase3time.Data(end));
force = squeeze( ...
    out_ks1000.R1interactionForce.Data(1,1,end-size(interaction_time)+1:end));

[force_settling_time, max_force] = findSettlingTime(spring_l, interaction_time);
TsLabel1000 = sprintf("Ts=%.4f s", force_settling_time);
plot(interaction_time, force, "LineWidth",2,"Color","g")

interaction_time = out.tout(out.tout >= out.phase3time.Data(end));
force = squeeze( ...
    out.R1interactionForce.Data(1,1,end-size(interaction_time)+1:end));

[force_settling_time, max_force] = findSettlingTime(spring_l, interaction_time);
TsLabel500 = sprintf("Ts=%.4f s", force_settling_time);
plot(interaction_time, force, "LineWidth",2,"Color","b")

xline(interaction_time(1)+force_settling_time,'r--',TsLabel10000, ...
    "LabelVerticalAlignment","top", ...
    "LabelOrientation","horizontal", ...
    "LabelHorizontalAlignment","right")
xline(interaction_time(1)+force_settling_time,'g--',TsLabel1000, ...
    "LabelVerticalAlignment","middle", ...
    "LabelOrientation","horizontal", ...
    "LabelHorizontalAlignment","right")
xline(interaction_time(1)+force_settling_time,'b--',TsLabel500, ...
    "LabelVerticalAlignment","bottom", ...
    "LabelOrientation","horizontal", ...
    "LabelHorizontalAlignment","right")


ax = gca;
ax.XAxis.Exponent = 0;
title(fig_title)
legend(["Ks=500N/m", "Ks=1,000N/m", "Ks=10,000N/m"])
xlabel("Time [s]")
ylabel("Interaction Force [N]")
xtickformat('%.0f')
hold off
