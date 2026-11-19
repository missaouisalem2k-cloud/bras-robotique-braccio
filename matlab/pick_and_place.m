function pick_and_place()
% PICK_AND_PLACE  Simulation pick-and-place du bras (fonction MATLAB)
clc; clear; close all;

%% ===== Paramètres géométriques =====
L1 = 5;   % hauteur base
L2 = 10;  % premier bras
L3 = 8;   % deuxième bras
L4 = 3;   % outil

%% ===== Paramètres de la simulation =====
frames = 80;
traj = zeros(3,0);   % matrice 3xN pour stocker la trajectoire

% Angles initiaux / cibles (radians)
t1_init   = 0;
t1_right  = -pi/6;            % tourner à droite (relative)
t2_init   = pi/6;
t2_grab   = pi/3;             % avancer pour attraper
t3        = -pi/6;
t4        = 0;
t5        = 0;

fig = figure('Name','Pick-and-Place simulation','NumberTitle','off');

%% ==== 1) Départ (déjà en position initiale) ====
[P0,P1,P2,P3,P4,P5] = forwardKinematics(t1_init,t2_init,t3,t4,t5,L1,L2,L3,L4);
traj(:,end+1) = P5;

%% ==== 2) Tourner à droite (de 0 vers t1_right) ====
for t1 = linspace(t1_init, t1_right, frames)
    [P0,P1,P2,P3,P4,P5] = forwardKinematics(t1,t2_init,t3,t4,t5,L1,L2,L3,L4);
    traj(:,end+1) = P5;
    clf(fig); hold on; grid on;
    plotRobot(P0,P1,P2,P3,P4,P5);
    plot3(traj(1,:),traj(2,:),traj(3,:),'r-','LineWidth',1.5);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Étape 2 : tourner à droite');
    axis equal; axis([-25 25 -25 25 0 30]); view(45,30);
    drawnow;
end

%% ==== 3) Avancer le bras pour attraper (modifier theta2) ====
for t2 = linspace(t2_init, t2_grab, frames)
    [P0,P1,P2,P3,P4,P5] = forwardKinematics(t1_right,t2,t3,t4,t5,L1,L2,L3,L4);
    traj(:,end+1) = P5;
    clf(fig); hold on; grid on;
    plotRobot(P0,P1,P2,P3,P4,P5);
    plot3(traj(1,:),traj(2,:),traj(3,:),'r-','LineWidth',1.5);
    title('Étape 3 : attraper l''objet');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; axis([-25 25 -25 25 0 30]); view(45,30);
    drawnow;
end

%% ==== 4) Tourner la base de +90° (relatif à sa position actuelle) ====
t1_target = t1_right + pi/2; % rotation de +90°
for t1 = linspace(t1_right, t1_target, frames)
    [P0,P1,P2,P3,P4,P5] = forwardKinematics(t1, t2_grab, t3, t4, t5, L1,L2,L3,L4);
    traj(:,end+1) = P5;
    clf(fig); hold on; grid on;
    plotRobot(P0,P1,P2,P3,P4,P5);
    plot3(traj(1,:),traj(2,:),traj(3,:),'r-','LineWidth',1.5);
    title('Étape 4 : rotation de 90°');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; axis([-25 25 -25 25 0 30]); view(45,30);
    drawnow;
end

%% ==== 5) Déposer l'objet (rétrograder theta2) ====
for t2 = linspace(t2_grab, t2_init, frames)
    [P0,P1,P2,P3,P4,P5] = forwardKinematics(t1_target, t2, t3, t4, t5, L1,L2,L3,L4);
    traj(:,end+1) = P5;
    clf(fig); hold on; grid on;
    plotRobot(P0,P1,P2,P3,P4,P5);
    plot3(traj(1,:),traj(2,:),traj(3,:),'r-','LineWidth',1.5);
    title('Étape 5 : déposer');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; axis([-25 25 -25 25 0 30]); view(45,30);
    drawnow;
end

%% ==== 6) Retour à la position initiale (base + bras) ====
for t1 = linspace(t1_target, t1_init, frames)
    [P0,P1,P2,P3,P4,P5] = forwardKinematics(t1, t2_init, t3, t4, t5, L1,L2,L3,L4);
    traj(:,end+1) = P5;
    clf(fig); hold on; grid on;
    plotRobot(P0,P1,P2,P3,P4,P5);
    plot3(traj(1,:),traj(2,:),traj(3,:),'r-','LineWidth',1.5);
    title('Étape 6 : retour à la base');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; axis([-25 25 -25 25 0 30]); view(45,30);
    drawnow;
end

%% ==== Tracé final de la trajectoire complète ====
figure('Name','Trajectoire','NumberTitle','off');
plot3(traj(1,:), traj(2,:), traj(3,:), 'r-', 'LineWidth',2);
grid on; xlabel('X'); ylabel('Y'); zlabel('Z'); title('Trajectoire complète de l''effecteur');
axis equal; axis([-25 25 -25 25 0 30]);

end % fin de la fonction pick_and_place

%% ===== Fonctions locales =====
function [P0,P1,P2,P3,P4,P5] = forwardKinematics(t1,t2,t3,t4,t5,L1,L2,L3,L4)
    % Matrices homogènes (selon ton modèle)
    T01 = [cos(t1) -sin(t1) 0 0;
           sin(t1)  cos(t1) 0 0;
           0        0       1 L1;
           0 0 0 1];

    T12 = [cos(t2) -sin(t2) 0 0;
           0 0 -1 0;
           sin(t2) cos(t2) 0 0;
           0 0 0 1];

    T23 = [cos(t3) -sin(t3) 0 L2;
           sin(t3)  cos(t3) 0 0;
           0 0 1 0;
           0 0 0 1];

    T34 = [cos(t4) -sin(t4) 0 L3;
           sin(t4)  cos(t4) 0 0;
           0 0 1 0;
           0 0 0 1];

    T45 = [cos(t5) -sin(t5) 0 0;
           0 0 1 L4;
           -sin(t5) -cos(t5) 0 0;
           0 0 0 1];

    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45; 

    P0 = [0;0;0];
    P1 = T01(1:3,4);
    P2 = T02(1:3,4);
    P3 = T03(1:3,4);
    P4 = T04(1:3,4);
    P5 = T05(1:3,4);
end

function plotRobot(P0,P1,P2,P3,P4,P5)
    % trace chaque segment
    plot3([P0(1) P1(1)], [P0(2) P1(2)], [P0(3) P1(3)], '-ko','LineWidth',2,'MarkerSize',6,'MarkerFaceColor','k');
    plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)], '-bo','LineWidth',2,'MarkerSize',6,'MarkerFaceColor','b');
    plot3([P2(1) P3(1)], [P2(2) P3(2)], [P2(3) P3(3)], '-go','LineWidth',2,'MarkerSize',6,'MarkerFaceColor','g');
    plot3([P3(1) P4(1)], [P3(2) P4(2)], [P3(3) P4(3)], '-mo','LineWidth',2,'MarkerSize',6,'MarkerFaceColor','m');
    plot3([P4(1) P5(1)], [P4(2) P5(2)], [P4(3) P5(3)], '-ro','LineWidth',2,'MarkerSize',6,'MarkerFaceColor','r');
    % point effecteur
    plot3(P5(1),P5(2),P5(3),'rp','MarkerSize',10,'MarkerFaceColor','r');
end
