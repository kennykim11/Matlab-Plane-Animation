% PlaneAnimation.m
% Kenny Kim, Brian Le, Tyler Rodgers
% Spring 2022


%% Class
classdef PlaneAnimation
    % This class can take the outputs from Flight Dynamics Project 4 and
    % animate the aircraft motion using that data in a 3D plot
    % Example usage:
    %  PlaneAnimation.run(Vt, alpha, beta, phi, theta, psi, time, "Extreme Input", false);

    methods(Static)
        function run(Vt, alpha, beta, phi, theta, psi, time, plotTitle, exportVideo)
            % Vt: Vector of true velocity data
            % alpha: Vector of angle of attack data in degrees
            % beta: Vector of sideslip data in degrees
            % phi: Vector of bank angle data in degrees
            % theta: Vector of pitch angle data in degrees
            % psi: Vector of heading angle data in degrees
            % time: Vector of time steps in seconds
            % plotTitle: String of what to make the title of the plot
            % exportVideo: Boolean of whether frames of animation should be
            %  captured and saved in a .avi file

           animatePlane(Vt, alpha, beta, phi, theta, psi, time, plotTitle, exportVideo)
       end
    end
end



%% Engine
function animatePlane(Vt, alpha, beta, phi, theta, psi, time, plotTitle, exportVideo)
    alpha = -deg2rad(alpha);
    beta = deg2rad(beta);
    phi = deg2rad(phi);
    theta = deg2rad(theta);
    psi = deg2rad(psi);
    
    % Precalculate Positions
    positions = zeros(length(time), 3);
    for i=2:length(time)
        Vt_i = Vt(i)*(time(i)-time(i-1));
        u = Vt_i*cos(alpha(i))*cos(beta(i));
        v = Vt_i*sin(beta(i));
        w = Vt_i*sin(alpha(i))*cos(beta(i));
        difPos = [u v w] * rotationMatrix(psi(i), theta(i), phi(i));
        positions(i, :) = positions(i-1, :) + difPos;
    end
    x = positions(:,1);
    y = positions(:,2);
    z = positions(:,3);

    %% Plane Model
    planeGeo = struct();
    planeGeo.wing = [
        0 -3  0;
        1  0  0;
        0  3  0];
    planeGeo.fslg = [
        3  0  0;
       -3 -1  0;
       -3  1  0];
    planeGeo.tail = [
       -3  0  1.5;
       -3  0  0;
       -1.5  0  0];
    planeGeo.scale = 100;
    planeGeo.color = 'g';
    
    %% Plotting
    % https://www.youtube.com/watch?v=dLyBejMmHqE&t=8s
    figure('Name','Aircraft Animation','NumberTitle','off');
    curve = animatedline('LineWidth', 1, 'Color', 'b');
    lims = getCoordLimits(x,y,z);
    planeGeo.scale = lims(4,1) / 20;
    set(gca, 'XLim', lims(1,:), 'YLim', lims(2,:), 'ZLim', lims(3,:))
    view(-37.5, 30); % Include this to force 3D plots
    title(plotTitle)
    grid on
    hold on
    plottedPlane = text(0, 0, ''); % Creating a placeholder UI object to delete
    xlabel("X [ft]")
    ylabel("Y [ft]")
    zlabel("Z [ft]")

    for i=1:length(time)
        delete(plottedPlane)
        addpoints(curve, x(i), y(i), z(i));

        % Plane Transformation
        plane = planeGeo;
        plane = scalePlane(plane);
        plane = rotatePlane(plane, psi(i), theta(i), phi(i));
        plane = translatePlane(plane, x(i), y(i), z(i));
        plottedPlane = plotPlane(plane);

        % Display
        drawnow
        if exportVideo
            frames(i) = getframe(gcf);
        end
    end

    % Export
    if exportVideo
        video = VideoWriter(append(plotTitle,'.avi'));
        open(video)
        writeVideo(video,frames)
        close(video)
    end
end

%% Helpers
function lims = getCoordLimits(x, y, z)
    % Get limits for each axis for plotting, all on 1:1:1 scale
    % Also return size
    ranges = [getRanges(x); getRanges(y); getRanges(z)];
    sz = max(ranges(:,2))/1.8;
    lims = [ranges(1,1)-sz, ranges(1,1)+sz;
        ranges(2,1)-sz, ranges(2,1)+sz;
        ranges(3,1)-sz, ranges(3,1)+sz;
        sz, 0];
end

function ranges = getRanges(vec)
    % Get middle and range of vector
    maxAns = max(vec);
    minAns = min(vec);
    midAns = (maxAns + minAns)/2;
    rangeAns = maxAns - minAns;
    ranges = [midAns, rangeAns];
end

function p = scalePlane(plane)
    plane.wing = plane.wing * plane.scale;
    plane.fslg = plane.fslg * plane.scale;
    plane.tail = plane.tail * plane.scale;
    p = plane;
end

function p = rotatePlane(plane, psi, theta, phi)
    rotMatr = rotationMatrix(psi, theta, phi);
    plane.wing = plane.wing * rotMatr;
    plane.fslg = plane.fslg * rotMatr;
    plane.tail = plane.tail * rotMatr;
    p = plane;
end

function p = translatePlane(plane, x, y, z)
    transMatr = [x y z; x y z; x y z];
    plane.wing = plane.wing + transMatr;
    plane.fslg = plane.fslg + transMatr;
    plane.tail = plane.tail + transMatr;
    p = plane;
end

function p = plotPlane(plane)
    % https://stackoverflow.com/a/19842820
    comb = [plane.wing(:),plane.fslg(:), plane.tail(:)].';
    mat = reshape(comb, 3, 3, 3);
    x = mat(:,:,1)';
    y = mat(:,:,2)';
    z = mat(:,:,3)';
    p = fill3(x, y, z, plane.color);
end

function rotMat = rotationMatrix(psi, theta, phi)
    % https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)#Transformations_(Euler_angles)
    % https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
    psi = -psi;
    yawMat = [
        cos(psi), -sin(psi), 0;
        sin(psi), cos(psi),  0;
        0,        0,         1];
    pitchMat = [
        cos(theta),  0, sin(theta);
        0,           1, 0;
        -sin(theta), 0, cos(theta)];
    rollMat = [
        1, 0,        0;
        0, cos(phi), -sin(phi);
        0, sin(phi), cos(phi)];
    rotMat = yawMat*pitchMat*rollMat;
end