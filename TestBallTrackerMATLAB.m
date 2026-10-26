function TestBallTrackerMATLAB
% CAMERA-ONLY DEMO. Never connects to RUMBA or energizes motors.
url='http://10.98.211.17:4747/video';
cfg.hMin=5;cfg.hMax=25;cfg.sMin=100;cfg.sMax=255;
cfg.vMin=80;cfg.vMax=255;
cfg.minArea=30;cfg.minRadius=4;cfg.minCircularity=0.45;
cfg.ballDiameterCm=4.0;
cfg.calibrationFile=fullfile(fileparts(mfilename('fullpath')),'calibration_config.json');
cam=StewartBallTracker('open',url);
cal=StewartBallTracker('calibrate',cam,cfg);
f=figure('Name','Camera-only ball tracking');
frame=snapshot(cam);
h=imshow(frame);ax=get(h,'Parent');
while isgraphics(f)
    frame=snapshot(cam);
    ball=StewartBallTracker('detect',frame,cfg);
    set(h,'CData',frame);
    if isempty(ball)
        title(ax,'NO BALL DETECTED');
    else
        err=([ball(1)-cal.center_x,ball(2)-cal.center_y])*cal.scale_px_to_cm;
        title(ax,sprintf('Ball (x,y) = (%.2f, %.2f) cm',err(1),err(2)));
    end
    drawnow limitrate;
end
end
