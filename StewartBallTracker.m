function varargout = StewartBallTracker(action, varargin)
% Native MATLAB port of tracker_test.py / BallBalancerPID.py calibration.
% Requires Image Processing Toolbox and MATLAB ipcam IP-camera support.
% Coordinates: image x right, image y down. Sign conventions MUST be verified.
switch lower(action)
    case 'open'
        url=varargin{1};
        try
            cam=ipcam(url);
        catch ME
            error('BallTracker:Camera','Cannot open %s using ipcam. Install MATLAB IP Camera support and verify MJPEG URL. %s',url,ME.message);
        end
        varargout{1}=cam;
    case 'detect'
        rgb=varargin{1}; cfg=varargin{2};
        if size(rgb,3)==1, rgb=repmat(rgb,[1 1 3]); end
        hsvImage=rgb2hsv(rgb);
        H=hsvImage(:,:,1)*179; S=hsvImage(:,:,2)*255; V=hsvImage(:,:,3)*255;
        mask=(H>=cfg.hMin & H<=cfg.hMax & S>=cfg.sMin & S<=cfg.sMax & V>=cfg.vMin & V<=cfg.vMax);
        mask=imopen(mask,strel('disk',1));
        mask=imclose(mask,strel('disk',2));
        stats=regionprops(mask,'Area','Centroid','EquivDiameter','Perimeter');
        detection=[];
        if ~isempty(stats)
            areas=[stats.Area]; [~,idx]=sort(areas,'descend');
            for ii=idx
                area=stats(ii).Area;
                radius=stats(ii).EquivDiameter/2;
                circularity=4*pi*area/max(stats(ii).Perimeter^2,1);
                if area>=cfg.minArea && radius>=cfg.minRadius && circularity>=cfg.minCircularity
                    detection=[stats(ii).Centroid radius area];
                    break;
                end
            end
        end
        varargout{1}=detection;
        if nargout>1,varargout{2}=mask;end
    case 'calibrate'
        camera=varargin{1}; cfg=varargin{2};
        frame=snapshot(camera);
        [ball,~]=StewartBallTracker('detect',frame,cfg);
        if isempty(ball)
            error('BallTracker:Calibration','Place orange ball at plate center and adjust HSV thresholds first. Ball not detected.');
        end
        f=figure('Name','Ball calibration - click red marker','NumberTitle','off');
        imshow(frame); hold on;
        plot(ball(1),ball(2),'g+','MarkerSize',16,'LineWidth',2);
        title('BALL MUST BE AT CENTER. Click the RED REFERENCE MARKER.');
        [rx,ry]=ginput(1);
        if ~ishghandle(f),error('BallTracker:Calibration','Calibration cancelled');end
        close(f);
        plateRadiusPx=hypot(rx-ball(1),ry-ball(2));
        if ~(isfinite(plateRadiusPx)&&plateRadiusPx>10),error('BallTracker:Calibration','Invalid red marker / radius');end
        calibration.center_x=ball(1);calibration.center_y=ball(2);
        calibration.scale_px_to_cm=cfg.ballDiameterCm/(2*ball(3));
        % Original Python measures heading clockwise from image up.
        heading=mod(atan2d(rx-ball(1),ball(2)-ry),360);
        calibration.zero_azimuth_deg=mod(-heading,360);
        calibration.plate_radius_px=plateRadiusPx;
        calibration.ball_diameter_cm=cfg.ballDiameterCm;
        fid=fopen(cfg.calibrationFile,'w');
        if fid<0,error('BallTracker:Calibration','Cannot write calibration file');end
        cleaner=onCleanup(@()fclose(fid));
        fwrite(fid,jsonencode(calibration),'char');
        varargout{1}=calibration;
    case 'loadcalibration'
        filename=varargin{1};
        if ~isfile(filename),error('BallTracker:Calibration','Missing %s. Run calibration first.',filename);end
        varargout{1}=jsondecode(fileread(filename));
    case 'pidinit'
        s.integral=[0 0];s.prevError=[0 0];s.filteredDerivative=[0 0];
        s.ready=false;
        varargout{1}=s;
    case 'pid'
        s=varargin{1}; ball=varargin{2}; cal=varargin{3}; cfg=varargin{4};dt=varargin{5};
        err=[(ball(1)-cal.center_x),(ball(2)-cal.center_y)]*cal.scale_px_to_cm;
        if ~s.ready
            s.prevError=err;s.ready=true;der=[0 0];
        else
            der=(err-s.prevError)/max(dt,0.001);
        end
        % Filtering reduces amplified camera jitter in the D term.
        s.filteredDerivative=cfg.derivativeAlpha*der+(1-cfg.derivativeAlpha)*s.filteredDerivative;
        s.integral=max(-cfg.integralLimit,min(cfg.integralLimit,s.integral+err*dt));
        cx=cfg.Kp*err(1)+cfg.Ki*s.integral(1)+cfg.Kd*s.filteredDerivative(1);
        cy=cfg.Kp*err(2)+cfg.Ki*s.integral(2)+cfg.Kd*s.filteredDerivative(2);
        pitch=-cy;roll=cx; % Original Python sign mapping.
        magnitude=hypot(pitch,roll);
        if magnitude>cfg.maxTiltDeg
            pitch=pitch*cfg.maxTiltDeg/magnitude;
            roll=roll*cfg.maxTiltDeg/magnitude;
            magnitude=cfg.maxTiltDeg;
        end
        az=mod(atan2d(roll,pitch)+cal.zero_azimuth_deg,360);
        s.prevError=err;
        varargout={s,magnitude,az,err,[pitch roll]};
    otherwise
        error('BallTracker:Action','Unknown action %s',action);
end
end
