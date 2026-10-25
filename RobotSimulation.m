function fig = RobotSimulation(varargin)
parser=inputParser; addParameter(parser,'Visible','on'); parse(parser,varargin{:});
[robot,param]=makeRobot(); q=[0 0 0]; scene=[]; lastUpdate=tic;
settingsFig=[];
savedExtension=0;
viewMode='rotate'; dragState=[];
fig=uifigure('Name','Robot Simulation | 3R / 3R + Prismatic','Position',[60 40 1320 850], ...
    'Visible',parser.Results.Visible);
main=uigridlayout(fig,[1 2]); main.ColumnWidth={370,'1x'};
left=uigridlayout(main,[9 1]); left.RowHeight={28,42,32,24,32,215,215,120,45};
left.RowSpacing=5;
left.Scrollable='on';
titleLabel=uilabel(left,'Text','3R Robot','FontSize',20,'FontWeight','bold');
uilabel(left,'Text','J1-J3: degrees. J4: metres along the forearm. Inverse Kinematics holds J4 at the entered extension.', ...
    'WordWrap','on');
uibutton(left,'Text','Setting Parameters','Tag','SettingsButton', ...
    'ButtonPushedFcn',@(~,~) openSettings());
typeLabel=uilabel(left,'Text','Active: 3 revolute joints','Tag','RobotTypeLabel');
typeButton=uibutton(left,'Text','Switch to 3R + Prismatic J4','Tag','RobotTypeButton', ...
    'ButtonPushedFcn',@(~,~) switchRobotType());
fig.DeleteFcn=@(~,~) closeSettings();
panel=uipanel(left,'Title','Joint control | J1-J3 (deg), J4 (m)');
g=uigridlayout(panel,[4 3]); g.ColumnWidth={30,80,'1x'}; g.RowHeight={36,36,36,36};
fields=gobjects(1,3); sliders=gobjects(1,3);
for j=1:3
    uilabel(g,'Text',sprintf('J%d',j));
    fields(j)=uieditfield(g,'numeric','Value',q(j),'Limits',param.jointLimits(j,:), ...
        'Tag',sprintf('Joint%d',j));
    fields(j).ValueChangedFcn=@(~,e) moveJoint(j,e.Value,true);
    sliders(j)=uislider(g,'Value',q(j),'Limits',param.jointLimits(j,:), ...
        'MajorTicks',linspace(param.jointLimits(j,1),param.jointLimits(j,2),3),'MinorTicks',[], ...
        'Tag',sprintf('Slider%d',j));
    sliders(j).ValueChangingFcn=@(~,e) moveJoint(j,e.Value,false);
    sliders(j).ValueChangedFcn=@(~,e) moveJoint(j,e.Value,true);
end
uilabel(g,'Text','J4');
extensionField=uieditfield(g,'numeric','Value',0,'Limits',param.prismaticLimits, ...
    'Tag','Joint4','Enable','off','ValueChangedFcn',@(~,e) moveExtension(e.Value,true));
extensionSlider=uislider(g,'Value',0,'Limits',param.prismaticLimits, ...
    'MajorTicks',linspace(param.prismaticLimits(1),param.prismaticLimits(2),3), ...
    'MinorTicks',[],'Tag','Slider4','Enable','off');
extensionSlider.ValueChangingFcn=@(~,e) moveExtension(e.Value,false);
extensionSlider.ValueChangedFcn=@(~,e) moveExtension(e.Value,true);
panel=uipanel(left,'Title','IK | End-effector target (metres)');
g=uigridlayout(panel,[4 3]); g.ColumnWidth={25,80,'1x'}; g.RowHeight={36,36,36,28};
xyz='XYZ'; target=[.4 0 .25]; targetFields=gobjects(1,3); targetSliders=gobjects(1,3);
reach=param.L2+param.L3; height=param.baseHeight+param.L1;
targetLimits=[-reach reach; -reach reach; height-reach height+reach];
for j=1:3
    uilabel(g,'Text',xyz(j));
    targetFields(j)=uieditfield(g,'numeric','Value',target(j),'Tag',['Target' xyz(j)]);
    targetFields(j).ValueChangedFcn=@(~,e) moveTarget(j,e.Value,true);
    targetSliders(j)=uislider(g,'Value',target(j),'Limits',targetLimits(j,:), ...
        'MajorTicks',linspace(targetLimits(j,1),targetLimits(j,2),3),'MinorTicks',[], ...
        'Tag',['TargetSlider' xyz(j)]);
    targetSliders(j).ValueChangingFcn=@(~,e) moveTarget(j,e.Value,false);
    targetSliders(j).ValueChangedFcn=@(~,e) moveTarget(j,e.Value,true);
end
label=uilabel(g,'Text','Elbow'); label.Layout.Row=4; label.Layout.Column=[1 2];
branch=uidropdown(g,'Items',{'1: positive J3','2: negative J3'},'ItemsData',[1 2], ...
    'Value',1,'Tag','Branch','ValueChangedFcn',@(~,~) solveTarget(true));
branch.Layout.Row=4; branch.Layout.Column=3;
tip=uilabel(left,'Text','','FontWeight','bold','FontSize',16,'WordWrap','on','Tag','ActualPosition');
status=uilabel(left,'Text','Ready.', ...
    'WordWrap','on','FontSize',14,'Tag','Status');
right=uigridlayout(main,[3 1]); right.RowHeight={30,'1x',215};
viewControls=uigridlayout(right,[1 2]); viewControls.Layout.Row=1;
viewControls.ColumnWidth={'1x',130}; viewControls.Padding=[0 0 0 0];
uilabel(viewControls,'Text','Drag to rotate | Scroll to zoom | Pan using the toolbar');
uibutton(viewControls,'Text','Reset View','Tag','ResetRobotView', ...
    'ButtonPushedFcn',@(~,~) resetRobotView());
ax=uiaxes(right,'Tag','RobotAxes'); ax.Layout.Row=2;
hold(ax,'on'); grid(ax,'on'); ax.DataAspectRatio=[1 1 1];
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
ax.XLim=[-.5 .5]; ax.YLim=[-.5 .5]; ax.ZLim=[0 .75];
ax.CameraPosition=[-1.15 1.15 1]; ax.CameraTarget=[0 0 .22];
ax.CameraUpVector=[0 0 1]; ax.CameraViewAngle=42;
ax.XLimMode='manual'; ax.YLimMode='manual'; ax.ZLimMode='manual';
ax.CameraPositionMode='manual'; ax.CameraTargetMode='manual';
ax.CameraUpVectorMode='manual'; ax.CameraViewAngleMode='manual';
% Own the zoom step and its bounds. Default zoomInteraction can increase the
% view angle until the entire fixed-limit axes box becomes tiny.
% Use one camera controller for drag and wheel events. Figure-level wheel
% callbacks can suppress MATLAB's automatic rotateInteraction handling.
disableDefaultInteractivity(ax);
ax.Interactions=[];
% Custom mode buttons avoid the built-in modes replacing window callbacks.
viewToolbar=axtoolbar(ax,{});
rotateButton=axtoolbarbtn(viewToolbar,'state','Icon','rotate', ...
    'Tag','RobotViewRotate','Tooltip','Rotate: drag on the graph');
panButton=axtoolbarbtn(viewToolbar,'state','Icon','pan', ...
    'Tag','RobotViewPan','Tooltip','Pan: drag on the graph');
rotateButton.ValueChangedFcn=@(~,~) selectViewMode('rotate');
panButton.ValueChangedFcn=@(~,~) selectViewMode('pan');
rotateButton.Value='on'; panButton.Value='off';
homeButton=axtoolbarbtn(viewToolbar,'push','Icon','restoreview', ...
    'Tag','RobotViewHome','Tooltip','Reset view');
homeButton.ButtonPushedFcn=@(~,~) resetRobotView();
axtoolbarbtn(viewToolbar,'push','Icon','zoomin','Tooltip','Zoom in', ...
    'Tag','RobotZoomIn','ButtonPushedFcn',@(~,~) zoomRobotView(-1));
axtoolbarbtn(viewToolbar,'push','Icon','zoomout','Tooltip','Zoom out', ...
    'Tag','RobotZoomOut','ButtonPushedFcn',@(~,~) zoomRobotView(1));
viewToolbar.Visible='on';
homeView=captureRobotView();
fig.WindowScrollWheelFcn=@scrollRobotView;
fig.WindowButtonDownFcn=@startViewDrag;
fig.WindowButtonMotionFcn=@moveViewDrag;
fig.WindowButtonUpFcn=@(~,~) stopViewDrag();
panel=uipanel(right,'Title','Transformation Matrix | End effector to World'); panel.Layout.Row=3;
g=uigridlayout(panel,[1 1]);
transformTable=uitable(g,'Data',eye(4), ...
    'ColumnName',{'X axis','Y axis','Z axis','Position (m)'}, ...
    'RowName',{'World X','World Y','World Z','Homogeneous'}, ...
    'ColumnEditable',false,'Tag','TransformationMatrix');
scene=drawRobot(ax,param,robot,q,scene);
setappdata(fig,'RobotModel',robot); setappdata(fig,'RobotParameters',param);
refresh(true);

    function savedView=captureRobotView()
        savedView=struct('XLim',ax.XLim,'YLim',ax.YLim,'ZLim',ax.ZLim, ...
            'CameraPosition',ax.CameraPosition,'CameraTarget',ax.CameraTarget, ...
            'CameraUpVector',ax.CameraUpVector,'CameraViewAngle',ax.CameraViewAngle);
    end
    function selectViewMode(mode)
        viewMode=mode; dragState=[];
        rotateButton.Value=strcmp(mode,'rotate');
        panButton.Value=strcmp(mode,'pan');
    end
    function startViewDrag(~,~)
        point=fig.CurrentPoint; bounds=getpixelposition(ax,true);
        % Exclude the toolbar strip so clicking a tool does not start a drag.
        if ~strcmp(fig.SelectionType,'normal') || ...
                point(1)<bounds(1) || point(1)>bounds(1)+bounds(3) || ...
                point(2)<bounds(2) || point(2)>bounds(2)+bounds(4)-32
            return;
        end
        dragState=struct('Point',point,'Mode',viewMode);
    end
    function moveViewDrag(~,~)
        if isempty(dragState), return; end
        point=fig.CurrentPoint; shift=point-dragState.Point;
        dragState.Point=point;
        if strcmp(dragState.Mode,'rotate')
            offset=ax.CameraPosition-ax.CameraTarget;
            radius=norm(offset);
            azimuth=atan2d(offset(2),offset(1))-0.4*shift(1);
            elevation=atan2d(offset(3),hypot(offset(1),offset(2)));
            elevation=max(-85,min(85,elevation+0.4*shift(2)));
            ax.CameraPosition=ax.CameraTarget+radius* ...
                [cosd(elevation)*cosd(azimuth),cosd(elevation)*sind(azimuth),sind(elevation)];
            ax.CameraUpVector=[0 0 1];
        else
            direction=ax.CameraTarget-ax.CameraPosition;
            radius=norm(direction); direction=direction/radius;
            horizontal=cross(direction,ax.CameraUpVector);
            horizontal=horizontal/norm(horizontal);
            vertical=cross(horizontal,direction);
            bounds=getpixelposition(ax,true);
            scale=2*radius*tand(ax.CameraViewAngle/2)/max(1,bounds(4));
            translation=-scale*(shift(1)*horizontal+shift(2)*vertical);
            ax.CameraPosition=ax.CameraPosition+translation;
            ax.CameraTarget=ax.CameraTarget+translation;
        end
        drawnow limitrate;
    end
    function stopViewDrag()
        dragState=[];
    end
    function resetRobotView()
        stopViewDrag(); selectViewMode('rotate');
        names=fieldnames(homeView);
        for n=1:numel(names), ax.(names{n})=homeView.(names{n}); end
        drawnow limitrate;
    end
    function zoomRobotView(steps)
        % At most three wheel notches per event; keep the model in a useful range.
        steps=max(-3,min(3,steps));
        ax.CameraViewAngle=max(12,min(45,ax.CameraViewAngle*1.10^steps));
        drawnow limitrate;
    end
    function scrollRobotView(~,event)
        % Scrolling the left input panel or matrix must not change the camera.
        point=fig.CurrentPoint;
        bounds=getpixelposition(ax,true);
        if point(1)>=bounds(1) && point(1)<=bounds(1)+bounds(3) && ...
                point(2)>=bounds(2) && point(2)<=bounds(2)+bounds(4)
            zoomRobotView(event.VerticalScrollCount);
        end
    end
    function switchRobotType()
        nextParam=param;
        nextParam.hasPrismatic=~param.hasPrismatic;
        if param.hasPrismatic
            savedExtension=q(4); nextQ=q(1:3);
        else
            nextQ=[q(1:3) savedExtension];
        end
        rebuildRobot(nextParam,nextQ);
        if param.hasPrismatic
            titleLabel.Text='3R + Prismatic Robot';
            typeLabel.Text='Active: 3 revolute + 1 prismatic joint';
            typeButton.Text='Switch to 3R Robot';
            extensionField.Enable='on'; extensionSlider.Enable='on';
            extensionField.Value=q(4); extensionSlider.Value=q(4);
        else
            titleLabel.Text='3R Robot';
            typeLabel.Text='Active: 3 revolute joints';
            typeButton.Text='Switch to 3R + Prismatic J4';
            extensionField.Enable='off'; extensionSlider.Enable='off';
        end
        status.Text='Robot type changed. XYZ now follows the current joints.';
    end
    function moveExtension(value,finished)
        if ~param.hasPrismatic, return; end
        if ~isfinite(value) || value<param.prismaticLimits(1) || value>param.prismaticLimits(2)
            extensionField.Value=q(4); return;
        end
        q(4)=value; savedExtension=value; extensionField.Value=value;
        if finished, extensionSlider.Value=value; end
        scene.target.Visible='off';
        T=getTransform(robot,robotConfiguration(q),'tool'); target=T(1:3,4).';
        syncTargets(); refresh(finished);
        status.Text='J4 extension changed; XYZ recalculated. Inverse Kinematics keeps this J4.';
    end
    function closeSettings()
        if ~isempty(settingsFig) && isgraphics(settingsFig)
            delete(settingsFig);
        end
        settingsFig=[];
    end
    function openSettings()
        if ~isempty(settingsFig) && isgraphics(settingsFig)
            figure(settingsFig); return;
        end
        settingsFig=uifigure('Name','Robot Parameters (metres)', ...
            'Position',[150 180 440 340],'Tag','ParameterSettings', ...
            'Visible',fig.Visible,'WindowStyle','modal');
        layout=uigridlayout(settingsFig,[7 2]);
        layout.ColumnWidth={'1x',150};
        layout.RowHeight={50,36,36,36,36,36,'1x'};
        note=uilabel(layout,'Text', ...
            'Enter positive lengths in metres. Apply keeps the joint angles and recalculates XYZ.', ...
            'WordWrap','on'); note.Layout.Column=[1 2];
        keys={'baseHeight','L1','L2','L3'};
        labels={'Base height (m)','L1: vertical link (m)', ...
            'L2: upper arm (m)','L3: forearm (m)'};
        edits=gobjects(1,4);
        for n=1:4
            uilabel(layout,'Text',labels{n});
            edits(n)=uieditfield(layout,'numeric','Value',param.(keys{n}), ...
                'Tag',['Parameter_' keys{n}]);
        end
        uibutton(layout,'Text','Cancel','ButtonPushedFcn',@(~,~) closeSettings());
        uibutton(layout,'Text','Apply','Tag','ApplyParameters', ...
            'ButtonPushedFcn',@(~,~) applyParameters(edits,keys));
        msg=uilabel(layout,'Text','','WordWrap','on','Tag','ParameterMessage');
        msg.Layout.Column=[1 2];
    end
    function applyParameters(edits,keys)
        values=arrayfun(@(h) h.Value,edits);
        if any(~isfinite(values)) || any(values<=0) || ~isfinite(sum(values)^2)
            msg=findobj(settingsFig,'Tag','ParameterMessage');
            msg.Text='Enter finite positive lengths. Parameters have not been changed.';
            return;
        end
        nextParam=param;
        for n=1:4, nextParam.(keys{n})=values(n); end
        rebuildRobot(nextParam,q);
        status.Text='Parameters applied. Joint values retained; XYZ and matrix recalculated.';
        closeSettings();
    end
    function rebuildRobot(nextParam,nextQ)
        stopViewDrag();
        [nextRobot,nextParam]=makeRobot(nextParam);
        % Recreate geometry because the old meshes contain the old lengths.
        delete(scene.bodies); delete(scene.frames); delete(scene.target);
        if isfield(scene,'extension'), delete(scene.extension); delete(scene.tool); end
        robot=nextRobot; param=nextParam; q=nextQ; scene=[];
        T=getTransform(robot,robotConfiguration(q),'tool'); target=T(1:3,4).';
        reach=param.L2+param.L3;
        if param.hasPrismatic, reach=reach+param.prismaticLimits(2); end
        height=param.baseHeight+param.L1;
        limits=[-reach reach; -reach reach; height-reach height+reach];
        for k=1:3
            targetSliders(k).Limits=limits(k,:);
            targetSliders(k).MajorTicks=linspace(limits(k,1),limits(k,2),3);
        end
        syncTargets();
        % Keep the displayed ground at Z = 0 for both robot types.
        margin=max(.08,.15*max(reach,height));
        ax.XLim=[-reach-margin reach+margin]; ax.YLim=ax.XLim;
        ax.ZLim=[0 height+reach+margin];
        center=[0 0 mean(ax.ZLim)]; span=max([diff(ax.XLim),diff(ax.ZLim)]);
        ax.CameraTarget=center; ax.CameraPosition=center+span*[-1.5 1.5 1.2];
        ax.CameraUpVector=[0 0 1]; ax.CameraViewAngle=42;
        homeView=captureRobotView();
        setappdata(fig,'RobotModel',robot); setappdata(fig,'RobotParameters',param);
        refresh(true);
    end

    function moveJoint(index,value,finished)
        q(index)=value; fields(index).Value=value;
        if finished, sliders(index).Value=value; end
        scene.target.Visible='off';
        if q(3)>1e-9, branch.Value=1; elseif q(3)<-1e-9, branch.Value=2; end
        T=getTransform(robot,robotConfiguration(q),'tool'); target=T(1:3,4).';
        syncTargets(); refresh(finished); status.Text='Joint control';
    end
    function moveTarget(index,value,finished)
        target(index)=value; targetFields(index).Value=value;
        if finished, syncTargets(); end
        solveTarget(finished);
    end
    function syncTargets()
        for k=1:3
            if ~isfinite(target(k)), continue; end
            targetFields(k).Value=target(k);
            bounds=targetSliders(k).Limits;
            targetSliders(k).Limits=[min(bounds(1),target(k)) max(bounds(2),target(k))];
            targetSliders(k).Value=target(k);
        end
    end
    function solveTarget(force)
        if ~force && toc(lastUpdate)<.04, return; end
        if any(~isfinite(target))
            scene.target.Visible='off'; status.Text='Enter finite X, Y and Z values.'; return
        end
        candidate=solveIK(target,param,branch.Value,q);
        if isempty(candidate)
            set(scene.target,'XData',target(1),'YData',target(2),'ZData',target(3), ...
                'Visible','on','Color',[.95 .2 .15]);
            status.Text='Target unavailable for this elbow and current J4 within joint limits. Robot stays at its last pose.';
        else
            q=candidate; scene.target.Visible='off';
            for k=1:3, fields(k).Value=q(k); sliders(k).Value=q(k); end
            if param.hasPrismatic
                status.Text=sprintf('Position control. J4 held at %.4f m.',q(4));
            else
                status.Text='Position control';
            end
        end
        refresh(force);
    end
    function refresh(force)
        if ~force && toc(lastUpdate)<.04, return; end
        scene=drawRobot(ax,param,robot,q,scene);
        T=getTransform(robot,robotConfiguration(q),'tool');
        transformTable.Data=T;
        jointText=sprintf('J1 %.4f   J2 %.4f   J3 %.4f deg',q(1:3));
        if param.hasPrismatic, jointText=sprintf('%s\nJ4 %.4f m',jointText,q(4)); end
        tip.Text=sprintf('OUTPUT\n%s\nX %+.6f   Y %+.6f\nZ %+.6f m',jointText,T(1:3,4).');
        setappdata(fig,'JointConfiguration',robotConfiguration(q));
        if param.hasPrismatic, setappdata(fig,'RobotType','3R+P'); else, setappdata(fig,'RobotType','3R'); end
        setappdata(fig,'JointAngles',q); lastUpdate=tic;
        if force, drawnow nocallbacks; else, drawnow limitrate nocallbacks; end
    end
end

function [robot,param] = makeRobot(param)
%CREATEROBOT3DOF Model dimensions in metres; robot configurations in radians.
if nargin==0
param.L1 = 0.18; param.L2 = 0.22; param.L3 = 0.18;
param.baseRadius = 0.05; param.baseHeight = 0.07;
param.link1Radius = 0.025;
param.link2Width = 0.050; param.link2Height = 0.050;
param.link3Width = 0.045; param.link3Height = 0.045;
param.jointRadius = 0.025; param.toolRadius = 0.018;
param.frameLength = 0.055; param.robotOpacity = 0.45;
param.jointLimits = [-180 180; -90 90; -120 120]; % degrees
param.initialAngles = [30 35 -40];
param.hasPrismatic = false;
param.prismaticLimits = [0 .15]; % metres; J4 moves along local +X
end
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',6);
base = rigidBody('base_link');
base.Joint = rigidBodyJoint('base_fixed','fixed');
addVisual(base,'Cylinder',[param.baseRadius param.baseHeight], ...
    trvec2tform([0 0 param.baseHeight/2]));
addBody(robot,base,robot.BaseName);
body = rigidBody('link1');
joint = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint,trvec2tform([0 0 param.baseHeight]));
joint.JointAxis = [0 0 1]; joint.PositionLimits = deg2rad(param.jointLimits(1,:));
body.Joint = joint;
addVisual(body,'Cylinder',[param.link1Radius param.L1],trvec2tform([0 0 param.L1/2]));
addVisual(body,'Sphere',param.jointRadius,trvec2tform([0 0 param.L1]));
addBody(robot,body,'base_link');
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint,trvec2tform([0 0 param.L1]));
joint.JointAxis = [0 1 0]; joint.PositionLimits = deg2rad(param.jointLimits(2,:));
body.Joint = joint;
addVisual(body,'Box',[param.L2 param.link2Width param.link2Height],trvec2tform([param.L2/2 0 0]));
addVisual(body,'Sphere',param.jointRadius,trvec2tform([param.L2 0 0]));
addBody(robot,body,'link1');
body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint,trvec2tform([param.L2 0 0]));
joint.JointAxis = [0 1 0]; joint.PositionLimits = deg2rad(param.jointLimits(3,:));
body.Joint = joint;
addVisual(body,'Box',[param.L3 param.link3Width param.link3Height],trvec2tform([param.L3/2 0 0]));
addVisual(body,'Sphere',param.toolRadius,trvec2tform([param.L3 0 0]));
addBody(robot,body,'link2');
if param.hasPrismatic
    body=rigidBody('slider_base');
    joint=rigidBodyJoint('slider_mount','fixed');
    setFixedTransform(joint,trvec2tform([param.L3 0 0]));
    body.Joint=joint; addBody(robot,body,'link3');
    body=rigidBody('tool');
    joint=rigidBodyJoint('joint4','prismatic');
    joint.JointAxis=[1 0 0]; joint.PositionLimits=param.prismaticLimits;
    body.Joint=joint;
    addVisual(body,'Sphere',param.toolRadius);
    addBody(robot,body,'slider_base');
else
body = rigidBody('tool');
joint = rigidBodyJoint('tool_fixed','fixed');
setFixedTransform(joint,trvec2tform([param.L3 0 0]));
body.Joint = joint;
addBody(robot,body,'link3');
end
end

function config=robotConfiguration(q)
% Revolute joints use radians; the optional prismatic joint stays in metres.
config=q(:); config(1:3)=deg2rad(config(1:3));
end

function [frames,bodies] = modelFrames(robot,q)
% World, J1, J2, J3, tool: all independent of the student algorithms.
names={'base_link','link1','link2','link3','tool'};
if numel(q)==4, names={'base_link','link1','link2','link3','slider_base','tool'}; end
frames=zeros(4,4,numel(names));
for k=1:numel(names)
    frames(:,:,k)=getTransform(robot,robotConfiguration(q),names{k});

    disp(names{k});
    disp(frames(:,:,k));
end
bodies=frames(:,:,1:4);
end

function scene = drawRobot(ax,param,robot,angles,scene)
%RENDERROBOT3DOF Persistent geometry; never calls show, cla or resets axes.
[frames,bodies] = modelFrames(robot,angles);
if nargin < 5 || isempty(scene)
    scene.bodies = gobjects(1,4); scene.frames = gobjects(1,size(frames,3));
    for k=1:4
        scene.bodies(k) = hgtransform('Parent',ax,'Tag','RobotBody');
    end
    cylinderMesh(scene.bodies(1),param.baseRadius,param.baseHeight,[0.3 0.35 0.4]);
    cylinderMesh(scene.bodies(2),param.link1Radius,param.L1,[0.3 0.6 0.85]);
    ball(scene.bodies(2),[0 0 param.L1],param.jointRadius);
    boxMesh(scene.bodies(3),param.L2,param.link2Width,param.link2Height,[0.9 0.55 0.15]);
    ball(scene.bodies(3),[param.L2 0 0],param.jointRadius);
    boxMesh(scene.bodies(4),param.L3,param.link3Width,param.link3Height,[0.3 0.7 0.65]);
    ball(scene.bodies(4),[param.L3 0 0],param.toolRadius);
    frameLabels={'W','J1','J2','J3','EE'};
    if param.hasPrismatic
        frameLabels={'W','J1','J2','J3','J4','EE'};
        scene.extension=hgtransform('Parent',ax,'Tag','PrismaticExtension');
        boxMesh(scene.extension,1,.022,.022,[.65 .65 .72]);
        scene.tool=hgtransform('Parent',ax,'Tag','PrismaticTool');
        ball(scene.tool,[0 0 0],param.toolRadius);
    end
    for k=1:size(frames,3)
        scene.frames(k) = drawCoordinateFrame(ax,frames(:,:,k),param.frameLength,frameLabels{k});
    end
    scene.target = line(ax,NaN,NaN,NaN,'LineStyle','none','Marker','x', ...
        'Color',[0.8 0.1 0.65],'MarkerSize',12,'LineWidth',2,'Tag','IKTarget', ...
        'HitTest','off','PickableParts','none');
end
for k=1:4
    scene.bodies(k).Matrix = bodies(:,:,k);
end
for k=1:size(frames,3)
    scene.frames(k).Matrix = frames(:,:,k);
end
if param.hasPrismatic
    % hgtransform requires a nonsingular matrix even for a retracted rod.
    scene.extension.Matrix=frames(:,:,5)*diag([max(angles(4),1e-6) 1 1 1]);
    if angles(4)==0, scene.extension.Visible='off'; else, scene.extension.Visible='on'; end
    scene.tool.Matrix=frames(:,:,6);
end

    function cylinderMesh(parent,radius,height,color)
        [X,Y,Z]=cylinder(radius,24);
        surf('Parent',parent,'XData',X,'YData',Y,'ZData',Z*height, ...
            'FaceColor',color,'FaceAlpha',param.robotOpacity,'EdgeColor','none', ...
            'HitTest','off','PickableParts','none');
        for cap=[0 height]
            patch('Parent',parent,'XData',X(1,:),'YData',Y(1,:), ...
                'ZData',cap*ones(size(X(1,:))),'FaceColor',color, ...
                'FaceAlpha',param.robotOpacity,'EdgeColor','none', ...
                'HitTest','off','PickableParts','none');
        end
    end
    function ball(parent,center,radius)
        [X,Y,Z]=sphere(16);
        surf('Parent',parent,'XData',center(1)+radius*X, ...
            'YData',center(2)+radius*Y,'ZData',center(3)+radius*Z, ...
            'FaceColor',[0.25 0.3 0.4],'FaceAlpha',param.robotOpacity, ...
            'EdgeColor','none','HitTest','off','PickableParts','none');
    end
    function boxMesh(parent,len,width,height,color)
        vertices=[0 -1 -1; 0 1 -1; 0 1 1; 0 -1 1; ...
                  1 -1 -1; 1 1 -1; 1 1 1; 1 -1 1].*[len width/2 height/2];
        faces=[1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1];
        patch('Parent',parent,'Vertices',vertices,'Faces',faces,'FaceColor',color, ...
            'FaceAlpha',param.robotOpacity,'EdgeColor',color*0.6, ...
            'HitTest','off','PickableParts','none');
    end
end

function h = drawCoordinateFrame(ax,T,frameLength,label,h)
%DRAWCOORDINATEFRAME Create once, then update the same transform handle.
if nargin < 5 || isempty(h) || ~isgraphics(h)
    h = hgtransform('Parent',ax,'Tag','RobotCoordinateFrame');
    colors = [0.85 0.15 0.15; 0.1 0.65 0.2; 0.15 0.35 0.95];
    names = 'XYZ';
    for k=1:3
        endpoint = zeros(3,1); endpoint(k)=frameLength;
        line('Parent',h,'XData',[0 endpoint(1)],'YData',[0 endpoint(2)], ...
            'ZData',[0 endpoint(3)],'Color',colors(k,:),'LineWidth',2, ...
            'Marker','none','HitTest','off','PickableParts','none');
        text('Parent',h,'Position',1.15*endpoint.', ...
            'String',[names(k) label],'Color',colors(k,:),'FontSize',10, ...
            'HitTest','off','PickableParts','none');
    end
    text('Parent',h,'Position',[0 0 -0.015],'String',['{' label '}'], ...
        'FontWeight','bold','HitTest','off','PickableParts','none');
end
h.Matrix = T;
end



function q = solveIK(target,param,branch,seed)
target=target(:); seed=seed(:).'; q=[];
extension=0;
if param.hasPrismatic, extension=seed(4); end
seed=seed(1:3);
effectiveL3=param.L3+extension;
fprintf('\n========== IK DEBUG ==========\n');
fprintf('Target XYZ (m): [%.6f %.6f %.6f]\n',target);
fprintf('Previous J1 J2 J3 (deg): [%.6f %.6f %.6f]\n',seed);
fprintf('Elbow branch: %d (1 = positive J3, 2 = negative J3)\n',branch);
if param.hasPrismatic
    fprintf('J4 held fixed = %.6f m; effective forearm L3 + J4 = %.6f m\n',extension,effectiveL3);
end
r=hypot(target(1),target(2));
v=param.baseHeight+param.L1-target(3);
D=(r^2+v^2-param.L2^2-effectiveL3^2)/(2*param.L2*effectiveL3);
fprintf('r = %.9f m, v = %.9f m, D = %.12f\n',r,v,D);
if abs(D)>1+1e-12
    fprintf('IK FAILED: target outside geometric reach.\n');
    fprintf('Joint angles unchanged.\n========== END IK DEBUG ==========\n');
    return;
end
D=max(-1,min(1,D));
if abs(1-abs(D))<1e-12, D=sign(D); end
fprintf('D after numerical correction = %.12f\n',D);
theta3=(3-2*branch)*acos(D);
if r<1e-12, yaw=deg2rad(seed(1)); else, yaw=atan2(target(2),target(1)); end
candidates=zeros(2,3); count=0;
for radialSign=[1 -1]
    theta1=yaw+(radialSign<0)*pi;
    theta2=atan2(v,radialSign*r)-atan2(effectiveL3*sin(theta3), ...
        param.L2+effectiveL3*cos(theta3));
    fprintf('\nCandidate radialSign = %+d\n',radialSign);
    fprintf('theta1 theta2 theta3 (rad): [%.9f %.9f %.9f]\n',theta1,theta2,theta3);
    fprintf('theta1 theta2 theta3 (deg, before wrapping): [%.6f %.6f %.6f]\n', ...
        rad2deg([theta1 theta2 theta3]));
    candidate=mod(rad2deg([theta1 theta2 theta3])+180,360)-180;
    if abs(candidate(1)+180)<1e-10 && seed(1)>0, candidate(1)=180; end
    fprintf('Wrapped J1 J2 J3 (deg): [%.6f %.6f %.6f]\n',candidate);
    if all(candidate>=param.jointLimits(:,1).'-1e-9 & ...
            candidate<=param.jointLimits(:,2).'+1e-9)
        candidate=max(param.jointLimits(:,1).',min(param.jointLimits(:,2).',candidate));
        count=count+1; candidates(count,:)=candidate;
        fprintf('Candidate ACCEPTED: within joint limits.\n');
    else
        fprintf('Candidate REJECTED: outside joint limits.\n');
    end
end
if count==0
    fprintf('IK FAILED: no candidate within joint limits for this elbow.\n');
    fprintf('Joint angles unchanged.\n========== END IK DEBUG ==========\n');
    return;
end
candidates=candidates(1:count,:);
delta=mod(candidates-seed+180,360)-180;
[~,index]=min(sum(delta.^2,2));
q=candidates(index,:);
fprintf('\nSelected J1 J2 J3 (deg): [%.6f %.6f %.6f]\n',q);
fprintf('Change from previous angles (deg, new minus old): [%.6f %.6f %.6f]\n',q-seed);
fprintf('Shortest wrapped change (deg): [%.6f %.6f %.6f]\n',delta(index,:));
fprintf('IK SUCCESS\n========== END IK DEBUG ==========\n');
if param.hasPrismatic, q=[q extension]; end
end
