% Frame

% %% Init Data Frame
% DataSet( counter , 1 : dataLen*agent ) = ...
% 	zeros( 1 ,  AgentNumber * dataLen );
% 
% 
% 	
% %% Location of agent
% for agent = 1 : AgentNumber
%     [ x , y , rot ]= ...
% 		fun_trackInterface( theClient , agent );% Location
%     DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+3 )  = [ x , y , rot ];
% end
% clear x y rot
% 
% 
% 
% %% Reading of Sensor
% for agent = 1 : AgentNumber
% 	DataSet( counter , dataLen*(agent-1)+4 ) = ...
% 		fun_SensorReader( mbed(agent) , DataSet( counter-1 , dataLen*(agent-1)+4 ) );
% end



%%  Simple Filter for Reading
for agent = 1 : AgentNumber
	DataSet( counter , dataLen*(agent-1)+5 ) = ...
		ffactor(1) * ( DataSet( counter-2 , dataLen*(agent-1)+4 ) - sensorBG(agent) + iniBG ) + ...
		ffactor(2) * ( DataSet( counter-1 , dataLen*(agent-1)+4 ) - sensorBG(agent) + iniBG ) + ...
		ffactor(3) * ( DataSet( counter-0 , dataLen*(agent-1)+4 ) - sensorBG(agent) + iniBG ) + ...
		0;
	% prepare to show
	values( agent ) = DataSet( counter , dataLen*(agent-1)+5 );
end


%% Show Values
display('Sensor Reading:')
values
clear values

% timer
timer(counter) = toc;


%% Run Algorithm
% Input DataSet / s
% Output filterOut / s
% prepare data
values = zeros( 2 , AgentNumber );
rn = zeros( 2 , AgentNumber );
rk = zeros( 2 , AgentNumber );
for agent = 1 : AgentNumber
	% sensor reading for this frame
	values( 1 , agent ) = DataSet( counter-0 , dataLen*(agent-1)+5 );
	% sensor reading for last frame
	values( 2 , agent ) = DataSet( counter-1 , dataLen*(agent-1)+5 );
	% location of agent this frame
	rn( : , agent ) = DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 )';
	% location of agent last frame
	rk( : , agent ) = DataSet( counter-1 , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 )';
end
rc = mean( rn' )';
rck = mean( rk' )';
% measure average distance between agent and center
for agent = 1 : AgentNumber
    drc(:,agent) = rn(:,agent) - rc;
    drc(1,agent) = drc(1,agent)^2;
    drc(2,agent) = drc(2,agent)^2;
end
aveDist = mean(sum(drc));
% Estimate Laplacian ( Last Frame )
l1 = ( 1 / (16*aveDist ) ) *...
    ( ( sum( values(2,:) ) ) - AgentNumber * s.x( 5 , 1 ) );
% Call cooperative Kalman filter
values2 = [ values( 1 , : )' ; values( 2 , : )' ];
s = fun_kalmanf3...
	( s , values2 , rn(:,1) , rn(:,2) , rn(:,3) , rn(:,4) , ...
	rc , rk(:,1) , rk(:,2) , rk(:,3) , rk(:,4) , rck , l1 );
% Estimate Laplacian Again ( This Frame )
l2 = ( 1 / (aveDist ) ) *...
    ( ( sum( values(1,:) ) ) - AgentNumber * s.x( 5 , 1 ) );
% Estimate Error in the Center of Agent Group
error = s.x(1,1) - s.x(5,1);
errort = error / dt;
% Center Gradient
grad = [ s.x(2,1) ; s.x(3,1) ] / norm([ s.x(2,1) , s.x(3,1) ]);
% Next Center
rc = rc + gradCoe * dt*grad;
% Collect Data to DataSet
filterOut( counter , : ) = zeros( 1 , filOLen );
filterOut( counter , 1 ) = rc(1);
filterOut( counter , 2 ) = rc(2);
filterOut( counter , 3 ) = grad(1);
filterOut( counter , 4 ) = grad(2);
filterOut( counter , 5 ) = error;
filterOut( counter , 7 ) = l2;
clear rc rck rn rk  values grad error l
	

%% Test
% filterOut( counter , 1 ) = 0;
% filterOut( counter , 2 ) = 0;
	
	
	
%% Formation Control for Khepera Robot
% Input filterOut
% Output DataSet
[r1,r2,r3,r4] = fun_jacobi( initDist(1,:)' , initDist(2,:)' , ...
	initDist(3,:)' , initDist(4,:)' , filterOut( counter , 1:2 )' );
DataSet( counter , dataLen*(1-1)+6:dataLen*(1-1)+7 ) = r1';
DataSet( counter , dataLen*(2-1)+6:dataLen*(2-1)+7 ) = r2';
DataSet( counter , dataLen*(3-1)+6:dataLen*(3-1)+7 ) = r3';
DataSet( counter , dataLen*(4-1)+6:dataLen*(4-1)+7 ) = r4';
clear r1 r2 r3 r4



%% Parameter Estimation
r = fun_RLS1 (r,errort,filterOut(counter,7));
est = r.x;
if est < estThreshold
	est = estThreshold;
end
s.x(4,1) = est;
filterOut( counter , 6 ) = est;
clear est



%% Prepare Instruction for Agent
for agent = 1 : AgentNumber
	% Calculate the velocity and turning angle for agent
	alpha1 = iniOri( agent , 2 );% Initial Orientation in Tracking System
	alpha2 = iniOri( agent , 1 );% Initial Orientation in x-y Coordinate
	beta1 = DataSet( counter , dataLen*(agent-1)+3 );% Orientation in Tracking Sys
	dx = DataSet( counter , dataLen*(agent-1)+6 ) - ...
			DataSet( counter , dataLen*(agent-1)+1 );
	dy = DataSet( counter , dataLen*(agent-1)+7 ) - ...
			DataSet( counter , dataLen*(agent-1)+2 );
	beta2 = atan2( dx , dy ) * 180 / pi;
	% calculate velocity
	velocity = sqrt( dx^2 + dy^2 );
	DataSet( counter , dataLen*(agent-1)+8 ) = velocity;
	% calculate angle
	angle = beta2 - ( alpha2 + ( beta1 - alpha1 ) );
    angle = fun_rotRegulator( angle );% angle regulator
    angle = angle * pi / 180;
	DataSet( counter , dataLen*(agent-1)+9 ) = angle * 180 / pi;
	% Translate to wheel speed for agent
	lspeed = vmulti * velocity + angle * wmulti * ( agentLen / 2 );
	DataSet( counter , dataLen*(agent-1)+10 ) = lspeed;
	rspeed = vmulti * velocity - angle * wmulti * ( agentLen / 2 );
	DataSet( counter , dataLen*(agent-1)+11 ) = rspeed;
	
end
clear alpha1 alpha2 beta1 beta2 velocity angle lspeed rspeed



%% Send Instruction to Khepera
inst2Khepera = fun_int2instruction( 0 , 0 );
for agent = 1 : AgentNumber
	lspeed = DataSet( counter , dataLen*(agent-1)+10 );
	rspeed = DataSet( counter , dataLen*(agent-1)+11 );
	inst2Khepera = fun_int2instruction( lspeed , rspeed );
%  	kheperaOutput( agent ).writeBytes( inst2Khepera );
end
clear lspeed rspeed



%% Draw Figure
% draw source
scatter(rs(1),rs(2));
hold on;
rn = zeros( 2 , AgentNumber );
for agent = 1 : AgentNumber
	% location of agent this frame
	rn( : , agent ) = DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 )';
end
rc = mean( rn' )';
% center now
scatter(rc(1),rc(2),'+');
rct = filterOut(counter,1:2)';
% target 
scatter( rct(1) , rct(2) , '*' );
% direction
plot([rc(1),rct(1)],[rc(2),rct(2)]);
grid on;
axis([-2,2,-2,2])
for agent = 1 : AgentNumber
    % trajectory
	x = DataSet( 3:counter , dataLen*(agent-1)+1 );
	y = DataSet( 3:counter , dataLen*(agent-1)+2 );
	scatter( x , y , '.' );
    % khepera
	x0 = DataSet( counter , dataLen*(agent-1)+1 );
	y0 = DataSet( counter , dataLen*(agent-1)+2 ); 
	khepera = scatter( x0 , y0 , 'O' );
    % sensor 
    co2 = DataSet( counter , dataLen*(agent-1)+5 );
    text( x0 , y0 , num2str(co2) );
    drawnow;
end
% zero direction
for agent = 1 : AgentNumber
    rnow = DataSet( counter , dataLen*(agent-1)+1:dataLen*(agent-1)+2 )';
    rnext = DataSet( counter , dataLen*(agent-1)+6:dataLen*(agent-1)+7 )';
    rdir = rnow+1.5.*iniOri( agent , 4:5 )';
    plot([rnow(1),rdir(1)],[rnow(2),rdir(2)],'LineWidth',2);
    plot([rnow(1),rnext(1)],[rnow(2),rnext(2)],'LineWidth',2);
end

hold off;



%% Pause
pause(movPause);