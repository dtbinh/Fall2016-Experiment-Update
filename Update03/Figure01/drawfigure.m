% scatter

[counter,~]  = size(DataSet);
radKhepera = 0.06;
basis2pi = 0:0.02:2*pi;

% pale green / spring green / chartreuse / olivedrab
% colorKhepera = [ 84 , 139 , 84 ; 0 , 139 , 69 ; 69 , 139 , 0 ; 105 , 139 , 34 ]./256;% dark
% colorKhepera = [ 124 , 205 , 124 ; 0 , 205 , 102 ; 102  ,205 , 0 ; 154 , 205 , 50 ]./256;% light

% RoyalBlue / DodgerBlue / SteelBlue / DeepSkyBlue
% colorKhepera = [ 39 , 64 , 139 ; 16 , 78 , 139 ; 54 , 100 , 139 ; 0  ,104 , 139 ]./256;%dark
% colorKhepera = [ 58 , 95 , 205 ; 24 , 116 , 205 ; 79 , 148 , 205 ; 0 , 154 , 205 ]./256;%light

% MediumPurple / DarkOrchid / MediumOrchid / Orchid
% colorKhepera = [ 93 , 71 , 139 ; 104 , 34 , 139 ; 122 ,55,139 ; 139 ,71 , 137 ]./256;%dark
% colorKhepera = [ 127 ,104 , 205 ; 154 , 50 , 205 ; 180 , 82 , 205 ; 205 , 105 , 201 ]./256;%light

% LightSalmon / Orange / Coral / Tomato
% colorKhepera = [ 139 , 87 , 66 ; 139 , 90 , 0; 139 , 62 , 47 ; 139 , 54 , 38]./256;%dark
% colorKhepera = [ 205 , 129 , 98 ; 205 , 133 , 0 ; 205 , 91 , 69 ; 205 , 79 , 57]./256;%light

% PaleTurquoise / Turquoise / Cyan /Aquamarine 
% colorKhepera = [ 102 , 139 , 139 ; 0 , 134, 139 ; 0 , 139 , 139 ; 69 , 139 , 116]./256;%dark
% colorKhepera = [ 150 , 205 , 205 ; 0 , 197 , 205 ; 0 , 205 , 205 ; 102 , 205 , 170 ]./256;%light

% khaki / Gold / Goldenrod / Tan
% colorKhepera = [ 139 , 134 , 78 ; 138 , 117, 0 ; 139 , 105,25;139,90,43 ]./256;%dark
% colorKhepera = [ 205,198,115;205,173,0;205,119,29;205,133,63]./256;%light



scatter(rs(1),rs(2));
hold on;
rn = zeros( 2 , AgentNumber );
for agent = 1 : AgentNumber
	% location of agent this frame
	rn( : , agent ) = DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 )';
end
rc = mean( rn' )';
% center now
plot(rc(1),rc(2),'+','color' , colorKhepera(1,:));
rct = filterOut(counter,1:2)';
% target 
plot( rct(1) , rct(2) , '*' ,'color' , colorKhepera(2,:));
% direction
% plot([rc(1),rct(1)],[rc(2),rct(2)]);
grid on;
axis([-2,2,-2,2])
for agent = 1 : AgentNumber
    % trajectory
	x = DataSet( 3:counter , dataLen*(agent-1)+1 );
	y = DataSet( 3:counter , dataLen*(agent-1)+2 );
	plot( x , y , 'Marker','.','Markersize',0.5 ,'color' , colorKhepera(agent,:));
    % khepera
	xc = DataSet( counter , dataLen*(agent-1)+1 );
	yc = DataSet( counter , dataLen*(agent-1)+2 ); 
    sqx(agent) = xc;
    sqy(agent) = yc;
    x0 = xc + radKhepera * cos(basis2pi);
    y0 = yc + radKhepera * sin(basis2pi);
    plot(x0,y0,'color' , colorKhepera(agent,:));
   
% 	khepera = scatter( x0 , y0 , 'O' );
    % sensor 
%     co2 = DataSet( counter , dataLen*(agent-1)+5 );
%     text( xc , yc , num2str(co2) );
    % direction
    rnow = [ xc ; yc ];
    rnext = radKhepera.* ( rct - rc ) ./ norm( rct-rc ) + rnow;
    plot([rnow(1),rnext(1)],[rnow(2),rnext(2)],'LineWidth',1.5,'color' , colorKhepera(agent,:));
    drawnow;
end
% square
sqx(AgentNumber+1) = sqx(1);
sqy(AgentNumber+1) = sqy(1);
plot(sqx,sqy,'color',colorKhepera(3,:),'LineStyle',':');
% % zero direction
% for agent = 1 : AgentNumber
%     rnow = DataSet( counter , dataLen*(agent-1)+1:dataLen*(agent-1)+2 )';
%     rnext = DataSet( counter , dataLen*(agent-1)+6:dataLen*(agent-1)+7 )';
%     rdir = rnow+1.5.*iniOri( agent , 4:5 )';
%     plot([rnow(1),rdir(1)],[rnow(2),rdir(2)],'LineWidth',2);
%     plot([rnow(1),rnext(1)],[rnow(2),rnext(2)],'LineWidth',2);
% end