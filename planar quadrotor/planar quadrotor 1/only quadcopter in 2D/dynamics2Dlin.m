function statedot=dynamics2Dlin(t,state)
g=9.8;
l=1;
M=1;
I=1;

% F1=(u1+u2/l)/2;
% F2=(u1-u2/l)/2;

setx=10;
setxv=0;
setxacc=0;
sety=0;
setyv=0;
setyacc=0;
setth=0;
setthv=0;

% xdata=10:10:100;
% ydata=10:10:100;
% 
% for i=length(ydata):1
%     if state(3)<ydata(i)-3
%         sety=ydata(i);
%         setx=xdata(i);
%     end
% end



% if state(3)<29
%     setx=30; sety=30;
%     if state(3)<19
%         setx=20; sety=20;
%         if state(3)<9
%             setx=10; sety=10;
%         end
%     end
% end

% if state(3)<59
% 	setx=60; sety=60;
% 	if state(3)<49
%         	setx=50; sety=50;
% 				setx=30; sety=30;
% 				end
% 				if state(3)<19
%  					setx=20; sety=20;
% 					if state(3)<9
% 						setx=10; sety=10;
% 					end
% 			end
% 		end
% 	end
% end
% 	        if state(3)<39
% 			setx=40; sety=40;
% 			if state(3)<29
% setx=10
% for i = 59:-10:9
%     if(state(3)<i)
%         setx=i+5;
%         sety=i+5;
%     end
% end
% setxx=10:10:70;
% setyy=10:10:70;
% 
% for i=5:1
%     if (state(1)-setxx(i))^2 + (state(3)-setyy(i))^2 >= (state(1)-setxx(i+1))^2 + (state(3)-setyy(i+1))^2
%         setx=state(i+1)
%         sety=state(i+1)
%         disp(setx)
%         disp(sety)
%     end
%     
% end

% disp([state(1) setx sety])

% kpx=0.08;
% kdx=0.65;
% kpy=0.5;
% kdy=0.5;
% kpth=0.5;
% kdth=1;

% kpx=0.5;
% kdx=0.5;
% kpy=10;
% kdy=10;
% kpth=10;
% kdth=10;

% kpx=0.076;
% kdx=0.5;
% kpy=1;
% kdy=6;
% kpth=1;
% kdth=1;

% kpx=0.077;
% kdx=0.5;
% kpy=1;
% kdy=6;
% kpth=3;
% kdth=2;

kpx=0.7;
kdx=1;
kpy=0.35;
kdy=0.65;
kpth=6;
kdth=3.25;

setth=(-1/g)*(setxacc + kpx*(setx-state(1)) + kdx*(setxv-state(2)));
u1=M*(g + setyacc + kpy*(sety-state(3)) + kdy*(setyv-state(4)));
u2=kpth*(setth-state(5)) + kdth*(setthv-state(6));

statedot(1)=state(2);
statedot(2)=-(u1/M)*sin(state(5));
statedot(3)=state(4);
statedot(4)=(u1/M)*cos(state(5)) - g;
statedot(5)=state(6);
statedot(6)=u2/I;
statedot=statedot';
end