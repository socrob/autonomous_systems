%%Script
% Path 1

v=130;              %velocity

pause on;           %enables pauses

%strart of the program
pause(4);
walk_front(v);
pause(7.5);
turn('right',130);
pause(8);
turn('back','right');
pause(2);
walk_front(v);
pause(8.5);
turn('left',130);
pause(5);
turn('back','left');

%%
pause on;
walk_front(130);
pause(7.5);
stop();

%%
turn('right',0);
pause(8);
stop();

%%
turn('back','right');

%%
walk_front(v);
pause(8.5);
stop();

%%
turn('left',130);
pause(5);
stop();

%%
turn('back','left');