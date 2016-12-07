db_connect;
global conn;
tablename = 'positions';
colnames = {'x','y','sessionId'};

numberOfUpdates = 100;
i = 0;
x = 0;
y = 0;
dx = 2;
dy = 1;
A = [];
while(i<numberOfUpdates)
    if(i == numberOfUpdates/2)
        dx = -2;
    end
    x = x+dx;
    y = y+dy;
    
    data = {x,y,1};
    data_table = cell2table(data,'VariableNames',colnames);
    tic;
    fastinsert(conn,tablename,colnames,data_table);
    A = [A toc];
    pause(0.1);
    i = i+1;
end

disp('Mean: ');
disp(mean(A));
disp('Max: ');
disp(max(A));
%sqlquery = 'select * from positions';

%curs = exec(conn,sqlquery);
%curs = fetch(curs);
%curs.Data