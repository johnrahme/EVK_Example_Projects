db_connect;
global conn;
tablename = 'positions';
colnames = {'x','y','sessionId'};

numberOfUpdates = 300;
i = 0;
x = 0;
y = 0;
dx = 2;
dy = 1;
while(i<numberOfUpdates)
    if(i == 150)
        dx = -2;
    end
    x = x+dx;
    y = y+dy;
    
    data = {x,y,1};
    data_table = cell2table(data,'VariableNames',colnames);
    fastinsert(conn,tablename,colnames,data_table);
    pause(0.1);
    i = i+1;
end

%sqlquery = 'select * from positions';

%curs = exec(conn,sqlquery);
%curs = fetch(curs);
%curs.Data