function []  = sendPositionDb(xPos,yPos,sessionId)
global conn;
tablename = 'positions';
colnames = {'x','y','sessionId'};

data = {xPos,yPos,sessionId};
data_table = cell2table(data,'VariableNames',colnames);
fastinsert(conn,tablename,colnames,data_table);

clear conn;

end