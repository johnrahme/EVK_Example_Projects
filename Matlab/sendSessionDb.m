function []  = sendSessionDb(placeId,running)
global conn;
startTime = datestr(now(), 'yyyy-mm-dd HH:MM:SS.FFF');
tablename = 'sessions';
colnames = {'placeId','running','startedAt'};

data = {placeId,running,startTime};
data_table = cell2table(data,'VariableNames',colnames);
fastinsert(conn,tablename,colnames,data_table);

end