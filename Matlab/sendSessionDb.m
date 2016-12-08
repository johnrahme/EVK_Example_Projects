function []  = sendSessionDb(placeId,running)
global conn;
tablename = 'sessions';
colnames = {'placeId','running'};

data = {placeId,running};
data_table = cell2table(data,'VariableNames',colnames);
fastinsert(conn,tablename,colnames,data_table);

end