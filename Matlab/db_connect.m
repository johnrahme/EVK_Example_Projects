clear all;
javaaddpath('mysql-connector-java-5.1.40-bin.jar');
conn = database('uwb','test','test','Vendor','MySQL','Server','192.168.1.104');
            
tablename = 'positions';
colnames = {'x','y','sessionId'};
              
data = {1,3,1};
data_table = cell2table(data,'VariableNames',colnames);
fastinsert(conn,tablename,colnames,data_table);

sqlquery = 'select * from positions';

curs = exec(conn,sqlquery);
curs = fetch(curs);
curs.Data
