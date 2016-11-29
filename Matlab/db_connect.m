clear all;
javaaddpath('mysql-connector-java-5.1.40-bin.jar');
conn = database('uwb','root','','Vendor','MySQL','Server','localhost');
            
tablename = 'positions';
colnames = {'x','y'};
              
data = {1,3};
data_table = cell2table(data,'VariableNames',colnames);
fastinsert(conn,tablename,colnames,data_table);

sqlquery = 'select * from positions';

curs = exec(conn,sqlquery);
curs = fetch(curs);
curs.Data(1,4)
