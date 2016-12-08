function [result]  = getQuery(sqlquery)
global conn;
curs = exec(conn,sqlquery);
curs = fetch(curs);
result = curs.Data;
end