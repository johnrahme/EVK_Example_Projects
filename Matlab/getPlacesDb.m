function [places]  = getPlacesDb()
global conn;
sqlquery = 'select name from places';
curs = exec(conn,sqlquery);
curs = fetch(curs);
places = curs.Data;
end