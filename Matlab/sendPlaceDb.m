function []  = sendPlaceDb(name,url,width,height,xPosA,yPosA,xPosB,yPosB,xPosC,yPosC)
global conn;
tablename = 'places';
colnames = {'name','url','width','height','coordAX','coordAY','coordBX','coordBY','coordCX','coordCY'};

data = {name,url,width,height,xPosA,yPosA,xPosB,yPosB,xPosC,yPosC};
data_table = cell2table(data,'VariableNames',colnames);
fastinsert(conn,tablename,colnames,data_table);

%Example
%sendPlaceDb('TestPlace','Test.jpg',10,20,1.',3.4,1.2,3.4,1.2,3.4)
end