%javaaddpath('mysql-connector-java-5.1.40-bin.jar');
global conn;
conn = database('uwb','test','test','Vendor','MySQL','Server','192.168.1.104');
