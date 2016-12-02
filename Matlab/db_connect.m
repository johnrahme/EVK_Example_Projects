clear all;
javaaddpath('mysql-connector-java-5.1.40-bin.jar');
global conn;
conn = database('uwb','root','','Vendor','MySQL','Server','localhost');
