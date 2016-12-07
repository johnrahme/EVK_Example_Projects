%javaaddpath('mysql-connector-java-5.1.40-bin.jar');
global conn;
%conn = database('uwb','test','test','Vendor','MySQL','Server','192.168.220.1');
%Live
conn = database('futf_se_db_2', 'john@f160124','6EAUbs6sdCYb', 'Vendor', 'MySQL','Server','mysql525.loopia.se');
