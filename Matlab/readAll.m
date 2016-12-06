
    port = 'COM3'; %Where 3 is COMport number (usually standard)
    BR = 9600; % BaudRate of port
    obj = serial(port, 'BaudRate', BR); % Creating object to read serial % with baudrate 9600
    obj.BytesAvailableFcnMode = 'terminator';
    obj.BytesAvailableFcn = @callbackCom;
    fopen(obj); %opens object
    %fprintf(obj,'*IDN?')
    
    pause(0.8);
    %out = fscanf(s);
    
%     i = 0;
%     while(i<100)
%         A = fread(obj,300, 'schar');
%         s = sprintf('%.2x', A);
%         %data = sscanf('D1: %i D:2 %i D3: %i', xdata);
%         string = char(A);
%         string = string(~isspace(string));
%         string = string'
%         %C = strsplit(string,delimiter);
%         %data = sscanf('D1: %i D:2 %i D3: %i', string);
%         pause(0.02);
%         i = i + 1;
%     end
      
    
    fclose(obj);
    delete(obj);
    clear obj;
    function callbackCom(obj,event)
        fprintf(obj,'*IDN?')
        %fscanf(obj, ['D1: %d D2: %d D3: %d'])
    end