%TM5_700 Virtual Model, Peter corkes Serial Link object

L(1)= Revolute('d', 145.1, ...   % link length (Dennavit-Hartenberg notation)
    'a', 0, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', -pi/2, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [-277 277]*pi/180 ); % minimum and maximum joint angle

L(2)= Revolute('offset', -pi/2, ...
    'd', 0, ...   % link length (Dennavit-Hartenberg notation)
    'a', 329, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [-187 187]*pi/180 ); % minimum and maximum joint angle

L(3)= Revolute('offset', 0, ...
    'd', 0, ...   % link length (Dennavit-Hartenberg notation)
    'a', 311.5, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [-162 162]*pi/180 ); % minimum and maximum joint angle

L(4)= Revolute('offset', pi/2, ...
    'd', -122.2, ...   % link length (Dennavit-Hartenberg notation)
    'a', 0, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', pi/2, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [-187 187]*pi/180 ); % minimum and maximum joint angle

L(5)= Revolute('offset', 0, ...
    'd', 106, ...   % link length (Dennavit-Hartenberg notation)
    'a', 0, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', pi/2, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [-187 187]*pi/180 ); % minimum and maximum joint angle

L(6)= Revolute('offset', 0, ...
    'd', 114.4, ...   % link length (Dennavit-Hartenberg notation)
    'a', 0, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [-277 277]*pi/180 ); % minimum and maximum joint angle

tm5_700=SerialLink(L,'name', 'TM5_700');


