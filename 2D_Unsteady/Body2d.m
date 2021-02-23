

% The class for arbitrary body

% Methods: Body2d(x,y,ccw) x,y: coordinates; ccw: (counter)clockwise
%          glide(self,theta,a,b,x0,y0) theta: Angle; a,b: coordinates
%          moved; x0, y0: origin of the pitch axis

% Added 20210219: acceleration, velocity of the body

classdef Body2d < handle
    properties
        x;      % horizontal coordinates of body nodes
        y;      % vertical coordinates of body nodes
        n;      % number of nodes
        panels; % cell array of Panel2d objects
        perim;  % perimeter (sum of panel lengths) of the body
        vx;     % velocity in x direction.
        vy;     % velocity in y direction.
        ax;     % acceleration in x direction.
        ay;     % acceleration in y direction.
        m;      % Mass
    end % properties
    
    methods
        % x:    x coordinates of nodes along body's perimeter
        % y:    y coordiantes of nodes along body's perimeter
        % ccw:  true/false if panel normal vectors should be rotated
        %       counterclockwise/clockwise from panel tangent vectors.
        %       Default=false (i.e. clockwise).
        function obj = Body2d(x,y,ccw)
            obj.vx = 0; obj.vy = 0; obj.ax = 0; obj.ay = 0; % Starting from zero motion
            obj.m =1;
            if (nargin > 0)
                if (nargin < 3)
                    ccw = false;
                end
                if (length(x) ~= length(y))
                    error('Length mismatch');
                end
                obj.x = x;
                obj.y = y;
                obj.n = length(obj.x);
                obj.panels{obj.n - 1} = [];
                for i=1:obj.n - 1
                    obj.panels{i} = Panel2d(x(i),y(i),x(i+1),y(i+1),ccw);
                end
                obj.perim = 0;
                for i=1:obj.n - 1
                    obj.perim = obj.perim + obj.panels{i}.len;
                end
            end
        end
        
        function npan = getNumberOfPanels(self)
            npan = length(self.panels);
        end
        
        function [xm,ym] = getMidpoints(self)
            N = length(self.panels);
            xm = zeros(1,N);
            ym = zeros(1,N);
            for i=1:N
                xm(i) = self.panels{i}.xmid;
                ym(i) = self.panels{i}.ymid;
            end
        end
        
        function glide(self,theta,a,b,x0,y0)
            if (nargin < 5)
                x0 = 0;
                y0 = 0;
            end
            if (nargin < 3)
                a = 0;
                b = 0;
            end
            cth = cos(theta);
            sth = sin(theta);
            % Action of SE2 motion on nodes
            xx = (self.x - x0)*cth - (self.y-y0)*sth + x0 + a;
            yy = (self.x - x0)*sth + (self.y-y0)*cth + y0 + b;
            self.x = xx;
            self.y = yy;
            % Action of SE2 motion on panels
            for i=1:length(self.panels)
                self.panels{i}.x1 = xx(i);
                self.panels{i}.y1 = yy(i);
                self.panels{i}.x2 = xx(i+1);
                self.panels{i}.y2 = yy(i+1);
                self.panels{i}.update();
            end
        end
    end % methods
end