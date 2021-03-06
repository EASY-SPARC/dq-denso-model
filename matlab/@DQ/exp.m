% The function exp(g) returns the exponential of the pure dual quaternion g

% (C) Copyright 2015 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br

function res=exp(g)
g=DQ(g);

if Re(g) ~= 0
   error('The exponential operation is defined only for pure dual quaternions');
end

phi=norm(g.P.q);

if(phi ~= 0)
    prim = cos(phi) + (sin(phi)/phi)*g.P;
else
    prim = DQ(1);
end

res = prim+DQ.E*g.D*prim;

end
