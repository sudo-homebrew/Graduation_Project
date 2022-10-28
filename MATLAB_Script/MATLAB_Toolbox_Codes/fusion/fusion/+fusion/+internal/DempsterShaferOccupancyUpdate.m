function [m_o, m_f] = DempsterShaferOccupancyUpdate(m_o1, m_f1, m_o2, m_f2)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2020 The MathWorks, Inc.

% [m_o, m_f] = fusion.internal.DempsterShaferOccupancyUpdate(m_o1,m_f1,m_o2,m_f2)
% combines the occupancy evidences m_o1, m_o2 and free evidences m_f1, m_f2
% from two sources using Dempster-Shafer combination rule.

%#codegen
m_u1 = max(0, 1 - m_o1 - m_f1);
m_u2 = max(0, 1 - m_o2 - m_f2);

K = m_f1.*m_o2 + m_o1.*m_f2;
oneK = max(eps,1 - K);

m_o = m_u1.*m_o2 + m_o1.*m_u2 + m_o1.*m_o2;
m_o = m_o./oneK;
m_f = m_u1.*m_f2 + m_f1.*m_u2 + m_f1.*m_f2;
m_f = m_f./oneK;
m_o = max(0,min(m_o,1));
m_f = min(m_f,1 - m_o);
end