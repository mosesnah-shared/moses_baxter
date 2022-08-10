function [pos, vel] = submovement( t, ti, pi, pf, D )

% Check whether t is an array or not 

if      t <= ti
    pos = pi;
    vel = zeros( 1, length( pi ) );
    
elseif  ti <= t && t <= ti + D
    pos = pi + (pf - pi)   * ( 10 * ( (t-ti)/D )^3 - 15 * ( (t-ti)/D )^4 + 6  *( (t-ti)/D )^5  );
    vel = (pf - pi)/D * ( 30 * ( (t-ti)/D )^2 - 60 * ( (t-ti)/D )^3 + 30 *( (t-ti)/D )^4  );
else
    pos = pf;
    vel = zeros( 1, length( pi ) );

end
