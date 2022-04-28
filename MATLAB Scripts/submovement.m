function [pos, vel] = submovement( t, ti, pi, pf, D )


if      t <= ti
    pos = pi;
    vel = 0;
    
elseif  ti <= t && t <= ti + D
    pos = pi + (pf - pi)   * ( 10 * ( (t-ti)/D )^3 - 15 * ( (t-ti)/D )^4 + 6  *( (t-ti)/D )^5  );
    vel = (pf - pi)/D * ( 30 * ( (t-ti)/D )^2 - 60 * ( (t-ti)/D )^3 + 30 *( (t-ti)/D )^4  );
else
    pos = pf;
    vel = 0;

end
