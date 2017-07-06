function b = bearing(DX, DY, T)

b = atan2( -DX*sin(T) + DY*cos(T), DX*cos(T)+DY*sin(T) );