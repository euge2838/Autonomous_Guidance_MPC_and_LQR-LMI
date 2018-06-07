function ang = limitare_unghi(a)
  if (a < -pi)
    ang = a + 2*pi;
  elseif (a > pi)
    ang = a - 2*pi;
  else
    ang = a;
  end