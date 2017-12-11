double ExtCall(double w1, double w2, double A, double M, double time)
{
double res;
res=A*cos(w1*time+(w2-w1)*time*time/(2*M));
return res;
}