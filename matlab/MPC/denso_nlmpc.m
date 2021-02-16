nx = 14;
ny = 14;
nu = 6;
nlobj = nlmpc(nx,ny,nu);

Ts = 0.1;
p = 30;
c = 4;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;

nlobj.Model.StateFcn = "densoStateFunction";
nlobj.Jacobian.StateFcn = "densoJacobianFunction";

denso = DQ_DENSO;

theta = [0, 0, 0, 0, 0, 0]';
xm = denso.fkm(theta);
x0 = [vec8(xm); theta];
u0 = [0, 0, 0, 0, 0, 0]';

validateFcns(nlobj,x0,u0);