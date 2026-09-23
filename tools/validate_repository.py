"""Independent ODE/energy check and repository-link checks (run from any cwd)."""
from pathlib import Path
import re
import numpy as np
from scipy.integrate import solve_ivp
import build_previews as b

root=Path(__file__).resolve().parents[1]
for mode in ['NONE','PID','LQR']:
    for road in ['step','sine']:
        r=b.simulate(mode,road)
        K=r['K']; F=-b.p['Kd']*(b.C@b.Bw) if mode=='PID' else 0
        def rhs(t,y):
            w=b.p['step_amplitude'] if road=='step' else b.p['sine_amplitude']*np.sin(2*np.pi*b.p['sine_frequency']*t)
            x=y[:5]; sd=b.A[2]@x[:4]+b.Bw[2]*w
            u=-K@x+F*w
            acc=(-b.Ks*x[2]-b.Cs*sd+u)/b.Ms
            tire=x[0]-x[2]-w
            return np.r_[b.Aa@x+b.Ba*u+b.Wa*w,acc*acc,tire*tire,u*u]
        start=b.p['step_time'] if road=='step' else 0.
        out=solve_ivp(rhs,(start,b.p['duration']),np.zeros(8),method='Radau',rtol=1e-9,atol=1e-11)
        assert out.success
        np.testing.assert_allclose(out.y[5:,-1],r['energies'],rtol=2e-6,atol=1e-8)
        np.testing.assert_allclose(out.y[0,-1],r['zs'][-1],atol=2e-7)
        print('PASS independent ODE and energy:',mode,road)
for path in root.rglob('*.md'):
    text=path.read_text(encoding='utf-8-sig')
    for target in re.findall(r'\]\(([^)]+)\)',text):
        if target.startswith(('http:','https:','#')): continue
        target=target.split('#')[0]
        assert (path.parent/target).exists(),(path,target)
for path in root.rglob('*'):
    if path.is_file() and '.git' not in path.parts:
        assert path.stat().st_size<50*1024*1024,path
print('PASS local Markdown links and artifact file sizes')
