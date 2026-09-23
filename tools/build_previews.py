from pathlib import Path
import json
import numpy as np
from scipy.linalg import solve_continuous_are, expm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[1]
p = json.loads((ROOT/'solutions/config.json').read_text())
Ms,Mu,Ks,Cs,Kt,Ct = (p[k] for k in ('Ms','Mu','Ks','Cs','Kt','Ct'))
a = Cs/Ms+Cs/Mu+Ct/Mu
A = np.array([[0,1,0,0],[-Cs*Ct/(Ms*Mu),0,Cs/Ms*a-Ks/Ms,-Cs/Ms],[Ct/Mu,0,-a,1],[Kt/Mu,0,-(Ks/Ms+Ks/Mu+Kt/Mu),0]])
Bu = np.array([0,1/Ms,0,1/Ms+1/Mu])
Bw = np.array([0,Cs*Ct/(Ms*Mu),-Ct/Mu,-Kt/Mu])
C = np.array([0,0,1,0])
Aa = np.zeros((5,5)); Aa[:4,:4]=A; Aa[4,:4]=C
Ba = np.r_[Bu,0]; Wa=np.r_[Bw,0]
COLORS={'NONE':'#64748b','PID':'#d97706','LQR':'#007f8b'}
NAMES={'NONE':'Passive','PID':'PID','LQR':'LQR + integral'}


def validate_model():
    # Independently construct the conventional physical plant q=[zs,vs,zu,vu].
    Ap=np.array([[0,1,0,0],[-Ks/Ms,-Cs/Ms,Ks/Ms,Cs/Ms],[0,0,0,1],[Ks/Mu,Cs/Mu,-(Ks+Kt)/Mu,-(Cs+Ct)/Mu]])
    bp=np.array([0,1/Ms,0,-1/Mu])
    wp=np.array([0,0,0,Kt/Mu]); dp=np.array([0,0,0,Ct/Mu])
    T=np.array([[1,0,0,0],[0,1,0,0],[1,0,-1,0],[a-Ct/Mu,1,-a,-1]])
    h=np.array([0,0,0,Ct/Mu])
    np.testing.assert_allclose(T@Ap@np.linalg.inv(T),A,atol=1e-10)
    np.testing.assert_allclose(T@bp,Bu,atol=1e-12)
    np.testing.assert_allclose(T@wp-A@h,Bw,atol=1e-10)
    np.testing.assert_allclose(T@dp+h,0,atol=1e-12)
    assert np.max(np.linalg.eigvals(A).real)<0
    for mat,b in [(A,Bu),(Aa,Ba)]:
        ctrb=np.column_stack([np.linalg.matrix_power(mat,i)@b for i in range(len(b))])
        assert np.linalg.matrix_rank(ctrb)==len(b)
    # Transfer functions checked against independently derived two-mass formulas.
    for w in np.logspace(-2,3,30):
        s=1j*w
        delta=(Ms*s*s+Cs*s+Ks)*(Mu*s*s+(Cs+Ct)*s+Ks+Kt)-(Cs*s+Ks)**2
        gu=((Ms+Mu)*s*s+Ct*s+Kt)/delta
        gw=-Ms*s*s*(Ct*s+Kt)/delta
        np.testing.assert_allclose(C@np.linalg.solve(s*np.eye(4)-A,Bu),gu,rtol=1e-9)
        np.testing.assert_allclose(C@np.linalg.solve(s*np.eye(4)-A,Bw),gw,rtol=1e-9)


def simulate(mode,road,dt=None):
    dt=p['dt'] if dt is None else dt
    F=0.
    if mode=='LQR':
        P=solve_continuous_are(Aa,Ba[:,None],np.diag(p['Q']),np.array([[p['R']]]))
        K=Ba@P/p['R']
        residual=Aa.T@P+P@Aa-np.outer(P@Ba,Ba@P)/p['R']+np.diag(p['Q'])
        assert np.linalg.norm(residual)/np.linalg.norm(np.diag(p['Q']))<1e-7
    elif mode=='PID':
        K=np.r_[p['Kp']*C+p['Kd']*(C@A),p['Ki']]
        F=-p['Kd']*(C@Bw)
    else:
        K=np.zeros(5)
    Ac=Aa-np.outer(Ba,K); Wc=Wa+Ba*F
    poles=np.linalg.eigvals(Ac)
    if mode!='NONE':
        assert max(poles.real)<0
        eq=np.linalg.solve(Ac,-Wc*p['step_amplitude'])
        np.testing.assert_allclose(eq[[0,1,2]],[p['step_amplitude'],0,0],atol=1e-8)
    t=np.arange(round(p['duration']/dt)+1)*dt
    road_signal=p['step_amplitude']*(t>=p['step_time']) if road=='step' else p['sine_amplitude']*np.sin(2*np.pi*p['sine_frequency']*t)
    n=6 if road=='step' else 7
    H=np.zeros((n,n)); H[:5,:5]=Ac; H[:5,5]=Wc
    if road=='sine':
        H[5,6]=1; H[6,5]=-(2*np.pi*p['sine_frequency'])**2
    E=expm(H*dt)
    z=np.zeros((len(t),n))
    if road=='sine': z[0,6]=p['sine_amplitude']*2*np.pi*p['sine_frequency']
    for j in range(1,len(t)):
        z[j]=E@z[j-1]
        if road=='step': z[j,5]=road_signal[j]
    x=z[:,:5]
    np.testing.assert_allclose(z[:,5],road_signal,atol=1e-10)
    # Exact interval energy integrals via a finite-horizon observability Gramian.
    hu=np.zeros(n); hu[:5]=-K; hu[5]=F
    hs=np.zeros(n); hs[:4]=A[2]; hs[5]=Bw[2]
    ha=(-Cs*hs+hu)/Ms; ha[2]-=Ks/Ms
    ht=np.zeros(n); ht[0]=1; ht[2]=-1; ht[5]=-1
    energies=[]
    for h in (ha,ht,hu):
        V=np.block([[-H.T,np.outer(h,h)],[np.zeros_like(H),H]])
        ev=expm(V*dt)
        G=E.T@ev[:n,n:]
        energies.append(float(np.einsum('ij,jk,ik->',z[:-1],G,z[:-1])))
    sd=x[:,:4]@A[2]+Bw[2]*road_signal
    u=-x@K+F*road_signal
    if mode=='PID':
        np.testing.assert_allclose(u,-p['Kp']*x[:,2]-p['Kd']*sd-p['Ki']*x[:,4],atol=1e-7)
    acc=(-Ks*x[:,2]-Cs*sd+u)/Ms
    assert np.all(np.isfinite(x))
    return dict(t=t,zr=road_signal,zs=x[:,0],zu=x[:,0]-x[:,2],s=x[:,2],u=u,acc=acc,K=K,poles=poles,mode=mode,road=road,energies=energies)


def metrics(r):
    t=r['t']; rms=lambda y:float(np.sqrt(np.trapezoid(y*y,t)/(t[-1]-t[0])))
    er=np.sqrt(np.maximum(r['energies'],0)/(t[-1]-t[0]))
    m=dict(acceleration_rms_m_s2=float(er[0]),tire_deflection_rms_m=float(er[1]),max_suspension_deflection_m=float(max(abs(r['s']))),force_rms_kN=float(er[2])/1000,force_peak_kN=float(max(abs(r['u']))/1000),body_overshoot_percent=None,body_settling_time_s=None,suspension_recovery_time_s=None)
    if r['road']=='step':
        ix=np.flatnonzero(t>=p['step_time']); y=r['zs'][ix]; target=p['step_amplitude']
        m['body_overshoot_percent']=float(max(0,100*max(np.sign(target)*(y-target))/abs(target)))
        def settling(error):
            outside=np.flatnonzero(abs(error)>0.02*abs(target))
            return 0. if not len(outside) else ('not settled' if outside[-1]==len(ix)-1 else float(t[ix[outside[-1]+1]]-p['step_time']))
        m['body_settling_time_s']=settling(y-target)
        m['suspension_recovery_time_s']=settling(r['s'][ix])
    return m


def plot_runs(runs,road):
    fig,axs=plt.subplots(3,2,figsize=(11,9),layout='constrained')
    keys=[('zs','Body displacement (m)'),('s','Suspension deflection (m)'),('acc','Body acceleration (m/s²)'),('tire','Tire deflection (m)'),('u','Actuator force (kN)'),('zr','Road displacement (m)')]
    for ax,(key,label) in zip(axs.flat,keys):
        for r in runs:
            y=r['zu']-r['zr'] if key=='tire' else r[key]/1000 if key=='u' else r[key]
            ax.plot(r['t'],y,color=COLORS[r['mode']],lw=1.3,label=NAMES[r['mode']])
        ax.set(xlabel='Time (s)',ylabel=label,xlim=(0,p['duration']))
        if key in ('acc','u') and road=='step':
            ax.set_yscale('symlog',linthresh=1)
            ax.set_title('Symmetric-log scale: preserves step spikes',fontsize=9)
        ax.grid(alpha=.2)
    axs[0,0].legend(fontsize=9)
    fig.suptitle('Quarter-Car Control Lab | '+road.capitalize()+' road input',fontsize=17,fontweight='bold')
    fig.savefig(ROOT/f'solutions/figures/{road}-comparison.png',dpi=150)
    plt.close(fig)


def animate(runs,road):
    fig,axes=plt.subplots(1,3,figsize=(10,4.8))
    fig.subplots_adjust(left=.025,right=.975,bottom=.17,top=.82,wspace=.12)
    fig.suptitle('Quarter-Car Control Lab | '+road.capitalize()+' road input',fontsize=16,fontweight='bold')
    objects=[]
    for ax,r in zip(axes,runs):
        color=COLORS[r['mode']]
        ax.set(xlim=(-.65,.65),ylim=(-.15,2.0),xticks=[],yticks=[])
        ax.set_title(NAMES[r['mode']],color=color,fontweight='bold')
        for spine in ax.spines.values(): spine.set_visible(False)
        body=Rectangle((-.4,1),.8,.22,color=color)
        wheel=Rectangle((-.19,.23),.38,.12,color='#334155')
        ax.add_patch(body); ax.add_patch(wheel)
        spring,=ax.plot([],[],color=color,lw=2)
        tire,=ax.plot([],[],color='#334155',lw=3)
        ground,=ax.plot([],[],color='#64748b',lw=3)
        label=ax.text(0,1.90,'',ha='center',va='top',fontsize=10)
        objects.append((body,wheel,spring,tire,ground,label))
    fig.text(.5,.08,'Schematic: fixed drawing offsets; vertical motion scale 1:1',ha='center',fontsize=9,color='#475569')
    stamp=fig.text(.5,.035,'',ha='center',fontsize=10)
    # Include the exact step-onset frame; 20 fps, real-time playback.
    frames=np.arange(0,len(runs[0]['t']),round(.05/p['dt']))
    def update(k):
        for r,(body,wheel,spring,tire,ground,label) in zip(runs,objects):
            b=1+r['zs'][k]; w=.23+r['zu'][k]; z=r['zr'][k]
            body.set_y(b); wheel.set_y(w)
            sy=np.linspace(w+.12,b,17); sx=.065*(-1.)**np.arange(17)
            spring.set_data(sx,sy); tire.set_data([0,0],[z,w])
            ground.set_data([-.6,.6],[z,z])
            label.set_text(f"deflection {1000*r['s'][k]:+.1f} mm\nforce {r['u'][k]/1000:+.1f} kN")
        stamp.set_text(f"t = {runs[0]['t'][k]:.2f} s   |   identical road input and mechanical parameters")
    ani=FuncAnimation(fig,update,frames=frames,interval=50)
    ani.save(ROOT/f'media/animations/{road}-comparison.gif',writer=PillowWriter(fps=20),dpi=85)
    plt.close(fig)


def main():
    for directory in ['solutions/figures','media/animations']:
        (ROOT/directory).mkdir(parents=True,exist_ok=True)
    validate_model()
    results={}
    for road in ['step','sine']:
        for mode in ['NONE','PID','LQR']:
            coarse=simulate(mode,road); fine=simulate(mode,road,p['dt']/2)
            np.testing.assert_allclose(coarse['energies'],fine['energies'],rtol=1e-7,atol=1e-8)
            np.testing.assert_allclose(coarse['zs'],fine['zs'][::2],atol=1e-8)
    lines=['# Reference comparison','', 'Generated from `config.json`; 10 s records; exact continuous step/sine propagation with 1 ms output samples and exact interval RMS integration. These are illustrative gains, not an optimized design.','', 'Body overshoot and body settling are supplemental metrics. Suspension recovery uses an absolute ±0.004 m band (2% of the 0.2 m road step). Percentage overshoot about zero suspension deflection is undefined. See [clarifications](../docs/clarifications.md).','']
    for road in ['step','sine']:
        runs=[simulate(mode,road) for mode in ['NONE','PID','LQR']]
        results[road]={r['mode']:metrics(r) for r in runs}
        lines += ['## '+road.capitalize(),'', '| Metric | Passive | PID | LQR |','|---|---:|---:|---:|']
        for key in results[road]['NONE']:
            vals=[results[road][m][key] for m in ['NONE','PID','LQR']]
            fmt=lambda v:'N/A' if v is None else v if isinstance(v,str) else f'{v:.4g}'
            lines.append('| '+key.replace('_',' ')+' | '+' | '.join(map(fmt,vals))+' |')
        lines.append('')
        plot_runs(runs,road)
        animate(runs,road)
    (ROOT/'solutions/results.json').write_text(json.dumps(results,indent=2)+'\n')
    (ROOT/'solutions/results.md').write_text('\n'.join(lines)+'\n')
    print('PASS: physical coordinate transform, transfer functions, controllability, passive stability, active closed-loop stability, LQR Riccati residual, step equilibria, PID force law, and time-step convergence of states and exact RMS integrals.')
    print(json.dumps(results,indent=2))

if __name__=='__main__':
    main()
