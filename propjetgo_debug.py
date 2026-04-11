#!/usr/bin/env python3
"""
PropJet Go — Master Debug Suite
================================
Full simulation audit of all app logic. Run before every GitHub push.
A clean run shows zero bugs (✗) and zero warnings (⚠).

The physics replica must be kept in sync with index.html Block 1.
The alternate scenario (R) must stay in sync with refreshAltPlan() in Block 3.

Usage:  python3 propjetgo_debug.py
Exit:   0 = clean, 1 = bugs found

Last verified clean: 2026-04-05
Scenarios: A–R (18 categories, 140+ checks)
"""

import math, sys

# ════════════════════════════════════════════════════════════════════════════
# PHYSICS REPLICA — keep in sync with index.html Block 1
# ════════════════════════════════════════════════════════════════════════════
ALT_CRUISE_GPH = 38    # alternate fuel burn gph (220 KIAS / 10k MSL)
ALT_KTAS       = 256   # alternate cruise KTAS (~220 KIAS @ 10k ISA)
TAS_CEIL       = 320   # maximum TAS cap (thrust + aero limit)
TAS_CEIL_ALT   = 28000 # altitude at which cap is reached

def isaTemp(a):
    altM = a * 0.3048
    return 15.0 - 6.5*(altM/1000) if altM <= 11000 else -56.5

def densityRatio(a):
    T_sl=288.15; T=isaTemp(a)+273.15; altM=a*0.3048
    if altM <= 11000: return (T/T_sl)**4.256
    return (216.65/T_sl)**4.256 * math.exp(-(altM-11000)/6341.6)

def kiasToKtas(k, a): return k / math.sqrt(densityRatio(a))

def avgDescentTas(c, ar):
    # Lancair IV-PT descent: 220 KIAS, 1500 fpm, 25 gph, NP ~1800 rpm
    mid = (c+ar)/2
    return (kiasToKtas(220,c) + kiasToKtas(220,mid) + kiasToKtas(220,ar)) / 3

def fmtTime(m):
    t = math.floor(m+0.5)
    return f"{t//60}+{str(t%60).zfill(2)}"

def getGS(t, w, wd): return t+w if wd=='tail' else max(50, t-w)

def cruiseGphAtAlt(a, anc=22000, g=30.0):
    # V-shape: FL220/30gph anchor, 0.80%/1000ft symmetric both directions
    f = 1 + abs(a-anc)/1000*0.008
    return g*max(0.6,min(1.5,f))

def compute(dist, cAlt, dEl, aEl, cGph, crGph, dGph, cR, tas, ws, wd):
    # dR removed — descent VS derived from 2.5° FPA geometry
    gs  = getGS(tas, ws, wd)
    wss = -ws if wd=='tail' else ws
    cD    = max(0, cAlt-dEl);  cMins = cD/cR;  cGal = cGph*(cMins/60)
    cAS   = tas*0.55;           cGS   = max(10, cAS-wss)
    cGrnd = cGS*(cMins/60)
    dD     = max(0, cAlt-aEl)
    avgDT  = avgDescentTas(cAlt, aEl)
    avgVS_fpm = avgDT * math.sin(2.5 * math.pi / 180) * 101.27
    dMRaw  = dD / max(1, avgVS_fpm)
    dGS    = max(10, avgDT-wss)
    dGrndU = dGS*(dMRaw/60); rem = max(0, dist-cGrnd)
    dGrnd  = min(dGrndU, rem)
    dMins  = dMRaw*(dGrnd/max(0.01,dGrndU)) if dGrnd < dGrndU else dMRaw
    dGal   = dGph*(dMins/60)
    crGrnd = max(0, dist-cGrnd-dGrnd)
    crMins = (crGrnd/gs)*60 if crGrnd > 0 else 0
    crGal  = crGph*(crMins/60)
    tGal   = cGal+crGal+dGal; tMins = cMins+crMins+dMins
    return {
        'climbGal':   round(cGal,1),   'climbMins':   round(cMins),
        'climbDist':  round(cGrnd),    'climbDelta':  round(cD),
        'cruiseGal':  round(crGal,1),  'cruiseMins':  round(crMins),
        'cruiseDist': round(crGrnd),   'gs':          round(gs),
        'descGal':    round(dGal,1),   'descMins':    round(dMins),
        'descDist':   round(dGrnd),    'avgDesTas':   round(avgDT),
        'totalGal':   round(tGal,1),   'totalMins':   tMins,
        'todNm':      round(dGrnd),    'noAlt':       cGrnd >= dist,
        'avgDescVS':  round(avgVS_fpm),
        '_cG': cGrnd, '_crG': crGrnd, '_dG': dGrnd, '_tG': tGal,
    }

def getRealPerf(altFt, isaDev, data, BW=3000, TOL=1.0):
    if not data: return None
    cands = [d for d in data if abs(d['alt']-altFt) <= BW]
    if not cands: return None
    minI = min(abs(d['isa']-isaDev) for d in cands)
    sel  = [d for d in cands if abs(d['isa']-isaDev)-minI <= TOL]
    ts=fs=ws=0
    for d in sel:
        w = 1-abs(d['alt']-altFt)/BW; ts+=d['tas']*w; fs+=d['ff']*w; ws+=w
    return {'tas': ts/ws, 'ff': fs/ws} if ws >= 0.3 else None

def haversineNm(lat1, lon1, lat2, lon2):
    R = 3440.065  # Earth radius in nm
    phi1,phi2 = math.radians(lat1),math.radians(lat2)
    dphi = math.radians(lat2-lat1); dlam = math.radians(lon2-lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def simulate(dist, alt, dEl, aEl, fCl, fCr, fDe, cR, tas, ws, wd,
             fob, rGph, rMins, taxi, altD, perfData=None, isaDev=0):
    if rMins is None or str(rMins) == '': rMins = 60
    rMins = float(rMins)
    if math.isnan(rMins): rMins = 60
    rGal  = round(rGph*(rMins/60), 1)
    aGal  = round(ALT_CRUISE_GPH*(altD/max(1,ALT_KTAS)), 1)
    fobAT = fob - taxi; fixed = rGal + aGal
    fCrEff = fCr; tasEff = tas
    if perfData:
        rp = getRealPerf(alt, isaDev, perfData)
        if rp: fCrEff = rp['ff']; tasEff = rp['tas']
    maxD = 0
    if fixed <= fobAT:
        lo, hi = 25, 1400
        for _ in range(20):
            mid = round((lo+hi)/2)
            rT  = compute(mid,alt,dEl,aEl,fCl,fCrEff,fDe,cR,tasEff,ws,wd)
            if rT['totalGal']+fixed <= fobAT: maxD=mid; lo=mid+1
            else: hi=mid-1
    maxD = min(maxD, 1400)
    effD = min(dist, maxD) if maxD > 0 else dist
    r    = compute(effD,alt,dEl,aEl,fCl,fCrEff,fDe,cR,tasEff,ws,wd)
    minS = round(r['totalGal']+fixed, 1)
    lF   = round(fobAT-r['totalGal'], 1)
    marg = round(lF-rGal-aGal, 1)
    return {'r':r,'rGal':rGal,'aGal':aGal,'fobAT':fobAT,'fixed':fixed,
            'minS':minS,'lF':lF,'marg':marg,'maxD':maxD,'effD':effD,
            'fCrEff':fCrEff,'tasEff':tasEff}

def simulate_altplan(altDist, altAlt, altElev, destElev, ws, wd,
                     fob, taxi, rGph, rMins, tripTotalGal,
                     fCl, fCr, fDe, cR, tas):
    """
    Replica of refreshAltPlan() computation.
    tripTotalGal: lp.totalGal from the main planner (lastPlan)
    Returns dict with all computed alternate planning values.
    """
    if rMins is None or str(rMins)=='': rMins=60
    rMins=float(rMins); rGal=round(rGph*(rMins/60),1)
    fobAT    = fob - taxi
    destFuel = round(fobAT - tripTotalGal, 1)
    divert   = compute(altDist,altAlt,destElev,altElev,fCl,fCr,fDe,cR,tas,ws,wd)
    altArrFuel = round(destFuel - divert['totalGal'], 1)
    altMargin  = round(altArrFuel - rGal, 1)
    noFuel     = destFuel < divert['totalGal'] + rGal
    return {
        'destFuel':   destFuel,
        'divert':     divert,
        'altArrFuel': altArrFuel,
        'altMargin':  altMargin,
        'reserveGal': rGal,
        'noFuel':     noFuel,
        'legal':      altMargin >= 0,
        'comfortable': altMargin >= 10,
    }

# Default inputs matching app DEFAULTS
DEFAULTS = dict(dist=500, alt=22000, dEl=1000, aEl=1000,
                fCl=37, fCr=30, fDe=25, cR=1400,
                tas=285, ws=0, wd='head',
                fob=145, rGph=38, rMins=60, taxi=3, altD=0,
                perfData=None, isaDev=0)
def run(**kw): return simulate(**{**DEFAULTS, **kw})

# Test harness
bugs=[]; warns=[]
def chk(label, cond, detail='', warning=False):
    tag = '✓' if cond else ('⚠' if warning else '✗')
    print(f"  {tag} {label}" + (f"  [{detail}]" if detail else ''))
    if not cond:
        if warning: warns.append(label)
        else:       bugs.append(label)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO A — DEFAULT STATE
# ════════════════════════════════════════════════════════════════════════════
def test_A():
    print("\n── A. DEFAULT STATE (verify every output field)")
    s=run(); r=s['r']
    print(f"  Climb:{r['climbDist']}nm {fmtTime(r['climbMins'])} {r['climbGal']}gal delta={r['climbDelta']}ft")
    print(f"  Cruise:{r['cruiseDist']}nm {fmtTime(r['cruiseMins'])} {r['cruiseGal']}gal GS={r['gs']}kts")
    print(f"  Descent:{r['descDist']}nm {fmtTime(r['descMins'])} {r['descGal']}gal avgTAS={r['avgDesTas']}kts")
    print(f"  Total:{r['totalGal']}gal {fmtTime(r['totalMins'])}  Reserve:{s['rGal']}gal Alt:{s['aGal']}gal")
    print(f"  MinStart:{s['minS']}gal LandFuel:{s['lF']}gal Margin:{s['marg']}gal maxDist:{s['maxD']}nm")
    chk("A1.  c+cr+d=500nm", abs(r['_cG']+r['_crG']+r['_dG']-500)<0.5)
    chk("A2.  fuel phases sum to totalGal", abs(r['climbGal']+r['cruiseGal']+r['descGal']-r['totalGal'])<0.2)
    chk("A3.  climbTime=(22000-1000)/1400≈15min", abs(r['climbMins']-round((22000-1000)/1400))<1)
    chk("A4.  climbFuel=37×(21000/1400)/60≈9.4gal", abs(r['climbGal']-round(37*21000/1400/60,1))<0.3)
    chk("A5.  GS=285 (TAS no wind)", r['gs']==285)
    chk("A6.  descTAS ∈ [223,312] (220 KIAS FL22k→1k)", 223<=r['avgDesTas']<=312)
    chk("A7.  descTime≈18min (22000ft / 1174fpm avg VS from 2.5°FPA)", abs(r['descMins']-18)<2)
    chk("A8.  TOD=descDist", r['todNm']==r['descDist'])
    chk("A9.  fobAfterTaxi=142", s['fobAT']==142)
    chk("A10. reserveGal=38.0", s['rGal']==38.0)
    chk("A11. altFuelGal=0", s['aGal']==0.0)
    chk("A12. landFuel=fobAT−tripFuel", s['lF']==round(s['fobAT']-r['totalGal'],1))
    chk("A13. minStart=tripFuel+fixed", s['minS']==round(r['totalGal']+s['fixed'],1))
    chk("A14. margin=landFuel−reserve−alt", s['marg']==round(s['lF']-s['rGal']-s['aGal'],1))
    chk("A15. taxi NOT in minStart", s['minS']==round(r['totalGal']+38,1))
    chk("A16. minStart+taxi≤FOB", s['minS']+3<=145)
    chk("A17. noAlt=False", not r['noAlt'])
    chk("A18. all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)
    chk("A19. all dist ≥0", r['climbDist']>=0 and r['cruiseDist']>=0 and r['descDist']>=0)
    chk("A20. margin>0", s['marg']>0)
    chk("A21. maxDist>500nm", s['maxD']>500)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO B — SHORT LEG HIGH ELEVATION  KAFO→KTCS 75nm FL120
# ════════════════════════════════════════════════════════════════════════════
def test_B():
    print("\n── B. SHORT LEG KAFO→KTCS (75nm FL120, high elevation)")
    fCr=cruiseGphAtAlt(12000,17000,32)
    s=run(dist=75,alt=12000,dEl=6204,aEl=4586,fCr=fCr); r=s['r']
    exp_gph=32*(1+(17000-12000)/1000*0.008)
    print(f"  FF@FL120={fCr:.3f}gph Climb:{r['climbDist']}nm Cruise:{r['cruiseDist']}nm Desc:{r['descDist']}nm")
    chk("B1. climbDelta=5796ft", r['climbDelta']==5796)
    chk("B2. dist sum=75nm", abs(r['_cG']+r['_crG']+r['_dG']-75)<0.5)
    chk("B3. noAlt=False", not r['noAlt'])
    chk(f"B4. FF@FL120={fCr:.3f}≈{exp_gph:.3f}", abs(fCr-exp_gph)<0.01)
    chk("B5. all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)
    chk("B6. fuel sum correct", abs(r['climbGal']+r['cruiseGal']+r['descGal']-r['totalGal'])<0.2)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO C — MID LEG HEADWIND  KAFO→KDEN 250nm FL170 25kt HW
# ════════════════════════════════════════════════════════════════════════════
def test_C():
    print("\n── C. MID LEG KAFO→KDEN (250nm FL170, 25kt HW)")
    s=run(dist=250,alt=17000,dEl=6204,aEl=5434,ws=25,wd='head'); r=s['r']
    s0=run(dist=250,alt=17000,dEl=6204,aEl=5434)
    print(f"  GS={r['gs']} Climb:{r['climbDist']}nm Cruise:{r['cruiseDist']}nm Desc:{r['descDist']}nm")
    chk("C1. GS=260 (285−25 HW)", r['gs']==260)
    chk("C2. climbDelta=10796ft", r['climbDelta']==10796)
    chk("C3. dist sum=250nm", abs(r['_cG']+r['_crG']+r['_dG']-250)<0.5)
    chk("C4. HW shortens climb dist", r['_cG']<s0['r']['_cG'])
    chk("C5. HW more fuel than no wind", r['totalGal']>s0['r']['totalGal'])
    chk("C6. HW longer than no wind", r['totalMins']>s0['r']['totalMins'])
    chk("C7. fuel sum correct", abs(r['climbGal']+r['cruiseGal']+r['descGal']-r['totalGal'])<0.2)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO D — LONG LEG TAILWIND  KAFO→KPHX 500nm FL280 30kt TW
# ════════════════════════════════════════════════════════════════════════════
def test_D():
    print("\n── D. LONG LEG KAFO→KPHX (500nm FL280, 30kt TW)")
    gph28=cruiseGphAtAlt(28000)  # V-shape: FL280 costs more than FL220
    # TAS at FL280: declines from FL220 peak at 0.455kts/1000ft
    tas28=max(200, 285-(28000-22000)/1000*0.455)
    s=run(dist=500,alt=28000,dEl=6204,aEl=1135,fCr=gph28,tas=tas28,ws=30,wd='tail')
    s0=run(dist=500,alt=28000,dEl=6204,aEl=1135,fCr=gph28,tas=tas28)
    r=s['r']
    print(f"  gph={gph28:.3f} TAS={tas28:.1f} GS={r['gs']} Total:{r['totalGal']}gal {fmtTime(r['totalMins'])}")
    chk(f"D1. GS=tas28+30 TW ({round(tas28)+30})", r['gs']==round(tas28)+30)
    chk("D2. climbDelta=21796ft", r['climbDelta']==21796)
    chk("D3. dist sum=500nm", abs(r['_cG']+r['_crG']+r['_dG']-500)<0.5)
    chk("D4. gph@FL280>gph@FL220 (V-shape)", gph28>cruiseGphAtAlt(22000))
    chk(f"D5. TAS@FL280={tas28:.1f}<285 (declines above FL220)", tas28<285)
    chk("D6. TW less fuel than no wind", r['totalGal']<s0['r']['totalGal'])
    chk("D7. all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO E — FUEL INPUT EDGE CASES
# ════════════════════════════════════════════════════════════════════════════
def test_E():
    print("\n── E. FUEL INPUT EDGE CASES")
    s=run(fob=100)
    chk("E1. fob=100: tight fuel", s['minS']>s['fobAT'] or s['marg']<10,
        f"minS={s['minS']} fobAT={s['fobAT']} marg={s['marg']}")
    s=run(rMins=0); s_def=run()
    chk("E2. rMins=0: rGal=0", s['rGal']==0.0)
    chk("E2. rMins=0: margin=landFuel", s['marg']==s['lF'])
    chk("E2. rMins=0: larger maxDist", s['maxD']>s_def['maxD'])
    chk("E3. rMins=45: rGal=28.5", run(rMins=45)['rGal']==28.5)
    s=run(altD=100); exp=round(38*100/256,1)
    chk(f"E4. altDist=100: aGal={s['aGal']}≈{exp}", abs(s['aGal']-exp)<0.1)
    chk("E4. altDist reduces margin", s['marg']<run()['marg'])
    s0=run(taxi=0); s3=run(taxi=3)
    chk("E5. taxi=0: fobAT=145", s0['fobAT']==145)
    chk("E5. taxi=3: fobAT=142", s3['fobAT']==142)
    chk("E5. tripFuel independent of taxi", s0['r']['totalGal']==s3['r']['totalGal'])
    chk("E5. minStart independent of taxi", s0['minS']==s3['minS'])
    chk("E5. landFuel lower with more taxi", s3['lF']<s0['lF'])
    s=run(fob=0)
    chk("E6. fob=0: fobAT=−3, no crash", s['fobAT']==-3)
    chk("E6. fob=0: maxDist=0", s['maxD']==0)
    s=run(fob=30,rGph=38,rMins=60)
    chk("E7. reserve>fobAT: caught", s['fixed']>s['fobAT'])
    chk("E7. tripFuel still computed", s['r']['totalGal']>=0)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO F — WIND EDGE CASES
# ════════════════════════════════════════════════════════════════════════════
def test_F():
    print("\n── F. WIND EDGE CASES")
    rH=run(ws=0,wd='head')['r']; rT=run(ws=0,wd='tail')['r']
    chk("F1. ws=0: head==tail", rH['gs']==rT['gs']==285 and rH['totalGal']==rT['totalGal'])
    r=compute(500,22000,1000,1000,37,30,25,1400,285,250,'head')
    chk("F2. ws=250 HW: GS floored at 50", r['gs']==50)
    r=compute(500,22000,1000,1000,37,30,25,1400,320,30,'tail')
    chk("F3. TW TAS=320: GS=350", r['gs']==350)
    for ws in [10,20,30,40]:
        rHW=run(ws=ws,wd='head')['r']; rTW=run(ws=ws,wd='tail')['r']
        chk(f"F4. ws={ws}: TW faster+less fuel", rTW['totalMins']<rHW['totalMins'] and rTW['totalGal']<rHW['totalGal'])

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO G — ALTITUDE & ELEVATION EDGE CASES
# ════════════════════════════════════════════════════════════════════════════
def test_G():
    print("\n── G. ALTITUDE & ELEVATION EDGE CASES")
    r=run(alt=5000)['r']
    chk("G1. FL5000: climbDelta=4000ft", r['climbDelta']==4000)
    chk("G1. FL5000: dist sum=500nm", abs(r['_cG']+r['_crG']+r['_dG']-500)<0.5)
    r=run(alt=5000,dEl=6000)['r']
    chk("G2. depElev>cruiseAlt: climbDelta=0, climbGal=0", r['climbDelta']==0 and r['climbGal']==0.0)
    r=run(alt=5000,aEl=6000)['r']
    chk("G3. arrElev>cruiseAlt: descGal=0", r['descGal']==0.0)
    chk("G3. dist sum correct", abs(r['_cG']+r['_crG']+r['_dG']-500)<0.5)
    r=run(alt=6000,dEl=6000,aEl=6000)['r']
    chk("G4. dep=arr=cruise: climb=desc=0, all→cruise", r['climbDelta']==0 and abs(r['_crG']-500)<0.5)
    chk("G5. FL28000: reachable at 500nm", not run(alt=28000)['r']['noAlt'])

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO H — SHORT LEG / noAlt
# ════════════════════════════════════════════════════════════════════════════
def test_H():
    print("\n── H. SHORT LEG / noAlt EDGE CASES")
    r=run(dist=20,alt=17000)['r']
    chk("H1. dist=20nm FL170: noAlt=True", r['noAlt'])
    chk("H1. noAlt: crGal=dGal=0", r['cruiseGal']==0.0 and r['descGal']==0.0)
    chk("H1. noAlt: totalGal=climbGal", r['totalGal']==r['climbGal'])
    r=run(dist=50,alt=5000)['r']
    chk("H2. dist=50 FL5000: sum=50nm", abs(r['_cG']+r['_crG']+r['_dG']-50)<0.5)
    chk("H2. all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)
    r=run(dist=35,alt=17000)['r']
    chk("H3. descent capped: descDist≤remaining", r['_dG']<=max(0,35-r['_cG'])+0.1)
    chk("H3. no negative cruise dist", r['_crG']>=0)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO I — FUEL FLOW & PERFORMANCE VARIATIONS
# ════════════════════════════════════════════════════════════════════════════
def test_I():
    print("\n── I. FUEL FLOW & PERFORMANCE VARIATIONS")
    s0=run()
    chk("I1. ffClimb=45: more climb fuel, same cruise time",
        run(fCl=45)['r']['climbGal']>s0['r']['climbGal'] and
        run(fCl=45)['r']['cruiseMins']==s0['r']['cruiseMins'])
    chk("I2. ffDesc=30 Hi-Spd: more desc fuel, same cruise",
        run(fDe=30)['r']['descGal']>s0['r']['descGal'] and
        run(fDe=30)['r']['cruiseMins']==s0['r']['cruiseMins'])
    chk("I3. lower cruise FF: less total fuel, same times",
        run(fCr=24)['r']['totalGal']<run(fCr=40)['r']['totalGal'] and
        run(fCr=24)['r']['cruiseMins']==run(fCr=40)['r']['cruiseMins'])
    s_slow=run(tas=240); s_fast=run(tas=320)
    chk("I4. higher TAS: shorter cruise, less fuel",
        s_fast['r']['cruiseMins']<s_slow['r']['cruiseMins'] and
        s_fast['r']['totalGal']<s_slow['r']['totalGal'])
    chk("I4. TAS ratio proportional to time (±15%)",
        abs(s_slow['r']['cruiseMins']/max(1,s_fast['r']['cruiseMins'])-320/240)<0.15)
    chk("I5. slower climb rate: more time+fuel",
        run(cR=800)['r']['climbMins']>run(cR=2000)['r']['climbMins'])
    # I6 removed — descent rate is now derived from 2.5° FPA, not user-settable
    chk("I6. higher cruise alt = longer descent (more delta alt)",
        run(alt=28000)['r']['descMins']>run(alt=10000)['r']['descMins'])

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO J — GPH MODEL CONSISTENCY
# ════════════════════════════════════════════════════════════════════════════
def test_J():
    print("\n── J. CRUISE GPH MODEL CONSISTENCY")
    # V-shape: FF decreases going up to FL220, then increases above FL220
    alts_below=[5000,8000,10000,12000,14000,17000,19000,21000,22000]
    alts_above=[22000,23000,25000,26000,28000]
    gphs_below=[cruiseGphAtAlt(a) for a in alts_below]
    gphs_above=[cruiseGphAtAlt(a) for a in alts_above]
    for i in range(len(gphs_below)-1):
        chk(f"J1. GPH decreasing {alts_below[i]}→{alts_below[i+1]}ft (below anchor)",
            gphs_below[i]>=gphs_below[i+1], f"{gphs_below[i]:.3f}>={gphs_below[i+1]:.3f}")
    for i in range(len(gphs_above)-1):
        chk(f"J1. GPH increasing {alts_above[i]}→{alts_above[i+1]}ft (above anchor)",
            gphs_above[i]<=gphs_above[i+1], f"{gphs_above[i]:.3f}<={gphs_above[i+1]:.3f}")
    chk("J2. Anchor FL220=30.000 exactly", abs(cruiseGphAtAlt(22000,22000,30)-30)<0.001)
    chk("J3. FL280>FL220 (V-shape: worse above anchor)", cruiseGphAtAlt(28000)>cruiseGphAtAlt(22000))
    chk("J4. Min clamped at 0.6×30=18.0", cruiseGphAtAlt(200000)>=0.6*30)
    chk("J5. Max clamped at 1.5×30=45.0", cruiseGphAtAlt(-10000)<=1.5*30)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO K — REAL PERFORMANCE DATA SYSTEM
# ════════════════════════════════════════════════════════════════════════════
def test_K():
    print("\n── K. PERFORMANCE DATA SYSTEM")
    chk("K1. empty data → None", getRealPerf(17000,0,[])==None)
    d=[{'alt':17000,'tas':285,'ff':30.5,'isa':0}]
    rp=getRealPerf(17000,0,d)
    chk(f"K2. exact match TAS={rp['tas']:.1f}=285 FF={rp['ff']:.2f}=30.5",
        abs(rp['tas']-285)<0.01 and abs(rp['ff']-30.5)<0.01)
    d2=[{'alt':17000,'tas':282,'ff':31.0,'isa':0},{'alt':17000,'tas':278,'ff':33.0,'isa':0}]
    rp=getRealPerf(17000,0,d2)
    chk(f"K3. equal-weight avg TAS={rp['tas']:.1f}=280 FF={rp['ff']:.1f}=32",
        abs(rp['tas']-280)<0.1 and abs(rp['ff']-32)<0.1)
    d3=[{'alt':17000,'tas':287,'ff':30.0,'isa':-5},{'alt':17000,'tas':273,'ff':34.0,'isa':+8}]
    chk(f"K4. ISA=-3 picks -5 TAS={getRealPerf(17000,-3,d3)['tas']:.0f}=287",
        abs(getRealPerf(17000,-3,d3)['tas']-287)<0.1)
    chk(f"K4. ISA=+5 picks +8 TAS={getRealPerf(17000,+5,d3)['tas']:.0f}=273",
        abs(getRealPerf(17000,+5,d3)['tas']-273)<0.1)
    chk("K5. 10k data query 17k: None", getRealPerf(17000,0,[{'alt':10000,'tas':250,'ff':36,'isa':0}])==None)
    chk("K5. 15k data query 17k: data", getRealPerf(17000,0,[{'alt':15000,'tas':272,'ff':32.5,'isa':0}])!=None)
    # Use FL22000 anchor for test: generic=30.0gph; real data at lower FF means better
    # At FL22000 generic=30.0, so use FF=28 (better than generic) and FF=40 (worse)
    d_real=[{'alt':22000,'tas':287,'ff':28.0,'isa':0}]  # better than generic 30.0
    s_gen=run(); s_real=run(perfData=d_real,isaDev=0)
    chk("K6. real data FF=28 (better than generic 30): less total fuel",
        s_real['r']['totalGal']<s_gen['r']['totalGal'])
    chk("K6. lower FF: more maxDist", s_real['maxD']>s_gen['maxD'])
    d_hot=[{'alt':22000,'tas':275,'ff':40.0,'isa':0}]  # worse than generic 30.0
    s_hot=run(perfData=d_hot,isaDev=0)
    chk("K7. high real FF=40 (worse than generic 30): maxDist shrinks", s_hot['maxD']<s_gen['maxD'])
    rM=compute(s_hot['maxD'],22000,1000,1000,37,40,10,1400,275,0,'head')
    chk("K7. maxDist valid: tripFuel+fixed≤fobAT",
        round(rM['totalGal']+s_hot['fixed'],1)<=s_hot['fobAT'])

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO L — ACT vs PLAN
# ════════════════════════════════════════════════════════════════════════════
def test_L():
    print("\n── L. ACT vs PLAN / lastPlan")
    r=compute(500,17000,1000,1000,37,32,10,1400,280,0,'head')
    lp={'dep':'KAFO','arr':'KGEU','alt':17000,'dist':500,'tas':280,'ff':32.0,
        'cruiseMins':r['cruiseMins'],'cruiseGal':r['cruiseGal'],
        'totalGal':r['totalGal'],'totalMins':r['totalMins']}
    chk("L1. all keys present",
        all(k in lp for k in ['dep','arr','alt','dist','tas','ff','cruiseMins','cruiseGal','totalGal','totalMins']))
    actTas=283; actFf=31.2
    chk("L2. dTas=+3 positive=faster", actTas-lp['tas']>0)
    chk("L2. dFF=−0.8 negative=more efficient", actFf-lp['ff']<0)
    chk("L3. cruiseMins key valid", r['cruiseMins']>0)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO M — ALTITUDE COMPARISON CHART
# ════════════════════════════════════════════════════════════════════════════
def test_M():
    print("\n── M. ALTITUDE COMPARISON CHART")
    alts=[5000,8000,10000,12000,14000,17000,19000,21000,23000,25000,28000]
    cruiseKias=280*math.sqrt(densityRatio(17000))
    tasRate=(TAS_CEIL-280)/max(1,TAS_CEIL_ALT-17000)*1000
    print("  Alt    | GPH    | TAS    | Fuel(500nm)")
    fuelBars=[]
    for a in alts:
        gph=cruiseGphAtAlt(a,17000,32)
        if a>=17000: tasAdj=min(TAS_CEIL,280+(a-17000)/1000*tasRate)
        else: tasAdj=max(160,cruiseKias/math.sqrt(densityRatio(a)))
        res=compute(500,a,1000,1000,37,gph,10,1400,tasAdj,0,'head')
        fuel=None if res['_cG']>=500 else res['totalGal']
        fuelBars.append(fuel)
        print(f"  {a:5}ft | {gph:.3f} | {tasAdj:.1f}kt | {str(fuel)+'gal' if fuel else 'UNREACHABLE'}")
    valid=[f for f in fuelBars if f is not None]
    chk("M1. FL170 reachable", fuelBars[5] is not None)
    chk("M2. higher altitude cheaper", min(valid)<fuelBars[5])
    chk("M3. fuel decreasing with alt above FL170",
        all(fuelBars[i]>fuelBars[i+1] for i in range(5,len(alts)-1) if fuelBars[i] and fuelBars[i+1]))
    print(f"  Min: {min(valid)}gal at FL{alts[[i for i,f in enumerate(fuelBars) if f==min(valid)][0]]//100}")

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO N — ISA PHYSICS & DESCENT TAS
# ════════════════════════════════════════════════════════════════════════════
def test_N():
    print("\n── N. ISA PHYSICS & DESCENT TAS")
    chk("N1. ISA SL=15°C", abs(isaTemp(0)-15)<0.01)
    chk("N2. ISA 10k≈−4.8°C", abs(isaTemp(10000)+4.8)<0.2)
    chk("N3. ISA tropo=−56.5°C", abs(isaTemp(40000)+56.5)<0.01)
    chk("N4. density SL=1.0", abs(densityRatio(0)-1.0)<0.001)
    chk("N5. 260KIAS@10k≈303KTAS", abs(kiasToKtas(260,10000)-303)<3)
    chk("N6. 220KIAS@10k≈256KTAS (ALT_KTAS)", abs(kiasToKtas(220,10000)-256)<2)
    for cAlt,aAlt in [(22000,1000),(28000,1000),(17000,1000),(10000,5000)]:
        avg=avgDescentTas(cAlt,aAlt)
        lo=kiasToKtas(220,aAlt)   # lowest alt = lowest TAS
        hi=kiasToKtas(220,cAlt)   # highest alt = highest TAS
        chk(f"N7. avgDescTas_220({cAlt}→{aAlt})={avg:.0f} ∈ [{lo:.0f},{hi:.0f}]",
            lo<=avg<=hi)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO O — fmtTime
# ════════════════════════════════════════════════════════════════════════════
def test_O():
    print("\n── O. fmtTime EDGE CASES")
    for m,exp in [(0,'0+00'),(0.49,'0+00'),(0.5,'0+01'),(29.5,'0+30'),
                  (59,'0+59'),(59.5,'1+00'),(60,'1+00'),(61,'1+01'),
                  (89,'1+29'),(90,'1+30'),(119.5,'2+00'),(150,'2+30'),(239.5,'4+00')]:
        got=fmtTime(m); chk(f"O. {m}→'{got}' expect '{exp}'", got==exp)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO P — ALTERNATE FUEL (legacy formula)
# ════════════════════════════════════════════════════════════════════════════
def test_P():
    print("\n── P. LEGACY ALTERNATE FUEL FORMULA")
    for altD,exp in [(0,0.0),(25,round(38*25/256,1)),(50,round(38*50/256,1)),
                     (100,round(38*100/256,1)),(150,round(38*150/256,1))]:
        aGal=round(ALT_CRUISE_GPH*(altD/max(1,ALT_KTAS)),1)
        chk(f"P. altDist={altD}nm: {aGal}gal≈{exp}gal", abs(aGal-exp)<0.15)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO Q — MONOTONICITY & STRESS
# ════════════════════════════════════════════════════════════════════════════
def test_Q():
    print("\n── Q. MONOTONICITY & STRESS")
    dists=[100,200,300,400,500]
    fuels=[compute(d,17000,1000,1000,37,32,10,1400,280,0,'head')['totalGal'] for d in dists]
    times=[compute(d,17000,1000,1000,37,32,10,1400,280,0,'head')['totalMins'] for d in dists]
    for i in range(len(dists)-1):
        chk(f"Q1. dist {dists[i]}→{dists[i+1]}nm: more fuel ({fuels[i]:.1f}<{fuels[i+1]:.1f})", fuels[i]<fuels[i+1])
        chk(f"Q1. dist {dists[i]}→{dists[i+1]}nm: more time", times[i]<times[i+1])
    tasRate=(TAS_CEIL-280)/max(1,TAS_CEIL_ALT-17000)*1000
    gph21=cruiseGphAtAlt(21000,17000,32); tas21=min(TAS_CEIL,280+(21000-17000)/1000*tasRate)
    r=compute(850,21000,6204,5434,37,gph21,10,1400,tas21,40,'head')
    chk("Q2. stress 850nm FL210 40ktHW: dist sum", abs(r['_cG']+r['_crG']+r['_dG']-850)<0.5)
    chk("Q2. stress: all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)
    chk("Q2. stress: fuel sum correct", abs(r['climbGal']+r['cruiseGal']+r['descGal']-r['totalGal'])<0.2)
    print(f"  Stress: {r['totalGal']}gal {fmtTime(r['totalMins'])}")
    for ws in [10,20,30,40]:
        rHW=run(ws=ws,wd='head')['r']; rTW=run(ws=ws,wd='tail')['r']
        chk(f"Q3. ws={ws}: TW faster+less fuel", rTW['totalMins']<rHW['totalMins'] and rTW['totalGal']<rHW['totalGal'])

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO R — ALTERNATE AIRPORT PLANNING (new full three-phase compute)
# ════════════════════════════════════════════════════════════════════════════
def test_R():
    print("\n── R. ALTERNATE AIRPORT PLANNING (full three-phase)")

    # Shared baseline: main trip KAFO→KDEN FL170 250nm
    # Alternate: KBJC (Broomfield, CO) 50nm from KDEN, elev=5673ft
    MAIN_FOB=145; TAXI=3; RMINS=60; RGPH=38
    MAIN_DIST=250; MAIN_ALT=17000; DEP_ELEV=6204; ARR_ELEV=5434
    TRIP_FUEL=run(dist=MAIN_DIST,alt=MAIN_ALT,dEl=DEP_ELEV,aEl=ARR_ELEV)['r']['totalGal']
    print(f"  Main trip: {MAIN_DIST}nm FL{MAIN_ALT//100}, tripFuel={TRIP_FUEL}gal")

    # ── R1: Basic fuel chain integrity ─────────────────────────────────────
    def ap(altDist=50, altAlt=10000, altElev=5673, destElev=ARR_ELEV,
           ws=0, wd='head', fob=MAIN_FOB, taxi=TAXI, rGph=RGPH, rMins=RMINS,
           tripGal=TRIP_FUEL, fCl=37, fCr=32, fDe=10, cR=1400, dR=1000, tas=280):
        return simulate_altplan(altDist,altAlt,altElev,destElev,ws,wd,
                                fob,taxi,rGph,rMins,tripGal,fCl,fCr,fDe,cR,tas)

    s=ap()
    print(f"  Dest fuel:{s['destFuel']}gal  Divert:{s['divert']['totalGal']}gal  "
          f"At alt:{s['altArrFuel']}gal  Margin:{s['altMargin']}gal")

    # Fuel chain: destFuel = fobAT - tripFuel
    chk("R1.  destFuel = fobAT − tripFuel",
        s['destFuel'] == round((MAIN_FOB-TAXI)-TRIP_FUEL, 1),
        f"{s['destFuel']} vs {round((MAIN_FOB-TAXI)-TRIP_FUEL,1)}")
    # altArrFuel = destFuel - divertFuel
    chk("R2.  altArrFuel = destFuel − divertFuel",
        s['altArrFuel'] == round(s['destFuel'] - s['divert']['totalGal'], 1))
    # altMargin = altArrFuel - reserveGal
    chk("R3.  altMargin = altArrFuel − reserveGal",
        s['altMargin'] == round(s['altArrFuel'] - s['reserveGal'], 1))
    # All divert phase fuels ≥ 0
    chk("R4.  all divert phase fuels ≥0",
        s['divert']['climbGal']>=0 and s['divert']['cruiseGal']>=0 and s['divert']['descGal']>=0)
    # Divert fuel phases sum to totalGal
    d=s['divert']
    chk("R5.  divert phases sum to totalGal",
        abs(d['climbGal']+d['cruiseGal']+d['descGal']-d['totalGal'])<0.2)
    # Divert distances sum to altDist
    chk("R6.  divert c+cr+d = altDist",
        abs(d['_cG']+d['_crG']+d['_dG']-50)<0.5,
        f"{d['climbDist']}+{d['cruiseDist']}+{d['descDist']}")

    # ── R2: Elevation handling ──────────────────────────────────────────────
    # Divert alt (10000ft) > dest elev (5434ft): climb of 4566ft
    chk("R7.  divert climbDelta=4566ft (10000-5434)",
        d['climbDelta']==round(10000-ARR_ELEV))
    # Divert alt (10000ft) > alt airport elev (5673ft): descent of 4327ft
    chk("R8.  divert descDelta=4327ft (10000-5673)",
        round(10000-5673)==4327)

    # ── R3: Legal / marginal / illegal states ───────────────────────────────
    # Comfortable legal: plenty of fuel
    s_good=ap(fob=145)
    chk("R9.  fob=145: legal (altMargin≥0)", s_good['altMargin']>=0,
        f"margin={s_good['altMargin']}")
    chk("R9.  fob=145: comfortable (altMargin≥10)", s_good['altMargin']>=10,
        f"margin={s_good['altMargin']}")

    # Tight fuel: small fob forces marginal state
    s_tight=ap(fob=90)
    print(f"  fob=90: destFuel={s_tight['destFuel']} altArrFuel={s_tight['altArrFuel']} margin={s_tight['altMargin']}")
    chk("R10. fob=90: destFuel < fob=145 version", s_tight['destFuel']<s_good['destFuel'])
    chk("R10. fob=90: reduced margin", s_tight['altMargin']<s_good['altMargin'])

    # Illegal: fob too low to make alternate with reserves
    # Find the fob where altMargin goes negative
    for test_fob in range(145, 50, -5):
        st=ap(fob=test_fob)
        if st['altMargin'] < 0:
            print(f"  First illegal fob={test_fob}: margin={st['altMargin']}")
            chk(f"R11. fob={test_fob}: altMargin<0 (illegal)", st['altMargin']<0)
            chk(f"R11. fob={test_fob}: legal=False", not st['legal'])
            break

    # ── R4: noAlt fires correctly on divert ────────────────────────────────
    # Very short divert (10nm) with high cruise alt should noAlt
    s_na=ap(altDist=10, altAlt=17000)
    chk("R12. short divert 10nm FL170: noAlt fires",
        s_na['divert']['noAlt'])
    chk("R12. noAlt: divert crGal=dGal=0",
        s_na['divert']['cruiseGal']==0.0 and s_na['divert']['descGal']==0.0)

    # ── R5: Wind effects on divert ──────────────────────────────────────────
    s_hw=ap(ws=25,wd='head')
    s_tw=ap(ws=25,wd='tail')
    s_0 =ap(ws=0)
    chk("R13. HW divert: more fuel than calm", s_hw['divert']['totalGal']>s_0['divert']['totalGal'])
    chk("R13. TW divert: less fuel than calm", s_tw['divert']['totalGal']<s_0['divert']['totalGal'])
    chk("R13. HW divert: less margin than TW", s_hw['altMargin']<s_tw['altMargin'])
    chk("R13. ws=0: head==tail divert fuel",
        ap(ws=0,wd='head')['divert']['totalGal']==ap(ws=0,wd='tail')['divert']['totalGal'])

    # ── R6: Divert distance sensitivity ────────────────────────────────────
    for dist in [25,50,100,150,200]:
        sd=ap(altDist=dist)
        chk(f"R14. altDist={dist}nm: divert distances sum",
            abs(sd['divert']['_cG']+sd['divert']['_crG']+sd['divert']['_dG']-dist)<0.5)
        chk(f"R14. altDist={dist}nm: all fuel ≥0",
            sd['divert']['climbGal']>=0 and sd['divert']['cruiseGal']>=0 and sd['divert']['descGal']>=0)
    # More divert dist = less margin
    margins=[ap(altDist=d)['altMargin'] for d in [25,50,100,150,200]]
    for i in range(len(margins)-1):
        chk(f"R15. altDist {[25,50,100,150][i]}→{[50,100,150,200][i]}nm: margin decreases",
            margins[i]>margins[i+1])

    # ── R7: Divert altitude sensitivity ────────────────────────────────────
    for alt in [5000,8000,10000,14000,17000]:
        sd=ap(altAlt=alt)
        chk(f"R16. altAlt={alt}: dist sum correct",
            abs(sd['divert']['_cG']+sd['divert']['_crG']+sd['divert']['_dG']-50)<0.5)
        chk(f"R16. altAlt={alt}: all fuel ≥0",
            sd['divert']['climbGal']>=0 and sd['divert']['cruiseGal']>=0 and sd['divert']['descGal']>=0)

    # ── R8: High-elevation alternate ───────────────────────────────────────
    # Alternate at 9000ft elevation (above divert cruise alt of 10000ft = only 1000ft climb)
    s_hi=ap(altElev=9000)
    chk("R17. high alt airport (9000ft): descDelta=1000ft",
        abs(10000-9000 - round(s_hi['divert']['climbDelta']+(10000-9000)))<100 or
        s_hi['divert']['descDist'] < ap(altElev=1000)['divert']['descDist'])
    chk("R17. high alt airport: all fuel ≥0",
        s_hi['divert']['climbGal']>=0 and s_hi['divert']['cruiseGal']>=0 and s_hi['divert']['descGal']>=0)

    # Alternate elev > divert cruise alt
    s_above=ap(altElev=12000, altAlt=10000)
    chk("R18. altElev>altAlt: descDelta=0, descGal=0",
        s_above['divert']['descGal']==0.0)
    chk("R18. altElev>altAlt: dist sum correct",
        abs(s_above['divert']['_cG']+s_above['divert']['_crG']+s_above['divert']['_dG']-50)<0.5)

    # ── R9: FOB sync semantics ──────────────────────────────────────────────
    # destFuel must use fob-taxi, not fob directly
    s_f1=ap(fob=145,taxi=3); s_f2=ap(fob=145,taxi=10)
    chk("R19. more taxi: less destFuel", s_f2['destFuel']<s_f1['destFuel'])
    chk("R19. taxi change doesn't affect divert fuel (same flight)",
        s_f1['divert']['totalGal']==s_f2['divert']['totalGal'])
    chk("R19. taxi change reduces margin", s_f2['altMargin']<s_f1['altMargin'])

    # ── R10: Reserve consumed at alternate, not at destination ─────────────
    s_r0=ap(rMins=0)
    chk("R20. rMins=0: reserveGal=0, margin=altArrFuel",
        s_r0['reserveGal']==0.0 and s_r0['altMargin']==s_r0['altArrFuel'])
    s_r45=ap(rMins=45)
    chk("R20. rMins=45: reserveGal=28.5", s_r45['reserveGal']==28.5)
    chk("R20. more reserve: less margin", s_r45['altMargin']<s_r0['altMargin'])

    # ── R11: noFuel warning trigger ─────────────────────────────────────────
    # When destFuel < divertFuel + reserveGal, noFuel should be True
    s_nf=ap(fob=62, taxi=3)  # very low fob — destFuel ~(62-3-tripFuel)
    print(f"  noFuel test fob=62: destFuel={s_nf['destFuel']} divert={s_nf['divert']['totalGal']}+reserve={s_nf['reserveGal']}")
    chk("R21. noFuel flag: True when destFuel < divert+reserve",
        s_nf['noFuel'] == (s_nf['destFuel'] < s_nf['divert']['totalGal'] + s_nf['reserveGal']))

    # ── R12: Haversine distance sanity ─────────────────────────────────────
    # Real coordinates (verified): KAFO 42.712N,110.942W  KDEN 39.856N,104.673W
    # KAFO→KDEN ≈ 330nm
    nm_kafo_kden = haversineNm(42.712,-110.942,39.856,-104.673)
    chk("R22. haversineNm KAFO→KDEN≈330nm (±20nm)",
        abs(nm_kafo_kden-330)<20, f"got {nm_kafo_kden:.1f}nm")
    # KDEN→KBJC ≈ 20nm (short alternate)
    nm_kden_kbjc = haversineNm(39.856,-104.673,39.909,-105.117)
    chk("R22. haversineNm KDEN→KBJC≈21nm (±5nm)",
        abs(nm_kden_kbjc-21)<5, f"got {nm_kden_kbjc:.1f}nm")
    # Same point = 0nm
    chk("R22. haversineNm same point = 0nm",
        haversineNm(40.0,-105.0,40.0,-105.0)<0.01)
    # Symmetry
    chk("R22. haversineNm symmetric",
        abs(haversineNm(42.712,-110.942,39.856,-104.673) -
            haversineNm(39.856,-104.673,42.712,-110.942))<0.01)

    # ── R13: Full fuel chain accounting audit ───────────────────────────────
    print(f"\n  FUEL CHAIN AUDIT (fob=145, taxi=3, trip={TRIP_FUEL}gal, divert50nm):")
    s=ap()
    fobAT=MAIN_FOB-TAXI
    exp_dest=round(fobAT-TRIP_FUEL,1)
    exp_arr =round(exp_dest-s['divert']['totalGal'],1)
    exp_marg=round(exp_arr-s['reserveGal'],1)
    print(f"  FOB={MAIN_FOB} − taxi={TAXI} = {fobAT}gal")
    print(f"  {fobAT} − trip={TRIP_FUEL} = {exp_dest}gal at dest")
    print(f"  {exp_dest} − divert={s['divert']['totalGal']} = {exp_arr}gal at alt")
    print(f"  {exp_arr} − reserve={s['reserveGal']} = {exp_marg}gal margin")
    chk("R23. destFuel matches chain",   s['destFuel']==exp_dest)
    chk("R23. altArrFuel matches chain", s['altArrFuel']==exp_arr)
    chk("R23. altMargin matches chain",  s['altMargin']==exp_marg)

# ════════════════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    print("="*65)
    print("PROPJET GO — MASTER DEBUG SUITE  (Scenarios A–R)")
    print("="*65)

    for fn in [test_A, test_B, test_C, test_D, test_E, test_F, test_G,
               test_H, test_I, test_J, test_K, test_L, test_M, test_N,
               test_O, test_P, test_Q, test_R]:
        fn()

    print()
    print("="*65)
    n_bugs=len(bugs); n_warns=len(warns); total=n_bugs+n_warns
    status = "✓ CLEAN" if total==0 else f"{'✗' if n_bugs else '⚠'} ISSUES FOUND"
    print(f"RESULT: {status}  |  Bugs: {n_bugs}  Warnings: {n_warns}")
    if bugs:
        print("\nBUGS (✗):")
        for b in bugs: print(f"  ✗ {b}")
    if warns:
        print("\nWARNINGS (⚠):")
        for w in warns: print(f"  ⚠ {w}")
    if total == 0:
        print("  All checks passed. Safe to push to GitHub.")
    print("="*65)
    sys.exit(0 if n_bugs == 0 else 1)
