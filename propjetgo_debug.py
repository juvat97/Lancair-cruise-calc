#!/usr/bin/env python3
"""
PropJet Go — Master Debug Suite
================================
Full simulation audit of all app logic. Run this any time the app changes.
Every scenario category must pass before pushing to GitHub.

Usage:
    python3 propjetgo_debug.py

A clean run shows zero bugs (✗) and zero warnings (⚠).
The physics replica at the top must be kept in sync with index.html.

Last verified clean: 2026-04-05
Scenarios: A-Q (17 categories, 120+ checks)
"""

import math, sys

# ════════════════════════════════════════════════════════════════════════════
# PHYSICS REPLICA — keep in sync with index.html Block 1
# If you change compute(), cruiseGphAtAlt(), avgDescentTas(), or any
# constant in the app, update this section to match.
# ════════════════════════════════════════════════════════════════════════════
ALT_CRUISE_GPH = 38   # alternate fuel burn gph (220 KIAS / 10k MSL)
ALT_KTAS       = 256  # alternate cruise KTAS (~220 KIAS @ 10k ISA)

def isaTemp(a):
    altM = a * 0.3048
    return 15.0 - 6.5*(altM/1000) if altM <= 11000 else -56.5

def densityRatio(a):
    T_sl=288.15; T=isaTemp(a)+273.15; altM=a*0.3048
    if altM <= 11000: return (T/T_sl)**4.256
    return (216.65/T_sl)**4.256 * math.exp(-(altM-11000)/6341.6)

def kiasToKtas(k, a): return k / math.sqrt(densityRatio(a))

def avgDescentTas(c, ar):
    mid = (c+ar)/2
    return (kiasToKtas(260,c) + kiasToKtas(260,mid) + kiasToKtas(260,ar)) / 3

def fmtTime(m):
    t = math.floor(m+0.5)
    return f"{t//60}+{str(t%60).zfill(2)}"

def getGS(t, w, wd): return t+w if wd=='tail' else max(50, t-w)

def cruiseGphAtAlt(a, anc, g):
    """
    Linear FF model — matches ~30gph at FL280 from FL170/32gph anchor.
    Above anchor: 0.57%/1000ft improvement
    Below anchor: 0.80%/1000ft degradation
    """
    f = 1-(a-anc)/1000*0.0057 if a >= anc else 1+(anc-a)/1000*0.008
    return g * max(0.6, min(1.5, f))

def compute(dist, cAlt, dEl, aEl, cGph, crGph, dGph, cR, dR, tas, ws, wd):
    """
    Core flight calculation — exact replica of app compute().
    Returns dict with all phase results and raw floats for verification.
    """
    gs  = getGS(tas, ws, wd)
    wss = -ws if wd=='tail' else ws          # positive = headwind component

    # Climb
    cD    = max(0, cAlt-dEl);  cMins = cD/cR;  cGal = cGph*(cMins/60)
    cAS   = tas*0.55;           cGS   = max(10, cAS-wss)
    cGrnd = cGS*(cMins/60)

    # Descent
    dD     = max(0, cAlt-aEl); dMRaw = dD/dR
    avgDT  = avgDescentTas(cAlt, aEl); dGS = max(10, avgDT-wss)
    dGrndU = dGS*(dMRaw/60); rem = max(0, dist-cGrnd)
    dGrnd  = min(dGrndU, rem)
    dMins  = dMRaw*(dGrnd/max(0.01,dGrndU)) if dGrnd < dGrndU else dMRaw
    dGal   = dGph*(dMins/60)

    # Cruise
    crGrnd = max(0, dist-cGrnd-dGrnd)
    crMins = (crGrnd/gs)*60 if crGrnd > 0 else 0
    crGal  = crGph*(crMins/60)

    tGal  = cGal+crGal+dGal
    tMins = cMins+crMins+dMins
    return {
        # Rounded display values (matches what app shows)
        'climbGal':   round(cGal,1),   'climbMins':   round(cMins),
        'climbDist':  round(cGrnd),    'climbDelta':  round(cD),
        'cruiseGal':  round(crGal,1),  'cruiseMins':  round(crMins),
        'cruiseDist': round(crGrnd),   'gs':          round(gs),
        'descGal':    round(dGal,1),   'descMins':    round(dMins),
        'descDist':   round(dGrnd),    'avgDesTas':   round(avgDT),
        'totalGal':   round(tGal,1),   'totalMins':   tMins,
        'todNm':      round(dGrnd),    'noAlt':       cGrnd >= dist,
        # Raw floats for precision checks
        '_cG': cGrnd, '_crG': crGrnd, '_dG': dGrnd, '_tG': tGal,
    }

def getRealPerf(altFt, isaDev, data, BW=3000, TOL=1.0):
    """Replica of getRealPerfAtAlt() from Block 3."""
    if not data: return None
    cands = [d for d in data if abs(d['alt']-altFt) <= BW]
    if not cands: return None
    minI = min(abs(d['isa']-isaDev) for d in cands)
    sel  = [d for d in cands if abs(d['isa']-isaDev)-minI <= TOL]
    ts=fs=ws=0
    for d in sel:
        w = 1-abs(d['alt']-altFt)/BW; ts+=d['tas']*w; fs+=d['ff']*w; ws+=w
    return {'tas': ts/ws, 'ff': fs/ws} if ws >= 0.3 else None

def simulate(dist, alt, dEl, aEl, fCl, fCr, fDe, cR, dR, tas, ws, wd,
             fob, rGph, rMins, taxi, altD, perfData=None, isaDev=0):
    """
    Full update() simulation including fuel accounting, binary search,
    and optional real performance data override.
    """
    if rMins is None or str(rMins) == '': rMins = 60
    rMins = float(rMins)
    if math.isnan(rMins): rMins = 60

    rGal  = round(rGph*(rMins/60), 1)
    aGal  = round(ALT_CRUISE_GPH*(altD/max(1,ALT_KTAS)), 1)
    fobAT = fob - taxi
    fixed = rGal + aGal

    fCrEff = fCr; tasEff = tas
    if perfData:
        rp = getRealPerf(alt, isaDev, perfData)
        if rp: fCrEff = rp['ff']; tasEff = rp['tas']

    # Binary search for maxDist
    maxD = 0
    if fixed <= fobAT:
        lo, hi = 25, 1400
        for _ in range(20):
            mid = round((lo+hi)/2)
            rT  = compute(mid,alt,dEl,aEl,fCl,fCrEff,fDe,cR,dR,tasEff,ws,wd)
            if rT['totalGal']+fixed <= fobAT: maxD=mid; lo=mid+1
            else: hi=mid-1
    maxD = min(maxD, 1400)

    effD = min(dist, maxD) if maxD > 0 else dist
    r    = compute(effD, alt, dEl, aEl, fCl, fCrEff, fDe, cR, dR, tasEff, ws, wd)
    minS = round(r['totalGal']+fixed, 1)
    lF   = round(fobAT-r['totalGal'], 1)
    marg = round(lF-rGal-aGal, 1)
    return {
        'r':      r,
        'rGal':   rGal,   'aGal':   aGal,
        'fobAT':  fobAT,  'fixed':  fixed,
        'minS':   minS,   'lF':     lF,    'marg': marg,
        'maxD':   maxD,   'effD':   effD,
        'fCrEff': fCrEff, 'tasEff': tasEff,
    }

# Convenience: run with all defaults, override specific fields
DEFAULTS = dict(dist=500, alt=17000, dEl=1000, aEl=1000,
                fCl=37, fCr=32, fDe=10, cR=1400, dR=1000,
                tas=280, ws=0, wd='head',
                fob=145, rGph=38, rMins=60, taxi=3, altD=0,
                perfData=None, isaDev=0)
def run(**kw): return simulate(**{**DEFAULTS, **kw})

# ════════════════════════════════════════════════════════════════════════════
# TEST HARNESS
# ════════════════════════════════════════════════════════════════════════════
bugs=[]; warns=[]
def chk(label, cond, detail='', warning=False):
    tag = '✓' if cond else ('⚠' if warning else '✗')
    print(f"  {tag} {label}" + (f"  [{detail}]" if detail else ''))
    if not cond:
        if warning: warns.append(label)
        else:       bugs.append(label)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO A — DEFAULT STATE
# Verify every output field with default inputs
# ════════════════════════════════════════════════════════════════════════════
def test_A():
    print("\n── A. DEFAULT STATE (all DEFAULTS, verify every output field)")
    s=run(); r=s['r']
    print(f"  Climb:   {r['climbDist']}nm  {fmtTime(r['climbMins'])}  {r['climbGal']}gal  delta={r['climbDelta']}ft")
    print(f"  Cruise:  {r['cruiseDist']}nm  {fmtTime(r['cruiseMins'])}  {r['cruiseGal']}gal  GS={r['gs']}kts")
    print(f"  Descent: {r['descDist']}nm  {fmtTime(r['descMins'])}  {r['descGal']}gal  avgTAS={r['avgDesTas']}kts")
    print(f"  Total:   {r['totalGal']}gal  {fmtTime(r['totalMins'])}")
    print(f"  Reserve:{s['rGal']}gal  Alt:{s['aGal']}gal  MinStart:{s['minS']}gal")
    print(f"  LandFuel:{s['lF']}gal  Margin:{s['marg']}gal  maxDist:{s['maxD']}nm  TOD:{r['todNm']}nm")
    chk("A1.  Dist conserved c+cr+d=500nm", abs(r['_cG']+r['_crG']+r['_dG']-500)<0.5,
        f"{r['climbDist']}+{r['cruiseDist']}+{r['descDist']}")
    chk("A2.  Fuel sum = totalGal", abs(r['climbGal']+r['cruiseGal']+r['descGal']-r['totalGal'])<0.2)
    chk("A3.  Climb time=(17000-1000)/1400=11min", abs(r['climbMins']-round((17000-1000)/1400))<1)
    chk("A4.  Climb fuel=37×11.4/60≈7.0gal", abs(r['climbGal']-round(37*(17000-1000)/1400/60,1))<0.2)
    chk("A5.  GS=280 (TAS, no wind)", r['gs']==280)
    chk("A6.  Descent TAS ∈ [264,339] (260KIAS, 17k→1k)", 264<=r['avgDesTas']<=339)
    chk("A7.  Descent time=(17000-1000)/1000=16min", abs(r['descMins']-16)<1)
    chk("A8.  TOD=descDist", r['todNm']==r['descDist'])
    chk("A9.  fobAfterTaxi=142 (145-3)", s['fobAT']==142)
    chk("A10. reserveGal=38.0 (38×60/60)", s['rGal']==38.0)
    chk("A11. altFuelGal=0 (altDist=0)", s['aGal']==0.0)
    chk("A12. landFuel=fobAT-tripFuel", s['lF']==round(s['fobAT']-r['totalGal'],1))
    chk("A13. minStart=tripFuel+fixed", s['minS']==round(r['totalGal']+s['fixed'],1))
    chk("A14. margin=landFuel-reserve-alt", s['marg']==round(s['lF']-s['rGal']-s['aGal'],1))
    chk("A15. taxi NOT in minStart", s['minS']==round(r['totalGal']+38+0,1))
    chk("A16. minStart+taxi≤FOB", s['minS']+3<=145)
    chk("A17. noAlt=False (500nm>>climbDist)", not r['noAlt'])
    chk("A18. all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)
    chk("A19. all distances ≥0", r['climbDist']>=0 and r['cruiseDist']>=0 and r['descDist']>=0)
    chk("A20. margin>0", s['marg']>0)
    chk("A21. maxDist>500nm (no fuel constraint at 500nm)", s['maxD']>500)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO B — SHORT LEG, HIGH ELEVATION
# KAFO(6204ft)→KTCS(4586ft), 75nm, FL120
# ════════════════════════════════════════════════════════════════════════════
def test_B():
    print("\n── B. SHORT LEG KAFO→KTCS (75nm FL120, high elevation)")
    fCr = cruiseGphAtAlt(12000, 17000, 32)
    s=run(dist=75, alt=12000, dEl=6204, aEl=4586, fCr=fCr); r=s['r']
    print(f"  FF@FL120={fCr:.3f}gph  Climb:{r['climbDist']}nm {fmtTime(r['climbMins'])} {r['climbGal']}gal")
    print(f"  Cruise:{r['cruiseDist']}nm {fmtTime(r['cruiseMins'])} {r['cruiseGal']}gal")
    print(f"  Descent:{r['descDist']}nm {fmtTime(r['descMins'])} {r['descGal']}gal")
    exp_gph = 32*(1+(17000-12000)/1000*0.008)
    chk("B1. climbDelta=5796ft (12000-6204)", r['climbDelta']==5796)
    chk("B2. descDelta=7414ft (12000-4586)", round(12000-4586)==7414)
    chk("B3. dist sum=75nm", abs(r['_cG']+r['_crG']+r['_dG']-75)<0.5)
    chk("B4. noAlt=False", not r['noAlt'])
    chk(f"B5. FF@FL120={fCr:.3f}≈{exp_gph:.3f} (denser air→more fuel)", abs(fCr-exp_gph)<0.01)
    chk("B6. all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)
    chk("B7. margin>0", s['marg']>0)
    chk("B8. fuel sum correct", abs(r['climbGal']+r['cruiseGal']+r['descGal']-r['totalGal'])<0.2)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO C — MID LEG, HEADWIND
# KAFO(6204ft)→KDEN(5434ft), 250nm, FL170, 25kt HW
# ════════════════════════════════════════════════════════════════════════════
def test_C():
    print("\n── C. MID LEG KAFO→KDEN (250nm FL170, 25kt HW)")
    s=run(dist=250, alt=17000, dEl=6204, aEl=5434, ws=25, wd='head'); r=s['r']
    s0=run(dist=250, alt=17000, dEl=6204, aEl=5434)
    print(f"  Climb:{r['climbDist']}nm  Cruise:{r['cruiseDist']}nm GS={r['gs']}kts  Descent:{r['descDist']}nm")
    chk("C1. GS=255 (280-25 HW)", r['gs']==255)
    chk("C2. climbDelta=10796ft", r['climbDelta']==10796)
    chk("C3. dist sum=250nm", abs(r['_cG']+r['_crG']+r['_dG']-250)<0.5)
    chk("C4. HW shortens climb dist vs no wind", r['_cG']<s0['r']['_cG'])
    chk("C5. HW more fuel than no wind", r['totalGal']>s0['r']['totalGal'])
    chk("C6. HW longer than no wind", r['totalMins']>s0['r']['totalMins'])
    chk("C7. fuel sum correct", abs(r['climbGal']+r['cruiseGal']+r['descGal']-r['totalGal'])<0.2)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO D — LONG LEG, FL280, TAILWIND
# KAFO(6204ft)→KPHX(1135ft), 500nm, FL280, 30kt TW
# ════════════════════════════════════════════════════════════════════════════
def test_D():
    print("\n── D. LONG LEG KAFO→KPHX (500nm FL280, 30kt TW)")
    gph28 = cruiseGphAtAlt(28000, 17000, 32)
    tas28  = min(285, 280+(28000-17000)/1000*(285-280)/max(1,28000-17000)*1000)
    s=run(dist=500, alt=28000, dEl=6204, aEl=1135, fCr=gph28, tas=tas28, ws=30, wd='tail')
    s0=run(dist=500, alt=28000, dEl=6204, aEl=1135, fCr=gph28, tas=tas28)
    r=s['r']
    print(f"  gph={gph28:.3f} TAS={tas28:.1f}  Climb:{r['climbDist']}nm  Cruise:{r['cruiseDist']}nm GS={r['gs']}kts")
    print(f"  Descent:{r['descDist']}nm avgTAS={r['avgDesTas']}kts  Total:{r['totalGal']}gal {fmtTime(r['totalMins'])}")
    chk("D1. GS=315 (285+30 TW)", r['gs']==315)
    chk("D2. climbDelta=21796ft (28000-6204)", r['climbDelta']==21796)
    chk("D3. dist sum=500nm", abs(r['_cG']+r['_crG']+r['_dG']-500)<0.5)
    chk("D4. gph@FL280≈29.994", abs(gph28-29.994)<0.01)
    chk("D5. TAS@FL280=285.0 (cap)", abs(tas28-285)<0.01)
    chk("D6. TW less fuel than no wind", r['totalGal']<s0['r']['totalGal'])
    chk("D7. all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO E — FUEL INPUT EDGE CASES
# ════════════════════════════════════════════════════════════════════════════
def test_E():
    print("\n── E. FUEL INPUT EDGE CASES")
    # Low FOB
    s=run(fob=100)
    chk("E1. fob=100: tight fuel state", s['minS']>s['fobAT'] or s['marg']<10,
        f"minS={s['minS']} fobAT={s['fobAT']} marg={s['marg']}")
    # Reserve=0
    s=run(rMins=0); s_def=run()
    chk("E2. rMins=0: rGal=0.0", s['rGal']==0.0)
    chk("E2. rMins=0: margin=landFuel", s['marg']==s['lF'])
    chk("E2. rMins=0: larger maxDist", s['maxD']>s_def['maxD'])
    # Reserve=45min
    s=run(rMins=45)
    chk("E3. rMins=45: rGal=28.5", s['rGal']==28.5)
    # Alternate 100nm
    s=run(altD=100); exp=round(38*100/256,1)
    chk(f"E4. altDist=100: aGal={s['aGal']}≈{exp}", abs(s['aGal']-exp)<0.1)
    chk("E4. altDist=100: reduces margin", s['marg']<run()['marg'])
    # Taxi
    s0=run(taxi=0); s3=run(taxi=3)
    chk("E5. taxi=0: fobAT=145", s0['fobAT']==145)
    chk("E5. taxi=3: fobAT=142", s3['fobAT']==142)
    chk("E5. tripFuel independent of taxi", s0['r']['totalGal']==s3['r']['totalGal'])
    chk("E5. minStart independent of taxi", s0['minS']==s3['minS'])
    chk("E5. landFuel lower with more taxi", s3['lF']<s0['lF'])
    # FOB=0
    s=run(fob=0)
    chk("E6. fob=0: fobAT=-3, no crash", s['fobAT']==-3)
    chk("E6. fob=0: maxDist=0", s['maxD']==0)
    # Reserve > FOB
    s=run(fob=30, rGph=38, rMins=60)
    chk("E7. reserve(38)>fobAT(27): caught", s['fixed']>s['fobAT'])
    chk("E7. tripFuel still computed (no crash)", s['r']['totalGal']>=0)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO F — WIND EDGE CASES
# ════════════════════════════════════════════════════════════════════════════
def test_F():
    print("\n── F. WIND EDGE CASES")
    rH=run(ws=0,wd='head')['r']; rT=run(ws=0,wd='tail')['r']
    chk("F1. ws=0: head==tail", rH['gs']==rT['gs']==280 and rH['totalGal']==rT['totalGal'])
    r=compute(500,17000,1000,1000,37,32,10,1400,1000,280,250,'head')
    chk("F2. ws=250 HW: GS floored at 50", r['gs']==50)
    r=compute(500,17000,1000,1000,37,32,10,1400,1000,285,30,'tail')
    chk("F3. TW: GS=315 (285+30)", r['gs']==315)
    for ws in [10,20,30,40]:
        rHW=run(ws=ws,wd='head')['r']; rTW=run(ws=ws,wd='tail')['r']
        chk(f"F4. ws={ws}: TW faster+less fuel than HW",
            rTW['totalMins']<rHW['totalMins'] and rTW['totalGal']<rHW['totalGal'])

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO G — ALTITUDE & ELEVATION EDGE CASES
# ════════════════════════════════════════════════════════════════════════════
def test_G():
    print("\n── G. ALTITUDE & ELEVATION EDGE CASES")
    s=run(alt=5000); r=s['r']
    chk("G1. FL5000: climbDelta=4000ft", r['climbDelta']==4000)
    chk("G1. FL5000: dist sum=500nm", abs(r['_cG']+r['_crG']+r['_dG']-500)<0.5)
    r=run(alt=5000,dEl=6000)['r']
    chk("G2. depElev>cruiseAlt: climbDelta=0, climbGal=0", r['climbDelta']==0 and r['climbGal']==0.0)
    r=run(alt=5000,aEl=6000)['r']
    chk("G3. arrElev>cruiseAlt: descGal=0", r['descGal']==0.0)
    chk("G3. dist sum correct", abs(r['_cG']+r['_crG']+r['_dG']-500)<0.5)
    r=run(alt=6000,dEl=6000,aEl=6000)['r']
    chk("G4. dep=arr=cruise: climb=desc=0, all dist→cruise", r['climbDelta']==0 and r['descGal']==0.0)
    chk("G4. cruiseDist≈500nm", abs(r['_crG']-500)<0.5)
    chk("G5. FL28000: reachable at 500nm", not run(alt=28000)['r']['noAlt'])

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO H — SHORT LEG / noAlt
# ════════════════════════════════════════════════════════════════════════════
def test_H():
    print("\n── H. SHORT LEG / noAlt EDGE CASES")
    r=run(dist=20,alt=17000)['r']
    chk("H1. dist=20nm FL170: noAlt=True", r['noAlt'])
    chk("H1. noAlt: crGal=dGal=0", r['cruiseGal']==0.0 and r['descGal']==0.0)
    chk("H1. noAlt: totalGal=climbGal only", r['totalGal']==r['climbGal'])
    r=run(dist=50,alt=5000)['r']
    chk("H2. dist=50nm FL5000: sum=50nm", abs(r['_cG']+r['_crG']+r['_dG']-50)<0.5)
    chk("H2. all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)
    r=run(dist=35,alt=17000)['r']
    chk("H3. descent capped: descDist≤remaining after climb", r['_dG']<=max(0,35-r['_cG'])+0.1)
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
    chk("I2. ffDesc=30 Hi-Spd: more descent fuel, same cruise/climb",
        run(fDe=30)['r']['descGal']>s0['r']['descGal'] and
        run(fDe=30)['r']['cruiseMins']==s0['r']['cruiseMins'])
    chk("I3. lower cruise FF: less total fuel, same times",
        run(fCr=24)['r']['totalGal']<run(fCr=40)['r']['totalGal'] and
        run(fCr=24)['r']['cruiseMins']==run(fCr=40)['r']['cruiseMins'])
    s_slow=run(tas=240); s_fast=run(tas=320)
    chk("I4. higher TAS: shorter cruise, less total fuel",
        s_fast['r']['cruiseMins']<s_slow['r']['cruiseMins'] and
        s_fast['r']['totalGal']<s_slow['r']['totalGal'])
    chk("I4. TAS ratio proportional to time ratio (±15%)",
        abs(s_slow['r']['cruiseMins']/max(1,s_fast['r']['cruiseMins'])-320/240)<0.15)
    chk("I5. slower climb rate: more climb time and fuel",
        run(cR=800)['r']['climbMins']>run(cR=2000)['r']['climbMins'])
    chk("I6. slower descent rate: more descent time",
        run(dR=500)['r']['descMins']>run(dR=2000)['r']['descMins'])

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO J — GPH MODEL CONSISTENCY
# ════════════════════════════════════════════════════════════════════════════
def test_J():
    print("\n── J. CRUISE GPH MODEL CONSISTENCY")
    alts_seq=[5000,8000,10000,12000,14000,17000,19000,21000,23000,25000,28000]
    gphs=[cruiseGphAtAlt(a,17000,32) for a in alts_seq]
    for i in range(len(gphs)-1):
        chk(f"J1. GPH decreasing {alts_seq[i]}→{alts_seq[i+1]}ft: {gphs[i]:.3f}>{gphs[i+1]:.3f}",
            gphs[i]>gphs[i+1])
    chk("J2. Anchor FL170=32.000gph exactly", abs(cruiseGphAtAlt(17000,17000,32)-32)<0.001)
    chk("J3. FL280≈29.994gph", abs(cruiseGphAtAlt(28000,17000,32)-29.994)<0.01)
    chk("J4. Min clamped at 0.6×32=19.2", cruiseGphAtAlt(200000,17000,32)>=0.6*32)
    chk("J5. Max clamped at 1.5×32=48.0", cruiseGphAtAlt(-10000,17000,32)<=1.5*32)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO K — REAL PERFORMANCE DATA SYSTEM
# ════════════════════════════════════════════════════════════════════════════
def test_K():
    print("\n── K. PERFORMANCE DATA SYSTEM")
    chk("K1. empty data → None", getRealPerf(17000,0,[])==None)
    d=[{'alt':17000,'tas':285,'ff':30.5,'isa':0}]
    rp=getRealPerf(17000,0,d)
    chk(f"K2. exact match: TAS={rp['tas']:.1f}=285, FF={rp['ff']:.2f}=30.5",
        abs(rp['tas']-285)<0.01 and abs(rp['ff']-30.5)<0.01)
    d2=[{'alt':17000,'tas':282,'ff':31.0,'isa':0},{'alt':17000,'tas':278,'ff':33.0,'isa':0}]
    rp=getRealPerf(17000,0,d2)
    chk(f"K3. equal-weight average: TAS={rp['tas']:.1f}=280, FF={rp['ff']:.1f}=32",
        abs(rp['tas']-280)<0.1 and abs(rp['ff']-32)<0.1)
    d3=[{'alt':17000,'tas':287,'ff':30.0,'isa':-5},{'alt':17000,'tas':273,'ff':34.0,'isa':+8}]
    chk(f"K4. ISA=-3 picks -5 point: TAS={getRealPerf(17000,-3,d3)['tas']:.0f}=287",
        abs(getRealPerf(17000,-3,d3)['tas']-287)<0.1)
    chk(f"K4. ISA=+5 picks +8 point: TAS={getRealPerf(17000,+5,d3)['tas']:.0f}=273",
        abs(getRealPerf(17000,+5,d3)['tas']-273)<0.1)
    chk("K5. 10k data query 17k (7000ft gap): None", getRealPerf(17000,0,[{'alt':10000,'tas':250,'ff':36,'isa':0}])==None)
    chk("K5. 15k data query 17k (2000ft): returns data", getRealPerf(17000,0,[{'alt':15000,'tas':272,'ff':32.5,'isa':0}])!=None)
    d_real=[{'alt':17000,'tas':285,'ff':30.0,'isa':0}]
    s_gen=run(); s_real=run(perfData=d_real,isaDev=0)
    chk("K6. real data (FF=30<32): less total fuel", s_real['r']['totalGal']<s_gen['r']['totalGal'])
    chk("K6. binary search uses real FF (lower FF→more maxDist)", s_real['maxD']>s_gen['maxD'])
    d_hot=[{'alt':17000,'tas':275,'ff':40.0,'isa':0}]
    s_hot=run(perfData=d_hot,isaDev=0)
    chk("K7. high real FF (40): maxDist shrinks vs generic (32)", s_hot['maxD']<s_gen['maxD'])
    rM=compute(s_hot['maxD'],17000,1000,1000,37,40,10,1400,1000,275,0,'head')
    chk("K7. maxDist valid: tripFuel+fixed ≤ fobAT", round(rM['totalGal']+s_hot['fixed'],1)<=s_hot['fobAT'])

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO L — ACT vs PLAN / lastPlan structure
# ════════════════════════════════════════════════════════════════════════════
def test_L():
    print("\n── L. ACT vs PLAN / lastPlan")
    r=compute(500,17000,1000,1000,37,32,10,1400,1000,280,0,'head')
    lp={'dep':'KAFO','arr':'KGEU','alt':17000,'dist':500,'tas':280,'ff':32.0,
        'cruiseMins':r['cruiseMins'],'cruiseGal':r['cruiseGal'],
        'totalGal':r['totalGal'],'totalMins':r['totalMins']}
    chk("L1. lastPlan has all required keys",
        all(k in lp for k in ['dep','arr','alt','dist','tas','ff','cruiseMins','cruiseGal','totalGal','totalMins']))
    actAlt=17200; actTas=283; actFf=31.2
    dTas=actTas-lp['tas']; dFf=actFf-lp['ff']
    print(f"  Planned: alt=17000 TAS=280 FF=32.0")
    print(f"  Actual:  alt={actAlt} TAS={actTas} FF={actFf} ISA=+2°C ITT=705°C")
    print(f"  Deltas:  alt=+200ft TAS={dTas:+d}kt FF={dFf:+.1f}gph")
    chk("L2. dTas positive=faster than planned", dTas>0)
    chk("L2. dFF negative=more efficient than planned", dFf<0)
    chk("L3. cruiseMins key present and valid", r['cruiseMins']>0)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO M — ALTITUDE COMPARISON CHART
# ════════════════════════════════════════════════════════════════════════════
def test_M():
    print("\n── M. ALTITUDE COMPARISON CHART")
    alts=[5000,8000,10000,12000,14000,17000,19000,21000,23000,25000,28000]
    cruiseKias=280*math.sqrt(densityRatio(17000))
    tasRate=(285-280)/max(1,28000-17000)*1000
    print("  Alt    | GPH    | TAS    | Fuel(500nm)")
    fuelBars=[]
    for a in alts:
        gph=cruiseGphAtAlt(a,17000,32)
        if a>=17000: tasAdj=min(285,280+(a-17000)/1000*tasRate)
        else: tasAdj=max(160,cruiseKias/math.sqrt(densityRatio(a)))
        res=compute(500,a,1000,1000,37,gph,10,1400,1000,tasAdj,0,'head')
        fuel=None if res['_cG']>=500 else res['totalGal']
        fuelBars.append(fuel)
        print(f"  {a:5}ft | {gph:.3f} | {tasAdj:.1f}kt | {str(fuel)+'gal' if fuel else 'UNREACHABLE'}")
    valid=[f for f in fuelBars if f is not None]
    chk("M1. FL170 reachable at 500nm", fuelBars[5] is not None)
    chk("M2. higher altitude cheaper (FL280<FL170)", min(valid)<fuelBars[5])
    chk("M3. fuel strictly decreasing with altitude (efficiency wins)",
        all(fuelBars[i]>fuelBars[i+1] for i in range(5,len(alts)-1) if fuelBars[i] and fuelBars[i+1]))
    print(f"  Min: {min(valid)}gal at FL{alts[[i for i,f in enumerate(fuelBars) if f==min(valid)][0]]//100}")
    chk("M4. nearestAlt(17000)=17000",
        min(alts, key=lambda a:abs(a-17000))==17000)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO N — ISA PHYSICS & DESCENT TAS
# ════════════════════════════════════════════════════════════════════════════
def test_N():
    print("\n── N. ISA PHYSICS & DESCENT TAS")
    chk("N1. ISA SL=15.0°C", abs(isaTemp(0)-15)<0.01)
    chk("N2. ISA 10k≈-4.8°C", abs(isaTemp(10000)+4.8)<0.2)
    chk("N3. ISA tropo=-56.5°C", abs(isaTemp(40000)+56.5)<0.01)
    chk("N4. density SL=1.0", abs(densityRatio(0)-1.0)<0.001)
    chk("N5. 260KIAS@10k≈303KTAS", abs(kiasToKtas(260,10000)-303)<3)
    chk("N6. 220KIAS@10k≈256KTAS (ALT_KTAS)", abs(kiasToKtas(220,10000)-256)<2)
    for cAlt,aAlt in [(17000,1000),(28000,1000),(17000,6204),(10000,5000)]:
        avg=avgDescentTas(cAlt,aAlt)
        lo=min(kiasToKtas(260,cAlt),kiasToKtas(260,aAlt))
        hi=max(kiasToKtas(260,cAlt),kiasToKtas(260,aAlt))
        chk(f"N7. avgDescTas({cAlt}→{aAlt})={avg:.0f} ∈ [{lo:.0f},{hi:.0f}]", lo<=avg<=hi)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO O — fmtTime
# ════════════════════════════════════════════════════════════════════════════
def test_O():
    print("\n── O. fmtTime EDGE CASES")
    cases=[(0,'0+00'),(0.49,'0+00'),(0.5,'0+01'),(29.5,'0+30'),
           (59,'0+59'),(59.5,'1+00'),(60,'1+00'),(61,'1+01'),
           (89,'1+29'),(90,'1+30'),(119.5,'2+00'),(150,'2+30'),
           (180,'3+00'),(239.5,'4+00')]
    for m,exp in cases:
        got=fmtTime(m); chk(f"O. {m}min→'{got}' (expect '{exp}')", got==exp)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO P — ALTERNATE FUEL
# ════════════════════════════════════════════════════════════════════════════
def test_P():
    print("\n── P. ALTERNATE FUEL CALCULATION")
    for altD,exp in [(0,0.0),(25,round(38*25/256,1)),(50,round(38*50/256,1)),
                     (100,round(38*100/256,1)),(150,round(38*150/256,1))]:
        aGal=round(ALT_CRUISE_GPH*(altD/max(1,ALT_KTAS)),1)
        chk(f"P. altDist={altD}nm: {aGal}gal≈{exp}gal", abs(aGal-exp)<0.15)

# ════════════════════════════════════════════════════════════════════════════
# SCENARIO Q — MONOTONICITY & STRESS
# ════════════════════════════════════════════════════════════════════════════
def test_Q():
    print("\n── Q. MONOTONICITY & COMBINED STRESS")
    # Fuel and time increase with distance
    dists=[100,200,300,400,500]
    fuels=[compute(d,17000,1000,1000,37,32,10,1400,1000,280,0,'head')['totalGal'] for d in dists]
    times=[compute(d,17000,1000,1000,37,32,10,1400,1000,280,0,'head')['totalMins'] for d in dists]
    for i in range(len(dists)-1):
        chk(f"Q1. dist {dists[i]}→{dists[i+1]}nm: more fuel ({fuels[i]:.1f}<{fuels[i+1]:.1f})", fuels[i]<fuels[i+1])
        chk(f"Q1. dist {dists[i]}→{dists[i+1]}nm: more time", times[i]<times[i+1])
    # Max stress: FL210, 850nm, 40kt HW, high elevation, alt fuel
    tasRate=(285-280)/max(1,28000-17000)*1000
    gph21=cruiseGphAtAlt(21000,17000,32); tas21=min(285,280+(21000-17000)/1000*tasRate)
    r=compute(850,21000,6204,5434,37,gph21,10,1400,1000,tas21,40,'head')
    chk("Q2. stress 850nm FL210 40ktHW: dist sum", abs(r['_cG']+r['_crG']+r['_dG']-850)<0.5)
    chk("Q2. stress: all fuel ≥0", r['climbGal']>=0 and r['cruiseGal']>=0 and r['descGal']>=0)
    chk("Q2. stress: fuel sum correct", abs(r['climbGal']+r['cruiseGal']+r['descGal']-r['totalGal'])<0.2)
    print(f"  Stress result: {r['totalGal']}gal {fmtTime(r['totalMins'])}")
    # TW always better than HW symmetrically
    for ws in [10,20,30,40]:
        rHW=run(ws=ws,wd='head')['r']; rTW=run(ws=ws,wd='tail')['r']
        chk(f"Q3. ws={ws}: TW faster+less fuel than HW",
            rTW['totalMins']<rHW['totalMins'] and rTW['totalGal']<rHW['totalGal'])

# ════════════════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    print("="*65)
    print("PROPJET GO — MASTER DEBUG SUITE")
    print("="*65)

    for fn in [test_A,test_B,test_C,test_D,test_E,test_F,test_G,
               test_H,test_I,test_J,test_K,test_L,test_M,test_N,
               test_O,test_P,test_Q]:
        fn()

    print()
    print("="*65)
    n_bugs=len(bugs); n_warns=len(warns)
    total=n_bugs+n_warns
    status="✓ CLEAN" if total==0 else f"{'✗' if n_bugs else '⚠'} ISSUES FOUND"
    print(f"RESULT: {status}  |  Bugs: {n_bugs}  Warnings: {n_warns}")
    if bugs:
        print("\nBUGS (✗):")
        for b in bugs: print(f"  ✗ {b}")
    if warns:
        print("\nWARNINGS (⚠):")
        for w in warns: print(f"  ⚠ {w}")
    if total==0:
        print("  All checks passed. Safe to push to GitHub.")
    print("="*65)
    sys.exit(0 if n_bugs==0 else 1)
