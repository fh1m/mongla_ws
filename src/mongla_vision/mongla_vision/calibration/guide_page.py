"""The calibration page `guide.py` serves at `/`: one HTML document, inline
script and style, no build step.

`test_calibration_page.py` parses its script with node and checks every route
it fetches is one the server answers.
"""

PAGE = """<!doctype html><meta charset=utf-8>
<title>Camera calibration</title>
<style>
 :root{--bg:#0e1116;--fg:#e6edf3;--dim:#8b949e;--ok:#3fb950;--no:#f85149;
       --line:#21262d;--acc:#58a6ff;--warn:#d29922}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--fg);
      font:14px/1.55 ui-monospace,SFMono-Regular,Menlo,monospace}
 header{padding:12px 20px;border-bottom:1px solid var(--line);display:flex;
        gap:16px;align-items:baseline}
 h1{font-size:14px;margin:0;letter-spacing:.09em;text-transform:uppercase}
 .sub{color:var(--dim);font-size:12px}
 main{display:grid;grid-template-columns:minmax(0,1fr) 340px}
 .vid{padding:16px}
 img{width:100%;display:block;border:1px solid var(--line);border-radius:4px}
 aside{padding:18px;border-left:1px solid var(--line)}
 .step{font-size:12px;color:var(--dim);letter-spacing:.1em;
       text-transform:uppercase}
 .todo{font-size:21px;line-height:1.35;margin:6px 0 14px}
 .todo b{color:var(--acc)}
 .chk{display:flex;gap:10px;align-items:center;margin:6px 0;font-size:13px}
 .dot{width:9px;height:9px;border-radius:50%;background:#30363d;flex:none}
 .dot.on{background:var(--ok)} .dot.off{background:var(--no)}
 .msg{margin:14px 0;padding:9px 11px;border-radius:4px;background:#161b22;
      border-left:3px solid var(--warn);font-size:13px;min-height:38px}
 .bar{height:6px;background:#161b22;border-radius:3px;overflow:hidden;
      margin:10px 0 4px}
 .bar i{display:block;height:100%;background:var(--acc);width:0;
        transition:width .12s linear}
 .prog{height:6px;background:#161b22;border-radius:3px;overflow:hidden}
 .prog i{display:block;height:100%;background:var(--ok);width:0}
 button{width:100%;padding:10px;border:0;border-radius:4px;background:var(--acc);
        color:#04121f;font:inherit;font-weight:700;cursor:pointer;margin-top:6px}
 button:disabled{background:#21262d;color:var(--dim);cursor:not-allowed}
 button.ghost{background:#21262d;color:var(--dim);font-weight:400}
 table#ans{width:100%;border-collapse:collapse;font-size:12px}
 table#ans td{padding:2px 0;vertical-align:top}
 table#ans td:first-child{color:var(--dim);white-space:nowrap;padding-right:10px}
 table#ans td:last-child{text-align:right;font-variant-numeric:tabular-nums}
 .good{color:var(--ok)} .bad{color:var(--no)} .warn2{color:var(--warn)}
 .med{font:700 15px/1.6 ui-monospace,monospace;letter-spacing:.16em;
 padding:6px 10px;border-radius:6px;text-align:center;margin-bottom:8px}
.med.air{background:#12331e;color:#7fe3a1;border:1px solid #2c6b41}
.med.water{background:#0d2740;color:#79c6ff;border:1px solid #2b5f8c}
.lib{border:1px solid #333;border-radius:6px;padding:7px 9px;margin:6px 0}
.cam{border:1px solid var(--line);border-radius:4px;padding:8px 10px;
      margin-bottom:8px;font-size:12px}
 .cam.act{border-color:var(--acc)}
 .cam b{font-weight:600} .cam .d{color:var(--dim)}
 .cam button{width:auto;padding:4px 10px;margin-top:6px;font-size:11px}
 .btnrow{display:flex;gap:6px;margin-top:6px}
 .btnrow button{margin-top:0}
 .btnrow a{flex:1;text-decoration:none}
 .btnrow input{flex:1;min-width:0;background:#161b22;border:1px solid var(--line);
               color:var(--fg);border-radius:4px;padding:6px;font:inherit;font-size:12px}
 pre{background:#161b22;padding:10px;border-radius:4px;font-size:11px;
     max-height:34vh;overflow:auto;white-space:pre-wrap;margin-top:8px}
 h2{font-size:11px;letter-spacing:.12em;text-transform:uppercase;
    color:var(--dim);margin:20px 0 6px}
</style>
<header><h1>Calibration</h1>
 <span class=sub>each step is checked before it counts</span>
 <span class=sub id=perf style="margin-left:auto"></span></header>
<main>
 <div class=vid><img src="/video"></div>
 <aside>
  <div class=step id=stepno></div>
  <div class=todo id=todo></div>
  <div class=chk><span class=dot id=dc></span><span id=tc></span></div>
  <div class=chk><span class=dot id=dt></span><span id=tt></span></div>
  <div class=chk><span class=dot id=ds></span><span id=ts></span></div>
  <div class=prog><i id=hold></i></div>
  <div class=msg id=msg></div>
  <h2>Medium</h2>
  <div id=medbar class=med>AIR</div>
  <div class=btnrow>
    <button class=ghost id=mair>Calibrate in AIR</button>
    <button class=ghost id=mwat>Calibrate in WATER</button>
  </div>
  <div class=sub>air and water differ by ~1.44&times; on this hull. The
   medium is written into the file and onto its name, so the two can never
   be confused. Calibrate in air unless you are validating.</div>
  <h2>Cameras on this vehicle</h2>
  <div id=cams class=sub>scanning…</div>
  <h2>Saved calibrations</h2>
  <div class=d id=libdir></div>
  <div id=lib class=sub>scanning…</div>
  <h2>The answer so far</h2>
  <table id=ans><tr><td colspan=2 class=sub>needs 6 usable views</td></tr></table>
  <h2>Certainty <span class=sub>(AprilCal Max ERE)</span></h2>
  <div class=bar><i id=eb></i></div>
  <div class=sub id=et></div>
  <div class=sub id=sg style="margin-top:6px"></div>
  <h2>Progress</h2>
  <div class=bar><i id=pb></i></div>
  <div class=sub id=pt></div>
  <button class=ghost id=skip>Skip this pose</button>
  <div class=btnrow>
    <button class=ghost id=undo>Delete last</button>
    <button class=ghost id=undist>Undistorted view</button>
  </div>
  <h2>Board</h2>
  <div class=btnrow>
    <input id=gr size=6 title="inner corners, e.g. 8x6">
    <input id=sq size=7 title="square size in metres">
    <button class=ghost id=setb>Apply</button>
  </div>
  <div class=sub>a wrong grid detects NOTHING and looks like bad lighting</div>
  <h2>Finish</h2>
  <button id=go>Run calibration</button>
  <div class=btnrow>
    <a id=dl href="/calibration.json" download><button class=ghost
       style="width:100%">Download result</button></a>
    <button class=ghost id=rst>Reset all captures</button>
  </div>
  <div class=sub id=sv style="margin-top:6px"></div>
  <pre id=out></pre>
 </aside>
</main>
<script>
const $=i=>document.getElementById(i);
async function tick(){
 try{
  const s=await (await fetch('/status')).json();
  $('perf').textContent=s.fps.toFixed(0)+' fps · detect '+s.det_ms.toFixed(0)+' ms';
  if(s.done){
    $('stepno').textContent='all poses captured';
    $('todo').innerHTML='Press <b>Run calibration</b>.';
    $('tc').textContent='';$('tt').textContent='';$('ts').textContent='';
  }else{
    $('stepno').textContent='Step '+(s.i+1)+' of '+s.n;
    $('todo').innerHTML='Put the board in the <b>'+s.want_cell+
      '</b> of the frame,<br>and hold it <b>'+s.want_tilt+'</b>.';
    $('tc').textContent='position: '+(s.cell? s.cell : 'no board');
    $('tt').textContent='tilt: '+(s.band!==null? s.band : '-');
    $('ts').textContent = s.sharp===null ? 'sharpness: -'
        : 'sharpness: '+s.sharp.toFixed(1)+' px (want <'+s.sharp_max+')';
    $('ds').className='dot '+(s.sharp===null?'':(s.sharp<=s.sharp_max?'on':'off'));
    $('dc').className='dot '+(s.ok_cell?'on':'off');
    $('dt').className='dot '+(s.ok_tilt?'on':'off');
  }
  $('hold').style.width=(100*s.hold)+'%';
  $('msg').textContent=s.err||s.msg;
  $('pb').style.width=(100*s.i/s.n)+'%';
  $('pt').textContent=s.i+' of '+s.n+' poses captured ('+s.n_boards+' usable)';
  // Certainty bar: full when Max ERE is under the bar. Log scale, because
  // it starts in the tens of pixels and the last factor of two is the part
  // that matters.
  let pct=0, txt='need 6 usable views before this can be computed';
  if (s.ere!==null && isFinite(s.ere)) {
    pct = Math.max(0, Math.min(100, 100*(1 - Math.log10(Math.max(s.ere,s.ere_bar))
                                          /Math.log10(40))));
    txt = 'Max ERE '+s.ere.toFixed(2)+' px  (bar <'+s.ere_bar.toFixed(1)+')';
    if (s.ere<=s.ere_bar) txt += '  — DONE, calibration is certain';
  }
  $('eb').style.width=pct+'%';
  $('et').textContent=txt;
  // The answer as it forms, with a verdict per line, so a calibration that
  // is going wrong is visible EARLY rather than at the end.
  const rows=[];
  if (s.fx!==null){
    const sdpct = s.fx_sd!==null ? 100*s.fx_sd/s.fx : null;
    const cls = sdpct===null ? '' : (sdpct<0.5?'good':(sdpct<1.5?'warn2':'bad'));
    rows.push(['focal fx / fy', s.fx.toFixed(1)+' / '+s.fy.toFixed(1)+' px','']);
    rows.push(['centre cx / cy', s.cx.toFixed(0)+' / '+s.cy.toFixed(0)+' px','']);
    if (sdpct!==null)
      rows.push(['fx spread across folds', sdpct.toFixed(2)+' %', cls]);
    rows.push(['HFOV in air', s.hfov_air.toFixed(2)+'°','']);
    rows.push(['HFOV in water', s.hfov_water.toFixed(2)+'°','good']);
    rows.push(['usable views', s.n_boards, s.n_boards>=10?'good':'warn2']);
  }
  $('ans').innerHTML = rows.length
    ? rows.map(r=>'<tr><td>'+r[0]+'</td><td class="'+r[2]+'">'+r[1]+'</td></tr>').join('')
    : '<tr><td colspan=2 class=sub>needs 6 usable views</td></tr>';
  $('sg').textContent = s.suggest
      ? 'solver suggests next: '+s.suggest+
        (s.ere_pred? '  (predicts '+s.ere_pred.toFixed(1)+' px)':'')
      : '';
  const enough = (s.ere!==null && isFinite(s.ere) && s.ere<=s.ere_bar) || s.done;
  const b=$('go'); b.disabled=(s.solve==='running')||!enough;
  b.textContent=s.solve==='running'?'solving...':
    (!enough?'keep capturing — not certain yet':
     (s.solve==='done'?'Re-run calibration':'Run calibration'));
  $('sv').textContent = s.solve==='done'?'installed — rebuild to use it':
    s.solve==='failed'?'solve failed, see below':'';
  $('out').textContent=s.solve_out||'';
  $('skip').style.display=s.done?'none':'block';
  const mb=$('medbar'); mb.textContent=(s.medium||'air').toUpperCase();
  mb.className='med '+(s.medium||'air');
  $('undist').textContent = s.undistort?'Raw view':'Undistorted view';
  $('undist').disabled = s.fx===null;
  $('dl').style.pointerEvents = s.have_result?'auto':'none';
  $('dl').style.opacity = s.have_result?1:0.4;
  if(!$('gr').matches(':focus') && !$('gr').value) $('gr').value=s.grid;
  if(!$('sq').matches(':focus') && !$('sq').value) $('sq').value=s.square||'';
 }catch(e){}
}
$('go').onclick=async()=>{await fetch('/solve',{method:'POST'});tick();};
$('undo').onclick=async()=>{await fetch('/undo',{method:'POST'});tick();};
$('undist').onclick=async()=>{await fetch('/undistort',{method:'POST'});tick();};
$('rst').onclick=async()=>{
  if(confirm('Delete every capture and start over?')){
    await fetch('/reset',{method:'POST'});tick();}};
$('setb').onclick=async()=>{
  const g=$('gr').value.trim(), q=$('sq').value.trim();
  if(!g&&!q) return;
  $('msg').textContent='restarting with the new board…';
  await fetch('/board?grid='+encodeURIComponent(g)+'&square='+encodeURIComponent(q),
              {method:'POST'});
  setTimeout(()=>location.reload(), 5000);};
$('skip').onclick=async()=>{await fetch('/skip',{method:'POST'});tick();};
async function cams(){
 try{
  const r=await (await fetch('/cameras')).json();
  window._cams=r.cameras;
  document.getElementById('cams').innerHTML = r.cameras.length ? r.cameras.map(c=>{
    const active = String(c.device).endsWith(String(r.active));
    const cal=c.calibration;
    let line, cls;
    if(!cal){ line='<span class=bad>NO CALIBRATION</span>'; cls='bad'; }
    else if(c.match==='confirmed'){
      line='<span class=good>calibrated</span> · fx '+cal.fx.toFixed(0)+
           ' · HFOV '+cal.hfov_air.toFixed(1)+'° air / '+cal.hfov_water.toFixed(1)+'° water'+
           (cal.max_ere_px?' · ERE '+cal.max_ere_px.toFixed(1)+'px':'')+
           ' · '+(cal.captured||'?'); }
    else { line='<span class=warn2>UNVERIFIED</span> — '+cal.file+
           ' fits by name but records no camera identity, so it cannot be'+
           ' proven to be this unit'; }
    const prof = cal && cal.applies_to ? cal.applies_to[0]
               : (String(c.card||'').toLowerCase().includes('fantech')
                  ? 'pi_forward' : 'pi_downward');
    return '<div class="cam'+(active?' act':'')+'">'+
      '<b>'+(c.card||c.device)+'</b> <span class=d>'+c.device+
      ' · usb '+(c.usb_vid||'?')+':'+(c.usb_pid||'?')+
      (active?' · ACTIVE':'')+'</span><br>'+line+
      (active?'':'<button onclick="sw(\\''+c.device+'\\',\\''+prof+'\\')">'+
        (cal?'Re-calibrate':'Calibrate')+' this camera</button>')+'</div>';
  }).join('') : '<span class=bad>no USB cameras found</span>';
 }catch(e){}
}
$('mair').onclick=()=>setmed('air');
$('mwat').onclick=()=>setmed('water');
async function setmed(m){
  if(m==='water' && !confirm(
     'Calibrate IN WATER?\\n\\nThe normal path is to calibrate in air once '+
     'and correct refraction analytically. An in-water calibration is a '+
     'VALIDATION of that correction. It is saved separately and never '+
     'overwrites the air one.')) return;
  $('msg').textContent='restarting in '+m+'…';
  await fetch('/switch?medium='+m,{method:'POST'});
  setTimeout(()=>location.reload(), 5000);
}
window._cams=[];
async function lib(){
 try{
  const r=await (await fetch('/library')).json();
  document.getElementById('lib').innerHTML = r.library.length ? r.library.map(c=>{
    // Only offer to apply a calibration to a camera that is actually here.
    // Applying to an absent device would write a file claiming a unit
    // nobody can check.
    const btns=(window._cams||[]).map(cam=>
      '<button onclick="ap(\\''+c.file+'\\',\\''+cam.device+'\\',\\''+
      ((c.applies_to&&c.applies_to[0])||'pi_forward')+'\\')">apply to '+
      (cam.card||cam.device)+'</button>').join(' ');
    return '<div class=lib><b>'+c.file+'</b> <span class=d>'+
      (c.medium||'air').toUpperCase()+' · '+c.width+'x'+c.height+
      (c.fx?' · fx '+c.fx.toFixed(0):'')+
      (c.views?' · '+c.views+' views':'')+
      (c.max_ere_px?' · ERE '+c.max_ere_px.toFixed(1)+'px':'')+
      '</span><br><span class=sub>'+(c.camera||'no camera recorded')+
      ' · '+(c.captured||'?')+'</span><br>'+btns+'</div>';
  }).join('') : '<span class=sub>none saved yet</span>';
  // Name the directory on the page. It is resolved by walking up for
  // `src/mongla_vision/config/calibration`, and if that walk ever fails the
  // fallback is a folder OUTSIDE the package -- which would look exactly
  // like an empty library.
  document.getElementById('libdir').textContent = r.dir;
 }catch(e){}
}
async function ap(file,dev,prof){
  if(!confirm('Apply '+file+' to '+dev+'?\\n\\nThis asserts they are the SAME '+
    'physical camera. The file records that YOU asserted it, not that it '+
    'was measured here.')) return;
  await fetch('/apply?file='+encodeURIComponent(file)+'&device='+
    encodeURIComponent(dev)+'&applies_to='+encodeURIComponent(prof),
    {method:'POST'});
  setTimeout(()=>{lib();cams();}, 500);
}
async function sw(dev,prof){
  const n=dev.replace(/\\D+/g,'');
  document.getElementById('cams').innerHTML='<span class=warn2>switching to '+dev+'…</span>';
  await fetch('/switch?device='+n+'&applies_to='+prof,{method:'POST'});
  setTimeout(()=>location.reload(), 5000);
}
setInterval(tick,300);tick();
setInterval(cams,4000);cams();
setInterval(lib,6000);lib();
</script>
"""
