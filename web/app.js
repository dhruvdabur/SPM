const $ = id => document.getElementById(id);
const axes = ['yaw', 'pitch', 'roll'];
let target = {yaw:0,pitch:0,roll:0}, pending = null, busy = false, connected = false;
function refresh() {
  for (const axis of axes) { $(axis).value = target[axis]; $(axis+'-value').textContent = target[axis].toFixed(1)+'°'; }
  $('stick').style.transform = `translate(${target.roll/20*76}px,${-target.pitch/20*76}px)`;
  draw();
}
function changed() { refresh(); if ($('live').checked) pending = {...target}; }
for (const axis of axes) $(axis).addEventListener('input', e => { target[axis] = Number(e.target.value); changed(); });
$('home').onclick = () => {target = {yaw:0,pitch:0,roll:0}; refresh(); pending = {...target};};
$('send').onclick = () => {pending = {...target};};
$('live').onchange = () => {pending = $('live').checked ? {...target} : null;};
let pointer = null;
function moveStick(e) {
  const box = $('joystick').getBoundingClientRect();
  let x = (e.clientX-box.left-box.width/2)/76, y = (box.top+box.height/2-e.clientY)/76;
  const radius = Math.max(1, Math.hypot(x,y)); x/=radius; y/=radius;
  target.roll = Math.round(x*200)/10; target.pitch = Math.round(y*200)/10; changed();
}
$('joystick').onpointerdown = e => { if(pointer!==null)return; pointer=e.pointerId; e.currentTarget.setPointerCapture(pointer); moveStick(e); };
$('joystick').onpointermove = e => {if(e.pointerId===pointer)moveStick(e);};
$('joystick').onpointerup = $('joystick').onpointercancel = () => {pointer=null;};
function showState(state) {
  state.joint_degrees.forEach((v,i)=> { $('q'+(i+1)).textContent=v.toFixed(2)+'°'; $('r'+(i+1)).textContent=state.joint_radians[i].toFixed(4)+' rad'; });
  $('mode').textContent=state.ros?'ROS output enabled · joint commands in radians':'Local IK · ROS output off';
  $('connection').textContent='● Connected'; connected=true;
}
setInterval(async () => {
  if(busy || !pending)return;
  const command=pending; pending=null; busy=true;
  try {
    const response=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(command),signal:AbortSignal.timeout(5000)});
    const state=await response.json();
    if(!response.ok)throw new Error(state.error || 'Command rejected');
    showState(state); $('status').className='';
    $('status').textContent=`Accepted Y/P/R: ${axes.map(a=>command[a].toFixed(1)+'°').join(' / ')}. Closure: ${(state.closure_error_m*1e6).toFixed(2)} µm.`;
  } catch(error) {
    $('status').className='error'; $('status').textContent=error.message+'. Joint targets show the last accepted command.';
    if(error instanceof TypeError || error.name==='TimeoutError') {connected=false; $('connection').textContent='○ Connection lost'; pending=null;}
  } finally {busy=false;}
},100);
async function initialize() {
  try { const response=await fetch('/api/state'); if(!response.ok)throw new Error('Connection failed'); const state=await response.json(); showState(state); axes.forEach((a,i)=>target[a]=state.ypr_degrees[i]); refresh(); }
  catch(e) { $('connection').textContent='○ Offline'; $('status').textContent='Start python web_control.py, then reload this page.'; }
}
// Lightweight projected schematic. Rotation is the same ZYX convention as IK.
function draw() {
  const canvas=$('preview'), ctx=canvas.getContext('2d'), box=canvas.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  canvas.width=Math.round(box.width*dpr); canvas.height=Math.round(box.height*dpr); ctx.scale(dpr,dpr);
  const w=box.width,h=box.height,scale=Math.min(w/3.2,h/2.8);
  const project=([x,y,z])=>[w/2+scale*(.87*x-.5*y),h*.7+scale*(.25*x+.43*y-.95*z)];
  const rotate=p=>{let [x,y,z]=p; const [a,b,c]=axes.map(k=>target[k]*Math.PI/180); [y,z]=[Math.cos(c)*y-Math.sin(c)*z,Math.sin(c)*y+Math.cos(c)*z]; [x,z]=[Math.cos(b)*x+Math.sin(b)*z,-Math.sin(b)*x+Math.cos(b)*z];return [Math.cos(a)*x-Math.sin(a)*y,Math.sin(a)*x+Math.cos(a)*y,z+1];};
  function line(points,color,width=1,fill=null){ctx.beginPath();points.map(project).forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y)); if(fill){ctx.closePath();ctx.fillStyle=fill;ctx.fill();}ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
  for(let i=-4;i<=4;i++){line([[i/3,-1.4,0],[i/3,1.4,0]],'#293946');line([[-1.4,i/3,0],[1.4,i/3,0]],'#293946');}
  const base=[], top=[];
  for(let i=0;i<3;i++){let a=i*Math.PI*2/3-Math.PI/2;base.push([.83*Math.cos(a),.83*Math.sin(a),0]);top.push(rotate([.62*Math.cos(a),.62*Math.sin(a),0]));}
  line([...base,base[0]],'#637080',2,'#1b2733');
  for(let i=0;i<3;i++){const mid=[base[i][0]*1.08,base[i][1]*1.08,.5];line([base[i],mid],'#668394',7);line([mid,top[i]],'#c0f582',5);}
  line([...top,top[0]],'#d4ffa4',2,'#8cbc6955');
  for(const p of top){const [x,y]=project(p);ctx.beginPath();ctx.arc(x,y,5,0,Math.PI*2);ctx.fillStyle='#d7f9b7';ctx.fill();}
  const center=rotate([0,0,0]);
  [[[.45,0,0],'#ff9ba5','X'],[[0,.45,0],'#c0f582','Y'],[[0,0,.55],'#8caaff','Z']].forEach(([p,color,label])=>{const end=rotate(p);line([center,end],color,2); const [x,y]=project(end);ctx.fillStyle=color;ctx.font='11px system-ui';ctx.fillText(label,x+5,y-5);});
}
new ResizeObserver(draw).observe($('preview').parentElement); refresh(); initialize();
