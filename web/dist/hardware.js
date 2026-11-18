const $=id=>document.getElementById(id);
const escape=value=>String(value??'').replace(/[&<>"']/g,char=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[char]));
export function createHardwarePage({notify,log,onTargetRequested,onAdvanced}){
 let current=null,pending=false,dead=false,timer=null,deviceKey='';
 async function api(path,payload){
  const response=await fetch(path,{cache:'no-store',...(payload===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)}),signal:AbortSignal.timeout(40000)});
  const data=await response.json();if(!response.ok||data.success===false)throw Error(data.message||'连接请求失败');return data;
 }
 function render(data){
  current=data;
  if(!data.available){$('hardware-summary').textContent='当前服务未开启本机连接管理';$('hardware-message').textContent=data.message||'请使用本机网页工作台。';$('hardware-connect').disabled=true;return}
  const connection=data.connection,busy=pending||!!data.switching_to||['starting','stopping'].includes(connection.status),connected=connection.status==='running';
  const devices=data.devices,selected=$('hardware-device').value,key=JSON.stringify(devices.map(device=>[device.path,device.accessible,device.owners]));
  if(key!==deviceKey){deviceKey=key;$('hardware-device').innerHTML='<option value="">'+(devices.length?'请选择机械臂串口':'未检测到 USB 串口')+'</option>'+devices.map(device=>`<option value="${escape(device.path)}">${escape(device.label)} · ${escape(device.device)}${device.accessible?'':' · 无读写权限'}${device.owners.length?' · 已占用':''}</option>`).join('');$('hardware-device').value=devices.some(device=>device.path===selected)?selected:devices.length===1?devices[0].path:'';}
  $('hardware-device').disabled=busy||connected;
  const choice=devices.find(device=>device.path===$('hardware-device').value);
  $('hardware-connect').disabled=busy||connected||!choice?.accessible||!!choice?.owners.length;
  $('hardware-connect').textContent=connected?'真机已连接':connection.status==='starting'||data.switching_to==='hardware'?'正在连接…':'一键连接真机';
  $('hardware-disconnect').disabled=pending||!!data.switching_to||!['starting','running'].includes(connection.status);
  $('hardware-simulation').disabled=busy||data.active_target==='fake';
  $('hardware-refresh').disabled=pending;
  $('hardware-workspace').hidden=!connected||data.active_target!=='hardware';
  const labels={stopped:'真机未连接',starting:'正在连接真机',running:'DM 真机已连接',stopping:'正在断开真机',error:'真机连接失败 / 已中断'};
  $('hardware-summary').textContent=data.switching_to?'正在切换控制对象…':labels[connection.status];
  $('hardware-summary').dataset.status=connection.status;
  $('hardware-message').textContent=data.error||connection.error||(connected?'工作空间、动作实验室、视觉与交互已共用此真机连接。':!devices.length?'连接机械臂电源与 USB 适配器后，点击刷新设备。':choice&&!choice.accessible?'当前账户没有此串口的读写权限，请先完成系统设备授权。':choice?.owners.length?'此串口正被其他程序占用。':data.active_target==='fake'?'当前正在使用仿真。点击连接后，网页会停止仿真并切换到真机。':'请选择设备并连接；页面刷新不会自动连接或重连真机。');
  $('hardware-target').textContent=data.active_target==='hardware'?'真机':'本机仿真';
  $('hardware-port').textContent=connection.device||'尚未选择';
  const feedback=data.feedback,fresh=feedback?.joint_age_sec!=null&&feedback.joint_age_sec<1.5&&feedback.status_age_sec!=null&&feedback.status_age_sec<2;
  $('hardware-driver').textContent=fresh?(feedback.control_mode||'未知'):'等待原 DM 驱动反馈';
  $('hardware-enabled').textContent=fresh?(feedback.enabled?'已使能':'已失能'):'—';
  $('hardware-feedback-age').textContent=fresh?`${Math.round(feedback.joint_age_sec*1000)} ms`:'无实时反馈';
  $('hardware-arm-state').textContent=fresh?feedback.state_machine:'—';
  const stages={idle:0,driver:1,feedback:2,ready:3,error:0,stopping:0};
  document.querySelectorAll('[data-hardware-step]').forEach(element=>element.classList.toggle('complete',Number(element.dataset.hardwareStep)<=stages[connection.stage]));
  $('hardware-joints').innerHTML=Array.from({length:6},(_,i)=>{const name=`joint${i+1}`,motor=feedback?.motor_states?.[name],position=fresh?feedback.positions[name]:null,velocity=fresh?feedback.velocities[name]:null;return `<tr><td>J${i+1}</td><td>${Number.isFinite(position)?(position*180/Math.PI).toFixed(2)+'°':'—'}</td><td>${Number.isFinite(velocity)?velocity.toFixed(3):'—'}</td><td>${motor&&motor.age_sec<1.5?motor.torque.toFixed(3):'—'}</td><td>${motor&&motor.age_sec<1.5?motor.status_code:'—'}</td><td>${Number.isFinite(position)?'收到反馈':'等待反馈'}</td></tr>`}).join('');
  $('hardware-errors').textContent=feedback?.error_codes?.length?feedback.error_codes.join(' · '):fresh?'未报告控制器故障':'连接后显示控制器故障信息';
 }
 async function refresh(){try{render(await api('/api/hardware'))}catch(error){$('hardware-message').textContent=error.message}}
 async function act(path,payload={}){
  if(pending)return;pending=true;if(current)render(current);
  try{const result=await api(path,payload);onTargetRequested();render(result);log(path.endsWith('connect')&&!path.endsWith('disconnect')?'已请求连接 DM 真机':path.endsWith('simulation')?'已请求切换到本机仿真':'已请求停止并断开真机');}
  catch(error){notify(error.message);$('hardware-message').textContent=error.message;log(error.message,'error')}
  finally{pending=false;await refresh()}
 }
 $('hardware-connect').onclick=()=>act('/api/hardware/connect',{device:$('hardware-device').value});
 $('hardware-disconnect').onclick=()=>act('/api/hardware/disconnect');
 $('hardware-simulation').onclick=()=>act('/api/hardware/simulation');
 $('hardware-refresh').onclick=refresh;
 $('hardware-device').onchange=()=>{if(current)render(current)};
 $('hardware-advanced').onclick=onAdvanced;
 async function poll(){if(dead)return;if(location.hash==='#hardware')await refresh();timer=setTimeout(poll,700)}
 refresh();poll();
 return {close(){dead=true;clearTimeout(timer)}};
}
