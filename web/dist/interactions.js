const $=id=>document.getElementById(id);
const options=[
 ['gesture','手势动作','识别石头、布和比心，执行原版出拳、八字和爱心。'],
 ['palm','手掌位置跟随','自动进入起手位，张开手掌控制跟随，握拳暂停。'],
 ['voice','本地 Whisper 语音','开启本地模型后，点击录音，说出动作指令。'],
 ['agent','文字与 MCP','输入文字执行原版动作，同时开启本机 MCP 状态服务。']
];

export function createInteractions({getBase,getLive,onState,notify,log,onView}){
 let current=null,stream=null,frameTimer=null,pollTimer=null,uploading=false,ownedVision=null,active=false,recording=null,dead=false,transitioning=false,cameraEpoch=0,voicePending=false,transcribing=false;
 const names={camera:'摄像头识别',...Object.fromEntries(options.map(([id,name])=>[id,name]))};
 $('vision-modules').innerHTML=options.map(([id,title,description])=>`<article class="panel module-card"><span class="tiny-badge">LOCAL</span><h3>${title}</h3><p>${description}</p><div class="feature-status" id="feature-status-${id}">未开启</div><button class="button button-dark" data-feature="${id}">开启${title}</button></article>`).join('');
 async function api(path,options={}){
  const response=await fetch(`${getBase()}${path}`,{cache:'no-store',...options,signal:AbortSignal.timeout(path.endsWith('/audio')?40000:15000)});
  const data=await response.json();if(!response.ok||data.success===false)throw Error(data.message||'功能请求失败');return data;
 }
 function render(state){
  current=state;onState(state);
  for(const [id,name]of options){
   const feature=state.features[id],busy=['starting','running'].includes(feature.status),button=document.querySelector(`[data-feature="${id}"]`);
   button.textContent=busy?`关闭${name}`:`开启${name}`;
   const label=$(`feature-status-${id}`);label.dataset.featureState=feature.status;
   label.textContent=feature.error||({stopped:'未开启',starting:'正在准备…',running:id==='agent'?'已开启 · 本地 MCP 与文字指令就绪':'已开启'}[feature.status]??feature.status);
  }
  if(state.features.voice.status!=='running'&&recording){const task=recording;recording=null;clearTimeout(task.timer);task.processor.disconnect();task.source.disconnect();task.stream.getTracks().forEach(track=>track.stop());task.context.close();$('voice-record-stop').hidden=true;$('voice-record').hidden=false;$('voice-feedback').textContent='语音已关闭，录音已取消';}
  const voice=state.features.voice.status==='running';$('voice-record').disabled=!voice||state.voice_busy||!!recording||voicePending||transcribing||state.control_reserved;
  const agent=state.features.agent.status==='running';$('feature-text').disabled=!agent||state.control_reserved;$('feature-text-send').disabled=!agent||state.control_reserved;
  if(state.vision&&state.frame_sequence>0){
   if(!$('recognition-image').getAttribute('src'))$('recognition-image').src=`${getBase()}/api/features/camera?t=${Date.now()}`;
   $('camera-placeholder').hidden=true;$('recognition-image').hidden=false;$('camera-badge').textContent='MEDIAPIPE · LIVE';
   $('camera-status').textContent=`${names[state.vision]} · 识别中`;
   $('recognition-rate').textContent=state.frame_age_sec<1?'识别画面实时更新':'等待新的识别画面';
  }
  if(state.vision&&!state.frame_sequence){$('camera-status').textContent='正在准备摄像头识别…';$('camera-badge').textContent='STARTING';}
  if(!state.vision&&!transitioning){releaseCamera();$('camera-status').textContent='摄像头已关闭';}
  $('feature-note').textContent=state.features.palm.status==='running'?'手掌跟随已开启：张手跟随，握拳暂停。左侧显示机械臂实际反馈。':'选择功能即可启停；原生 RViz 与摄像头识别画面同时显示。';
 }
 async function cameras(){
  const devices=(await navigator.mediaDevices.enumerateDevices()).filter(device=>device.kind==='videoinput');
  const selected=$('camera-device').value;const select=$('camera-device');select.replaceChildren();
  for(const [i,device]of devices.entries()){const option=document.createElement('option');option.value=device.deviceId;option.textContent=device.label||`摄像头 ${i+1}`;select.append(option)}
  if(devices.some(device=>device.deviceId===selected))select.value=selected;
 }
 function releaseCamera(){
  active=false;cameraEpoch++;clearTimeout(frameTimer);stream?.getTracks().forEach(track=>track.stop());stream=null;
  $('camera-start').hidden=false;$('camera-detail').textContent='点击开启，允许浏览器使用摄像头即可';$('camera').srcObject=null;$('recognition-image').removeAttribute('src');$('recognition-image').hidden=true;$('camera-placeholder').hidden=false;$('camera-stop').disabled=true;$('camera-badge').textContent='CAMERA OFF';ownedVision=null;
 }
 async function capture(){
  if(stream)return;
  if(!navigator.mediaDevices?.getUserMedia)throw Error('当前浏览器无法使用摄像头，请在本机浏览器打开工作台。');
  const device=$('camera-device').value;
  stream=await navigator.mediaDevices.getUserMedia({video:{width:{ideal:640},height:{ideal:480},...(device?{deviceId:{exact:device}}:{})},audio:false});
  if(dead){releaseCamera();throw Error('页面已关闭，摄像头已释放。')}
  $('camera').srcObject=stream;await $('camera').play();await cameras();
  $('camera-detail').textContent='正在加载原项目识别模型…';$('camera-start').hidden=true;
 }
 async function sendFrame(){
  if(!active||dead)return;
  const epoch=cameraEpoch;
  if($('camera').readyState>=2&&!uploading){
   uploading=true;
   try{
    const canvas=$('capture-canvas');canvas.getContext('2d').drawImage($('camera'),0,0,640,480);
    const blob=await new Promise(resolve=>canvas.toBlob(resolve,'image/jpeg',.72));
    if(blob)await api('/api/features/frame',{method:'POST',headers:{'Content-Type':'image/jpeg'},body:blob});
   }catch(error){if(epoch===cameraEpoch){notify(error.message);await stopVision()}return}finally{uploading=false}
  }
  if(epoch===cameraEpoch&&active)frameTimer=setTimeout(sendFrame,33);
 }
 async function stopVision(){
  const feature=ownedVision||current?.vision;
  releaseCamera();$('camera-start').hidden=false;$('camera-detail').textContent='点击开启，允许浏览器使用摄像头即可';
  if(feature)try{render(await api('/api/features/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({feature})}))}catch(error){notify(error.message)}
 }
 async function toggle(feature){
  if(transitioning)return;
  transitioning=true;
  const status=current?.features[feature]?.status;
  try{
   if(['starting','running'].includes(status)){
    if(['camera','gesture','palm'].includes(feature))return await stopVision();
    render(await api('/api/features/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({feature})}));return;
   }
   if(!getLive())throw Error('本机仿真尚未连接，请等待页面自动连接。');
   if(['camera','gesture','palm'].includes(feature)){active=false;cameraEpoch++;clearTimeout(frameTimer);await capture()}
   const requestedEpoch=cameraEpoch;
   const state=await api('/api/features/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({feature,speed:Number($('feature-speed').value)})});
   if(['camera','gesture','palm'].includes(feature)&&(dead||requestedEpoch!==cameraEpoch)){await api('/api/features/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({feature})});return}
   render(state);
   if(['camera','gesture','palm'].includes(feature)){
    ownedVision=feature;active=true;$('camera-stop').disabled=false;
    $('recognition-image').src=`${getBase()}/api/features/camera?t=${Date.now()}`;sendFrame();
   }
   location.hash='vision';onView();log(`已从网页开启${names[feature]}`);
  }catch(error){if(!active){releaseCamera();$('camera-start').hidden=false}notify(error.message);log(error.message,'error')}finally{transitioning=false}
 }
 document.querySelectorAll('[data-feature]').forEach(button=>button.onclick=()=>toggle(button.dataset.feature));
 $('camera-preview').onclick=()=>toggle('camera');$('camera-start').onclick=()=>toggle('gesture');$('camera-stop').onclick=stopVision;
 $('camera-device').onchange=async()=>{if(active){const kind=ownedVision;await stopVision();await toggle(kind)}};
 $('feature-text-form').onsubmit=async event=>{event.preventDefault();try{const result=await api('/api/features/text',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({text:$('feature-text').value})});$('agent-feedback').textContent=`已执行：${result.text}`;log(`文字指令：${result.text}`)}catch(error){notify(error.message)}};
 async function stopRecording(){
  if(!recording)return;
  const task=recording;recording=null;transcribing=true;clearTimeout(task.timer);
  task.processor.disconnect();task.source.disconnect();task.stream.getTracks().forEach(track=>track.stop());await task.context.close();
  $('voice-record-stop').hidden=true;$('voice-record').hidden=false;$('voice-feedback').textContent='正在本机识别…';
  try{
   const length=task.chunks.reduce((sum,chunk)=>sum+chunk.length,0),audio=new Float32Array(length);let offset=0;for(const chunk of task.chunks){audio.set(chunk,offset);offset+=chunk.length}
   const ratio=task.rate/16000,pcm=new Int16Array(Math.floor(length/ratio));
   for(let i=0;i<pcm.length;i++){let value=0,count=0;for(let j=Math.floor(i*ratio);j<Math.min(length,Math.floor((i+1)*ratio));j++){value+=audio[j];count++}pcm[i]=Math.round(Math.max(-1,Math.min(1,value/Math.max(1,count)))*32767)}
   const result=await api('/api/features/audio',{method:'POST',headers:{'Content-Type':'audio/pcm'},body:pcm.buffer});$('voice-feedback').textContent=`识别并执行：${result.text}`;log(`语音指令：${result.text}`);
  }catch(error){$('voice-feedback').textContent=error.message;notify(error.message)}finally{transcribing=false;if(current)render(current)}
 }
 $('voice-record').onclick=async()=>{
  if(voicePending||recording||transcribing)return;
  let microphone=null,context=null;voicePending=true;
  $('voice-record').disabled=true;
  try{
   microphone=await navigator.mediaDevices.getUserMedia({audio:true,video:false});
   context=new AudioContext();await context.resume();
   if(dead||current?.features.voice.status!=='running')throw Error('语音功能已关闭，录音已取消。');
   const source=context.createMediaStreamSource(microphone),processor=context.createScriptProcessor(2048,1,1),mute=context.createGain();mute.gain.value=0;
   const task={context,source,processor,stream:microphone,chunks:[],rate:context.sampleRate};recording=task;
   processor.onaudioprocess=event=>{if(recording===task)task.chunks.push(new Float32Array(event.inputBuffer.getChannelData(0)))};
   source.connect(processor);processor.connect(mute);mute.connect(context.destination);
   task.timer=setTimeout(stopRecording,8000);$('voice-record').hidden=true;$('voice-record-stop').hidden=false;$('voice-feedback').textContent='正在录音，请说出动作…';
  }catch(error){microphone?.getTracks().forEach(track=>track.stop());context?.close();notify(error.message)}finally{voicePending=false;if(current)render(current)}
 };
 $('voice-record-stop').onclick=stopRecording;
 async function poll(){if(dead)return;try{render(await api('/api/features'))}catch{/* Main connection UI reports a service interruption. */}pollTimer=setTimeout(poll,800)}
 poll();
 return {onState:render,onView(){onView()},close(){dead=true;clearTimeout(pollTimer);const feature=ownedVision;releaseCamera();if(feature)fetch(`${getBase()}/api/features/stop`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({feature}),keepalive:true}).catch(()=>{});if(recording){recording.stream.getTracks().forEach(track=>track.stop());recording.context.close();recording=null}}};
}
