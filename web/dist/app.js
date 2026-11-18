import {createHardwarePage} from './hardware.js?v=20261117-manual-6';
import {createInteractions} from './interactions.js?v=20261117-manual-6';
import {forwardKinematics} from './kinematics.js?v=20261117-manual-6';
import {ReleaseScheduler,moveDuration,controlLoopReady} from './manual-control.js?v=20261117-manual-6';
import {actionSpeedLabel,actionBusy} from './action-presets.js?v=20261117-manual-6';
const presetResponse=await fetch('actions.json',{cache:'no-store'});
if(!presetResponse.ok)throw Error('预设动作加载失败');
const presetActions=(await presetResponse.json()).actions;

const $=id=>document.getElementById(id);
const icons={
 arm:'M5 20h14M9 20v-6l7-8M9 14 4 8m10-4 5 5m-2-4 2-2',cube:'m12 3 9 5v8l-9 5-9-5V8l9-5Zm0 9 9-4M12 12 3 8m9 4v9',play:'m8 4 12 8-12 8V4Z',scan:'M8 3H4v4m12-4h4v4M4 17v4h4m12-4v4h-4M8 8h8v8H8Z',network:'M9 3h6v5H9ZM2 17h6v5H2Zm14 0h6v5h-6M12 8v5M5 17v-4h14v4',terminal:'m5 7 5 5-5 5m8 0h6M3 3h18v18H3Z',book:'M3 4h6l3 2 3-2h6v16h-6l-3 2-3-2H3Zm9 2v16',help:'M9 9a3 3 0 1 1 5 2c-2 1-2 2-2 3m0 3v.1M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20',plug:'M8 3v5m8-5v5M5 8h14v3a7 7 0 0 1-14 0V8Zm7 10v4',activity:'M2 12h5l3-8 4 16 3-8h5',target:'M12 8a4 4 0 1 0 0 8 4 4 0 0 0 0-8Zm0-6v4m0 12v4M2 12h4m12 0h4M12 4a8 8 0 1 0 0 16 8 8 0 0 0 0-16',radio:'M12 11v10m-3 0h6M8 5a7 7 0 0 0 0 10m8-10a7 7 0 0 1 0 10M5 2a11 11 0 0 0 0 16M19 2a11 11 0 0 1 0 16m-7-9v2',expand:'M8 3H3v5m13-5h5v5M3 16v5h5m13-5v5h-5',mouse:'M8 2h8a3 3 0 0 1 3 3v10a7 7 0 0 1-14 0V5a3 3 0 0 1 3-3Zm4 0v7',erase:'m16 3 5 5-12 13H5l-4-4L16 3ZM6 12l6 6m-2 3h12',grip:'M8 3v6l-4 5v6h5v-5l3-4 3 4v5h5v-6l-4-5V3M8 6h8',home:'m3 11 9-8 9 8m-16-1v11h14V10M9 21v-8h6v8',power:'M12 2v10M7 5a9 9 0 1 0 10 0',wave:'M5 13V7a2 2 0 0 1 4 0v5-8a2 2 0 0 1 4 0v8-6a2 2 0 0 1 4 0v7-4a2 2 0 0 1 4 0v7c0 4-4 7-8 7-3 0-5-2-7-4l-4-4a2 2 0 0 1 3-3l3 3',music:'M9 18V5l12-3v13M9 5l12-3M9 18a3 3 0 1 1-3-3c2 0 3 1 3 3Zm12-3a3 3 0 1 1-3-3c2 0 3 1 3 3Z',route:'M5 5h14v14H5V5Zm0 0 4 4M5 5v5m14 9-4-4m4 4v-5',info:'M12 11v6m0-10v.1M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20',camera:'M3 6h4l2-3h6l2 3h4v14H3V6Zm9 3a4 4 0 1 0 0 8 4 4 0 0 0 0-8',download:'M12 3v12m-5-5 5 5 5-5M3 16v5h18v-5',stop:'M5 5h14v14H5Z',code:'m8 5-7 7 7 7m8-14 7 7-7 7m-3-16-2 18'};
function drawIcons(root=document){root.querySelectorAll('[data-icon]').forEach(el=>{el.innerHTML=`<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="${icons[el.dataset.icon]||icons.cube}"/></svg>`})}
drawIcons();
let toastTimer;function toast(message){$('toast').textContent=message;$('toast').classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('show'),4200)}
const escapeHtml=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let jointStream=null,viewerLoading=null;
let visualMode=new URLSearchParams(location.search).get('view')==='browser'?'browser':'rviz',rvizBase='',rvizPollTimer=null,rvizPollGeneration=0,rvizFrameKey='',rvizConnected=false,lastPosePaint=0;
let logs=[],viewer,manifest,model='dm',current={},target={},gripper=45,animation=null,live=null,pollTimer=null,interactions=null,hardwarePage=null,featureState=null,pollGeneration=0,loadingModel=false;
const rad=n=>n*Math.PI/180,deg=n=>n*180/Math.PI,clamp=(n,a,b)=>Math.min(b,Math.max(a,n));
const homePose=()=>[0,-1.25,-1.0,0,.35,0];
let manualAcceptedSequence=0,manualReport='',manualRequests=0,actionReport='',actionRequesting=false;
const releaseScheduler=new ReleaseScheduler((joint,value)=>commitReleasedTarget(joint,value));
const jointMap=values=>Object.fromEntries(values.map((v,i)=>[`joint${i+1}`,v]));
function log(message,level='info'){logs.unshift({time:new Date().toISOString(),level,message});logs=logs.slice(0,300);renderLogs()}
function logRow(l){return `<div class="log-line"><time>${new Date(l.time).toLocaleTimeString('zh-CN',{hour12:false})}</time><span class="log-level ${l.level}">${l.level.toUpperCase()}</span><span class="log-message">${escapeHtml(l.message)}</span></div>`}
function renderLogs(){const query=$('log-search').value.toLowerCase(),filter=$('log-filter').value;const filtered=logs.filter(l=>(filter==='all'||l.level===filter)&&l.message.toLowerCase().includes(query));$('full-logs').innerHTML=filtered.length?filtered.map(logRow).join(''):'<div class="empty-state">没有符合条件的日志</div>';$('activity-lines').innerHTML=logs.slice(0,2).map(logRow).join('');$('log-count').textContent=logs.length;$('log-total').textContent=`${filtered.length} 条记录`;}
const pages={hardware:['连接真机，<span>沿用熟悉的操作。</span>','查看串口、控制器和六轴反馈，连接后在原有页面操作机械臂。'],workspace:['让每一次运动，<span>清晰可见。</span>','从三维姿态到实时状态，在一个工作空间探索你的机械臂。'],actions:['从一个动作，<span>发现更多可能。</span>','问候、舞蹈、轨迹与抓取，把项目能力变成可感知的体验。'],vision:['让机械臂，<span>看见并回应。</span>','连接视觉、手势与语音，探索更自然的人机交互。'],topology:['每一个节点，<span>都有迹可循。</span>','从感知与交互，到规划与控制，读懂整个 ROS 2 系统。'],logs:['运行的每一步，<span>都留有记录。</span>','查看状态与操作记录，让调试更清楚。'],guide:['把你的工作空间，<span>连接起来。</span>','从独立演示到 ROS 2 实时反馈，循序开始。']};
function navigate(){let page=location.hash.slice(1)||'workspace';if(!pages[page])page='workspace';document.querySelectorAll('.page').forEach(el=>el.classList.toggle('active',el.id===`page-${page}`));document.querySelectorAll('[data-nav]').forEach(el=>el.classList.toggle('active',el.dataset.nav===page));$('page-title').innerHTML=pages[page][0];$('page-subtitle').textContent=pages[page][1];if(page==='workspace'||page==='vision'){const slot=$(`${page}-visual-slot`),panel=$('robot-visual-panel');if(panel.parentElement!==slot)slot.appendChild(panel);requestAnimationFrame(()=>viewer?.resize());if(page==='vision'&&interactions&&visualMode!=='rviz')chooseRvizVisual().catch(error=>toast(error.message));}}
addEventListener('hashchange',navigate);navigate();
function updateJointFeedback(){for(let i=1;i<=6;i++){const el=$(`actual-${i}`);if(el)el.textContent=`当前 ${deg(current[`joint${i}`]??0).toFixed(1)}°`}}
function onPose(p){
 const now=performance.now();if(now-lastPosePaint<100)return;lastPosePaint=now;updateJointFeedback();$('stat-position').textContent=[p.x,p.y,p.z].map(n=>(n*1000).toFixed(0)).join(' / ');
 for(const [key,value] of Object.entries(p)){let el=$(`pose-${key}`);if(!el){$('pose-values').innerHTML=Object.keys(p).map(k=>`<div><small>${k.toUpperCase()}</small><b id="pose-${k}"></b></div>`).join('');el=$(`pose-${key}`)}el.textContent=(['x','y','z'].includes(key)?value*1000:deg(value)).toFixed(1)}
}
function updateNumericPose(){if(manifest)onPose(forwardKinematics(manifest,current))}
function fillRange(input){input.style.setProperty('--fill',`${100*(Number(input.value)-Number(input.min))/(Number(input.max)-Number(input.min))}%`)}
function updateSliders(){for(let i=1;i<=6;i++){const input=$(`joint-${i}`),number=$(`angle-${i}`);if(input){input.value=deg(target[`joint${i}`]);number.value=deg(target[`joint${i}`]).toFixed(1);fillRange(input)}}$('gripper').value=gripper;$('gripper-value').textContent=Math.round(gripper);fillRange($('gripper'))}
function makeControls(){
 const names=['底座旋转','肩部俯仰','肘部俯仰','腕部旋转','腕部俯仰','末端旋转'];
 $('joint-controls').innerHTML=Array.from({length:6},(_,i)=>{const j=manifest.joints.find(j=>j.name===`joint${i+1}`);return `<div class="joint-control"><div class="joint-label"><span><em>J${i+1}</em>${names[i]}</span><b><input id="angle-${i+1}" aria-label="J${i+1} 目标角度" type="number" min="${Math.ceil(deg(j.lower)*10)/10}" max="${Math.floor(deg(j.upper)*10)/10}" step="0.1"><small>°</small></b></div><input id="joint-${i+1}" aria-label="J${i+1} ${names[i]}" type="range" min="${Math.ceil(deg(j.lower)*10)/10}" max="${Math.floor(deg(j.upper)*10)/10}" step="0.1"><div class="range-labels"><span>${deg(j.lower).toFixed(0)}°</span><span id="actual-${i+1}" class="actual-angle">当前 —</span><span>${deg(j.upper).toFixed(0)}°</span></div></div>`}).join('');
 for(let i=1;i<=6;i++){
  const name=`joint${i}`,joint=manifest.joints.find(j=>j.name===name),range=$(`joint-${i}`),number=$(`angle-${i}`);
  const change=e=>{const typed=e.target.value;const v=e.target.type==='number'?e.target.valueAsNumber:Number(typed);if(!Number.isFinite(v))return;if(animation)stopAnimation(false);target[name]=clamp(rad(v),joint.lower,joint.upper);targetDirty=true;releaseScheduler.edit(name,target[name]);updateSliders();if(e.target.type==='number'&&e.type==='input')e.target.value=typed;$('control-note-text').textContent='目标设定中，松开滑块后按设定速度移动';};
  range.addEventListener('pointerdown',()=>releaseScheduler.begin(name));
  range.addEventListener('input',change);
  range.addEventListener('pointerup',()=>releaseScheduler.release(name));
  range.addEventListener('change',()=>releaseScheduler.schedule(name));
  range.addEventListener('pointercancel',()=>{releaseScheduler.cancelJoint(name);target[name]=current[name];updateSliders()});
  number.addEventListener('input',change);number.addEventListener('change',()=>releaseScheduler.schedule(name));
  number.addEventListener('keydown',e=>{if(e.key==='Enter'){e.preventDefault();releaseScheduler.schedule(name);number.blur()}});
 }
 updateSliders();updateJointFeedback();
}
async function changeModel(next){
 if(next!=='dm')throw Error('当前工作台仅支持 B601 DM');
 if(live){toast('ROS 接入时使用桥接服务报告的型号');return}
 releaseScheduler.cancel();
 if(loadingModel)return;loadingModel=true;stopAnimation(false);$('model-loading').hidden=false;document.querySelectorAll('[data-model]').forEach(b=>b.disabled=true);
 try{const response=await fetch(`assets/models/${next}.json`);if(!response.ok)throw Error('模型参数加载失败');const nextManifest=await response.json();if(viewer)await viewer.load(next);manifest=nextManifest;model=next;current=jointMap(homePose(model));target={...current};gripper=45;makeControls();viewer?.update(current,gripper,{record:false});viewer?.resetCamera();updateNumericPose();$('stat-model').textContent=`B601 · ${model.toUpperCase()}`;$('viewer-model').textContent=`B601_${model.toUpperCase()}`;document.querySelectorAll('[data-model]').forEach(b=>b.classList.toggle('selected',b.dataset.model===model));log(`已加载 B601 ${model.toUpperCase()} 真实模型 · 6 个旋转关节 · URDF 限位同步`,'success');$('model-loading').hidden=true;
 }catch(error){$('model-loading').querySelector('span').textContent=`模型加载失败：${error.message}`;log(error.message,'error');toast('三维模型加载失败，请刷新重试')}finally{loadingModel=false;document.querySelectorAll('[data-model]').forEach(b=>b.disabled=false)}
}
await changeModel('dm');
showRvizPlaceholder('正在等待本机 ROS 2…','仿真就绪后自动打开真正的 RViz 窗口');

document.querySelectorAll('[data-model]').forEach(b=>b.onclick=()=>changeModel(b.dataset.model));
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{viewer?.resetCamera(b.dataset.view);document.querySelectorAll('[data-view]').forEach(el=>el.classList.toggle('selected',el===b))});
$('reset-camera').onclick=()=>{viewer?.resetCamera();document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('selected',b.dataset.view==='perspective'))};
for(const [id,key]of [['show-grid','grid'],['show-axes','axes']])$(id).onchange=e=>{if(viewer){viewer[key].visible=e.target.checked;viewer.invalidate()}};
$('gripper').oninput=e=>{gripper=Number(e.target.value);$('gripper-value').textContent=gripper;fillRange(e.target);releaseScheduler.edit('gripper',gripper);targetDirty=true;};
$('gripper').addEventListener('pointerdown',()=>releaseScheduler.begin('gripper'));
$('gripper').addEventListener('pointerup',()=>releaseScheduler.release('gripper'));
$('gripper').addEventListener('change',()=>releaseScheduler.schedule('gripper'));
$('gripper').addEventListener('pointercancel',()=>releaseScheduler.cancelJoint('gripper'));
$('joint-tab').onclick=()=>{ $('joint-controls').hidden=false;$('pose-panel').hidden=true;$('joint-tab').classList.add('selected');$('pose-tab').classList.remove('selected')};
$('pose-tab').onclick=()=>{$('joint-controls').hidden=true;$('pose-panel').hidden=false;$('pose-tab').classList.add('selected');$('joint-tab').classList.remove('selected')};
function stopAnimation(report=true){if(animation){cancelAnimationFrame(animation.frame);animation=null;$('task-progress').hidden=true;$('stat-status').textContent='演示就绪';$('execute').disabled=false;if(report)log('已停止网页演示，模型保持当前姿态');target={...current};updateSliders()}}
function animateFrames(label,frames,seconds,original=false){
 if(!viewer||visualMode!=='browser'){chooseBrowserVisual().then(()=>animateFrames(label,frames,seconds,original)).catch(error=>toast(error.message));return}if(!manifest)return;stopAnimation(false);location.hash='workspace';const all=[{joints:{...current},gripper},...frames.map(frame=>({joints:Object.fromEntries(Object.entries(jointMap(frame.joints)).map(([name,v])=>{const j=manifest.joints.find(j=>j.name===name);return[name,clamp(v,j.lower,j.upper)]})),gripper:frame.gripper??gripper,duration:frame.duration??seconds/frames.length}))];
 const ends=[];let total=0;for(const frame of all.slice(1)){total+=frame.duration;ends.push(total)}seconds=total;
 const start=performance.now();let lastAnimationUi=0;animation={frame:null};$('task-progress').hidden=false;$('task-name').textContent=label;$('stat-status').textContent='演示运行中';$('execute').disabled=true;log(`${label} · 网页模型演示 · ${seconds.toFixed(1)} s`);
 const tick=now=>{if(!animation)return;const progress=clamp((now-start)/(seconds*1000),0,1);const elapsed=progress*seconds,segment=Math.min(all.length-2,(ends.findIndex(end=>end>=elapsed)<0?all.length-2:ends.findIndex(end=>end>=elapsed))),local=clamp((elapsed-(segment?ends[segment-1]:0))/(ends[segment]-(segment?ends[segment-1]:0)),0,1),t=original?local:local*local*(3-2*local),a=all[segment],b=all[segment+1];for(let i=1;i<=6;i++)current[`joint${i}`]=a.joints[`joint${i}`]+(b.joints[`joint${i}`]-a.joints[`joint${i}`])*t;gripper=a.gripper+(b.gripper-a.gripper)*t;target={...current};viewer.update(current,gripper);if(now-lastAnimationUi>100||progress===1){lastAnimationUi=now;updateSliders();$('task-bar').style.width=`${progress*100}%`;$('task-percent').textContent=`${Math.round(progress*100)}%`;}if(progress<1)animation.frame=requestAnimationFrame(tick);else{animation=null;$('task-progress').hidden=true;$('stat-status').textContent='演示就绪';$('execute').disabled=false;log(`${label}完成 · 演示轨迹已更新`,'success')}};animation.frame=requestAnimationFrame(tick);
}
const capabilities=[
 {id:'greeting',title:'三位置问候',icon:'wave',tone:'mint',tag:'GREETING',description:'原版左中右点头、夹爪联动与挥手告别，结束后回零。'},
 {id:'dance',title:'机械臂舞蹈',icon:'music',tone:'peach',tag:'DANCE',description:'原版机械街舞两圈、空间滑步、高举指天与回零。默认与 dance 命令一致。'},
 {id:'rock',title:'出拳',icon:'arm',tone:'sand',tag:'rock_push',description:'与 punch 命令一致：原版出拳轨迹，平滑接入，完成后回零。'},
 {id:'paper',title:'画八字',icon:'route',tone:'lavender',tag:'paper_figure8',description:'与 figure8 命令一致：原版大八字与手腕扭转，完成后回零。'},
 {id:'heart',title:'画爱心',icon:'wave',tone:'mint',tag:'heart_draw',description:'与 heart 命令一致：完整回放原项目画爱心轨迹，完成后回零。'},
 {id:'circle',title:'画圆',icon:'route',tone:'peach',tag:'paper_circle',description:'与 circle 命令一致：完整回放原项目画圆轨迹，完成后回零。'},
 {id:'home',title:'安全回位',icon:'home',tone:'lavender',tag:'SAFE HOME',description:'连接 ROS 2 后调用驱动原有的 safe_home 服务。'}
];
for(const id of ['action-speed','workspace-action-speed','feature-speed'])$(id).onchange=()=>{const value=$(id).value;for(const peer of ['action-speed','workspace-action-speed','feature-speed'])$(peer).value=value;log(`后续动作倍率：${actionSpeedLabel(value)} · 保持原动作与轨迹`)};
$('action-grid').innerHTML=capabilities.map(c=>`<article class="panel action-card"><span class="feature-icon ${c.tone}" data-icon="${c.icon}"></span><h3>${c.title}</h3><p>${c.description}</p><div class="card-footer"><code>${c.tag}</code><button class="button" data-demo="${c.id}"><i data-icon="play"></i>执行动作</button></div></article>`).join('');
function observeAction(){location.hash='workspace';navigate();$('page-workspace').scrollIntoView({block:'start'});}
async function playDemo(id){
 if(hardwareTarget()&&!live)return toast('真机尚未连接，请先到真机连接页连接。');
 releaseScheduler.cancel();
 if(id==='home'&&live){try{await sendCommand('safe_home');observeAction()}catch(error){toast(error.message)}return}
 const preset=presetActions[id];
 if(!preset&&id!=='home')return toast('未知动作');
 if(live){
  const active=live;
  if(actionRequesting||actionBusy(active.state))return toast('动作正在执行，请等待完成或先停止');
  actionRequesting=true;updateLiveState(active.state);
  try{const result=await sendCommand('run_action',{action:id,speed:Number($('action-speed').value)});if(live===active){targetDirty=false;updateLiveState({...active.state,action_motion:result.action_motion});observeAction()}}
  catch(error){toast(error.message)}finally{actionRequesting=false;if(live===active&&active.state)updateLiveState(active.state)}
  return;
 }
 if(!manifest)return toast('请等待模型加载完成');
 if(id==='home'){const base=homePose();animateFrames('展示姿态复位',[{joints:base,gripper:45}],moveDuration(Math.max(...Object.values(current).map((value,i)=>Math.abs(value-base[i]))),Number($('motion-speed').value)));return}
 if(actionRequesting)return;
 actionRequesting=true;
 try{
  const response=await fetch('/api/actions/preview',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:id,speed:Number($('action-speed').value),positions:Array.from({length:6},(_,i)=>current[`joint${i+1}`]),opening_mm:gripper})});
  const program=await response.json();if(!response.ok)throw Error(program.message||'原版动作加载失败');
  if(live)throw Error('连接模式已改变，请重新选择动作');
  animateFrames(`${program.title} · ${actionSpeedLabel($('action-speed').value)}`,program.frames,program.duration,true);
 }catch(error){toast(error.message)}finally{actionRequesting=false}
}
document.addEventListener('click',e=>{const button=e.target.closest('[data-demo]');if(button)playDemo(button.dataset.demo);const copy=e.target.closest('[data-copy]');if(copy)copyText(copy.dataset.copy)});
async function copyText(text){try{await navigator.clipboard.writeText(text);toast('内容已复制')}catch{toast('剪贴板不可用，请手动复制命令')}}
$('stop-demo').onclick=()=>{if(live&&actionBusy(live.state))sendCommand('stop_action').catch(error=>toast(error.message));else stopAnimation()};
$('execute').onclick=()=>{};
$('home').onclick=()=>{releaseScheduler.cancel();if(live)sendCommand('safe_home').catch(e=>toast(e.message));else playDemo('home')};
$('enable').onclick=()=>{if(live)sendCommand(live.state?.enabled?'disable':'enable').catch(e=>toast(e.message));else{toast('浏览器演示无需使能。连接本地 ROS 2 后可控制驱动使能。');log('演示模式：使能按钮未连接硬件')}};
$('safety-stop').onclick=()=>{releaseScheduler.cancel();stopAnimation(false);if(live)sendCommand('safety_stop').then(()=>toast('已请求软件停止；请检查机械臂现场状态')).catch(e=>toast(e.message));else if(hardwareTarget()){fetchJson('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:'safety_stop'})}).then(()=>toast('已请求真机软件停止')).catch(error=>toast(error.message))}else{log('网页演示已停止，模型保持当前姿态');toast('演示已停止')}};

const nodes=[
 {title:'视觉感知',pkg:'rebotarm_vision',text:'USB 摄像头与 MediaPipe 提供手势命令和手掌坐标。手掌跟随节点读取位置并生成受限位约束的运动。'},
 {title:'本地语音',pkg:'rebotarm_voice',text:'本地 Whisper 将语音转为文字，发布 voice/transcript 与 voice/status，并通过 action/play 触发预计算动作。'},
 {title:'文本与 MCP Agent',pkg:'rebotarm_agent',text:'将高层工具映射到已有 ROS 2 topic、service 和 action。模型服务在本地配置；本网站不保存 API Key。'},
 {title:'动作与规划',pkg:'MoveIt 2 / gesture_actions',text:'MoveIt 2 进行逆解、碰撞检测与规划。手势动作分发器将 rock、paper、heart 映射到轨迹回放器。'},
 {title:'ROS 2 控制接口',pkg:'rebotarm_msgs',text:'统一关节状态、机械臂状态、夹爪、服务与 FollowJointTrajectory action。网页桥接直接复用这些类型。'},
 {title:'控制与执行',pkg:'rebotarmcontroller',text:'DM 机械臂驱动。含安全锁止、失能、回位、单关节与轨迹控制。'},
 {title:'仿真与可视化',pkg:'RViz / rebotarm_mujoco_rs',text:'Fake 驱动提供无硬件测试；MuJoCo 执行物理仿真、颜色块检测、抓取验证与动作录制。网页三维视图为 URDF 可视化。'}
];
$('topology-graph').innerHTML=`<div class="graph-col">${nodes.slice(0,3).map((n,i)=>`<button class="graph-node" data-node="${i}"><b>${n.title}</b><small>${n.pkg}</small></button>`).join('')}</div><div class="graph-arrow">→</div><div class="graph-col">${nodes.slice(3,5).map((n,i)=>`<button class="graph-node" data-node="${i+3}"><b>${n.title}</b><small>${n.pkg}</small></button>`).join('')}</div><div class="graph-arrow">→</div><div class="graph-col">${nodes.slice(5).map((n,i)=>`<button class="graph-node" data-node="${i+5}"><b>${n.title}</b><small>${n.pkg}</small></button>`).join('')}</div>`;
function selectNode(i){const n=nodes[i];$('node-detail').innerHTML=`<strong>${n.pkg}</strong>${n.text}`;document.querySelectorAll('[data-node]').forEach(el=>el.classList.toggle('selected',Number(el.dataset.node)===i))}document.querySelectorAll('[data-node]').forEach(el=>el.onclick=()=>selectNode(Number(el.dataset.node)));selectNode(4);
const topics=[['/{ns}/joint_states','sensor_msgs/JointState','六轴位置、速度与力矩'],['/{ns}/arm_status','rebotarm_msgs/ArmStatus','使能状态、安全锁止与错误码'],['/{ns}/follow_joint_trajectory','control_msgs/FollowJointTrajectory','标准关节轨迹 Action'],['/{ns}/gripper/set','rebotarm_msgs/SetGripper','夹爪位置与力矩上限'],['/{ns}/safety_stop','std_srvs/Trigger','软件停止并锁止后续运动'],['/{ns}/gesture_cmd','std_msgs/String','rock / paper / heart / none'],['/{ns}/voice/transcript','std_msgs/String','本地 Whisper 识别字幕']];
$('topic-table').innerHTML=`<table><thead><tr><th>Topic / Service / Action</th><th>ROS 类型</th><th>用途</th></tr></thead><tbody>${topics.map(r=>`<tr><td><code>${r[0]}</code></td><td><code>${r[1]}</code></td><td>${r[2]}</td></tr>`).join('')}</tbody></table>`;
const guides=[
 ['01 / OPEN & RUN','打开网页，自动启动仿真','本机安装网页入口后，双击桌面的 reBot Arm 仿真工作台，或直接打开下方地址。页面会自动启动 Fake 驱动并连接实时反馈。','http://127.0.0.1:8088','网页上可停止、重新启动仿真；刷新和多标签页共用同一个实例。网页、仿真与可视化均在本机运行。'],
 ['02 / CONNECT ROS','连接 ROS 2 反馈','先加载 ROS 2 与工作空间环境。在一个终端启动 Fake 驱动，另一个终端启动网页桥接。','source /opt/ros/jazzy/setup.bash\nsource install/setup.bash\nros2 launch rebotarm_bringup fake_dm_bringup.launch.py\n\n# 另一个终端同样 source 后运行\n/usr/bin/python3 web/server.py --ros','使用系统 Python，避免 Conda Python 无法导入 rclpy。桥接默认只读。'],
 ['03 / CONTROL','开启仿真控制','在本地仿真验证后，可在启动桥接时显式开放使能、回位、关节轨迹与夹爪控制。','/usr/bin/python3 web/server.py --ros --allow-motion','网站不会自动使能。发送运动前需有新鲜的状态、关节反馈和使能状态。软件停止不等同于硬件急停。'],
 ['04 / EXPLORE','启动交互与仿真','先启动控制器，再按需启动功能节点。动作实验室的网页预览可以独立播放。','ros2 launch rebotarm_greeting greeting.launch.py\nros2 launch rebotarm_dance dance.launch.py\nros2 launch rebotarm_vision vision.launch.py\nros2 launch rebotarm_voice voice_control.launch.py\nros2 launch rebotarm_moveit_config demo.launch.py','Whisper、MediaPipe、MoveIt 与 MuJoCo 的依赖和模型配置参见仓库各包 README。']
];
$('guide-content').innerHTML=guides.map(([step,title,description,command,note])=>`<article class="panel guide-card"><div class="step">${step}</div><h3>${title}</h3><p>${description}</p><div class="code-block"><button class="copy-command" data-copy="${escapeHtml(command)}">复制</button><pre>${escapeHtml(command)}</pre></div><p class="guide-note">${note}</p></article>`).join('');
$('help-open').onclick=()=>location.hash='guide';
$('log-search').oninput=renderLogs;$('log-filter').onchange=renderLogs;$('clear-logs').onclick=()=>{logs=[];renderLogs();toast('当前会话日志已清空')};
$('export-logs').onclick=()=>{const blob=new Blob([JSON.stringify(logs,null,2)],{type:'application/json'}),url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;a.download=`rebotarm-logs-${new Date().toISOString().slice(0,10)}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};

let targetDirty=false,commandBusy=false;
$('joint-controls').addEventListener('input',()=>targetDirty=true);$('joint-controls').addEventListener('change',()=>targetDirty=true);$('gripper').addEventListener('input',()=>targetDirty=true);
function openConnection(advanced=false){if(managedRuntime?.managed&&advanced!==true){location.hash='hardware';navigate();return}const isLocal=['127.0.0.1','localhost'].includes(location.hostname);$('prefer-same-origin').checked=isLocal;$('bridge-url').value=isLocal?location.origin:'http://127.0.0.1:8088';$('bridge-url').disabled=isLocal;$('connection-result').textContent='';$('connection-dialog').showModal()}
$('connection-open').onclick=openConnection;$('connect-button').onclick=openConnection;$('connection-close').onclick=()=>$('connection-dialog').close();$('prefer-same-origin').onchange=e=>$('bridge-url').disabled=e.target.checked;
async function fetchJson(url,options={}){const response=await fetch(url,{cache:'no-store',...options,signal:AbortSignal.timeout(options.method==='POST'?40000:4000)});let data;try{data=await response.json()}catch{throw Error('当前地址没有可用的 ROS 2 桥接 API，请先启动 web/server.py --ros')}if(!response.ok||data.success===false)throw Error(data.message||`HTTP ${response.status}`);return data}
function freshState(state){return state&&state.joint_age_sec!==null&&state.joint_age_sec<1.5&&state.status_age_sec!==null&&state.status_age_sec<2}
function updateLiveState(state){if(!live)return;live.state=state;const fresh=freshState(state),locked=state.state_machine==='SAFETY_STOPPED';$('stat-status').textContent=fresh?(state.state_machine||'反馈已连接'):'反馈过期 / 等待';$('stat-source').textContent=`/${state.namespace}`;$('data-age').textContent=state.joint_age_sec===null?'NO DATA':`${state.joint_age_sec.toFixed(1)}s`;$('status-badge').textContent=fresh?'LIVE':'STALE';$('status-badge').style.color=fresh?'':'#b58a73';
 const positions=state.positions||{};if(fresh&&Array.from({length:6},(_,i)=>Number.isFinite(positions[`joint${i+1}`])).every(Boolean)){if(!jointStream?.ready)current=Object.fromEntries(Array.from({length:6},(_,i)=>[`joint${i+1}`,positions[`joint${i+1}`]]));if(!targetDirty&&!releaseScheduler.busy){target={...current};gripper=state.gripper_opening_mm??gripper;updateSliders()}if(visualMode==='browser'&&!jointStream?.ready)viewer?.update(current,state.gripper_opening_mm??gripper);if(!jointStream?.ready)updateNumericPose()}
 $('execute').disabled=!state.motion_allowed||!fresh||!state.enabled||locked||commandBusy;$('home').disabled=!state.motion_allowed||!fresh||locked||commandBusy||actionBusy(state)||actionRequesting;$('enable').disabled=!state.motion_allowed||(!fresh&&!state.enabled)||commandBusy;$('enable').innerHTML=`<i data-icon="power"></i>${state.enabled?'失能控制':'使能控制'}`;drawIcons($('enable'));
 $('control-note-text').textContent=!state.motion_allowed?'只读连接 · 服务端未开放运动':!fresh?'状态过期，运动控制已禁用':locked?'安全锁止中，请通过现场流程解除':targetDirty?'目标设定中，松开滑块后自动移动':'松开滑块后自动移动 · 时间自动计算';
 updateManualState(state);
 $('gesture-feedback').textContent=featureState?.vision==='camera'?'识别结果见画面标注':featureState?.vision==='palm'?'张手跟随 · 握拳暂停':({rock:'石头 · 出拳',paper:'布 · 画八字',heart:'比心 · 画爱心',none:'未识别到动作手势'})[state.gesture]||'等待识别';
}
async function connect(base,{silent=false}={}){
 releaseScheduler.cancel();manualAcceptedSequence=0;manualReport='';
 const state=await fetchJson(`${base}/api/state`);if(state.mode!=='ros')throw Error('桥接正在演示模式，请使用 --ros 参数重新启动');if(state.model!=='dm')throw Error('当前工作台仅支持 B601 DM，无法连接其他型号');
 stopAnimation(false);if(live&&state.model!==model)disconnect(false);if(state.model!==model){await changeModel(state.model);if(state.model!==model)throw Error('模型加载失败，未连接控制器')}
 clearTimeout(pollTimer);const generation=++pollGeneration;live={base,state,namespace:state.namespace};targetDirty=false;$('sidebar-mode').textContent='ROS 2 已连接';$('sidebar-detail').textContent=state.motion_allowed?'运动接口已开放':'只读反馈模式';$('mode-label').textContent='ROS 2 实时反馈';$('viewer-mode').textContent='实时反馈';$('footer-mode').textContent='ROS 2 CONNECTED';$('execute').querySelector('span').textContent='发送关节与夹爪目标';$('control-description').textContent='拖动设目标，松开后按速度移动';document.querySelectorAll('[data-model]').forEach(b=>b.disabled=true);updateLiveState(state);
 if(visualMode==='rviz')ensureRviz(base);else startJointStream();
 if(!silent){log(`已连接 ROS 2 · /${state.namespace} · ${state.motion_allowed?'运动接口开放':'只读模式'}`,'success');toast('已连接 ROS 2 桥接')}
 const poll=async()=>{if(!live||generation!==pollGeneration)return;try{const next=await fetchJson(`${base}/api/state`);if(!live||generation!==pollGeneration)return;if(next.mode!=='ros'||next.model!==model||next.namespace!==live.namespace){disconnect(false);log('桥接模式或型号发生变化，已断开连接','error');toast('桥接配置已变化，请重新连接');return}updateLiveState(next)}catch(error){if(!live||generation!==pollGeneration)return;$('stat-status').textContent='连接中断';$('data-age').textContent='OFFLINE';$('execute').disabled=true;$('home').disabled=true;$('enable').disabled=true;$('control-note-text').textContent='桥接连接中断，运动控制已禁用';live.state=null;}if(live&&generation===pollGeneration)pollTimer=setTimeout(poll,300)};pollTimer=setTimeout(poll,300);
}
function disconnect(report=true){if(actionBusy(live?.state)){$('task-progress').hidden=true;log('已断开显示；已提交的 ROS 动作仍由驱动执行，可重新连接查看或停止。')}stopJointStream();releaseScheduler.cancel();clearTimeout(pollTimer);++pollGeneration;live=null;targetDirty=false;current={...target};$('sidebar-mode').textContent='浏览器演示';$('sidebar-detail').textContent='无需硬件，即刻探索';$('mode-label').textContent='浏览器演示';$('viewer-mode').textContent='交互演示';$('stat-status').textContent='演示就绪';$('status-badge').textContent='SIM';$('stat-source').textContent='本地交互模型';$('data-age').textContent='DEMO';$('footer-mode').textContent='BROWSER SIMULATION';$('execute').querySelector('span').textContent='运行演示';$('control-description').textContent='拖动设目标，松开后按速度移动';$('control-note-text').textContent='松开滑块后自动移动 · 浏览器演示';for(const id of ['execute','home','enable'])$(id).disabled=false;document.querySelectorAll('#joint-controls input,#gripper').forEach(input=>input.disabled=false);$('enable').innerHTML='<i data-icon="power"></i>使能控制';drawIcons($('enable'));document.querySelectorAll('[data-model]').forEach(b=>b.disabled=false);document.querySelectorAll('[data-demo]').forEach(b=>b.disabled=false);$('action-mode-note').textContent='网页预览：使用原项目动作与轨迹。连接仿真或已开放控制的 DM 驱动后，可执行相同动作。';$('disconnect').disabled=false;$('runtime-preview').disabled=false;$('gesture-feedback').textContent='等待本地 ROS 2 连接';$('voice-feedback').textContent='等待本地 ROS 2 连接';updateTargetAvailability();if(report){log('已断开 ROS 2 显示');toast(hardwareTarget()?'已断开真机显示':'已返回浏览器演示')}}
$('disconnect').onclick=()=>{if(managedRuntime)wantManagedLive=false;disconnect();$('connection-dialog').close()};
$('connection-form').onsubmit=async e=>{e.preventDefault();$('connection-submit').disabled=true;$('connection-result').textContent='正在检查桥接服务…';try{const base=$('prefer-same-origin').checked?'':new URL($('bridge-url').value).origin;if(base&&!['http:','https:'].includes(new URL(base).protocol))throw Error('仅支持 HTTP / HTTPS 桥接');await connect(base);$('connection-dialog').close()}catch(error){$('connection-result').textContent=`连接失败：${error.message}。如果在在线网站中访问，请改用本地地址 http://127.0.0.1:8088。`;log(`ROS 2 连接失败：${error.message}`,'error')}finally{$('connection-submit').disabled=false}};
async function sendCommand(command,args={}){
 if(!live)throw Error('未连接 ROS 2');const active=live;const acceptedOnly=['move_joint','run_action','stop_action'].includes(command);if(commandBusy&&!['safety_stop','stop_action'].includes(command))throw Error('上一条命令尚未完成');const isStop=['safety_stop','stop_action'].includes(command);if(!isStop&&!acceptedOnly){commandBusy=true;if(active.state)updateLiveState(active.state)}
 try{log(`ROS 请求：${command}`);const result=await fetchJson(`${active.base}/api/command`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command,...args})});if(live!==active)return result;if(!acceptedOnly)targetDirty=false;log(`${command}：${result.message||'完成'}`,'success');return result}catch(error){log(`${command} 失败：${error.message}`,'error');throw error}finally{if(!isStop&&!acceptedOnly)commandBusy=false;if(live===active&&active.state)updateLiveState(active.state)}
}
let managedRuntime=null,runtimeTimer=null,connectingManaged=false,wantManagedLive=true;
drawIcons();renderLogs();
initializeLocalRuntime();
addEventListener('pagehide',()=>{stopJointStream();interactions?.close();hardwarePage?.close();clearTimeout(pollTimer);clearTimeout(runtimeTimer);clearTimeout(rvizPollTimer);releaseScheduler.cancel();viewer?.destroy()});

function hardwareTarget(){return managedRuntime?.kind==='hardware'||managedRuntime?.switching_to==='hardware'}
function updateTargetAvailability(){
 if(!hardwareTarget()||live)return;
 releaseScheduler.cancel();
 for(const id of ['execute','home','enable'])$(id).disabled=true;
 document.querySelectorAll('#joint-controls input,#gripper,[data-demo]').forEach(input=>input.disabled=true);
 $('mode-label').textContent=managedRuntime?.status==='starting'||managedRuntime?.switching_to?'真机连接中':'真机未连接';
 $('sidebar-mode').textContent=$('mode-label').textContent;$('sidebar-detail').textContent='当前控制对象为真机';
 $('stat-status').textContent='等待真机连接';$('stat-source').textContent='真机 · 无实时反馈';$('data-age').textContent='OFFLINE';
 $('control-note-text').textContent='请先到“真机连接”页连接真机';
}
function renderRuntime(runtime){
 managedRuntime=runtime;$('runtime-bar').hidden=false;$('runtime-bar').dataset.status=runtime.status;$('runtime-bar').dataset.target=runtime.kind||'fake';
 const hardware=runtime.kind==='hardware',switching=runtime.switching_to;
 const labels=hardware?{stopped:['真机已断开','点击“真机连接”选择设备；不会自动连接真机。'],starting:['正在连接 DM 真机…','正在启动原 DM 驱动并等待六轴与控制器反馈。'],running:['真机已连接 · B601 DM','工作空间、动作实验室、视觉与交互当前均操作这台真机。'],stopping:['正在停止并断开真机…','正在停止输出、失能并释放串口。'],error:['真机连接已中断',runtime.error||'请到真机连接页检查状态并手动重连。']}:{stopped:['本机仿真已停止','点击启动仿真，或下次打开页面时自动启动。'],starting:['正在启动 ROS 2 仿真…','正在加载 DM Fake 驱动并等待关节反馈，无需打开终端。'],running:['本机 ROS 2 仿真已就绪',`B601 ${runtime.model.toUpperCase()} · 独立 ROS 域 ${runtime.domain_id} · 刷新和多标签页共用一个实例`],stopping:['正在停止仿真…','正在释放仿真节点，网页服务将继续待机。'],error:['本机仿真启动失败',runtime.error||'请尝试重新启动。']};
 const [title,description]=labels[runtime.status]||labels.error;$('runtime-title').textContent=switching?'正在切换到'+(switching==='hardware'?'真机…':'仿真…'):title;$('runtime-description').textContent=runtime.switch_error||description;
 $('runtime-action').hidden=false;$('runtime-action').disabled=!!switching||['starting','stopping'].includes(runtime.status);$('runtime-action').textContent=hardware?(runtime.status==='running'?'停止并断开真机':'查看真机连接'):runtime.status==='running'?'停止仿真':runtime.status==='error'?'重试启动':'启动仿真';
 $('runtime-preview').hidden=hardware||!!switching||runtime.status!=='running';$('runtime-preview').textContent=live?'仅看网页演示':'返回仿真实时反馈';
 $('connect-button').innerHTML=`<i data-icon="plug"></i>${hardware&&runtime.status==='running'?'真机已连接':'连接真机'}`;drawIcons($('connect-button'));
 $('connection-open').innerHTML='真机连接与状态 <span>↗</span>';
 updateTargetAvailability();
}
async function startManaged(){
 if(hardwareTarget()){location.hash='hardware';navigate();return}
 wantManagedLive=true;$('runtime-action').disabled=true;
 try{const result=await fetchJson('/api/runtime/start',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});renderRuntime(result.runtime);log('已请求自动启动本机 ROS 2 仿真');}
 catch(error){renderRuntime({...managedRuntime,status:'error',error:error.message});log(`本机仿真启动失败：${error.message}`,'error')}
}
async function syncManagedState(state){
 const runtime=state.runtime;if(!runtime?.managed)return;const previous=managedRuntime?.status;renderRuntime(runtime);
 if(live&&(runtime.switching_to||state.namespace!==live.namespace))disconnect(false);
 if(!runtime.switching_to&&runtime.status==='running'&&state.mode==='ros'&&(wantManagedLive||runtime.kind==='hardware')&&!live&&!connectingManaged){
  connectingManaged=true;try{await connect('',{silent:true});const hardware=runtime.kind==='hardware';$('sidebar-mode').textContent=hardware?'DM 真机已连接':'本机仿真已连接';$('sidebar-detail').textContent=hardware?'原 DM 驱动 · '+runtime.device:'自动启动 · Fake 驱动';if(hardware){$('mode-label').textContent='真机控制';$('viewer-mode').textContent='真机反馈';$('footer-mode').textContent='HARDWARE CONNECTED'}log(hardware?'已连接 DM 真机，三个操作页面均使用真机反馈和控制':'本机仿真已自动启动并连接，实时关节反馈已就绪','success');renderRuntime(runtime)}catch(error){log(`连接失败：${error.message}`,'error')}finally{connectingManaged=false}
 }else if(['stopped','error','stopping'].includes(runtime.status)&&live){disconnect(false)}
 if(runtime.kind==='hardware'&&['stopped','error'].includes(runtime.status))showRvizPlaceholder('真机未连接',runtime.error||'请到真机连接页选择设备并连接。');
 updateTargetAvailability();
 if(previous!==runtime.status&&runtime.status==='error')log(runtime.error||'本机控制对象运行失败','error');
}
async function initializeLocalRuntime(){
 if(!['127.0.0.1','localhost'].includes(location.hostname)){
  $('runtime-bar').hidden=false;$('runtime-title').textContent='打开本机仿真工作台';$('runtime-description').textContent='本机配置完成后，打开本地网页即可自动启动 ROS 2 仿真；在线站点继续提供浏览器演示。';$('runtime-action').hidden=true;$('runtime-local-link').hidden=false;return;
 }
 try{const state=await fetchJson('/api/state');if(!state.runtime?.managed){if(state.mode==='ros')await connect('',{silent:true});return}await syncManagedState(state);if(state.runtime.kind!=='hardware'&&!state.runtime.switching_to&&['stopped','error'].includes(state.runtime.status))await startManaged();
  const poll=async()=>{try{await syncManagedState(await fetchJson('/api/state'))}catch(error){$('runtime-title').textContent='本机网页服务连接中断';$('runtime-description').textContent='可双击桌面的仿真工作台重新打开。';$('runtime-action').disabled=true;}runtimeTimer=setTimeout(poll,800)};runtimeTimer=setTimeout(poll,300);
 }catch(error){log(`本地服务检查失败：${error.message}`,'error');showRvizPlaceholder('本机服务连接失败',`${error.message}；请点击下方“重新连接”重试。`)}
}
$('runtime-action').onclick=async()=>{
 if(!managedRuntime)return;
 if(managedRuntime.kind==='hardware'){if(managedRuntime.status!=='running'){location.hash='hardware';navigate();return}try{await fetchJson('/api/hardware/disconnect',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});if(live)disconnect(false);await syncManagedState(await fetchJson('/api/state'))}catch(error){toast(error.message)}return}
 if(managedRuntime.status!=='running')return startManaged();
 $('runtime-action').disabled=true;releaseScheduler.cancel();wantManagedLive=false;if(live)disconnect(false);
 try{const result=await fetchJson('/api/runtime/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});renderRuntime(result.runtime);log('正在停止本机仿真，网页服务继续待机');}catch(error){toast(error.message);$('runtime-action').disabled=false;}
};
$('runtime-preview').onclick=async()=>{if(live){wantManagedLive=false;disconnect();await chooseBrowserVisual();renderRuntime(managedRuntime)}else{wantManagedLive=true;try{await chooseRvizVisual();await connect('',{silent:true});renderRuntime(managedRuntime)}catch(error){toast(error.message)}}};

async function commitReleasedTarget(joint,value){
 if(hardwareTarget()&&!live){releaseScheduler.cancel();return toast('真机尚未连接，无法发送运动目标。')}
 const speed=Number($('motion-speed').value);
 if(joint==='gripper'){
  if(live){try{await sendCommand('gripper',{opening_mm:value});targetDirty=false}catch(error){toast(error.message)}}
  else{gripper=value;viewer?.update(current,gripper);targetDirty=false;log(`夹爪演示目标：${Math.round(value)} mm`)}
  return;
 }
 if(!live){
  const goals={...current,[joint]:value};const seconds=moveDuration(value-current[joint],speed);
  animateFrames(`${joint.toUpperCase()} 按速度移动`,[{joints:Object.values(goals),gripper}],seconds);return;
 }
 const active=live;manualRequests++;
 try{const result=await sendCommand('move_joint',{joint,position:value,speed});if(live===active)manualAcceptedSequence=Math.max(manualAcceptedSequence,result.sequence??0)}
 catch(error){toast(error.message);targetDirty=false;if(live===active&&active.state)updateLiveState(active.state)}finally{manualRequests--}
}
function updateManualState(state){
 const allowed=!featureState?.control_reserved&&state.motion_allowed&&freshState(state)&&state.enabled&&controlLoopReady(state)&&!state.error_codes?.length&&!['SAFETY_STOPPED','SAFETY_HOMING','SAFETY_FAULT','SAFE_HOMING','GRAVITY_COMP','FAULT','ERROR'].includes(state.state_machine);
 document.querySelectorAll('#joint-controls input,#gripper').forEach(input=>input.disabled=!allowed||commandBusy||actionBusy(state)||actionRequesting);
 if(featureState?.control_reserved){$('home').disabled=true;$('execute').disabled=true;$('control-note-text').textContent='手掌跟随正在控制机械臂，关闭跟随后可手动操作';}
 updateActionState(state,allowed);
 if(!allowed||actionBusy(state)||actionRequesting){releaseScheduler.cancel();return}
 const motion=state.manual_motion;if(!motion)return;
 const key=`${motion.sequence}:${motion.status}`;
 if(motion.status==='moving')$('control-note-text').textContent=`${motion.joint?.toUpperCase()}${motion.transport==='motor_pos_vel'?' · 原驱动':''} 正按 ${motion.speed} rad/s 上限运动${motion.duration_sec?` · 预计 ${motion.duration_sec.toFixed(1)} s`:''}${motion.queued?' · 新目标已排队':''}`;
 if(key!==manualReport){
  manualReport=key;
  if(motion.status==='completed'&&motion.sequence){log(`${motion.joint.toUpperCase()} 已到达目标位置`,'success')}
  if(motion.status==='error'){log(motion.error||'关节运动失败','error');toast(motion.error||'关节运动失败')}
 }
 if(['completed','error','stopped'].includes(motion.status)&&motion.sequence>=manualAcceptedSequence&&!motion.queued&&!releaseScheduler.busy&&!manualRequests){targetDirty=false;target={...current};updateSliders()}
}
addEventListener('pointerup',()=>{for(const joint of [...releaseScheduler.dragging])releaseScheduler.release(joint)});
addEventListener('pointercancel',()=>releaseScheduler.cancel());
addEventListener('blur',()=>releaseScheduler.cancel());

function updateActionState(state,allowed){
 const motion=state.action_motion,busy=actionBusy(state);
 $('action-mode-note').textContent=state.state_machine==='SAFETY_STOPPED'?'控制器已安全锁止，请在原 RViz 面板点击“解除锁止 / 恢复控制”后执行动作。':state.motion_allowed?`${state.control_mode==='fake_rs_pos_vel'?'ROS 仿真':'实机控制'}：与命令行共用原项目动作与逐帧轨迹，按动作倍率执行。`:'当前只读连接，动作按钮未开放。需要由本地桥接显式开放运动控制。';
 for(const id of ['action-speed','workspace-action-speed','feature-speed'])$(id).disabled=busy||actionRequesting;
 $('runtime-preview').disabled=busy||actionRequesting;$('disconnect').disabled=busy||actionRequesting;
 document.querySelectorAll('[data-demo]').forEach(button=>{button.disabled=!allowed||busy||actionRequesting||commandBusy});
 if(!motion)return;
 if(busy){
  $('task-progress').hidden=false;$('task-name').textContent=`${state.control_mode==='fake_rs_pos_vel'?'仿真':'实机'} · ${motion.title||'预设动作'}`;
  const progress=clamp(motion.progress??0,0,1);$('task-bar').style.width=`${progress*100}%`;$('task-percent').textContent=motion.status==='stopping'?'停止中':`${Math.round(progress*100)}%`;
  $('control-note-text').textContent=`${motion.title} · 第 ${motion.step??0}/${motion.total_steps??0} 步 · ${actionSpeedLabel(motion.speed)}`;
 }
 const key=`${motion.sequence}:${motion.status}`;
 if(key===actionReport)return;actionReport=key;
 if(motion.sequence&&['completed','error','stopped'].includes(motion.status)){
  $('task-progress').hidden=true;targetDirty=false;target={...current};updateSliders();
  if(motion.status==='completed')log(`${motion.title}已执行完成，关节反馈已确认`,'success');
  else if(motion.status==='error'){toast(motion.error||'动作执行失败');log(motion.error||'动作执行失败','error')}
  else log(`${motion.title}已停止`);
 }
}

function showRvizPlaceholder(title,detail){$('rviz-placeholder').hidden=false;$('rviz-message').textContent=title;$('rviz-detail').textContent=detail;const ring=$('rviz-placeholder').querySelector('.loading-ring');if(ring)ring.hidden=/失败|断开|不可用/.test(title);}
function applyVisualMode(mode){
 visualMode=mode;const browser=mode==='browser';$('viewer').hidden=!browser;$('browser-toolbar').hidden=!browser;$('rviz-surface').hidden=browser;$('rviz-toolbar').hidden=browser;
 $('visual-title').textContent=browser?'三维工作空间':'RViz 可视化';$('visual-description').textContent=browser?'浏览器 URDF 演示 · 独立于 RViz':'原生 RViz 窗口 · 直接订阅 ROS 2 / TF';
 document.querySelectorAll('[data-visual]').forEach(button=>button.classList.toggle('selected',button.dataset.visual===mode));
 if(browser){clearTimeout(rvizPollTimer);++rvizPollGeneration;$('rviz-frame').src='about:blank';rvizFrameKey='';rvizConnected=false;viewer?.resume?.();requestAnimationFrame(()=>viewer?.resize())}else{stopJointStream();viewer?.pause?.()}
}
function stopJointStream(){if(jointStream){jointStream.source.close();jointStream=null}viewer?.clearStream()}
function startJointStream(){
 if(!live||!viewer||visualMode!=='browser')return;
 if(jointStream?.active===live)return;
 stopJointStream();const active=live,source=new EventSource(`${active.base}/api/joints/stream`),stream={active,source,ready:false};jointStream=stream;
 source.addEventListener('joints',event=>{
  if(jointStream!==stream||live!==active||visualMode!=='browser')return;
  try{const frame=JSON.parse(event.data);if(frame.model!==model||frame.namespace!==active.namespace||frame.joint_age_sec===null||frame.joint_age_sec>1.5)return;
   if(!Array.from({length:6},(_,i)=>frame.positions?.[`joint${i+1}`]).every(Number.isFinite))return;
   stream.ready=true;current=Object.fromEntries(Array.from({length:6},(_,i)=>[`joint${i+1}`,frame.positions[`joint${i+1}`]]));viewer.followSample(frame);
  }catch{/* Invalid feedback is ignored; it never creates a motion command. */}
 });
 source.addEventListener('unavailable',()=>stopJointStream());
 source.onerror=()=>{if(jointStream===stream){stream.ready=false;viewer.clearStream()}};
}
async function chooseBrowserVisual(){
 applyVisualMode('browser');
 if(!viewerLoading&&!viewer){
  $('model-loading').hidden=false;
  viewerLoading=(async()=>{const {RobotViewer}=await import('./viewer.js?v=20261117-manual-6');viewer=new RobotViewer($('viewer'),onPose,({fps})=>{if(visualMode==='browser')$('rviz-connection-label').textContent=fps<1?'画面静止':`${Math.round(fps)} FPS`});await viewer.load(model);viewer.update(current,gripper,{record:false});viewer.resetCamera();$('model-loading').hidden=true;})().catch(error=>{viewer?.destroy();viewer=null;throw error}).finally(()=>{viewerLoading=null});
 }
 if(viewerLoading)await viewerLoading;
 if(visualMode==='browser')startJointStream();
 return viewer;
}
async function chooseRvizVisual(){
 applyVisualMode('rviz');
 if(live)return ensureRviz(live.base);
 const local=['127.0.0.1','localhost'].includes(location.hostname);
 $('rviz-local-link').hidden=local;
 showRvizPlaceholder(local?'正在等待本机 ROS 2…':'原生 RViz 在你的电脑上运行',local?'启动仿真或连接 ROS 2 后自动显示。':'打开本机 RViz 工作台后，会自动启动仿真和真正的 RViz；无需命令行。');
 $('rviz-toolbar').hidden=!local;
}
async function ensureRviz(base='',retry=false){
 clearTimeout(rvizPollTimer);const generation=++rvizPollGeneration;rvizBase=base;$('rviz-window').href=`${base}/rviz.html`;$('rviz-local-link').hidden=true;
 const poll=async()=>{
  if(generation!==rvizPollGeneration||visualMode!=='rviz')return;
  let status;
  try{
   status=await fetchJson(`${base}/api/rviz/status`);
   if((status.status==='stopped'||(status.status==='error'&&retry))&&live?.state?.mode==='ros'){
    retry=false;
    status=(await fetchJson(`${base}/api/rviz/start`,{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'})).rviz;
   }
   if(generation!==rvizPollGeneration||visualMode!=='rviz')return;
   $('rviz-connection-label').textContent=status.status==='running'?'RViz · ROS 实时显示':status.status==='starting'?'启动中':'本机运行';
   if(status.status==='running'){
    const key=`${base}:${status.generation}`;
    if(rvizFrameKey!==key){rvizFrameKey=key;rvizConnected=false;$('rviz-frame').src=`${base}/rviz.html?session=${status.generation}`;}
    $('rviz-frame').hidden=false;$('rviz-placeholder').hidden=rvizConnected;
    if(!rvizConnected)showRvizPlaceholder('正在连接原生 RViz…','正在建立本机显示通道');
   }else{
    $('rviz-frame').hidden=true;
    showRvizPlaceholder(status.status==='error'?'RViz 启动失败':status.status==='starting'?'正在启动原生 RViz…':'等待 ROS 2 / RViz',status.error||'独立显示会话正在准备中，首次启动需要几秒钟。');
   }
   if(status.status==='error')return;
  }catch(error){showRvizPlaceholder('RViz 显示暂不可用',`${error.message}；可点击重新连接，或使用网页 3D 演示。`);return;}
  if(generation===rvizPollGeneration)rvizPollTimer=setTimeout(poll,status?.status==='running'?1500:500);
 };
 await poll();
}
document.querySelectorAll('[data-visual]').forEach(button=>button.onclick=()=>{(button.dataset.visual==='browser'?chooseBrowserVisual():chooseRvizVisual()).catch(error=>toast(error.message))});
$('rviz-reconnect').onclick=()=>{rvizFrameKey='';rvizConnected=false;if(!live&&['127.0.0.1','localhost'].includes(location.hostname))initializeLocalRuntime();else ensureRviz(live?.base??rvizBase,true)};
$('rviz-fullscreen').onclick=()=>{$('rviz-surface').requestFullscreen?.().catch(error=>toast(error.message))};
addEventListener('message',event=>{
 const expected=rvizBase?new URL(rvizBase).origin:location.origin;
 if(event.origin!==expected||event.source!==$('rviz-frame').contentWindow)return;
 if(event.data?.type==='rebotarm-rviz-connected'){rvizConnected=true;$('rviz-placeholder').hidden=true;}
 if(event.data?.type==='rebotarm-rviz-disconnected'){rvizConnected=false;showRvizPlaceholder('RViz 显示连接已断开','点击重新连接即可恢复，ROS 控制状态仍在右侧显示。');}
 if(event.data?.type==='rebotarm-rviz-error')showRvizPlaceholder('RViz 显示连接失败',event.data.message);
});
if(visualMode==='browser')chooseBrowserVisual().catch(error=>toast(error.message));
else if(!['127.0.0.1','localhost'].includes(location.hostname))chooseRvizVisual();

interactions=createInteractions({getBase:()=>live?.base??'',getLive:()=>live?.state,onState:state=>{featureState=state;if(live?.state)updateManualState(live.state)},notify:toast,log,onView:()=>{navigate();if(visualMode!=='rviz')chooseRvizVisual().catch(error=>toast(error.message))}});
if(location.hash==='#vision'&&visualMode!=='rviz')chooseRvizVisual().catch(error=>toast(error.message));

hardwarePage=createHardwarePage({notify:toast,log,onAdvanced:()=>openConnection(true),onTargetRequested:()=>{wantManagedLive=true;releaseScheduler.cancel();if(live)disconnect(false);rvizFrameKey='';rvizConnected=false;syncManagedStateFromServer()}});
async function syncManagedStateFromServer(){try{await syncManagedState(await fetchJson('/api/state'))}catch(error){toast(error.message)}}
