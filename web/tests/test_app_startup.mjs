import vm from 'node:vm';
import {readFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import test from 'node:test';
import assert from 'node:assert/strict';
const root=fileURLToPath(new URL('../dist/',import.meta.url));
async function startApp({brokenModule=false,unavailable=false,page='workspace',devices=false,target='fake',connected=true,controllerState='IDLE',loopActive=true}={}){
const elements=[];
function matches(el,selector){
 selector=selector.trim();
 if(selector.includes(' ')){const parts=selector.split(/\s+/);return matches(el,parts.pop())&&el.parents().some(p=>matches(p,parts.join(' ')))}
 if(selector.startsWith('#'))return el.id===selector.slice(1);
 if(selector.startsWith('.'))return el.classes.has(selector.slice(1));
 const attr=selector.match(/^\[([^=\]]+)(?:="([^"]*)")?\]$/);if(attr)return attr[2]===undefined?attr[1] in el.attrs:el.attrs[attr[1]]===attr[2];
 return el.tag===selector;
}
class Element{
 constructor(tag,attrs={},parent=null){this.tag=tag;this.attrs=attrs;this.id=attrs.id;this.parent=parent;this.children=[];this.classes=new Set((attrs.class||'').split(/\s+/));this.dataset=Object.fromEntries(Object.entries(attrs).filter(([key])=>key.startsWith('data-')).map(([key,value])=>[key.slice(5).replace(/-([a-z])/g,(_,v)=>v.toUpperCase()),value]));this.style={setProperty(){}};this.classList={toggle:(key,force)=>force?this.classes.add(key):this.classes.delete(key),add:key=>this.classes.add(key),remove:key=>this.classes.delete(key)};this.value=attrs.value??'';this.checked='checked' in attrs;this.hidden='hidden' in attrs;this.listeners={};elements.push(this);}
 remove(){elements.splice(elements.indexOf(this),1)}
 parents(){return this.parent?[this.parent,...this.parent.parents()]:[]}
 querySelectorAll(selector){return elements.filter(el=>el.parents().includes(this)&&selector.split(',').some(s=>matches(el,s)))}
 querySelector(selector){return this.querySelectorAll(selector)[0]??null}
 set innerHTML(html){this.html=html;parse(html,this)}get innerHTML(){return this.html||''}
 addEventListener(name,fn){this.listeners[name]=fn}
 setAttribute(key,value){this.attrs[key]=value}
 getAttribute(key){return key==='src'?this.src??null:this.attrs[key]??null}
 removeAttribute(key){delete this.attrs[key]}
 get parentElement(){return this.parent}
 appendChild(child){if(child.parent)child.parent.children=child.parent.children.filter(el=>el!==child);child.parent=this;this.children.push(child);return child}
 append(child){this.appendChild(child)}
 replaceChildren(){this.children=[]}
 scrollIntoView(){this.scrolled=true}
 async play(){this.readyState=2}
 getContext(){return {drawImage(){}}}
 toBlob(callback){callback(new Blob(['jpeg']))}
}
function parse(html,parent){let stack=[parent];for(const match of html.matchAll(/<\/?[a-zA-Z][^>]*>/g)){const raw=match[0],name=raw.match(/^<\/?([\w-]+)/)[1].toLowerCase();if(raw.startsWith('</')){const at=stack.findLastIndex(e=>e.tag===name);if(at>0)stack.length=at;continue}const attrs={};for(const attr of raw.slice(name.length+1,-1).matchAll(/([\w-]+)(?:="([^"]*)"|='([^']*)'|=([^\s>]+))?/g))attrs[attr[1]]=attr[2]??attr[3]??attr[4]??'';const el=new Element(name,attrs,stack.at(-1));stack.at(-1).children.push(el);if(!['input','meta','link','br','img','hr','source'].includes(name))stack.push(el)}}
const doc=new Element('document');parse(await readFile(root+'index.html','utf8'),doc);
const get=id=>elements.find(e=>e.id===id);
for(const select of elements.filter(e=>e.tag==='select')){select.value=select.querySelectorAll('option').find(e=>'selected' in e.attrs)?.value??select.querySelectorAll('option')[0]?.value??''}
const calls=[],timers=[],events={};
const state={mode:'ros',model:'dm',namespace:'rebotarm_web',enabled:true,motion_allowed:true,control_mode:'fake_rs_pos_vel',state_machine:'IDLE',control_loop_active:true,error_codes:[],positions:Object.fromEntries(Array.from({length:6},(_,i)=>['joint'+(i+1),0])),joint_age_sec:.01,status_age_sec:.01,gripper_opening_mm:0,manual_motion:{status:'idle',sequence:0},action_motion:{status:'idle',sequence:0},runtime:{managed:true,status:'running',model:'dm',domain_id:187}};
const featureState={features:Object.fromEntries(['camera','gesture','palm','voice','agent'].map(name=>[name,{status:'stopped',error:''}])),vision:null,frame_sequence:0,control_reserved:false,voice_busy:false};
state.runtime.kind=target;
state.state_machine=controllerState;state.control_loop_active=loopActive;
const hardware={available:true,active_target:target,switching_to:null,error:'',connection:{kind:'hardware',status:'stopped',stage:'idle',device:''},devices:[{path:'/dev/serial/by-id/test-dm',device:'/dev/ttyACM0',label:'test-dm',accessible:true,owners:[]}],feedback:null};
function connectHardware(){state.namespace='rebotarm_web_hardware';state.control_mode='pos_vel';Object.assign(state.runtime,{kind:'hardware',status:'running',namespace:state.namespace,domain_id:188,device:hardware.devices[0].path});hardware.active_target='hardware';Object.assign(hardware.connection,{status:'running',stage:'ready',device:hardware.devices[0].path});hardware.feedback=state;}
if(target==='hardware'){if(connected)connectHardware();else{Object.assign(state,{mode:'demo',motion_allowed:false,positions:{},joint_age_sec:null,status_age_sec:null});state.runtime.status='stopped';}}
const requests=[],tracks=[{stopped:false,stop(){this.stopped=true}}];
const navigator=devices?{mediaDevices:{async getUserMedia(){return {getTracks:()=>tracks}},async enumerateDevices(){return [{kind:'videoinput',deviceId:'test-camera',label:'Camera'}]}}}:{};
const location={hostname:'127.0.0.1',_hash:'#'+page,get hash(){return this._hash},set hash(value){this._hash=value.startsWith('#')?value:'#'+value},search:'',origin:'http://127.0.0.1:8088'};
const context=vm.createContext({console,URL,URLSearchParams,AbortSignal,performance,Date,Map,Set,Math,Number,Object,Array,JSON,Error,Promise,Blob,document:{documentElement:doc,getElementById:get,createElement:tag=>new Element(tag),querySelectorAll:s=>doc.querySelectorAll(s),querySelector:s=>doc.querySelector(s),addEventListener(){}},location,navigator,addEventListener:(name,fn)=>events[name]=fn,requestAnimationFrame:()=>1,cancelAnimationFrame(){},setTimeout:(fn,ms)=>{timers.push({fn,ms});return timers.length},clearTimeout(){},fetch:async(url,options)=>{calls.push(url);requests.push({url,options});let data;if(url==='/api/state'){if(unavailable)throw Error('service unavailable');data=state;}else if(url==='/api/hardware')data=hardware;else if(url==='/api/hardware/connect'){connectHardware();data=hardware;}else if(url==='/api/features')data=featureState;else if(url==='/api/features/start'||url==='/api/features/stop'){const {feature}=JSON.parse(options.body),start=url.endsWith('start');featureState.features[feature].status=start?'running':'stopped';featureState.vision=start?feature:null;featureState.control_reserved=start&&feature==='palm';data=featureState;}else if(url==='/api/features/frame'){featureState.frame_sequence++;featureState.frame_age_sec=.01;data={success:true};}else if(url==='/api/rviz/status')data={status:'running',generation:1};else data=JSON.parse(await readFile(root+url,'utf8'));return{ok:true,json:async()=>data}}});
const cache=new Map();async function load(path){if(cache.has(path))return cache.get(path);const module=new vm.SourceTextModule(await readFile(path,'utf8'),{context,identifier:path,importModuleDynamically:async(specifier,reference)=>{if(brokenModule)throw SyntaxError('旧脚本导出不匹配');const module=await load(fileURLToPath(new URL(specifier,'file://'+reference.identifier)));await module.evaluate();return module}});cache.set(path,module);await module.link((specifier,reference)=>load(fileURLToPath(new URL(specifier,'file://'+reference.identifier))));return module}
const app=await load(root+'boot.js');await app.evaluate();
for(let i=0;i<25;i++)await new Promise(resolve=>setImmediate(resolve));
return {get,calls,doc,events,location,requests,timers,tracks};
}

test('clicking palm follow acquires browser camera, invokes local start, shows recognition and stops with one click',async()=>{
 const {get,doc,requests,timers,tracks}=await startApp({page:'vision',devices:true});
 get('feature-speed').value='0.5';
 const button=doc.querySelector('[data-feature="palm"]');
 await button.onclick();
 for(let i=0;i<5;i++)await new Promise(resolve=>setImmediate(resolve));
 const start=requests.find(request=>request.url==='/api/features/start');
 assert.deepEqual(JSON.parse(start.options.body),{feature:'palm',speed:.5});
 assert.ok(requests.some(request=>request.url==='/api/features/frame'&&request.options.headers['Content-Type']==='image/jpeg'));
 await timers.find(timer=>timer.ms===800).fn();
 assert.equal(get('recognition-image').hidden,false);
 assert.match(get('recognition-image').src,/\/api\/features\/camera\?t=/);
 assert.equal(get('robot-visual-panel').parentElement.id,'vision-visual-slot');
 assert.equal(get('rviz-frame').hidden,false);
 assert.equal(get('joint-1').disabled,true);
 await button.onclick();
 assert.ok(tracks.every(track=>track.stopped));
 assert.ok(requests.some(request=>request.url==='/api/features/stop'));
 assert.equal(get('camera-placeholder').hidden,false);
 assert.equal(get('camera-start').hidden,false);
 assert.equal(get('joint-1').disabled,false);
});

test('vision shows the existing RViz beside recognition and retains the single viewer on navigation',async()=>{
 const {get,calls,doc,events,location}=await startApp({page:'vision'});
 assert.equal(doc.dataset.boot,'ready');
 assert.equal(get('robot-visual-panel').parentElement.id,'vision-visual-slot');
 assert.equal(get('rviz-frame').src,'/rviz.html?session=1');
 assert.equal(get('recognition-image').parentElement,get('camera').parentElement);
 assert.equal(get('vision-modules').querySelectorAll('[data-feature]').length,4);
 assert.equal(get('vision-modules').querySelectorAll('.copy-command').length,0);
 location.hash='#workspace';events.hashchange();
 assert.equal(get('robot-visual-panel').parentElement.id,'workspace-visual-slot');
 assert.equal(doc.querySelectorAll('#rviz-frame').length,1);
 assert.ok(!calls.includes('/api/features/start'),'Opening a page must not start input devices or motion');
});

test('actual app startup connects managed ROS and assigns the native RViz frame',async()=>{
 const {get,calls,doc}=await startApp();
 assert.equal(doc.dataset.boot,'ready');
 assert.equal(get('runtime-title').textContent,'本机 ROS 2 仿真已就绪');
 assert.equal(get('rviz-frame').src,'/rviz.html?session=1');
 assert.equal(get('sidebar-mode').textContent,'本机仿真已连接');
 assert.ok(calls.includes('/api/state')&&calls.includes('/api/rviz/status'));
 assert.ok(!calls.includes('/api/command'),'Opening the UI must not submit motion');
});

test('module startup errors show a concrete failure and reload instead of an endless spinner',async()=>{
 const {get,doc}=await startApp({brokenModule:true});
 assert.equal(doc.dataset.boot,'failed');
 assert.equal(get('rviz-message').textContent,'网页初始化失败');
 assert.match(get('rviz-detail').textContent,/旧脚本导出不匹配/);
 assert.equal(get('rviz-placeholder').querySelector('.loading-ring'),null);
 assert.equal(typeof get('rviz-reconnect').onclick,'function');
});

test('an unavailable ROS web API shows a connection failure instead of waiting forever',async()=>{
 const {get}=await startApp({unavailable:true});
 assert.equal(get('rviz-message').textContent,'本机服务连接失败');
 assert.match(get('rviz-detail').textContent,/service unavailable/);
});

test('entrypoint and changed module imports use one explicit revision',async()=>{
 const html=await readFile(root+'index.html','utf8'),boot=await readFile(root+'boot.js','utf8'),app=await readFile(root+'app.js','utf8');
 const revision=boot.match(/const revision='([^']+)'/)[1];
 assert.ok(html.includes('boot.js?v='+revision));
 for(const dependency of ['kinematics.js','manual-control.js','action-presets.js','viewer.js','interactions.js','hardware.js'])assert.ok(app.includes(dependency+'?v='+revision));
});

test('opening hardware diagnostics only reads status; one click switches the shared control target',async()=>{
 const {get,requests,doc,timers}=await startApp({page:'hardware'});
 assert.equal(doc.dataset.boot,'ready');
 assert.equal(get('hardware-device').value,'/dev/serial/by-id/test-dm');
 assert.equal(get('hardware-connect').disabled,false);
 assert.ok(!requests.some(r=>r.options?.method==='POST'),'Diagnostics must not connect or send motion');
 await get('hardware-connect').onclick();
 for(let i=0;i<10;i++)await new Promise(resolve=>setImmediate(resolve));
 const request=requests.filter(r=>r.url==='/api/hardware/connect');
 assert.equal(request.length,1);
 assert.deepEqual(JSON.parse(request[0].options.body),{device:'/dev/serial/by-id/test-dm'});
 assert.equal(get('runtime-title').textContent,'真机已连接 · B601 DM');
 assert.equal(get('sidebar-mode').textContent,'DM 真机已连接');
 assert.equal(get('hardware-target').textContent,'真机');
 assert.equal(get('hardware-enabled').textContent,'已使能');
 assert.equal(get('hardware-workspace').hidden,false);
 assert.equal(get('runtime-preview').hidden,true);
 assert.ok(!requests.some(r=>r.url==='/api/runtime/start'));
});

test('refreshing a disconnected hardware target never starts Fake or reconnects hardware',async()=>{
 const {get,requests,doc}=await startApp({target:'hardware',connected:false});
 assert.equal(doc.dataset.boot,'ready');
 for(const id of ['joint-1','gripper','execute','home','enable'])assert.equal(get(id).disabled,true,id);
 assert.equal(get('runtime-title').textContent,'真机已断开');
 assert.equal(get('rviz-message').textContent,'真机未连接');
 assert.ok(!requests.some(r=>r.options?.method==='POST'));
});

test('refreshing connected hardware reuses live ROS without reconnecting or submitting motion',async()=>{
 const {get,requests}=await startApp({target:'hardware'});
 assert.equal(get('sidebar-mode').textContent,'DM 真机已连接');
 assert.equal(get('rviz-frame').src,'/rviz.html?session=1');
 assert.equal(get('runtime-preview').hidden,true);
 assert.ok(!requests.some(r=>r.url==='/api/hardware/connect'||r.url==='/api/command'||r.url==='/api/runtime/start'));
});

test('DM motor mode keeps manual and preset controls enabled after the end-pose loop stops',async()=>{
 const {get,doc,requests}=await startApp({target:'hardware',controllerState:'LOWLEVEL_STREAMING',loopActive:false});
 for(const id of ['joint-1','angle-4','gripper','home'])assert.equal(get(id).disabled,false,id);
 assert.ok(doc.querySelectorAll('[data-demo]').every(button=>!button.disabled));
 assert.ok(!requests.some(r=>r.url==='/api/command'),'Readiness must not unlock or move the robot');
});

test('safety latch still blocks motion and explains the original RViz release control',async()=>{
 const {get,doc,requests}=await startApp({target:'hardware',controllerState:'SAFETY_STOPPED'});
 assert.equal(get('joint-1').disabled,true);
 assert.ok(doc.querySelectorAll('[data-demo]').every(button=>button.disabled));
 assert.match(get('action-mode-note').textContent,/解除锁止/);
 assert.ok(!requests.some(r=>r.url==='/api/command'));
});
