import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { STLLoader } from './vendor/STLLoader.js';
import { JointPlayback } from './joint-playback.js?v=20261117-hardware-5';

export class RobotViewer {
  constructor(container, onPose, onPerformance=()=>{}) {
    this.container=container; this.onPose=onPose; this.onPerformance=onPerformance; this.playback=new JointPlayback(); this.drawable=true; this.inView=true; this.dirty=true; this.poseFrames=0; this.feedbackFrames=0; this.feedbackMark=0; this.posePoint=new THREE.Vector3(); this.poseQuaternion=new THREE.Quaternion(); this.poseEuler=new THREE.Euler(); this.activeModel=null; this.joints={}; this.links={}; this.modelVersion=0;
    this.scene=new THREE.Scene();
    this.camera=new THREE.PerspectiveCamera(35,1,.005,30);
    this.camera.up.set(0,0,1);
    this.renderer=new THREE.WebGLRenderer({antialias:true,alpha:true,powerPreference:'high-performance'});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio,1.25));
    this.renderer.setClearColor(0,0);this.renderer.shadowMap.enabled=false;
    this.renderer.outputColorSpace=THREE.SRGBColorSpace;this.renderer.toneMapping=THREE.ACESFilmicToneMapping;this.renderer.toneMappingExposure=1.3;
    this.renderer.domElement.setAttribute('aria-label','reBot Arm 三维机械模型，可拖动旋转，滚轮缩放');
    container.prepend(this.renderer.domElement);
    this.controls=new OrbitControls(this.camera,this.renderer.domElement);this.controls.enableDamping=true;this.controls.dampingFactor=.08;
    this.controls.minDistance=.3;this.controls.maxDistance=3;this.controls.maxPolarAngle=Math.PI*.49;this.controls.target.set(0,0,.23);
    this.scene.add(new THREE.HemisphereLight(0xffffff,0x91a27b,1.5));
    const key=new THREE.DirectionalLight(0xfff7e3,2.4);key.position.set(.4,-.5,1.3);this.scene.add(key);
    const rim=new THREE.DirectionalLight(0xe6f3d5,1);rim.position.set(-.5,.5,.6);this.scene.add(rim);
    const shadowCanvas=document.createElement('canvas');shadowCanvas.width=shadowCanvas.height=64;const shadowContext=shadowCanvas.getContext('2d');const gradient=shadowContext.createRadialGradient(32,32,4,32,32,32);gradient.addColorStop(0,'rgba(56,74,32,.20)');gradient.addColorStop(1,'rgba(56,74,32,0)');shadowContext.fillStyle=gradient;shadowContext.fillRect(0,0,64,64);
    const ground=new THREE.Mesh(new THREE.PlaneGeometry(.32,.32),new THREE.MeshBasicMaterial({map:new THREE.CanvasTexture(shadowCanvas),transparent:true,depthWrite:false}));ground.position.z=-.003;this.scene.add(ground);
    this.grid=new THREE.GridHelper(2.4,24,0xb2c19d,0xccd6bf);this.grid.rotation.x=Math.PI/2;this.grid.material.transparent=true;this.grid.material.opacity=.45;this.grid.position.z=-.001;this.scene.add(this.grid);
    this.axes=new THREE.Group();for(const [end,color,label] of [[[.16,0,.002],0xb18c75,'X'],[[0,.16,.002],0x8ba377,'Y'],[[0,0,.16],0x8a9aab,'Z']]){
      const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(...end)]),new THREE.LineBasicMaterial({color,transparent:true,opacity:.7}));this.axes.add(line);
      const canvas=document.createElement('canvas');canvas.width=64;canvas.height=64;const ctx=canvas.getContext('2d');ctx.fillStyle='#839275';ctx.font='32px monospace';ctx.textAlign='center';ctx.fillText(label,32,42);const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(canvas),transparent:true}));sprite.scale.set(.025,.025,.025);sprite.position.set(...end);sprite.position.multiplyScalar(1.1);this.axes.add(sprite);
    }this.scene.add(this.axes);
    const platform=new THREE.Mesh(new THREE.CylinderGeometry(.10,.11,.012,64),new THREE.MeshPhongMaterial({color:0xc5ceb8,shininess:12}));platform.rotation.x=Math.PI/2;platform.position.z=-.009;platform.receiveShadow=true;this.scene.add(platform);
    this.tcp=new THREE.Mesh(new THREE.SphereGeometry(.005,12,12),new THREE.MeshBasicMaterial({color:0x95ad63}));this.scene.add(this.tcp);
    this.resizeObserver=new ResizeObserver(()=>{this.resize()});this.resizeObserver.observe(container);this.resetCamera();
    this.intersectionObserver=new IntersectionObserver(entries=>{this.inView=entries[0].isIntersecting;if(this.inView)this.dirty=true});this.intersectionObserver.observe(container);
    this.perf={start:performance.now(),frames:0,poses:0};
    this.renderFrame=now=>{
      if(this.paused||document.hidden||!this.drawable||!this.inView){this.perf={start:now,frames:0,poses:0};this.feedbackMark=this.feedbackFrames;return}
      const sample=this.playback.sample(now);if(sample)this.applyPose(sample.positions,sample.gripper??this.lastGripper??45);
      if(this.poseFrames!==this.renderedPose){this.perf.poses++;this.renderedPose=this.poseFrames;}
      const cameraChanged=this.controls.update();
      if(this.dirty||cameraChanged){this.renderer.render(this.scene,this.camera);this.dirty=false;this.perf.frames++}
      if(now-this.perf.start>=1000){const fps=this.perf.frames*1000/(now-this.perf.start),poseFps=this.perf.poses*1000/(now-this.perf.start);Object.assign(this.renderer.domElement.dataset,{fps:fps.toFixed(1),poseFps:poseFps.toFixed(1),triangles:String(this.renderer.info.render.triangles),calls:String(this.renderer.info.render.calls),width:String(this.renderer.domElement.width),height:String(this.renderer.domElement.height)});if(poseFps>=20)this.renderer.domElement.dataset.motionFps=fps.toFixed(1);this.renderer.domElement.dataset.feedbackHz=((this.feedbackFrames-this.feedbackMark)*1000/(now-this.perf.start)).toFixed(1);this.feedbackMark=this.feedbackFrames;this.onPerformance({fps,poseFps});this.perf={start:now,frames:0,poses:0}}
    };this.renderer.setAnimationLoop(this.renderFrame);
    this.geometryCache=new Map();this.loader=new STLLoader();
  }
  resize(){const w=this.container.clientWidth,h=this.container.clientHeight;this.drawable=!!(w&&h);if(!this.drawable)return;this.dirty=true;this.camera.aspect=w/h;this.camera.updateProjectionMatrix();this.renderer.setSize(w,h,false)}
  resetCamera(view='perspective'){
    this.cameraView=view;const center=new THREE.Vector3(.18,0,.25);let radius=.32;if(this.root){this.root.updateMatrixWorld(true);const box=new THREE.Box3().setFromObject(this.root);box.getCenter(center);radius=box.getBoundingSphere(new THREE.Sphere()).radius;}const directions={perspective:[1,-1.5,.8],front:[0,-1,.12],top:[.001,-.001,1],side:[1,0,.12]};const aspect=this.container.clientWidth/Math.max(1,this.container.clientHeight);const halfFov=Math.atan(Math.tan(THREE.MathUtils.degToRad(17.5))*Math.min(1,aspect));const distance=Math.max(.65,radius/Math.sin(halfFov)*1.13);const direction=new THREE.Vector3(...(directions[view]||directions.perspective)).normalize();this.camera.position.copy(center).addScaledVector(direction,distance);this.controls.target.copy(center);this.controls.update();this.dirty=true;
  }
  origin(object,xyz,rpy){object.position.set(...xyz);object.quaternion.setFromEuler(new THREE.Euler(...rpy,'ZYX'))}
  async load(model){
    const version=++this.modelVersion;const response=await fetch(`assets/models/${model}.web.json`);if(!response.ok)throw Error('机械臂模型清单加载失败');const manifest=await response.json();
    const root=new THREE.Group(),links={},joints={};
    for(const link of manifest.links){links[link.name]=new THREE.Group();links[link.name].name=link.name}
    root.add(links.base_link);
    for(const joint of manifest.joints){const fixed=new THREE.Group();this.origin(fixed,joint.xyz,joint.rpy);const pivot=new THREE.Group();fixed.add(pivot);pivot.add(links[joint.child]);links[joint.parent].add(fixed);fixed.updateMatrix();fixed.matrixAutoUpdate=false;joints[joint.name]={...joint,pivot,axisVector:new THREE.Vector3(...joint.axis).normalize(),value:null}}
    await Promise.all(manifest.links.flatMap(link=>link.visuals.map(async v=>{
      if(!this.geometryCache.has(v.mesh))this.geometryCache.set(v.mesh,this.loader.loadAsync(v.mesh).catch(e=>{this.geometryCache.delete(v.mesh);throw e}));
      const geometry=await this.geometryCache.get(v.mesh);const color=new THREE.Color(...v.color.slice(0,3));
      const metallic=v.mesh.includes('metal')||v.mesh.includes('silver')||v.mesh.includes('grey');const mat=new THREE.MeshPhongMaterial({color,shininess:metallic?48:18,specular:metallic?0x494949:0x202020});const mesh=new THREE.Mesh(geometry,mat);this.origin(mesh,v.xyz,v.rpy);mesh.scale.set(...v.scale);mesh.updateMatrix();mesh.matrixAutoUpdate=false;links[link.name].add(mesh);
    })));
    if(version!==this.modelVersion){root.traverse(o=>{if(o.material)o.material.dispose()});return null}
    if(this.root){this.scene.remove(this.root);this.root.traverse(o=>{if(o.material)o.material.dispose()})}
    this.root=root;this.links=links;this.joints=joints;this.jointList=Object.values(joints);this.playback.clear();this.dirty=true;this.manifest=manifest;this.activeModel=model;this.scene.add(root);this.resize();return manifest;
  }
  update(positions,gripper=45,options={}){this.playback.clear();this.applyPose(positions,gripper,options)}
  followSample(frame){if(this.playback.push(frame))this.feedbackFrames++}
  clearStream(){this.playback.clear()}
  invalidate(){this.dirty=true}
  applyPose(positions,gripper=45,{record=true}={}){
    if(!this.root)return false;
    let changed=false;
    for(const joint of this.jointList){
      let value=positions[joint.name]??0;
      if(joint.type==='prismatic')value=joint.name==='finger_right'?-(gripper/90)*.05:(gripper/90)*.05;
      if(joint.mimic)value=(positions[joint.mimic.joint]??(gripper/90)*.05)*Number(joint.mimic.multiplier??1)+Number(joint.mimic.offset??0);
      if(joint.value!==null&&Math.abs(joint.value-value)<1e-8)continue;
      joint.value=value;changed=true;
      if(joint.type==='revolute'||joint.type==='continuous')joint.pivot.quaternion.setFromAxisAngle(joint.axisVector,value);
      else if(joint.type==='prismatic')joint.pivot.position.copy(joint.axisVector).multiplyScalar(value);
    }
    this.lastGripper=gripper;if(!changed)return false;this.poseFrames++;this.dirty=true;
    this.root.updateMatrixWorld(true);const point=this.posePoint;const end=this.links.end_link??this.links.gripper_end??this.links.link6;end.getWorldPosition(point);this.tcp.position.copy(point);
    const quaternion=this.poseQuaternion;end.getWorldQuaternion(quaternion);const euler=this.poseEuler.setFromQuaternion(quaternion,'ZYX');this.onPose({x:point.x,y:point.y,z:point.z,roll:euler.x,pitch:euler.y,yaw:euler.z});return true;
  }
  pause(){this.paused=true;this.renderer.setAnimationLoop(null)}
  resume(){this.paused=false;this.dirty=true;this.perf={start:performance.now(),frames:0,poses:0};this.renderer.setAnimationLoop(this.renderFrame);this.resize()}
  destroy(){this.resizeObserver.disconnect();this.intersectionObserver.disconnect();this.controls.dispose();this.scene.traverse(object=>{object.material?.map?.dispose();object.material?.dispose()});for(const promise of this.geometryCache.values())promise.then(geometry=>geometry.dispose(),()=>{});this.renderer.setAnimationLoop(null);this.renderer.dispose()}
}
