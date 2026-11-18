// Visual playback only: interpolate between received feedback samples; never predict a target.
export class JointPlayback {
  constructor(delayMs=80){this.delayMs=delayMs;this.clear();}
  clear(){this.samples=[];this.offset=null;this.lastReceived=0;}
  push(frame,received=performance.now()){
    const time=frame.sample_time*1000;
    if(!Number.isFinite(time)||!Object.values(frame.positions??{}).every(Number.isFinite))return false;
    if(this.samples.length&&time<=this.samples.at(-1).time)return false;
    if(received-this.lastReceived>300)this.clear();
    // Monotonic clocks have different origins. The smallest observed offset excludes queued delivery.
    this.offset=Math.min(this.offset??Infinity,received-time);this.lastReceived=received;
    this.samples.push({time,positions:{...frame.positions},gripper:frame.gripper_opening_mm});
    if(this.samples.length>24)this.samples.shift();return true;
  }
  sample(now=performance.now()){
    if(!this.samples.length)return null;
    const time=now-this.offset-this.delayMs;
    while(this.samples.length>2&&this.samples[1].time<=time)this.samples.shift();
    const a=this.samples[0],b=this.samples[1];
    if(!b||time<=a.time)return a;
    const t=Math.max(0,Math.min(1,(time-a.time)/(b.time-a.time)));
    const positions=Object.fromEntries(Object.entries(a.positions).map(([key,value])=>[key,value+((b.positions[key]??value)-value)*t]));
    const gripper=Number.isFinite(a.gripper)&&Number.isFinite(b.gripper)?a.gripper+(b.gripper-a.gripper)*t:b.gripper;
    return{positions,gripper};
  }
}
