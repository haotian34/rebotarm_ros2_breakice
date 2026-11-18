// A drag edits a target; only release (or the end of keyboard edits) commits it.
export const moveDuration=(delta,speed)=>Math.max(.25,1.6*Math.abs(delta)/speed);
// The original passthrough intentionally stops the end-pose loop while motors
// execute position/velocity targets. This state is ready for the next command.
export const controlLoopReady=state=>!!state.control_loop_active||(state.state_machine==='LOWLEVEL_STREAMING'&&['pos_vel','mit'].includes(state.control_mode));
export class ReleaseScheduler {
  constructor(commit,delay=180){this.commit=commit;this.delay=delay;this.dragging=new Set();this.pending=new Map();this.timers=new Map();}
  begin(joint){this.dragging.add(joint);this.clearTimer(joint);}
  edit(joint,value){this.pending.set(joint,value);this.clearTimer(joint);}
  release(joint){this.dragging.delete(joint);this.schedule(joint);}
  schedule(joint){if(this.dragging.has(joint)||!this.pending.has(joint))return;this.clearTimer(joint);this.timers.set(joint,setTimeout(()=>{this.timers.delete(joint);if(this.dragging.has(joint)||!this.pending.has(joint))return;const value=this.pending.get(joint);this.pending.delete(joint);this.commit(joint,value);},this.delay));}
  clearTimer(joint){clearTimeout(this.timers.get(joint));this.timers.delete(joint);}
  cancelJoint(joint){this.clearTimer(joint);this.dragging.delete(joint);this.pending.delete(joint);}
  cancel(){for(const joint of this.timers.keys())this.clearTimer(joint);this.dragging.clear();this.pending.clear();}
  get busy(){return this.pending.size>0||this.dragging.size>0;}
}
