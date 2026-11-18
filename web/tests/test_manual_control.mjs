import {test} from 'node:test';
import assert from 'node:assert/strict';
import {ReleaseScheduler,moveDuration,controlLoopReady} from '../dist/manual-control.js';
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));

test('original lowlevel motor mode remains ready after stopping the end-pose loop',()=>{
 for(const control_mode of ['pos_vel','mit'])assert.equal(controlLoopReady({state_machine:'LOWLEVEL_STREAMING',control_mode,control_loop_active:false}),true);
 for(const state_machine of ['IDLE','SAFETY_STOPPED','SAFETY_FAULT','GRAVITY_COMP'])assert.equal(controlLoopReady({state_machine,control_mode:'pos_vel',control_loop_active:false}),false);
 assert.equal(controlLoopReady({state_machine:'LOWLEVEL_STREAMING',control_mode:'fake_rs_pos_vel',control_loop_active:false}),false);
});

test('holding or pausing a drag never sends; release sends only the final target',async()=>{
 const sent=[],scheduler=new ReleaseScheduler((...args)=>sent.push(args),15);
 scheduler.begin('joint2');scheduler.edit('joint2',-.1);scheduler.schedule('joint2');await wait(30);assert.equal(sent.length,0);
 scheduler.edit('joint2',-.7);await wait(30);assert.equal(sent.length,0);
 scheduler.release('joint2');scheduler.schedule('joint2');await wait(30);assert.deepEqual(sent,[['joint2',-.7]]);
});
test('keyboard changes debounce and keep each joint separate',async()=>{
 const sent=[],scheduler=new ReleaseScheduler((...args)=>sent.push(args),15);
 scheduler.edit('joint1',.1);scheduler.schedule('joint1');scheduler.edit('joint1',.2);scheduler.schedule('joint1');
 scheduler.edit('joint3',-.3);scheduler.schedule('joint3');await wait(30);
 assert.deepEqual(sent,[['joint1',.2],['joint3',-.3]]);
});
test('stop, mode changes and canceled pointers clear pending motion',async()=>{
 const sent=[],scheduler=new ReleaseScheduler((...args)=>sent.push(args),15);
 scheduler.edit('joint1',1);scheduler.schedule('joint1');scheduler.cancel();await wait(30);assert.equal(sent.length,0);
 scheduler.begin('joint2');scheduler.edit('joint2',-1);scheduler.cancelJoint('joint2');scheduler.release('joint2');await wait(30);assert.equal(sent.length,0);
});
test('large moves become longer and stay under the selected peak speed',()=>{
 for(const speed of [.05,.1,.3,.5])for(const delta of [.001,.1,1,3.14,6.28]){
  const duration=moveDuration(delta,speed);assert.ok(1.5*delta/duration<=speed);assert.ok(Number.isFinite(duration));
 }
 assert.ok(moveDuration(3.14,.1)>30);
});
