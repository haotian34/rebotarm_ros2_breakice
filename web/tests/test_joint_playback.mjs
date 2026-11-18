import test from 'node:test';
import assert from 'node:assert/strict';
import {JointPlayback} from '../dist/joint-playback.js';
const frame=(seconds,position,gripper=0)=>({sample_time:seconds,positions:{joint1:position,joint2:-position},gripper_opening_mm:gripper});
test('renders received samples between feedback packets without extrapolating',()=>{
 const playback=new JointPlayback(80);
 playback.push(frame(10,0,0),1000);playback.push(frame(10.04,.04,40),1040);playback.push(frame(10.08,.08,80),1080);
 for(let now=1080;now<=1160;now+=10){const sample=playback.sample(now);assert.ok(Math.abs(sample.positions.joint1-(now-1080)/1000)<1e-10);assert.ok(sample.positions.joint1>=0&&sample.positions.joint1<=.08)}
 assert.equal(playback.sample(1120).gripper,40);
 assert.equal(playback.sample(5000).positions.joint1,.08); // Lost feedback never produces continued movement.
});
test('rejects old packets and resets after a gap or explicit disconnection',()=>{
 const playback=new JointPlayback();playback.push(frame(10,1),1000);playback.push(frame(10.04,2),1040);
 assert.equal(playback.push(frame(10.02,99),1050),false);
 assert.equal(playback.push(frame(10.08,NaN),1080),false);
 playback.push(frame(12,-1),3000);assert.equal(playback.sample(3000).positions.joint1,-1);
 playback.clear();assert.equal(playback.sample(3000),null);
});
test('gripper feedback can be missing without introducing a fabricated value',()=>{
 const playback=new JointPlayback();playback.push(frame(1,0,null),1000);playback.push(frame(1.04,.1,null),1040);
 assert.equal(playback.sample(1100).gripper,null);
});
