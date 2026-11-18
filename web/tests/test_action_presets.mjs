import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {actionSpeedLabel,actionBusy} from '../dist/action-presets.js';
const catalog=JSON.parse(await readFile(new URL('../dist/actions.json',import.meta.url),'utf8'));
test('buttons map to original ROS actions; alternate choreography is absent',()=>{
 assert.equal(catalog.kind,'original_ros_actions');
 for(const [id,track] of Object.entries({rock:'rock_push',paper:'paper_figure8',heart:'heart_draw',circle:'paper_circle'}))assert.equal(catalog.actions[id].track,track);
 assert.equal(catalog.actions.dance.track_speed,3);
 for(const action of Object.values(catalog.actions))assert.equal(action.steps,undefined);
 assert.equal(actionSpeedLabel(.5),'0.50×');
});
test('running and stopping actions reserve controls, completed jobs release them',()=>{
 for(const status of ['running','stopping'])assert.equal(actionBusy({action_motion:{status}}),true);
 for(const status of ['idle','completed','stopped','error'])assert.equal(actionBusy({action_motion:{status}}),false);
 assert.equal(actionBusy(null),false);
});
